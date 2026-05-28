#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
import tf
import threading
import numpy as np


class AbsPosNode:
    def __init__(self):
        self.have_odom = False
        self.odom_pos = np.zeros(3)
        self.odom_yaw = 0.0

        self.lock = threading.Lock()
        self.tf_listener = tf.TransformListener()

        # ===== ROS IO =====
        self.rel_sub = rospy.Subscriber(
            '/relative_pos', PointStamped, self.rel_callback)

        self.normal_sub = rospy.Subscriber(
            '/ndt_normal/target_pose_d435', PoseStamped, self.normal_callback)

        self.odom_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.odom_callback)

        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        self.target_pose_pub = rospy.Publisher(
            '/ndt_normal/target_pose_livox', PoseStamped, queue_size=1)

        self.dummytarget_pose_pub = rospy.Publisher(
            '/ndt_normal/dummytarget_pose_livox', PoseStamped, queue_size=1)

        # ===== 控制状态机 =====
        self.ctrl_stage = "IDLE"   # IDLE / DUMMY / TARGET / HOLD
        self.ctrl_timer = None

        self.target_setpoint = None
        self.dummy_setpoint = None

        self.target_pos = None
        self.dummy_target_pos = None

        self.pos_thresh = 0.05

    def odom_callback(self, msg):
        with self.lock:
            self.have_odom = True
            self.last_odom_stamp = rospy.Time.now()
            self.odom_pos = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            q = msg.pose.orientation
            quaternion = [q.x, q.y, q.z, q.w]
            self.odom_quat = quaternion
            self.odom_roll, self.odom_pitch, self.odom_yaw = tf.transformations.euler_from_quaternion(quaternion)

    def rel_callback(self, msg):
        rel_vec = np.array([msg.point.x, msg.point.y, msg.point.z])

        Rz = np.array([
            [np.cos(self.odom_yaw), -np.sin(self.odom_yaw), 0],
            [np.sin(self.odom_yaw),  np.cos(self.odom_yaw), 0],
            [0, 0, 1]
        ])
        Ry = np.array([
            [np.cos(self.odom_pitch), 0, np.sin(self.odom_pitch)],
            [0, 1, 0],
            [-np.sin(self.odom_pitch), 0, np.cos(self.odom_pitch)]
        ])
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(self.odom_roll), -np.sin(self.odom_roll)],
            [0, np.sin(self.odom_roll),  np.cos(self.odom_roll)]
        ])
        R = Rz @ Ry @ Rx
        rel_vec_global = R @ rel_vec

        distance = np.linalg.norm(rel_vec_global)

        if not np.isfinite(distance):
            return

        # -------- 核心逻辑 --------
        if distance <= 1.0:
            if not self.in_threshold:
                # 第一次进入 1.0 米以内 → 锁存
                self.abs_pos = self.odom_pos.copy()
                self.in_threshold = True
            # 已经在 1.0 内 → 保持锁存值
        else:
            # 离开 1.0 米 → 解锁
            self.in_threshold = False
            self.abs_pos = self.odom_pos + rel_vec_global
        # --------------------------

        setpoint = PositionTarget()
        setpoint.header.stamp = msg.header.stamp
        setpoint.header.frame_id = 'map'
        setpoint.type_mask = 0b100111111000
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.position.x = self.abs_pos[0]
        setpoint.position.y = self.abs_pos[1]
        setpoint.position.z = self.abs_pos[2]
        setpoint.yaw = self.odom_yaw
        self.setpoint_pub.publish(setpoint)

    # 使用 msg.pose 的位置和方向（相对位姿），结合当前里程计位姿，计算绝对目标位姿
    def normal_callback(self, msg: PoseStamped):
        """
        msg:
        - header.frame_id == d435_link
        - 表示【末端执行机构 actuator】在 d435_link 下期望到达的最终位姿
        本函数计算：
        - 无人机自身（livox_link）在 map 下应到达的位姿
        """

        # ---------- 0. TF 可用性检查 ----------
        try:
            self.tf_listener.waitForTransform(
                "base_link",
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(0.5)
            )
            self.tf_listener.waitForTransform(
                "base_link",
                "actuator_link",
                msg.header.stamp,
                rospy.Duration(0.5)
            )
            self.tf_listener.waitForTransform(
                "base_link",
                "dummyactuator_link",
                msg.header.stamp,
                rospy.Duration(0.5)
            )
            self.tf_listener.waitForTransform(
                "map",
                "base_link",
                msg.header.stamp,
                rospy.Duration(0.5)
            )
        except tf.Exception as e:
            rospy.logwarn_throttle(1.0, f"TF not ready: {e}")
            return

        # ---------- 1. camera → base（目标 actuator 位姿） ----------
        try:
            actuator_target_livox = self.tf_listener.transformPose(
                "base_link",
                msg
            )
        except tf.Exception as e:
            rospy.logwarn_throttle(1.0, f"TF camera → livox failed: {e}")
            return

        # ---------- 2. actuator 目标 → livox 自身目标 ----------
        try:
            # base → actuator 的静态 TF
            (t_la, q_la) = self.tf_listener.lookupTransform(
                "base_link",
                "actuator_link",
                msg.header.stamp
            )
            (t_ld, q_ld) = self.tf_listener.lookupTransform(
                "base_link",
                "dummyactuator_link",
                msg.header.stamp
            )
        except tf.Exception as e:
            rospy.logwarn_throttle(1.0, f"TF livox → actuator failed: {e}")
            return

        # 构造齐次矩阵：base → actuator
        T_livox_actuator = tf.transformations.quaternion_matrix(q_la)
        T_livox_actuator[0:3, 3] = np.array(t_la)
        T_livox_dummyactuator = tf.transformations.quaternion_matrix(q_ld)
        T_livox_dummyactuator[0:3, 3] = np.array(t_ld)

        # 求逆：actuator → livox
        T_actuator_livox = np.linalg.inv(T_livox_actuator)
        T_dummyactuator_livox = np.linalg.inv(T_livox_dummyactuator)

        # actuator 目标（base 坐标系下）→ 齐次矩阵
        p = actuator_target_livox.pose.position
        q = actuator_target_livox.pose.orientation
        T_livox_actuator_target = tf.transformations.quaternion_matrix(
            [q.x, q.y, q.z, q.w]
        )
        T_livox_actuator_target[0:3, 3] = np.array([p.x, p.y, p.z])

        # livox 自身目标位姿
        T_target_livox = T_livox_actuator_target @ T_actuator_livox
        T_dummytarget_livox = T_livox_actuator_target @ T_dummyactuator_livox
        q_target_livox = tf.transformations.quaternion_from_matrix(T_target_livox)

        # 转回 PoseStamped
        target_pose_base = PoseStamped()
        target_pose_base.header.stamp = msg.header.stamp
        target_pose_base.header.frame_id = "base_link"
        target_pose_base.pose.position.x = T_target_livox[0, 3]
        target_pose_base.pose.position.y = T_target_livox[1, 3]
        target_pose_base.pose.position.z = T_target_livox[2, 3]
        target_pose_base.pose.orientation.x = q_target_livox[0]
        target_pose_base.pose.orientation.y = q_target_livox[1]
        target_pose_base.pose.orientation.z = q_target_livox[2]
        target_pose_base.pose.orientation.w = q_target_livox[3]
        self.target_pose_pub.publish(target_pose_base)

        dummytarget_pose_base = PoseStamped()
        dummytarget_pose_base.header.stamp = msg.header.stamp
        dummytarget_pose_base.header.frame_id = "base_link"
        dummytarget_pose_base.pose.position.x = T_dummytarget_livox[0, 3]
        dummytarget_pose_base.pose.position.y = T_dummytarget_livox[1, 3]
        dummytarget_pose_base.pose.position.z = T_dummytarget_livox[2, 3]
        dummytarget_pose_base.pose.orientation.x = q_target_livox[0]
        dummytarget_pose_base.pose.orientation.y = q_target_livox[1]
        dummytarget_pose_base.pose.orientation.z = q_target_livox[2]
        dummytarget_pose_base.pose.orientation.w = q_target_livox[3]
        self.dummytarget_pose_pub.publish(dummytarget_pose_base)

        # return
        # ---------- 3. livox → map ----------
        target_pose_map = self.tf_listener.transformPose(
            "map", target_pose_base)

        dummytarget_pose_map = self.tf_listener.transformPose(
            "map", dummytarget_pose_base)

        _, _, yaw = tf.transformations.euler_from_quaternion([
            target_pose_map.pose.orientation.x,
            target_pose_map.pose.orientation.y,
            target_pose_map.pose.orientation.z,
            target_pose_map.pose.orientation.w
        ])

        # ===== 构造 setpoint（缓存，不发布）=====

        self.target_setpoint = PositionTarget()
        self.target_setpoint.header.frame_id = "map"
        self.target_setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.target_setpoint.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        self.target_setpoint.position = target_pose_map.pose.position
        self.target_setpoint.yaw = yaw

        self.dummy_setpoint = PositionTarget()
        self.dummy_setpoint.header.frame_id = "map"
        self.dummy_setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.dummy_setpoint.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        self.dummy_setpoint.position = dummytarget_pose_map.pose.position
        self.dummy_setpoint.yaw = yaw

        self.target_pos = np.array([
            target_pose_map.pose.position.x,
            target_pose_map.pose.position.y,
            target_pose_map.pose.position.z
        ])
        self.dummy_target_pos = np.array([
            dummytarget_pose_map.pose.position.x,
            dummytarget_pose_map.pose.position.y,
            dummytarget_pose_map.pose.position.z
        ])

        # ===== 启动状态机 =====
        self.ctrl_stage = "DUMMY"
        self.ctrl_stage = "TARGET" # debug

        if self.ctrl_timer:
            self.ctrl_timer.shutdown()

        self.ctrl_timer = rospy.Timer(
            rospy.Duration(1.0 / 60.0),
            self.control_timer_cb
        )

    # =============== 控制推进 Timer ===============
    def control_timer_cb(self, event):

        if not self.have_odom:
            return

        curr_pos = self.odom_pos.copy()

        # ===== Stage DUMMY =====
        if self.ctrl_stage == "DUMMY":
            self.dummy_setpoint.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(self.dummy_setpoint)

            if np.linalg.norm(curr_pos - self.dummy_target_pos) < self.pos_thresh:
                rospy.loginfo("Reached dummy target")
                self.ctrl_stage = "TARGET"

        # ===== Stage TARGET =====
        elif self.ctrl_stage == "TARGET":
            dx, dy, dz = self.target_pos - curr_pos
            rospy.loginfo_throttle(1.0, "err  x: %.3f  y: %.3f  z: %.3f", dx, dy, dz)
            dist = np.linalg.norm([dx, dy, dz])

            self.target_setpoint.header.stamp = rospy.Time.now()

            if dist > self.pos_thresh:
                scale = min(0.2 / dist, 1.0)
                self.target_setpoint.velocity.x = dx * scale
                self.target_setpoint.velocity.y = dy * scale
                self.target_setpoint.velocity.z = dz * scale
                self.setpoint_pub.publish(self.target_setpoint)
            else:
                rospy.loginfo("Reached final target")
                self.ctrl_stage = "HOLD"

        # ===== Stage HOLD =====
        elif self.ctrl_stage == "HOLD":
            hold = PositionTarget()
            hold.header.stamp = rospy.Time.now()
            hold.header.frame_id = "map"
            hold.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            hold.type_mask = (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
            hold.position.x = curr_pos[0]
            hold.position.y = curr_pos[1]
            hold.position.z = curr_pos[2]
            hold.yaw = self.odom_yaw

            self.setpoint_pub.publish(hold)

            self.ctrl_timer.shutdown()
            self.ctrl_stage = "IDLE"


def main():
    rospy.init_node('abs_pos')
    AbsPosNode()
    rospy.spin()


if __name__ == '__main__':
    main()
    