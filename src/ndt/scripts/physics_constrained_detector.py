#!/usr/bin/python3
"""
基于物理约束注意力机制的无人机接触检测节点。

实现论文第三章所述的物理约束Transformer接触检测模型：
- 构造物理约束矩阵 P（时间邻近 + 空间邻近 + 法向一致性）
- 将 P 以对数偏置形式注入自注意力计算
- 输出接触概率，实现优于纯数据驱动Transformer的接触判别

数据流：
  1. 用户通过RViz点击目标表面
  2. 从深度ROI提取视觉特征（平均深度、深度方差、梯度等）
  3. 结合无人机位姿和时间戳构造物理约束矩阵
  4. 物理约束Transformer输出接触概率序列
  5. 发布 /ndt/contact_probability

参数（ROS）：
    ~depth_topic (str)           : 深度图像话题（默认 /d435/aligned_depth_to_color/image_raw）
    ~camera_info_topic (str)     : 相机内参话题（默认 /d435/color/camera_info）
    ~pose_topic (str)            : 位姿话题（默认 /mavros/local_position/pose）
    ~click_topic (str)           : RViz点击话题（默认 /rviz/click_point）
    ~normal_topic (str)          : 法线话题（默认 /ndt_normal/target_pose_d435）
    ~output_topic (str)          : 输出话题（默认 /ndt/contact_probability）
    ~window_size (int)           : 时间窗口大小（默认 64）
    ~roi_size (int)              : ROI像素边长（默认 20）
    ~lambda_phys (float)         : 物理约束强度（默认 0.5）
    ~d_model (int)               : 模型特征维度（默认 128）
    ~model_path (str)            : 预训练权重路径（默认 ""）
"""
import collections
import math
import os
import sys
import threading

# catkin exec() wrapper 不会自动添加脚本目录到 sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rospy
import tf.transformations
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray

# PyTorch
import torch

from physics_attention import PhysicsConstrainedContactDetector


class FeatureBuffer:
    """滑动窗口特征缓冲区。"""

    def __init__(self, window_size, d_vis):
        self.window_size = window_size
        self.d_vis = d_vis

        # 环形缓冲区
        self.vis_buf = collections.deque(maxlen=window_size)
        self.pose_buf = collections.deque(maxlen=window_size)
        self.ts_buf = collections.deque(maxlen=window_size)

        self.lock = threading.Lock()

    def push(self, vis_feat, pose, stamp_sec):
        with self.lock:
            self.vis_buf.append(vis_feat)
            self.pose_buf.append(pose)
            self.ts_buf.append(stamp_sec)

    def ready(self):
        return len(self.vis_buf) >= self.window_size // 2

    def full(self):
        return len(self.vis_buf) >= self.window_size

    def snapshot(self):
        """取出缓冲区快照。"""
        with self.lock:
            vis = list(self.vis_buf)
            pose = list(self.pose_buf)
            ts = list(self.ts_buf)
        return vis, pose, ts


class PhysicsConstrainedDetectorNode:
    def __init__(self):
        rospy.init_node('physics_constrained_detector', anonymous=False)

        # ---- 参数 ----
        self.depth_topic = rospy.get_param('~depth_topic', '/d435/aligned_depth_to_color/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/d435/color/camera_info')
        self.pose_topic = rospy.get_param('~pose_topic', '/mavros/local_position/pose')
        self.click_topic = rospy.get_param('~click_topic', '/rviz/click_point')
        self.normal_topic = rospy.get_param('~normal_topic', '/ndt_normal/target_pose_d435')
        self.output_topic = rospy.get_param('~output_topic', '/ndt/contact_probability')
        self.window_size = int(rospy.get_param('~window_size', 64))
        self.roi_size = int(rospy.get_param('~roi_size', 20))
        self.lambda_phys = float(rospy.get_param('~lambda_phys', 0.5))
        d_model = int(rospy.get_param('~d_model', 128))
        model_path = rospy.get_param('~model_path', '')

        # ---- 视觉特征维度 ----
        self.d_vis = 6  # mean_depth, depth_var, grad_x, grad_y, norm_depth, fill_ratio

        # ---- 设备 ----
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # 模型（必须提供预训练权重路径）
        if not model_path:
            rospy.logfatal("model_path 参数未指定！物理约束Transformer需要预训练权重。"
                           "请通过 ~model_path 参数指定 .pth 文件路径。")
            rospy.signal_shutdown("缺少 model_path 参数")
            return

        self.model = PhysicsConstrainedContactDetector(
            d_vis=self.d_vis, d_model=d_model, n_heads=8, n_layers=2, dropout=0.1
        )
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device).eval()
        rospy.loginfo("已加载预训练模型: %s", model_path)

        # ---- 状态 ----
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.depth_image = None
        self.depth_encoding = None
        self.K = None
        self.last_pose = None     # (x, y, z, roll, pitch, yaw)
        self.last_pose_stamp = 0.0
        self.last_normal = None   # (nx, ny, nz) from target_pose_d435
        self.last_normal_stamp = 0.0
        self.last_click = None    # (u, v) pixel coordinates
        self.active = False       # 是否已点击并正在采集

        # 滑动窗口缓冲区
        self.buffer = FeatureBuffer(self.window_size, self.d_vis)

        # ---- ROS IO ----
        rospy.Subscriber(self.depth_topic, Image, self._depth_cb, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._caminfo_cb, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=10)
        rospy.Subscriber(self.click_topic, PointStamped, self._click_cb, queue_size=1)
        rospy.Subscriber(self.normal_topic, PoseStamped, self._normal_cb, queue_size=10)
        self.prob_pub = rospy.Publisher(
            self.output_topic, Float32MultiArray, queue_size=10)

        rospy.loginfo("物理约束接触检测节点已启动  window=%d  lambda=%.2f  roi=%d",
                       self.window_size, self.lambda_phys, self.roi_size)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.active:
                self._process_frame()
            rate.sleep()

    # ========== 订阅回调 ==========

    def _depth_cb(self, msg):
        with self.lock:
            try:
                self.depth_encoding = msg.encoding
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            except CvBridgeError as e:
                rospy.logerr_throttle(10, "深度转换错误: %s", e)

    def _caminfo_cb(self, msg):
        if len(msg.K) == 9:
            with self.lock:
                self.K = np.array(msg.K).reshape((3, 3))

    def _pose_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        rpy = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self.lock:
            self.last_pose = (p.x, p.y, p.z, rpy[0], rpy[1], rpy[2])
            self.last_pose_stamp = msg.header.stamp.to_sec()

    def _click_cb(self, msg):
        x, y = int(msg.point.x), int(msg.point.y)
        with self.lock:
            self.last_click = (x, y)
            self.active = True
        rospy.loginfo("物理约束检测开始: 点击位置 (%d, %d)", x, y)

    def _normal_cb(self, msg):
        q = msg.pose.orientation
        # 法向量：X轴（pose 的 x 轴方向）
        R = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        normal = R[:3, 0]  # X轴 = 拟合的法线方向
        with self.lock:
            self.last_normal = (normal[0], normal[1], normal[2])
            self.last_normal_stamp = msg.header.stamp.to_sec()

    # ========== 特征提取 ==========

    def _extract_visual_features(self, depth, K, click):
        """从深度ROI提取视觉特征向量。

        Returns:
            vis_feat: (d_vis,) 视觉特征向量
            centroid_3d: (3,) ROI质心在相机坐标系下的3D坐标
        """
        h, w = depth.shape
        u, v = click
        half = self.roi_size // 2

        x1 = max(0, u - half); y1 = max(0, v - half)
        x2 = min(w - 1, u + half); y2 = min(h - 1, v + half)

        patch = depth[y1:y2 + 1, x1:x2 + 1].astype(np.float32) / 1000.0
        patch[patch == 0] = np.nan

        valid = patch[~np.isnan(patch) & (patch > 0.2) & (patch < 2.0)]

        if len(valid) < 10:
            return None, None

        # 特征
        mean_depth = float(np.nanmean(patch))
        depth_var = float(np.nanvar(patch)) if len(valid) > 1 else 0.0

        # 深度梯度
        gy, gx = np.gradient(np.nan_to_num(patch, nan=mean_depth))
        grad_x = float(np.nanmean(np.abs(gx)))
        grad_y = float(np.nanmean(np.abs(gy)))

        norm_depth = mean_depth / 2.0  # 归一化到 [0, 1]（假设最大深度2m）
        fill_ratio = float(len(valid)) / (patch.shape[0] * patch.shape[1])

        # 3D 质心
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        centroid_z = mean_depth
        centroid_x = (u - cx) * centroid_z / fx
        centroid_y = (v - cy) * centroid_z / fy

        feat = np.array([mean_depth, depth_var, grad_x, grad_y, norm_depth, fill_ratio],
                        dtype=np.float32)
        centroid = np.array([centroid_x, centroid_y, centroid_z], dtype=np.float32)

        return feat, centroid

    # ========== 推理 ==========

    def _process_frame(self):
        """采集一帧特征并尝试推理。"""
        depth = None; K = None; click = None; pose = None; pose_stamp = 0.0
        with self.lock:
            if self.depth_image is not None:
                depth = self.depth_image.copy()
            if self.K is not None:
                K = self.K.copy()
            click = self.last_click
            pose = self.last_pose
            pose_stamp = self.last_pose_stamp

        if depth is None or K is None or click is None or pose is None:
            return

        # 提取视觉特征
        vis_feat, centroid = self._extract_visual_features(depth, K, click)
        if vis_feat is None:
            return

        # 位姿：使用ROI 3D质心近似空间位置（相对于地图的绝对位置需TF变换）
        # 此处使用相机坐标系下的质心 + 位姿近似
        spatial_pos = np.array([
            pose[0] + centroid[0],
            pose[1] + centroid[1],
            pose[2] + centroid[2],
        ], dtype=np.float32)

        self.buffer.push(vis_feat, spatial_pos, pose_stamp)

        # 缓冲区满时推理
        if self.buffer.full():
            self._infer_and_publish()

    def _infer_and_publish(self):
        """取缓冲区数据，运行物理约束Transformer推理，发布接触概率。"""
        vis_list, pos_list, ts_list = self.buffer.snapshot()
        T = len(vis_list)

        vis = torch.tensor(np.stack(vis_list), dtype=torch.float32).unsqueeze(0).to(self.device)
        pos = torch.tensor(np.stack(pos_list), dtype=torch.float32).unsqueeze(0).to(self.device)
        ts = torch.tensor(ts_list, dtype=torch.float32).unsqueeze(0).to(self.device)

        # 法向量
        normals = None
        with self.lock:
            if self.last_normal is not None:
                n = np.tile(np.array(self.last_normal), (T, 1))
                normals = torch.tensor(n, dtype=torch.float32).unsqueeze(0).to(self.device)

        # 物理约束Transformer推理
        with torch.no_grad():
            logits, _ = self.model(vis, ts, pos, normals, self.lambda_phys)
            contact_prob = torch.softmax(logits, dim=-1)[0, :, 1].cpu().numpy()

        # 发布
        msg = Float32MultiArray()
        msg.data = contact_prob.tolist()
        self.prob_pub.publish(msg)

        current_prob = float(contact_prob[-1])
        # 在日志中加入更多调试信息
        rospy.loginfo_throttle(
            1.0,
            "接触概率: %.3f  depth=%.3fm  window=%d  ts_range=%.1fs",
            current_prob, float(vis[-1, 0]), T, ts_list[-1] - ts_list[0] if len(ts_list) > 1 else 0)


if __name__ == '__main__':
    try:
        PhysicsConstrainedDetectorNode()
    except rospy.ROSInterruptException:
        pass
