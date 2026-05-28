#!/usr/bin/python3
"""
在ROI质心处发布一个帧，其中：
 - x轴 = 拟合的法线（指向外部）
 - y轴 = 相机水平方向在垂直于法线平面上的投影（保持图像意义上的"水平"）
 - z轴 = x轴与y轴的叉积（右手坐标系）

平滑处理：
 - 对法线和质心使用指数移动平均（EMA）以减少抖动。

点击来源：
 - 订阅RViz目标面板发布的点击话题（geometry_msgs/PointStamped），
   其中 point.x = 像素列, point.y = 像素行。

参数（ROS）：
    ~depth_topic (str)         : 深度图像话题（默认 /d435/aligned_depth_to_color/image_raw）
    ~camera_info_topic (str)   : 相机内参话题（默认 /d435/color/camera_info）
    ~click_topic (str)         : RViz点击话题（默认 /rviz/click_point）
    ~roi_size (int)            : ROI区域的像素边长（默认 10）
    ~collect_frames (int)      : 点击后积累帧数（默认 5）
    ~alpha_normal (float)      : 法线的EMA平滑系数（0-1，值越小越平滑）。默认 0.2
    ~alpha_centroid (float)    : 质心的EMA平滑系数（0-1）。默认 0.3
"""
from geometry_msgs.msg import PoseStamped, PointStamped
import rospy
import threading
import numpy as np
import math
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf
from tf.transformations import quaternion_from_matrix

class Normal:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('ndt_normal', anonymous=True)

        # 读取ROS参数，若未设置则使用默认值
        self.depth_topic = rospy.get_param('~depth_topic', '/d435/aligned_depth_to_color/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/d435/color/camera_info')
        # 发布姿态所用的父坐标系
        self.camera_frame = rospy.get_param('~camera_frame', 'd435_color_optical_frame')
        self.roi_size = int(rospy.get_param('~roi_size', 10))
        # 点击后积累帧数设置
        self.max_collect_frames = int(rospy.get_param('~collect_frames', 5))
        # 平滑系数：0 < alpha <= 1，值越小平滑效果越强
        self.alpha_normal = float(rospy.get_param('~alpha_normal', 0.2))
        self.alpha_centroid = float(rospy.get_param('~alpha_centroid', 0.3))

        # 状态变量初始化
        self.bridge = CvBridge()  # 用于ROS图像与OpenCV图像转换
        self.depth_image = None   # 存储深度图像
        self.depth_encoding = None# 深度图像编码格式
        self.K = None             # 相机内参矩阵
        self.lock = threading.Lock()  # 线程锁，确保多线程数据安全
        self.last_click = None    # 存储最后一次鼠标点击坐标

        # 滤波后的状态（EMA结果）
        self.normal_filtered = None   # 滤波后的法线向量（3维numpy数组）
        self.centroid_filtered = None # 滤波后的质心（3维numpy数组）
        # 点击后采样/锁定目标姿态状态
        self.collecting = False
        self.frames_collected = 0
        self.locked_pose = None  # (translation, x_axis, y_axis, z_axis)
        self._last_sample = None
        self.pose_published = False

        # 订阅RViz目标面板的点击话题
        self.click_topic = rospy.get_param('~click_topic', '/rviz/click_point')

        # 订阅ROS话题
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.caminfo_cb, queue_size=1)
        rospy.Subscriber(self.click_topic, PointStamped, self.click_cb, queue_size=1)

        # 创建PoseStamped消息发布者
        self.point_pub = rospy.Publisher('~target_point_d435', PointStamped, queue_size=1)
        self.pose_pub = rospy.Publisher('~target_pose_d435', PoseStamped, queue_size=1)
        # TF监听器（用于坐标系转换）
        self.tf_listener = tf.TransformListener()

        # 输出初始化信息
        rospy.loginfo("平滑系数: normal=%.3f centroid=%.3f", self.alpha_normal, self.alpha_centroid)

        # 设置循环频率并启动主循环
        self.rate = rospy.Rate(30)
        self.spin()

    def caminfo_cb(self, msg: CameraInfo):
        """相机内参回调函数：存储相机内参矩阵K"""
        if len(msg.K) == 9:
            # 将内参列表转换为3x3矩阵
            K = np.array(msg.K).reshape((3,3))
            with self.lock:
                self.K = K

    def depth_cb(self, msg: Image):
        """深度图像回调函数：将ROS深度图像消息转换为OpenCV格式并存储"""
        try:
            with self.lock:
                self.depth_encoding = msg.encoding
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                self.depth_image = cv_img.copy()
        except CvBridgeError as e:
            rospy.logerr_throttle(10, "CvBridge 深度转换错误: %s", e)

    def click_cb(self, msg: PointStamped):
        """RViz目标面板点击回调：记录像素坐标作为ROI中心"""
        x = int(msg.point.x)
        y = int(msg.point.y)
        with self.lock:
            self.last_click = (x, y)
            # 开始收集：重置计数并清除已锁定的姿态
            self.collecting = True
            self.frames_collected = 0
            self.locked_pose = None
            self._last_sample = None
            self.pose_published = False
        rospy.loginfo("点击位置: (%d, %d)", x, y)

    def depth_to_meters(self, depth_patch, encoding):
        """将深度图像数据转换为米单位
        
        参数:
            depth_patch: 深度图像块
            encoding: 深度图像编码格式
        返回:
            转换为米单位的深度数据（NaN表示无效值）
        """
        if depth_patch is None or encoding is None:
            return None
        # 16位编码通常以毫米为单位，转换为米
        depth_m = depth_patch.astype(np.float32) / 1000.0
        depth_m[depth_m == 0] = np.nan  # 0值视为无效
        return depth_m

    def pixel_to_pointcloud(self, u_coords, v_coords, depth_values, K):
        """将像素坐标和深度值转换为3D点云（相机坐标系下）
        
        参数:
            u_coords: 像素u坐标
            v_coords: 像素v坐标
            depth_values: 深度值（米）
            K: 相机内参矩阵
        返回:
            3D点数组，形状为(n, 3)
        """
        fx = K[0,0]; fy = K[1,1]; cx = K[0,2]; cy = K[1,2]
        Z = depth_values  # 深度即Z坐标
        # 计算X和Y坐标（相机坐标系）
        X = (u_coords - cx) * Z / fx
        Y = (v_coords - cy) * Z / fy
        # 组合为点云并返回
        pts = np.vstack((X, Y, Z)).T
        return pts

    def fit_plane_svd(self, points):
        """使用SVD方法拟合平面，计算平面法线和质心
        
        参数:
            points: 3D点数组，形状为(n, 3)
        返回:
            normal: 平面法线向量（单位向量）
            centroid: 点集的质心
        """
        if points.shape[0] < 3:  # 至少需要3个点才能拟合平面
            return None, None
        # 计算质心
        centroid = np.nanmean(points, axis=0)
        # 点集去中心化
        pts_centered = points - centroid
        # 过滤含NaN的无效点
        valid = ~np.isnan(pts_centered).any(axis=1)
        pts_centered = pts_centered[valid]
        if pts_centered.shape[0] < 30:  # 有效点不足3个
            return None, None
        # SVD分解求法线（最小奇异值对应的右奇异向量）
        U, S, Vt = np.linalg.svd(pts_centered, full_matrices=False)
        normal = Vt[-1, :]
        # 归一化法线
        nrm = np.linalg.norm(normal)
        if nrm == 0:  # 法线无效
            return None, None
        normal = normal / nrm
        return normal, centroid

    def project_point(self, pt3, K):
        """将3D点投影到图像平面，得到像素坐标
        
        参数:
            pt3: 3D点（X, Y, Z）
            K: 相机内参矩阵
        返回:
            (u, v) 像素坐标，无效时返回None
        """
        X, Y, Z = pt3
        if Z == 0 or np.isnan(Z):  # Z为0或无效时无法投影
            return None
        fx = K[0,0]; fy = K[1,1]; cx = K[0,2]; cy = K[1,2]
        # 投影公式
        u = fx * (X / Z) + cx
        v = fy * (Y / Z) + cy
        return int(round(u)), int(round(v))

    def build_frame_axes_from_normal(self, normal):
        """
        根据法线构建符合无人机作业要求的坐标系：
        1. Z轴：严格指向全局 camera_init 的上方 (0,0,1)
        2. X轴：在满足Z轴约束下，尽可能指向法线方向
        3. Y轴：右手系推导 (Z x X)
        4. 约束：X轴与原始法线夹角不得超过30度
        """

        # 获取map系下的“上方”向量
        z_axis = np.array([0.0, -1.0, 0.0])

        # 2. 奇异性检查：如果法线与全局Z轴几乎重合（检测地面或天花板），无法构建水平朝向的X轴
        dot_zn = np.dot(z_axis, normal)
        if abs(dot_zn) > 0.965:  # 约等于夹角 < 15度
            rospy.logwarn("法线与垂直方向过于接近，拒绝构建坐标系")
            return None

        # 3. 计算 Y 轴 (Y = Z x Normal)
        # y 轴将处于水平面内，且垂直于法线
        y_axis = np.cross(z_axis, normal)
        y_norm = np.linalg.norm(y_axis)
        if y_norm < 1e-6:
            return None
        y_axis /= y_norm

        # 4. 计算 X 轴 (X = Y x Z) 以确保三轴严格正交且符合右手系
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)

        # 5. 角度偏差校验
        # 计算最终构建的 x_axis 与原始输入 normal 的夹角
        # cos(theta) = (A·B) / (|A||B|)
        cos_theta = np.dot(x_axis, normal)
        # 限制在 [-1, 1] 范围内防止数值误差导致 acos 报错
        cos_theta = max(-1.0, min(1.0, cos_theta))
        angle_deg = math.degrees(math.acos(cos_theta))
        if angle_deg > 30.0:
            rospy.logwarn("pitch过大，拒绝该姿态")
            return None
        
        camera_view = np.array([0.0, 0.0, 1.0])
        if(np.dot(x_axis, camera_view)) < 0:
            x_axis = -x_axis
            y_axis = -y_axis

        # 最终结果：x_axis(接近法线), y_axis(水平), z_axis(世界向上)
        return x_axis, y_axis, z_axis

    def publish_normal_pose(self, translation, x_axis, y_axis, z_axis):
        """发布表示坐标系的PoseStamped消息
        
        参数:
            translation: 坐标系原点（平移向量）
            x_axis, y_axis, z_axis: 坐标系的三个轴
        """
        # 构建4x4旋转矩阵（前3列分别为x,y,z轴）
        R = np.eye(4, dtype=np.float64)
        R[0:3, 0] = x_axis
        R[0:3, 1] = y_axis
        R[0:3, 2] = z_axis

        # 将旋转矩阵转换为四元数（x,y,z,w）
        quat = quaternion_from_matrix(R)

        # 构建PointStamped消息
        target_point_d435 = PointStamped()
        target_point_d435.header.stamp = rospy.Time.now()
        target_point_d435.header.frame_id = self.camera_frame  # 父坐标系
        # 设置位置
        target_point_d435.point.x = float(translation[0])
        target_point_d435.point.y = float(translation[1])
        target_point_d435.point.z = float(translation[2])
        self.point_pub.publish(target_point_d435)

        # 构建PoseStamped消息
        target_pose_d435 = PoseStamped()
        target_pose_d435.header.stamp = rospy.Time.now()
        target_pose_d435.header.frame_id = self.camera_frame  # 父坐标系
        # 设置位置
        target_pose_d435.pose.position.x = float(translation[0])
        target_pose_d435.pose.position.y = float(translation[1])
        target_pose_d435.pose.position.z = float(translation[2])
        # 设置姿态（四元数）
        target_pose_d435.pose.orientation.x = float(quat[0])
        target_pose_d435.pose.orientation.y = float(quat[1])
        target_pose_d435.pose.orientation.z = float(quat[2])
        target_pose_d435.pose.orientation.w = float(quat[3])
        self.pose_pub.publish(target_pose_d435)

    def ema_update_vector(self, prev, new, alpha):
        """对向量进行指数移动平均更新
        
        参数:
            prev: 上一次的滤波结果（可为None）
            new: 新的测量值
            alpha: 平滑系数（0 < alpha <= 1）
        返回:
            新的滤波结果，公式：new_filtered = alpha*new + (1-alpha)*prev
        """
        if prev is None:  # 首次更新，直接使用新值
            return new.copy()
        return alpha * new + (1.0 - alpha) * prev

    def compute_and_process(self):
        """主处理函数：计算ROI的平面和坐标系"""
        # 线程安全地获取数据
        depth = None; encoding = None; K = None; click = None
        with self.lock:
            if self.depth_image is not None:
                depth = self.depth_image.copy()
                encoding = self.depth_encoding
            if self.K is not None:
                K = self.K.copy()
            click = self.last_click

        # 若未点击或缺少深度/内参，跳过
        if click is None:
            return

        if depth is None or K is None:
            return

        # 计算ROI区域（以点击点为中心）
        x, y = click
        half = self.roi_size // 2
        h, w = depth.shape[:2]
        x1 = max(0, x - half); y1 = max(0, y - half)
        x2 = min(w-1, x + half); y2 = min(h-1, y + half)

        # 提取ROI区域的深度数据
        depth_patch = depth[y1:y2+1, x1:x2+1]
        if depth_patch.size == 0:
            return

        # 将深度数据转换为米单位
        depth_m = self.depth_to_meters(depth_patch, encoding)
        if depth_m is None:
            return

        # 生成ROI内所有像素的坐标
        ys, xs = np.meshgrid(np.arange(y1, y2+1), np.arange(x1, x2+1), indexing='ij')
        xs_flat = xs.flatten().astype(np.float32)
        ys_flat = ys.flatten().astype(np.float32)
        depth_flat = depth_m.flatten()
        # 过滤有效深度值
        valid_mask = ~np.isnan(depth_flat) & (depth_flat > 0.2) & (depth_flat < 2.0)
        if np.count_nonzero(valid_mask) < 30:
            return

        # 提取有效点并转换为3D点云
        xs_valid = xs_flat[valid_mask]; ys_valid = ys_flat[valid_mask]; depths_valid = depth_flat[valid_mask]
        points3d = self.pixel_to_pointcloud(xs_valid, ys_valid, depths_valid, K)
        # 拟合平面，得到法线和质心
        normal, centroid = self.fit_plane_svd(points3d)
        if normal is None:
            return

        # 对法线进行平滑处理（EMA）
        # 确保新法线与历史法线方向一致（避免符号跳变）
        if self.normal_filtered is not None:
            if np.dot(self.normal_filtered, normal) < 0:
                normal = -normal

        # EMA更新法线
        normal_smoothed = self.ema_update_vector(self.normal_filtered, normal, self.alpha_normal)
        # 重新归一化
        nrm = np.linalg.norm(normal_smoothed)
        if nrm == 0:
            normal_smoothed = normal.copy()
            nrm = np.linalg.norm(normal_smoothed)
            if nrm == 0:
                return
        normal_smoothed = normal_smoothed / nrm

        # 对质心进行EMA平滑
        centroid_smoothed = self.ema_update_vector(self.centroid_filtered, centroid, self.alpha_centroid)

        # 更新滤波状态
        self.normal_filtered = normal_smoothed
        self.centroid_filtered = centroid_smoothed

        # 根据平滑后的法线构建坐标系
        axes = self.build_frame_axes_from_normal(self.normal_filtered)
        if axes is None:
            return
        x_axis, y_axis, z_axis = axes

        # 根据收集状态决定是否发布/锁定姿态
        if self.collecting:
            self._last_sample = (self.centroid_filtered.copy(), x_axis.copy(), y_axis.copy(), z_axis.copy())
            self.frames_collected += 1
            rospy.logdebug("收集姿态帧: %d/%d", self.frames_collected, self.max_collect_frames)

            if self.frames_collected >= self.max_collect_frames:
                self.collecting = False
                self.locked_pose = self._last_sample

                if not self.pose_published:
                    t, xa, ya, za = self.locked_pose
                    self.publish_normal_pose(t, xa, ya, za)
                    self.pose_published = True
                    rospy.loginfo("已发布最终姿态")

    def spin(self):
        """主循环：持续处理"""
        while not rospy.is_shutdown():
            try:
                self.compute_and_process()
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

if __name__ == '__main__':
    try:
        node = Normal()
    except rospy.ROSInterruptException:
        pass
