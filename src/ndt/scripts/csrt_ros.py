#!/usr/bin/python3
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters


# 三维常速度卡尔曼滤波器
class Kalman3D:
    def __init__(self, process_var=1e-4, measure_var=1e-2):
        # 状态: [x, y, z, vx, vy, vz]
        self.x = np.zeros((6, 1))
        self.P = np.eye(6)
        self.F = np.eye(6)
        self.F[0, 3] = 1.0
        self.F[1, 4] = 1.0
        self.F[2, 5] = 1.0
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0
        self.Q = process_var * np.eye(6)
        self.R = measure_var * np.eye(3)
        self.initialized = False

    def update(self, measurement):
        z = np.array(measurement, dtype=np.float64).reshape((3, 1))
        if not self.initialized:
            self.x[0:3, 0] = z[:, 0]
            self.x[3:6, 0] = 0.0
            self.initialized = True
        # 预测
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # 更新
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        return self.x[0:3, 0]


rospy.loginfo('Initializing ROS node and publishers...')
rospy.init_node('csrt_tracker', anonymous=True)
pub = rospy.Publisher('/relative_pos', PointStamped, queue_size=10)
click_pub = rospy.Publisher('/clicked_relative_pos', PointStamped, queue_size=10)
rospy.loginfo('ROS node initialized.')

# 全局变量
tracker = None
selecting = False
start_point = ()
current_point = ()
fixed_frame = None
current_display_frame = None
clicked_points = []  # 存储点击的点坐标
allow_click_only = False  # 是否只允许点击选点，不初始化追踪器

# 三维卡尔曼滤波器（追踪用）
kalman3d = Kalman3D()
# 三维卡尔曼滤波器（点击用）
kalman3d_click = Kalman3D()

# ROS 图像接收相关
bridge = CvBridge()
color_frame_latest = None
depth_frame_latest = None
intrinsics = None  # dict with fx, fy, cx, cy

# 回调：接收并同步彩色与深度
def synced_cb(color_msg, depth_msg):
    global color_frame_latest, depth_frame_latest
    try:
        color_cv = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f'Failed to convert color image: {e}')
        return

    try:
        # depth可能是16UC1或32FC1
        depth_cv = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    except Exception as e:
        rospy.logerr(f'Failed to convert depth image: {e}')
        return

    color_frame_latest = color_cv
    depth_frame_latest = depth_cv


def camera_info_cb(info_msg):
    global intrinsics
    K = info_msg.K
    fx = K[0]
    fy = K[4]
    cx = K[2]
    cy = K[5]
    intrinsics = {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy}


# 鼠标回调函数
def mouse_callback(event, x, y, flags, param):
    global selecting, start_point, current_point, fixed_frame, tracker, allow_click_only, clicked_points

    if allow_click_only:
        if event == cv2.EVENT_LBUTTONDOWN:
            # 记录点击点
            clicked_points.append((x, y))
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        selecting = True
        start_point = (x, y)
        current_point = (x, y)
        if current_display_frame is not None:
            fixed_frame = current_display_frame.copy()
        tracker = None

    elif event == cv2.EVENT_MOUSEMOVE and selecting:
        current_point = (x, y)

    elif event == cv2.EVENT_LBUTTONUP and selecting:
        x_min = min(start_point[0], current_point[0])
        y_min = min(start_point[1], current_point[1])
        w = abs(current_point[0] - start_point[0])
        h = abs(current_point[1] - start_point[1])
        bbox = (x_min, y_min, w, h)

        tracker = cv2.TrackerCSRT_create()
        if fixed_frame is not None:
            tracker.init(fixed_frame, bbox)
        selecting = False
        allow_click_only = False  # 启动追踪器，关闭点击选点模式


rospy.loginfo('Setting up ROS image subscribers (expects realsense node publishing color & aligned depth)...')
# 话题名根据你的realsense_ros配置修改（下面为常用名字）
color_topic = '/d435/color/image_raw'
depth_topic = '/d435/aligned_depth_to_color/image_raw'
camera_info_topic = '/d435/color/camera_info'

# 使用 message_filters 同步 color & depth
color_sub = message_filters.Subscriber(color_topic, Image)
depth_sub = message_filters.Subscriber(depth_topic, Image)
ats = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=0.05)
ats.registerCallback(synced_cb)

# 订阅相机内参
rospy.Subscriber(camera_info_topic, CameraInfo, camera_info_cb)

cv2.namedWindow('Tracking')
cv2.setMouseCallback('Tracking', mouse_callback)
rospy.loginfo('Entering main loop. Press q to quit.')

rate = rospy.Rate(30)
try:
    while not rospy.is_shutdown():
        if color_frame_latest is None or depth_frame_latest is None or intrinsics is None:
            # 等待第一次数据到达
            rospy.logdebug('Waiting for color, depth and camera_info...')
            rate.sleep()
            continue

        frame = color_frame_latest.copy()
        depth_image = depth_frame_latest.copy()
        current_display_frame = frame.copy()

        # 处理点击点：逐个发布
        if allow_click_only and clicked_points:
            cx, cy = clicked_points.pop(0)
            rospy.loginfo(f'Clicked point: ({cx}, {cy})')

            # 读取深度并转换为米
            if depth_image.dtype == np.uint16:
                depth_value = float(depth_image[cy, cx]) / 1000.0  # mm->m
            else:
                depth_value = float(depth_image[cy, cx])  # assume meters (float32)

            fx = intrinsics['fx']
            fy = intrinsics['fy']
            cx0 = intrinsics['cx']
            cy0 = intrinsics['cy']

            Z = depth_value
            X_cam = (cx - cx0) * Z / fx
            Y_cam = (cy - cy0) * Z / fy

            # 坐标系转换
            X_drone = Z
            Y_drone = -X_cam
            Z_drone = -Y_cam

            # 三维卡尔曼滤波
            X_drone_f, Y_drone_f, Z_drone_f = kalman3d_click.update([X_drone, Y_drone, Z_drone])

            point_msg2 = PointStamped()
            point_msg2.header.stamp = rospy.Time.now()
            point_msg2.header.frame_id = 'base_link'
            point_msg2.point.x = X_drone_f
            point_msg2.point.y = Y_drone_f
            point_msg2.point.z = Z_drone_f
            click_pub.publish(point_msg2)
            rospy.loginfo(f'/clicked_relative_pos: ({X_drone_f:.2f}, {Y_drone_f:.2f}, {Z_drone_f:.2f})')

        if tracker is None:
            if selecting:
                if fixed_frame is not None:
                    frame = fixed_frame.copy()
                if start_point and current_point:
                    cv2.rectangle(frame, start_point, current_point, (0, 255, 0), 2)
        else:
            success, bbox = tracker.update(frame)
            if success:
                x, y, w, h = [int(v) for v in bbox]
                cx = x + w // 2
                cy = y + h // 2

                # 获取深度并转换为米
                if depth_image.dtype == np.uint16:
                    depth_value = float(depth_image[cy, cx]) / 1000.0
                else:
                    depth_value = float(depth_image[cy, cx])

                fx = intrinsics['fx']
                fy = intrinsics['fy']
                cx0 = intrinsics['cx']
                cy0 = intrinsics['cy']

                Z = depth_value
                X_cam = (cx - cx0) * Z / fx
                Y_cam = (cy - cy0) * Z / fy

                # 坐标系转换
                X_drone = Z
                Y_drone = -X_cam
                Z_drone = -Y_cam

                # 三维卡尔曼滤波
                X_drone_f, Y_drone_f, Z_drone_f = kalman3d.update([X_drone, Y_drone, Z_drone])
                distance = np.sqrt(X_drone_f**2 + Y_drone_f**2 + Z_drone_f**2)

                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = 'base_link'

                if distance < 1.0:
                    rospy.loginfo(f'Target close (distance={distance:.2f}m)')
                    point_msg.point.x = 0.0
                    point_msg.point.y = 0.0
                    point_msg.point.z = 0.0
                    allow_click_only = False
                else:
                    point_msg.point.x = X_drone_f
                    point_msg.point.y = Y_drone_f
                    point_msg.point.z = Z_drone_f

                pub.publish(point_msg)
                rospy.loginfo(f'/relative_pos: ({point_msg.point.x:.2f}, {point_msg.point.y:.2f}, {point_msg.point.z:.2f})')

                # 画出追踪框和坐标信息
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"({X_drone_f:.2f}, {Y_drone_f:.2f}, {Z_drone_f:.2f})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                # 追踪失败
                rospy.logwarn('Tracking failed.')
                cv2.putText(frame, 'Tracking failed', (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        # 显示点击的点
        for pt in clicked_points:
            cv2.circle(frame, pt, 4, (255, 0, 0), -1)

        cv2.imshow('Tracking', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            rospy.loginfo('Quit key pressed. Exiting...')
            break

        if cv2.getWindowProperty('Tracking', cv2.WND_PROP_VISIBLE) < 1:
            rospy.loginfo('Window closed. Exiting...')
            break

        rate.sleep()

finally:
    cv2.destroyAllWindows()
    rospy.loginfo('Program exited.')
