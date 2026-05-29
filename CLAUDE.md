# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

ROS Noetic catkin workspace for autonomous drone ultrasonic Non-Destructive Testing (NDT). A PX4 multirotor equipped with Livox MID-360 LiDAR, Intel RealSense D435 RGBD camera, and EMAT electromagnetic ultrasonic probe performs autonomous surface detection and contact-state classification. Runs on NVIDIA Jetson (aarch64).

## Tech Stack

| Layer | Technology | Version/Notes |
|-------|-----------|---------------|
| Language | C++17 / Python 3 | C++ for drivers & plugins, Python for control & signal processing |
| ROS | ROS Noetic | catkin build system, roscpp + rospy |
| Build | catkin_make | CMake 3.0.2+, AUTOMOC for Qt5 |
| Qt | Qt5 Widgets | RViz panel plugins (emat, ndt) |
| Math | Eigen3 | SVD plane fitting, matrix ops |
| Point Cloud | PCL >= 1.8 | FAST-LIO, livox drivers |
| Vision | OpenCV | CSRT tracker, cv_bridge |
| USB | libusb-1.0 | EMAT CH346C communication |
| LiDAR SDK | Livox-LiDAR-SDK | Static lib `liblivox_lidar_sdk_static.a` |
| Camera SDK | librealsense2 >= 2.50.0 | RealSense D435 |
| Flight | MAVROS + PX4 | mavros_msgs, tf2 |
| Lint | flake8 | max-line-length=120, ignore E501,W503 |
| CI | GitHub Actions | `.github/workflows/ci.yml` |

## Common Commands

```bash
# Build
cd ~/ndt_ws && catkin_make                # full workspace
cd ~/ndt_ws && catkin_make --pkg emat     # single package
cd ~/ndt_ws && catkin_make --pkg ndt
source ~/ndt_ws/devel/setup.bash          # source before running

# Lint (matches CI)
flake8 --max-line-length=120 --ignore=E501,W503 src/bringup/scripts/ src/ndt/scripts/

# Run
roslaunch bringup bringup.launch          # full system (MAVROS + LiDAR + FAST-LIO + RViz)
roslaunch ndt normal.launch               # surface normal targeting + flight control
roslaunch ndt csrt.launch                 # CSRT visual tracking + flight control
roslaunch emat emat.launch                # EMAT thickness gauge (needs USB access)
roslaunch record record.launch            # multimodal data recorder
roslaunch record record_bag.launch        # rosbag recorder

# USB permission (one-time setup for EMAT)
sudo tee /etc/udev/rules.d/99-ch346-emat.rules << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55eb", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55e0", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

# RViz on Jetson (software rendering)
LIBGL_ALWAYS_SOFTWARE=1 rviz
```

## Architecture

### Data Flow Pipeline

```
Sensors:
  Livox MID-360 ──→ livox_ros_driver2 ──→ FAST-LIO ──→ /Odometry ──→ lidar_to_mavros.py ──→ /mavros/vision_pose/pose
  RealSense D435 ──→ realsense2_camera ──→ /d435/color/image_raw + /d435/aligned_depth_to_color/image_raw
  EMAT probe (USB) ──→ emat_thickness_gauge_node ──→ /emat/waveform ──→ emat_feature_extractor.py ──→ /emat/features

Targeting (one of):
  normal_ros.py: RViz click ──→ SVD plane fit ──→ /ndt_normal/target_pose_d435
  csrt_ros.py:   CSRT tracker ──→ /relative_pos

Control:
  abs_pos.py ──→ /mavros/setpoint_raw/local (DUMMY→TARGET→HOLD state machine) ──→ PX4

Recording:
  record/multimodal_recorder ──→ CSV + RGB/depth video to runs/
```

### Packages

| Package | Lang | Build Target | Purpose |
|---------|------|-------------|---------|
| `bringup` | Python | — | System integration, launches all subsystems, lidar_to_mavros bridge |
| `ndt` | C++17/Python | `rviz_target_panel` (RViz plugin) | Visual targeting, surface normal estimation, flight control, feature extraction |
| `emat` | C++17 | `emat_thickness_gauge_node`, `rviz_emat_panel` (RViz plugin) | EMAT USB driver, waveform visualization |
| `record` | C++17 | `multimodal_recorder` | Multi-modal data recording (EMAT + pose + RGBD) |
| `fast_lio` | C++14 | `fastlio_mapping` | LiDAR-inertial odometry (IEKF + ikd-Tree) |
| `livox_ros_driver2` | C++14 | `livox_ros_driver2_node` | Livox MID-360 LiDAR driver |
| `realsense2_camera` | C++11 | `realsense2_camera` (nodelet) | Intel RealSense D435 driver |

### Custom ROS Messages

| Message | Package | Key Fields |
|---------|---------|-----------|
| `EmatWaveform` | emat | `raw_data(uint8[])`, `speed_of_voice`, `excitation_frequency_mhz` |
| `EmatFeatures` | emat | `energy`, `peak_amplitude`, `arrival_time`, `spectral_centroid`, `kurtosis`, `phase`, `band_energies[8]` |
| `EmatEnvelope` | emat | `envelope(float32[])`, `sampling_rate` |
| `EmatDeviceStatus` | emat | `is_connected`, `status_message` |
| `CustomMsg` | livox_ros_driver2 | `points(CustomPoint[])`, `lidar_id` |

### RViz Panel Plugins

| Plugin | Class | Description |
|--------|-------|-------------|
| `ndt/RvizTargetPanel` | `ndt::RvizTargetPanel` | D435 RGB image display, click publishes `PointStamped` to `~click_point` |
| `emat/RvizEmatPanel` | `emat::RvizEmatPanel` | EMAT waveform visualization with DC removal and grid overlay |

### Key ROS Topics

| Topic | Type | Source |
|-------|------|--------|
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | lidar_to_mavros.py |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | abs_pos.py |
| `/d435/color/image_raw` | `sensor_msgs/Image` | realsense2_camera |
| `/d435/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | realsense2_camera |
| `/ndt_normal/target_pose_d435` | `geometry_msgs/PoseStamped` | normal_ros.py |
| `/rviz/click_point` | `geometry_msgs/PointStamped` | RvizTargetPanel |
| `/emat/waveform` | `EmatWaveform` | emat_thickness_gauge_node |
| `/emat/features` | `EmatFeatures` | emat_feature_extractor.py |
| `/emat/envelope` | `EmatEnvelope` | emat_feature_extractor.py |

## Coding Guidelines

### C++ (drivers, plugins, recorder)
- **Standard**: C++17 (`set(CMAKE_CXX_STANDARD 17)`)
- **Indent**: 4 spaces, no tabs
- **Naming**: `snake_case` for functions/variables, `PascalCase` for classes, `kConstantName` for constants
- **Headers**: `#pragma once`, separate `.h`/`.cpp` for classes
- **Qt**: Use `AUTOMOC ON`, `Q_OBJECT` macro, mutex-protected shared state between ROS callbacks and Qt GUI thread
- **RViz plugins**: Inherit `rviz::Panel`, implement `onInitialize()` for ROS setup, export via `PLUGINLIB_EXPORT_CLASS`
- **Error handling**: `ROS_ERROR`/`ROS_WARN` for diagnostics, `ROS_INFO_ONCE` for one-time messages, `ROS_*_THROTTLE` for rate-limited output
- **USB**: Thread-safe with `std::mutex`, auto-reconnect on failure (configurable retries)

### Python (control, signal processing, bridge)
- **Style**: flake8 with max-line-length=120, ignore E501 (long lines) and W503 (line break before binary operator)
- **Naming**: `snake_case` for functions/variables, `PascalCase` for classes
- **ROS nodes**: `rospy.init_node()` in `__main__`, class-based structure with callbacks
- **Signal processing**: numpy/scipy for DSP (Hilbert, Butterworth, FFT), OpenCV for image processing
- **Imports**: stdlib → third-party → ROS, one import per line

### Build System
- **catkin**: Each package has `CMakeLists.txt` + `package.xml`
- **Dependencies**: Declare in both `find_package(catkin ...)` and `package.xml`
- **Qt5 plugins**: `set(CMAKE_AUTOMOC ON)`, `find_package(Qt5 COMPONENTS Widgets REQUIRED)`, build as shared library, install `plugin_description.xml`
- **Install targets**: `RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}`, `LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}`

## Dependencies (external, non-ROS)

- **libusb-1.0** -- EMAT USB device communication
- **Livox-LiDAR-SDK** -- static lib `liblivox_lidar_sdk_static.a` for `livox_ros_driver2`
- **librealsense2** (>= 2.50.0) -- Intel RealSense SDK
- **Qt5 Widgets** -- RViz panel plugins
- **Eigen3** -- linear algebra (ndt, FAST-LIO)
- **PCL** (>= 1.8) -- point cloud processing

## EMAT Package Notes

- **Protocol**: binary, header `0xAB`, CRC-8 (poly 0x07). Commands: `0x00` thickness, `0x01` waveform (4 chunks, ~8185 samples each, 8-bit ADC with DC offset 127), `0x03`/`0x04` set/get params.
- **USB**: CH346C chip, VID `0x1A86`, PID `0x55EB` (normal) / `0x55E0` (bootrom). Interface 2, bulk EP 0x06/0x86.
- **Topics**: `emat/waveform` (~40 Hz), `emat/thickness` (not populated), `emat/device_status` (latched).
- **Viz**: `rviz_emat_panel` is an RViz panel plugin (shared library). Add via RViz → Panels → Add New Panel → `emat/RvizEmatPanel`. Requires Qt5 Widgets and rviz.
- **Architecture**: `WaveformWidget` (QWidget) owns a ring buffer and QTimer (25ms/40Hz). The ROS callback pushes frames directly into the widget (mutex-protected). No Qt signal/slot cross-thread — the callback writes to the deque, the timer triggers `update()` → `paintEvent()` reads from it. The RViz plugin (`RvizEmatPanel`) wraps this widget as a `rviz::Panel`.

## NDT RViz Target Panel

`ndt/RvizTargetPanel` is an RViz panel plugin that displays the D435 RGB image and publishes click positions. Add via RViz → Panels → Add New Panel → `ndt/RvizTargetPanel`.

- Subscribes to `/d435/color/image_raw`
- On left-click, publishes `geometry_msgs/PointStamped` to `~click_point` (x=column, y=row, z=0, frame=`d435_color_optical_frame`)
- Since it's an RViz plugin, `~` resolves to the RViz node namespace, so the actual topic is `/rviz/click_point`
- Fixed 640×480 size
- `normal_ros.py` subscribes to this topic for click input (no OpenCV GUI needed)

## RViz on Jetson (headless/software rendering)

`bringup/rviz_safe_start.sh` kills any existing RViz, sets `LIBGL_ALWAYS_SOFTWARE=1`, and launches RViz. Use this on Jetson when GPU rendering is unavailable.

## Known Issues

- `bringup.launch` has RealSense camera commented out -- enable the `realsense2_camera` include when D435 is connected.
- `ndt/package.xml` is missing several dependencies found in CMakeLists.txt (`tf`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `mavros_msgs`, `nav_msgs`).
- `normal_ros.py` no longer uses OpenCV GUI -- click input comes from the RViz target panel.
- Experimental data records to `src/ndt/runs/` with timestamped directories containing `record_log.csv`, `rgb.mp4`, `depth.mp4`.

## 毕业设计 Context

> Source: 中期报告表 (2026-05-27), 华羽霄, 控制科学与工程, 电子科技大学

### 论文题目

基于多模态逐级物理约束的无人机自主电磁超声探测技术研究

### 核心问题

无人机无损检测(NDT)中，探头与被测表面的接触状态难以精确判定。传统单一超声回波阈值法在飞行扰动下不可靠，需融合EMAT回波、位姿误差、RGBD视觉三模态进行联合判别。

### 理论框架 (6条主线)

1. **电磁-弹性耦合** -- EMAT信号由激励→传播→反射→接收四环节串联，回波携带材料电磁属性和相对运动信息
2. **接触边界与升距效应** -- 接触状态由升距、法向约束、末端力学稳定性、回波可观测性共同定义；判别需时间窗口内的能量/相位/峰度/互相关/频谱重心
3. **多模态时序对齐** -- EMAT ~40 Hz / 飞控 ~100 Hz / RGBD ~30 Hz 异步采样，需统一时间基准+加权插值+独立编码器映射到统一隐空间
4. **物理约束注意力** -- 在Transformer注意力中加入物理约束矩阵P(时间间隔+空间残差)，以对数偏置形式注入先验
5. **不确定性传播** -- 协方差传播+贝叶斯融合似然+门控权重自适应分配模态可靠性
6. **数据闭环与可证伪性** -- 弱标签+人工复核；三类消融实验(去除物理约束/门控/平滑损失)验证各模块功能

### 控制闭环

视觉ROI→TF坐标变换→MAVROS局部位置目标→位置-速度串级PID→接近控制。多模态接触模型作为状态机观测层，双阈值+持续时间约束实现滞回判别。

### 评价指标

- 接触判别准确率 (Precision/Recall/F1)
- 临界状态误报率
- 接触概率时序平滑度
- 无人机闭环扫查稳定性
- 单帧推理延迟目标 <50ms, 系统刷新率目标 >30 Hz
