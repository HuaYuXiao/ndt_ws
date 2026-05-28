# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

ROS Noetic catkin workspace. Build from workspace root:

```bash
cd ~/ndt_ws && catkin_make           # full build
cd ~/ndt_ws && catkin_make --pkg ndt # single package
source ~/ndt_ws/devel/setup.bash     # source before running
```

Lint (matches CI):

```bash
flake8 --max-line-length=120 --ignore=E501,W503 src/bringup/scripts/ src/ndt/scripts/
```

Run launch files:

```bash
roslaunch bringup bringup_ndt.launch   # full system (MAVROS + LiDAR + FAST-LIO)
roslaunch ndt normal.launch                  # surface normal targeting + flight control
roslaunch ndt csrt.launch                    # CSRT visual tracking + flight control
roslaunch emat emat.launch                   # EMAT thickness gauge (requires sudo for USB)
```

## Platform

Target is **aarch64** (ARM, NVIDIA Jetson). PCL/VTK paths may differ on x86.

## Architecture

Autonomous drone visual targeting and approach system running on PX4 via MAVROS. The `ndt` package name is misleading -- it does not implement Normal Distributions Transform. It is a visual targeting and surface normal estimation system.

### Pipeline

1. **Sensors** -- Livox MID-360 LiDAR (`livox_ros_driver2`) + Intel RealSense D435 (`realsense2_camera`)
2. **LiDAR-inertial odometry** -- `fast_lio` (FAST-LIO 2.0, IEKF + ikd-Tree) produces real-time 6-DOF pose from LiDAR+IMU
3. **Pose bridging** -- `bringup/lidar_to_mavros.py` converts FAST-LIO odometry to MAVROS vision pose for PX4
4. **Visual targeting** (`ndt` package) -- two modes:
   - **Surface normal** (`normal.launch`): user clicks a surface, SVD fits a plane to depth data, computes approach pose with world-up constraint via TF
   - **CSRT tracking** (`csrt.launch`): OpenCV CSRT tracker selects a target in RGB, depth gives 3D relative position
5. **Flight control** -- `ndt/abs_pos.py` transforms target to global frame, publishes `mavros_msgs/PositionTarget` setpoints (DUMMY->TARGET->HOLD state machine)
6. **Recording** -- `ndt/record` logs CSV + RGB/depth video to `runs/` directories

### Packages

| Package | Lang | Purpose |
|---------|------|---------|
| `ndt` | C++17 / Python | Visual targeting, surface normal estimation, flight control, data recording |
| `fast_lio` | C++14 | LiDAR-inertial odometry (FAST-LIO 2.0) |
| `livox_ros_driver2` | C++14 | Livox MID-360/HAP LiDAR driver (Livox-LiDAR-SDK) |
| `livox_ros_driver` | C++11 | Older Livox driver (Livox-SDK, Hub/LVX support) -- not used in main pipeline |
| `realsense2_camera` | C++11 | Intel RealSense D435 camera driver |
| `realsense2_description` | -- | URDF/xacro models for RealSense cameras |
| `emat` | C++17 / Python | EMAT ultrasonic thickness gauge driver (USB via libusb, requires sudo) |
| `bringup` | Python | System integration: launches all subsystems, bridges odometry to MAVROS |

### Key ROS Topics

| Topic | Type | Source |
|-------|------|--------|
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | lidar_to_mavros.py |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | abs_pos.py |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO |
| `/ndt_normal/target_pose_d435` | `geometry_msgs/PoseStamped` | normal_ros.py |

## Dependencies (external, non-ROS)

- **libusb-1.0** -- EMAT USB device communication
- **Livox-LiDAR-SDK** -- static lib `liblivox_lidar_sdk_static.a` for `livox_ros_driver2`
- **librealsense2** (>= 2.50.0) -- Intel RealSense SDK

## EMAT Package Notes

The `emat` package is a self-contained EMAT thickness gauge driver. Key details:

- **Protocol**: binary, header `0xAB`, CRC-8 (poly 0x07). Commands: `0x00` thickness, `0x01` waveform (4 chunks, ~8185 samples each, 8-bit ADC with DC offset 127), `0x03`/`0x04` set/get params.
- **USB**: CH346C chip, VID `0x1A86`, PID `0x55EB` (normal) / `0x55E0` (bootrom). Interface 2, bulk EP 0x06/0x86.
- **Udev rule** (avoids needing `sudo`):
  ```bash
  sudo tee /etc/udev/rules.d/99-ch346-emat.rules << 'EOF'
  SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55eb", MODE="0666", GROUP="plugdev"
  SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55e0", MODE="0666", GROUP="plugdev"
  EOF
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
- **Topics**: `emat/waveform` (EmatWaveform, ~40 Hz), `emat/thickness` (EmatThickness -- not populated), `emat/device_status` (EmatDeviceStatus, latched).
- **Viz**: `rviz_emat_panel` is an RViz panel plugin (shared library). Add via RViz → Panels → Add New Panel → `emat/RvizEmatPanel`. Requires Qt5 Widgets and rviz.
- **Architecture**: `WaveformWidget` (QWidget) owns a ring buffer and QTimer (25ms/40Hz). The ROS callback pushes frames directly into the widget (mutex-protected). No Qt signal/slot cross-thread — the callback writes to the deque, the timer triggers `update()` → `paintEvent()` reads from it. The RViz plugin (`RvizEmatPanel`) wraps this widget as a `rviz::Panel`.

## RViz on Jetson (headless/software rendering)

`bringup/rviz_safe_start.sh` kills any existing RViz, sets `LIBGL_ALWAYS_SOFTWARE=1`, and launches RViz. Use this on Jetson when GPU rendering is unavailable.

## Known Issues

- `bringup_ndt.launch` has RealSense camera commented out -- enable the `realsense2_camera` include when D435 is connected.
- `ndt/package.xml` is missing several dependencies found in CMakeLists.txt (`tf`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `mavros_msgs`, `nav_msgs`).
- Python nodes use OpenCV GUI (`cv2.imshow`) and require a display. Use `LIBGL_ALWAYS_SOFTWARE=1` on headless systems.
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
