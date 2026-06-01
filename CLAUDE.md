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
roslaunch record record.launch            # multimodal data recorder
roslaunch record record_bag.launch        # rosbag recorder
rosrun emat emat_thickness_gauge_node     # EMAT driver only (for debugging)
rosrun emat emat_feature_extractor.py     # feature extractor only

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
- **USB**: CH346C chip, VID `0x1A86`, PID `0x55EB` (normal) / `0x55E0` (bootrom). Interface 2 (vendor, class 255), bulk EP 0x06 OUT / 0x86 IN. Interface 0 is CDC-ACM (kernel cdc_acm driver → `/dev/ttyACM0`), not used by our code.
- **Topics**: `emat/waveform` (~40 Hz), `emat/thickness` (not populated), `emat/device_status` (latched, 2s interval).
- **Viz**: `rviz_emat_panel` is an RViz panel plugin (shared library). Add via RViz → Panels → Add New Panel → `emat/RvizEmatPanel`.
- **Architecture**: `WaveformWidget` (QWidget) owns a ring buffer and QTimer (25ms/40Hz). ROS callback pushes frames directly into the widget (mutex-protected). No Qt signal/slot cross-thread.
- **Driver params** (set in launch or via `rosrun _param:=value`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `read_interval_ms` | 100 | Waveform acquisition interval (ms) → 10 Hz by default |
| `num_chunks` | 4 | Chunks per waveform |
| `chunk_delay_ms` | 30 | Delay after write before read (CH346C processing time) |
| `write_timeout_ms` | 200 | USB bulk write timeout (ms) |
| `read_timeout_ms` | 500 | USB bulk read timeout (ms) |
| `max_startup_retries` | 30 | Max connection attempts at startup |
| `max_consecutive_failures` | 5 | Protocol errors before reconnect |
| `max_chunk_retries` | 2 | Retries per chunk on transient error |
| `max_read_timeouts` | 5 | Consecutive read timeouts before declaring device unresponsive |
| `reconnect_delay_s` | 3 | Wait time between disconnect and reconnect |

- **Auto-recovery**: Five-level recovery — transfer retry (TIMEOUT/IO) → clear_halt (PIPE) → chunk retry with backoff → close+reopen (protocol errors) → background reconnect thread (5s interval).

- **Known hardware issue**: EMAT probe excitation (~4 MHz, high voltage) generates EMI that can disrupt CH346C USB communication. USB writes succeed but reads timeout — the MCU receives commands but cannot return data during EMI events. Mitigation requires ferrite cores on USB cable, shielded USB cable, USB isolator, or increased physical separation. See `issues/2026-06-01-emat-usb-disconnection-emi.md`.

## NDT RViz Target Panel

`ndt/RvizTargetPanel` is an RViz panel plugin that displays the D435 RGB image and publishes click positions. Add via RViz → Panels → Add New Panel → `ndt/RvizTargetPanel`.

- Subscribes to `/d435/color/image_raw`
- On left-click, publishes `geometry_msgs/PointStamped` to `~click_point` (x=column, y=row, z=0, frame=`d435_color_optical_frame`)
- Since it's an RViz plugin, `~` resolves to the RViz node namespace, so the actual topic is `/rviz/click_point`
- Fixed 640×480 size
- `normal_ros.py` subscribes to this topic for click input (no OpenCV GUI needed)

## RViz on Jetson (headless/software rendering)

`bringup/rviz_safe_start.sh` kills any existing RViz, sets `LIBGL_ALWAYS_SOFTWARE=1`, and launches RViz. Use this on Jetson when GPU rendering is unavailable.

## EMAT Feature Extractor (`emat_feature_extractor.py`)

Signal processing node: subscribes `/emat/waveform` → publishes `/emat/features` and `/emat/envelope`.

**Processing pipeline** (matching MATLAB `extractDelay.m` reference):
1. Slice raw waveform `[slice_start, slice_end)` — defaults to `[200, 1000)` to skip excitation pulse and EM crosstalk blind zone
2. DC removal: `signal = raw - 127.0`
3. Hilbert transform → analytic signal → envelope = `|analytic|`
4. Low-pass filter envelope: FIR filter (Hamming window, `lp_order`+1 taps, cutoff `lp_cutoff` Hz), zero-phase via `filtfilt`
5. Feature extraction: energy, peak amplitude, arrival time (threshold detection), spectral centroid, kurtosis, instantaneous phase, 8-band energy decomposition
6. Thickness estimate: dual-peak method — `d = |x1 - x2| / fs × v / 2 × 1000` (mm)

**Parameters** (all in `~` private namespace):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `waveform_topic` | `/emat/waveform` | Input topic |
| `features_topic` | `/emat/features` | Feature output |
| `envelope_topic` | `/emat/envelope` | Envelope output |
| `sampling_rate` | `1000000.0` | ADC sampling rate (Hz) |
| `slice_start` | `200` | Envelope slice start (skip blind zone) |
| `slice_end` | `1000` | Envelope slice end |
| `lp_cutoff` | `10.0` | Low-pass filter cutoff (Hz) |
| `lp_order` | `256` | FIR filter order (taps = order+1) |
| `arrival_threshold` | `0.1` | Arrival time detection (fraction of peak) |
| `speed_of_sound` | `3240.0` | Default sound speed (m/s, aluminum shear wave) |

Reference: 孙广宇《基于电磁超声体波的铝板缺陷检测》(HIT, 2025). Summary stored at `~/.claude/projects/-home-cwkj-ndt-ws/memory/reference_emat_thickness_benchmark.md`.

## Known Issues

- **EMAT USB EMI disconnection (#4)**: EMAT probe excitation EMI disrupts CH346C USB chip, causing read timeouts and device disconnections. Software recovery is implemented but hardware mitigation (ferrite cores, shielded cable, USB isolator) is needed for reliable communication. See `issues/2026-06-01-emat-usb-disconnection-emi.md`.
- `bringup.launch` has RealSense camera included — disable the `realsense2_camera` include when D435 is not connected.
- `ndt/package.xml` is missing several dependencies found in CMakeLists.txt (`tf`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `mavros_msgs`, `nav_msgs`).
- `normal_ros.py` no longer uses OpenCV GUI — click input comes from the RViz target panel.
- Experimental data records to `src/ndt/runs/` with timestamped directories containing `record_log.csv`, `rgb.mp4`, `depth.mp4`.
- `emat/launch/` directory is empty — there is no standalone EMAT launch file. EMAT nodes are launched from `bringup.launch`.

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
