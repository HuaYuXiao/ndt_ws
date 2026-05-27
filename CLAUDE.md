# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

ROS Noetic catkin workspace. Build from workspace root:

```bash
cd ~/ndt_ws && catkin_make           # full build
cd ~/ndt_ws && catkin_make --pkg ndt # single package
source ~/ndt_ws/devel/setup.bash     # source before running
```

Run launch files:

```bash
roslaunch robot_bringup bringup_ndt.launch   # full system (MAVROS + LiDAR + FAST-LIO)
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
3. **Pose bridging** -- `robot_bringup/lidar_to_mavros.py` converts FAST-LIO odometry to MAVROS vision pose for PX4
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
| `robot_bringup` | Python | System integration: launches all subsystems, bridges odometry to MAVROS |

### Key ROS Topics

| Topic | Type | Source |
|-------|------|--------|
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | lidar_to_mavros.py |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | abs_pos.py |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO |
| `/ndt/aruco_pose/` | `geometry_msgs/PoseStamped` | aruco_pose (C++) |
| `/ndt_normal/target_pose_d435` | `geometry_msgs/PoseStamped` | normal_ros.py |

## Dependencies (external, non-ROS)

- **libaruco** -- ArUco marker detection (`/usr/local/lib/libaruco.so`), used by `ndt/aruco_pose`
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
- **Topics**: `emat/waveform` (EmatWaveform, ~88 Hz), `emat/thickness` (EmatThickness -- not populated), `emat/device_status` (EmatDeviceStatus, latched).
- **Unused code**: `ch346_driver.h/.cpp` and `protocol_codec.h/.cpp` define a cleaner abstraction layer but are not compiled or used by the node.
- **Viz**: `emat_waveform_viz_node` is a C++ Qt5 widget (`roslaunch emat emat_viz.launch`). Requires Qt5 Widgets. On Jetson, use `LIBGL_ALWAYS_SOFTWARE=1` for software rendering.
- **Architecture**: `WaveformWidget` (QWidget) owns a ring buffer and QTimer (25ms/40Hz). The ROS callback pushes frames directly into the widget (mutex-protected). No Qt signal/slot cross-thread — the callback writes to the deque, the timer triggers `update()` → `paintEvent()` reads from it.

## RViz on Jetson (headless/software rendering)

`robot_bringup/rviz_safe_start.sh` kills any existing RViz, sets `LIBGL_ALWAYS_SOFTWARE=1`, and launches RViz. Use this on Jetson when GPU rendering is unavailable.

## Known Issues

- `bringup_ndt.launch` has RealSense camera commented out -- enable the `realsense2_camera` include when D435 is connected.
- `ndt/normal.launch` references `config/aruco_pose.yaml` which does not exist; `config/aruco_estimator.yaml` exists instead.
- `ndt/package.xml` is missing several dependencies found in CMakeLists.txt (`tf`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `mavros_msgs`, `nav_msgs`).
- Python nodes use OpenCV GUI (`cv2.imshow`) and require a display. Use `LIBGL_ALWAYS_SOFTWARE=1` on headless systems.
- Experimental data records to `src/ndt/runs/` with timestamped directories containing `record_log.csv`, `rgb.mp4`, `depth.mp4`.
