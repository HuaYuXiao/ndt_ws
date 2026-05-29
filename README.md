# 基于多模态逐级物理约束的无人机自主电磁超声探测技术研究

> ROS Noetic catkin workspace for autonomous drone-based EMAT non-destructive testing with multi-modal contact detection.

---

## Overview

This project implements an autonomous drone inspection system that combines electromagnetic acoustic transducer (EMAT) ultrasonic sensing, RGB-D vision, and LiDAR-inertial odometry for structural health monitoring. The system addresses the challenge of precisely determining probe-to-surface contact state during flight — a critical requirement for reliable ultrasonic thickness measurements on aerial platforms.

The core contribution is a **multi-modal fusion framework** that jointly models EMAT echo signals, drone pose errors, and visual ROI features through a physics-constrained Transformer architecture, achieving robust contact detection under flight disturbances.

## Key Features

- **EMAT Ultrasonic Driver** — CH346C USB-based EMAT probe driver with ~40 Hz waveform acquisition, automatic reconnection, and binary protocol with CRC-8 verification
- **Signal Processing Pipeline** — Hilbert envelope extraction, bandpass filtering, and 14-dimensional feature extraction (energy, peak amplitude, arrival time, spectral centroid, kurtosis, phase, 8-band energy decomposition)
- **Multi-Modal Temporal Alignment** — Synchronizes EMAT (~40 Hz), drone pose (~100 Hz), and visual ROI (~30 Hz) streams with nearest-neighbor and linear interpolation
- **Feature Encoding** — PyTorch encoders mapping heterogeneous sensor data to a shared embedding space (1D-CNN for ultrasound, MLP for pose, ResNet18 for visual)
- **Visual Targeting** — RViz-based target selection with SVD plane fitting for surface normal estimation and approach pose generation
- **Visual Tracking** — OpenCV CSRT tracker with 3D Kalman filtering for real-time target following
- **Flight Control** — MAVROS integration with 3-stage state machine (DUMMY → TARGET → HOLD) and proportional velocity control
- **Multi-Modal Data Recording** — Synchronous recording of EMAT waveforms, drone pose, RGB-D video, and flight setpoints

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Sensors                                  │
│  ┌──────────┐  ┌──────────────┐  ┌───────────────────────────┐ │
│  │ EMAT     │  │ RealSense    │  │ Livox MID-360 + IMU       │ │
│  │ Probe    │  │ D435 RGB-D   │  │ (LiDAR-inertial)          │ │
│  └────┬─────┘  └──────┬───────┘  └─────────────┬─────────────┘ │
│       │               │                         │               │
│       ▼               ▼                         ▼               │
│  ┌─────────┐   ┌───────────┐            ┌──────────────┐       │
│  │ EMAT    │   │ Visual    │            │ FAST-LIO 2.0 │       │
│  │ Feature │   │ Tracking  │            │ Odometry     │       │
│  │ Extract │   │ (CSRT)    │            └──────┬───────┘       │
│  └────┬────┘   └─────┬─────┘                   │               │
│       │               │                         ▼               │
│       │               │              ┌──────────────────┐       │
│       │               │              │ lidar_to_mavros  │       │
│       │               │              └────────┬─────────┘       │
└───────┼───────────────┼───────────────────────┼─────────────────┘
        │               │                       │
        ▼               ▼                       ▼
   /emat/features  /relative_pos      /mavros/local_position/pose
        │               │                       │
        └───────────────┼───────────────────────┘
                        ▼
              ┌───────────────────┐
              │ Temporal Alignment │
              │ (Phase 3)         │
              └─────────┬─────────┘
                        ▼
              /ndt/multimodal_features
                        │
                        ▼
              ┌───────────────────┐
              │ Multi-Modal       │
              │ Fusion Model      │
              │ (Transformer)     │
              └─────────┬─────────┘
                        ▼
              /mavros/setpoint_raw/local → PX4
```

### Packages

| Package | Purpose |
|---------|---------|
| `emat` | EMAT probe driver, Qt5 waveform visualizer, ROS message definitions |
| `ndt` | Signal processing, visual targeting, flight control, feature encoding |
| `bringup` | System integration launch files and LiDAR-MAVROS bridge |
| `record` | Multi-modal data recorder for experiments |
| `fast_lio` | FAST-LIO 2.0 LiDAR-inertial odometry |
| `livox_ros_driver2` | Livox MID-360 LiDAR driver |
| `realsense2_camera` | Intel RealSense D435 camera driver |

### Key ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/emat/waveform` | `EmatWaveform` | Raw ultrasonic waveform (~40 Hz) |
| `/emat/features` | `EmatFeatures` | Extracted signal features |
| `/emat/envelope` | `EmatEnvelope` | Hilbert envelope (low-pass filtered) |
| `/mavros/local_position/pose` | `PoseStamped` | Drone odometry |
| `/relative_pos` | `PointStamped` | Visual target position (base_link frame) |
| `/ndt/multimodal_features` | `MultiModalFeatures` | Aligned multi-modal feature vector |
| `/mavros/setpoint_raw/local` | `PositionTarget` | Flight control setpoints |

## Prerequisites

- **OS**: Ubuntu 20.04 (Jetson aarch64 or x86_64)
- **ROS**: Noetic
- **Python**: 3.8+
- **CUDA**: 11.4+ (Jetson) — required for PyTorch inference

### System Dependencies

```bash
sudo apt install libusb-1.0-0-dev  # EMAT USB communication
```

### Python Dependencies

```bash
pip install numpy scipy torch torchvision
```

## Installation & Quick Start

### 1. Clone and Build

```bash
cd ~/ndt_ws
catkin_make
source ~/ndt_ws/devel/setup.bash
```

### 2. USB Permissions (EMAT Probe)

```bash
sudo tee /etc/udev/rules.d/99-ch346-emat.rules << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55eb", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55e0", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Reconnect the USB device after setup.

### 3. Launch the Full System

```bash
roslaunch bringup bringup.launch
```

This starts MAVROS, LiDAR driver, FAST-LIO, RealSense camera, EMAT driver, feature extractor, and RViz.

### 4. Launch Individual Components

```bash
# Visual targeting with surface normal estimation
roslaunch ndt normal.launch

# CSRT visual tracking mode
roslaunch ndt csrt.launch

# Data recording
roslaunch record record.launch
```

## EMAT Driver

### Hardware Specifications

| Item | Value |
|------|-------|
| Probe | EMAT electromagnetic ultrasonic pen probe |
| USB Chip | WCH CH346C_M0 (VID: `0x1A86`) |
| Normal Mode PID | `0x55EB` |
| Bootrom/ISP Mode PID | `0x55E0` (abnormal — requires USB re-plug) |
| Interface | USB Interface 2 (Vendor Specific) |
| Endpoints | EP `0x06` OUT / EP `0x86` IN (Bulk, 512B) |

### Verify Connection

```bash
lsusb | grep 1a86
```

Expected: `1a86:55eb` (normal mode). If `1a86:55e0`, the probe is in bootrom mode and needs a physical power cycle.

### ROS Messages

**EmatWaveform** — Raw waveform data
```
time    stamp
uint32  sample_count
uint8[] raw_data                  # 8-bit ADC, DC offset 127
uint32  speed_of_voice            # m/s
uint8   average_count
float32 excitation_frequency_mhz
float32 thickness_mm
string  device_id
```

**EmatFeatures** — Extracted signal features
```
time    stamp
float32 energy
float32 peak_amplitude
float32 arrival_time
float32 spectral_centroid
float32 kurtosis
float32 phase
float32[8] band_energies
float32 thickness_estimate
```

**EmatEnvelope** — Low-pass filtered Hilbert envelope
```
time    stamp
float32[] envelope
uint32  sample_count
float32 sampling_rate
```

**MultiModalFeatures** — Aligned multi-modal vector
```
Header header
float32 energy, peak_amplitude, arrival_time, spectral_centroid, kurtosis, phase
float32[8] band_energies
float32 thickness_estimate
float32 pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw
float32 vel_x, vel_y, vel_z
float32 roi_x, roi_y, roi_z
time visual_stamp
```

### Driver Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `read_interval_ms` | `100` | Waveform acquisition interval (ms) |
| `num_chunks` | `4` | Chunks per waveform (~8185 samples each) |
| `default_speed` | `3230.0` | Default speed of sound (m/s) |
| `chunk_delay_ms` | `10` | Delay between chunk reads (ms) |
| `read_timeout_ms` | `500` | USB read timeout (ms) |
| `max_startup_retries` | `30` | Max connection retries at startup |
| `max_consecutive_failures` | `5` | Failures before auto-reconnect |
| `reconnect_delay_s` | `3` | Reconnect wait time (s) |

### Communication Protocol

Binary protocol with CRC-8 (polynomial `0x07`, init `0x00`).

Packet format: `[0xAB] [0x00] [0x01] [func] [len_hi] [len_lo] [payload...] [CRC]`

| Function Code | Command | Payload |
|---------------|---------|---------|
| `0x00` | Read thickness | None |
| `0x01` | Read waveform | 1 byte chunk index (1-4) |
| `0x03` | Set parameters | 5 bytes |
| `0x04` | Get parameters | None |

Waveform data: 4 chunks × ~8185 samples = ~32740 points (1 MHz sampling, 8-bit ADC).

### Auto-Reconnect

The driver has two recovery mechanisms:
1. **Consecutive failure reconnect** — After `max_consecutive_failures` consecutive errors, automatically reopens the USB connection.
2. **Background reconnect thread** — Scans USB bus every 5 seconds when disconnected, distinguishing normal mode (`0x55EB`) from bootrom mode (`0x55E0`).

### Reference Sound Speeds

| Material | Speed (m/s) |
|----------|-------------|
| Steel | 3230 |
| Cast Iron | 2210 |
| Aluminum | 3100 |
| Copper | 2320 |

## Signal Processing Pipeline

The `emat_feature_extractor` node processes raw waveforms through:

1. **DC offset removal** — Subtract 127 from uint8 samples
2. **Hilbert transform** — Compute analytic signal → envelope + instantaneous phase
3. **Envelope slicing** — Extract samples `[slice_start, slice_end)` (default 200–5000)
4. **Low-pass filtering** — FIR filter (Hamming window, configurable cutoff)
5. **Feature extraction** — 14-dimensional feature vector
6. **Thickness estimation** — `d = (speed_of_sound × arrival_time) / 2`

### Feature Extractor Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sampling_rate` | `1000000.0` | ADC sampling rate (Hz) |
| `slice_start` | `200` | Envelope slice start index |
| `slice_end` | `5000` | Envelope slice end index |
| `lp_cutoff` | `10.0` | Low-pass filter cutoff (Hz) |
| `lp_order` | `256` | FIR filter order (taps = order + 1) |
| `arrival_threshold` | `0.1` | Arrival detection threshold (fraction of peak) |
| `speed_of_sound` | `3240.0` | Default sound speed (m/s) |

## Multi-Modal Temporal Alignment

The `temporal_aligner` node synchronizes three sensor streams:

- **EMAT features** (~40 Hz) — anchor clock
- **Drone pose** (~30 Hz) — nearest-neighbor interpolation
- **Visual ROI** (~30 Hz) — linear interpolation

Velocity is estimated via least-squares linear fit over a configurable window.

### Alignment Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pose_buffer_duration` | `0.5` | Pose buffer window (s) |
| `visual_buffer_duration` | `0.5` | Visual buffer window (s) |
| `velocity_window` | `0.2` | Velocity estimation window (s) |
| `max_pose_age` | `0.1` | Max acceptable pose latency (s) |
| `max_visual_age` | `0.15` | Max acceptable visual latency (s) |

## Feature Encoders

Pure PyTorch module (`ndt.lib.feature_encoders`) with no ROS dependency:

| Encoder | Input | Architecture | Output |
|---------|-------|--------------|--------|
| `UltrasoundEncoder` | (batch, 14) | 1D-CNN + positional encoding | (batch, d_model) |
| `PoseEncoder` | (batch, 9) | 3-layer MLP | (batch, d_model) |
| `VisualEncoder` | (batch, 3, 224, 224) | ResNet18 + projection head | (batch, d_model) |
| `MultiModalEncoder` | All three | Stacks outputs | (batch, 3, d_model) |

## Data Recording

The `multimodal_recorder` node synchronously records:
- EMAT waveforms and features
- Drone pose and flight setpoints
- RGB and depth video
- Timestamped CSV logs

Output directory: `src/ndt/runs/<timestamp>/`

## Research Context

### Thesis

**Title**: 基于多模态逐级物理约束的无人机自主电磁超声探测技术研究

**Author**: 华羽霄, 控制科学与工程, 电子科技大学

### Core Problem

Contact state determination between EMAT probe and surface is unreliable with single-mode ultrasonic threshold methods under flight disturbances. The system fuses EMAT echoes, pose errors, and RGB-D vision for joint discrimination.

### Theoretical Framework

1. **Electromagnetic-elastic coupling** — EMAT signal propagation carries material and motion information
2. **Contact boundary & lift-off effect** — Multi-feature contact discrimination (energy, phase, kurtosis, cross-correlation, spectral centroid)
3. **Multi-modal temporal alignment** — Unified time base with weighted interpolation and independent encoders
4. **Physics-constrained attention** — Transformer attention with dynamics consistency matrix P
5. **Uncertainty propagation** — Covariance propagation + Bayesian fusion + gated modality weighting
6. **Data loop & falsifiability** — Weak labels + manual review; three ablation experiments

### Evaluation Metrics

- Contact detection accuracy (Precision / Recall / F1)
- Critical state false alarm rate
- Contact probability temporal smoothness
- Closed-loop scanning stability
- Single-frame inference latency (< 50ms target)
- System refresh rate (> 30 Hz target)
