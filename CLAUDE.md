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
| Qt | Qt5 Widgets | RViz panel plugins (emat, ndt, record) |
| Math | Eigen3 | SVD plane fitting, matrix ops |
| Point Cloud | PCL >= 1.8 | FAST-LIO, livox drivers |
| Vision | OpenCV | CSRT tracker, cv_bridge |
| USB | libusb-1.0 | EMAT CH346C communication |
| LiDAR SDK | Livox-LiDAR-SDK | Static lib `liblivox_lidar_sdk_static.a` |
| Camera SDK | librealsense2 >= 2.50.0 | RealSense D435 |
| Flight | MAVROS + PX4 | mavros_msgs, tf2 |
| ML | PyTorch 2.1 | Contact detection model (CUDA on Jetson) |
| GUI | PyQt5 + matplotlib | Labeling tool, RViz panel plugins |
| Lint | flake8 | max-line-length=120, ignore E501,W503 |

## Common Commands

```bash
# Build
cd ~/ndt_ws && catkin_make                # full workspace
cd ~/ndt_ws && catkin_make --pkg emat     # single package
cd ~/ndt_ws && catkin_make --pkg ndt
cd ~/ndt_ws && catkin_make --pkg record
source ~/ndt_ws/devel/setup.bash          # source before running

# Lint (matches CI)
flake8 --max-line-length=120 --ignore=E501,W503 src/bringup/scripts/ src/ndt/scripts/

# Run
roslaunch bringup bringup.launch          # full system (MAVROS + LiDAR + FAST-LIO + RViz)
roslaunch ndt normal.launch               # surface normal targeting + flight control
roslaunch ndt thesis_pipeline.launch      # full thesis pipeline (normal + physics detector)
roslaunch ndt csrt.launch                 # CSRT visual tracking + flight control
roslaunch record record.launch            # multimodal data recorder (or use RViz RecordPanel)
rosrun emat emat_thickness_gauge_node     # EMAT driver only (for debugging)
rosrun emat emat_feature_extractor.py     # feature extractor only

# Data labeling
python3 src/record/scripts/label_tool.py src/record/datasets/20260604/0
python3 src/record/scripts/label_tool.py  # file dialog

# Training
python3 src/ndt/scripts/train_contact_detector.py --epochs 30 --batch 4

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
  record/multimodal_recorder ──→ frame_index.csv + depth PNGs + RGB/depth video + EMAT waveform CSV to datasets/
  record/RvizRecordPanel ──→ one-click toggle in RViz to start/stop recording

Data conversion:
  rosbag_to_dataset.py ──→ numpy arrays (pose_odom.npy, depth_frames.npy, emat_features.npy, normals.npy, contact_labels.npy)
```

### Packages

| Package | Lang | Build Target | Purpose |
|---------|------|-------------|---------|
| `bringup` | Python | — | System integration, launches all subsystems, lidar_to_mavros bridge |
| `ndt` | C++17/Python | `rviz_target_panel` (RViz plugin) | Visual targeting, surface normal estimation, flight control, feature extraction |
| `emat` | C++17 | `emat_thickness_gauge_node`, `rviz_emat_panel` (RViz plugin) | EMAT USB driver, waveform visualization |
| `record` | C++17 | `multimodal_recorder`, `rviz_record_panel` (RViz plugin) | Multi-modal data recording + RViz one-click record panel |
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
| `MultiModalFeatures` | emat | `energy`, `peak_amplitude`, `arrival_time`, `spectral_centroid`, `kurtosis`, `phase`, `band_energies[8]`, `thickness_estimate`, `pose_x/y/z`, `pose_roll/pitch/yaw`, `vel_x/y/z`, `roi_x/y/z` |
| `CustomMsg` | livox_ros_driver2 | `points(CustomPoint[])`, `lidar_id` |

### RViz Panel Plugins

| Plugin | Class | Description |
|--------|-------|-------------|
| `ndt/RvizTargetPanel` | `ndt::RvizTargetPanel` | D435 RGB image display, click publishes `PointStamped` to `~click_point` |
| `emat/RvizEmatPanel` | `emat::RvizEmatPanel` | EMAT waveform visualization with DC removal and grid overlay |
| `record/RvizRecordPanel` | `record::RvizRecordPanel` | One-click start/stop data recording toggle button with elapsed timer |

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
| `/ndt/contact_probability` | `Float32MultiArray` | physics_constrained_detector.py |
| `/ndt/multimodal_features` | `MultiModalFeatures` | temporal_alignment.py |

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
- **sys.path**: catkin's `exec()` wrapper does NOT add the script directory to `sys.path`. Scripts that import local modules (e.g., `from physics_attention import ...`) must add `sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))` at the top.
- **Naming**: `snake_case` for functions/variables, `PascalCase` for classes
- **ROS nodes**: `rospy.init_node()` in `__main__`, class-based structure with callbacks
- **Signal processing**: numpy/scipy for DSP (Hilbert, Butterworth, FFT), OpenCV for image processing
- **Imports**: stdlib → third-party → ROS, one import per line

### Build System
- **catkin**: Each package has `CMakeLists.txt` + `package.xml`
- **Dependencies**: Declare in both `find_package(catkin ...)` and `package.xml`
- **Qt5 plugins**: `set(CMAKE_AUTOMOC ON)`, `find_package(Qt5 COMPONENTS Widgets REQUIRED)`, build as shared library, install `plugin_description.xml`
- **RViz plugin registration**: `package.xml` must have `<export><rviz plugin="${prefix}/plugin_description.xml" /></export>` — without this, RViz cannot discover the plugin
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

## Record Package

`record/RvizRecordPanel` is an RViz panel plugin with a toggle button to start/stop data recording. Add via RViz → Panels → Add New Panel → `record/RvizRecordPanel`.

- Uses `QProcess` to spawn `rosrun record multimodal_recorder`
- Button toggles between red "● 开始录制" and green "■ 停止录制"
- Status label shows elapsed time during recording and save path after stopping
- Destructor ensures the child process is terminated on RViz exit

**Recorder output** (`src/record/datasets/YYYYMMDD/N/`):
- `frame_index.csv` — per-frame aligned data (pose, normals, EMAT features, contact probability)
- `depth/*.png` — 16-bit depth images (visualization/debugging)
- `rgb.mp4` / `depth.mp4` — compressed video (visualization only)
- `emat_waveform.csv` — raw EMAT waveform hex data
- `record_log.csv` — legacy format (backward compat)

**Data conversion**: `rosbag_to_dataset.py <run_dir>` converts CSV recordings to a single `dataset.npz` for PyTorch training. The model (`physics_constrained_detector.py`) consumes pre-extracted features, not raw depth — depth PNGs are retained only for visualization.

```bash
python3 rosbag_to_dataset.py datasets/20260604/0          # training data only (no depth)
python3 rosbag_to_dataset.py datasets/20260604/0 --include-depth  # include 16-bit depth
```

`dataset.npz` keys: `timestamps`(N,), `pose`(N,6), `normals`(N,3), `emat_features`(N,14), `contact_prob`(N,). All arrays float32/float64, NaN for missing data.

**EMAT optional**: The recorder gracefully handles EMAT probe absence — EMAT feature columns in `frame_index.csv` are filled with `nan` when no EMAT data is available.

**Dataset directory structure**:
```
src/record/datasets/
├── YYYYMMDD/N/     # New format recordings (2026-06-04+)
│   ├── dataset.npz         # Training data (timestamps, pose, normals, emat_features, contact_prob)
│   ├── frame_index.csv      # Per-frame aligned data
│   ├── depth/               # 16-bit depth PNGs
│   ├── rgb.mp4 / depth.mp4  # Video (visualization only)
│   ├── emat_waveform.csv    # Raw EMAT waveforms
│   ├── record_log.csv       # Legacy log
│   └── metadata.json
```

## Contact Labeling Tool (`label_tool.py`)

PyQt5 GUI for annotating contact/no-contact labels on recorded datasets. Displays synchronized RGB/depth video, EMAT features, altitude trajectory, and an interactive timeline.

```bash
python3 src/record/scripts/label_tool.py src/record/datasets/20260604/0   # load specific run
python3 src/record/scripts/label_tool.py                                       # file dialog
```

**Keyboard shortcuts**: `←`/`→` navigate frames, `Shift+←`/`→` jump ±10, `Space` play/pause, `C` mark contact start, `V` mark contact end, `U` undo, `S` save, `Delete` remove region at cursor. Timeline supports click-to-jump and Ctrl+drag to create regions.

**Output**: Updates `contact_prob` in `dataset.npz`, `frame_index.csv`, and `metadata.json`. Binary labels: 1.0 = contact, 0.0 = no-contact.

## RViz on Jetson (headless/software rendering)

`bringup/rviz_safe_start.sh` kills any existing RViz, sets `LIBGL_ALWAYS_SOFTWARE=1`, and launches RViz. Use this on Jetson when GPU rendering is unavailable.

`bringup.launch` auto-maximizes RViz via `xdotool`. This requires `DISPLAY=:0` to be set explicitly when launching from SSH/tmux sessions (already configured in the launch file).

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

## Physics-Constrained Contact Detector

### Model Architecture (`src/ndt/scripts/physics_attention.py`)

Pure PyTorch module (no ROS dependency). Implements the physics-constrained Transformer from thesis Chapter 3.

**Core components**:
- `build_physics_constraint_matrix()` — Constructs P ∈ R^(T×T) from temporal proximity, spatial proximity, and normal consistency
- `PhysicsConstrainedAttention` — Multi-head attention with physics bias: `softmax(QKᵀ/√d + λ·P) V`
- `PhysicsConstrainedTransformerEncoder` — Single encoder layer (attention + FFN + LayerNorm)
- `ContactClassifier` — Classification head (d_model → 64 → 2)
- `PhysicsConstrainedContactDetector` — Full model: input projection → positional encoding → N encoder layers → classifier

**Model parameters**: d_vis=6, d_model=128, n_heads=8, n_layers=2, dropout=0.1 (537K params)

**Inputs**:
- `vis_features`: (B, T, 6) — 6D visual features from depth ROI (mean_depth, depth_var, grad_x, grad_y, norm_depth, fill_ratio)
- `timestamps`: (B, T) — relative timestamps (seconds)
- `positions`: (B, T, 3) — xyz positions (meters)
- `normals`: (B, T, 3) — surface normal vectors (optional)

**Outputs**:
- `logits`: (B, T, 2) — contact/no-contact logits
- `attn_weights`: List of attention weight matrices

**Loss functions**:
- `compute_smoothness_loss()` — Temporal smoothness: penalizes abrupt probability changes between adjacent frames
- `compute_physics_consistency_loss()` — Physics consistency: contact probability should correlate with depth gradient

### Inference (`src/ndt/scripts/physics_constrained_detector.py`)

ROS node wrapping the model. Subscribes to depth/camera_info/pose/click/normal topics, extracts 6D visual features from depth ROI, runs sliding-window inference, publishes `Float32MultiArray` contact probability.

```bash
roslaunch ndt thesis_pipeline.launch    # normal + physics_detector (full thesis pipeline)
roslaunch ndt physics_detector.launch   # detector only
```

**Checkpoint loading**: Supports both raw state_dict and training checkpoint format (`{'model_state': ..., 'val_f1': ..., 'epoch': ...}`) via `checkpoint.get('model_state', checkpoint)`.

**CRITICAL — Feature extraction consistency**: The 6D visual features MUST be computed identically between training (`train_contact_detector.py:extract_depth_features`) and inference (`physics_constrained_detector.py:_extract_visual_features`). Both use raw uint16 mm values as the base, then normalize:
- `mean_depth`: `mean_d / 1000.0` (meters)
- `depth_var`: `var_d / 1e6` (m²)
- `grad_x/grad_y`: `cv2.Sobel(raw_mm) / 1000.0`
- `norm_depth`: `mean_d / 5000.0`
- `fill_ratio`: `len(valid) / patch_size`

Any mismatch in normalization (e.g., computing features on already-converted meter values) will produce inputs orders of magnitude off from training distribution, causing the model to output constant ~0.5 probability.

### Training (`src/ndt/scripts/train_contact_detector.py`)

```bash
python3 src/ndt/scripts/train_contact_detector.py --epochs 50 --batch 4 --window 64 --stride 16
```

**Data pipeline**:
1. Loads all labeled `dataset.npz` from `src/record/datasets/`
2. Extracts 6D depth features from depth PNGs (if available) or uses pose (xyz + rpy) as fallback
3. Builds sliding windows (64 frames, stride 16)
4. Pre-computes physics constraint matrices for each window
5. 80/20 train/val split

**Training config**:
- Optimizer: AdamW (lr=1e-3, weight_decay=1e-4)
- Scheduler: CosineAnnealing
- Loss: CrossEntropy (3x weight on contact class) + 0.1 × smoothness loss
- Gradient clipping: max_norm=1.0

**Output**:
- Best model: `src/ndt/models/contact_detector_best.pt`
- Training log: `src/ndt/models/training_log.csv` (epoch, lr, train/val loss, acc, precision, recall, F1)

**Current results** (14 labeled recordings, 4786 windows):
- Val F1: 0.845 (best, epoch 26)
- Val Accuracy: 92.3%
- Precision: 0.708, Recall: 0.954

## Thesis Writing (LaTeX)

The Master's thesis is in `毕业设计/论文/` using the `thesis-uestc` document class (UESTC official template). Two entry points exist: `main.tex` (single-file) and `main_multifile.tex` (split into `chapters/` and `misc/`). Always edit via the multi-file version.

**Compilation** (requires MiKTeX with XeLaTeX on Windows; `latexmk` needs Strawberry Perl installed):
```powershell
$env:PATH = "C:\Program Files\MiKTeX\miktex\bin\x64;" + $env:PATH
Set-Location "C:\Users\easonhua\OneDrive\UESTC\ndt_ws\毕业设计\论文"
xelatex -synctex=1 -interaction=nonstopmode main_multifile.tex
bibtex main_multifile
bibtex accomplish               # \thesisaccomplish needs a separate bibliography
xelatex -synctex=1 -interaction=nonstopmode main_multifile.tex
xelatex -synctex=1 -interaction=nonstopmode main_multifile.tex
```

**Key facts:**
- Engine: XeLaTeX only (thesis-uestc.cls line 24: `\RequireXeTeX`)
- Fonts: SimSun/SimHei (Chinese), Times New Roman (English) — available on Windows, substitute warnings on other platforms
- References: `reference.bib` (49 entries), `thesis-uestc.bst` style, BibTeX pass required
- Accomplish: `publications.bib` (placeholder), `bibtex accomplish` after first xelatex pass
- Output: PDF with TOC, cross-references, bibliography (5 chapters + appendices)
- Recompile after any `.tex` or `.bib` change — the auto-recompile rule is stored in memory
- If PDF is locked: `taskkill /f /im Acrobat.exe; taskkill /f /im msedge.exe` then delete and recompile

**File structure:**
```
毕业设计/
├── 论文/
│   ├── main.tex                 # single-file version
│   ├── main_multifile.tex       # multi-file entry point
│   ├── thesis-uestc.cls         # UESTC official class
│   ├── thesis-uestc.bst         # bibliography style
│   ├── reference.bib            # 49 refs (EMAT theory, UAV NDT, PINN, visual inspection)
│   ├── publications.bib         # 1 entry (accomplishments)
│   ├── pic/
│   │   ├── bachelor_font.pdf    # 学士学位字体文件
│   │   ├── logo.pdf             # 校徽
│   │   └── c1/                  # Chapter 1 figures (by chapter)
│   │       ├── wind_turbine_blade.png
│   │       ├── storage_tank.jpg
│   │       ├── pressure_vessel.jpg
│   │       ├── gonzalez2019payload.png
│   │       ├── kocer2019inspection.png
│   │       ├── watson2022dry.png
│   │       ├── marcellini2024development.png
│   │       ├── tu2021magnetic.png
│   │       ├── sun2025emat.png
│   │       ├── feroz2021uav.png
│   │       ├── memari2024windturbine.png
│   │       └── omar2017uavir.png
│   ├── chapters/
│   │   ├── c1.tex               # 绪论 (Nature-style, 12 figures, ~269 lines)
│   │   ├── c2.tex               # 电磁超声换能机理与接触边界建模 (7 subsections, ~171 lines)
│   │   ├── c3.tex               # 物理约束注意力机制（仅视觉）+ 消融实验
│   │   ├── c4.tex               # 多模态融合的物理约束注意力机制 + 门控融合
│   │   └── c5.tex               # 总结与展望 (~28 lines)
│   └── misc/
│       ├── chinese_abstract.tex
│       ├── english_abstract.tex
│       ├── acknowledgement.tex
│       └── appendix.tex          # EMAT protocol spec + symbol table
├── 中期/华羽霄_中期报告表.docx
├── 综述/华羽霄_文献综述.docx
├── PLAN.md                    # 下一阶段行动计划 (205 lines)
└── issues/
    └── 2026-06-01-emat-usb-disconnection-emi.md
```

**Figures** are organized by chapter under `pic/cN/`:
```bash
py figures/fig_industrial_structures.py   # Bridge/wind/pressure vessel schematics (replaced by Unsplash photos)
```
Output: bridge_cable.png, wind_turbine_blade.png, pressure_vessel.png at `figures/` (legacy).

Live thesis figures live at `pic/c1/` through `pic/c7/`. Graphics path in thesis-uestc.cls: `\graphicspath{{./pic/}}`. Reference as `c1/filename.png` in `\includegraphics`.

## Known Issues

- **EMAT USB EMI disconnection (#4)**: EMAT probe excitation EMI disrupts CH346C USB chip, causing read timeouts and device disconnections. Software recovery is implemented but hardware mitigation (ferrite cores, shielded cable, USB isolator) is needed for reliable communication. See `issues/2026-06-01-emat-usb-disconnection-emi.md`.
- `bringup.launch` has RealSense camera included — disable the `realsense2_camera` include when D435 is not connected.
- `ndt/package.xml` is missing several dependencies found in CMakeLists.txt (`tf`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `mavros_msgs`, `nav_msgs`).
- `normal_ros.py` no longer uses OpenCV GUI — click input comes from the RViz target panel. It publishes once per click (not continuous) after collecting `collect_frames` depth frames.
- `physics_constrained_detector.py` requires `~model_path` parameter — node will `logfatal` and shutdown if not provided.
- **Feature extraction must match training**: Inference-time 6D features in `_extract_visual_features()` must use the exact same normalization as `train_contact_detector.py:extract_depth_features()`. The training script uses raw uint16 mm values as base; inference must NOT pre-convert to meters before computing features. See "Physics-Constrained Contact Detector" section for details.
- Experimental data records to `src/record/datasets/` with timestamped directories containing `frame_index.csv`, `depth/*.png`, `rgb.mp4`, `depth.mp4`, `emat_waveform.csv`.
- `emat/launch/` directory is empty — there is no standalone EMAT launch file. EMAT nodes are launched from `bringup.launch`.
- `rosbag_to_dataset.py` only supports CSV directory conversion (rosbag `--bag` mode was removed).

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
