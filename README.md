# 基于多模态逐级物理约束的无人机自主电磁超声探测技术研究

> ROS Noetic catkin 工作空间，用于无人机自主 EMAT 无损检测与多模态接触状态判别。

---

## 项目概述

无人机无损检测中，探头与被测表面的接触状态难以精确判定。传统单一超声回波阈值法在飞行扰动下不可靠，需融合 EMAT 回波、位姿误差和 RGB-D 视觉三模态进行联合判别。

本项目实现了一套无人机自主巡检系统，融合电磁超声换能器（EMAT）超声感知、RGB-D 视觉和激光惯性里程计，用于结构健康监测。系统解决了飞行过程中探头与被测表面接触状态精确判定的难题——这是航空平台超声测厚可靠性的关键前提。

核心贡献是一个**多模态融合框架**，通过物理约束 Transformer 架构联合建模 EMAT 回波信号、无人机位姿误差和视觉 ROI 特征，在飞行扰动下实现鲁棒的接触状态检测。

## 主要功能

- **EMAT 超声驱动** — 基于 CH346C USB 的 EMAT 探头驱动，~40 Hz 波形采集，自动重连，CRC-8 校验二进制协议
- **信号处理流水线** — Hilbert 包络提取、带通滤波、14 维特征提取（能量、峰值幅度、到达时间、频谱重心、峰度、相位、8 频段能量分解）
- **多模态时序对齐** — 同步 EMAT（~40 Hz）、无人机位姿（~100 Hz）和视觉 ROI（~30 Hz）数据流，最近邻与线性插值
- **特征编码** — PyTorch 编码器将异构传感器数据映射到统一嵌入空间（超声 1D-CNN、位姿 MLP、视觉 ResNet18）
- **视觉目标选择** — 基于 RViz 的目标选取，SVD 平面拟合估计表面法向量并生成接近位姿
- **视觉跟踪** — OpenCV CSRT 跟踪器 + 3D 卡尔曼滤波，实时目标跟随
- **飞行控制** — MAVROS 集成，三阶段状态机（DUMMY → TARGET → HOLD）+ 比例速度控制
- **多模态数据录制** — 同步录制 EMAT 波形、无人机位姿、RGB-D 视频和飞行指令

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        传感器层                                  │
│  ┌──────────┐  ┌──────────────┐  ┌───────────────────────────┐ │
│  │ EMAT     │  │ RealSense    │  │ Livox MID-360 + IMU       │ │
│  │ 探头     │  │ D435 RGB-D   │  │ （激光惯性）              │ │
│  └────┬─────┘  └──────┬───────┘  └─────────────┬─────────────┘ │
│       │               │                         │               │
│       ▼               ▼                         ▼               │
│  ┌─────────┐   ┌───────────┐            ┌──────────────┐       │
│  │ EMAT    │   │ 视觉跟踪  │            │ FAST-LIO 2.0 │       │
│  │ 特征提取│   │ (CSRT)    │            │ 里程计       │       │
│  └────┬────┘   └─────┬─────┘            └──────┬───────┘       │
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
              │ 多模态时序对齐    │
              └─────────┬─────────┘
                        ▼
              /ndt/multimodal_features
                        │
                        ▼
              ┌───────────────────┐
              │ 多模态融合模型    │
              │ （Transformer）   │
              └─────────┬─────────┘
                        ▼
              /mavros/setpoint_raw/local → PX4
```

### 功能包

| 包名 | 用途 |
|------|------|
| `emat` | EMAT 探头驱动、Qt5 波形可视化、ROS 消息定义 |
| `ndt` | 信号处理、视觉目标选择、飞行控制、特征编码 |
| `bringup` | 系统集成启动文件、激光-MAVROS 桥接 |
| `record` | 多模态数据录制 |
| `fast_lio` | FAST-LIO 2.0 激光惯性里程计 |
| `livox_ros_driver2` | Livox MID-360 激光雷达驱动 |
| `realsense2_camera` | Intel RealSense D435 相机驱动 |

### 核心 ROS 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/emat/waveform` | `EmatWaveform` | 原始超声波形（~40 Hz） |
| `/emat/features` | `EmatFeatures` | 提取的信号特征 |
| `/emat/envelope` | `EmatEnvelope` | Hilbert 包络（低通滤波） |
| `/mavros/local_position/pose` | `PoseStamped` | 无人机里程计 |
| `/relative_pos` | `PointStamped` | 视觉目标位置（base_link 坐标系） |
| `/ndt/multimodal_features` | `MultiModalFeatures` | 对齐后的多模态特征向量 |
| `/mavros/setpoint_raw/local` | `PositionTarget` | 飞行控制指令 |

## 环境要求

- **操作系统**：Ubuntu 20.04（Jetson aarch64 或 x86_64）
- **ROS**：Noetic
- **Python**：3.8+
- **CUDA**：11.4+（Jetson）— PyTorch 推理必需

### 系统依赖

```bash
sudo apt install libusb-1.0-0-dev  # EMAT USB 通信
```

### Python 依赖

```bash
pip install numpy scipy torch torchvision
```

## 安装与快速开始

### 1. 克隆并编译

```bash
cd ~/ndt_ws
catkin_make
source ~/ndt_ws/devel/setup.bash
```

### 2. USB 权限配置（EMAT 探头）

```bash
sudo tee /etc/udev/rules.d/99-ch346-emat.rules << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55eb", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55e0", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

配置完成后重新连接 USB 设备。

### 3. 启动完整系统

```bash
roslaunch bringup bringup.launch
```

启动 MAVROS、激光雷达驱动、FAST-LIO、RealSense 相机、EMAT 驱动、特征提取器和 RViz。

### 4. 单独启动各模块

```bash
# 视觉目标选择 + 表面法向量估计
roslaunch ndt normal.launch

# CSRT 视觉跟踪模式
roslaunch ndt csrt.launch

# 数据录制
roslaunch record record.launch
```

## EMAT 驱动

### 硬件规格

| 项目 | 参数 |
|------|------|
| 探头 | EMAT 电磁超声笔式探头 |
| USB 芯片 | WCH CH346C_M0（VID: `0x1A86`） |
| 正常模式 PID | `0x55EB` |
| Bootrom/ISP 模式 PID | `0x55E0`（异常——需重新插拔 USB） |
| 接口 | USB Interface 2（Vendor Specific） |
| 端点 | EP `0x06` OUT / EP `0x86` IN（Bulk, 512B） |

### 验证连接

```bash
lsusb | grep 1a86
```

预期输出：`1a86:55eb`（正常模式）。若显示 `1a86:55e0`，探头处于 bootrom 模式，需物理断电重启。

### ROS 消息定义

**EmatWaveform** — 原始波形数据
```
time    stamp
uint32  sample_count
uint8[] raw_data                  # 8 位 ADC，直流偏置 127
uint32  speed_of_voice            # 声速 (m/s)
uint8   average_count
float32 excitation_frequency_mhz
float32 thickness_mm
string  device_id
```

**EmatEnvelope** — 低通滤波 Hilbert 包络
```
time    stamp
float32[] envelope
uint32  sample_count
float32 sampling_rate
```

**MultiModalFeatures** — 对齐后的多模态特征向量
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

### 驱动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `read_interval_ms` | `100` | 波形采集间隔（ms） |
| `num_chunks` | `4` | 每次波形的分块数（每块 ~8185 采样点） |
| `default_speed` | `3230.0` | 默认声速（m/s） |
| `chunk_delay_ms` | `10` | 分块间延迟（ms） |
| `read_timeout_ms` | `500` | USB 读取超时（ms） |
| `max_startup_retries` | `30` | 启动时最大重连次数 |
| `max_consecutive_failures` | `5` | 连续失败次数阈值（触发自动重连） |
| `reconnect_delay_s` | `3` | 重连等待时间（s） |

### 通信协议

CRC-8 校验的二进制协议（多项式 `0x07`，初始值 `0x00`）。

数据包格式：`[0xAB] [0x00] [0x01] [func] [len_hi] [len_lo] [payload...] [CRC]`

| 功能码 | 命令 | 载荷 |
|--------|------|------|
| `0x00` | 读取厚度 | 无 |
| `0x01` | 读取波形 | 1 字节分块索引（1-4） |
| `0x03` | 设置参数 | 5 字节 |
| `0x04` | 获取参数 | 无 |

波形数据：4 分块 × ~8185 采样点 = ~32740 点（1 MHz 采样率，8 位 ADC）。

### 自动重连机制

驱动具备两级恢复机制：
1. **连续失败重连** — 连续 `max_consecutive_failures` 次错误后自动重新打开 USB 连接
2. **后台重连线程** — 断开时每 5 秒扫描 USB 总线，区分正常模式（`0x55EB`）和 bootrom 模式（`0x55E0`）

### 参考声速

| 材料 | 声速 (m/s) |
|------|-----------|
| 钢 | 3230 |
| 铸铁 | 2210 |
| 铝 | 3100 |
| 铜 | 2320 |

## 信号处理流水线

`emat_feature_extractor` 节点处理原始波形的流程：

1. **直流偏置去除** — uint8 采样值减去 127
2. **Hilbert 变换** — 计算解析信号 → 包络 + 瞬时相位
3. **包络截取** — 提取 `[slice_start, slice_end)` 区间（默认 0–1000）
4. **低通滤波** — FIR 滤波器（Hamming 窗，可配置截止频率）
5. **特征提取** — 14 维特征向量
6. **厚度估计** — `d = (声速 × 到达时间) / 2`

### 特征提取器参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sampling_rate` | `1000000.0` | ADC 采样率（Hz） |
| `slice_start` | `0` | 包络截取起始索引 |
| `slice_end` | `1000` | 包络截取结束索引 |
| `lp_cutoff` | `10.0` | 低通滤波截止频率（Hz） |
| `lp_order` | `256` | FIR 滤波器阶数（taps = order + 1） |
| `arrival_threshold` | `0.1` | 到达检测阈值（峰值比例） |
| `speed_of_sound` | `3240.0` | 默认声速（m/s） |

## 多模态时序对齐

`temporal_aligner` 节点同步三个传感器数据流：

- **EMAT 特征**（~40 Hz）— 主时钟
- **无人机位姿**（~30 Hz）— 最近邻插值
- **视觉 ROI**（~30 Hz）— 线性插值

速度通过可配置窗口的最小二乘线性拟合估计。

### 对齐参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pose_buffer_duration` | `0.5` | 位姿缓冲窗口（s） |
| `visual_buffer_duration` | `0.5` | 视觉缓冲窗口（s） |
| `velocity_window` | `0.2` | 速度估计窗口（s） |
| `max_pose_age` | `0.1` | 位姿最大可接受延迟（s） |
| `max_visual_age` | `0.15` | 视觉最大可接受延迟（s） |

## 特征编码器

纯 PyTorch 模块（`ndt.lib.feature_encoders`），无 ROS 依赖：

| 编码器 | 输入 | 架构 | 输出 |
|--------|------|------|------|
| `UltrasoundEncoder` | (batch, 14) | 1D-CNN + 位置编码 | (batch, d_model) |
| `PoseEncoder` | (batch, 9) | 3 层 MLP | (batch, d_model) |
| `VisualEncoder` | (batch, 3, 224, 224) | ResNet18 + 投影头 | (batch, d_model) |
| `MultiModalEncoder` | 全部三者 | 堆叠输出 | (batch, 3, d_model) |

## 数据录制

`multimodal_recorder` 节点同步录制：
- EMAT 波形和特征
- 无人机位姿和飞行指令
- RGB 和深度视频
- 带时间戳的 CSV 日志

输出目录：`src/record/datasets/YYYYMMDD/N/`，包含 `dataset.npz`（关闭时自动转换）。

通过 `record/RvizRecordPanel` 插件在 RViz 中一键录制。

## 接触标注工具

PyQt5 GUI，用于标注录制数据集的接触/非接触标签。同步显示 RGB/深度视频、EMAT 特征、误差轨迹和交互式时间线。

```bash
python3 src/record/scripts/label_tool.py
```

快捷键：`←`/`→` 帧导航，`Space` 播放/暂停，`C` 标记接触起点，`V` 标记接触终点，`U` 撤销，`S` 保存。时间线支持点击跳转和拖拽标注。

## 物理约束接触检测器

核心模型是一个物理约束 Transformer，融合视觉特征与运动学数据进行接触状态分类。

**模型架构**（`src/ndt/scripts/physics_attention.py`）：
- 输入投影：6D 视觉特征 → 128 维嵌入
- 物理约束矩阵 P：时间邻近性 + 空间邻近性 + 法向量一致性
- Transformer 编码器：2 层、8 头、物理偏置注意力 `softmax(QKᵀ/√d + λ·P) V`
- 分类头：128 → 64 → 2（接触/非接触）

**训练**（`src/ndt/scripts/train_contact_detector.py`）：
```bash
python3 src/ndt/scripts/train_contact_detector.py --epochs 50 --batch 4
```

基于 `src/record/datasets/` 中已标注数据集训练。从 PNG 序列提取 6D 深度特征，构建滑动窗口（64 帧），使用交叉熵 + 时序平滑损失优化。

**迁移训练**：
```bash
python3 src/ndt/scripts/train_contact_detector.py --resume src/ndt/runs/1/contact_detector_best.pt --epochs 50 --lr 5e-4
```

加载已有模型权重继续训练，结果自动保存到 `src/ndt/runs/N/` 目录（N 依次递增）。

### 评价指标

- 接触判别准确率（Precision / Recall / F1）
- 临界状态误报率
- 接触概率时序平滑度
- 无人机闭环扫查稳定性
- 单帧推理延迟目标 < 50ms
- 系统刷新率目标 > 30 Hz
