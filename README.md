# 基于多模态逐级物理约束的无人机自主电磁超声探测技术研究

EMAT（电磁超声）测厚仪 ROS 驱动包。通过 USB 直连 CH346C 芯片，以约 40 Hz 频率采集并发布超声回波波形数据，支持设备断线自动重连。

## 系统总体设计

### 研究背景与关键技术问题

**核心痛点**: 在无人机搭载电磁超声探头进行结构无损检测时，探头与被测表面的接触状态难以精确判定。传统方法依赖单一超声回波信号的阈值判断，在无人机飞行扰动与姿态变化条件下，信号易受接触不稳定、电磁干扰及结构耦合变化影响，导致误判或漏判。

**三大关键问题**:
1. **单模态局限性**: 传统EMAT检测基于单一回波信号，无法应对动态飞行平台的复杂工况
2. **物理约束缺失**: 纯数据驱动模型忽略物理规律，泛化能力不足
3. **状态时序抖动**: 逐帧分类导致频繁状态跳变，影响控制闭环稳定性

### 总体系统组成

#### 无人机平台层
- **飞行平台**: 基于PX4飞控的多旋翼无人机
- **位姿估计**: FAST-LIO 2.0 LiDAR-inertial odometry → MAVROS vision pose
- **飞控接口**: mavros_msgs/PositionTarget setpoint，DUMMY→TARGET→HOLD状态机

#### 多模态感知层
- **电磁超声模块**: EMAT探头，发射0xAB协议命令，接收回波信号（8-bit ADC，~8185 samples/chunk）
- **视觉模块**: Intel RealSense D435 RGBD相机，提供RGB图像+深度信息
- **位姿模块**: FAST-LIO odometry提供6-DOF位姿+速度估计

#### 机载处理层
- **硬件**: NVIDIA Jetson（aarch64），运行ROS Noetic
- **数据采集**: 多模态数据同步记录（RGB/depth video）
- **模型推理**: Transformer网络的嵌入式部署（考虑CNN/GRU轻量化替代方案）

#### 决策控制层
- **接触状态概率输出**: 多模态融合模型输出接触概率分布
- **状态转移约束**: HMM/CRF平滑接触状态演化
- **控制回路**: 接触概率→PositionTarget setpoint→PX4飞控

### 分阶段应用物理约束的策略

#### 阶段一：粗定位（视觉引导）
- 用户在RGB图像中选取目标区域
- 视觉ROI路径规划→无人机自主接近目标结构

#### 阶段二：接近与接触建立
- 切换至力控模式，无人机减速并建立初始接触
- 动力学一致性约束：法向力突变检测，||Δv_t|| ≈ 0判断接触发生

#### 阶段三：精扫查（稳定接触检测）
- 维持恒定法向接触力（参考Watson的20N目标）
- EMAT回波信号实时采集与质量评估
- 物理约束注意力机制：在Transformer自注意力计算中引入动力学一致性矩阵P

#### 阶段四：状态判定与反馈
- 多模态融合→接触概率输出
- 状态转移约束模型（HMM/CRF）平滑状态演化
- 接触概率作为控制反馈信号，实现闭环调节

### 多模态数据实时配准与融合算法设计

#### 模态编码
- **超声信号**: 1D-CNN + 位置编码 → 超声特征向量 f_u(t)
- **位姿误差**: MLP编码 → 位姿特征向量 f_p(t)
- **视觉ROI**: CNN/ViT提取 → 视觉特征向量 f_v(t)

#### 时序对齐
- 参考MPFusionNet的DTW+三次样条插值方法
- 将不同采样频率的传感器数据重采样至统一时序尺度

#### 跨模态融合
- 跨模态注意力机制：Q_u × K_p^T / √d → 超声-位姿交互
- 最终融合表示：f_fused = [f_up; f_uv]，拼接后通过时序Transformer建模

#### 物理约束嵌入
- **物理约束矩阵P**: 基于无人机动力学模型构造
  - 平移运动：m·a = F_thrust - mg - F_drag
  - 姿态变化：I·α = τ - τ_disturbance
  - 接触物理先验：法向力突变 → ||Δv_t|| ≈ 0
- **损失函数**: L = L_CE + λ·L_physics
  - L_CE: 接触概率的交叉熵损失
  - L_physics: 物理残差损失，强制输出满足动力学合理性

### 实验验证方案

#### 评价指标
- **接触判定准确率**: Precision, Recall, F1-score
- **状态稳定性**: 状态跳变频率（次/分钟），连续正确判定时长
- **实时性**: 单帧推理延迟（目标<50ms），系统刷新率（目标>30Hz）
- **测厚精度**: 与基准方法对比的绝对误差和相对误差
- **鲁棒性**: 不同风速、光照、材料条件下的性能退化程度

#### 消融实验设计
1. 单模态 vs 多模态（验证多模态融合的必要性）
2. 无物理约束 vs 有物理约束（验证物理约束注意力机制的增益）
3. 无状态转移约束 vs 有状态转移约束（验证HMM/CRF的平滑效果）
4. 不同Transformer架构对比（标准Transformer vs PINNsFormer vs 轻量化CNN-GRU）

### 预期创新点

1. **多模态接触感知框架**: 首次将EMAT回波、位姿误差及视觉ROI特征进行统一时序建模，实现接触状态的概率化表达
2. **物理约束注意力机制**: 在自注意力计算中引入动力学一致性约束矩阵P，将无人机运动模型与接触物理先验嵌入特征交互过程
3. **多模态时序跨域对齐与不确定性建模**: 跨模态注意力实现深层耦合+模态权重自适应机制，在环境扰动下实现动态模态可信度分配

---

## EMAT 驱动包

以下为 EMAT 电磁超声测厚仪 ROS 驱动包的技术细节。

### 硬件

| 项目 | 参数 |
|------|------|
| 探头 | EMAT 电磁超声笔式探头 |
| USB 芯片 | WCH CH346C_M0 (VID=`0x1A86`) |
| 正常模式 PID | `0x55EB` |
| Bootrom/ISP 模式 PID | `0x55E0`（异常状态，需重新插拔 USB） |
| 通信接口 | USB Interface 2 (Vendor Specific) |
| 端点 | EP `0x06` OUT / EP `0x86` IN (Bulk, 512B) |

## 依赖

- **ROS Noetic**: `roscpp`, `std_msgs`, `message_generation`, `message_runtime`
- **libusb-1.0**: USB 设备通信（通过 `pkg-config` 自动发现）

```bash
sudo apt install libusb-1.0-0-dev
```

## 编译

```bash
cd ~/ndt_ws && catkin_make
source ~/ndt_ws/devel/setup.bash
```

## USB 权限设置

节点需要 USB 设备访问权限。推荐使用 udev 规则免去 `sudo`：

```bash
sudo tee /etc/udev/rules.d/99-ch346-emat.rules << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55eb", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55e0", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

设置完成后重新插入 USB 设备即可生效。

## 使用

### 启动驱动节点

```bash
roslaunch emat emat.launch
```

节点启动后会自动搜索设备（最多重试 30 次，每次间隔 2 秒）。连接成功后开始持续采集波形数据。

### 验证设备连接

```bash
lsusb | grep 1a86
```

应看到 `1a86:55eb`（正常模式）。如果显示 `1a86:55e0` 则为 bootrom 异常模式，需要物理断电后重新插入 USB 线。

## 话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `emat/waveform` | `EmatWaveform` | ~40 Hz | 原始波形数据（每帧为一个数据块） |
| `emat/thickness` | `EmatThickness` | -- | 厚度测量值 |
| `emat/device_status` | `EmatDeviceStatus` | 0.5 Hz (latched) | 设备连接状态 |

### 消息定义

**EmatWaveform**
```
time    stamp
uint32  sample_count              # 本帧采样点数
uint8[] raw_data                  # 原始 ADC 数据（8-bit, DC 偏置 127）
uint32  speed_of_voice            # 声速 (m/s)
uint8   average_count             # 平均次数 / 分块数
float32 excitation_frequency_mhz  # 激励频率 (MHz)
float32 thickness_mm              # 厚度 (mm, 当前未计算)
string  device_id                 # 设备 ID
```

**EmatDeviceStatus**
```
time    stamp
bool    is_connected              # 是否已连接
string  device_id
string  product_string
string  manufacturer_string
string  serial_number
string  status_message            # 状态描述
```

**EmatThickness**
```
time    stamp
float32 thickness_mm
uint32  speed_of_voice
float32 confidence
```

## 参数

通过 launch 文件或 `rosparam` 设置（节点私有命名空间 `~`）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `read_interval_ms` | `100` | 波形采集间隔 (ms)，决定发布频率 |
| `num_chunks` | `4` | 波形分块数，每块 ~8185 采样点 |
| `default_speed` | `3230.0` | 默认声速 (m/s) |
| `chunk_delay_ms` | `10` | 分块读取间延迟 (ms) |
| `read_timeout_ms` | `500` | USB 读取超时 (ms) |
| `max_startup_retries` | `30` | 启动时最大重试次数（间隔 2s） |
| `max_consecutive_failures` | `5` | 连续失败 N 次后触发自动重连 |
| `reconnect_delay_s` | `3` | 重连等待时间 (s) |

## 参考声速

| 材质 | 声速 (m/s) |
|------|-----------|
| 钢 (steel) | 3230 |
| 铸铁 (cast_iron) | 2210 |
| 铝 (aluminum) | 3100 |
| 铜 (copper) | 2320 |

## 通信协议

二进制协议，CRC-8 校验（多项式 `0x07`，初值 `0x00`）。

数据包格式：`[0xAB] [0x00] [0x01] [功能码] [长度高字节] [长度低字节] [payload...] [CRC]`

| 功能码 | 命令 | payload |
|--------|------|---------|
| `0x00` | 读取厚度 | 无 |
| `0x01` | 读取波形 | 1 字节块索引 (1-4) |
| `0x03` | 设置采集参数 | 5 字节 |
| `0x04` | 获取采集参数 | 无 |

波形数据分 4 块传输，每块 ~8185 采样点，合计 ~32740 点（1 MHz 采样率，8-bit ADC，DC 偏置 127）。

## 自动重连机制

节点具备两级自动恢复能力：

1. **连续失败重连**：采集过程中连续写入失败达到 `max_consecutive_failures` 次后，自动关闭并重新打开设备连接。
2. **后台重连线程**：当设备未连接时（包括启动重试耗尽后），每 5 秒扫描一次 USB 总线，检测到设备后自动连接。

同时会区分正常模式（PID `0x55EB`）和 bootrom 异常模式（PID `0x55E0`），后者需要物理重新插拔 USB 线。

## 已知限制

- `emat/thickness` 话题已定义但当前未填充数据，仅发布波形数据
- 串口不响应协议命令，必须使用 Interface 2 的 Bulk 端点通信
- Bootrom 模式（PID `0x55E0`）需物理断电后重新插拔，无法通过软件恢复
