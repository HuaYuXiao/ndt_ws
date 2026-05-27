# 基于多模态逐级物理约束的无人机自主电磁超声探测技术研究

EMAT（电磁超声）测厚仪 ROS 驱动包。通过 USB 直连 CH346C 芯片，以约 40 Hz 频率采集并发布超声回波波形数据，支持设备断线自动重连。

## 硬件

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
- **Python 可视化节点（可选）**: `rospy`, `numpy`, `matplotlib` (Qt5Agg 后端)

```bash
sudo apt install libusb-1.0-0-dev
```

## 编译

```bash
cd ~/ndt_ws && catkin_make --pkg emat
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

### 启动波形可视化

```bash
roslaunch emat emat_viz.launch
```

需要 Qt5 和 matplotlib，在本地显示器上实时显示波形。

### 查看话题数据

```bash
# 查看设备连接状态
rostopic echo /emat/device_status

# 查看波形发布频率（正常约 40 Hz）
rostopic hz /emat/waveform

# 查看波形数据
rostopic echo /emat/waveform
```

### 验证设备连接

```bash
lsusb | grep 1a86
```

应看到 `1a86:55eb`（正常模式）。如果显示 `1a86:55e0` 则为 bootrom 异常模式，需要物理断电后重新插入 USB 线。

## 话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `emat/waveform` | `EmatWaveform` | ~40 Hz | 原始波形数据（每帧为一个数据块） |
| `emat/thickness` | `EmatThickness` | -- | 厚度测量值（当前未使用） |
| `emat/device_status` | `EmatDeviceStatus` | 0.5 Hz (latched) | 设备连接状态 |

### 消息定义

**EmatWaveform**
```
time    stamp
uint32  sample_count              # 本帧采样点数
uint8[] raw_data                  # 原始 ADC 数据（8-bit, DC 偏置 127）
string  material                  # 材质名称
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

**EmatThickness**（当前未使用）
```
time    stamp
float32 thickness_mm
string  material
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
| `default_material` | `steel` | 默认材质名称 |
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

可参考 `config/emat_params.yaml` 中的材质声速表。

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
