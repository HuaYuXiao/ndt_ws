# emat

EMAT 电磁超声测厚仪 ROS 驱动包。通过 USB 直连 CH346C 芯片，发布超声回波波形数据和厚度测量结果。

## 硬件

| 项目 | 参数 |
|------|------|
| 探头 | EMAT 电磁超声笔式探头 |
| USB 芯片 | WCH CH346C_M0 (VID=1A86, PID=55EB) |
| 通信接口 | USB Interface 2 (Vendor Specific) |
| 端点 | EP 0x06 OUT / EP 0x86 IN (Bulk, 512B) |

## 通信协议

CRC-8 校验，多项式 0x07，初值 0x00。



| 功能码 | 命令 | 说明 |
|--------|------|------|
| 0x00 | 读取厚度 | payload 为空 |
| 0x01 | 读取波形 | payload 为 1 字节块索引 (1-4) |
| 0x03 | 设置采集参数 | payload 5 字节 |
| 0x04 | 获取采集参数 | payload 为空 |

波形数据分 4 个块，每块 8185 采样点，共 32740 点（1 MHz 采样率，8-bit ADC，DC 偏置 127）。

## 话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
|  |  | 每 2 秒（可配置） | 原始波形数据 |
|  |  | 随波形发布 | 厚度测量值 |
|  |  | 2 秒（latched） | 设备连接状态 |

### EmatWaveform



### EmatThickness



### EmatDeviceStatus



## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
|  | 500 | 波形读取间隔 (ms) |
|  | 4 | 波形分块数 |
|  | 3230.0 | 默认声速 (m/s, 钢) |
|  | steel | 默认材质 |

## 使用



## 依赖

- ROS Noetic (roscpp, std_msgs, message_generation)
- libusb-1.0

## 已知限制

- 需要 root 权限访问 USB 设备
-  串口不响应协议命令，必须使用 Interface 2 Bulk 端点
- 不支持热插拔自动恢复（需重启节点）
