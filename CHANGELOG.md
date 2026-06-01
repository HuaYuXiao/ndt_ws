# Changelog

All notable changes to this project will be documented in this file.

## [v1.2.0] - 2026-06-01

### Bug Fixes
- **fix(emat):** 完善 USB 断连恢复机制，新增五级容错恢复（传输重试→端点clear_halt→chunk重试→close+reopen→后台重连线程）
- **fix(emat):** 首次 USB 通信失败时立即标记设备断开，避免等待5次累积失败
- **fix(emat):** 处理所有 libusb 错误类型（`LIBUSB_ERROR_TIMEOUT`/`IO`/`PIPE`），不仅限于 `NO_DEVICE`
- **fix(emat):** `usb_read` 遇到瞬时超时不立即放弃，最多连续重试5次
- **fix(emat):** PIPE 错误先执行 `libusb_clear_halt` 清除端点 stall，再重试一次
- **fix(emat):** 移除 `libusb_reset_device` 调用（USB 端口复位导致设备重新枚举，对 CH346C 恢复反而有害）

### Features
- **feat(emat):** 特征提取器实现正确的信号处理流水线（截取→DC去除→Hilbert变换→低通滤波→峰值检测→厚度估计）
- **feat(emat):** 双峰法厚度估计 (`extractDelay.m` 算法) — `d = |x1-x2| / fs × v / 2`
- **feat(emat):** 添加 raw waveform slice 支持，跳过始波和电磁串扰盲区 (`slice_start=200`)
- **feat(emat):** 新增 `thickness_estimate` 字段至 `EmatFeatures` 消息

### Refactors
- **refactor(emat):** 将 `emat_feature_extractor.py` 和 `extractDelay.m` 从 ndt 迁移至 emat 包
- **refactor(emat):** 切片区间从 `[0, 5000)` 改为 `[200, 1000)`，匹配论文盲区处理
- **refactor(emat):** 低通滤波器从 Butterworth 改为 FIR (Hamming窗, 257 taps)，匹配 MATLAB `lowpass` 行为
- **refactor(emat):** 优化驱动参数默认值（`chunk_delay_ms`: 10→30ms, `write_timeout_ms`: 1000→200ms）
- **refactor(emat):** 从 RViz 面板移除厚度显示和 `/emat/features` 订阅
- **refactor(emat):** 删除 `emat.launch`（EMAT 节点现已集成至 `bringup.launch`）

### Documentation
- **docs:** 更新 CLAUDE.md — 补充 EMAT 驱动参数表、五级恢复机制、硬件 EMI 已知问题、特征提取器处理流水线
- **docs:** 添加 EMAT USB 断连 Issue #4 详细分析文档 (`issues/2026-06-01-emat-usb-disconnection-emi.md`)
- **docs:** 研读孙广宇《基于电磁超声体波的铝板缺陷检测》(HIT, 2025) 并建立 EMAT 测厚技术基准记忆文件

## [v1.1.0] - 2026-05-29

### Features
- **feat(emat):** add `EmatFeatures` and `EmatEnvelope` message types for signal processing pipeline
- **feat(emat):** add envelope visualization mode with toggle button in RViz waveform panel
- **feat(emat):** display thickness estimate in waveform widget info overlay
- **feat(ndt):** add `emat_feature_extractor.py` — EMAT signal processing pipeline (Hilbert envelope, lowpass filter, feature extraction, thickness estimation)
- **feat(ndt):** add `extractDelay.m` MATLAB reference implementation for delay extraction
- **feat(bringup):** integrate `emat_feature_extractor` node into bringup.launch

### Bug Fixes
- **fix(record):** remove altitude gate that prevented recording when z <= 0
- **fix(record):** add `tryRecord()` call in `ematCallback` so EMAT data triggers recording

### Refactors
- **refactor(emat):** add raw/envelope display mode toggle in `WaveformWidget`
- **refactor(emat):** subscribe to `/emat/envelope` and `/emat/features` in RViz panel
- **refactor(bringup):** update RViz launch to maximize window with xdotool

## [v1.0.0] - 2026-05-28

### Features
- **feat(emat):** replace Python waveform visualizer with Qt5 C++ `WaveformWidget` with ring buffer, grid overlay, DC-removed waveform rendering, and RMS display
- **feat(emat):** add RViz waveform panel plugin (`rviz_emat_panel`), remove standalone viz node
- **feat(ndt):** add RViz target selection panel plugin (`RvizTargetPanel`) for D435 — displays RGB image, publishes click position as `PointStamped`

### Refactors
- **refactor(bringup):** rename `robot_bringup` to `bringup`, replace `bringup_ndt.launch` with `bringup.launch`
- **refactor(ndt):** remove ArUco estimator and related files
- **refactor(ndt):** replace OpenCV GUI with RViz panel click subscription in `normal_ros.py`
- **refactor(emat):** remove unused code and parameters, update message definitions, enhance waveform visualization
- **refactor(emat):** rename `publish_waveform_chunk` to `publish_waveform`

### Bug Fixes
- **fix(emat):** add signal handler for clean USB shutdown on SIGTERM/SIGINT (reverted and re-applied)

### Documentation
- **docs:** rewrite emat README with comprehensive usage, protocol, and configuration docs
- **docs:** add system design overview with 4-layer architecture and multi-modal fusion algorithm design
- **docs(毕业设计):** expand mid-term report theoretical analysis with 20+ formulas
- **docs(毕业设计):** add thesis context to CLAUDE.md with theoretical framework and evaluation metrics
