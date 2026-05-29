# Changelog

All notable changes to this project will be documented in this file.

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
