# AGENTS.md

This file describes the agents and automation workflows in this ROS workspace.

## Available Agents

### Record Agent (RViz Panel)
- **Trigger**: Manual via RViz `record/RvizRecordPanel` toggle button
- **Action**: Spawns `multimodal_recorder` process via `QProcess`
- **Output**: `src/record/datasets/YYYYMMDD/N/` with `dataset.npz`, depth PNGs, RGB/depth video, EMAT waveforms
- **Shutdown**: Auto-converts CSV to `dataset.npz` via `rosbag_to_dataset.py`

### EMAT Feature Extractor
- **Trigger**: ROS node launched via `bringup.launch` or standalone `rosrun`
- **Input**: `/emat/waveform` (~40 Hz)
- **Output**: `/emat/features` (14D), `/emat/envelope` (filtered Hilbert envelope)
- **Pipeline**: slice → DC removal → Hilbert → low-pass FIR → feature extraction → thickness estimate

### Labeling Tool
- **Trigger**: Desktop shortcut or `python3 src/record/scripts/label_tool.py`
- **Input**: `dataset.npz` from a recording directory
- **Action**: Interactive GUI for annotating contact/no-contact labels
- **Output**: Updates `contact_prob` in `dataset.npz`, `frame_index.csv`, and `metadata.json`

### Training Pipeline
- **Trigger**: `python3 src/ndt/scripts/train_contact_detector.py`
- **Input**: All labeled `dataset.npz` from `src/record/datasets/`
- **Action**: Trains PhysicsConstrainedContactDetector with sliding window approach
- **Output**: `src/ndt/models/contact_detector_best.pt` + `training_log.csv`

## Data Flow

```
Recording → dataset.npz → Labeling → dataset.npz (with contact_prob) → Training → model.pt
```

1. **Record**: RViz RecordPanel starts `multimodal_recorder`, saves multi-modal data
2. **Convert**: Recorder destructor auto-calls `rosbag_to_dataset.py` → `dataset.npz`
3. **Label**: `label_tool.py` annotates contact regions → updates `contact_prob` array
4. **Train**: `train_contact_detector.py` loads all labeled data → trains model
5. **Deploy**: `physics_constrained_detector.py` loads model → publishes `/ndt/contact_probability`
