## Summary

Archived recordings from `2025-11-18/0` have corrupted mp4 files (moov atom missing), making video playback impossible.

## Environment

- **Package**: record
- **OS**: Linux 5.10.216-tegra (Jetson)
- **Affected**: `src/record/datasets/archive/2025-11-18/0/rgb.mp4`, `depth.mp4`

## Root Cause

The old recorder (pre-v1.1.0) lacked SIGTERM signal handling. When the process was killed, `cv::VideoWriter` destructor never ran, so the moov atom (MP4 index) was never written. File size is non-zero but the container is unreadable.

## Reproduction

```bash
python3 -c "import cv2; cap = cv2.VideoCapture('src/record/datasets/archive/2025-11-18/0/rgb.mp4'); print(cap.isOpened())"
# Output: False
```

## Impact

**Low** — Only 1 of 14 archived recordings affected. Pose data (record_log.csv) is intact and usable for labeling. The SIGTERM handler fix (v1.1.0) prevents this for all new recordings.

## Resolution

- **Prevention**: Fixed in v1.1.0 via `signal(SIGTERM, shutdownHandler)` in `multimodal_recorder.cpp`
- **Affected data**: Unrecoverable. Pose data remains usable; video lost.
- **Label tool**: Updated to suppress ffmpeg stderr warnings when loading corrupted mp4
