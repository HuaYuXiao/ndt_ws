#!/usr/bin/env python3
"""Convert multimodal recorded data to numpy dataset format.

Usage:
    python3 rosbag_to_dataset.py /path/to/run_dir

Produces:
    pose_odom.npy          (N, 6) float64
    pose_timestamps.npy    (N,) float64
    depth_frames.npy       (N, H, W) uint16  (from depth/*.png)
    depth_timestamps.npy   (N,) float64
    emat_waveform.npy      (N_emat, max_samples) uint8
    emat_timestamps.npy    (N_emat,) float64
    emat_features.npy      (N, 14) float32   (NaN if no EMAT)
    normals.npy            (N, 3) float32    (NaN if no normal)
    contact_labels.npy     (N,) float32      (NaN if unlabeled)
    rgb.mp4, depth.mp4     (copied)
    video_timestamps.npy   (N_video,) float64
    metadata.json
"""

import argparse
import csv
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

import numpy as np


def convert_from_csv(run_dir: str, output_dir: str):
    """Convert live-recording CSV data to numpy dataset."""
    run = Path(run_dir)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # --- Parse frame_index.csv (新版格式) ---
    index_csv = run / "frame_index.csv"
    legacy_csv = run / "record_log.csv"

    if index_csv.exists():
        _convert_frame_index(index_csv, run, out)
    elif legacy_csv.exists():
        _convert_legacy_csv(legacy_csv, run, out)
    else:
        print(f"ERROR: neither frame_index.csv nor record_log.csv found in {run}")
        sys.exit(1)

    # --- Parse emat_waveform.csv（原始波形，独立于 frame_index） ---
    emat_csv = run / "emat_waveform.csv"
    emat_timestamps = []
    emat_frames = []
    max_samples = 0

    if emat_csv.exists():
        with open(emat_csv, "r") as f:
            reader = csv.reader(f)
            next(reader, None)  # skip header
            for row in reader:
                if len(row) < 3:
                    continue
                stamp = float(row[0])
                raw_bytes = bytes(int(h, 16) for h in row[2:])
                emat_timestamps.append(stamp)
                emat_frames.append(np.frombuffer(raw_bytes, dtype=np.uint8))
                if len(raw_bytes) > max_samples:
                    max_samples = len(raw_bytes)

        if emat_frames:
            emat_waveform = np.zeros(
                (len(emat_frames), max_samples), dtype=np.uint8
            )
            for i, frame in enumerate(emat_frames):
                emat_waveform[i, :len(frame)] = frame
            np.save(out / "emat_waveform.npy", emat_waveform)
            np.save(out / "emat_timestamps.npy",
                    np.array(emat_timestamps, dtype=np.float64))
            print(f"  emat waveform: {len(emat_frames)} frames, "
                  f"{max_samples} samples/frame")
    else:
        print(f"  emat waveform: {emat_csv} not found, skipping")

    # --- Copy videos ---
    for name in ("rgb.mp4", "depth.mp4"):
        src = run / name
        if src.exists():
            shutil.copy2(src, out / name)
            print(f"  copied {name}")

    # --- Video timestamps (approximate) ---
    rgb_path = out / "rgb.mp4"
    if rgb_path.exists():
        try:
            import cv2
            cap = cv2.VideoCapture(str(rgb_path))
            frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            cap.release()
            if frame_count > 0:
                fps = 30.0
                np.save(out / "video_timestamps.npy",
                        np.arange(frame_count) / fps)
                print(f"  video_timestamps: {frame_count} frames @ {fps} fps")
        except ImportError:
            pass


def _float_or_nan(s):
    """Parse a string as float, returning np.nan for 'nan' or empty."""
    if not s or s == "nan":
        return np.nan
    return float(s)


def _convert_frame_index(csv_path, run, out):
    """Convert frame_index.csv (new format) to numpy arrays."""
    import cv2

    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        print("  frame_index.csv is empty")
        return

    N = len(rows)
    print(f"  frame_index.csv: {N} frames")

    # Pose
    stamps = np.zeros(N, dtype=np.float64)
    pose = np.zeros((N, 6), dtype=np.float64)
    normals = np.full((N, 3), np.nan, dtype=np.float32)
    emat_features = np.full((N, 14), np.nan, dtype=np.float32)
    contact_labels = np.full(N, np.nan, dtype=np.float32)

    for i, row in enumerate(rows):
        stamps[i] = float(row["stamp"])
        pose[i] = [float(row["pose_x"]), float(row["pose_y"]),
                    float(row["pose_z"]), float(row["pose_roll"]),
                    float(row["pose_pitch"]), float(row["pose_yaw"])]

        # 法向量
        nx = _float_or_nan(row.get("normal_x", "nan"))
        ny = _float_or_nan(row.get("normal_y", "nan"))
        nz = _float_or_nan(row.get("normal_z", "nan"))
        normals[i] = [nx, ny, nz]

        # EMAT features (14-dim)
        emat_keys = ["emat_energy", "emat_peak_amplitude", "emat_arrival_time",
                     "emat_spectral_centroid", "emat_kurtosis", "emat_phase",
                     "emat_band0", "emat_band1", "emat_band2", "emat_band3",
                     "emat_band4", "emat_band5", "emat_band6", "emat_band7",
                     "emat_thickness"]
        # 15 fields → 取前 14 个 (band_energies 8 个 + 其他 6 个)
        for j, key in enumerate(emat_keys[:14]):
            emat_features[i, j] = _float_or_nan(row.get(key, "nan"))

        # 接触概率
        contact_labels[i] = _float_or_nan(row.get("contact_prob", "nan"))

    np.save(out / "pose_timestamps.npy", stamps)
    np.save(out / "pose_odom.npy", pose)
    np.save(out / "normals.npy", normals)
    np.save(out / "emat_features.npy", emat_features)
    np.save(out / "contact_labels.npy", contact_labels)
    print(f"  pose: {N} samples")
    print(f"  normals: {N} samples "
          f"({np.sum(~np.isnan(normals[:, 0]))} valid)")
    print(f"  emat_features: {N} samples "
          f"({np.sum(~np.isnan(emat_features[:, 0]))} valid)")
    print(f"  contact_labels: {N} samples "
          f"({np.sum(~np.isnan(contact_labels))} labeled)")

    # 16-bit depth PNGs
    depth_dir = run / "depth"
    if depth_dir.exists():
        depth_files = sorted(depth_dir.glob("*.png"))
        if depth_files:
            # 读取第一帧获取尺寸
            d0 = cv2.imread(str(depth_files[0]), cv2.IMREAD_UNCHANGED)
            H, W = d0.shape[:2]
            depth_frames = np.zeros((len(depth_files), H, W), dtype=np.uint16)
            for i, df in enumerate(depth_files):
                depth_frames[i] = cv2.imread(str(df), cv2.IMREAD_UNCHANGED)
            np.save(out / "depth_frames.npy", depth_frames)
            np.save(out / "depth_timestamps.npy", stamps[:len(depth_files)])
            print(f"  depth: {len(depth_files)} frames, {H}x{W}, 16-bit")
        else:
            print("  depth: no PNG files found")
    else:
        print(f"  depth: {depth_dir} not found, skipping")

    # Metadata
    has_emat = bool(np.sum(~np.isnan(emat_features[:, 0])))
    has_normals = bool(np.sum(~np.isnan(normals[:, 0])))
    has_labels = bool(np.sum(~np.isnan(contact_labels)))
    metadata = {
        "source_dir": str(run),
        "created_at": datetime.now().isoformat(),
        "format_version": "2.0",
        "num_frames": N,
        "pose_samples": N,
        "depth_format": "16bit_png",
        "has_emat_features": has_emat,
        "has_normals": has_normals,
        "has_contact_labels": has_labels,
    }
    with open(out / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"  metadata.json written")


def _convert_legacy_csv(csv_path, run, out):
    """Convert legacy record_log.csv to numpy arrays (backward compat)."""
    pose_timestamps = []
    pose_data = []

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pose_timestamps.append(float(row["t"]))
            pose_data.append([
                float(row["odom_x"]), float(row["odom_y"]),
                float(row["odom_z"]), float(row["odom_roll"]),
                float(row["odom_pitch"]), float(row["odom_yaw"]),
            ])

    np.save(out / "pose_timestamps.npy",
            np.array(pose_timestamps, dtype=np.float64))
    np.save(out / "pose_odom.npy", np.array(pose_data, dtype=np.float64))
    print(f"  pose (legacy): {len(pose_data)} samples")
    print("  NOTE: run uses old format — no EMAT features, normals, "
          "or contact labels available. Re-record with updated recorder.")


def main():
    parser = argparse.ArgumentParser(
        description="Convert multimodal recording to numpy dataset"
    )
    parser.add_argument(
        "input",
        help="Path to recording directory",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output directory (default: same as input)",
    )
    args = parser.parse_args()

    out = args.output or args.input
    print(f"Converting recording: {args.input}")
    print(f"  output: {out}")
    convert_from_csv(args.input, out)
    print("Done.")


if __name__ == "__main__":
    main()
