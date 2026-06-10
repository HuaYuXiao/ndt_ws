#!/usr/bin/env python3
"""Convert multimodal recorded data to training-ready .npz dataset.

Usage:
    python3 rosbag_to_dataset.py /path/to/run_dir
    python3 rosbag_to_dataset.py /path/to/run_dir --skip-depth
    python3 rosbag_to_dataset.py /path/to/run_dir -o /path/to/output

Produces (per run):
    dataset.npz            # All training arrays in one file
    emat_waveform.npy      # Raw EMAT waveforms (variable-length, separate file)
    metadata.json          # Dataset metadata

dataset.npz keys:
    timestamps       (N,)      float64   Frame timestamps
    pose             (N, 6)    float64   x,y,z,roll,pitch,yaw
    target_pose      (N, 3)    float64   Target position in map frame (NaN if unavailable)
    normals          (N, 3)    float32   Surface normal vectors (NaN if unavailable)
    emat_features    (N, 14)   float32   EMAT signal features (NaN if unavailable)
    contact_prob     (N,)      float32   Contact probability labels (NaN if unlabeled)

Optional (with --include-depth):
    depth_frames     (N, H, W) uint16    Raw 16-bit depth images
"""

import argparse
import csv
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np


def _float_or_nan(s):
    """Parse a string as float, returning np.nan for 'nan' or empty."""
    if not s or s == "nan":
        return np.nan
    return float(s)


def _convert_frame_index(csv_path, run, out, include_depth=False):
    """Convert frame_index.csv to compressed .npz dataset."""
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

    # Allocate arrays
    timestamps = np.zeros(N, dtype=np.float64)
    pose = np.zeros((N, 6), dtype=np.float64)
    target_pose = np.full((N, 3), np.nan, dtype=np.float64)
    normals = np.full((N, 3), np.nan, dtype=np.float32)
    emat_features = np.full((N, 14), np.nan, dtype=np.float32)
    contact_prob = np.full(N, np.nan, dtype=np.float32)

    emat_keys = [
        "emat_energy", "emat_peak_amplitude", "emat_arrival_time",
        "emat_spectral_centroid", "emat_kurtosis", "emat_phase",
        "emat_band0", "emat_band1", "emat_band2", "emat_band3",
        "emat_band4", "emat_band5", "emat_band6", "emat_band7",
    ]

    for i, row in enumerate(rows):
        timestamps[i] = float(row["stamp"])
        pose[i] = [
            float(row["pose_x"]), float(row["pose_y"]),
            float(row["pose_z"]), float(row["pose_roll"]),
            float(row["pose_pitch"]), float(row["pose_yaw"]),
        ]

        # Target pose
        target_pose[i] = [
            _float_or_nan(row.get("target_x", "nan")),
            _float_or_nan(row.get("target_y", "nan")),
            _float_or_nan(row.get("target_z", "nan")),
        ]

        # Normals
        normals[i] = [
            _float_or_nan(row.get("normal_x", "nan")),
            _float_or_nan(row.get("normal_y", "nan")),
            _float_or_nan(row.get("normal_z", "nan")),
        ]

        # EMAT features (14-dim, drop thickness_estimate which is redundant)
        for j, key in enumerate(emat_keys):
            emat_features[i, j] = _float_or_nan(row.get(key, "nan"))

        # Contact probability
        contact_prob[i] = _float_or_nan(row.get("contact_prob", "nan"))

    # Build save dict
    save_dict = {
        "timestamps": timestamps,
        "pose": pose,
        "target_pose": target_pose,
        "normals": normals,
        "emat_features": emat_features,
        "contact_prob": contact_prob,
    }

    # Optional: include raw 16-bit depth frames
    if include_depth:
        import cv2
        depth_dir = run / "depth"
        if depth_dir.exists():
            depth_files = sorted(depth_dir.glob("*.png"))
            if depth_files:
                d0 = cv2.imread(str(depth_files[0]), cv2.IMREAD_UNCHANGED)
                H, W = d0.shape[:2]
                depth_frames = np.zeros(
                    (len(depth_files), H, W), dtype=np.uint16
                )
                for i, df in enumerate(depth_files):
                    depth_frames[i] = cv2.imread(
                        str(df), cv2.IMREAD_UNCHANGED
                    )
                save_dict["depth_frames"] = depth_frames
                print(f"  depth: {len(depth_files)} frames, {H}x{W}, 16-bit")
            else:
                print("  depth: no PNG files found")
        else:
            print(f"  depth: {depth_dir} not found, skipping")

    # Save compressed .npz
    np.savez_compressed(out / "dataset.npz", **save_dict)

    # Stats
    n_normals = int(np.sum(~np.isnan(normals[:, 0])))
    n_emat = int(np.sum(~np.isnan(emat_features[:, 0])))
    n_labels = int(np.sum(~np.isnan(contact_prob)))
    print(f"  pose: {N} samples")
    print(f"  normals: {N} samples ({n_normals} valid)")
    print(f"  emat_features: {N} samples ({n_emat} valid)")
    print(f"  contact_prob: {N} samples ({n_labels} labeled)")

    return N, n_normals, n_emat, n_labels


def _convert_legacy_csv(csv_path, run, out):
    """Convert legacy record_log.csv to .npz (backward compat)."""
    timestamps = []
    pose_data = []

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamps.append(float(row["t"]))
            pose_data.append([
                float(row["odom_x"]), float(row["odom_y"]),
                float(row["odom_z"]), float(row["odom_roll"]),
                float(row["odom_pitch"]), float(row["odom_yaw"]),
            ])

    N = len(pose_data)
    np.savez_compressed(
        out / "dataset.npz",
        timestamps=np.array(timestamps, dtype=np.float64),
        pose=np.array(pose_data, dtype=np.float64),
    )
    print(f"  pose (legacy): {N} samples")
    print("  NOTE: old format — no EMAT features, normals, or contact labels.")
    print("  Re-record with updated recorder for full dataset.")
    return N, 0, 0, 0


def _convert_emat_waveform(csv_path, out):
    """Convert emat_waveform.csv to .npy (variable-length, separate file)."""
    timestamps = []
    frames = []
    max_samples = 0

    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        next(reader, None)  # skip header
        for row in reader:
            if len(row) < 3:
                continue
            stamp = float(row[0])
            raw_bytes = bytes(int(h, 16) for h in row[2:])
            timestamps.append(stamp)
            frames.append(np.frombuffer(raw_bytes, dtype=np.uint8))
            if len(raw_bytes) > max_samples:
                max_samples = len(raw_bytes)

    if not frames:
        return 0

    waveform = np.zeros((len(frames), max_samples), dtype=np.uint8)
    for i, frame in enumerate(frames):
        waveform[i, :len(frame)] = frame

    np.save(out / "emat_waveform.npy", waveform)
    np.save(out / "emat_timestamps.npy",
            np.array(timestamps, dtype=np.float64))
    print(f"  emat waveform: {len(frames)} frames, {max_samples} samples/frame")
    return len(frames)


def convert_from_csv(run_dir, output_dir, include_depth=False):
    """Convert live-recording CSV data to .npz dataset."""
    run = Path(run_dir)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    index_csv = run / "frame_index.csv"
    legacy_csv = run / "record_log.csv"

    if index_csv.exists():
        stats = _convert_frame_index(index_csv, run, out, include_depth)
    elif legacy_csv.exists():
        stats = _convert_legacy_csv(legacy_csv, run, out)
    else:
        print(f"ERROR: neither frame_index.csv nor record_log.csv found in {run}")
        sys.exit(1)

    # EMAT waveform (separate file, variable-length)
    emat_csv = run / "emat_waveform.csv"
    n_emat_wave = 0
    if emat_csv.exists():
        n_emat_wave = _convert_emat_waveform(emat_csv, out)
    else:
        print(f"  emat waveform: {emat_csv} not found, skipping")

    # Metadata
    N, n_normals, n_emat, n_labels = stats
    metadata = {
        "source_dir": str(run),
        "created_at": datetime.now().isoformat(),
        "format_version": "3.0",
        "num_frames": N,
        "has_normals": n_normals > 0,
        "has_emat_features": n_emat > 0,
        "has_emat_waveform": n_emat_wave > 0,
        "has_contact_labels": n_labels > 0,
        "includes_depth": include_depth,
    }
    with open(out / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"  metadata.json written")

    # Print output summary
    print(f"\nOutput files:")
    print(f"  {out / 'dataset.npz'}  (training data)")
    if n_emat_wave > 0:
        print(f"  {out / 'emat_waveform.npy'}  (raw waveforms)")
    print(f"  {out / 'metadata.json'}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert multimodal recording to .npz training dataset"
    )
    parser.add_argument(
        "input",
        help="Path to recording directory (e.g. datasets/run_20260604/0)",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output directory (default: same as input)",
    )
    parser.add_argument(
        "--include-depth",
        action="store_true",
        default=False,
        help="Include raw 16-bit depth frames in dataset.npz (large file). "
             "By default, depth is excluded — use depth/ PNGs for visualization.",
    )
    args = parser.parse_args()

    out = args.output or args.input
    print(f"Converting: {args.input}")
    print(f"Output:     {out}")
    convert_from_csv(args.input, out, args.include_depth)
    print("Done.")


if __name__ == "__main__":
    main()
