#!/usr/bin/env python3
"""Convert multimodal recorded data to numpy dataset format.

Usage:
    # From live recording directory:
    python3 rosbag_to_dataset.py /path/to/run_dir

    # From rosbag file:
    python3 rosbag_to_dataset.py /path/to/recording.bag --bag

Produces:
    emat_waveform.npy      (N_emat, max_samples) uint8
    emat_timestamps.npy    (N_emat,) float64
    pose_odom.npy          (N_pose, 6) float64
    pose_timestamps.npy    (N_pose,) float64
    rgb.mp4, depth.mp4     (copied)
    video_timestamps.npy   (N_video,) float64
    contact_labels.npy     (N_emat,) int8  (-1 = unlabeled)
    metadata.json
"""

import argparse
import csv
import json
import os
import shutil
import struct
import sys
from datetime import datetime
from pathlib import Path

import numpy as np


def convert_from_csv(run_dir: str, output_dir: str):
    """Convert live-recording CSV data to numpy dataset."""
    run = Path(run_dir)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # --- Parse record_log.csv ---
    csv_path = run / "record_log.csv"
    if not csv_path.exists():
        print(f"ERROR: {csv_path} not found")
        sys.exit(1)

    pose_timestamps = []
    pose_data = []       # [x, y, z, roll, pitch, yaw]
    emat_stamps_in_main = []

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row["t"])
            pose_timestamps.append(t)
            pose_data.append([
                float(row["odom_x"]),
                float(row["odom_y"]),
                float(row["odom_z"]),
                float(row["odom_roll"]),
                float(row["odom_pitch"]),
                float(row["odom_yaw"]),
            ])
            emat_stamp = row.get("emat_stamp", "nan")
            if emat_stamp != "nan":
                emat_stamps_in_main.append(float(emat_stamp))

    pose_timestamps = np.array(pose_timestamps, dtype=np.float64)
    pose_odom = np.array(pose_data, dtype=np.float64)

    np.save(out / "pose_timestamps.npy", pose_timestamps)
    np.save(out / "pose_odom.npy", pose_odom)
    print(f"  pose: {pose_odom.shape[0]} samples")

    # --- Parse emat_waveform.csv ---
    emat_csv = run / "emat_waveform.csv"
    emat_timestamps = []
    emat_frames = []
    max_samples = 0

    if emat_csv.exists():
        with open(emat_csv, "r") as f:
            reader = csv.reader(f)
            header = next(reader, None)  # skip header
            for row in reader:
                if len(row) < 3:
                    continue
                stamp = float(row[0])
                sample_count = int(row[1])
                # row[2:] are hex-encoded bytes
                raw_bytes = bytes(int(h, 16) for h in row[2:])
                emat_timestamps.append(stamp)
                emat_frames.append(np.frombuffer(raw_bytes, dtype=np.uint8))
                if len(raw_bytes) > max_samples:
                    max_samples = len(raw_bytes)

        if emat_frames:
            # Pad to uniform length
            emat_waveform = np.zeros(
                (len(emat_frames), max_samples), dtype=np.uint8
            )
            for i, frame in enumerate(emat_frames):
                emat_waveform[i, :len(frame)] = frame

            emat_timestamps = np.array(emat_timestamps, dtype=np.float64)
            np.save(out / "emat_waveform.npy", emat_waveform)
            np.save(out / "emat_timestamps.npy", emat_timestamps)
            np.save(
                out / "contact_labels.npy",
                np.full(len(emat_frames), -1, dtype=np.int8),
            )
            print(f"  emat: {len(emat_frames)} frames, "
                  f"{max_samples} samples/frame")
        else:
            print("  emat: no frames found in CSV")
    else:
        print(f"  emat: {emat_csv} not found, skipping")

    # --- Copy videos ---
    for name in ("rgb.mp4", "depth.mp4"):
        src = run / name
        if src.exists():
            shutil.copy2(src, out / name)
            print(f"  copied {name}")
        else:
            print(f"  {name} not found, skipping")

    # --- Video timestamps (approximate) ---
    rgb_path = out / "rgb.mp4"
    if rgb_path.exists():
        # Estimate frame count from file size heuristic is unreliable.
        # Use opencv if available, otherwise skip.
        try:
            import cv2
            cap = cv2.VideoCapture(str(rgb_path))
            frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            cap.release()
            if frame_count > 0:
                fps = 30.0
                video_ts = np.arange(frame_count) / fps
                np.save(out / "video_timestamps.npy", video_ts)
                print(f"  video_timestamps: {frame_count} frames @ {fps} fps")
        except ImportError:
            print("  cv2 not available, skipping video_timestamps")

    # --- Metadata ---
    metadata = {
        "source_dir": str(run),
        "created_at": datetime.now().isoformat(),
        "pose_samples": int(pose_odom.shape[0]),
        "emat_frames": int(len(emat_frames)) if emat_frames else 0,
        "emat_max_samples": int(max_samples),
        "format_version": "1.0",
    }
    with open(out / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"  metadata.json written")


def convert_from_bag(bag_path: str, output_dir: str):
    """Convert rosbag data to numpy dataset."""
    try:
        import rosbag
    except ImportError:
        print("ERROR: rosbag Python module not available.")
        print("Install with: pip3 install rosbag  (or source ROS setup)")
        sys.exit(1)

    bag = rosbag.Bag(bag_path)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # --- Extract pose ---
    pose_timestamps = []
    pose_data = []
    for topic, msg, t in bag.read_messages(
        topics=["/mavros/local_position/odom"]
    ):
        ts = msg.header.stamp.toSec()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Quaternion to euler
        import math
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        pose_timestamps.append(ts)
        pose_data.append([p.x, p.y, p.z, roll, pitch, yaw])

    if pose_data:
        np.save(out / "pose_timestamps.npy",
                np.array(pose_timestamps, dtype=np.float64))
        np.save(out / "pose_odom.npy",
                np.array(pose_data, dtype=np.float64))
        print(f"  pose: {len(pose_data)} samples")

    # --- Extract EMAT ---
    emat_timestamps = []
    emat_frames = []
    max_samples = 0
    speed_of_voice = 3230

    for topic, msg, t in bag.read_messages(
        topics=["/emat/waveform"]
    ):
        ts = msg.stamp.toSec()
        raw = np.frombuffer(bytes(msg.raw_data), dtype=np.uint8)
        emat_timestamps.append(ts)
        emat_frames.append(raw)
        if len(raw) > max_samples:
            max_samples = len(raw)
        speed_of_voice = msg.speed_of_voice

    if emat_frames:
        emat_waveform = np.zeros(
            (len(emat_frames), max_samples), dtype=np.uint8
        )
        for i, frame in enumerate(emat_frames):
            emat_waveform[i, :len(frame)] = frame

        np.save(out / "emat_waveform.npy", emat_waveform)
        np.save(out / "emat_timestamps.npy",
                np.array(emat_timestamps, dtype=np.float64))
        np.save(out / "contact_labels.npy",
                np.full(len(emat_frames), -1, dtype=np.int8))
        print(f"  emat: {len(emat_frames)} frames, "
              f"{max_samples} samples/frame")

    # --- Extract images (save as video using cv2) ---
    try:
        import cv2
        _extract_video_from_bag(bag, out, "/d435/color/image_raw", "rgb.mp4")
        _extract_video_from_bag(
            bag, out, "/d435/aligned_depth_to_color/image_raw",
            "depth.mp4", is_depth=True
        )
    except ImportError:
        print("  cv2 not available, skipping video extraction")

    # --- Metadata ---
    metadata = {
        "source_bag": str(bag_path),
        "created_at": datetime.now().isoformat(),
        "pose_samples": len(pose_data),
        "emat_frames": len(emat_frames),
        "emat_max_samples": max_samples,
        "speed_of_voice": speed_of_voice,
        "format_version": "1.0",
    }
    with open(out / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"  metadata.json written")
    bag.close()


def _extract_video_from_bag(bag, out_dir, topic, filename, is_depth=False):
    """Extract image topic from bag to mp4 video."""
    import cv2
    from sensor_msgs.msg import Image as SensorImage
    from cv_bridge import CvBridge

    bridge = CvBridge()
    writer = None
    fps = 30.0

    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        try:
            if is_depth:
                cv_img = bridge.imgmsg_to_cv2(msg, "16UC1")
                cv_img = (cv_img.astype(np.float32) * 255.0 / 5000.0)
                cv_img = cv_img.clip(0, 255).astype(np.uint8)
            else:
                cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            continue

        if writer is None:
            h, w = cv_img.shape[:2]
            fourcc = cv.VideoWriter_fourcc(*"mp4v")
            color = not is_depth
            writer = cv2.VideoWriter(
                str(out_dir / filename), fourcc, fps, (w, h), color
            )

        if is_depth:
            # Convert single channel to 3-channel for VideoWriter
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        writer.write(cv_img)

    if writer is not None:
        writer.release()
        print(f"  extracted {filename}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert multimodal recording to numpy dataset"
    )
    parser.add_argument(
        "input",
        help="Path to recording directory (CSV mode) or .bag file",
    )
    parser.add_argument(
        "--bag", action="store_true",
        help="Treat input as rosbag file",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output directory (default: <input>_dataset or <bag_stem>_dataset)",
    )
    args = parser.parse_args()

    if args.bag:
        out = args.output or (args.input.rsplit(".", 1)[0] + "_dataset")
        print(f"Converting bag: {args.input}")
        print(f"  output: {out}")
        convert_from_bag(args.input, out)
    else:
        out = args.output or args.input
        print(f"Converting CSV dir: {args.input}")
        print(f"  output: {out}")
        convert_from_csv(args.input, out)

    print("Done.")


if __name__ == "__main__":
    main()
