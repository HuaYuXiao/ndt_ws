#!/usr/bin/env python3
"""训练物理约束探测状态检测模型。

从 datasets/ 加载已打标数据（dataset.npz 中的 state_labels，四类：
接近/接触/撤离/无效），提取深度特征，训练 PhysicsConstrainedContactDetector。
旧版二分类 contact_prob 标签不兼容，需先用 label_tool.py 重新标注。

Usage:
    python3 train_contact_detector.py [--epochs 50] [--batch 8] [--window 64]
"""

import argparse
import csv
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

# Add parent dir to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent))
from physics_attention import (
    PhysicsConstrainedContactDetector,
    compute_smoothness_loss,
    N_STATES,
    STATE_NAMES,
    STATE_INVALID,
    CONTACT_CLASS,
)


# ── Feature Extraction ──────────────────────────────────────────

def extract_depth_features(depth_dir, n_frames):
    """从深度 PNG 序列提取 6D 视觉特征。

    Returns: (n_frames, 6) — mean_depth, depth_var, grad_x, grad_y, norm_depth, fill_ratio
    """
    features = np.zeros((n_frames, 6), dtype=np.float32)
    for i in range(n_frames):
        path = os.path.join(depth_dir, f"{i:06d}.png")
        if not os.path.exists(path):
            continue
        img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if img is None:
            continue
        valid = img[img > 0]
        if len(valid) == 0:
            continue

        mean_d = float(np.mean(valid))
        var_d = float(np.var(valid))

        # Gradients (Sobel on float32)
        img_f = img.astype(np.float32)
        grad_x = np.abs(cv2.Sobel(img_f, cv2.CV_32F, 1, 0, ksize=3))
        grad_y = np.abs(cv2.Sobel(img_f, cv2.CV_32F, 0, 1, ksize=3))
        gx = float(np.mean(grad_x[img > 0]))
        gy = float(np.mean(grad_y[img > 0]))

        features[i] = [
            mean_d / 1000.0,       # mean depth (m)
            var_d / 1e6,           # depth variance (m²)
            gx / 1000.0,           # horizontal gradient
            gy / 1000.0,           # vertical gradient
            mean_d / 5000.0,       # normalized depth
            len(valid) / img.size, # fill ratio
        ]
    return features


def extract_pose_features(pose):
    """从位姿提取 6D 特征 (xyz + rpy)。"""
    return pose[:, :6].astype(np.float32)


# ── Dataset ──────────────────────────────────────────────────────

class ContactDataset(Dataset):
    """滑窗数据集：从多个录制中提取固定长度窗口。"""

    def __init__(self, windows, labels, timestamps, positions, residuals, physics_mats):
        self.windows = windows          # (N, T, 6)
        self.labels = labels            # (N, T)
        self.timestamps = timestamps    # (N, T)
        self.positions = positions      # (N, T, 3)
        self.residuals = residuals      # (N, T, 3)
        self.physics_mats = physics_mats  # (N, T, T)

    def __len__(self):
        return len(self.windows)

    def __getitem__(self, idx):
        return {
            'vis': self.windows[idx],
            'labels': self.labels[idx],
            'ts': self.timestamps[idx],
            'pos': self.positions[idx],
            'res': self.residuals[idx],
            'P': self.physics_mats[idx],
        }


def build_windows(features, labels, timestamps, positions, residuals,
                  window_size=64, stride=16):
    """从单个录制构建滑窗数据（丢弃包含未标注帧的窗口）。"""
    n = len(features)
    if n < window_size:
        return [], [], [], [], [], []

    windows, lbls, ts_list, pos_list, res_list = [], [], [], [], []

    for start in range(0, n - window_size + 1, stride):
        end = start + window_size
        lbl = labels[start:end]
        if np.any(lbl < 0):  # -1 = 未标注
            continue
        windows.append(features[start:end])
        lbls.append(lbl)
        ts_list.append(timestamps[start:end])
        pos_list.append(positions[start:end])
        res_list.append(residuals[start:end])

    return windows, lbls, ts_list, pos_list, res_list


def build_physics_matrices_batch(timestamps_batch, positions_batch, residuals_batch):
    """为 batch 构建物理约束矩阵。"""
    from physics_attention import build_physics_constraint_matrix
    B, T = timestamps_batch.shape
    P_batch = torch.zeros(B, T, T)
    for i in range(B):
        P_batch[i] = build_physics_constraint_matrix(
            timestamps_batch[i], positions_batch[i], residuals_batch[i])
    return P_batch


# ── Data Loading ─────────────────────────────────────────────────

def load_all_datasets(datasets_dir, window_size=64, stride=16):
    """加载所有已打标数据集并构建训练数据。"""
    datasets_dir = Path(datasets_dir)
    all_windows, all_labels = [], []
    all_ts, all_pos, all_res = [], [], []

    # Find all dataset.npz files
    npz_files = sorted(datasets_dir.rglob("dataset.npz"))

    for npz_path in npz_files:
        run_dir = npz_path.parent
        npz = np.load(str(npz_path), allow_pickle=True)

        n = len(npz["timestamps"])

        # Handle missing keys gracefully
        def _get(key, default_shape, dtype):
            if key in npz:
                return npz[key].astype(dtype)
            return np.full(default_shape, np.nan, dtype=dtype)

        # 四类状态标签（label_tool.py 输出，-1 = 未标注）。
        # 旧版二分类 contact_prob 无法区分接近/撤离/无效，需重新标注。
        if "state_labels" not in npz:
            print(f"  SKIP {run_dir.relative_to(datasets_dir)}: "
                  f"无 state_labels（请用 label_tool.py 重新标注四类状态）")
            continue
        labels = npz["state_labels"].astype(np.float32)
        labels = np.where(np.isnan(labels), -1, labels).astype(np.int8)

        # Skip unlabeled datasets
        if np.all(labels < 0) or np.all(labels == STATE_INVALID):
            print(f"  SKIP {run_dir.relative_to(datasets_dir)}: no valid labels")
            continue

        timestamps = npz["timestamps"].astype(np.float64)
        positions = _get("pose", (n, 6), np.float64)[:, :3].astype(np.float32)
        # 加载 target_pose 并计算位姿残差 (target - odom)
        target_pose = _get("target_pose", (n, 3), np.float32)
        residuals = target_pose - positions  # (N, 3): target - odom
        # Replace NaN residuals with zeros (physics matrix will skip residual term)
        residuals = np.nan_to_num(residuals, nan=0.0)

        # Extract features
        depth_dir = run_dir / "depth"
        if depth_dir.exists() and len(list(depth_dir.glob("*.png"))) > 0:
            features = extract_depth_features(str(depth_dir), n)
            src = "depth"
        elif "pose" in npz:
            features = extract_pose_features(npz["pose"])
            src = "pose"
        else:
            print(f"  SKIP {run_dir.relative_to(datasets_dir)}: no features")
            continue

        # Normalize timestamps to relative
        t_rel = timestamps - timestamps[0]

        # Build windows
        w, l, t, p, r = build_windows(
            features, labels, t_rel, positions, residuals, window_size, stride)

        if w:
            all_windows.extend(w)
            all_labels.extend(l)
            all_ts.extend(t)
            all_pos.extend(p)
            all_res.extend(r)
            counts = {STATE_NAMES[c]: sum(int(np.sum(x == c)) for x in l)
                      for c in range(N_STATES)}
            print(f"  LOAD {run_dir.relative_to(datasets_dir)}: "
                  f"{len(w)} windows ({src}), {counts}")

    if not all_windows:
        return None

    # Convert to tensors
    vis = torch.tensor(np.stack(all_windows), dtype=torch.float32)
    lbl = torch.tensor(np.stack(all_labels), dtype=torch.long)
    ts = torch.tensor(np.stack(all_ts), dtype=torch.float32)
    pos = torch.tensor(np.stack(all_pos), dtype=torch.float32)
    res = torch.tensor(np.stack(all_res), dtype=torch.float32)

    # Pre-compute physics matrices
    print("  Computing physics constraint matrices...")
    P = build_physics_matrices_batch(ts, pos, res)

    return ContactDataset(vis, lbl, ts, pos, res, P)


# ── Training ─────────────────────────────────────────────────────

def compute_class_metrics(preds, labels):
    """计算准确率、每类 precision/recall/F1 与 macro-F1。

    Args:
        preds:  (M,) long tensor 预测类别
        labels: (M,) long tensor 真实类别
    Returns:
        acc, per_class (dict: class -> (prec, rec, f1)), macro_f1
    """
    preds = preds.detach().cpu().numpy()
    labels = labels.detach().cpu().numpy()
    acc = float(np.mean(preds == labels))
    per_class = {}
    f1s = []
    for c in range(N_STATES):
        tp = int(np.sum((preds == c) & (labels == c)))
        fp = int(np.sum((preds == c) & (labels != c)))
        fn = int(np.sum((preds != c) & (labels == c)))
        prec = tp / max(tp + fp, 1)
        rec = tp / max(tp + fn, 1)
        f1 = 2 * prec * rec / max(prec + rec, 1e-8)
        per_class[c] = (prec, rec, f1)
        if tp + fp + fn > 0:  # 仅统计出现在该批中的类别
            f1s.append(f1)
    macro_f1 = float(np.mean(f1s)) if f1s else 0.0
    return acc, per_class, macro_f1


def train(model, train_loader, val_loader, device, epochs=50, lr=1e-3,
          csv_path=None, save_dir=None):
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    criterion = nn.CrossEntropyLoss(
        weight=torch.tensor([1.0, 1.0, 3.0, 1.0], device=device))  # 接触类 3x 权重

    best_val_f1 = 0.0

    # CSV logging
    csv_file = None
    csv_writer = None
    if csv_path:
        csv_file = open(csv_path, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            'epoch', 'lr',
            'train_loss', 'train_acc', 'train_f1_macro', 'train_f1_contact',
            'val_loss', 'val_acc', 'val_f1_macro', 'val_f1_contact',
        ])

    for epoch in range(epochs):
        # Train
        model.train()
        train_loss = 0.0
        train_preds, train_labels = [], []

        for batch in train_loader:
            vis = batch['vis'].to(device)
            labels = batch['labels'].to(device).long()
            P = batch['P'].to(device)

            optimizer.zero_grad()
            logits, _ = model(vis, batch['ts'].to(device),
                              batch['pos'].to(device), batch['res'].to(device))
            logits = logits.view(-1, N_STATES)
            labels_flat = labels.view(-1)

            loss = criterion(logits, labels_flat)

            # Smoothness loss on full state distribution
            probs = torch.softmax(logits, dim=-1).view(
                vis.shape[0], -1, N_STATES)
            loss = loss + 0.1 * compute_smoothness_loss(probs)

            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()

            train_loss += loss.item()
            train_preds.append(logits.argmax(dim=-1))
            train_labels.append(labels_flat)

        scheduler.step()

        train_acc, train_pc, train_f1m = compute_class_metrics(
            torch.cat(train_preds), torch.cat(train_labels))
        train_f1c = train_pc[CONTACT_CLASS][2]

        # Validation
        val_loss, val_acc, val_pc, val_f1m = evaluate(
            model, val_loader, criterion, device)
        val_f1c = val_pc[CONTACT_CLASS][2]

        current_lr = optimizer.param_groups[0]['lr']
        print(f"Epoch {epoch+1:3d}/{epochs} | "
              f"Train Loss={train_loss:.4f} Acc={train_acc:.3f} "
              f"F1m={train_f1m:.3f} F1c={train_f1c:.3f} | "
              f"Val Loss={val_loss:.4f} Acc={val_acc:.3f} "
              f"F1m={val_f1m:.3f} F1c={val_f1c:.3f} | "
              f"ap={val_pc[0][2]:.2f} ct={val_pc[1][2]:.2f} "
              f"rt={val_pc[2][2]:.2f} iv={val_pc[3][2]:.2f}")

        # Log to CSV
        if csv_writer:
            csv_writer.writerow([
                epoch + 1, f"{current_lr:.6f}",
                f"{train_loss:.4f}", f"{train_acc:.4f}",
                f"{train_f1m:.4f}", f"{train_f1c:.4f}",
                f"{val_loss:.4f}", f"{val_acc:.4f}",
                f"{val_f1m:.4f}", f"{val_f1c:.4f}",
            ])
            csv_file.flush()

        if val_f1m > best_val_f1:
            best_val_f1 = val_f1m
            save_path = Path(save_dir) if save_dir else (
                Path(__file__).resolve().parent.parent / "models")
            save_path.mkdir(exist_ok=True)
            torch.save({
                'model_state': model.state_dict(),
                'val_f1': val_f1m,
                'epoch': epoch,
            }, str(save_path / "contact_detector_best.pt"))
            print(f"  -> Saved best model (F1={val_f1m:.3f})")

    if csv_file:
        csv_file.close()
        print(f"Training log saved to {csv_path}")


def evaluate(model, loader, criterion, device):
    model.eval()
    total_loss = 0.0
    all_preds, all_labels = [], []

    with torch.no_grad():
        for batch in loader:
            vis = batch['vis'].to(device)
            labels = batch['labels'].to(device).long()

            logits, _ = model(vis, batch['ts'].to(device),
                              batch['pos'].to(device), batch['res'].to(device))
            logits = logits.view(-1, N_STATES)
            labels_flat = labels.view(-1)

            loss = criterion(logits, labels_flat)
            total_loss += loss.item()

            all_preds.append(logits.argmax(dim=-1))
            all_labels.append(labels_flat)

    acc, per_class, macro_f1 = compute_class_metrics(
        torch.cat(all_preds), torch.cat(all_labels))
    return total_loss, acc, per_class, macro_f1


# ── Main ─────────────────────────────────────────────────────────

def find_next_run_dir():
    """Find the next available run directory (src/ndt/runs/N)."""
    runs_dir = Path(__file__).resolve().parent.parent / "runs"
    runs_dir.mkdir(exist_ok=True)
    existing = [int(d.name) for d in runs_dir.iterdir()
                if d.is_dir() and d.name.isdigit()]
    next_idx = max(existing, default=-1) + 1
    return runs_dir / str(next_idx)


def main():
    parser = argparse.ArgumentParser(description="Train contact detector")
    parser.add_argument("--datasets", default="src/record/datasets",
                        help="Path to datasets directory")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--window", type=int, default=64)
    parser.add_argument("--stride", type=int, default=16)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--val-split", type=float, default=0.2)
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to .pt file for transfer learning")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory (default: auto-increment runs/N)")
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # Load data
    print("Loading datasets...")
    dataset = load_all_datasets(args.datasets, args.window, args.stride)
    if dataset is None:
        print("ERROR: No labeled data found")
        return

    print(f"\nTotal windows: {len(dataset)}")

    # Train/val split
    n_val = max(1, int(len(dataset) * args.val_split))
    n_train = len(dataset) - n_val
    train_ds, val_ds = torch.utils.data.random_split(dataset, [n_train, n_val])

    train_loader = DataLoader(train_ds, batch_size=args.batch, shuffle=True,
                              num_workers=0, drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch, shuffle=False,
                            num_workers=0)

    # Model
    model = PhysicsConstrainedContactDetector(
        d_vis=6, d_model=128, n_heads=8, n_layers=2, dropout=0.1
    ).to(device)

    # Resume from checkpoint for transfer learning
    if args.resume:
        ckpt = torch.load(args.resume, map_location=device)
        try:
            model.load_state_dict(ckpt['model_state'])
        except RuntimeError as e:
            print(f"ERROR: 权重加载失败（旧版二分类权重与四类输出头不兼容？）: {e}")
            print("       请使用四类状态标注重新训练，或移除 --resume 参数。")
            return
        print(f"Resumed from {args.resume} (val_f1={ckpt.get('val_f1', '?')}, "
              f"epoch={ckpt.get('epoch', '?')})")

    n_params = sum(p.numel() for p in model.parameters())
    print(f"Model: {n_params:,} parameters")
    print(f"Train: {n_train} windows, Val: {n_val} windows")
    print(f"Training for {args.epochs} epochs...\n")

    # Output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        output_dir = find_next_run_dir()
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output: {output_dir}\n")

    csv_path = str(output_dir / "training_log.csv")
    train(model, train_loader, val_loader, device, args.epochs, args.lr,
          csv_path=csv_path, save_dir=str(output_dir))
    print("\nDone!")


if __name__ == "__main__":
    main()
