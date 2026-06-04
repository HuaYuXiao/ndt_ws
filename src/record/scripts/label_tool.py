#!/usr/bin/env python3
"""Multi-modal contact labeling tool for EMAT contact detection datasets.

Usage:
    python3 label_tool.py [run_dir]

Keyboard shortcuts:
    Left/Right     Previous/next frame
    Shift+Left/Right  Jump ±10 frames
    Space          Toggle playback
    C              Mark contact start
    V              Mark contact end
    U              Undo last operation
    S              Save labels
    Delete         Remove region under cursor
"""

import argparse
import csv
import json
import os
import sys
from datetime import datetime
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import numpy as np
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QFont, QKeySequence
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QToolBar, QAction, QLabel, QSlider, QPushButton, QFileDialog,
    QSpinBox, QShortcut, QSizePolicy, QMessageBox,
)


def _safe_float(s):
    """Parse string as float, return None for 'nan' or invalid."""
    if not s or s.strip() == "nan":
        return None
    try:
        return float(s)
    except ValueError:
        return None


# ────────────────────────────────────────────────────────────────
# Data Loading
# ────────────────────────────────────────────────────────────────

class DataLoader:
    """Loads all data for a single recording run."""

    def __init__(self, run_dir: str):
        self.run_dir = Path(run_dir)

        # dataset.npz
        npz_path = self.run_dir / "dataset.npz"
        if not npz_path.exists():
            raise FileNotFoundError(f"dataset.npz not found in {run_dir}")
        npz = np.load(npz_path, allow_pickle=True)
        self.timestamps = npz["timestamps"].astype(np.float64)
        self.pose = npz["pose"].astype(np.float64)
        self.n_frames = len(self.timestamps)

        def _get(key, shape_col, dtype):
            if key in npz:
                return npz[key].astype(dtype)
            return np.full((self.n_frames, shape_col), np.nan, dtype=dtype) \
                if shape_col > 1 else np.full(self.n_frames, np.nan, dtype=dtype)

        self.normals = _get("normals", 3, np.float32)
        self.emat_features = _get("emat_features", 14, np.float32)
        self.contact_prob = _get("contact_prob", 0, np.float32)

        self.t_rel = self.timestamps - self.timestamps[0]

        # Video capture (suppress ffmpeg stderr for corrupted mp4)
        self.rgb_cap = None
        self.depth_cap = None
        rgb_path = self.run_dir / "rgb.mp4"
        depth_path = self.run_dir / "depth.mp4"
        devnull = open(os.devnull, 'w')
        old_stderr = os.dup(2)
        os.dup2(devnull.fileno(), 2)
        try:
            if rgb_path.exists():
                self.rgb_cap = cv2.VideoCapture(str(rgb_path))
                if not self.rgb_cap.isOpened():
                    self.rgb_cap = None
            if depth_path.exists():
                self.depth_cap = cv2.VideoCapture(str(depth_path))
                if not self.depth_cap.isOpened():
                    self.depth_cap = None
        finally:
            os.dup2(old_stderr, 2)
            os.close(old_stderr)
            devnull.close()

        # EMAT waveform (optional)
        wf_path = self.run_dir / "emat_waveform.npy"
        self.emat_waveform = np.load(str(wf_path)) if wf_path.exists() else None

        # Pre-existing labels
        self.has_emat = bool(
            self.n_frames > 0
            and not np.all(np.isnan(self.emat_features[:, 0]))
            and np.nanmax(self.emat_features[:, 0]) > 0
        )

        # Target data from record_log.csv (for error computation)
        self.error_x = self._load_error_x()

    def get_rgb_frame(self, idx: int):
        if self.rgb_cap is None:
            return None
        self.rgb_cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = self.rgb_cap.read()
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) if ok else None

    def get_depth_frame(self, idx: int):
        if self.depth_cap is None:
            return None
        self.depth_cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = self.depth_cap.read()
        if not ok:
            return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)

    def release(self):
        if self.rgb_cap:
            self.rgb_cap.release()
        if self.depth_cap:
            self.depth_cap.release()

    def _load_error_x(self):
        """Load target_x from record_log.csv, compute error_x = target_x - pose_x."""
        csv_path = self.run_dir / "record_log.csv"
        if not csv_path.exists():
            return None
        try:
            log_ts, log_target_x, log_odom_x = [], [], []
            with open(csv_path) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    tx = _safe_float(row.get("target_x"))
                    ox = _safe_float(row.get("odom_x"))
                    ts = _safe_float(row.get("t"))
                    if ts is not None and tx is not None and ox is not None:
                        log_ts.append(ts)
                        log_target_x.append(tx)
                        log_odom_x.append(ox)
            if not log_ts:
                return None
            log_ts = np.array(log_ts)
            log_target_x = np.array(log_target_x)
            log_odom_x = np.array(log_odom_x)
            # Interpolate to match frame timestamps
            target_x_interp = np.interp(self.timestamps, log_ts, log_target_x)
            pose_x = self.pose[:, 0]
            return target_x_interp - pose_x
        except Exception:
            return None


# ────────────────────────────────────────────────────────────────
# Timeline Widget (custom painted)
# ────────────────────────────────────────────────────────────────

class TimelineWidget(QWidget):
    """Custom timeline with contact regions, click-to-navigate, drag-to-label."""

    frame_changed = pyqtSignal(int)
    region_created = pyqtSignal(int, int)

    HANDLE_W = 6  # pixels for drag handle

    def __init__(self, parent=None):
        super().__init__(parent)
        self.n_frames = 1
        self.current_frame = 0
        self.contact_regions = []  # list of [start, end]
        self.setMinimumHeight(64)
        self.setMaximumHeight(84)
        self.setMouseTracking(True)

        self._dragging = False
        self._drag_start_frame = 0
        self._drag_mode = None   # 'create', 'move_start', 'move_end'
        self._drag_region_idx = -1
        self._moved = False

    def set_frames(self, n: int):
        self.n_frames = max(n, 1)

    def set_frame(self, f: int):
        self.current_frame = max(0, min(f, self.n_frames - 1))
        self.update()

    def set_regions(self, regions):
        self.contact_regions = sorted(regions, key=lambda r: r[0])
        self.update()

    def _frame_to_x(self, frame):
        margin = 10
        w = self.width() - 2 * margin
        return margin + int(frame / max(self.n_frames - 1, 1) * w)

    def _x_to_frame(self, x):
        margin = 10
        w = self.width() - 2 * margin
        f = int((x - margin) / max(w, 1) * (self.n_frames - 1))
        return max(0, min(f, self.n_frames - 1))

    def _region_at_x(self, x):
        frame = self._x_to_frame(x)
        for i, (s, e) in enumerate(self.contact_regions):
            sx = self._frame_to_x(s)
            ex = self._frame_to_x(e)
            if abs(x - sx) <= self.HANDLE_W:
                return i, 'start'
            if abs(x - ex) <= self.HANDLE_W:
                return i, 'end'
            if sx <= x <= ex:
                return i, 'body'
        return -1, None

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Background
        p.fillRect(0, 0, w, h, QColor(250, 250, 250))

        margin = 12
        bar_top = 16
        bar_h = h - 38
        bar_w = w - 2 * margin

        # Bar background (non-contact)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(224, 224, 224)))
        p.drawRoundedRect(margin, bar_top, bar_w, bar_h, 4, 4)

        # Contact regions (Material Green 500)
        for s, e in self.contact_regions:
            sx = self._frame_to_x(s)
            ex = self._frame_to_x(e)
            p.setBrush(QBrush(QColor(76, 175, 80, 200)))
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(sx, bar_top, ex - sx, bar_h, 4, 4)

        # Current frame cursor (Material Red A200)
        cx = self._frame_to_x(self.current_frame)
        p.setPen(QPen(QColor(255, 82, 82), 2))
        p.drawLine(cx, bar_top - 4, cx, bar_top + bar_h + 4)

        # Frame ticks
        p.setPen(QColor(158, 158, 158))
        p.setFont(QFont("Segoe UI", 8))
        n_ticks = min(10, self.n_frames)
        for i in range(n_ticks + 1):
            f = int(i / n_ticks * (self.n_frames - 1))
            tx = self._frame_to_x(f)
            p.drawLine(tx, bar_top + bar_h, tx, bar_top + bar_h + 4)
            p.drawText(tx - 15, bar_top + bar_h + 5, 30, 15,
                       Qt.AlignCenter, str(f))

        # Current frame label
        p.setPen(QColor(255, 82, 82))
        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        p.drawText(cx - 20, bar_top - 15, 40, 13,
                   Qt.AlignCenter, str(self.current_frame))

        p.end()

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._moved = False
            region_idx, part = self._region_at_x(ev.x())
            if part in ('start', 'end'):
                self._drag_mode = f'move_{part}'
                self._drag_region_idx = region_idx
                self._dragging = True
            else:
                self._drag_start_frame = self._x_to_frame(ev.x())
                self._drag_mode = 'create'
                self._dragging = True

    def mouseMoveEvent(self, ev):
        if self._dragging:
            self._moved = True

    def mouseReleaseEvent(self, ev):
        if ev.button() != Qt.LeftButton:
            return
        if not self._dragging:
            return
        self._dragging = False
        end_frame = self._x_to_frame(ev.x())

        if self._drag_mode == 'create':
            if self._moved:
                s = min(self._drag_start_frame, end_frame)
                e = max(self._drag_start_frame, end_frame)
                if e > s:
                    self.region_created.emit(s, e)
            else:
                self.frame_changed.emit(end_frame)

        elif self._drag_mode in ('move_start', 'move_end'):
            idx = self._drag_region_idx
            if 0 <= idx < len(self.contact_regions):
                if self._drag_mode == 'move_start':
                    self.contact_regions[idx][0] = min(end_frame,
                                                       self.contact_regions[idx][1])
                else:
                    self.contact_regions[idx][1] = max(end_frame,
                                                       self.contact_regions[idx][0])
                self.update()

        self._drag_mode = None
        self._drag_region_idx = -1

    def _add_region(self, start, end):
        new = [min(start, end), max(start, end)]
        merged = []
        for s, e in self.contact_regions:
            if new[0] <= e + 1 and new[1] >= s - 1:
                new[0] = min(new[0], s)
                new[1] = max(new[1], e)
            else:
                merged.append([s, e])
        merged.append(new)
        self.contact_regions = sorted(merged, key=lambda r: r[0])
        self.update()


# ────────────────────────────────────────────────────────────────
# Matplotlib Canvas (reusable)
# ────────────────────────────────────────────────────────────────

class MplCanvas(FigureCanvasQTAgg):
    """Base matplotlib canvas with click-to-navigate support."""

    clicked_frame = pyqtSignal(int)

    # Fixed margins to align axes across all canvases (matches TimelineWidget margin)
    MARGIN_LEFT = 0.05
    MARGIN_RIGHT = 0.97

    def __init__(self, nrows=1, ncols=1, figsize=(12, 3)):
        self.fig = Figure(figsize=figsize, facecolor='white')
        self.fig.subplots_adjust(
            left=self.MARGIN_LEFT, right=self.MARGIN_RIGHT,
            hspace=0.35, top=0.92, bottom=0.12)
        super().__init__(self.fig)
        self.axes = [self.fig.add_subplot(nrows, ncols, i + 1)
                     for i in range(nrows * ncols)]
        self.n_frames = 1
        self._cid = self.fig.canvas.mpl_connect(
            'button_press_event', self._on_click)

    def set_frames(self, n):
        self.n_frames = max(n, 1)

    def _on_click(self, ev):
        if ev.inaxes and ev.xdata is not None:
            f = int(max(0, min(ev.xdata, self.n_frames - 1)))
            self.clicked_frame.emit(f)

    def clear_axes(self):
        for ax in self.axes:
            ax.clear()


# ────────────────────────────────────────────────────────────────
# Video Canvas
# ────────────────────────────────────────────────────────────────

class VideoCanvas(MplCanvas):
    """Displays RGB and depth side by side."""

    def __init__(self):
        super().__init__(nrows=1, ncols=2, figsize=(12, 3.5))
        self.axes[0].set_title("RGB", fontsize=10)
        self.axes[1].set_title("Depth", fontsize=10)
        for ax in self.axes:
            ax.set_xticks([])
            ax.set_yticks([])

    def update_frames(self, rgb, depth, frame_idx, t):
        for ax in self.axes:
            ax.clear()
            ax.set_xticks([])
            ax.set_yticks([])

        if rgb is not None:
            self.axes[0].imshow(rgb)
            self.axes[0].set_title(f"RGB  #{frame_idx}  t={t:.2f}s", fontsize=9)
        else:
            self.axes[0].text(0.5, 0.5, "No RGB", ha='center', va='center',
                              transform=self.axes[0].transAxes, fontsize=12)
        if depth is not None:
            self.axes[1].imshow(depth)
            self.axes[1].set_title(f"Depth  #{frame_idx}", fontsize=9)
        else:
            self.axes[1].text(0.5, 0.5, "No Depth", ha='center', va='center',
                              transform=self.axes[1].transAxes, fontsize=12)
        self.draw_idle()


# ────────────────────────────────────────────────────────────────
# EMAT / Pose Canvas
# ────────────────────────────────────────────────────────────────

class DataCanvas(MplCanvas):
    """EMAT features (top) and error_x (bottom) plots."""

    def __init__(self):
        super().__init__(nrows=2, ncols=1, figsize=(12, 4))
        self.vline = [None, None]

    def plot_data(self, data: DataLoader, contact_regions):
        self.set_frames(data.n_frames)
        x = np.arange(data.n_frames)

        # --- EMAT features ---
        ax0 = self.axes[0]
        ax0.clear()
        if data.has_emat:
            energy = data.emat_features[:, 0]
            peak = data.emat_features[:, 1]
            ax0.plot(x, energy, 'b-', lw=0.8, label='energy', alpha=0.8)
            ax0_twin = ax0.twinx()
            ax0_twin.plot(x, peak, 'm-', lw=0.8, label='peak_amp', alpha=0.6)
            ax0_twin.set_ylabel('Peak Amp', fontsize=8, color='m')
            ax0_twin.tick_params(axis='y', labelcolor='m', labelsize=7)
            ax0.legend(loc='upper left', fontsize=7)
        else:
            ax0.text(0.5, 0.5, "No EMAT data", ha='center', va='center',
                     transform=ax0.transAxes, fontsize=11, color='gray')
        ax0.set_ylabel("EMAT", fontsize=9)
        ax0.tick_params(labelsize=7)
        ax0.grid(True, alpha=0.3)

        # --- Error X (target_x - pose_x) ---
        ax1 = self.axes[1]
        ax1.clear()
        if data.error_x is not None:
            ax1.plot(x, data.error_x, 'b-', lw=0.8)
            ax1.axhline(0, color='gray', lw=0.5, ls='--')
        else:
            ax1.text(0.5, 0.5, "No target data", ha='center', va='center',
                     transform=ax1.transAxes, fontsize=11, color='gray')
        ax1.set_ylabel("error_x (m)", fontsize=9)
        ax1.set_xlabel("Frame", fontsize=9)
        ax1.tick_params(labelsize=7)
        ax1.grid(True, alpha=0.3)

        # Contact region shading
        for ax in self.axes:
            for s, e in contact_regions:
                ax.axvspan(s, e, alpha=0.2, color='green')
            ax.set_xlim(0, data.n_frames - 1)

        self.draw_idle()

    def update_cursor(self, frame):
        for i, ax in enumerate(self.axes):
            if self.vline[i]:
                self.vline[i].remove()
            self.vline[i] = ax.axvline(frame, color='red', lw=1.5, alpha=0.8)
        self.draw_idle()


# ────────────────────────────────────────────────────────────────
# Main Window
# ────────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self, run_dir=None):
        super().__init__()
        self.setWindowTitle("Contact Labeling Tool")
        self.setMinimumSize(1100, 800)
        self.setWindowState(Qt.WindowMaximized)

        self.data = None
        self.current_frame = 0
        self.playing = False
        self.play_timer = QTimer(self)
        self.play_timer.timeout.connect(self._advance_frame)
        self.playback_speed = 1.0  # 1.0x = real-time

        # Labeling state
        self.contact_regions = []   # [[start, end], ...]
        self.undo_stack = []
        self.marking_start = -1     # -1 = idle, >=0 = waiting for end

        self._build_ui()
        self._bind_shortcuts()

        if run_dir:
            self._load_dataset(run_dir)

    # ── UI construction ──

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(6)

        # Toolbar row
        toolbar = QHBoxLayout()
        self.btn_load = QPushButton("Load Dataset")
        self.btn_load.setStyleSheet(
            "background-color: #1976D2; color: white; font-weight: bold; "
            "border: none; border-radius: 6px; padding: 5px 16px;")
        self.btn_load.clicked.connect(self._on_load)
        toolbar.addWidget(self.btn_load)

        nav_style = ("background-color: #FFFFFF; border: 1px solid #E0E0E0; "
                     "border-radius: 6px; padding: 4px;")
        self.btn_prev10 = QPushButton("◀◀")
        self.btn_prev = QPushButton("◀")
        self.btn_next = QPushButton("▶")
        self.btn_next10 = QPushButton("▶▶")
        self.btn_play = QPushButton("▶/⏸")
        for btn, slot in [
            (self.btn_prev10, lambda: self._step(-10)),
            (self.btn_prev, lambda: self._step(-1)),
            (self.btn_next, lambda: self._step(1)),
            (self.btn_next10, lambda: self._step(10)),
            (self.btn_play, self._toggle_play),
        ]:
            btn.setFixedWidth(42)
            btn.setStyleSheet(nav_style)
            btn.clicked.connect(slot)
            toolbar.addWidget(btn)

        toolbar.addWidget(QLabel("  Frame:"))
        self.frame_spin = QSpinBox()
        self.frame_spin.setMinimum(0)
        self.frame_spin.setMaximum(0)
        self.frame_spin.valueChanged.connect(self._on_spin)
        toolbar.addWidget(self.frame_spin)

        self.lbl_total = QLabel("/ 0")
        toolbar.addWidget(self.lbl_total)

        self.lbl_time = QLabel("  Time: 0.00s")
        toolbar.addWidget(self.lbl_time)

        self.lbl_labels = QLabel("  Labels: 0")
        toolbar.addWidget(self.lbl_labels)

        toolbar.addStretch()

        md_primary = ("background-color: #1976D2; color: white; font-weight: bold; "
                      "border: none; border-radius: 6px; padding: 4px 12px;")
        md_secondary = ("background-color: #4CAF50; color: white; font-weight: bold; "
                        "border: none; border-radius: 6px; padding: 4px 12px;")
        md_outline = ("background-color: transparent; border: 1px solid #BDBDBD; "
                      "border-radius: 6px; padding: 4px 12px;")
        md_accent = ("background-color: #FF9800; color: white; font-weight: bold; "
                     "border: none; border-radius: 6px; padding: 4px 12px;")

        self.btn_mark_start = QPushButton("Contact Start (C)")
        self.btn_mark_start.setStyleSheet(md_primary)
        self.btn_mark_start.clicked.connect(self._mark_contact_start)
        toolbar.addWidget(self.btn_mark_start)

        self.btn_mark_end = QPushButton("Contact End (V)")
        self.btn_mark_end.setStyleSheet(md_secondary)
        self.btn_mark_end.clicked.connect(self._mark_contact_end)
        toolbar.addWidget(self.btn_mark_end)

        self.btn_clear = QPushButton("Clear All")
        self.btn_clear.setStyleSheet(md_outline)
        self.btn_clear.clicked.connect(self._clear_all)
        toolbar.addWidget(self.btn_clear)

        self.btn_undo = QPushButton("Undo (U)")
        self.btn_undo.setStyleSheet(md_outline)
        self.btn_undo.clicked.connect(self._undo)
        toolbar.addWidget(self.btn_undo)

        self.btn_save = QPushButton("Save (S)")
        self.btn_save.setStyleSheet(md_accent)
        self.btn_save.clicked.connect(self._save)
        toolbar.addWidget(self.btn_save)

        self.lbl_status = QLabel("  Ready")
        self.lbl_status.setStyleSheet(
            "color: #757575; font-size: 12px; padding-left: 8px;")
        toolbar.addWidget(self.lbl_status)

        layout.addLayout(toolbar)

        # Frame slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(0)
        self.slider.valueChanged.connect(self._on_slider)
        layout.addWidget(self.slider)

        # Video canvas
        self.video_canvas = VideoCanvas()
        self.video_canvas.clicked_frame.connect(self._go_to_frame)
        layout.addWidget(self.video_canvas)

        # Data canvas (EMAT + altitude)
        self.data_canvas = DataCanvas()
        self.data_canvas.clicked_frame.connect(self._go_to_frame)
        layout.addWidget(self.data_canvas)

        # Timeline
        self.timeline = TimelineWidget()
        self.timeline.frame_changed.connect(self._go_to_frame)
        self.timeline.region_created.connect(self._on_region_created)
        layout.addWidget(self.timeline)

    def _bind_shortcuts(self):
        QShortcut(QKeySequence(Qt.Key_Left), self, lambda: self._step(-1))
        QShortcut(QKeySequence(Qt.Key_Right), self, lambda: self._step(1))
        QShortcut(QKeySequence(Qt.SHIFT | Qt.Key_Left), self,
                  lambda: self._step(-10))
        QShortcut(QKeySequence(Qt.SHIFT | Qt.Key_Right), self,
                  lambda: self._step(10))
        QShortcut(QKeySequence(Qt.Key_Space), self, self._toggle_play)
        QShortcut(QKeySequence(Qt.Key_C), self, self._mark_contact_start)
        QShortcut(QKeySequence(Qt.Key_V), self, self._mark_contact_end)
        QShortcut(QKeySequence(Qt.Key_U), self, self._undo)
        QShortcut(QKeySequence(Qt.Key_S), self, self._save)
        QShortcut(QKeySequence(Qt.Key_Delete), self, self._delete_region)

    # ── Data loading ──

    def _on_load(self):
        d = QFileDialog.getExistingDirectory(
            self, "Select Recording Directory",
            str(Path(__file__).resolve().parent.parent / "datasets"))
        if d:
            self._load_dataset(d)

    def _load_dataset(self, run_dir):
        try:
            self.data = DataLoader(run_dir)
        except FileNotFoundError as e:
            QMessageBox.warning(self, "Error", str(e))
            return

        n = self.data.n_frames
        self.frame_spin.setMaximum(n - 1)
        self.slider.setMaximum(n - 1)
        self.lbl_total.setText(f"/ {n}")
        self.video_canvas.set_frames(n)
        self.data_canvas.set_frames(n)
        self.timeline.set_frames(n)

        # Load existing labels
        self.contact_regions = []
        labels = self.data.contact_prob
        if not np.all(np.isnan(labels)):
            in_region = False
            start = 0
            for i, v in enumerate(labels):
                if v == 1.0 and not in_region:
                    start = i
                    in_region = True
                elif v != 1.0 and in_region:
                    self.contact_regions.append([start, i - 1])
                    in_region = False
            if in_region:
                self.contact_regions.append([start, n - 1])

        self.undo_stack = []
        self.marking_start = -1
        self.current_frame = 0

        self.timeline.set_regions(self.contact_regions)
        self.data_canvas.plot_data(self.data, self.contact_regions)
        self._update_frame(0)
        self._update_label_count()
        self.lbl_status.setText(f"Loaded: {Path(run_dir).name}")
        self.setWindowTitle(f"Label Tool — {Path(run_dir).name}")

    # ── Navigation ──

    def _step(self, delta):
        self._go_to_frame(self.current_frame + delta)

    def _go_to_frame(self, frame):
        if self.data is None:
            return
        frame = max(0, min(frame, self.data.n_frames - 1))
        self._update_frame(frame)
        # If playing, restart timer with correct delay from this position
        if self.playing:
            self._schedule_next_frame()

    def _on_slider(self, val):
        self._update_frame(val)

    def _on_spin(self, val):
        self._update_frame(val)

    def _update_frame(self, frame):
        if self.data is None:
            return
        self.current_frame = frame
        self.slider.blockSignals(True)
        self.slider.setValue(frame)
        self.slider.blockSignals(False)
        self.frame_spin.blockSignals(True)
        self.frame_spin.setValue(frame)
        self.frame_spin.blockSignals(False)

        t = self.data.t_rel[frame] if frame < len(self.data.t_rel) else 0
        self.lbl_time.setText(f"  Time: {t:.2f}s")

        # Update video
        rgb = self.data.get_rgb_frame(frame)
        depth = self.data.get_depth_frame(frame)
        self.video_canvas.update_frames(rgb, depth, frame, t)

        # Update data cursor
        self.data_canvas.update_cursor(frame)

        # Update timeline
        self.timeline.set_frame(frame)

    def _advance_frame(self):
        if self.current_frame < self.data.n_frames - 1:
            self.current_frame += 1
            self._update_frame(self.current_frame)
            self._schedule_next_frame()
        else:
            self.playing = False
            self.play_timer.stop()

    def _schedule_next_frame(self):
        """Start timer with real inter-frame delay for the next frame."""
        if self.current_frame + 1 < self.data.n_frames:
            dt = self.data.t_rel[self.current_frame + 1] - self.data.t_rel[self.current_frame]
            delay_ms = max(1, int(dt * 1000 / self.playback_speed))
            self.play_timer.start(delay_ms)
        else:
            self.play_timer.stop()

    def _toggle_play(self):
        if self.data is None:
            return
        self.playing = not self.playing
        if self.playing:
            # Show current frame immediately, then schedule next
            self._update_frame(self.current_frame)
            self._schedule_next_frame()
        else:
            self.play_timer.stop()

    # ── Labeling ──

    def _mark_contact_start(self):
        if self.data is None:
            return
        self.marking_start = self.current_frame
        self.btn_mark_start.setStyleSheet(
            "background-color: #D32F2F; color: white; font-weight: bold; "
            "border: none; border-radius: 6px; padding: 4px 12px;")
        self.lbl_status.setText(
            f"Marking: start={self.current_frame}, navigate and press V")

    def _mark_contact_end(self):
        if self.data is None:
            return
        if self.marking_start < 0:
            self.lbl_status.setText("Press C first to mark contact start")
            return
        start = min(self.marking_start, self.current_frame)
        end = max(self.marking_start, self.current_frame)
        self._add_region(start, end)
        self.marking_start = -1
        self.btn_mark_start.setStyleSheet(
            "background-color: #1976D2; color: white; font-weight: bold; "
            "border: none; border-radius: 6px; padding: 4px 12px;")
        self.lbl_status.setText(f"Added contact region: [{start}, {end}]")

    def _on_region_created(self, start, end):
        self._add_region(start, end)
        self.lbl_status.setText(f"Added contact region: [{start}, {end}]")

    def _add_region(self, start, end):
        self.undo_stack.append([r[:] for r in self.contact_regions])
        new = [start, end]
        merged = []
        for s, e in self.contact_regions:
            if new[0] <= e + 1 and new[1] >= s - 1:
                new[0] = min(new[0], s)
                new[1] = max(new[1], e)
            else:
                merged.append([s, e])
        merged.append(new)
        self.contact_regions = sorted(merged, key=lambda r: r[0])
        self._refresh_labels()

    def _undo(self):
        if not self.undo_stack:
            self.lbl_status.setText("Nothing to undo")
            return
        self.contact_regions = self.undo_stack.pop()
        self._refresh_labels()
        self.lbl_status.setText("Undo")

    def _clear_all(self):
        if not self.contact_regions:
            return
        self.undo_stack.append([r[:] for r in self.contact_regions])
        self.contact_regions = []
        self._refresh_labels()
        self.lbl_status.setText("Cleared all labels")

    def _delete_region(self):
        if self.data is None:
            return
        f = self.current_frame
        for i, (s, e) in enumerate(self.contact_regions):
            if s <= f <= e:
                self.undo_stack.append([r[:] for r in self.contact_regions])
                self.contact_regions.pop(i)
                self._refresh_labels()
                self.lbl_status.setText(f"Deleted region [{s}, {e}]")
                return
        self.lbl_status.setText("No region at current frame")

    def _refresh_labels(self):
        self.timeline.set_regions(self.contact_regions)
        self.data_canvas.plot_data(self.data, self.contact_regions)
        self._update_label_count()

    def _update_label_count(self):
        n = sum(e - s + 1 for s, e in self.contact_regions)
        self.lbl_labels.setText(f"  Labels: {n} frames in {len(self.contact_regions)} regions")

    # ── Save ──

    def _save(self):
        if self.data is None:
            return

        # Build frame-level labels
        labels = np.zeros(self.data.n_frames, dtype=np.float32)
        for s, e in self.contact_regions:
            labels[s:e + 1] = 1.0

        run_dir = self.data.run_dir

        # Update dataset.npz
        npz_path = run_dir / "dataset.npz"
        npz = np.load(str(npz_path), allow_pickle=True)
        data = {k: npz[k] for k in npz.files}
        data["contact_prob"] = labels
        np.savez_compressed(str(npz_path), **data)

        # Update frame_index.csv
        csv_path = run_dir / "frame_index.csv"
        if csv_path.exists():
            rows = []
            with open(csv_path, "r") as f:
                reader = csv.DictReader(f)
                fieldnames = reader.fieldnames
                for i, row in enumerate(reader):
                    if i < len(labels):
                        row["contact_prob"] = f"{labels[i]:.1f}"
                    rows.append(row)
            with open(csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)

        # Update metadata.json
        meta_path = run_dir / "metadata.json"
        if meta_path.exists():
            with open(meta_path) as f:
                meta = json.load(f)
        else:
            meta = {}
        meta["has_contact_labels"] = True
        meta["num_contact_frames"] = int(np.sum(labels == 1.0))
        meta["num_contact_regions"] = len(self.contact_regions)
        meta["labeled_at"] = datetime.now().isoformat()
        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=2)

        n_contact = int(np.sum(labels == 1.0))
        self.lbl_status.setText(
            f"Saved! {n_contact} contact frames in {len(self.contact_regions)} regions")

    def closeEvent(self, event):
        if self.data:
            self.data.release()
        event.accept()


# ────────────────────────────────────────────────────────────────
# Entry point
# ────────────────────────────────────────────────────────────────

MATERIAL_STYLE = """
QMainWindow {
    background-color: #FAFAFA;
}
QWidget {
    font-family: "Segoe UI", "Roboto", "Noto Sans SC", sans-serif;
    font-size: 13px;
    color: #212121;
}
QPushButton {
    background-color: #FFFFFF;
    color: #212121;
    border: 1px solid #E0E0E0;
    border-radius: 6px;
    padding: 5px 14px;
    min-height: 22px;
}
QPushButton:hover {
    background-color: #F5F5F5;
    border: 1px solid #BDBDBD;
}
QPushButton:pressed {
    background-color: #EEEEEE;
}
QPushButton:disabled {
    color: #BDBDBD;
}
QSlider::groove:horizontal {
    height: 4px;
    background: #E0E0E0;
    border-radius: 2px;
}
QSlider::handle:horizontal {
    background: #1976D2;
    width: 16px;
    height: 16px;
    margin: -6px 0;
    border-radius: 8px;
}
QSlider::sub-page:horizontal {
    background: #90CAF9;
    border-radius: 2px;
}
QSpinBox {
    background: #FFFFFF;
    border: 1px solid #E0E0E0;
    border-radius: 4px;
    padding: 2px 6px;
    min-height: 22px;
}
QSpinBox:focus {
    border: 1px solid #1976D2;
}
QLabel {
    color: #424242;
}
"""


def main():
    parser = argparse.ArgumentParser(description="Contact labeling tool")
    parser.add_argument("run_dir", nargs="?", default=None,
                        help="Path to recording directory")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setStyleSheet(MATERIAL_STYLE)
    win = MainWindow(run_dir=args.run_dir)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
