#!/usr/bin/env python3
"""Plot appended train/val loss curves from runs/*/training_log.csv.

Usage:
    python plot_loss.py                # append every run by index, save loss_*.png
    python plot_loss.py 3 4            # append runs 3 and 4 in that order -> loss_3_4.png
    python plot_loss.py 3 4 --log      # y axis in log scale
    python plot_loss.py 3 4 --show     # also open the figure window
    python plot_loss.py 3 4 -o loss_3_4.pdf   # save to explicit path (extension picks format,
                                               # no extension defaults to .pdf)
"""
import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

TRAIN_COLOR = '#2a78d6'
VAL_COLOR = '#eb6834'
FONT_SIZE = 10.5

EPOCH_TICK = 10  # x-axis tick interval (epochs)


def find_runs(runs_dir, names):
    """Return sorted (name, csv_path) pairs for the requested runs."""
    if names:
        out = []
        for n in names:
            csv_path = runs_dir / n / 'training_log.csv'
            if not csv_path.is_file():
                print(f"ERROR: no training_log.csv in {runs_dir / n}",
                      file=sys.stderr)
                sys.exit(1)
            out.append((n, csv_path))
        return out

    out = sorted(
        ((d.name, d / 'training_log.csv')
         for d in runs_dir.iterdir()
         if d.is_dir() and (d / 'training_log.csv').is_file()),
        key=lambda x: (0, int(x[0])) if x[0].isdigit() else (1, x[0]))
    if not out:
        print(f"ERROR: no training_log.csv found under {runs_dir}",
              file=sys.stderr)
        sys.exit(1)
    return out


def read_log(csv_path):
    """Read training_log.csv -> (epochs, train_loss, val_loss)."""
    epochs, train, val = [], [], []
    with open(csv_path, newline='') as f:
        for row in csv.DictReader(f):
            epochs.append(int(row['epoch']))
            train.append(float(row['train_loss']))
            val.append(float(row['val_loss']))
    return epochs, train, val


def main():
    runs_dir = Path(__file__).resolve().parent
    plt.rcParams.update({
        'font.family': ['Times New Roman', 'SimSun'],
        'font.size': FONT_SIZE,
        'axes.labelsize': FONT_SIZE,
        'xtick.labelsize': FONT_SIZE,
        'ytick.labelsize': FONT_SIZE,
        'legend.fontsize': FONT_SIZE,
    })
    parser = argparse.ArgumentParser(
        description='Plot appended train/val loss curves from runs/*/training_log.csv')
    parser.add_argument('runs', nargs='*',
                        help='Run folder names to plot (default: all folders with a training_log.csv)')
    parser.add_argument('-o', '--output',
                        help='Save figure to this path (extension picks format, no extension defaults to .pdf; without -o a PNG is saved next to this script)')
    parser.add_argument('--show', action='store_true',
                        help='Also open the figure window after saving')
    parser.add_argument('--log', action='store_true',
                        help='Log-scale y axis (axis label is marked "Log scale")')
    parser.add_argument('--dpi', type=int, default=300, help='Save DPI (default 300)')
    args = parser.parse_args()

    runs = find_runs(runs_dir, args.runs)
    fig, ax = plt.subplots(figsize=(8, 5), dpi=args.dpi)
    max_epoch = 0
    plotted = []
    all_epochs, all_train, all_val = [], [], []
    for name, csv_path in runs:
        epochs, train, val = read_log(csv_path)
        if not epochs:
            print(f"WARN: {csv_path} has no data rows, skipped", file=sys.stderr)
            continue
        epoch_offset = max_epoch + 1 - epochs[0]
        appended_epochs = [epoch + epoch_offset for epoch in epochs]
        all_epochs.extend(appended_epochs)
        all_train.extend(train)
        all_val.extend(val)
        max_epoch = appended_epochs[-1]
        plotted.append(f"run {name}: epochs {appended_epochs[0]}-{appended_epochs[-1]} "
                       f"({len(epochs)} rows), "
                       f"train {train[0]:.1f}->{train[-1]:.1f}, "
                       f"val {val[0]:.1f}->{val[-1]:.1f}")

    if not all_epochs:
        print("ERROR: no training data to plot", file=sys.stderr)
        sys.exit(1)

    ax.plot(all_epochs, all_train, color=TRAIN_COLOR, linestyle='-',
            linewidth=1.5, label='Train')
    ax.plot(all_epochs, all_val, color=VAL_COLOR, linestyle='--',
            linewidth=1.5, label='Validation')
    ax.set_xlabel('Epoch', fontsize=FONT_SIZE)
    ax.set_ylabel('Loss (Log scale)' if args.log else 'Loss',
                  fontsize=FONT_SIZE)
    ax.xaxis.set_major_locator(MultipleLocator(EPOCH_TICK))
    ax.set_xlim(0, max_epoch)
    if args.log:
        ax.set_yscale('log')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(fontsize=FONT_SIZE, loc='best', ncol=2)
    fig.tight_layout()

    for line in plotted:
        print(line)

    if args.output:
        out_path = Path(args.output)
        if not out_path.suffix:
            out_path = out_path.with_suffix('.pdf')
    else:
        suffix = '_'.join(n for n, _ in runs) if args.runs else 'all'
        out_path = runs_dir / f'loss_{suffix}.png'
    fig.savefig(out_path, dpi=args.dpi, bbox_inches='tight')
    print(f"Saved: {out_path}")
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
