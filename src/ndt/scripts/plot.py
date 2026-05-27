#!/usr/bin/python3
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import font_manager as fm

# Try to use common Chinese-capable fonts. If none are installed, matplotlib will
# fall back to the default; install one of the fonts below on your system to
# display Chinese correctly (see notes below).
mpl.rcParams['font.sans-serif'] = [
    'Noto Sans CJK SC',
    'SimHei',
    'WenQuanYi Zen Hei',
    'Microsoft YaHei',
    'AR PL UKai CN',
    'DejaVu Sans'
]
# Ensure minus sign is shown correctly
mpl.rcParams['axes.unicode_minus'] = False


def load_csv_data(csv_path):
    data = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            data.append([float(x) for x in row])
    return np.array(data), header

def plot_data(data, header):
    t = np.arange(data.shape[0])
    plt.figure(figsize=(12, 8))

    # 里程计
    plt.subplot(3, 1, 1)
    plt.plot(t, data[:,3], label='odom_x')
    plt.plot(t, data[:,4], label='odom_y')
    plt.plot(t, data[:,5], label='odom_z')
    plt.title('Drone Position')
    plt.legend()
    # 相对位置
    plt.subplot(3, 1, 2)
    plt.plot(t, data[:,0], label='rel_x')
    plt.plot(t, data[:,1], label='rel_y')
    plt.plot(t, data[:,2], label='rel_z')
    plt.title('Object Relative Position')
    plt.legend()
    # 绝对位置
    plt.subplot(3, 1, 3)
    plt.plot(t, data[:,6], label='abs_x')
    plt.plot(t, data[:,7], label='abs_y')
    plt.plot(t, data[:,8], label='abs_z')
    plt.title('Object Absolute Position')

    plt.legend()
    plt.tight_layout()
    plt.show()

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Plot CSV data produced by the NDT runs.')
    parser.add_argument('csv', nargs='?', help='Path to CSV file to visualize. If omitted, the latest in runs/ is used.')
    args = parser.parse_args()
    csv_path = args.csv

    print(f'Visualizing: {csv_path}')
    data, header = load_csv_data(csv_path)
    plot_data(data, header)

if __name__ == '__main__':
    main()
