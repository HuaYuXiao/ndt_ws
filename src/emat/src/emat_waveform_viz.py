#!/usr/bin/env python3
"""EMAT waveform real-time visualization node (Qt5, single plot)."""
import rospy
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import threading
from matplotlib.animation import FuncAnimation

from emat.msg import EmatWaveform


class EmatWaveformViz:
    def __init__(self):
        rospy.init_node('emat_waveform_viz', anonymous=True)
        self.fig_title = rospy.get_param('~title', 'EMAT Waveform')
        self.lock = threading.Lock()
        self.latest_data = None
        self.frame_count = 0

        # Single plot: DC-removed signal
        self.fig, self.ax = plt.subplots(figsize=(12, 5))
        self.fig.suptitle(self.fig_title, fontsize=14)
        self.ax.set_title('Signal (DC Removed)')
        self.ax.set_xlabel('Sample')
        self.ax.set_ylabel('Amplitude')
        self.ax.grid(True, alpha=0.3)
        self.line_signal, = self.ax.plot([], [], 'r-', linewidth=0.5)
        self.info_text = self.fig.text(0.02, 0.02, '', fontsize=9,
                                       family='monospace', verticalalignment='bottom')
        plt.tight_layout()

        self.sub = rospy.Subscriber('/emat/waveform', EmatWaveform, self.waveform_cb)

        # Qt timer-based update (20 Hz)
        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self.update_plot)
        self.timer.start()

        rospy.loginfo('EMAT Waveform Visualizer started (Qt5 interactive)')
        rospy.loginfo('Subscribing to /emat/waveform')

    def waveform_cb(self, msg):
        with self.lock:
            self.latest_data = msg
            self.frame_count += 1

    def update_plot(self):
        with self.lock:
            if self.latest_data is None:
                return
            msg = self.latest_data
        raw = np.frombuffer(msg.raw_data, dtype=np.uint8).astype(np.float64)
        signal = raw - 127.0
        N = len(signal)
        x = np.arange(N)
        self.line_signal.set_data(x, signal)
        self.ax.set_xlim(0, N)
        max_amp = max(np.max(np.abs(signal)), 10)
        self.ax.set_ylim(-max_amp * 1.1, max_amp * 1.1)
        rms = np.sqrt(np.mean(signal ** 2))
        info = ('Samples: %d  |  RMS: %.1f  |  Material: %s'
                '  |  Speed: %d m/s  |  Frame: %d') % (
                    N, rms, msg.material, msg.speed_of_voice, self.frame_count)
        self.info_text.set_text(info)

    def run(self):
        plt.show()


if __name__ == '__main__':
    try:
        viz = EmatWaveformViz()
        viz.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
