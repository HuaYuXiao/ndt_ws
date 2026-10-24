#!/usr/bin/python3
"""
EMAT厚度解算节点。

订阅 /emat/waveform，从波形计算厚度，发布 /emat/thickness。

对应 EmatPenThicknessGauge-cpp 的 ProtocolDataProcessor::processWaveform
主机端计算路径（其 ExtractDelayService 为留空的桩，此处为实际实现）。

算法（自相关法，实测验证）：
  1. 截取 [slice_start, slice_end)（默认 250~2000；200 处仍有激励残留包）
  2. DC 去除（-127）→ RMS 包络（窗口 ~2.5 个激励周期，抑制载波纹波）
  3. 包络零均值自相关，取 [100, 1300] 样本滞后内最大峰
  4. d = lag / fs * v / 2 * 1000 (mm)
  5. 5 帧中值滤波抑制帧间抖动；包络峰值低于噪声底 → NaN（无回波）

实测（25mm 铝板，fs=40MHz）：自相关滞后稳定在 618-622 样本 → 25.0mm，
60/60 帧全部正确。旧的"双峰法"（两个最大峰间距）被激励残留包/模式转换
回波干扰，帧间跳变 18.8/25/43.9mm。

参数（ROS ~）：
    ~waveform_topic (str)   : 输入波形话题（默认 /emat/waveform）
    ~thickness_topic (str)  : 厚度输出话题（默认 /emat/thickness）
    ~sampling_rate (float)  : ADC采样率 Hz（默认 40000000；实测验证值，
                              1MHz 下同一波形解出 ~1000mm 不合理）
    ~speed_of_sound (float) : 材料声速 m/s（默认 3230，铝横波）
    ~slice_start (int)      : 分析起点（默认 250）
    ~slice_end (int)        : 分析终点（默认 2000）
    ~env_noise_floor (float): 包络峰值低于此值判无回波（默认 20；
                              回波帧实测 61-88，噪声 <15）
"""
import rospy
import threading
from collections import deque

import numpy as np

from emat.msg import EmatWaveform, EmatThickness


class EmatThicknessNode:
    def __init__(self):
        rospy.init_node('emat_thickness', anonymous=False)

        # ---- 参数 ----
        self.waveform_topic = rospy.get_param('~waveform_topic', '/emat/waveform')
        self.thickness_topic = rospy.get_param('~thickness_topic', '/emat/thickness')
        self.sampling_rate = float(rospy.get_param('~sampling_rate', 40000000.0))
        self.speed_of_sound = float(rospy.get_param('~speed_of_sound', 3230.0))
        self.slice_start = int(rospy.get_param('~slice_start', 250))
        self.slice_end = int(rospy.get_param('~slice_end', 2000))
        self.env_window = int(rospy.get_param('~envelope_window_samples', 30))
        self.env_noise_floor = float(rospy.get_param('~env_noise_floor', 20.0))

        # ---- 状态 ----
        self.lock = threading.Lock()
        self.latest_msg = None
        self.history = deque(maxlen=5)  # 有效厚度的中值滤波窗口

        # ---- ROS IO ----
        rospy.Subscriber(self.waveform_topic, EmatWaveform, self.waveform_cb, queue_size=1)
        self.pub = rospy.Publisher(self.thickness_topic, EmatThickness, queue_size=10)

        rospy.loginfo("EMAT测厚节点启动  fs=%.0f Hz  slice=[%d,%d)  v=%.0f m/s",
                      self.sampling_rate, self.slice_start, self.slice_end,
                      self.speed_of_sound)

        self.rate = rospy.Rate(30)
        self.spin()

    def waveform_cb(self, msg):
        with self.lock:
            self.latest_msg = msg

    def estimate(self, raw_data):
        """自相关法测厚，返回 mm 或 NaN（无效）。"""
        # Python3 的 rospy 将 uint8[] 反序列化为 bytes，需用 frombuffer 还原
        raw = np.frombuffer(raw_data, dtype=np.uint8).astype(np.float64)
        start = min(self.slice_start, len(raw))
        end = min(self.slice_end, len(raw))
        sig = raw[start:end] - 127.0
        n = len(sig)
        if n < 300:
            return float('nan')

        # RMS 包络
        w = max(1, self.env_window)
        sq = sig * sig
        cs = np.convolve(sq, np.ones(w) / w, mode='same')
        env = np.sqrt(cs)

        # 无回波判定
        if env.max() < self.env_noise_floor:
            return float('nan')

        # 零均值自相关，搜索 [100, 1300] 样本滞后 (2~52mm @ v=3230, fs=40MHz)
        e = env - env.mean()
        ac = np.correlate(e, e, mode='full')[n - 1:]
        lo, hi = 100, min(1300, len(ac) - 1)
        if hi <= lo:
            return float('nan')
        lag = int(np.argmax(ac[lo:hi + 1])) + lo

        return lag / self.sampling_rate * self.speed_of_sound / 2.0 * 1000.0

    def spin(self):
        while not rospy.is_shutdown():
            msg = None
            with self.lock:
                if self.latest_msg is not None:
                    msg = self.latest_msg
                    self.latest_msg = None

            out = EmatThickness()
            out.stamp = rospy.Time.now()
            if msg is not None:
                t = self.estimate(msg.raw_data)
                if np.isnan(t):
                    self.history.clear()  # 无回波立即清零，输出 NaN
                else:
                    self.history.append(t)
                if self.history:
                    out.thickness_mm = float(np.median(self.history))
                else:
                    out.thickness_mm = float('nan')
                out.stamp = msg.stamp
                self.pub.publish(out)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        EmatThicknessNode()
    except rospy.ROSInterruptException:
        pass
