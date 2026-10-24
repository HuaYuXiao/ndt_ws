#!/usr/bin/python3
"""
EMAT厚度解算节点。

订阅 /emat/waveform，从波形计算厚度，发布 /emat/thickness。

对应 EmatPenThicknessGauge-cpp 的 ProtocolDataProcessor::processWaveform
主机端计算路径（其 ExtractDelayService 为留空的桩，此处为实际实现）。

算法（自相关法，实测验证）：
  1. 自适应确定 slice_start（默认开启）：不同试件激励残留范围不同，
     固定起点会切进激励残留（污染自相关）或浪费窗口（丢回波）。
     RMS 包络上从激励主峰向后扫描，找到第一个"连续 hang 个样本低于
     阈值"的静默区段，其起点即激励残留结束位置；slice_start = 该点 +
     保护余量，夹在 [slice_start_min, slice_start_max]；找不到静默间隙
     （如薄试件回波紧贴激励残留）时回退固定 fallback_slice_start
  2. 截取 [分析起点, slice_end)（回退默认 250~2000；200 处仍有激励残留包）
  3. DC 去除（-127）→ RMS 包络（窗口 ~2.5 个激励周期，抑制载波纹波）
  4. 包络零均值自相关，取 [100, 1300] 样本滞后内最大峰。薄件真周期
     <100 样本（4mm≈99）会被下界挡掉：峰顶顶在 100 或落在谐波上
     （<300）时，放宽下界到 lag_lo_thin 二次搜索，候选须为局部峰且与
     pass-1 峰谐波一致
  5. 弱回波区（包络峰值低于 env_noise_floor 但高于 env_weak_floor）：
     前后半窗自相关滞后须与整窗一致，噪声随机峰不满足 → NaN
  6. d = lag / fs * v / 2 * 1000 (mm)
  7. 5 帧中值滤波抑制帧间抖动

注：slice_end 与自相关搜索上限仍为固定值，窗口须覆盖 ≥2 个回波；
更厚试件需同步增大 slice_end（如 4000）并放宽自相关滞后上限。

实测（25mm 铝板，fs=40MHz）：自相关滞后稳定在 618-622 样本 → 25.0mm，
60/60 帧全部正确。旧的"双峰法"（两个最大峰间距）被激励残留包/模式转换
回波干扰，帧间跳变 18.8/25/43.9mm。

参数（ROS ~）：
    ~waveform_topic (str)   : 输入波形话题（默认 /emat/waveform）
    ~thickness_topic (str)  : 厚度输出话题（默认 /emat/thickness）
    ~sampling_rate (float)  : ADC采样率 Hz（默认 40000000；实测验证值，
                              1MHz 下同一波形解出 ~1000mm 不合理）
    ~speed_of_sound (float) : 材料声速 m/s（默认 3230，铝横波）
    ~fallback_slice_start (int): 自适应失败时的固定回退起点（默认 250）
    ~slice_end (int)        : 分析终点（默认 2000）
    ~env_noise_floor (float): 包络峰值低于此值判无回波（默认 20；
                              回波帧实测 61-88，噪声 <15）
    ~adaptive_slice_start (bool)     : 自适应确定 slice_start（默认 True）；
                                       False 时使用固定 ~fallback_slice_start
    ~excitation_noise_mult (float)   : 激励结束阈值 = max(尾部包络中位数×此系数,
                                       env_noise_floor)（默认 4.0）
    ~excitation_hang_samples (int)   : 静默区段最小持续样本数（默认 80，
                                       ~2µs，应小于回波间隙以免跨到回波之后）
    ~excitation_guard_margin (int)   : 激励残留结束后再退让的样本数（默认 20）
    ~slice_start_min (int)           : 自适应 slice_start 下限（默认 200）
    ~slice_start_max (int)           : 自适应 slice_start 上限（默认 800，
                                       超过仍无静默间隙则回退固定值）
    ~lag_lo_thin (int)               : 薄件二次搜索的自相关滞后下界（默认 40，
                                       约 1.6mm；峰顶顶在 100 下界或谐波区时启用）
    ~env_weak_floor (float)          : 弱回波区下限（默认 10）：包络峰值低于
                                       env_noise_floor 时须高于此值才继续
                                       （实测弱回波串 16-18.5，纯噪声帧 <5）
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
        self.fallback_slice_start = int(rospy.get_param('~fallback_slice_start', 250))
        self.slice_end = int(rospy.get_param('~slice_end', 2000))
        self.env_window = int(rospy.get_param('~envelope_window_samples', 30))
        self.env_noise_floor = float(rospy.get_param('~env_noise_floor', 20.0))
        # 自适应 slice_start：不同试件激励残留范围不同，动态检测余振结束位置
        self.adaptive_slice_start = bool(rospy.get_param('~adaptive_slice_start', True))
        self.excitation_noise_mult = float(rospy.get_param('~excitation_noise_mult', 4.0))
        self.excitation_hang_samples = int(rospy.get_param('~excitation_hang_samples', 80))
        self.excitation_guard_margin = int(rospy.get_param('~excitation_guard_margin', 20))
        self.slice_start_min = int(rospy.get_param('~slice_start_min', 200))
        self.slice_start_max = int(rospy.get_param('~slice_start_max', 800))
        self.lag_lo_thin = int(rospy.get_param('~lag_lo_thin', 40))
        self.env_weak_floor = float(rospy.get_param('~env_weak_floor', 10.0))
        self._last_logged_start = None

        # ---- 状态 ----
        self.lock = threading.Lock()
        self.latest_msg = None
        self.history = deque(maxlen=5)  # 有效厚度的中值滤波窗口

        # ---- ROS IO ----
        rospy.Subscriber(self.waveform_topic, EmatWaveform, self.waveform_cb, queue_size=1)
        self.pub = rospy.Publisher(self.thickness_topic, EmatThickness, queue_size=10)

        rospy.loginfo("EMAT测厚节点启动  fs=%.0f Hz  slice=[%d,%d)  v=%.0f m/s  "
                      "自适应slice_start=%s",
                      self.sampling_rate, self.fallback_slice_start, self.slice_end,
                      self.speed_of_sound, self.adaptive_slice_start)

        self.rate = rospy.Rate(30)
        self.spin()

    def waveform_cb(self, msg):
        with self.lock:
            self.latest_msg = msg

    def find_slice_start(self, raw):
        """自适应确定 slice_start：检测激励主峰+余振的结束位置。

        波形开头是激励脉冲与换能器余振（幅度大），衰减后进入静默区，
        第一个回波到达前存在静默间隙。不同试件的余振范围不同，固定
        分析起点会切进激励残留（污染自相关）或浪费窗口（丢回波）。

        方法：RMS 包络上从激励主峰向后扫描，找到第一个"连续 hang 个
        样本低于阈值"的区段，其起点即余振结束位置；slice_start =
        该点 + 保护余量，并夹在 [slice_start_min, slice_start_max]。
        阈值取 max(尾部包络中位数 × excitation_noise_mult,
        env_noise_floor)，不依赖激励峰值（逐帧波动大）。
        找不到静默间隙（如薄试件回波紧贴余振）时回退固定 fallback_slice_start。
        """
        n = len(raw)
        sig = raw - 127.0
        w = max(1, self.env_window)
        cs = np.convolve(sig * sig, np.ones(w) / w, mode='same')
        env = np.sqrt(cs)

        p0 = int(np.argmax(env[:600]))  # 激励主峰必在波形最前部
        tail_start = min(4000, n // 2)
        noise = float(np.median(env[tail_start:]))
        thr = max(noise * self.excitation_noise_mult, self.env_noise_floor)

        limit = min(n, self.slice_start_max + self.excitation_hang_samples)
        hang = 0
        residue_end = p0
        for i in range(p0, limit):
            if env[i] < thr:
                hang += 1
                if hang >= self.excitation_hang_samples:
                    residue_end = i - self.excitation_hang_samples + 1
                    break
            else:
                hang = 0
                residue_end = i
        else:
            return self.fallback_slice_start  # 未找到静默间隙，回退固定值

        start = residue_end + self.excitation_guard_margin
        return int(max(self.slice_start_min, min(start, self.slice_start_max)))

    def estimate(self, raw_data):
        """自相关法测厚，返回 mm 或 NaN（无效）。"""
        # Python3 的 rospy 将 uint8[] 反序列化为 bytes，需用 frombuffer 还原
        raw = np.frombuffer(raw_data, dtype=np.uint8).astype(np.float64)
        if self.adaptive_slice_start:
            start = self.find_slice_start(raw)
            if self._last_logged_start is None or abs(start - self._last_logged_start) >= 5:
                self._last_logged_start = start
                rospy.loginfo("自适应 slice_start=%d (固定回退=%d)",
                              start, self.fallback_slice_start)
        else:
            start = self.fallback_slice_start
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

        # 无回波判定 + 自相关搜索（含薄件支持）
        e = env - env.mean()
        ac = np.correlate(e, e, mode='full')[n - 1:]
        lo, hi = 100, min(1300, len(ac) - 1)
        if hi <= lo:
            return float('nan')
        lag = int(np.argmax(ac[lo:hi + 1])) + lo

        weak = env.max() < self.env_noise_floor
        # 薄件真周期 <100 样本(如4mm≈99): pass-1 峰顶会顶在下界或落在谐波上
        if (lag == lo or lag < 3 * lo) and self.lag_lo_thin < hi:
            lag_thin = int(np.argmax(ac[self.lag_lo_thin:hi + 1])) + self.lag_lo_thin
            local_max = (ac[lag_thin] > ac[max(lag_thin - 3, 0)] and
                         ac[lag_thin] > ac[min(lag_thin + 3, len(ac) - 1)])
            tol = max(5, int(0.08 * lag))
            harmonic = (abs(lag - 2 * lag_thin) <= tol or
                        abs(lag - 3 * lag_thin) <= tol)
            if local_max and ac[lag_thin] > ac[lag] and (lag == lo or harmonic):
                lag = lag_thin

        # 弱回波区: 包络须高于弱回波下限, 且前后半窗自相关滞后与整窗一致
        if weak:
            if env.max() < self.env_weak_floor:
                return float('nan')
            half = n // 2
            half_lags = []
            for seg in (env[:half], env[half:]):
                es = seg - seg.mean()
                acs = np.correlate(es, es, mode='full')[len(es) - 1:]
                hilo = min(1300, len(acs) - 1)
                if hilo > self.lag_lo_thin:
                    half_lags.append(int(np.argmax(acs[self.lag_lo_thin:hilo + 1]))
                                     + self.lag_lo_thin)
            tol = max(6, int(0.1 * lag))
            if not any(abs(lag - lh) <= tol for lh in half_lags):
                return float('nan')

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
