#!/usr/bin/python3
"""
EMAT波形特征提取节点。

订阅 /emat/waveform，执行信号处理流水线，发布 /emat/features 和 /emat/envelope。

信号处理流程：
  1. DC偏移去除（减去127）
  2. Hilbert变换 -> 包络
  3. 截取包络有效区间 [slice_start, slice_end)
  4. 低通滤波包络（Butterworth，零相位sosfiltfilt）
  5. 特征提取：能量、峰值、到达时间、频谱重心、峰度、相位、分频段能量
  6. 厚度估计：d = (v * t_arrival) / 2

参数（ROS）：
    ~waveform_topic (str)      : 输入话题（默认 /emat/waveform）
    ~features_topic (str)      : 特征输出话题（默认 /emat/features）
    ~envelope_topic (str)      : 包络输出话题（默认 /emat/envelope）
    ~arrival_threshold (float) : 到达时间检测阈值，占峰值比例（默认 0.1）
    ~speed_of_sound (float)    : 默认声速 m/s（默认 3240）
    ~sampling_rate (float)     : ADC采样率 Hz（默认 1000000）
    ~slice_start (int)         : 包络截取起始采样点（默认 200）
    ~slice_end (int)           : 包络截取结束采样点（默认 1000）
    ~lp_cutoff (float)         : 包络低通截止频率 Hz（默认 10）
    ~lp_order (int)            : 低通滤波器阶数（默认 4）
"""
import rospy
import threading
import numpy as np
from scipy import signal as sig
from scipy.stats import kurtosis as scipy_kurtosis
from emat.msg import EmatWaveform, EmatFeatures, EmatEnvelope


class EmatFeatureExtractor:
    def __init__(self):
        rospy.init_node('emat_feature_extractor', anonymous=False)

        # ---- 参数 ----
        self.waveform_topic = rospy.get_param('~waveform_topic', '/emat/waveform')
        self.features_topic = rospy.get_param('~features_topic', '/emat/features')
        self.envelope_topic = rospy.get_param('~envelope_topic', '/emat/envelope')
        self.arrival_threshold = float(rospy.get_param('~arrival_threshold', 0.1))
        self.speed_of_sound = float(rospy.get_param('~speed_of_sound', 3240.0))
        self.sampling_rate = float(rospy.get_param('~sampling_rate', 1000000.0))
        self.slice_start = int(rospy.get_param('~slice_start', 200))
        self.slice_end = int(rospy.get_param('~slice_end', 1000))
        lp_cutoff = float(rospy.get_param('~lp_cutoff', 10.0))
        lp_order = int(rospy.get_param('~lp_order', 256))

        # ---- 低通FIR滤波器系数（预计算，匹配MATLAB lowpass行为）----
        nyquist = self.sampling_rate / 2.0
        if lp_cutoff >= nyquist:
            rospy.logwarn("低通截止频率 %.0f Hz >= Nyquist %.0f Hz，禁用低通滤波",
                           lp_cutoff, nyquist)
            self.lp_taps = None
        else:
            self.lp_taps = sig.firwin(lp_order + 1, lp_cutoff, fs=self.sampling_rate,
                                       window='hamming')
            rospy.loginfo("低通FIR滤波: cutoff=%.0f Hz, taps=%d", lp_cutoff, lp_order + 1)

        # ---- 状态 ----
        self.lock = threading.Lock()
        self.latest_msg = None

        # ---- ROS IO ----
        rospy.Subscriber(self.waveform_topic, EmatWaveform, self.waveform_cb, queue_size=1)
        self.feat_pub = rospy.Publisher(self.features_topic, EmatFeatures, queue_size=10)
        self.env_pub = rospy.Publisher(self.envelope_topic, EmatEnvelope, queue_size=10)

        rospy.loginfo("EMAT特征提取器已启动  fs=%.0f Hz  slice=[%d,%d)  lp_cutoff=%.0f Hz",
                       self.sampling_rate, self.slice_start, self.slice_end, lp_cutoff)

        self.rate = rospy.Rate(30)
        self.spin()

    def waveform_cb(self, msg):
        with self.lock:
            self.latest_msg = msg

    def process_waveform(self, msg):
        """处理单帧波形，返回 (EmatFeatures, EmatEnvelope) 或 (None, None)。"""
        raw = np.frombuffer(msg.raw_data, dtype=np.uint8).astype(np.float32)
        if raw.size < 10:
            rospy.logwarn_throttle(5.0, "波形数据过短 (%d samples)，跳过", raw.size)
            return None, None

        # ---- Stage 1: DC偏移去除 ----
        sig_data = raw - 127.0

        # ---- Stage 2: Hilbert变换（在完整信号上）----
        analytic = sig.hilbert(sig_data)
        envelope_full = np.abs(analytic)

        # ---- Stage 3: 截取有效区间 ----
        start = min(self.slice_start, len(envelope_full))
        end = min(self.slice_end, len(envelope_full))
        if end - start < 10:
            rospy.logwarn_throttle(5.0, "截取区间过短 (%d samples)，跳过", end - start)
            return None, None
        envelope = envelope_full[start:end]

        # ---- Stage 4: 低通滤波包络 ----
        if self.lp_taps is not None:
            envelope = sig.filtfilt(self.lp_taps, 1.0, envelope)

        # ---- Stage 5: 特征提取 ----
        peak_amplitude = float(np.max(envelope))

        if peak_amplitude < 1e-6:
            rospy.logdebug("包络峰值接近零，跳过")
            return None, None

        # 能量（基于截取区间的原始信号）
        sliced_raw = sig_data[start:end]
        energy = float(np.sum(sliced_raw ** 2))

        # 到达时间（首波阈值检测）
        thresh_val = self.arrival_threshold * peak_amplitude
        arrival_indices = np.where(envelope > thresh_val)[0]
        if len(arrival_indices) > 0:
            arrival_time = float(arrival_indices[0]) / self.sampling_rate
        else:
            arrival_time = 0.0

        # 频谱重心（基于截取区间的原始信号）
        fft_mag = np.abs(np.fft.rfft(sliced_raw))
        freqs = np.fft.rfftfreq(len(sliced_raw), d=1.0 / self.sampling_rate)
        mag_sum = np.sum(fft_mag)
        if mag_sum > 0:
            spectral_centroid = float(np.sum(freqs * fft_mag) / mag_sum)
        else:
            spectral_centroid = 0.0

        # 峰度（基于截取区间的原始信号）
        kurt = float(scipy_kurtosis(sliced_raw))

        # 峰值处相位（从完整analytic信号中截取）
        inst_phase_full = np.angle(analytic)
        sliced_phase = inst_phase_full[start:end]
        peak_idx = int(np.argmax(envelope))
        phase = float(sliced_phase[peak_idx])

        # 分频段能量（8段，基于截取区间的原始信号）
        nyquist = self.sampling_rate / 2.0
        band_width = nyquist / 8.0
        fft_power = fft_mag ** 2
        band_energies = []
        for i in range(8):
            low = i * band_width
            high = (i + 1) * band_width
            mask = (freqs >= low) & (freqs < high)
            band_energies.append(float(np.sum(fft_power[mask])))

        # ---- Stage 6: 厚度估计 ----
        sos_val = float(msg.speed_of_voice) if msg.speed_of_voice > 0 else self.speed_of_sound
        thickness_estimate = (sos_val * arrival_time) / 2.0 * 1000.0  # m -> mm

        # ---- 构造特征消息 ----
        feat = EmatFeatures()
        feat.stamp = msg.stamp
        feat.energy = energy
        feat.peak_amplitude = peak_amplitude
        feat.arrival_time = arrival_time
        feat.spectral_centroid = spectral_centroid
        feat.kurtosis = kurt
        feat.phase = phase
        feat.band_energies = band_energies
        feat.thickness_estimate = thickness_estimate

        # ---- 构造包络消息（低通滤波后的包络）----
        env_msg = EmatEnvelope()
        env_msg.stamp = msg.stamp
        env_msg.envelope = envelope.tolist()
        env_msg.sample_count = len(envelope)
        env_msg.sampling_rate = self.sampling_rate

        return feat, env_msg

    def spin(self):
        while not rospy.is_shutdown():
            msg = None
            with self.lock:
                if self.latest_msg is not None:
                    msg = self.latest_msg
                    self.latest_msg = None

            if msg is not None:
                feat, env_msg = self.process_waveform(msg)
                if feat is not None:
                    self.feat_pub.publish(feat)
                if env_msg is not None:
                    self.env_pub.publish(env_msg)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        EmatFeatureExtractor()
    except rospy.ROSInterruptException:
        pass
