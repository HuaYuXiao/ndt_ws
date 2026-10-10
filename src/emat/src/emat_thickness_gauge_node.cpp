#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <libusb-1.0/libusb.h>

#include "emat/EmatWaveform.h"

namespace {

struct Config {
    int read_interval_ms = 50;  // 50ms节奏下固件干净帧率实测92%; 100ms节奏仅20-30%
    int waveform_average_n = 4;  // 官方"平均次数"软件复刻: 采集N帧对齐平均后发布1帧 (1=直接发布)
    int warmup_requests = 3;    // 复位/上电后设备前2-3次请求不响应, 用牺牲性请求跳过预热窗口
    int sat_median_threshold = 40;  // |signal|中位数超过该值判为饱和帧(ADC满量程噪声)
    double quiet_std_threshold = 10.0;  // 静默区[3500,7000]std阈值: 半饱和帧(满幅噪声包稀疏分布)
                                        // 中位数仅5-13但静默区std>40, 干净帧~1.7 (25mm试件窗口)
    double default_speed = 3230.0;
    uint16_t vid = 0x1a86;
    uint16_t pid_emat = 0x55eb;     // 正常工作模式
    uint16_t pid_bootrom = 0x55e0;  // bootrom/ISP 模式 (错误状态)
    int vendor_iface = 2;
    uint8_t ep_out = 0x06;
    uint8_t ep_in = 0x86;
    int chunk_delay_ms = 30;        // CH346C命令到ADC数据准备的最小延迟（ms）
    int read_timeout_ms = 500;
    int write_timeout_ms = 200;     // USB写超时（ms），写操作不应需要长时间
    int reconnect_delay_s = 3;      // 重连等待秒数
    int max_startup_retries = 5;    // 启动时快速重试次数, 之后由后台重连线程接管
    int max_consecutive_failures = 5; // 连续失败 N 次后触发重连
    int max_chunk_retries = 2;      // 单chunk的最大重试次数
} cfg;

libusb_context* g_ctx = nullptr;
libusb_device_handle* g_handle = nullptr;
std::mutex g_usb_mutex;
std::atomic<bool> g_device_open{false};
std::atomic<int> g_consecutive_failures{0};
std::atomic<bool> g_running{true};

ros::Publisher* g_pub_waveform = nullptr;

std::vector<uint8_t> g_last_frame;  // 上一帧波形, 用于极性对齐
std::vector<std::vector<uint8_t>> g_batch;  // 平均模式下的待平均干净帧

uint8_t crc8(const uint8_t* data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = ((crc << 1) ^ 0x07) & 0xFF;
            else crc = (crc << 1) & 0xFF;
        }
    }
    return crc;
}

std::vector<uint8_t> build_command(uint8_t func, const std::vector<uint8_t>& payload = {}) {
    std::vector<uint8_t> pkt = {0xAB, 0x00, 0x01, func};
    pkt.push_back((payload.size() >> 8) & 0xFF);
    pkt.push_back(payload.size() & 0xFF);
    pkt.insert(pkt.end(), payload.begin(), payload.end());
    pkt.push_back(crc8(pkt.data(), (int)pkt.size()));
    return pkt;
}

// 检测设备是否存在，并区分正常模式和 bootrom 模式
// 返回: 0 = 未找到, 1 = 正常 (55eb), 2 = bootrom (55e0)
int detect_device(libusb_context* ctx) {
    libusb_device** devs;
    ssize_t cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0) return 0;
    int result = 0;
    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devs[i], &desc) == 0) {
            if (desc.idVendor == cfg.vid && desc.idProduct == cfg.pid_emat) {
                result = 1;
                break;
            } else if (desc.idVendor == cfg.vid && desc.idProduct == cfg.pid_bootrom) {
                result = 2;
                break;
            }
        }
    }
    libusb_free_device_list(devs, 1);
    return result;
}

// 关闭当前设备连接 (线程安全)
void close_device_internal() {
    if (g_handle) {
        libusb_release_interface(g_handle, cfg.vendor_iface);
        libusb_close(g_handle);
        g_handle = nullptr;
    }
    if (g_ctx) {
        libusb_exit(g_ctx);
        g_ctx = nullptr;
    }
    g_device_open = false;
}

void close_device() {
    std::lock_guard<std::mutex> lock(g_usb_mutex);
    close_device_internal();
}

// 打开设备连接，返回 true/false
// 调用前不需要持有锁，内部会加锁
bool open_device() {
    std::lock_guard<std::mutex> lock(g_usb_mutex);

    // 确保先清理旧连接
    close_device_internal();

    // 初始化 libusb
    if (libusb_init(&g_ctx) < 0) {
        ROS_ERROR("libusb_init failed");
        return false;
    }

    // 检测设备状态
    int dev_status = detect_device(g_ctx);
    if (dev_status == 0) {
        ROS_WARN("EMAT device not found (VID=0x%04X). Check USB connection.", cfg.vid);
        libusb_exit(g_ctx);
        g_ctx = nullptr;
        return false;
    }
    if (dev_status == 2) {
        ROS_ERROR("EMAT device in BOOTROM mode (PID=0x%04X)! "
                  "Physically unplug and replug the USB cable, then restart.",
                  cfg.pid_bootrom);
        libusb_exit(g_ctx);
        g_ctx = nullptr;
        return false;
    }

    // 打开设备
    libusb_device** devs;
    ssize_t cnt = libusb_get_device_list(g_ctx, &devs);
    if (cnt < 0) {
        libusb_exit(g_ctx);
        g_ctx = nullptr;
        return false;
    }
    libusb_device* target = nullptr;
    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devs[i], &desc) == 0) {
            if (desc.idVendor == cfg.vid && desc.idProduct == cfg.pid_emat) {
                target = devs[i];
                libusb_ref_device(target);
                break;
            }
        }
    }
    libusb_free_device_list(devs, 1);
    if (!target) {
        libusb_exit(g_ctx);
        g_ctx = nullptr;
        return false;
    }

    if (libusb_open(target, &g_handle) < 0) {
        ROS_ERROR("libusb_open failed. Permission denied? Try: sudo roslaunch ...");
        libusb_unref_device(target);
        libusb_exit(g_ctx);
        g_handle = nullptr;
        g_ctx = nullptr;
        return false;
    }
    libusb_unref_device(target);

    // detach kernel driver on Interface 2
    if (libusb_kernel_driver_active(g_handle, cfg.vendor_iface) == 1) {
        ROS_INFO("Detaching kernel driver from Interface %d", cfg.vendor_iface);
        if (libusb_detach_kernel_driver(g_handle, cfg.vendor_iface) != 0) {
            ROS_WARN("Failed to detach kernel driver, continuing anyway...");
        }
    }

    // claim Interface 2
    if (libusb_claim_interface(g_handle, cfg.vendor_iface) < 0) {
        ROS_ERROR("Failed to claim Interface %d. Another process may be using it.", cfg.vendor_iface);
        libusb_close(g_handle);
        g_handle = nullptr;
        libusb_exit(g_ctx);
        g_ctx = nullptr;
        return false;
    }

    g_device_open = true;
    g_consecutive_failures = 0;
    ROS_INFO("EMAT device opened (IF%d, EP 0x%02X/0x%02X)", cfg.vendor_iface, cfg.ep_out, cfg.ep_in);
    return true;
}

bool usb_write(const std::vector<uint8_t>& data) {
    if (!g_device_open) return false;
    int actual = 0;
    int rc = libusb_bulk_transfer(g_handle, cfg.ep_out,
                                  const_cast<uint8_t*>(data.data()),
                                  (int)data.size(), &actual, cfg.write_timeout_ms);
    if (rc == LIBUSB_ERROR_NO_DEVICE) {
        ROS_WARN("USB write: device disconnected");
        g_device_open = false;
        return false;
    }
    if (rc != 0 || actual != (int)data.size()) {
        // 瞬时错误(超时/IO/stall): 重试一次; 端点stall由重连的端口复位统一清除
        if (rc == LIBUSB_ERROR_PIPE || rc == LIBUSB_ERROR_TIMEOUT || rc == LIBUSB_ERROR_IO) {
            ROS_WARN("USB write: transient error (rc=%d), retrying...", rc);
            rc = libusb_bulk_transfer(g_handle, cfg.ep_out,
                                      const_cast<uint8_t*>(data.data()),
                                      (int)data.size(), &actual, cfg.write_timeout_ms);
            if (rc == 0 && actual == (int)data.size()) return true;
        }
        ROS_WARN("USB write: failed (rc=%d), marking disconnected", rc);
        g_device_open = false;
        return false;
    }
    return true;
}

std::vector<uint8_t> usb_read(int timeout_ms = 500) {
    if (!g_device_open) return {};
    // 单次大缓冲读: 分块小读(512B/200ms)在EMI状态下会在分块间隙超时,
    // 取消IN URB会破坏CH346上传状态, 之后设备对所有请求只回空包(ZLP僵死)
    uint8_t buf[9000];
    int actual = 0;
    int rc = libusb_bulk_transfer(g_handle, cfg.ep_in, buf, sizeof(buf),
                                  &actual, timeout_ms);
    if (rc == LIBUSB_ERROR_NO_DEVICE) {
        ROS_WARN("USB read: device disconnected");
        g_device_open = false;
        return {};
    }
    if (rc == LIBUSB_ERROR_TIMEOUT || rc == LIBUSB_ERROR_IO) {
        // 瞬时EMI干扰: 返回空, 由请求层重试; 不在此反复读(避免取消URB)
        return {};
    }
    if (rc < 0) {
        // 端点stall等: 置断开, 由重连的端口复位清除
        ROS_WARN("USB read: error (rc=%d), marking disconnected", rc);
        g_device_open = false;
        return {};
    }
    return std::vector<uint8_t>(buf, buf + actual);
}

void publish_waveform(std::vector<uint8_t> data) {
    // 固件相邻采集的激励极性交替, 与上一帧反相时翻转, 保持显示稳定
    if (!g_last_frame.empty() && g_last_frame.size() == data.size()) {
        int64_t dot = 0;
        for (size_t i = 0; i < data.size(); i++) {
            dot += (int64_t)((int)data[i] - 127) * ((int)g_last_frame[i] - 127);
        }
        if (dot < 0) {
            for (auto& v : data) v = (uint8_t)std::max(0, 254 - (int)v);
        }
    }
    g_last_frame = data;

    emat::EmatWaveform msg;
    msg.stamp = ros::Time::now();
    msg.raw_data = data;
    g_pub_waveform->publish(msg);
}

// 帧质量判据:
// 1) |signal|中位数: 全饱和帧接近满量程(实测>79), 干净帧<15
// 2) 静默区std: 半饱和帧(满幅噪声包稀疏散布)中位数只有5-13, 但静默区std>40; 干净帧~1.7
bool is_saturated(const std::vector<uint8_t>& data) {
    std::vector<double> abs_sig;
    abs_sig.reserve(data.size());
    for (uint8_t v : data) abs_sig.push_back(std::abs((double)v - 127.0));
    size_t mid = abs_sig.size() / 2;
    std::nth_element(abs_sig.begin(), abs_sig.begin() + mid, abs_sig.end());
    if (abs_sig[mid] > cfg.sat_median_threshold) return true;

    const size_t qs = 3500, qe = std::min<size_t>(7000, data.size());
    double qsum = 0, qsum2 = 0;
    size_t qn = qe - qs;
    for (size_t i = qs; i < qe; i++) {
        double v = (double)data[i] - 127.0;
        qsum += v;
        qsum2 += v * v;
    }
    double qmean = qsum / qn;
    double qstd = std::sqrt(std::max(0.0, qsum2 / qn - qmean * qmean));
    return qstd > cfg.quiet_std_threshold;
}

// 软件平均 (官方"平均次数"的复刻):
// 1) 极性对齐到 batch 首帧 (固件激励极性逐次交替, 不先对齐会互相抵消)
// 2) 时间对齐: 激励区 [0,400) 互相关修正触发相位抖动 (整数样本平移)
// 3) 整数域平均+四舍五入 (官方设备内部平均后输出仍是 8-bit 整数)
void average_and_publish() {
    const size_t n = g_batch.size();
    const size_t L = g_batch[0].size();

    std::vector<std::vector<int>> sigs(n, std::vector<int>(L));
    for (size_t k = 0; k < n; k++) {
        int64_t dot = 0;
        for (size_t i = 0; i < L; i++) {
            dot += (int64_t)((int)g_batch[k][i] - 127) * ((int)g_batch[0][i] - 127);
        }
        int sign = (dot >= 0) ? 1 : -1;
        for (size_t i = 0; i < L; i++) sigs[k][i] = sign * ((int)g_batch[k][i] - 127);
    }

    std::vector<int> shifts(n, 0);
    for (size_t k = 1; k < n; k++) {
        int64_t best = INT64_MIN;
        int best_s = 0;
        for (int s = -24; s <= 24; s++) {
            int64_t c = 0;
            for (int i = 0; i < 400; i++) {
                int j = i + s;
                if (j < 0 || j >= (int)L) continue;
                c += (int64_t)sigs[k][j] * sigs[0][i];
            }
            if (c > best) { best = c; best_s = s; }
        }
        shifts[k] = best_s;
    }

    std::vector<int> sum(L, 0), cnt(L, 0);
    for (size_t k = 0; k < n; k++) {
        for (size_t i = 0; i < L; i++) {
            int j = (int)i - shifts[k];
            if (j < 0 || j >= (int)L) continue;
            sum[i] += sigs[k][j];
            cnt[i]++;
        }
    }

    std::vector<uint8_t> out(L, 127);
    for (size_t i = 0; i < L; i++) {
        if (cnt[i] == 0) continue;
        int v = (int)std::lround((double)sum[i] / cnt[i]);
        v = std::max(-127, std::min(128, v));
        out[i] = (uint8_t)(v + 127);
    }
    g_batch.clear();
    publish_waveform(std::move(out));
}

// 干净帧入口: n<=1 直接发布, 否则攒够 waveform_average_n 帧后平均发布
void handle_clean_frame(std::vector<uint8_t> wave) {
    if (cfg.waveform_average_n <= 1) {
        publish_waveform(std::move(wave));
        return;
    }
    if (!g_batch.empty() && g_batch[0].size() != wave.size()) g_batch.clear();
    g_batch.push_back(std::move(wave));
    if ((int)g_batch.size() >= cfg.waveform_average_n) {
        average_and_publish();
    }
}


// 采集一次完整波形
// 固件对每次 0x01 请求都触发一次新采集并返回完整的 8185 样本(单chunk),
// chunk 编号无效, 因此每帧只请求一次 (参考 EmatPenThicknessGauge: WaveformChunkCount=1)
// 返回: 0 = 成功发布, 1 = 饱和帧(已丢弃, 非错误), -1 = 协议/传输错误
int acquire_one_waveform() {
    for (int attempt = 0; attempt < cfg.max_chunk_retries; attempt++) {
        if (!g_device_open) return -1;

        if (attempt > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(cfg.chunk_delay_ms));
        }

        // 注意: 不要在请求前对 EP0x86 做超时读"排空"。带超时的 IN URB 被
        // 取消会破坏 CH346 上传状态, 实测导致后续响应大量变为饱和帧。
        auto cmd = build_command(0x01, {1});
        {
            std::lock_guard<std::mutex> lock(g_usb_mutex);
            if (!usb_write(cmd)) {
                if (!g_device_open) return -1;
                continue;  // 写失败但设备仍在，重试
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg.chunk_delay_ms));

        std::vector<uint8_t> resp;
        {
            std::lock_guard<std::mutex> lock(g_usb_mutex);
            resp = usb_read(cfg.read_timeout_ms);
        }

        if (!g_device_open) return -1;

        // 帧格式: 0xAB | 源=设备0x01 | 目的=主机0x00 | 功能码0x01 | 长度(2B) | 数据 | 0xFF终止符
        if (resp.size() >= 7 && resp[0] == 0xAB && resp[1] == 0x01 &&
            resp[2] == 0x00 && resp[3] == 0x01) {
            uint16_t dlen = (resp[4] << 8) | resp[5];
            if (resp.size() >= (size_t)(6 + dlen)) {
                std::vector<uint8_t> wave(resp.begin() + 6, resp.begin() + 6 + dlen);
                if (!is_saturated(wave)) {
                    handle_clean_frame(std::move(wave));
                    return 0;
                }
                return 1;  // 饱和帧: 请求落在固件采集窗口内, 下个周期再取
            }
        }
        ROS_WARN("Waveform attempt %d/%d: bad response (%zu bytes, hdr=0x%02X)",
                 attempt + 1, cfg.max_chunk_retries,
                 resp.size(), resp.empty() ? 0 : resp[0]);
    }
    ROS_WARN("Waveform: all %d attempts exhausted", cfg.max_chunk_retries);
    return -1;
}

// 复位/上电后设备前2-3次请求不响应(内部初始化), 用牺牲性请求跳过预热窗口
// 实测: 端口复位后第3次请求才开始返回干净帧; 不预热则永远在"断连-复位"循环
void warmup_device() {
    if (cfg.warmup_requests <= 0) return;
    ROS_INFO("Warming up device (%d sacrificial requests)...", cfg.warmup_requests);
    for (int i = 0; i < cfg.warmup_requests && g_device_open; i++) {
        auto cmd = build_command(0x01, {1});
        {
            std::lock_guard<std::mutex> lock(g_usb_mutex);
            if (!g_device_open || !usb_write(cmd)) continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg.chunk_delay_ms));
        {
            std::lock_guard<std::mutex> lock(g_usb_mutex);
            usb_read(300);  // 短超时, 响应内容丢弃
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
    }
}

// 设备恢复 + 重连
// CH346C在EMI冲击后需要时间恢复，关闭→等待→重开即可
bool reset_and_reconnect() {
    {
        std::lock_guard<std::mutex> lock(g_usb_mutex);
        if (g_handle) {
            // 端口复位: 设备进入ZLP僵死(返回空包但不响应)时, 仅close+reopen救不回来
            libusb_reset_device(g_handle);
        }
    }
    close_device();
    ROS_INFO("Waiting %ds for device to recover...", cfg.reconnect_delay_s);
    std::this_thread::sleep_for(std::chrono::seconds(cfg.reconnect_delay_s));

    if (open_device()) {
        ROS_INFO("Reconnect successful!");
        warmup_device();
        return true;
    } else {
        ROS_WARN("Reconnect failed, will retry...");
        return false;
    }
}

void acquisition_loop() {
    ros::Rate rate(1000.0 / cfg.read_interval_ms);
    while (ros::ok() && g_running) {
        if (!g_device_open) {
            rate.sleep();
            continue;
        }

        int result = acquire_one_waveform();
        if (result < 0) {
            g_batch.clear();  // 出错后丢弃未完成的 batch, 避免跨长时间断连的帧混在一起
            if (!g_device_open) {
                // USB层已检测到设备断开，立即带端口复位的重连
                ROS_ERROR("Device disconnected during acquisition, resetting and reconnecting...");
                reset_and_reconnect();
            } else {
                // 响应格式错误（非USB层错误），累计后触发重连
                g_consecutive_failures++;
                if (g_consecutive_failures.load() >= cfg.max_consecutive_failures) {
                    ROS_ERROR("Consecutive protocol failures: %d, resetting and reconnecting...",
                              g_consecutive_failures.load());
                    reset_and_reconnect();
                }
            }
        } else {
            g_consecutive_failures = 0;
        }

        rate.sleep();
    }
}

// 后台重连线程：当设备未连接时，周期性尝试重新连接
void reconnect_loop() {
    while (ros::ok() && g_running) {
        if (!g_device_open) {
            int dev_status = 0;
            {
                libusb_context* probe_ctx = nullptr;
                if (libusb_init(&probe_ctx) == 0) {
                    dev_status = detect_device(probe_ctx);
                    libusb_exit(probe_ctx);
                }
            }

            if (dev_status == 2) {
                ROS_ERROR_THROTTLE(10, "Device in BOOTROM mode (PID=0x55E0). Replug USB cable!");
            } else if (dev_status == 1) {
                ROS_INFO("Device detected, attempting reconnect...");
                reset_and_reconnect();
            }
            // dev_status == 0: 设备不存在，静默等待
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

} // anonymous namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "emat");
    ros::NodeHandle nh("~");

    nh.param("read_interval_ms", cfg.read_interval_ms, 50);
    nh.param("waveform_average_n", cfg.waveform_average_n, 4);
    if (cfg.waveform_average_n < 1) cfg.waveform_average_n = 1;
    if (cfg.waveform_average_n > 16) cfg.waveform_average_n = 16;
    nh.param("warmup_requests", cfg.warmup_requests, 3);
    nh.param("sat_median_threshold", cfg.sat_median_threshold, 40);
    nh.param("quiet_std_threshold", cfg.quiet_std_threshold, 10.0);
    nh.param("default_speed", cfg.default_speed, 3230.0);
    nh.param("chunk_delay_ms", cfg.chunk_delay_ms, 30);
    nh.param("read_timeout_ms", cfg.read_timeout_ms, 500);
    nh.param("reconnect_delay_s", cfg.reconnect_delay_s, 3);
    nh.param("max_startup_retries", cfg.max_startup_retries, 5);
    nh.param("max_consecutive_failures", cfg.max_consecutive_failures, 5);

    ros::NodeHandle n;
    ros::Publisher pub_wave = n.advertise<emat::EmatWaveform>("emat/waveform", 10);
    g_pub_waveform = &pub_wave;

    ROS_INFO("EMAT Thickness Gauge (libusb IF2, auto-recovery enabled)");
    ROS_INFO("Waveform averaging: %d frame(s) per publish, request interval %d ms",
             cfg.waveform_average_n, cfg.read_interval_ms);

    // === 启动阶段：带重试的设备连接 ===
    bool connected = false;
    for (int attempt = 1; attempt <= cfg.max_startup_retries && ros::ok(); attempt++) {
        int dev_status = 0;
        {
            libusb_context* probe_ctx = nullptr;
            if (libusb_init(&probe_ctx) == 0) {
                dev_status = detect_device(probe_ctx);
                libusb_exit(probe_ctx);
            }
        }

        if (dev_status == 0) {
            ROS_WARN("Attempt %d/%d: EMAT device not found. "
                     "Check USB cable. Retrying in 2s...",
                     attempt, cfg.max_startup_retries);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        if (dev_status == 2) {
            ROS_ERROR("Attempt %d/%d: Device in BOOTROM mode (PID=0x55E0). "
                      "Physically unplug and replug the USB cable!",
                      attempt, cfg.max_startup_retries);
            std::this_thread::sleep_for(std::chrono::seconds(3));
            continue;
        }

        // dev_status == 1: 设备正常，尝试打开
        connected = open_device();
        if (connected) {
            ROS_INFO("Device connected on attempt %d", attempt);
            break;
        }
        ROS_WARN("Attempt %d/%d: open_device failed. Retrying in 2s...",
                 attempt, cfg.max_startup_retries);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    if (!connected) {
        ROS_ERROR("Failed to connect after %d attempts. Node will keep running "
                  "and retry in background...", cfg.max_startup_retries);
    } else {
        warmup_device();
    }

    // 采集线程 + 后台重连线程
    std::thread acq_thread(acquisition_loop);
    std::thread recon_thread(reconnect_loop);
    ROS_INFO("Topics: emat/waveform");
    ROS_INFO("Auto-recovery: enabled (failures=%d, reconnect_delay=%ds)",
             cfg.max_consecutive_failures, cfg.reconnect_delay_s);

    ros::spin();
    g_running = false;
    acq_thread.join();
    recon_thread.join();
    close_device();
    return 0;
}
