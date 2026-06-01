#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <libusb-1.0/libusb.h>

#include "emat/EmatWaveform.h"
#include "emat/EmatThickness.h"
#include "emat/EmatDeviceStatus.h"

namespace {

struct Config {
    int read_interval_ms = 100;
    int status_interval_ms = 2000;
    int num_chunks = 4;
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
    int max_startup_retries = 5;   // 启动时最大重试次数 (30 × 2s = 60s)
    int max_consecutive_failures = 5; // 连续失败 N 次后触发重连
    int max_chunk_retries = 2;      // 单chunk的最大重试次数
    int max_read_timeouts = 5;      // usb_read中连续超时多少次后放弃
} cfg;

libusb_context* g_ctx = nullptr;
libusb_device_handle* g_handle = nullptr;
std::mutex g_usb_mutex;
std::atomic<bool> g_device_open{false};
std::atomic<int> g_consecutive_failures{0};
std::atomic<bool> g_running{true};

ros::Publisher* g_pub_waveform = nullptr;
ros::Publisher* g_pub_thickness = nullptr;
ros::Publisher* g_pub_status = nullptr;

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
    if (rc == LIBUSB_ERROR_PIPE) {
        ROS_WARN("USB write: endpoint stalled, clearing halt...");
        libusb_clear_halt(g_handle, cfg.ep_out);
        // Retry once after clearing halt
        rc = libusb_bulk_transfer(g_handle, cfg.ep_out,
                                  const_cast<uint8_t*>(data.data()),
                                  (int)data.size(), &actual, cfg.write_timeout_ms);
        if (rc == 0 && actual == (int)data.size()) return true;
        g_device_open = false;
        return false;
    }
    if (rc == LIBUSB_ERROR_TIMEOUT) {
        ROS_WARN("USB write: timed out, retrying...");
        // One retry on timeout — may be transient EMI
        rc = libusb_bulk_transfer(g_handle, cfg.ep_out,
                                  const_cast<uint8_t*>(data.data()),
                                  (int)data.size(), &actual, cfg.write_timeout_ms);
        if (rc == 0 && actual == (int)data.size()) return true;
        ROS_WARN("USB write: retry also failed, marking disconnected");
        g_device_open = false;
        return false;
    }
    if (rc == LIBUSB_ERROR_IO) {
        ROS_WARN("USB write: I/O error, retrying...");
        rc = libusb_bulk_transfer(g_handle, cfg.ep_out,
                                  const_cast<uint8_t*>(data.data()),
                                  (int)data.size(), &actual, cfg.write_timeout_ms);
        if (rc == 0 && actual == (int)data.size()) return true;
        ROS_WARN("USB write: retry also failed, marking disconnected");
        g_device_open = false;
        return false;
    }
    if (rc != 0) {
        ROS_WARN("USB write: unexpected error (rc=%d), marking disconnected", rc);
        g_device_open = false;
        return false;
    }
    if (actual != (int)data.size()) {
        ROS_WARN("USB write: short write (%d/%zu bytes)", actual, data.size());
        g_device_open = false;
        return false;
    }
    return true;
}

std::vector<uint8_t> usb_read(int timeout_ms = 500) {
    if (!g_device_open) return {};
    std::vector<uint8_t> all;
    uint8_t buf[512];
    int consecutive_timeouts = 0;

    for (int i = 0; i < 30 && consecutive_timeouts < cfg.max_read_timeouts; i++) {
        int actual = 0;
        int rc = libusb_bulk_transfer(g_handle, cfg.ep_in, buf, sizeof(buf),
                                      &actual, std::min(timeout_ms, 200));
        // ---- 致命错误：设备不存在 ----
        if (rc == LIBUSB_ERROR_NO_DEVICE) {
            ROS_WARN("USB read: device disconnected");
            g_device_open = false;
            return all;
        }

        // ---- PIPE 错误：端点halt，清除后重试 ----
        if (rc == LIBUSB_ERROR_PIPE) {
            ROS_WARN("USB read: endpoint stalled, clearing halt...");
            libusb_clear_halt(g_handle, cfg.ep_in);
            consecutive_timeouts = 0;
            continue;  // 清除halt后继续尝试读取
        }

        // ---- 超时：递增计数器，到达阈值后放弃 ----
        if (rc == LIBUSB_ERROR_TIMEOUT) {
            consecutive_timeouts++;
            if (consecutive_timeouts >= cfg.max_read_timeouts) {
                ROS_WARN("USB read: %d consecutive timeouts, device unresponsive",
                         consecutive_timeouts);
                g_device_open = false;
            }
            continue;  // 不要break，EMI可能只是瞬时干扰
        }

        // ---- I/O 错误：EMI瞬时干扰，重试 ----
        if (rc == LIBUSB_ERROR_IO) {
            consecutive_timeouts = 0;  // 不同于超时，重置计数器
            continue;  // I/O error通常是瞬时EMI，继续重试
        }

        // ---- 其他未知错误 ----
        if (rc < 0) {
            ROS_WARN("USB read: unexpected error (rc=%d), marking disconnected", rc);
            g_device_open = false;
            break;
        }

        // ---- 成功读取到数据 ----
        if (rc >= 0 && actual > 0) {
            consecutive_timeouts = 0;  // 重置超时计数
            all.insert(all.end(), buf, buf + actual);
            if (all.size() >= 8192) break;
        } else {
            // actual == 0: 设备没有更多数据，正常结束
            break;
        }
    }
    return all;
}

void publish_status(bool connected, const std::string& msg = "") {
    emat::EmatDeviceStatus s;
    s.stamp = ros::Time::now();
    s.is_connected = connected;
    s.status_message = msg.empty() ? (connected ? "connected" : "disconnected") : msg;
    g_pub_status->publish(s);
}

void publish_waveform(const std::vector<uint8_t>& data) {
    emat::EmatWaveform msg;
    msg.stamp = ros::Time::now();
    msg.sample_count = data.size();
    msg.raw_data = data;
    msg.speed_of_voice = (uint32_t)cfg.default_speed;
    msg.average_count = cfg.num_chunks;
    msg.excitation_frequency_mhz = 3.0f;
    msg.thickness_mm = std::numeric_limits<float>::quiet_NaN();
    g_pub_waveform->publish(msg);
}

// 采集一次完整波形，返回是否成功
// 每个chunk最多重试 max_chunk_retries 次，处理EMI瞬时干扰
bool acquire_one_waveform() {
    std::vector<uint8_t> all_waveform;

    for (int chunk = 1; chunk <= cfg.num_chunks; chunk++) {
        bool chunk_ok = false;

        for (int attempt = 0; attempt < cfg.max_chunk_retries && !chunk_ok; attempt++) {
            if (!g_device_open) return false;

            if (attempt > 0) {
                // 重试前给CH346C更多恢复时间，逐次递增
                int backoff_ms = cfg.chunk_delay_ms * (attempt + 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
            }

            auto cmd = build_command(0x01, {(uint8_t)chunk});
            {
                std::lock_guard<std::mutex> lock(g_usb_mutex);
                if (!usb_write(cmd)) {
                    if (!g_device_open) return false;
                    continue;  // 写失败但设备仍在，重试
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(cfg.chunk_delay_ms));

            std::vector<uint8_t> resp;
            {
                std::lock_guard<std::mutex> lock(g_usb_mutex);
                resp = usb_read(cfg.read_timeout_ms);
            }

            if (!g_device_open) return false;

            if (resp.size() >= 7 && resp[0] == 0xAB && resp[3] == 0x01) {
                uint16_t dlen = (resp[4] << 8) | resp[5];
                if (resp.size() >= (size_t)(6 + dlen)) {
                    std::vector<uint8_t> chunk_data(resp.begin() + 6, resp.begin() + 6 + dlen);
                    all_waveform.insert(all_waveform.end(), chunk_data.begin(), chunk_data.end());
                    chunk_ok = true;
                }
            }

            if (!chunk_ok) {
                ROS_WARN("Chunk %d: attempt %d/%d failed (%zu bytes, hdr=0x%02X)",
                         chunk, attempt + 1, cfg.max_chunk_retries,
                         resp.size(), resp.empty() ? 0 : resp[0]);
            }
        }

        if (!chunk_ok) {
            ROS_WARN("Chunk %d: all %d attempts exhausted", chunk, cfg.max_chunk_retries);
            return false;
        }
    }
    publish_waveform(all_waveform);
    return true;
}

// 设备恢复 + 重连
// CH346C在EMI冲击后需要时间恢复，关闭→等待→重开即可
bool reset_and_reconnect() {
    close_device();
    ROS_INFO("Waiting %ds for device to recover...", cfg.reconnect_delay_s);
    std::this_thread::sleep_for(std::chrono::seconds(cfg.reconnect_delay_s));

    if (open_device()) {
        ROS_INFO("Reconnect successful!");
        publish_status(true, "reconnected");
        return true;
    } else {
        ROS_WARN("Reconnect failed, will retry...");
        publish_status(false, "reconnect failed, retrying...");
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

        bool ok = acquire_one_waveform();
        if (!ok) {
            if (!g_device_open) {
                // USB层已检测到设备断开，立即带端口复位的重连
                ROS_ERROR("Device disconnected during acquisition, resetting and reconnecting...");
                publish_status(false, "reconnecting after disconnection");
                reset_and_reconnect();
            } else {
                // 响应格式错误（非USB层错误），累计后触发重连
                g_consecutive_failures++;
                if (g_consecutive_failures.load() >= cfg.max_consecutive_failures) {
                    ROS_ERROR("Consecutive protocol failures: %d, resetting and reconnecting...",
                              g_consecutive_failures.load());
                    publish_status(false, "reconnecting after protocol errors");
                    reset_and_reconnect();
                }
            }
        } else {
            g_consecutive_failures = 0;
        }

        ros::spinOnce();
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
                publish_status(false, "device in bootrom mode, replug USB!");
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

    nh.param("read_interval_ms", cfg.read_interval_ms, 100);
    nh.param("num_chunks", cfg.num_chunks, 4);
    nh.param("default_speed", cfg.default_speed, 3230.0);
    nh.param("chunk_delay_ms", cfg.chunk_delay_ms, 10);
    nh.param("read_timeout_ms", cfg.read_timeout_ms, 500);
    nh.param("reconnect_delay_s", cfg.reconnect_delay_s, 3);
    nh.param("max_startup_retries", cfg.max_startup_retries, 30);
    nh.param("max_consecutive_failures", cfg.max_consecutive_failures, 5);

    ros::NodeHandle n;
    ros::Publisher pub_wave = n.advertise<emat::EmatWaveform>("emat/waveform", 10);
    ros::Publisher pub_thick = n.advertise<emat::EmatThickness>("emat/thickness", 10);
    ros::Publisher pub_stat = n.advertise<emat::EmatDeviceStatus>("emat/device_status", 1, true);
    g_pub_waveform = &pub_wave;
    g_pub_thickness = &pub_thick;
    g_pub_status = &pub_stat;

    ROS_INFO("EMAT Thickness Gauge (libusb IF2, auto-recovery enabled)");

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
            publish_status(false, "waiting for device...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        if (dev_status == 2) {
            ROS_ERROR("Attempt %d/%d: Device in BOOTROM mode (PID=0x55E0). "
                      "Physically unplug and replug the USB cable!",
                      attempt, cfg.max_startup_retries);
            publish_status(false, "device in bootrom mode, replug USB!");
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

    publish_status(connected);
    if (!connected) {
        ROS_ERROR("Failed to connect after %d attempts. Node will keep running "
                  "and retry in background...", cfg.max_startup_retries);
    }

    // 定时发布设备状态
    ros::Timer status_timer = n.createTimer(
        ros::Duration(cfg.status_interval_ms / 1000.0),
        [](const ros::TimerEvent&) {
            publish_status(g_device_open.load());
        });

    // 采集线程 + 后台重连线程
    std::thread acq_thread(acquisition_loop);
    std::thread recon_thread(reconnect_loop);
    ROS_INFO("Topics: emat/waveform, emat/thickness, emat/device_status");
    ROS_INFO("Auto-recovery: enabled (failures=%d, reconnect_delay=%ds)",
             cfg.max_consecutive_failures, cfg.reconnect_delay_s);

    ros::spin();
    g_running = false;
    acq_thread.join();
    recon_thread.join();
    close_device();
    return 0;
}
