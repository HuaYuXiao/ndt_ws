#pragma once
#include <libusb-1.0/libusb.h>
#include <string>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <memory>
#include <cstdint>

namespace emat {
constexpr uint16_t CH346_VID = 0x1A86;
constexpr uint16_t SUPPORTED_PIDS[] = { 0x55DA, 0x55DB, 0x5512, 0x5523 };
constexpr uint32_t CH346_DEV_ARRIVAL = 3;
constexpr uint32_t CH346_DEV_REMOVE  = 0;
constexpr uint32_t TIMEOUT_INFINITE  = 0xFFFFFFFF;
constexpr int READ_BUFFER_SIZE = 1024 * 10;
struct DeviceInfo { std::string devID, product, manufacturer, serial; uint8_t chipMode = 0; };
class CH346Driver {
public:
    static CH346Driver& instance();
    ~CH346Driver();
    bool openDevice(int idx = 0); void closeDevice();
    bool isOpen() const { return _open.load(); }
    bool readData(uint8_t* b, uint32_t& l); bool writeData(const uint8_t* d, uint32_t& l);
    bool setBufUpload(int m, int s); bool setTimeout(uint32_t r, uint32_t w);
    bool getDeviceInfo(DeviceInfo& info);
    void startMonitor(); void stopMonitor();
    std::function<void()> onArrived, onRemoved;
    using NotifyCb = std::function<void(uint32_t)>;
    bool setNotify(NotifyCb cb);
private:
    CH346Driver(); CH346Driver(const CH346Driver&) = delete;
    bool findDevice(); void getEndpoints(); void monitorLoop();
    libusb_context* _ctx = nullptr; libusb_device_handle* _h = nullptr; libusb_device* _dev = nullptr;
    std::atomic<bool> _open{false}, _monRun{false}, _monStop{false};
    uint8_t _epIn = 0x82, _epOut = 0x02; uint16_t _epInMax = 64;
    uint32_t _rto = TIMEOUT_INFINITE, _wto = TIMEOUT_INFINITE;
    std::mutex _mx; NotifyCb _ncb; std::unique_ptr<std::thread> _mt;
};
}
