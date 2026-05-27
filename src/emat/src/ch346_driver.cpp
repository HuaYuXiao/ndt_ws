#include "emat/ch346_driver.h"
#include <ros/ros.h>
#include <cstring>
#include <chrono>
#include <algorithm>
namespace emat {
CH346Driver& CH346Driver::instance() { static CH346Driver d; return d; }
CH346Driver::CH346Driver() { if (libusb_init(&_ctx) < 0) { ROS_ERROR("CH346: init fail"); _ctx = nullptr; } }
CH346Driver::~CH346Driver() { stopMonitor(); closeDevice(); if (_ctx) libusb_exit(_ctx); }
bool CH346Driver::findDevice() {
    if (!_ctx) return false;
    libusb_device** devs; ssize_t n = libusb_get_device_list(_ctx, &devs);
    if (n < 0) return false;
    for (ssize_t i = 0; i < n; i++) {
        libusb_device_descriptor d;
        if (libusb_get_device_descriptor(devs[i], &d) == 0 && d.idVendor == CH346_VID)
            for (auto p : SUPPORTED_PIDS) if (d.idProduct == p) {
                _dev = devs[i]; libusb_ref_device(_dev);
                ROS_INFO("CH346: found %04x:%04x", d.idVendor, d.idProduct);
                libusb_free_device_list(devs, 1); return true;
            }
    }
    libusb_free_device_list(devs, 1); return false;
}
bool CH346Driver::openDevice(int) {
    std::lock_guard<std::mutex> lk(_mx);
    if (_open) return true;
    if (!findDevice()) { ROS_WARN("CH346: not found"); return false; }
    if (libusb_open(_dev, &_h) < 0) { ROS_ERROR("CH346: open fail"); return false; }
    if (libusb_kernel_driver_active(_h, 0) == 1) libusb_detach_kernel_driver(_h, 0);
    if (libusb_claim_interface(_h, 0) < 0) { libusb_close(_h); _h = nullptr; return false; }
    getEndpoints(); _open = true; ROS_INFO("CH346: opened"); return true;
}
void CH346Driver::closeDevice() {
    std::lock_guard<std::mutex> lk(_mx);
    if (_h) { libusb_release_interface(_h, 0); libusb_close(_h); _h = nullptr; }
    if (_dev) { libusb_unref_device(_dev); _dev = nullptr; } _open = false;
}
void CH346Driver::getEndpoints() {
    if (!_dev) return;
    libusb_config_descriptor* c;
    if (libusb_get_active_config_descriptor(_dev, &c) < 0) return;
    for (uint8_t i = 0; i < c->bNumInterfaces; i++)
        for (int j = 0; j < c->interface[i].num_altsetting; j++) {
            const auto& a = c->interface[i].altsetting[j];
            for (uint8_t k = 0; k < a.bNumEndpoints; k++)
                if (a.endpoint[k].bmAttributes == LIBUSB_TRANSFER_TYPE_BULK) {
                    if (a.endpoint[k].bEndpointAddress & LIBUSB_ENDPOINT_IN)
                        { _epIn = a.endpoint[k].bEndpointAddress; _epInMax = a.endpoint[k].wMaxPacketSize; }
                    else _epOut = a.endpoint[k].bEndpointAddress;
                }
        }
    libusb_free_config_descriptor(c);
}
bool CH346Driver::readData(uint8_t* b, uint32_t& l) {
    if (!_open || !_h) return false;
    int a = 0; unsigned int t = (_rto == TIMEOUT_INFINITE) ? 0 : _rto;
    if (libusb_bulk_transfer(_h, _epIn, b, l, &a, t) < 0) return false;
    l = a; return l > 0;
}
bool CH346Driver::writeData(const uint8_t* d, uint32_t& l) {
    if (!_open || !_h) return false;
    int a = 0; unsigned int t = (_wto == TIMEOUT_INFINITE) ? 5000 : _wto;
    if (libusb_bulk_transfer(_h, _epOut, const_cast<uint8_t*>(d), l, &a, t) < 0) return false;
    l = a; return true;
}
bool CH346Driver::setBufUpload(int m, int s) { return _h && libusb_control_transfer(_h, 0x40, 0x10, m, s, nullptr, 0, 1000) >= 0; }
bool CH346Driver::setTimeout(uint32_t r, uint32_t w) { _rto = r; _wto = w; return _h && libusb_control_transfer(_h, 0x40, 0x0f, w, r, nullptr, 0, 1000) >= 0; }
bool CH346Driver::getDeviceInfo(DeviceInfo& info) {
    if (!_h || !_dev) return false;
    libusb_device_descriptor d;
    if (libusb_get_device_descriptor(_dev, &d) < 0) return false;
    char buf[128];
    snprintf(buf, 128, "USB\VID_%04X&PID_%04X", d.idVendor, d.idProduct); info.devID = buf;
    auto rs = [&](uint8_t i, std::string& o) {
        if (!i) return; unsigned char t[256];
        int l = libusb_get_string_descriptor_ascii(_h, i, t, sizeof(t));
        if (l > 0) o.assign((char*)t, l);
    };
    rs(d.iProduct, info.product); rs(d.iManufacturer, info.manufacturer); rs(d.iSerialNumber, info.serial);
    return true;
}
bool CH346Driver::setNotify(NotifyCb c) { _ncb = std::move(c); return true; }
void CH346Driver::startMonitor() { if (!_monRun) { _monStop = false; _mt = std::make_unique<std::thread>(&CH346Driver::monitorLoop, this); } }
void CH346Driver::stopMonitor() { if (_monRun) { _monStop = true; if (_mt && _mt->joinable()) _mt->join(); _mt.reset(); } }
void CH346Driver::monitorLoop() {
    _monRun = true; bool was = _open.load();
    while (!_monStop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        bool now = _open.load();
        if (now && !was) { ROS_INFO("CH346: arrived"); if (_ncb) _ncb(CH346_DEV_ARRIVAL); if (onArrived) onArrived(); }
        else if (!now && was) { ROS_INFO("CH346: removed"); if (_ncb) _ncb(CH346_DEV_REMOVE); if (onRemoved) onRemoved(); }
        was = now;
    }
    _monRun = false;
}
} // namespace emat
