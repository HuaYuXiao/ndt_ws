#include "emat/protocol_codec.h"
#include <ros/ros.h>
#include <algorithm>
namespace emat {
std::vector<uint8_t> ProtocolCodec::encode(uint8_t fc, const uint8_t* d, uint16_t n) {
    std::vector<uint8_t> p; p.reserve(n + 7);
    p.push_back(PROTO_HEAD); p.push_back(0x00); p.push_back(0x01); p.push_back(fc);
    p.push_back((n >> 8) & 0xFF); p.push_back(n & 0xFF);
    if (d && n) p.insert(p.end(), d, d + n);
    p.push_back(cs(p.data(), p.size())); return p;
}
uint8_t ProtocolCodec::cs(const uint8_t* d, int n) { uint8_t s = 0; for (int i = 0; i < n; i++) s += d[i]; return s; }
std::vector<Pkt> ProtocolCodec::feed(const uint8_t* d, size_t n) {
    _b.insert(_b.end(), d, d + n); std::vector<Pkt> pkts;
    while (_b.size() >= PACKET_FIXED) {
        auto it = std::find(_b.begin(), _b.end(), PROTO_HEAD);
        if (it == _b.end()) { _b.clear(); break; }
        if (it != _b.begin()) { _b.erase(_b.begin(), it); continue; }
        if (_b.size() < PACKET_FIXED) break;
        uint16_t dl = (uint16_t(_b[4]) << 8) | _b[5];
        size_t tot = PACKET_FIXED + dl;
        if (_b.size() < tot) break;
        Pkt p; p.func = _b[3]; p.dlen = dl; p.data.assign(_b.begin() + 6, _b.begin() + 6 + dl);
        p.ok = (cs(_b.data(), 6 + dl) == _b[6 + dl]);
        if (p.ok) pkts.push_back(p);
        _b.erase(_b.begin(), _b.begin() + tot);
    }
    if (_b.size() > 500) _b.clear(); return pkts;
}
void ProtocolCodec::clear() { _b.clear(); }
}
