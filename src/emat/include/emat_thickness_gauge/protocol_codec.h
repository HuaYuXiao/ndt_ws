#pragma once
#include <cstdint>
#include <vector>
namespace emat {
constexpr uint8_t PROTO_HEAD  = 0xAB;
constexpr uint8_t FUNC_THICKNESS = 0x00, FUNC_WAVEFORM = 0x01, FUNC_SET_PARAM = 0x03;
constexpr int PACKET_FIXED = 7;
struct Pkt { uint8_t func = 0; uint16_t dlen = 0; std::vector<uint8_t> data; bool ok = false; };
class ProtocolCodec {
public:
    std::vector<uint8_t> encode(uint8_t fc, const uint8_t* d, uint16_t n);
    std::vector<Pkt> feed(const uint8_t* d, size_t n); void clear();
private:
    std::vector<uint8_t> _b; uint8_t cs(const uint8_t* d, int n);
};
}
