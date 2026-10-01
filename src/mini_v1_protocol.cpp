#include "tianbot_core/mini_v1_protocol.hpp"

namespace tianbot_core::mini_v1_5 {
namespace { void u16_put(std::uint8_t *p, std::uint16_t v) { p[0] = static_cast<std::uint8_t>(v); p[1] = static_cast<std::uint8_t>(v >> 8); } }
std::uint16_t read_u16(const std::uint8_t *p) { return static_cast<std::uint16_t>(p[0]) | static_cast<std::uint16_t>(p[1]) << 8; }
std::uint32_t read_u32(const std::uint8_t *p) { return static_cast<std::uint32_t>(p[0]) | static_cast<std::uint32_t>(p[1]) << 8 | static_cast<std::uint32_t>(p[2]) << 16 | static_cast<std::uint32_t>(p[3]) << 24; }
void write_u32(std::uint8_t *p, std::uint32_t v) { p[0] = static_cast<std::uint8_t>(v); p[1] = static_cast<std::uint8_t>(v >> 8); p[2] = static_cast<std::uint8_t>(v >> 16); p[3] = static_cast<std::uint8_t>(v >> 24); }
std::uint32_t crc32(const std::uint8_t *data, std::size_t size) { std::uint32_t crc = 0xffffffffU; for (std::size_t i = 0; i < size; ++i) { crc ^= data[i]; for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U); } return crc ^ 0xffffffffU; }
bool decode_frame(const std::uint8_t *data, std::size_t size, Frame *frame) { if (!data || !frame || size < kFrameOverhead || read_u16(data) != kMagic || data[2] != 1 || data[5] != kHeaderSize) return false; const auto payload_size = read_u16(data + 10); if (size != kFrameOverhead + payload_size || read_u32(data + kHeaderSize + payload_size) != crc32(data, kHeaderSize + payload_size)) return false; frame->type = data[3]; frame->flags = data[4]; frame->sequence = read_u32(data + 6); frame->payload.assign(data + kHeaderSize, data + kHeaderSize + payload_size); return true; }
bool encode_frame(std::uint8_t type, std::uint8_t flags, std::uint32_t sequence, const std::vector<std::uint8_t> &payload, std::vector<std::uint8_t> *out)
{ if (out == nullptr || payload.size() > 1200U || (flags & ~0x03U) != 0U) return false; out->resize(kFrameOverhead + payload.size()); auto *p = out->data(); u16_put(p, kMagic); p[2] = 1; p[3] = type; p[4] = flags; p[5] = kHeaderSize; write_u32(p + 6, sequence); u16_put(p + 10, static_cast<std::uint16_t>(payload.size())); std::copy(payload.begin(), payload.end(), p + kHeaderSize); write_u32(p + kHeaderSize + payload.size(), crc32(p, kHeaderSize + payload.size())); return true; }
}
