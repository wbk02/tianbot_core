#include "tianbot_core/mini_v1_protocol.hpp"

namespace tianbot_core::mini_v1_5 {
namespace { std::uint16_t u16(const std::uint8_t *p) { return static_cast<std::uint16_t>(p[0]) | static_cast<std::uint16_t>(p[1]) << 8; } std::uint32_t u32(const std::uint8_t *p) { return static_cast<std::uint32_t>(p[0]) | static_cast<std::uint32_t>(p[1]) << 8 | static_cast<std::uint32_t>(p[2]) << 16 | static_cast<std::uint32_t>(p[3]) << 24; } }
std::uint32_t crc32(const std::uint8_t *data, std::size_t size) { std::uint32_t crc = 0xffffffffU; for (std::size_t i = 0; i < size; ++i) { crc ^= data[i]; for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U); } return crc ^ 0xffffffffU; }
bool decode_frame(const std::uint8_t *data, std::size_t size, Frame *frame) { if (!data || !frame || size < kFrameOverhead || u16(data) != kMagic || data[2] != 1 || data[5] != kHeaderSize) return false; const auto payload_size = u16(data + 10); if (size != kFrameOverhead + payload_size || u32(data + kHeaderSize + payload_size) != crc32(data, kHeaderSize + payload_size)) return false; frame->type = data[3]; frame->flags = data[4]; frame->sequence = u32(data + 6); frame->payload.assign(data + kHeaderSize, data + kHeaderSize + payload_size); return true; }
}
