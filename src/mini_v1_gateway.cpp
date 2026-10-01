#include "tianbot_core/mini_v1_gateway.hpp"

namespace tianbot_core { namespace mini_v1_5 {
namespace { std::uint16_t u16(const std::uint8_t *p) { return static_cast<std::uint16_t>(p[0]) | static_cast<std::uint16_t>(p[1]) << 8; } std::uint32_t u32(const std::uint8_t *p) { return static_cast<std::uint32_t>(p[0]) | static_cast<std::uint32_t>(p[1]) << 8 | static_cast<std::uint32_t>(p[2]) << 16 | static_cast<std::uint32_t>(p[3]) << 24; } }
std::uint32_t mini_crc32(const std::uint8_t *data, std::size_t size) { std::uint32_t crc = 0xffffffffU; for (std::size_t i = 0; i < size; ++i) { crc ^= data[i]; for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U); } return crc ^ 0xffffffffU; }
bool decode_mini_frame(const std::uint8_t *data, std::size_t size, Frame *frame) { if (!data || !frame || size < 16U || u16(data) != 0x5442U || data[2] != 1U || data[5] != 12U) return false; const auto len = u16(data + 10); if (size != 16U + len || u32(data + 12U + len) != mini_crc32(data, 12U + len)) return false; frame->type = data[3]; frame->sequence = u32(data + 6); frame->payload.assign(data + 12, data + 12 + len); return true; }
bool valid_session(std::uint32_t session_id, std::uint32_t deadline_ms, std::uint32_t now_ms) { return session_id != 0U && static_cast<std::int32_t>(deadline_ms - now_ms) > 0; }
} }
