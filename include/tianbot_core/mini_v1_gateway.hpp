#pragma once

#include <cstdint>
#include <vector>

namespace tianbot_core { namespace mini_v1_5 {
struct Frame { std::uint8_t type; std::uint32_t sequence; std::vector<std::uint8_t> payload; };
std::uint32_t mini_crc32(const std::uint8_t *data, std::size_t size);
bool decode_mini_frame(const std::uint8_t *data, std::size_t size, Frame *frame);
bool valid_session(std::uint32_t session_id, std::uint32_t deadline_ms, std::uint32_t now_ms);
} }
