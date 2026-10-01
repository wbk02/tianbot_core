#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace tianbot_core::mini_v1_5 {
constexpr std::uint16_t kMagic = 0x5442;
constexpr std::size_t kHeaderSize = 12;
constexpr std::size_t kFrameOverhead = 16;
struct Frame { std::uint8_t type; std::uint8_t flags; std::uint32_t sequence; std::vector<std::uint8_t> payload; };
std::uint32_t crc32(const std::uint8_t *data, std::size_t size);
bool decode_frame(const std::uint8_t *data, std::size_t size, Frame *frame);
bool encode_frame(std::uint8_t type, std::uint8_t flags, std::uint32_t sequence,
                  const std::vector<std::uint8_t> &payload, std::vector<std::uint8_t> *out);
std::uint16_t read_u16(const std::uint8_t *data);
std::uint32_t read_u32(const std::uint8_t *data);
void write_u32(std::uint8_t *data, std::uint32_t value);
}
