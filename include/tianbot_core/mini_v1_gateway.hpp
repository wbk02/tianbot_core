#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
namespace tianbot_core { namespace mini_v1_5 {
struct Frame { std::uint8_t type{}, flags{}; std::uint32_t sequence{}; std::vector<std::uint8_t> payload; };
std::uint32_t mini_crc32(const std::uint8_t *, std::size_t);
bool decode_mini_frame(const std::uint8_t *, std::size_t, Frame *);
bool encode_mini_frame(std::uint8_t, std::uint8_t, std::uint32_t, const std::vector<std::uint8_t>&, std::vector<std::uint8_t>*);
std::uint16_t read_u16(const std::uint8_t *); std::uint32_t read_u32(const std::uint8_t *); void write_u16(std::uint8_t *, std::uint16_t); void write_u32(std::uint8_t *, std::uint32_t);
} }
