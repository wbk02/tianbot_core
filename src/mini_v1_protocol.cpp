#include "tianbot_core/mini_v1_protocol.hpp"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
namespace tianbot_core::mini_v1_5 {
void write_u16(std::uint8_t *p, std::uint16_t v) {
  p[0] = v;
  p[1] = v >> 8;
}
std::uint16_t read_u16(const std::uint8_t *p) { return p[0] | (p[1] << 8); }
std::uint32_t read_u32(const std::uint8_t *p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}
void write_u32(std::uint8_t *p, std::uint32_t v) {
  p[0] = v;
  p[1] = v >> 8;
  p[2] = v >> 16;
  p[3] = v >> 24;
}
std::uint32_t crc32(const std::uint8_t *d, std::size_t n) {
  std::uint32_t c = 0xffffffffU;
  for (std::size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++)
      c = (c >> 1) ^ ((c & 1U) ? 0xedb88320U : 0);
  }
  return c ^ 0xffffffffU;
}
bool decode_frame(const std::uint8_t *d, std::size_t n, Frame *f) {
  if (!d || !f || n < kFrameOverhead || read_u16(d) != kMagic || d[2] != 1 ||
      d[5] != kHeaderSize)
    return false;
  auto l = read_u16(d + 10);
  if (n != kFrameOverhead + l || l > 1200 ||
      read_u32(d + 12 + l) != crc32(d, 12 + l))
    return false;
  f->type = d[3];
  f->flags = d[4];
  f->sequence = read_u32(d + 6);
  f->payload.assign(d + 12, d + 12 + l);
  return true;
}
bool encode_frame(std::uint8_t t, std::uint8_t fl, std::uint32_t seq,
                  const std::vector<std::uint8_t> &p,
                  std::vector<std::uint8_t> *o) {
  if (!o || p.size() > 1200)
    return false;
  o->assign(kFrameOverhead + p.size(), 0);
  auto *d = o->data();
  write_u16(d, kMagic);
  d[2] = 1;
  d[3] = t;
  d[4] = fl;
  d[5] = 12;
  write_u32(d + 6, seq);
  write_u16(d + 10, p.size());
  std::copy(p.begin(), p.end(), d + 12);
  write_u32(d + 12 + p.size(), crc32(d, 12 + p.size()));
  return true;
}
bool parse_discovery(const Frame &f, DeviceInfo *i) {
  if (!i || f.type != kDiscoverResponse || f.payload.size() != 49)
    return false;
  std::copy_n(f.payload.begin() + 4, 6, i->mac.begin());
  i->name.assign(reinterpret_cast<const char *>(f.payload.data() + 10), 32);
  {
    auto z = i->name.find('\0');
    if (z != std::string::npos)
      i->name.resize(z);
  }
  i->firmware_version = read_u32(f.payload.data() + 42);
  i->state = f.payload[46];
  i->faults = read_u16(f.payload.data() + 47);
  return true;
}
std::string namespace_name(const std::string &n, const std::array<std::uint8_t, 6> &) {
  std::string o;
  for (unsigned char c : n) {
    if (std::isalnum(c) || c == '_') o += static_cast<char>(c);
    else if (!o.empty() && o.back() != '_') o += '_';
  }
  while (!o.empty() && o.back() == '_') o.pop_back();
  return o.empty() ? "robot" : o;
}
} // namespace tianbot_core::mini_v1_5
