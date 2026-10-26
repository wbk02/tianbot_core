#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
namespace tianbot_core::mini_v1_5 {
constexpr std::uint16_t kMagic = 0x5442;
constexpr std::size_t kHeaderSize = 12;
constexpr std::size_t kFrameOverhead = 16;
enum Message : std::uint8_t {
  kHello = 0x01,
  kHelloAck = 0x02,
  kLinkHeartbeat = 0x03,
  kControlLease = 0x10,
  kVelocity = 0x11,
  kStop = 0x12,
  kVehicleState = 0x20,
  kImu = 0x21,
  kOdometry = 0x22,
  kLocalEvent = 0x23,
  kDiscoverRequest = 0x30,
  kDiscoverResponse = 0x31,
  kRegister = 0x32,
  kRegisterAck = 0x33,
  kHeartbeat = 0x34,
  kSessionGrant = 0x35,
  kSessionAck = 0x36,
  kSessionRelease = 0x37,
  kLidarFragment = 0x40,
  kConfigGet = 0x50,
  kConfigSet = 0x51,
  kOtaBegin = 0x60,
  kOtaChunk = 0x61,
  kOtaQuery = 0x62,
  kOtaFinish = 0x63,
  kOtaResult = 0x64
};
struct Frame {
  std::uint8_t type{};
  std::uint8_t flags{};
  std::uint32_t sequence{};
  std::vector<std::uint8_t> payload;
};
struct DeviceInfo {
  std::array<std::uint8_t, 6> mac{};
  std::string name;
  std::uint32_t firmware_version{};
  std::uint8_t state{};
  std::uint16_t faults{};
};
std::uint32_t crc32(const std::uint8_t *, std::size_t);
bool decode_frame(const std::uint8_t *, std::size_t, Frame *);
bool encode_frame(std::uint8_t, std::uint8_t, std::uint32_t,
                  const std::vector<std::uint8_t> &,
                  std::vector<std::uint8_t> *);
std::uint16_t read_u16(const std::uint8_t *);
std::uint32_t read_u32(const std::uint8_t *);
void write_u16(std::uint8_t *, std::uint16_t);
void write_u32(std::uint8_t *, std::uint32_t);
bool parse_discovery(const Frame &, DeviceInfo *);
std::string namespace_name(const std::string &,
                           const std::array<std::uint8_t, 6> &);
} // namespace tianbot_core::mini_v1_5
