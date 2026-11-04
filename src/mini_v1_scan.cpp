#include "tianbot_core/mini_v1_protocol.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <deque>
#include <map>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace mini = tianbot_core::mini_v1_5;

namespace {
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
constexpr std::uint16_t kManagementPort = 5003;
constexpr auto kBroadcastPeriod = std::chrono::seconds(1);
constexpr auto kMinimumScanTime = std::chrono::seconds(2);
constexpr auto kQuietPeriod = std::chrono::milliseconds(1500);
constexpr auto kMaximumScanTime = std::chrono::seconds(10);
constexpr std::size_t kRecentRequestLimit = 4;

volatile std::sig_atomic_t g_running = 1;

struct DeviceRecord {
  mini::DeviceInfo info;
  std::string ip;
};

void stop_handler(int) { g_running = 0; }

std::string mac_text(const std::array<std::uint8_t, 6> &mac) {
  char text[18]{};
  std::snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return text;
}

std::string ip_text(const sockaddr_in &address) {
  char text[INET_ADDRSTRLEN]{};
  return inet_ntop(AF_INET, &address.sin_addr, text, sizeof(text)) != nullptr
             ? text
             : "?";
}

bool is_recent_request(std::uint32_t request,
                       const std::deque<std::uint32_t> &recent_requests) {
  for (const auto known : recent_requests) {
    if (known == request) return true;
  }
  return false;
}

bool send_discovery(int fd, std::uint32_t request) {
  std::vector<std::uint8_t> payload(4);
  mini::write_u32(payload.data(), request);
  std::vector<std::uint8_t> frame;
  if (!mini::encode_frame(mini::kDiscoverRequest, 0, request, payload, &frame))
    return false;
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(kManagementPort);
  destination.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  return ::sendto(fd, frame.data(), frame.size(), 0,
                  reinterpret_cast<sockaddr *>(&destination),
                  sizeof(destination)) == static_cast<ssize_t>(frame.size());
}

void print_device(const DeviceRecord &record) {
  const auto &device = record.info;
  std::printf("设备: %-16s IP: %-15s MAC: %-17s 固件: %u 状态: %u 故障: 0x%04X\n",
              device.name.c_str(), record.ip.c_str(),
              mac_text(device.mac).c_str(), device.firmware_version,
              static_cast<unsigned>(device.state), device.faults);
}

int open_socket() {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    std::perror("创建 UDP socket 失败");
    return -1;
  }
  int enabled = 1;
  (void)setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled));
  (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
  timeval timeout{0, 100000};
  (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_port = htons(kManagementPort);
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) {
    std::fprintf(stderr, "绑定 UDP 端口 %u 失败，请先退出其他扫描/多车程序\n",
                 kManagementPort);
    ::close(fd);
    return -1;
  }
  return fd;
}

void print_summary(const std::map<std::string, DeviceRecord> &devices) {
  std::printf("\n设备扫描结束，共发现 %zu 台设备。\n", devices.size());
  if (!devices.empty()) {
    std::printf("启动全部设备：ros2 launch tianbot_core mini_v1_5_multi.launch.py vehicle_ips:=");
    bool first = true;
    for (const auto &entry : devices) {
      if (!first) std::printf(",");
      std::printf("%s", entry.second.ip.c_str());
      first = false;
    }
    std::printf("\n");
  }
  std::printf("设备扫描已退出。\n");
}
} // namespace

int main() {
  std::signal(SIGINT, stop_handler);
  std::signal(SIGTERM, stop_handler);
  const int fd = open_socket();
  if (fd < 0) return 2;

  const auto scan_start = Clock::now();
  auto last_new_device = scan_start;
  auto next_broadcast = scan_start;
  std::uint32_t next_request = 1;
  std::deque<std::uint32_t> recent_requests;
  std::map<std::string, DeviceRecord> devices;

  std::printf("Mini V1.5 设备扫描开始\n");
  std::printf("管理端口: %u；至少扫描 2 秒，无新增设备 1.5 秒后结束，最长 10 秒\n",
              kManagementPort);
  std::printf("正在广播发现请求到 255.255.255.255:%u ...\n", kManagementPort);
  std::fflush(stdout);

  while (g_running) {
    const auto now = Clock::now();
    const auto elapsed = now - scan_start;
    if (elapsed >= kMaximumScanTime ||
        (elapsed >= kMinimumScanTime && now - last_new_device >= kQuietPeriod))
      break;

    if (now >= next_broadcast) {
      const auto request = next_request++;
      if (recent_requests.size() >= kRecentRequestLimit)
        recent_requests.pop_front();
      recent_requests.push_back(request);
      if (!send_discovery(fd, request)) std::perror("发送发现请求失败");
      next_broadcast = now + kBroadcastPeriod;
    }

    std::uint8_t buffer[1400]{};
    sockaddr_in source{};
    socklen_t source_length = sizeof(source);
    const auto received = ::recvfrom(fd, buffer, sizeof(buffer), 0,
                                     reinterpret_cast<sockaddr *>(&source),
                                     &source_length);
    if (received <= 0) continue;
    mini::Frame frame;
    mini::DeviceInfo device;
    if (!mini::decode_frame(buffer, static_cast<std::size_t>(received), &frame) ||
        !mini::parse_discovery(frame, &device) || frame.payload.size() < 4 ||
        !is_recent_request(mini::read_u32(frame.payload.data()), recent_requests))
      continue;

    DeviceRecord record{device, ip_text(source)};
    const auto key = mac_text(device.mac);
    if (devices.emplace(key, record).second) {
      print_device(record);
      last_new_device = Clock::now();
    }
  }
  print_summary(devices);
  ::close(fd);
  return 0;
}
