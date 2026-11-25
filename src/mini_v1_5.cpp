#include "tianbot_core/mini_v1_5.h"

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace mini = tianbot_core::mini_v1_5;

namespace {
const char *kRed = "\033[31m";
const char *kGreen = "";
const char *kCyan = "";
const char *kReset = "\033[0m";
template <class T> T loadValue(const uint8_t *p) {
  T v{};
  std::memcpy(&v, p, sizeof(v));
  return v;
}

bool parseMac(const std::string &s, std::array<uint8_t, 6> *m) {
  unsigned v[6]{};
  if (!m || s.empty() ||
      std::sscanf(s.c_str(), "%2x:%2x:%2x:%2x:%2x:%2x", v, v + 1, v + 2, v + 3,
                  v + 4, v + 5) != 6) {
    return false;
  }
  for (size_t i = 0; i < 6; i++) {
    (*m)[i] = static_cast<uint8_t>(v[i]);
  }
  return true;
}
} // namespace

TianbotMiniV15::TianbotMiniV15(const std::shared_ptr<rclcpp::Node> &node)
    : TianbotChasis(node) {
  const char *color_override = std::getenv("RCUTILS_COLORIZED_OUTPUT");
  const bool use_color = std::getenv("NO_COLOR") == nullptr &&
                         (color_override ? std::string(color_override) == "1"
                                         : ::isatty(STDOUT_FILENO));
  if (!use_color)
    kRed = kGreen = kCyan = kReset = "";
  // The Mini V1.5 link is not comm_inf_-based. Stop the legacy heartbeat and
  // communication-timeout timers that TianbotCore started in its constructor.
  heartbeat_timer_->cancel();
  communication_timer_->cancel();

  remote_ip_ =
      this->node->declare_parameter<std::string>("remote_ip", "192.168.4.1");
  device_name_ = this->node->declare_parameter<std::string>("device_name", "");
  auto mac = this->node->declare_parameter<std::string>("device_mac", "");
  if (!mac.empty() && !parseMac(mac, &mac_filter_)) {
    throw std::invalid_argument("device_mac must be aa:bb:cc:dd:ee:ff");
  }
  mac_filter_valid_ = !mac.empty();

  command_port_ = this->node->declare_parameter<int>("command_port", 5000);
  management_port_ =
      this->node->declare_parameter<int>("management_port", 5003);
  bind_telemetry_port_ =
      this->node->declare_parameter<int>("bind_telemetry_port", 5001);
  bind_lidar_port_ =
      this->node->declare_parameter<int>("bind_lidar_port", 5002);
  max_v_ = this->node->declare_parameter<double>("max_linear_velocity", 1.5);
  max_w_ = this->node->declare_parameter<double>("max_angular_velocity", 2.0);
  timeout_ms_ = this->node->declare_parameter<int>("command_timeout_ms", 500);
  lidar_frame_ =
      this->node->declare_parameter<std::string>("lidar_frame", "laser");
  auto initial_session = this->node->declare_parameter<int>("session_id", 0);
  if (initial_session < 0) {
    throw std::invalid_argument("session_id must be non-negative");
  }
  session_ = static_cast<uint32_t>(initial_session);

  if (command_port_ < 1 || command_port_ > 65535 || management_port_ < 1 ||
      management_port_ > 65535 || bind_telemetry_port_ < 1 ||
      bind_telemetry_port_ > 65535 || bind_lidar_port_ < 1 ||
      bind_lidar_port_ > 65535 || timeout_ms_ <= 0 || timeout_ms_ > 65535 ||
      max_v_ < 0.0 || max_w_ < 0.0) {
    throw std::invalid_argument("invalid mini_v1_5 parameter range");
  }

  scan_pub_ = this->node->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS());
  lidar_points_pub_ =
      this->node->create_publisher<sensor_msgs::msg::PointCloud2>(
          "lidar_points", rclcpp::SensorDataQoS());
  raw_lidar_pub_ = this->node->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "lidar_raw", rclcpp::SensorDataQoS());
  vehicle_state_pub_ =
      this->node->create_publisher<std_msgs::msg::UInt32>("vehicle_state", 10);
  link_pub_ = this->node->create_publisher<std_msgs::msg::Bool>("spi_link", 10);
  battery_pub_ = this->node->create_publisher<sensor_msgs::msg::BatteryState>(
      "battery_state", 10);

  mini_cmd_vel_sub_ =
      this->node->create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", 10,
          std::bind(&TianbotMiniV15::velocityCallback, this,
                    std::placeholders::_1));

  rx_timer_ = this->node->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&TianbotMiniV15::receive, this));
  safety_timer_ = this->node->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&TianbotMiniV15::safetyCheck, this));
  session_timer_ = this->node->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TianbotMiniV15::sessionTick, this));

  initDone_ = true;
}

TianbotMiniV15::~TianbotMiniV15() {
  sendStop(4);
  if (telemetry_fd_ >= 0) {
    ::close(telemetry_fd_);
  }
  if (lidar_fd_ >= 0) {
    ::close(lidar_fd_);
  }
}

void TianbotMiniV15::open(void) {
  RCLCPP_INFO(node->get_logger(), "%sMini V1.5 driver starting%s", kCyan,
              kReset);
  RCLCPP_INFO(node->get_logger(), "  remote_ip: %s", remote_ip_.c_str());
  RCLCPP_INFO(node->get_logger(), "  device_name: %s",
              device_name_.empty() ? "any" : device_name_.c_str());
  RCLCPP_INFO(node->get_logger(), "  command_port: %d", command_port_);
  RCLCPP_INFO(node->get_logger(), "  management_port: %d", management_port_);
  RCLCPP_INFO(node->get_logger(), "  telemetry_port: %d", bind_telemetry_port_);
  RCLCPP_INFO(node->get_logger(), "  lidar_port: %d", bind_lidar_port_);
  RCLCPP_INFO(node->get_logger(), "  command_timeout_ms: %d", timeout_ms_);
  RCLCPP_INFO(node->get_logger(), "  max_linear_velocity: %.3f", max_v_);
  RCLCPP_INFO(node->get_logger(), "  max_angular_velocity: %.3f", max_w_);
  RCLCPP_INFO(node->get_logger(), "  lidar_frame: %s", lidar_frame_.c_str());
  openSockets();
  started_ = std::chrono::steady_clock::now();
}

void TianbotMiniV15::openSockets() {
  telemetry_fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  lidar_fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (telemetry_fd_ < 0 || lidar_fd_ < 0) {
    if (telemetry_fd_ >= 0) {
      ::close(telemetry_fd_);
    }
    if (lidar_fd_ >= 0) {
      ::close(lidar_fd_);
    }
    telemetry_fd_ = lidar_fd_ = -1;
    RCLCPP_FATAL(this->node->get_logger(), "%sCannot create UDP socket%s", kRed,
                 kReset);
    exit(-1);
  }

  // C5 会在一个周期内连续发送状态、IMU、里程计和雷达分片。扩大接收
  // 缓冲区，配合 receiveFrom() 排空 socket，避免单次回调只取一个包时
  // 在高峰期积压导致状态帧或雷达分片丢失。
  constexpr int kReceiveBufferBytes = 256 * 1024;
  (void)::setsockopt(telemetry_fd_, SOL_SOCKET, SO_RCVBUF, &kReceiveBufferBytes,
                     sizeof(kReceiveBufferBytes));
  (void)::setsockopt(lidar_fd_, SOL_SOCKET, SO_RCVBUF, &kReceiveBufferBytes,
                     sizeof(kReceiveBufferBytes));

  remote_ = {};
  remote_.sin_family = AF_INET;
  remote_.sin_port = htons(static_cast<uint16_t>(command_port_));
  if (inet_pton(AF_INET, remote_ip_.c_str(), &remote_.sin_addr) != 1) {
    RCLCPP_FATAL(this->node->get_logger(), "%sInvalid remote_ip: %s%s", kRed,
                 remote_ip_.c_str(), kReset);
    exit(-1);
  }

  auto bindSocket = [this](int fd, int port) {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(static_cast<uint16_t>(port));
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) != 0) {
      RCLCPP_FATAL(
          this->node->get_logger(),
          "%sCannot bind UDP port %d; stop other scan or driver processes%s",
          kRed, port, kReset);
      exit(-1);
    }
  };
  bindSocket(telemetry_fd_, bind_telemetry_port_);
  bindSocket(lidar_fd_, bind_lidar_port_);

  management_ = remote_;
  management_.sin_port = htons(static_cast<uint16_t>(management_port_));

  RCLCPP_INFO(node->get_logger(),
              "%sUDP transport ready; searching for vehicle%s", kGreen, kReset);
}

void TianbotMiniV15::sendFrame(std::uint8_t type,
                               const std::vector<std::uint8_t> &payload,
                               bool management) {
  if (telemetry_fd_ < 0) {
    return;
  }
  std::vector<std::uint8_t> buf;
  if (!mini::encode_frame(type, 0, seq_++, payload, &buf)) {
    return;
  }
  const sockaddr_in &addr = management ? management_ : remote_;
  ::sendto(telemetry_fd_, buf.data(), buf.size(), 0,
           reinterpret_cast<const sockaddr *>(&addr), sizeof(addr));
}

void TianbotMiniV15::velocityCallback(
    const geometry_msgs::msg::Twist::ConstSharedPtr &msg) {
  if (!online_ || !session_ || !std::isfinite(msg->linear.x) ||
      !std::isfinite(msg->angular.z)) {
    return;
  }
  const float v =
      static_cast<float>(std::clamp(msg->linear.x, -max_v_, max_v_));
  const float w =
      static_cast<float>(std::clamp(msg->angular.z, -max_w_, max_w_));

  std::vector<std::uint8_t> p(18);
  mini::write_u32(p.data(), session_);
  mini::write_u32(p.data() + 4, cmd_seq_++);
  std::memcpy(p.data() + 8, &v, 4);
  std::memcpy(p.data() + 12, &w, 4);
  mini::write_u16(p.data() + 16, static_cast<uint16_t>(timeout_ms_));
  sendFrame(mini::kVelocity, p, false);

  session_activity_ = std::chrono::steady_clock::now();
  last_cmd_ = this->node->now();
  if (!active_)
    control_started_ = std::chrono::steady_clock::now();
  active_ = true;
}

void TianbotMiniV15::sendStop(std::uint8_t reason) {
  if (!session_ || telemetry_fd_ < 0) {
    return;
  }
  std::vector<std::uint8_t> p(5);
  mini::write_u32(p.data(), session_);
  p[4] = reason;
  sendFrame(mini::kStop, p, false);
  active_ = false;
  session_ = 0; // STOP 会释放 C5 租约，下次控制前必须重新握手。
}

void TianbotMiniV15::sendHeartbeat() {
  if (!found_ || telemetry_fd_ < 0 || mac_ == std::array<std::uint8_t, 6>{})
    return;
  std::vector<std::uint8_t> p(10, 0);
  std::copy(mac_.begin(), mac_.end(), p.begin());
  p[6] = 2; // 上位机存活；不代表电机控制权。
  sendFrame(mini::kHeartbeat, p, true);
}

void TianbotMiniV15::sessionTick() {
  const auto now = std::chrono::steady_clock::now();
  if (found_ && (last_heartbeat_.time_since_epoch().count() == 0 ||
                 now - last_heartbeat_ >= std::chrono::milliseconds(200))) {
    sendHeartbeat();
    last_heartbeat_ = now;
  }
  if (!found_) {
    if (!missing_reported_ && now - started_ >= std::chrono::seconds(5)) {
      missing_reported_ = true;
      RCLCPP_ERROR(node->get_logger(),
                   "%sVehicle not found: %s (still searching)%s", kRed,
                   remote_ip_.c_str(), kReset);
    }
    std::vector<std::uint8_t> p(4);
    mini::write_u32(p.data(), request_++);
    pending_request_ = mini::read_u32(p.data());
    sendFrame(mini::kDiscoverRequest, p, true);
    return;
  }
  if (session_) {
    if (std::chrono::steady_clock::now() - session_activity_ <
        std::chrono::milliseconds(400))
      return;
    session_ = 0; // 本地提前放弃闲置租约，不延长下位机保护窗口。
  }
  std::vector<std::uint8_t> p(13);
  std::copy(mac_.begin(), mac_.end(), p.begin());
  mini::write_u32(p.data() + 6, pending_session_);
  mini::write_u16(p.data() + 10, static_cast<uint16_t>(timeout_ms_));
  p[12] = 0;
  sendFrame(mini::kSessionAck, p, true);
  session_ = pending_session_;
  if (session_)
    session_activity_ = std::chrono::steady_clock::now();
  pending_session_ = 0;
}

void TianbotMiniV15::safetyCheck() {
  const auto now = this->node->now();
  const auto age = last_cmd_.nanoseconds() == 0
                       ? static_cast<int64_t>(timeout_ms_) + 1
                       : (now - last_cmd_).nanoseconds() / 1000000;
  if (active_ && age > timeout_ms_) {
    sendStop(2);
  }
  if (online_ && (now - last_rx_).nanoseconds() > 3000LL * 1000000LL) {
    online_ = false;
    publishLink(false);
    sendStop(3);
    found_ = false;
    session_ = pending_session_ = 0;
  }
}

void TianbotMiniV15::receive() {
  receiveFrom(telemetry_fd_);
  receiveFrom(lidar_fd_);
}

void TianbotMiniV15::receiveFrom(int fd) {
  if (fd < 0)
    return;
  std::array<std::uint8_t, 1216> b{};
  // 限制单回调工作量，持续 UDP 输入时也给控制和安全定时器执行机会。
  for (unsigned drained = 0; drained < 256; ++drained) {
    sockaddr_in source{};
    socklen_t source_len = sizeof(source);
    const auto n =
        ::recvfrom(fd, b.data(), b.size(), MSG_DONTWAIT,
                   reinterpret_cast<sockaddr *>(&source), &source_len);
    if (n <= 0)
      return;
    if (source.sin_addr.s_addr != remote_.sin_addr.s_addr)
      continue;

    mini::Frame x{};
    if (!mini::decode_frame(b.data(), static_cast<size_t>(n), &x))
      continue;

    if (x.type == mini::kDiscoverResponse) {
      mini::DeviceInfo d{};
      if (!mini::parse_discovery(x, &d) ||
          mini::read_u32(x.payload.data()) != pending_request_ ||
          (!device_name_.empty() && d.name != device_name_) ||
          (mac_filter_valid_ && d.mac != mac_filter_))
        continue;
      mac_ = d.mac;
      if (!found_)
        RCLCPP_INFO(node->get_logger(), "%sVehicle discovered: %s ip=%s%s",
                    kGreen, d.name.c_str(), remote_ip_.c_str(), kReset);
      found_ = true;
      missing_reported_ = false;
      publishVehicleState(d.state, d.faults, 0, 0, false);
      continue;
    }
    if (x.type == mini::kSessionGrant && x.payload.size() == 13 &&
        x.payload[12] == 0 &&
        std::equal(mac_.begin(), mac_.end(), x.payload.begin())) {
      const auto granted = mini::read_u32(x.payload.data() + 6);
      if (granted != 0)
        pending_session_ = granted;
      continue;
    }
    if (x.type == mini::kHeartbeat && x.payload.size() == 10 &&
        std::equal(mac_.begin(), mac_.end(), x.payload.begin())) {
      online_ = true;
      last_rx_ = this->node->now();
      publishLink(true);
      // 心跳只表示在线；车辆状态话题仅统计真实状态遥测。
      continue;
    }
    if (x.type == mini::kVehicleState && x.payload.size() == 6) {
      // 持续输入期间下位机仍为 READY，说明租约/控制已被撤销。
      // 清会话后重新握手，只允许后续新 cmd_vel 继续，不重放旧速度。
      if (active_ && x.payload[0] == 2 &&
          std::chrono::steady_clock::now() - control_started_ >
              std::chrono::milliseconds(500)) {
        session_ = pending_session_ = 0;
        active_ = false;
      }
      online_ = true;
      last_rx_ = this->node->now();
      publishLink(true);
      publishVehicleState(x.payload[0], mini::read_u16(x.payload.data() + 1),
                          mini::read_u16(x.payload.data() + 3), x.payload[5],
                          true);
      continue;
    }
    if (x.type == mini::kOdometry && x.payload.size() == 28) {
      online_ = true;
      last_rx_ = this->node->now();
      const float yaw = loadValue<float>(x.payload.data() + 16);
      publishOdom(loadValue<float>(x.payload.data() + 8),
                  loadValue<float>(x.payload.data() + 12), 0.0,
                  createQuaternionMsgFromYaw(yaw),
                  loadValue<float>(x.payload.data() + 20), 0.0, 0.0, 0.0, 0.0,
                  loadValue<float>(x.payload.data() + 24));
      continue;
    }
    if (x.type == mini::kImu &&
        (x.payload.size() == 32 || x.payload.size() == 48)) {
      online_ = true;
      last_rx_ = this->node->now();
      geometry_msgs::msg::Quaternion orientation;
      orientation.x = loadValue<float>(x.payload.data() + 8);
      orientation.y = loadValue<float>(x.payload.data() + 12);
      orientation.z = loadValue<float>(x.payload.data() + 16);
      orientation.w = loadValue<float>(x.payload.data() + 20);
      const double nan = std::numeric_limits<double>::quiet_NaN();
      if (x.payload.size() == 48) {
        publishImu(loadValue<float>(x.payload.data() + 36),
                   loadValue<float>(x.payload.data() + 40),
                   loadValue<float>(x.payload.data() + 44),
                   loadValue<float>(x.payload.data() + 24),
                   loadValue<float>(x.payload.data() + 28),
                   loadValue<float>(x.payload.data() + 32), &orientation);
      } else {
        publishImu(nan, nan, nan, nan, nan, nan, &orientation);
      }
      continue;
    }
    if (x.type == mini::kLidarRaw)
      handleRawLidar(x);
    // 0x40 是网页 90 格显示快照；建图 scan 只取原始流，不能混发。
  }
}

void TianbotMiniV15::publishLink(bool up) {
  if (up != link_log_online_) {
    link_log_online_ = up;
    if (up)
      RCLCPP_INFO(node->get_logger(), "%sVehicle online: %s%s", kGreen,
                  remote_ip_.c_str(), kReset);
    else
      RCLCPP_ERROR(node->get_logger(),
                   "%sVehicle offline: %s (no telemetry for 3s)%s", kRed,
                   remote_ip_.c_str(), kReset);
  }
  std_msgs::msg::Bool m;
  m.data = up;
  link_pub_->publish(m);
}

void TianbotMiniV15::publishVehicleState(std::uint8_t state,
                                         std::uint16_t faults,
                                         std::uint16_t millivolts,
                                         std::uint8_t source,
                                         bool hasBatteryVoltage) {
  std_msgs::msg::UInt32 m;
  m.data = state | (uint32_t(faults) << 8) | (uint32_t(source) << 24);
  vehicle_state_pub_->publish(m);

  if (hasBatteryVoltage) {
    // Keep the chassis-wide voltage topic (Float32, volts) consistent with
    // the other vehicle types, and additionally expose BatteryState so that
    // consumers of the former gateway topic keep working.
    publishVoltage(static_cast<float>(millivolts) / 1000.0f);

    sensor_msgs::msg::BatteryState b;
    b.header.stamp = this->node->now();
    b.header.frame_id = baseFrame();
    b.voltage = static_cast<float>(millivolts) / 1000.0f;
    b.percentage = std::numeric_limits<float>::quiet_NaN();
    battery_pub_->publish(b);
  }
}

void TianbotMiniV15::handleLidarFragment(
    const std::vector<std::uint8_t> &payload) {
  if (payload.size() < 12) {
    return;
  }
  const auto id = mini::read_u32(payload.data());
  const auto idx = mini::read_u16(payload.data() + 4);
  const auto count = mini::read_u16(payload.data() + 6);
  const auto start = mini::read_u16(payload.data() + 8);
  const auto bins = payload[10];
  if (!count || count > 90 || idx >= count || !bins || payload[11] != 0 ||
      start + bins > 90 || payload.size() != 12 + size_t(bins) * 2) {
    return;
  }
  if (id != scan_id_ || count != frag_count_) {
    scan_id_ = id;
    frag_count_ = count;
    seen_ = 0;
    ranges_.assign(90, NAN);
    got_.assign(count, false);
    scan_started_ = this->node->now();
  }
  // 已完成或超时的同 ID 包直接丢弃，避免访问已清空的重组数组。
  if (got_.size() != count || got_[idx]) {
    return;
  }
  got_[idx] = true;
  seen_++;
  for (uint16_t i = 0; i < bins; i++) {
    const auto mm = mini::read_u16(payload.data() + 12 + 2 * i);
    ranges_[start + i] = mm ? static_cast<float>(mm) / 1000.0f : NAN;
  }
  if (seen_ != frag_count_) {
    if ((this->node->now() - scan_started_).nanoseconds() > 200000000LL) {
      ranges_.clear();
      got_.clear();
      seen_ = 0;
    }
    return;
  }
  publishScan();
  ranges_.clear();
  got_.clear();
  seen_ = 0;
}

void TianbotMiniV15::publishScan() {
  sensor_msgs::msg::LaserScan m;
  m.header.stamp = this->node->now();
  m.header.frame_id = lidar_frame_;
  m.angle_min = 0;
  m.angle_increment = static_cast<float>(4 * M_PI / 180);
  m.angle_max = m.angle_increment * 89;
  m.scan_time = 0.1f; // C5 发布的是 10 Hz 滚动快照，非逐点采样时刻。
  m.range_min = 0.02f;
  m.range_max = 12.0f;
  m.ranges = ranges_;
  scan_pub_->publish(m);
}

void TianbotMiniV15::handleRawLidar(const mini::Frame &frame) {
  const auto now = this->node->now();
  mini::LidarScanAssembler::Scan scan;
  const auto previous_bad = raw_scan_.bad;
  const double local_s =
      std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  const bool ready = raw_scan_.push(frame, local_s, scan);
  if (raw_scan_.bad == previous_bad) {
    std_msgs::msg::UInt8MultiArray raw;
    raw.data = frame.payload; // 原始角度、强度、距离及校验，逐帧保留。
    raw_lidar_pub_->publish(raw);
  }
  if (ready) {
    sensor_msgs::msg::LaserScan msg;
    // 雷达没有硬件时间戳：使用本地到达时间估算首点时刻，不能当作跨板延迟。
    msg.header.stamp = now - rclcpp::Duration::from_seconds(scan.duration_s);
    msg.header.frame_id = lidar_frame_;
    // 原始角采样非完全等间距，先发布无降采样点云，再映射标准 LaserScan。
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg.header;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                             "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z",
                             1, sensor_msgs::msg::PointField::FLOAT32,
                             "intensity", 1,
                             sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(scan.ranges.size());
    cloud.is_dense = false;
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
        z(cloud, "z"), intensity(cloud, "intensity");
    for (size_t i = 0; i < scan.ranges.size();
         ++i, ++x, ++y, ++z, ++intensity) {
      *x = std::isfinite(scan.ranges[i])
               ? scan.ranges[i] * std::cos(scan.angles[i])
               : NAN;
      *y = std::isfinite(scan.ranges[i])
               ? scan.ranges[i] * std::sin(scan.angles[i])
               : NAN;
      *z = 0;
      *intensity = scan.intensities[i];
    }
    lidar_points_pub_->publish(cloud);
    // 0.25°网格比典型0.9°原始间距更细；每点投到最近格，空格为无回波。
    constexpr size_t bins = 1440;
    msg.angle_min = 0;
    msg.angle_increment = 2 * M_PI / bins;
    msg.angle_max = (bins - 1) * msg.angle_increment;
    msg.scan_time = scan.duration_s;
    msg.time_increment = 0; // 网格不是原始采样次序，不伪造逐点时间。
    msg.range_min = 0.02f;
    msg.range_max = 12.0f;
    msg.ranges.assign(bins, INFINITY);
    msg.intensities.assign(bins, 0);
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const size_t bin = static_cast<size_t>(std::lround(scan.angles[i] /
                                                         msg.angle_increment)) %
                         bins;
      if (scan.ranges[i] < msg.ranges[bin]) {
        msg.ranges[bin] = scan.ranges[i];
        msg.intensities[bin] = scan.intensities[i];
      }
    }
    scan_pub_->publish(msg);
  }
  if ((now - raw_diag_time_).seconds() >= 1.0) {
    raw_diag_time_ = now;
    RCLCPP_DEBUG(this->node->get_logger(),
                 "lidar raw=%llu points=%llu scan=%llu incomplete=%llu "
                 "bad=%llu seq_anomaly=%llu",
                 (unsigned long long)raw_scan_.frames,
                 (unsigned long long)raw_scan_.points,
                 (unsigned long long)raw_scan_.completed,
                 (unsigned long long)raw_scan_.incomplete,
                 (unsigned long long)raw_scan_.bad,
                 (unsigned long long)raw_scan_.sequence_gaps);
  }
}
