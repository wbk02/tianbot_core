#include "tianbot_core/mini_v1_protocol.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
namespace mini = tianbot_core::mini_v1_5;
namespace {
template <class T> T load(const uint8_t *p) {
  T v{};
  std::memcpy(&v, p, sizeof(v));
  return v;
}
bool parse_mac(const std::string &s, std::array<uint8_t, 6> *m) {
  unsigned v[6]{};
  if (!m || s.empty() ||
      std::sscanf(s.c_str(), "%2x:%2x:%2x:%2x:%2x:%2x", v, v + 1, v + 2, v + 3,
                  v + 4, v + 5) != 6)
    return false;
  for (size_t i = 0; i < 6; i++)
    (*m)[i] = static_cast<uint8_t>(v[i]);
  return true;
}
} // namespace
class MiniGateway final : public rclcpp::Node {
public:
  MiniGateway() : Node("mini_v1_5_gateway") {
    remote_ip_ = declare_parameter<std::string>("remote_ip", "192.168.4.1");
    device_name_ = declare_parameter<std::string>("device_name", "");
    auto mac = declare_parameter<std::string>("device_mac", "");
    mac_filter_valid_ = parse_mac(mac, &mac_filter_);
    command_port_ = declare_parameter<int>("command_port", 5000);
    management_port_ = declare_parameter<int>("management_port", 5003);
    bind_telemetry_port_ = declare_parameter<int>(
        "bind_telemetry_port", declare_parameter<int>("telemetry_port", 5001));
    bind_lidar_port_ = declare_parameter<int>(
        "bind_lidar_port", declare_parameter<int>("lidar_port", 5002));
    max_v_ = declare_parameter<double>("max_linear_velocity", 1.5);
    max_w_ = declare_parameter<double>("max_angular_velocity", 2.0);
    timeout_ms_ = declare_parameter<int>("command_timeout_ms", 500);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "laser");
    session_ = static_cast<uint32_t>(declare_parameter<int>("session_id", 0));
    open();
    odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    scan_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    state_ = create_publisher<std_msgs::msg::UInt32>("vehicle_state", 10);
    battery_ =
        create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    link_ = create_publisher<std_msgs::msg::Bool>("spi_link", 10);
    cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr m) { velocity(*m); });
    rx_ =
        create_wall_timer(std::chrono::milliseconds(10), [this] { receive(); });
    tick_ =
        create_wall_timer(std::chrono::milliseconds(50), [this] { safety(); });
    discover_ = create_wall_timer(std::chrono::milliseconds(500),
                                  [this] { session_tick(); });
  }
  ~MiniGateway() override {
    stop(4);
    if (fd_ >= 0)
      close(fd_);
    if (lidar_fd_ >= 0)
      close(lidar_fd_);
  }

private:
  void open() {
    fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    lidar_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd_ < 0 || lidar_fd_ < 0)
      throw std::runtime_error("cannot create UDP socket");
    remote_ = {};
    remote_.sin_family = AF_INET;
    remote_.sin_port = htons(command_port_);
    if (inet_pton(AF_INET, remote_ip_.c_str(), &remote_.sin_addr) != 1)
      throw std::runtime_error("invalid remote_ip");
    bind(fd_, bind_telemetry_port_);
    bind(lidar_fd_, bind_lidar_port_);
    management_ = remote_;
    management_.sin_port = htons(management_port_);
  }
  void bind(int f, int p) {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(static_cast<uint16_t>(p));
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(f, reinterpret_cast<sockaddr *>(&a), sizeof(a)) != 0)
      throw std::runtime_error("cannot bind UDP socket");
  }
  void send(uint8_t type, const std::vector<uint8_t> &p, bool mgmt = false) {
    std::vector<uint8_t> b;
    if (!mini::encode_frame(type, 0, seq_++, p, &b))
      return;
    auto &a = mgmt ? management_ : remote_;
    sendto(fd_, b.data(), b.size(), 0, reinterpret_cast<sockaddr *>(&a),
           sizeof(a));
  }
  void velocity(const geometry_msgs::msg::Twist &m) {
    if (!online_ || !session_ || !std::isfinite(m.linear.x) ||
        !std::isfinite(m.angular.z))
      return;
    float v = static_cast<float>(std::clamp(m.linear.x, -max_v_, max_v_)),
          w = static_cast<float>(std::clamp(m.angular.z, -max_w_, max_w_));
    std::vector<uint8_t> p(18);
    mini::write_u32(p.data(), session_);
    mini::write_u32(p.data() + 4, cmd_seq_++);
    std::memcpy(p.data() + 8, &v, 4);
    std::memcpy(p.data() + 12, &w, 4);
    mini::write_u16(p.data() + 16, static_cast<uint16_t>(timeout_ms_));
    send(mini::kVelocity, p);
    last_cmd_ = now();
    active_ = true;
  }
  void stop(uint8_t reason) {
    if (!session_)
      return;
    std::vector<uint8_t> p(5);
    mini::write_u32(p.data(), session_);
    p[4] = reason;
    send(mini::kStop, p);
    active_ = false;
  }
  void session_tick() {
    if (!found_) {
      std::vector<uint8_t> p(4);
      mini::write_u32(p.data(), request_++);
      pending_request_ = mini::read_u32(p.data());
      send(mini::kDiscoverRequest, p, true);
      return;
    }
    if (session_)
      return;
    std::vector<uint8_t> p(13);
    std::copy(mac_.begin(), mac_.end(), p.begin());
    mini::write_u32(p.data() + 6, pending_session_);
    mini::write_u16(p.data() + 10, static_cast<uint16_t>(timeout_ms_));
    p[12] = 0;
    send(mini::kSessionAck, p, true);
    if (pending_session_ != 0) {
      session_ = pending_session_;
      pending_session_ = 0;
    }
  }
  void safety() {
    auto age = last_cmd_.nanoseconds() == 0
                   ? timeout_ms_ + 1
                   : (now() - last_cmd_).nanoseconds() / 1000000;
    if (active_ && age > timeout_ms_)
      stop(2);
    if (online_ && (now() - last_rx_).nanoseconds() > 3000LL * 1000000LL) {
      online_ = false;
      publish_link(false);
      stop(3);
    }
  }
  void receive() {
    recv(fd_);
    recv(lidar_fd_);
  }
  void recv(int f) {
    std::array<uint8_t, 1216> b{};
    auto n = recvfrom(f, b.data(), b.size(), MSG_DONTWAIT, nullptr, nullptr);
    if (n <= 0)
      return;
    mini::Frame x{};
    if (!mini::decode_frame(b.data(), static_cast<size_t>(n), &x))
      return;
    if (x.type == mini::kDiscoverResponse) {
      mini::DeviceInfo d{};
      if (!mini::parse_discovery(x, &d) ||
          mini::read_u32(x.payload.data()) != pending_request_ ||
          (!device_name_.empty() && d.name != device_name_) ||
          (mac_filter_valid_ && d.mac != mac_filter_))
        return;
      mac_ = d.mac;
      found_ = true;
      publish_state(d.state, d.faults, 0, 0);
      return;
    }
    if (x.type == mini::kSessionGrant && x.payload.size() == 13 &&
        x.payload[12] == 0 &&
        std::equal(mac_.begin(), mac_.end(), x.payload.begin())) {
      pending_session_ = mini::read_u32(x.payload.data() + 6);
      return;
    }
    if (x.type == mini::kHeartbeat && x.payload.size() == 10 &&
        std::equal(mac_.begin(), mac_.end(), x.payload.begin())) {
      online_ = true;
      last_rx_ = now();
      publish_link(true);
      publish_state(x.payload[6], mini::read_u16(x.payload.data() + 7), 0,
                    x.payload[9]);
      return;
    }
    if (x.type == mini::kVehicleState && x.payload.size() == 6) {
      online_ = true;
      last_rx_ = now();
      publish_link(true);
      publish_state(x.payload[0], mini::read_u16(x.payload.data() + 1),
                    mini::read_u16(x.payload.data() + 3), x.payload[5]);
      return;
    }
    if (x.type == mini::kOdometry && x.payload.size() == 28) {
      last_rx_ = now();
      online_ = true;
      publish_odom(x.payload);
      return;
    }
    if (x.type == mini::kImu && x.payload.size() == 32) {
      last_rx_ = now();
      online_ = true;
      publish_imu(x.payload);
      return;
    }
    if (x.type == mini::kLidarFragment)
      lidar(x.payload);
  }
  void publish_link(bool v) {
    std_msgs::msg::Bool m;
    m.data = v;
    link_->publish(m);
  }
  void publish_state(uint8_t s, uint16_t f, uint16_t mv, uint8_t src) {
    std_msgs::msg::UInt32 m;
    m.data = s | (uint32_t(f) << 8) | (uint32_t(src) << 24);
    state_->publish(m);
    sensor_msgs::msg::BatteryState b;
    b.header.stamp = now();
    b.header.frame_id = base_frame_;
    b.voltage = float(mv) / 1000.f;
    b.percentage = std::numeric_limits<float>::quiet_NaN();
    battery_->publish(b);
  }
  void publish_odom(const std::vector<uint8_t> &p) {
    nav_msgs::msg::Odometry m;
    m.header.stamp = now();
    m.header.frame_id = "odom";
    m.child_frame_id = base_frame_;
    float yaw = load<float>(p.data() + 16);
    m.pose.pose.position.x = load<float>(p.data() + 8);
    m.pose.pose.position.y = load<float>(p.data() + 12);
    m.pose.pose.orientation.z = sin(yaw / 2);
    m.pose.pose.orientation.w = cos(yaw / 2);
    m.twist.twist.linear.x = load<float>(p.data() + 20);
    m.twist.twist.angular.z = load<float>(p.data() + 24);
    odom_->publish(m);
  }
  void publish_imu(const std::vector<uint8_t> &p) {
    sensor_msgs::msg::Imu m;
    m.header.stamp = now();
    m.header.frame_id = base_frame_;
    m.linear_acceleration.x = load<float>(p.data() + 8);
    m.linear_acceleration.y = load<float>(p.data() + 12);
    m.linear_acceleration.z = load<float>(p.data() + 16);
    m.angular_velocity.x = load<float>(p.data() + 20);
    m.angular_velocity.y = load<float>(p.data() + 24);
    m.angular_velocity.z = load<float>(p.data() + 28);
    imu_->publish(m);
  }
  void lidar(const std::vector<uint8_t> &p) {
    if (p.size() < 12)
      return;
    auto id = mini::read_u32(p.data());
    auto idx = mini::read_u16(p.data() + 4);
    auto cnt = mini::read_u16(p.data() + 6);
    auto start = mini::read_u16(p.data() + 8);
    auto bins = p[10];
    if (!cnt || idx >= cnt || !bins || start + bins > 90 ||
        p.size() != 12 + size_t(bins) * 2)
      return;
    if (id != scan_id_ || cnt != frag_count_) {
      scan_id_ = id;
      frag_count_ = cnt;
      seen_ = 0;
      ranges_.assign(90, NAN);
      got_.assign(cnt, false);
      scan_started_ = now();
    }
    if (got_[idx])
      return;
    got_[idx] = true;
    seen_++;
    for (uint16_t i = 0; i < bins; i++) {
      auto mm = mini::read_u16(p.data() + 12 + 2 * i);
      ranges_[start + i] = mm ? float(mm) / 1000.f : NAN;
    }
    if (seen_ != frag_count_) {
      if ((now() - scan_started_).nanoseconds() > 200000000) {
        ranges_.clear();
        got_.clear();
        seen_ = 0;
      }
      return;
    }
    sensor_msgs::msg::LaserScan m;
    m.header.stamp = now();
    m.header.frame_id = lidar_frame_;
    m.angle_min = 0;
    m.angle_increment = float(4 * M_PI / 180);
    m.angle_max = m.angle_increment * 89;
    m.range_min = .02;
    m.range_max = 12;
    m.ranges = ranges_;
    scan_->publish(m);
    ranges_.clear();
    got_.clear();
    seen_ = 0;
  }
  int fd_{-1}, lidar_fd_{-1}, command_port_{5000}, management_port_{5003},
      bind_telemetry_port_{5001}, bind_lidar_port_{5002};
  sockaddr_in remote_{}, management_{};
  std::string remote_ip_, device_name_, base_frame_, lidar_frame_;
  double max_v_{1.5}, max_w_{2};
  int timeout_ms_{500};
  uint32_t session_{0}, pending_session_{0}, seq_{1}, cmd_seq_{1}, request_{1},
      pending_request_{0}, scan_id_{0};
  bool found_{false}, online_{false}, active_{false}, mac_filter_valid_{false};
  std::array<uint8_t, 6> mac_{}, mac_filter_{};
  uint16_t frag_count_{0}, seen_{0};
  std::vector<bool> got_;
  std::vector<float> ranges_;
  rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME}, last_rx_{0, 0, RCL_ROS_TIME},
      scan_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr state_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  rclcpp::TimerBase::SharedPtr rx_, tick_, discover_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MiniGateway>());
  } catch (const std::exception &e) {
    std::fprintf(stderr, "%s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
