#include "tianbot_core/mini_v1_5_multi.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sys/socket.h>
#include <unistd.h>

namespace mini = tianbot_core::mini_v1_5;
namespace {
/* 错误结束后恢复终端默认颜色，避免后续普通日志被染红。 */
constexpr const char *kReset = "\033[0m";
constexpr const char *kRed = "\033[31m";
constexpr const char *kYellow = "";
constexpr const char *kGreen = "";
/* 所有普通状态使用终端默认白色，仅错误保留红色。 */
constexpr const char *kCyan = "";
bool g_use_color = false;
const char *color(const char *value) { return g_use_color ? value : ""; }
std::string macKey(const std::array<uint8_t, 6> &m) {
  char b[18];
  std::snprintf(b, sizeof b, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2],
                m[3], m[4], m[5]);
  return b;
}
std::string ipOf(const sockaddr_in &a) {
  char b[INET_ADDRSTRLEN]{};
  inet_ntop(AF_INET, &a.sin_addr, b, sizeof b);
  return b;
}
template <class T> T load(const uint8_t *p) {
  T v{};
  std::memcpy(&v, p, sizeof v);
  return v;
}
} // namespace
TianbotMiniV15Multi::TianbotMiniV15Multi(const std::shared_ptr<rclcpp::Node> &n)
    : TianbotCore(n) {
  const char *color_override = std::getenv("RCUTILS_COLORIZED_OUTPUT");
  g_use_color = std::getenv("NO_COLOR") == nullptr &&
                (color_override ? std::string(color_override) == "1"
                                : ::isatty(STDOUT_FILENO));
  // 多车 Mini 使用自己的 UDP 发现、遥测和会话定时器；关闭基类面向
  // 串口/旧 UDP 底盘的心跳与通信超时定时器，避免启动后误报通信错误。
  heartbeat_timer_->cancel();
  communication_timer_->cancel();
  command_port_ = n->declare_parameter("command_port", 5000);
  management_port_ = n->declare_parameter("management_port", 5003);
  telemetry_port_ = n->declare_parameter("bind_telemetry_port", 5001);
  lidar_port_ = n->declare_parameter("bind_lidar_port", 5002);
  vehicle_ips_ = n->declare_parameter<std::string>("vehicle_ips", "");
  discovery_address_ =
      n->declare_parameter<std::string>("discovery_address", "");
  scan_only_ = n->declare_parameter("scan_only", false);
  publish_lidar_debug_ = n->declare_parameter("publish_lidar_debug", false);
  max_vehicles_ = n->declare_parameter("max_vehicles", 5);
  timeout_ms_ = n->declare_parameter("command_timeout_ms", 500);
  max_v_ = n->declare_parameter("max_linear_velocity", 1.5);
  max_w_ = n->declare_parameter("max_angular_velocity", 2.0);
  /* main.cpp 已为兼容旧底盘预声明这三个公共 frame 参数；多车模式读取
   * 已存在参数，避免重复 declare 导致节点启动即退出。 */
  (void)n->get_parameter("base_frame", frame_base_);
  (void)n->get_parameter("odom_frame", frame_odom_);
  (void)n->get_parameter("imu_frame", frame_imu_);
  frame_lidar_ = n->declare_parameter<std::string>("lidar_frame", "laser");
  if (!scan_only_ && vehicle_ips_.empty())
    throw std::invalid_argument("vehicle_ips is required for mini_v1_5_multi");
  if (vehicle_ips_ != "*") {
    size_t b = 0;
    while (b < vehicle_ips_.size()) {
      auto e = vehicle_ips_.find(',', b);
      ips_.push_back(
          vehicle_ips_.substr(b, e == std::string::npos ? e : e - b));
      if (e == std::string::npos)
        break;
      b = e + 1;
    }
  }
  const auto valid_port = [](int port) { return port > 0 && port <= 65535; };
  if (!valid_port(command_port_) || !valid_port(management_port_) ||
      !valid_port(telemetry_port_) || !valid_port(lidar_port_) ||
      management_port_ == telemetry_port_ || management_port_ == lidar_port_ ||
      telemetry_port_ == lidar_port_ || timeout_ms_ <= 0 || timeout_ms_ > 500 ||
      max_vehicles_ <= 0 || !std::isfinite(max_v_) || !std::isfinite(max_w_) ||
      max_v_ < 0 || max_w_ < 0) {
    RCLCPP_FATAL(node->get_logger(), "%sInvalid Mini driver parameters%s",
                 color(kRed), color(kReset));
    throw std::invalid_argument("invalid Mini driver parameters");
  }
  for (const auto &ip : ips_) {
    in_addr address{};
    if (::inet_pton(AF_INET, ip.c_str(), &address) != 1)
      throw std::invalid_argument("invalid vehicle IP: " + ip);
  }
  if (!discovery_address_.empty()) {
    in_addr address{};
    if (::inet_pton(AF_INET, discovery_address_.c_str(), &address) != 1)
      throw std::invalid_argument("invalid discovery address: " +
                                  discovery_address_);
  }
  discovery_pub_ = n->create_publisher<tianbot_core::msg::MiniDiscovery>(
      "/mini_discovery", rclcpp::QoS(1).transient_local());
  static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(n);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(n);
  RCLCPP_INFO(node->get_logger(), "%sMini V1.5 multi-driver starting%s",
              color(kCyan), color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  vehicle_ips: %s%s", color(kCyan),
              vehicle_ips_.c_str(), color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  discovery_address: %s%s", color(kCyan),
              discovery_address_.empty() ? "auto" : discovery_address_.c_str(),
              color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  command_port: %d%s", color(kCyan),
              command_port_, color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  management_port: %d%s", color(kCyan),
              management_port_, color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  telemetry_port: %d%s", color(kCyan),
              telemetry_port_, color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  lidar_port: %d%s", color(kCyan),
              lidar_port_, color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  command_timeout_ms: %d%s", color(kCyan),
              timeout_ms_, color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  max_linear_velocity: %.3f%s",
              color(kCyan), max_v_, color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  max_angular_velocity: %.3f%s",
              color(kCyan), max_w_, color(kReset));
  RCLCPP_INFO(node->get_logger(), "%s  publish_lidar_debug: %s%s", color(kCyan),
              publish_lidar_debug_ ? "true" : "false", color(kReset));
  RCLCPP_INFO(node->get_logger(), "  max_vehicles: %d", max_vehicles_);
  RCLCPP_INFO(node->get_logger(), "  scan_only: %s",
              scan_only_ ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "  base_frame: %s", frame_base_.c_str());
  RCLCPP_INFO(node->get_logger(), "  odom_frame: %s", frame_odom_.c_str());
  RCLCPP_INFO(node->get_logger(), "  imu_frame: %s", frame_imu_.c_str());
  RCLCPP_INFO(node->get_logger(), "  lidar_frame: %s", frame_lidar_.c_str());
  timer_ = n->create_wall_timer(std::chrono::milliseconds(10),
                                std::bind(&TianbotMiniV15Multi::tick, this));
  initDone_ = true;
}
TianbotMiniV15Multi::~TianbotMiniV15Multi() {
  for (auto &[_, v] : vehicles_)
    if (v->session)
      stop(*v, 4);
  for (int f : {mgmt_, telemetry_, lidar_})
    if (f >= 0)
      ::close(f);
}
void TianbotMiniV15Multi::open() {
  auto make = [&](int port) {
    int f = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (f < 0)
      throw std::runtime_error("socket");
    int sz = 256 * 1024;
    setsockopt(f, SOL_SOCKET, SO_RCVBUF, &sz, sizeof sz);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(f, (sockaddr *)&a, sizeof a) != 0) {
      ::close(f);
      throw std::runtime_error("bind UDP port " + std::to_string(port));
    }
    return f;
  };
  try {
    mgmt_ = make(management_port_);
    telemetry_ = make(telemetry_port_);
    lidar_ = make(lidar_port_);
    started_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(node->get_logger(),
                "%sUDP transport ready; searching for vehicles%s",
                color(kGreen), color(kReset));
  } catch (const std::exception &e) {
    for (int *fd : {&mgmt_, &telemetry_, &lidar_}) {
      if (*fd >= 0)
        ::close(*fd);
      *fd = -1;
    }
    RCLCPP_FATAL(node->get_logger(), "%sMini transport failed: %s%s",
                 color(kRed), e.what(), color(kReset));
    throw;
  }
}
void TianbotMiniV15Multi::send(const Vehicle &v, uint8_t type,
                               const std::vector<uint8_t> &p, uint16_t port) {
  if (mgmt_ < 0)
    return;
  std::vector<uint8_t> b;
  if (!mini::encode_frame(type, 0, seq_++, p, &b))
    return;
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  inet_pton(AF_INET, v.ip.c_str(), &a.sin_addr);
  ::sendto(mgmt_, b.data(), b.size(), 0, (sockaddr *)&a, sizeof a);
}
void TianbotMiniV15Multi::discover() {
  std::vector<uint8_t> p(4);
  const uint32_t request = request_++;
  mini::write_u32(p.data(), request);
  recent_discovery_requests_.push_back(request);
  while (recent_discovery_requests_.size() > 8)
    recent_discovery_requests_.pop_front();
  // 单设备模式只提供 vehicle_ips 时，直接向该设备发发现请求；广播模式
  // （vehicle_ips=* 或扫描模式）仍使用局域网广播。这样不要求用户重复填写 IP。
  std::string dst = discovery_address_;
  if (dst.empty()) {
    if (vehicle_ips_ != "*" && ips_.size() == 1)
      dst = ips_.front();
    else
      dst = "255.255.255.255";
  }
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(management_port_);
  if (inet_pton(AF_INET, dst.c_str(), &a.sin_addr) != 1)
    return;
  std::vector<uint8_t> b;
  mini::encode_frame(mini::kDiscoverRequest, 0, seq_++, p, &b);
  int one = 1;
  setsockopt(mgmt_, SOL_SOCKET, SO_BROADCAST, &one, sizeof one);
  ::sendto(mgmt_, b.data(), b.size(), 0, (sockaddr *)&a, sizeof a);
}
TianbotMiniV15Multi::Vehicle *
TianbotMiniV15Multi::byMac(const std::array<uint8_t, 6> &m) {
  auto i = vehicles_.find(macKey(m));
  return i == vehicles_.end() ? nullptr : i->second.get();
}
void TianbotMiniV15Multi::createInterfaces(Vehicle &v) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  auto ns = v.ns;
  auto topic = [&](const char *x) { return "/" + ns + "/" + x; };
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node->now();
  tf.header.frame_id = ns + "/base_link";
  tf.child_frame_id = ns + "/laser";
  tf.transform.rotation.w = 1.0;
  tf.transform.translation.x = .070;
  tf.transform.translation.z = .055;
  static_tf_->sendTransform(tf);
  tf.child_frame_id = ns + "/imu_link";
  tf.transform.translation.x = 0;
  tf.transform.translation.z = .020;
  static_tf_->sendTransform(tf);
  v.odom = node->create_publisher<nav_msgs::msg::Odometry>(topic("odom"), 10);
  v.imu = node->create_publisher<sensor_msgs::msg::Imu>(topic("imu"), 10);
  v.scan =
      node->create_publisher<sensor_msgs::msg::LaserScan>(topic("scan"), qos);
  if (publish_lidar_debug_) {
    v.points = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic("lidar_points"), qos);
    v.raw = node->create_publisher<std_msgs::msg::UInt8MultiArray>(
        topic("lidar_raw"), qos);
  }
  v.state_pub =
      node->create_publisher<std_msgs::msg::UInt32>(topic("vehicle_state"), 10);
  v.link = node->create_publisher<std_msgs::msg::Bool>(topic("spi_link"), 10);
  v.battery = node->create_publisher<sensor_msgs::msg::BatteryState>(
      topic("battery_state"), 10);
  v.cmd = node->create_subscription<geometry_msgs::msg::Twist>(
      topic("cmd_vel"), 10,
      [this, p = vehicles_[macKey(v.mac)]](
          const geometry_msgs::msg::Twist::ConstSharedPtr m) {
        command(p, m);
      });
}
void TianbotMiniV15Multi::publishDiscovery() {
  tianbot_core::msg::MiniDiscovery m;
  m.header.stamp = node->now();
  for (auto &[_, v] : vehicles_) {
    tianbot_core::msg::MiniDevice d;
    std::copy(v->mac.begin(), v->mac.end(), d.mac.begin());
    d.name = v->name;
    d.ip = v->ip;
    d.proto_version = v->fw;
    d.state = v->state;
    d.faults = v->faults;
    d.selected = v->selected;
    d.online = v->online;
    d.ros_namespace = v->ns;
    auto t = std::chrono::system_clock::now().time_since_epoch();
    d.last_seen.sec =
        (int32_t)std::chrono::duration_cast<std::chrono::seconds>(t).count();
    d.last_seen.nanosec = (uint32_t)(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t).count() %
        1000000000);
    m.devices.push_back(d);
  }
  discovery_pub_->publish(m);
}
void TianbotMiniV15Multi::command(
    const std::shared_ptr<Vehicle> &v,
    const geometry_msgs::msg::Twist::ConstSharedPtr &m) {
  if (!v || !v->online || !v->session || !std::isfinite(m->linear.x) ||
      !std::isfinite(m->angular.z))
    return;
  float x = std::clamp((double)m->linear.x, -max_v_, max_v_),
        z = std::clamp((double)m->angular.z, -max_w_, max_w_);
  std::vector<uint8_t> p(18);
  mini::write_u32(p.data(), v->session);
  mini::write_u32(p.data() + 4, v->cmd_seq++);
  std::memcpy(p.data() + 8, &x, 4);
  std::memcpy(p.data() + 12, &z, 4);
  mini::write_u16(p.data() + 16, (uint16_t)timeout_ms_);
  send(*v, mini::kVelocity, p, command_port_);
  v->last_cmd = std::chrono::steady_clock::now();
}
void TianbotMiniV15Multi::heartbeat(Vehicle &v) {
  std_msgs::msg::Bool link;
  link.data = v.online;
  v.link->publish(link);
  std::vector<uint8_t> p(10, 0);
  std::copy(v.mac.begin(), v.mac.end(), p.begin());
  p[6] = 2;
  send(v, mini::kHeartbeat, p, management_port_);
}
void TianbotMiniV15Multi::stop(Vehicle &v, uint8_t reason) {
  if (!v.session)
    return;
  std::vector<uint8_t> p(5);
  mini::write_u32(p.data(), v.session);
  p[4] = reason;
  send(v, mini::kStop, p, command_port_);
  v.session = 0;
}
void TianbotMiniV15Multi::handle(Vehicle &v, const mini::Frame &x, bool) {
  const auto now = std::chrono::steady_clock::now();
  if (!v.selected || v.ns.empty())
    return;
  if (x.type == mini::kSessionGrant) {
    if (x.payload.size() != 13 ||
        !std::equal(v.mac.begin(), v.mac.end(), x.payload.begin()))
      return;
    const auto granted = mini::read_u32(x.payload.data() + 6);
    if (x.payload[12] != 0 || granted == 0) {
      if (!v.grant_warning_reported)
        RCLCPP_WARN(
            node->get_logger(), "%sControl session rejected: %s result=%u%s",
            color(kYellow), v.name.c_str(), x.payload[12], color(kReset));
      v.grant_warning_reported = true;
      v.pending = 0;
      return;
    }
    v.pending = granted;
    v.grant_warning_reported = false;
    std::vector<uint8_t> p(13, 0);
    std::copy(v.mac.begin(), v.mac.end(), p.begin());
    mini::write_u32(p.data() + 6, v.pending);
    mini::write_u16(p.data() + 10, timeout_ms_);
    send(v, mini::kSessionAck, p, management_port_);
    v.session = v.pending;
    v.pending = 0;
    // 协议无最终授权确认，不能把 ACK 已发送描述为控制权已确认。
    RCLCPP_INFO(node->get_logger(), "%sControl session acknowledged: %s%s",
                color(kCyan), v.name.c_str(), color(kReset));
    return;
  }
  const bool heartbeat =
      x.type == mini::kHeartbeat && x.payload.size() == 10 &&
      std::equal(v.mac.begin(), v.mac.end(), x.payload.begin());
  const bool telemetry =
      (x.type == mini::kVehicleState && x.payload.size() == 6) ||
      (x.type == mini::kOdometry && x.payload.size() == 28) ||
      (x.type == mini::kImu &&
       (x.payload.size() == 32 || x.payload.size() == 48));
  if (heartbeat || telemetry) {
    v.last_rx = now;
    if (!v.online) {
      v.online = true;
      RCLCPP_INFO(node->get_logger(), "%sVehicle online: %s ip=%s ns=/%s%s",
                  color(kGreen), v.name.c_str(), v.ip.c_str(), v.ns.c_str(),
                  color(kReset));
    }
  }
  if (heartbeat)
    return;
  if (v.ns.empty())
    return;
  if (x.type == mini::kVehicleState && x.payload.size() == 6) {
    v.state = x.payload[0];
    v.faults = mini::read_u16(x.payload.data() + 1);
    std_msgs::msg::UInt32 q;
    q.data =
        v.state | (uint32_t(v.faults) << 8) | (uint32_t(x.payload[5]) << 24);
    v.state_pub->publish(q);
    sensor_msgs::msg::BatteryState b;
    b.header.stamp = node->now();
    b.voltage = mini::read_u16(x.payload.data() + 3) / 1000.f;
    b.percentage = 0.0f;
    v.battery->publish(b);
    return;
  }
  if (x.type == mini::kOdometry && x.payload.size() == 28) {
    nav_msgs::msg::Odometry o;
    o.header.stamp = node->now();
    o.header.frame_id = v.ns + "/" + frame_odom_;
    o.child_frame_id = v.ns + "/" + frame_base_;
    o.pose.pose.position.x = load<float>(x.payload.data() + 8);
    o.pose.pose.position.y = load<float>(x.payload.data() + 12);
    o.pose.pose.orientation.w = 1;
    o.twist.twist.linear.x = load<float>(x.payload.data() + 20);
    o.twist.twist.angular.z = load<float>(x.payload.data() + 24);
    v.odom->publish(o);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = o.header;
    tf.child_frame_id = o.child_frame_id;
    tf.transform.translation.x = o.pose.pose.position.x;
    tf.transform.translation.y = o.pose.pose.position.y;
    tf.transform.rotation = o.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
    return;
  }
  if (x.type == mini::kImu &&
      (x.payload.size() == 32 || x.payload.size() == 48)) {
    sensor_msgs::msg::Imu i;
    i.header.stamp = node->now();
    i.header.frame_id = v.ns + "/" + frame_imu_;
    i.orientation.x = load<float>(x.payload.data() + 8);
    i.orientation.y = load<float>(x.payload.data() + 12);
    i.orientation.z = load<float>(x.payload.data() + 16);
    i.orientation.w = load<float>(x.payload.data() + 20);
    // 当前数据来自已校准的板载 IMU；给出保守的固定方差，避免全零被
    // 消费者误解为“绝对无噪声”。具体标定参数可后续按实测替换。
    i.orientation_covariance[0] = 1e-3;
    i.orientation_covariance[4] = 1e-3;
    i.orientation_covariance[8] = 1e-3;
    geometry_msgs::msg::TransformStamped atf;
    atf.header = i.header;
    atf.header.frame_id = v.ns + "/" + frame_odom_;
    atf.child_frame_id = v.ns + "/imu_attitude";
    atf.transform.rotation = i.orientation;
    tf_broadcaster_->sendTransform(atf);
    if (x.payload.size() == 48) {
      i.linear_acceleration.x = load<float>(x.payload.data() + 24);
      i.linear_acceleration.y = load<float>(x.payload.data() + 28);
      i.linear_acceleration.z = load<float>(x.payload.data() + 32);
      i.angular_velocity.x = load<float>(x.payload.data() + 36);
      i.angular_velocity.y = load<float>(x.payload.data() + 40);
      i.angular_velocity.z = load<float>(x.payload.data() + 44);
      i.linear_acceleration_covariance[0] = 4e-4;
      i.linear_acceleration_covariance[4] = 4e-4;
      i.linear_acceleration_covariance[8] = 4e-4;
      i.angular_velocity_covariance[0] = 1e-5;
      i.angular_velocity_covariance[4] = 1e-5;
      i.angular_velocity_covariance[8] = 1e-5;
    } else {
      i.angular_velocity.x = i.angular_velocity.y = i.angular_velocity.z = 0.0;
      i.angular_velocity_covariance[0] = -1.0;
      i.linear_acceleration.x = i.linear_acceleration.y =
          i.linear_acceleration.z = 0.0;
      i.linear_acceleration_covariance[0] = -1.0;
    }
    v.imu->publish(i);
    return;
  }
  if (x.type == mini::kLidarRaw) {
    std_msgs::msg::UInt8MultiArray r;
    r.data = x.payload;
    if (v.raw)
      v.raw->publish(r);
    mini::LidarScanAssembler::Scan s;
    if (v.lidar.push(
            x, std::chrono::duration<double>(now.time_since_epoch()).count(),
            s)) {
      sensor_msgs::msg::LaserScan l;
      l.header.stamp = node->now();
      l.header.frame_id = v.ns + "/" + frame_lidar_;
      l.angle_min = 0;
      l.angle_increment = 2 * M_PI / 1440;
      l.angle_max = 2 * M_PI;
      l.range_min = .02;
      l.range_max = 12;
      l.ranges.assign(1440, INFINITY);
      for (size_t j = 0; j < s.ranges.size(); ++j) {
        auto k = (size_t)std::lround(s.angles[j] / l.angle_increment) % 1440;
        l.ranges[k] = std::min(l.ranges[k], s.ranges[j]);
      }
      v.scan->publish(l);
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = l.header;
      cloud.height = 1;
      cloud.width = 0;
      cloud.is_bigendian = false;
      cloud.is_dense = false;
      sensor_msgs::PointCloud2Modifier mod(cloud);
      mod.setPointCloud2FieldsByString(1, "xyz");
      size_t count = 0;
      for (float r : l.ranges)
        if (std::isfinite(r))
          ++count;
      mod.resize(count);
      cloud.width = static_cast<uint32_t>(count);
      sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"),
          iz(cloud, "z");
      for (size_t j = 0; j < l.ranges.size(); ++j) {
        const float r = l.ranges[j];
        if (!std::isfinite(r))
          continue;
        const float a = l.angle_min + l.angle_increment * j;
        *ix = r * std::cos(a);
        *iy = r * std::sin(a);
        *iz = 0.0f;
        ++ix;
        ++iy;
        ++iz;
      }
      if (v.points)
        v.points->publish(cloud);
    }
  }
}
void TianbotMiniV15Multi::receive(int fd) {
  std::array<uint8_t, 1400> b{};
  for (unsigned k = 0; k < 64; k++) {
    sockaddr_in src{};
    socklen_t sl = sizeof src;
    auto n =
        recvfrom(fd, b.data(), b.size(), MSG_DONTWAIT, (sockaddr *)&src, &sl);
    if (n <= 0)
      return;
    mini::Frame x;
    if (!mini::decode_frame(b.data(), n, &x))
      continue;
    if (x.type == mini::kDiscoverResponse) {
      mini::DeviceInfo d;
      if (!mini::parse_discovery(x, &d) ||
          std::find(recent_discovery_requests_.begin(),
                    recent_discovery_requests_.end(),
                    mini::read_u32(x.payload.data())) ==
              recent_discovery_requests_.end())
        continue;
      auto *old = byMac(d.mac);
      if (!old) {
        auto v = std::make_shared<Vehicle>();
        v->mac = d.mac;
        v->name = d.name;
        v->ip = ipOf(src);
        v->fw = d.firmware_version;
        v->state = d.state;
        v->faults = d.faults;
        v->found = true;
        v->selected = vehicle_ips_ == "*" ||
                      std::find(ips_.begin(), ips_.end(), v->ip) != ips_.end();
        RCLCPP_INFO(node->get_logger(),
                    "%sVehicle discovered: %s ip=%s mac=%s selected=%s%s",
                    color(kGreen), v->name.c_str(), v->ip.c_str(),
                    macKey(v->mac).c_str(), v->selected ? "yes" : "no",
                    color(kReset));
        size_t selected_count = 0;
        for (auto &[_, existing] : vehicles_)
          if (!existing->ns.empty())
            ++selected_count;
        if (v->selected && !scan_only_ &&
            selected_count < (size_t)max_vehicles_) {
          v->ns = mini::namespace_name(v->name, v->mac);
          std::string base_ns = v->ns;
          char suffix[5];
          std::snprintf(suffix, sizeof suffix, "%02X%02X", v->mac[4],
                        v->mac[5]);
          for (unsigned n = 2;; ++n) {
            bool used = false;
            for (auto &[_, other] : vehicles_)
              if (other->ns == v->ns)
                used = true;
            if (!used)
              break;
            v->ns = base_ns + "_" + suffix + "_" + std::to_string(n);
          }
          vehicles_[macKey(v->mac)] = v;
          createInterfaces(*v);
        } else {
          if (v->selected && !scan_only_)
            RCLCPP_WARN(
                node->get_logger(), "%sVehicle skipped: %s (max_vehicles=%d)%s",
                color(kYellow), v->name.c_str(), max_vehicles_, color(kReset));
          vehicles_[macKey(v->mac)] = v;
        }
        old = v.get();
      } else if (old->ip != ipOf(src)) {
        RCLCPP_WARN(node->get_logger(),
                    "%sVehicle address changed: %s %s -> %s; reconnecting%s",
                    color(kYellow), old->name.c_str(), old->ip.c_str(),
                    ipOf(src).c_str(), color(kReset));
        // 旧地址可能已分配给其他设备，不向旧地址补发控制命令。
        old->session = old->pending = 0;
        old->online = false;
        old->last_rx = old->last_cmd = old->last_heartbeat =
            old->last_session_attempt = {};
        old->lidar = {};
        if (old->link) {
          std_msgs::msg::Bool link;
          link.data = false;
          old->link->publish(link);
        }
        old->ip = ipOf(src);
      }
      old->last_discovery = std::chrono::steady_clock::now();
      continue;
    }
    for (auto &[_, v] : vehicles_)
      if (v->ip == ipOf(src)) {
        handle(*v, x, fd == lidar_);
        break;
      }
  }
}
void TianbotMiniV15Multi::tick() {
  auto now = std::chrono::steady_clock::now();
  if (now >= next_discovery_) {
    discover();
    next_discovery_ = now + std::chrono::seconds(1);
  }
  receive(mgmt_);
  receive(telemetry_);
  receive(lidar_);
  for (auto &[_, v] : vehicles_) {
    if (!v->selected || v->ns.empty())
      continue;
    if (v->found &&
        (v->last_heartbeat.time_since_epoch().count() == 0 ||
         now - v->last_heartbeat >= std::chrono::milliseconds(200))) {
      heartbeat(*v);
      v->last_heartbeat = now;
    }
    if (!v->session && v->found &&
        now - v->last_session_attempt > std::chrono::milliseconds(500)) {
      std::vector<uint8_t> p(13);
      std::copy(v->mac.begin(), v->mac.end(), p.begin());
      mini::write_u32(p.data() + 6, 0);
      mini::write_u16(p.data() + 10, timeout_ms_);
      send(*v, mini::kSessionAck, p, management_port_);
      v->last_session_attempt = now;
    }
    if (v->online && now - v->last_rx > std::chrono::seconds(3)) {
      v->online = false;
      RCLCPP_ERROR(node->get_logger(),
                   "%sVehicle offline: %s ip=%s ns=/%s (no telemetry for 3s)%s",
                   color(kRed), v->name.c_str(), v->ip.c_str(), v->ns.c_str(),
                   color(kReset));
      stop(*v, 3);
      std_msgs::msg::Bool q;
      q.data = false;
      v->link->publish(q);
    }
  }
  // 只在缺失状态第一次出现时报告；后台继续发现，迟到的设备仍可连接。
  if (now - started_ >= std::chrono::seconds(5)) {
    const auto report_missing = [this](const std::string &target, bool found) {
      if (found) {
        missing_reported_.erase(target);
        return;
      }
      if (missing_reported_.insert(target).second)
        RCLCPP_ERROR(node->get_logger(),
                     "%sVehicle not found: %s (still searching)%s", color(kRed),
                     target.c_str(), color(kReset));
    };
    if (vehicle_ips_ == "*" || scan_only_) {
      report_missing("any device", !vehicles_.empty());
    } else {
      for (const auto &ip : ips_) {
        bool found = false;
        for (const auto &entry : vehicles_)
          if (entry.second->ip == ip)
            found = true;
        report_missing(ip, found);
      }
    }
  }
  publishDiscovery();
}
