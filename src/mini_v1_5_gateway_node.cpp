#include "tianbot_core/mini_v1_gateway.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <cmath>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <limits>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
namespace mini = tianbot_core::mini_v1_5;
bool parse_mac(const std::string &s, std::array<uint8_t, 6> *out) {
  unsigned int v[6]{};
  if (!out || s.empty() ||
      std::sscanf(s.c_str(), "%2x:%2x:%2x:%2x:%2x:%2x", &v[0], &v[1], &v[2],
                  &v[3], &v[4], &v[5]) != 6)
    return false;
  for (size_t i = 0; i < 6; ++i)
    (*out)[i] = static_cast<uint8_t>(v[i]);
  return true;
}
template <class T> T load(const uint8_t *p) {
  T v{};
  std::memcpy(&v, p, sizeof(v));
  return v;
}
class Gateway {
public:
  Gateway(ros::NodeHandle &n) : n_(n) {
    n_.param("remote_ip", ip_, std::string("192.168.4.1"));
    n_.param("device_name", name_, std::string());
    std::string mac_text;
    n_.param("device_mac", mac_text, std::string());
    if (!mac_text.empty() && !parse_mac(mac_text, &mac_filter_))
      throw std::invalid_argument("device_mac must be aa:bb:cc:dd:ee:ff");
    mac_filter_valid_ = !mac_text.empty();
    n_.param("command_port", cmd_port_, 5000);
    n_.param("management_port", mgmt_port_, 5003);
    n_.param("bind_telemetry_port", telemetry_, 5001);
    n_.param("bind_lidar_port", lidar_port_, 5002);
    n_.param("max_linear_velocity", max_v_, 1.5);
    n_.param("max_angular_velocity", max_w_, 2.0);
    n_.param("command_timeout_ms", timeout_, 500);
    n_.param("base_frame", base_, std::string("base_link"));
    n_.param("lidar_frame", laser_, std::string("laser"));
    int initial_session = 0;
    n_.param("session_id", initial_session, 0);
    if (initial_session < 0)
      throw std::invalid_argument("session_id must be non-negative");
    session_ = static_cast<uint32_t>(initial_session);
    if (cmd_port_ < 1 || cmd_port_ > 65535 || mgmt_port_ < 1 ||
        mgmt_port_ > 65535 || telemetry_ < 1 || telemetry_ > 65535 ||
        lidar_port_ < 1 || lidar_port_ > 65535 || timeout_ <= 0 ||
        timeout_ > 65535 || max_v_ < 0.0 || max_w_ < 0.0)
      throw std::invalid_argument("invalid gateway parameter range");
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    lfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0 || lfd_ < 0) {
      if (fd_ >= 0) close(fd_);
      if (lfd_ >= 0) close(lfd_);
      throw std::runtime_error("socket");
    }
    remote_ = {};
    remote_.sin_family = AF_INET;
    remote_.sin_port = htons(cmd_port_);
    if (inet_pton(AF_INET, ip_.c_str(), &remote_.sin_addr) != 1)
      throw std::runtime_error("remote_ip");
    try {
      bindfd(fd_, telemetry_);
      bindfd(lfd_, lidar_port_);
    } catch (...) {
      close(fd_);
      close(lfd_);
      fd_ = lfd_ = -1;
      throw;
    }
    mgmt_ = remote_;
    mgmt_.sin_port = htons(mgmt_port_);
    cmd_ = n_.subscribe("cmd_vel", 10, &Gateway::velocity, this);
    odom_ = n_.advertise<nav_msgs::Odometry>("odom", 10);
    imu_ = n_.advertise<sensor_msgs::Imu>("imu/data", 10);
    scan_ = n_.advertise<sensor_msgs::LaserScan>("scan", 10);
    state_ = n_.advertise<std_msgs::UInt32>("vehicle_state", 10);
    battery_ = n_.advertise<sensor_msgs::BatteryState>("battery_state", 10);
    link_ = n_.advertise<std_msgs::Bool>("spi_link", 10);
    timer_ = n_.createTimer(ros::Duration(.01), &Gateway::receive, this);
    tick_ = n_.createTimer(ros::Duration(.05), &Gateway::safety, this);
    discover_ = n_.createTimer(ros::Duration(.5), &Gateway::session_tick, this);
  }
  ~Gateway() {
    stop(4);
    if (fd_ >= 0)
      close(fd_);
    if (lfd_ >= 0)
      close(lfd_);
  }

private:
  void bindfd(int f, int p) {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(p);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(f, (sockaddr *)&a, sizeof(a)) != 0)
      throw std::runtime_error("bind");
  }
  void send(uint8_t t, const std::vector<uint8_t> &p, bool m = false) {
    std::vector<uint8_t> b;
    if (!mini::encode_mini_frame(t, 0, seq_++, p, &b))
      return;
    auto &a = m ? mgmt_ : remote_;
    sendto(fd_, b.data(), b.size(), 0, (sockaddr *)&a, sizeof(a));
  }
  void velocity(const geometry_msgs::Twist::ConstPtr &m) {
    if (!online_ || !session_ || !std::isfinite(m->linear.x) ||
        !std::isfinite(m->angular.z))
      return;
    float v = std::max(-max_v_, std::min(max_v_, m->linear.x)),
          w = std::max(-max_w_, std::min(max_w_, m->angular.z));
    std::vector<uint8_t> p(18);
    mini::write_u32(p.data(), session_);
    mini::write_u32(p.data() + 4, cmdseq_++);
    memcpy(p.data() + 8, &v, 4);
    memcpy(p.data() + 12, &w, 4);
    mini::write_u16(p.data() + 16, timeout_);
    send(0x11, p);
    last_cmd_ = ros::Time::now();
    active_ = true;
  }
  void stop(uint8_t r) {
    if (!session_)
      return;
    std::vector<uint8_t> p(5);
    mini::write_u32(p.data(), session_);
    p[4] = r;
    send(0x12, p);
    active_ = false;
  }
  void session_tick(const ros::TimerEvent &) {
    if (!found_) {
      std::vector<uint8_t> p(4);
      mini::write_u32(p.data(), req_++);
      pending_req_ = mini::read_u32(p.data());
      send(0x30, p, true);
      return;
    }
    if (session_)
      return;
    if (pending_session_ == 0)
      return;
    std::vector<uint8_t> p(13);
    memcpy(p.data(), mac_.data(), 6);
    mini::write_u32(p.data() + 6, pending_session_);
    mini::write_u16(p.data() + 10, timeout_);
    p[12] = 0;
    send(0x36, p, true);
    if (pending_session_) {
      session_ = pending_session_;
      pending_session_ = 0;
    }
  }
  void safety(const ros::TimerEvent &) {
    if (active_ && (ros::Time::now() - last_cmd_).toSec() * 1000 > timeout_)
      stop(2);
    if (online_ && (ros::Time::now() - last_rx_).toSec() > 3) {
      online_ = false;
      std_msgs::Bool b;
      b.data = false;
      link_.publish(b);
      stop(3);
    }
  }
  void receive(const ros::TimerEvent &) {
    recv(fd_);
    recv(lfd_);
  }
  void recv(int f) {
    uint8_t b[1216]{};
    sockaddr_in source{};
    socklen_t source_len = sizeof(source);
    auto n = recvfrom(f, b, sizeof(b), MSG_DONTWAIT,
                      reinterpret_cast<sockaddr *>(&source), &source_len);
    if (n <= 0)
      return;
    if (source.sin_addr.s_addr != remote_.sin_addr.s_addr)
      return;
    mini::Frame x{};
    if (!mini::decode_mini_frame(b, n, &x))
      return;
    if (x.type == 0x31 && x.payload.size() == 49) {
      if (mini::read_u32(x.payload.data()) != pending_req_)
        return;
      std::string d((char *)x.payload.data() + 10, 32);
      const auto nul = d.find('\0');
      if (nul != std::string::npos)
        d.resize(nul);
      if (!name_.empty() && d != name_)
        return;
      if (mac_filter_valid_ &&
          !std::equal(mac_filter_.begin(), mac_filter_.end(),
                      x.payload.begin() + 4))
        return;
      memcpy(mac_.data(), x.payload.data() + 4, 6);
      found_ = true;
      return;
    }
    if (x.type == 0x35 && x.payload.size() == 13 &&
        std::equal(mac_.begin(), mac_.end(), x.payload.begin()) &&
        x.payload[12] == 0) {
      auto granted = mini::read_u32(x.payload.data() + 6);
      if (granted != 0)
        pending_session_ = granted;
      return;
    }
    if (x.type == 0x20 && x.payload.size() == 6) {
      online_ = true;
      last_rx_ = ros::Time::now();
      std_msgs::Bool l;
      l.data = true;
      link_.publish(l);
      std_msgs::UInt32 s;
      s.data = x.payload[0] |
               (uint32_t(mini::read_u16(x.payload.data() + 1)) << 8) |
               (uint32_t(x.payload[5]) << 24);
      state_.publish(s);
      sensor_msgs::BatteryState q;
      q.header.stamp = ros::Time::now();
      q.header.frame_id = base_;
      q.voltage = mini::read_u16(x.payload.data() + 3) / 1000.f;
      q.percentage = std::numeric_limits<float>::quiet_NaN();
      battery_.publish(q);
      return;
    }
    if (x.type == 0x34 && x.payload.size() == 10 &&
        std::equal(mac_.begin(), mac_.end(), x.payload.begin())) {
      online_ = true;
      last_rx_ = ros::Time::now();
      std_msgs::Bool l;
      l.data = true;
      link_.publish(l);
      std_msgs::UInt32 s;
      s.data = x.payload[6] |
               (uint32_t(mini::read_u16(x.payload.data() + 7)) << 8) |
               (uint32_t(x.payload[9]) << 24);
      state_.publish(s);
      return;
    }
    if (x.type == 0x22 && x.payload.size() == 28) {
      online_ = true;
      last_rx_ = ros::Time::now();
      nav_msgs::Odometry m;
      m.header.stamp = ros::Time::now();
      m.header.frame_id = "odom";
      m.child_frame_id = base_;
      float yaw = load<float>(x.payload.data() + 16);
      m.pose.pose.position.x = load<float>(x.payload.data() + 8);
      m.pose.pose.position.y = load<float>(x.payload.data() + 12);
      m.pose.pose.orientation.z = sin(yaw / 2);
      m.pose.pose.orientation.w = cos(yaw / 2);
      m.twist.twist.linear.x = load<float>(x.payload.data() + 20);
      m.twist.twist.angular.z = load<float>(x.payload.data() + 24);
      odom_.publish(m);
      return;
    }
    if (x.type == 0x21 && x.payload.size() == 32) {
      online_ = true;
      last_rx_ = ros::Time::now();
      sensor_msgs::Imu m;
      m.header.stamp = ros::Time::now();
      m.header.frame_id = base_;
      m.linear_acceleration.x = load<float>(x.payload.data() + 8);
      m.linear_acceleration.y = load<float>(x.payload.data() + 12);
      m.linear_acceleration.z = load<float>(x.payload.data() + 16);
      m.angular_velocity.x = load<float>(x.payload.data() + 20);
      m.angular_velocity.y = load<float>(x.payload.data() + 24);
      m.angular_velocity.z = load<float>(x.payload.data() + 28);
      imu_.publish(m);
      return;
    }
    if (x.type == 0x40)
      lidar(x.payload);
  }
  void lidar(const std::vector<uint8_t> &p) {
    if (p.size() < 12)
      return;
    uint32_t id = mini::read_u32(p.data());
    uint16_t idx = mini::read_u16(p.data() + 4),
             cnt = mini::read_u16(p.data() + 6),
             start = mini::read_u16(p.data() + 8);
    uint8_t bins = p[10];
    if (!cnt || idx >= cnt || !bins || p[11] != 0 || start + bins > 90 ||
        p.size() != 12 + bins * 2)
      return;
    if (id != scan_id_ || cnt != frag_count_) {
      scan_id_ = id;
      frag_count_ = cnt;
      seen_ = 0;
      ranges_.assign(90, NAN);
      got_.assign(cnt, false);
      scan_started_ = ros::Time::now();
    }
    if (got_[idx])
      return;
    got_[idx] = true;
    seen_++;
    for (uint16_t i = 0; i < bins; i++) {
      auto mm = mini::read_u16(p.data() + 12 + 2 * i);
      ranges_[start + i] = mm ? mm / 1000.f : NAN;
    }
    if (seen_ != frag_count_) {
      if ((ros::Time::now() - scan_started_).toSec() > 0.2) {
        ranges_.clear();
        got_.clear();
        seen_ = 0;
      }
      return;
    }
    sensor_msgs::LaserScan m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = laser_;
    m.angle_min = 0;
    m.angle_increment = 4 * M_PI / 180;
    m.angle_max = m.angle_increment * 89;
    m.range_min = .02;
    m.range_max = 12;
    m.ranges = ranges_;
    scan_.publish(m);
    ranges_.clear();
    got_.clear();
    seen_ = 0;
  }
  ros::NodeHandle &n_;
  int fd_{-1}, lfd_{-1}, cmd_port_{5000}, mgmt_port_{5003}, telemetry_{5001},
      lidar_port_{5002}, timeout_{500};
  double max_v_{1.5}, max_w_{2};
  uint32_t session_{0}, pending_session_{0}, seq_{1}, cmdseq_{1}, req_{1},
      pending_req_{0}, scan_id_{0};
  bool found_{false}, online_{false}, active_{false}, mac_filter_valid_{false};
  std::string ip_, name_, base_, laser_;
  sockaddr_in remote_{}, mgmt_{};
  std::array<uint8_t, 6> mac_{}, mac_filter_{};
  uint16_t frag_count_{0}, seen_{0};
  std::vector<bool> got_;
  std::vector<float> ranges_;
  ros::Time last_cmd_, last_rx_, scan_started_;
  ros::Subscriber cmd_;
  ros::Publisher odom_, imu_, scan_, state_, battery_, link_;
  ros::Timer timer_, tick_, discover_;
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "mini_v1_5_gateway");
  ros::NodeHandle n("~");
  try {
    Gateway g(n);
    ros::spin();
  } catch (const std::exception &e) {
    ROS_FATAL("%s", e.what());
  }
  return 0;
}
