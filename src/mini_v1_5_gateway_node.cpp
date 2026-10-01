#include <array>
#include <cmath>
#include <cstring>
#include <vector>

#include <arpa/inet.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sys/socket.h>
#include <unistd.h>

#include "tianbot_core/mini_v1_gateway.hpp"

namespace {
uint16_t read_u16(const uint8_t *p) { return static_cast<uint16_t>(p[0]) | static_cast<uint16_t>(p[1]) << 8; }
uint32_t read_u32(const uint8_t *p) { return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 | static_cast<uint32_t>(p[2]) << 16 | static_cast<uint32_t>(p[3]) << 24; }
void write_u32(uint8_t *p, uint32_t value) { p[0] = static_cast<uint8_t>(value); p[1] = static_cast<uint8_t>(value >> 8); p[2] = static_cast<uint8_t>(value >> 16); p[3] = static_cast<uint8_t>(value >> 24); }

bool encode_frame(uint8_t type, uint32_t sequence, const std::vector<uint8_t> &payload, std::vector<uint8_t> *out)
{
    if (out == nullptr || payload.size() > 1200U) return false;
    out->assign(16U + payload.size(), 0U); uint8_t *p = out->data(); p[0] = 0x42U; p[1] = 0x54U; p[2] = 1U; p[3] = type; p[5] = 12U; write_u32(p + 6U, sequence); p[10] = static_cast<uint8_t>(payload.size()); p[11] = static_cast<uint8_t>(payload.size() >> 8); std::copy(payload.begin(), payload.end(), p + 12U); write_u32(p + 12U + payload.size(), tianbot_core::mini_v1_5::mini_crc32(p, 12U + payload.size())); return true;
}
}

class MiniGateway {
 public:
  MiniGateway(ros::NodeHandle &node) : node_(node) {
    node_.param<std::string>("remote_ip", remote_ip_, "192.168.4.1"); node_.param("command_port", command_port_, 5000); node_.param("telemetry_port", telemetry_port_, 5001); node_.param("lidar_port", lidar_port_, 5002); node_.param("management_port", management_port_, 5003); node_.param("session_id", session_id_, 0);
    socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); if (socket_ < 0) throw std::runtime_error("cannot create Mini UDP socket");
    remote_.sin_family = AF_INET; remote_.sin_port = htons(static_cast<uint16_t>(command_port_)); if (inet_pton(AF_INET, remote_ip_.c_str(), &remote_.sin_addr) != 1) throw std::runtime_error("invalid remote_ip");
    sockaddr_in local{}; local.sin_family = AF_INET; local.sin_port = htons(static_cast<uint16_t>(telemetry_port_)); local.sin_addr.s_addr = htonl(INADDR_ANY); if (::bind(socket_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) throw std::runtime_error("cannot bind Mini telemetry socket");
    lidar_socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); if (lidar_socket_ < 0) throw std::runtime_error("cannot create Mini lidar UDP socket"); local.sin_port = htons(static_cast<uint16_t>(lidar_port_)); if (::bind(lidar_socket_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) throw std::runtime_error("cannot bind Mini lidar socket");
    management_ = remote_; management_.sin_port = htons(static_cast<uint16_t>(management_port_));
    cmd_sub_ = node_.subscribe("cmd_vel", 10, &MiniGateway::command, this); odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 10); imu_pub_ = node_.advertise<sensor_msgs::Imu>("imu/data", 10); scan_pub_ = node_.advertise<sensor_msgs::LaserScan>("scan", 10); timer_ = node_.createTimer(ros::Duration(0.01), &MiniGateway::receive, this); discovery_timer_ = node_.createTimer(ros::Duration(1.0), &MiniGateway::discover, this);
  }
  ~MiniGateway() { if (socket_ >= 0) ::close(socket_); if (lidar_socket_ >= 0) ::close(lidar_socket_); }
 private:
  void command(const geometry_msgs::Twist::ConstPtr &cmd) {
    if (session_id_ == 0) { ROS_WARN_THROTTLE(5.0, "Mini session_id is zero; command rejected"); return; }
    std::vector<uint8_t> payload(18U); write_u32(payload.data(), static_cast<uint32_t>(session_id_)); write_u32(payload.data() + 4U, command_sequence_++); const float vx = static_cast<float>(cmd->linear.x); const float wz = static_cast<float>(cmd->angular.z); std::memcpy(payload.data() + 8U, &vx, sizeof(vx)); std::memcpy(payload.data() + 12U, &wz, sizeof(wz)); payload[16] = 0xf4U; payload[17] = 0x01U; std::vector<uint8_t> packet; if (encode_frame(0x11U, frame_sequence_++, payload, &packet)) (void)::sendto(socket_, packet.data(), packet.size(), 0, reinterpret_cast<sockaddr *>(&remote_), sizeof(remote_));
  }
  void send_management(uint8_t type, const std::vector<uint8_t> &payload) { std::vector<uint8_t> packet; if (encode_frame(type, frame_sequence_++, payload, &packet)) (void)::sendto(socket_, packet.data(), packet.size(), 0, reinterpret_cast<sockaddr *>(&management_), sizeof(management_)); }
  void discover(const ros::TimerEvent &) { if (session_id_ != 0) return; if (!discovered_) { std::vector<uint8_t> payload(4U); write_u32(payload.data(), discovery_request_++); send_management(0x30U, payload); } else { std::vector<uint8_t> payload(13U); std::memcpy(payload.data(), mac_.data(), mac_.size()); write_u32(payload.data() + 6U, pending_session_); payload[10] = 0xf4U; payload[11] = 0x01U; send_management(0x36U, payload); if (pending_session_ != 0U) { session_id_ = static_cast<int>(pending_session_); pending_session_ = 0U; } } }
  void receive(const ros::TimerEvent &) {
    receive_from(socket_);
    receive_from(lidar_socket_);
  }
  void receive_from(int fd) {
    if (fd < 0) return;
    std::array<uint8_t, 1216> packet{}; const auto bytes = ::recv(fd, packet.data(), packet.size(), MSG_DONTWAIT); if (bytes <= 0) return; tianbot_core::mini_v1_5::Frame frame{}; if (!tianbot_core::mini_v1_5::decode_mini_frame(packet.data(), static_cast<size_t>(bytes), &frame)) return;
    if (frame.type == 0x22U && frame.payload.size() == 28U) { nav_msgs::Odometry msg; msg.header.stamp = ros::Time::now(); std::memcpy(&msg.pose.pose.position.x, frame.payload.data() + 8U, 4U); std::memcpy(&msg.pose.pose.position.y, frame.payload.data() + 12U, 4U); std::memcpy(&msg.twist.twist.linear.x, frame.payload.data() + 20U, 4U); std::memcpy(&msg.twist.twist.angular.z, frame.payload.data() + 24U, 4U); odom_pub_.publish(msg); }
    if (frame.type == 0x21U && frame.payload.size() == 32U) { sensor_msgs::Imu msg; msg.header.stamp = ros::Time::now(); std::memcpy(&msg.linear_acceleration.x, frame.payload.data() + 8U, 4U); std::memcpy(&msg.linear_acceleration.y, frame.payload.data() + 12U, 4U); std::memcpy(&msg.linear_acceleration.z, frame.payload.data() + 16U, 4U); std::memcpy(&msg.angular_velocity.x, frame.payload.data() + 20U, 4U); std::memcpy(&msg.angular_velocity.y, frame.payload.data() + 24U, 4U); std::memcpy(&msg.angular_velocity.z, frame.payload.data() + 28U, 4U); imu_pub_.publish(msg); }
    if (frame.type == 0x31U && frame.payload.size() == 45U) { std::memcpy(mac_.data(), frame.payload.data() + 4U, mac_.size()); discovered_ = true; }
    if (frame.type == 0x35U && frame.payload.size() == 13U) pending_session_ = read_u32(frame.payload.data() + 6U);
    if (frame.type == 0x40U) lidar(frame.payload);
  }
  void lidar(const std::vector<uint8_t> &p) { if (p.size() < 34U) return; const uint32_t id = read_u32(p.data()); const uint16_t count = read_u16(p.data() + 28U), offset = read_u16(p.data() + 30U), size = read_u16(p.data() + 32U); if (count == 0U || offset + size > count || p.size() != 34U + static_cast<size_t>(size) * 2U) return; if (id != scan_id_) { scan_id_ = id; ranges_.assign(count, NAN); std::memcpy(&angle_min_, p.data() + 12U, 4U); std::memcpy(&angle_increment_, p.data() + 16U, 4U); std::memcpy(&range_min_, p.data() + 20U, 4U); std::memcpy(&range_max_, p.data() + 24U, 4U); } if (ranges_.size() != count) return; for (uint16_t i = 0; i < size; ++i) { const uint16_t mm = read_u16(p.data() + 34U + i * 2U); ranges_[offset + i] = mm == 0U ? NAN : static_cast<float>(mm) / 1000.0F; } for (const float value : ranges_) if (std::isnan(value)) return; sensor_msgs::LaserScan scan; scan.header.stamp = ros::Time::now(); scan.header.frame_id = "laser"; scan.angle_min = angle_min_; scan.angle_increment = angle_increment_; scan.angle_max = angle_min_ + angle_increment_ * static_cast<float>(count - 1U); scan.range_min = range_min_; scan.range_max = range_max_; scan.ranges = ranges_; scan_pub_.publish(scan); scan_id_ = 0U; ranges_.clear(); }
  ros::NodeHandle &node_; int socket_{-1}, lidar_socket_{-1}, command_port_{5000}, telemetry_port_{5001}, lidar_port_{5002}, management_port_{5003}, session_id_{0}; uint32_t frame_sequence_{1}, command_sequence_{1}, discovery_request_{1}, pending_session_{0}, scan_id_{0}; bool discovered_{false}; std::array<uint8_t, 6> mac_{}; std::vector<float> ranges_; float angle_min_{}, angle_increment_{}, range_min_{}, range_max_{}; std::string remote_ip_; sockaddr_in remote_{}, management_{}; ros::Subscriber cmd_sub_; ros::Publisher odom_pub_, imu_pub_, scan_pub_; ros::Timer timer_, discovery_timer_;
};

int main(int argc, char **argv) { ros::init(argc, argv, "mini_v1_5_gateway"); ros::NodeHandle node("~"); MiniGateway gateway(node); ros::spin(); return 0; }
