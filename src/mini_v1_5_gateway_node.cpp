#include <array>
#include <chrono>
#include <cstring>
#include <vector>

#include <arpa/inet.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sys/socket.h>
#include <unistd.h>

#include "tianbot_core/mini_v1_protocol.hpp"

namespace mini = tianbot_core::mini_v1_5;

class MiniGateway final : public rclcpp::Node {
 public:
  MiniGateway() : Node("mini_v1_5_gateway") {
    remote_ip_ = declare_parameter<std::string>("remote_ip", "192.168.4.1");
    const auto command_port = declare_parameter<int>("command_port", 5000);
    const auto telemetry_port = declare_parameter<int>("telemetry_port", 5001);
    session_id_ = static_cast<uint32_t>(declare_parameter<int>("session_id", 0));
    socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ < 0) throw std::runtime_error("cannot create Mini UDP socket");
    std::memset(&remote_, 0, sizeof(remote_)); remote_.sin_family = AF_INET;
    remote_.sin_port = htons(static_cast<uint16_t>(command_port));
    if (inet_pton(AF_INET, remote_ip_.c_str(), &remote_.sin_addr) != 1) throw std::runtime_error("invalid remote_ip");
    sockaddr_in local{}; local.sin_family = AF_INET; local.sin_port = htons(static_cast<uint16_t>(telemetry_port)); local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(socket_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) throw std::runtime_error("cannot bind Mini telemetry socket");
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { send_velocity(*msg); });
    rx_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { receive(); });
  }
  ~MiniGateway() override { if (socket_ >= 0) ::close(socket_); }
 private:
  void send_velocity(const geometry_msgs::msg::Twist &cmd) {
    if (session_id_ == 0U) { RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "session_id is zero; command rejected"); return; }
    std::vector<uint8_t> payload(18U); mini::write_u32(payload.data(), session_id_); mini::write_u32(payload.data() + 4U, command_sequence_++);
    const float vx = static_cast<float>(cmd.linear.x); const float wz = static_cast<float>(cmd.angular.z);
    std::memcpy(payload.data() + 8U, &vx, sizeof(vx)); std::memcpy(payload.data() + 12U, &wz, sizeof(wz)); payload[16] = 0xf4U; payload[17] = 0x01U;
    std::vector<uint8_t> packet; if (!mini::encode_frame(0x11U, 0U, frame_sequence_++, payload, &packet)) return;
    (void)::sendto(socket_, packet.data(), packet.size(), 0, reinterpret_cast<sockaddr *>(&remote_), sizeof(remote_));
  }
  void receive() {
    std::array<uint8_t, 1216> packet{};
    const auto bytes = ::recv(socket_, packet.data(), packet.size(), MSG_DONTWAIT);
    if (bytes <= 0) return;
    mini::Frame frame{}; if (!mini::decode_frame(packet.data(), static_cast<size_t>(bytes), &frame)) return;
    if (frame.type == 0x22U && frame.payload.size() == 28U) publish_odom(frame.payload);
    if (frame.type == 0x21U && frame.payload.size() == 32U) publish_imu(frame.payload);
  }
  void publish_odom(const std::vector<uint8_t> &p) { nav_msgs::msg::Odometry msg; msg.header.stamp = now(); std::memcpy(&msg.pose.pose.position.x, p.data() + 8U, 4U); std::memcpy(&msg.pose.pose.position.y, p.data() + 12U, 4U); std::memcpy(&msg.twist.twist.linear.x, p.data() + 20U, 4U); std::memcpy(&msg.twist.twist.angular.z, p.data() + 24U, 4U); odom_pub_->publish(msg); }
  void publish_imu(const std::vector<uint8_t> &p) { sensor_msgs::msg::Imu msg; msg.header.stamp = now(); std::memcpy(&msg.linear_acceleration.x, p.data() + 8U, 4U); std::memcpy(&msg.linear_acceleration.y, p.data() + 12U, 4U); std::memcpy(&msg.linear_acceleration.z, p.data() + 16U, 4U); std::memcpy(&msg.angular_velocity.x, p.data() + 20U, 4U); std::memcpy(&msg.angular_velocity.y, p.data() + 24U, 4U); std::memcpy(&msg.angular_velocity.z, p.data() + 28U, 4U); imu_pub_->publish(msg); }
  int socket_{-1}; sockaddr_in remote_{}; std::string remote_ip_; uint32_t session_id_{0}, frame_sequence_{1}, command_sequence_{1};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_; rclcpp::TimerBase::SharedPtr rx_timer_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<MiniGateway>()); rclcpp::shutdown(); return 0; }
