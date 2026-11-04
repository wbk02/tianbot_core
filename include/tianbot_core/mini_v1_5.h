#ifndef __MINI_V1_5_H__
#define __MINI_V1_5_H__

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "mini_lidar_scan.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "chassis.h"
#include "tianbot_core/mini_v1_protocol.hpp"

// Mini V1.5 vehicle type. It speaks the Mini V1.5 UDP protocol instead of the
// TianbotCore serial/UDP protocol, so it never touches comm_inf_. Odometry, IMU
// and voltage are published through TianbotChasis helpers, which keeps frame
// parameters and TF broadcasting consistent with the other vehicle types.
class TianbotMiniV15 : public TianbotChasis {
public:
  explicit TianbotMiniV15(const std::shared_ptr<rclcpp::Node> &node);
  ~TianbotMiniV15() override;

  void open(void) override;

private:
  void velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);
  void receive();
  void receiveFrom(int fd);
  void safetyCheck();
  void sessionTick();
  void sendFrame(std::uint8_t type, const std::vector<std::uint8_t> &payload,
                 bool management);
  void sendStop(std::uint8_t reason);
  void sendHeartbeat();
  void publishLink(bool up);
  void publishVehicleState(std::uint8_t state, std::uint16_t faults,
                           std::uint16_t millivolts, std::uint8_t source,
                           bool hasBatteryVoltage);
  void publishScan();
  void handleRawLidar(const tianbot_core::mini_v1_5::Frame &frame);
  tianbot_core::mini_v1_5::LidarScanAssembler raw_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_points_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_lidar_pub_;
  rclcpp::Time raw_diag_time_{0, 0, RCL_ROS_TIME};
  void handleLidarFragment(const std::vector<std::uint8_t> &payload);
  void openSockets();

  // Transport.
  int telemetry_fd_{-1};
  int lidar_fd_{-1};
  sockaddr_in remote_{};
  sockaddr_in management_{};

  // Parameters.
  std::string remote_ip_;
  std::string device_name_;
  std::string lidar_frame_;
  int command_port_{5000};
  int management_port_{5003};
  int bind_telemetry_port_{5001};
  int bind_lidar_port_{5002};
  int timeout_ms_{500};
  double max_v_{1.5};
  double max_w_{2.0};

  // Session state.
  std::uint32_t session_{0};
  std::uint32_t pending_session_{0};
  std::uint32_t seq_{1};
  std::uint32_t cmd_seq_{1};
  std::uint32_t request_{1};
  std::uint32_t pending_request_{0};
  bool found_{false};
  bool missing_reported_{false};
  bool link_log_online_{false};
  std::chrono::steady_clock::time_point started_{};
  bool online_{false};
  bool active_{false};
  bool mac_filter_valid_{false};
  std::array<std::uint8_t, 6> mac_{};
  std::array<std::uint8_t, 6> mac_filter_{};

  // Lidar reassembly.
  std::uint32_t scan_id_{0};
  std::uint16_t frag_count_{0};
  std::uint16_t seen_{0};
  std::vector<bool> got_;
  std::vector<float> ranges_;

  std::chrono::steady_clock::time_point session_activity_{};
  std::chrono::steady_clock::time_point last_heartbeat_{};
  std::chrono::steady_clock::time_point control_started_{};

  // Timestamps.
  rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time scan_started_{0, 0, RCL_ROS_TIME};

  // ROS interfaces owned by this vehicle type.
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr vehicle_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mini_cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::TimerBase::SharedPtr session_timer_;
};

#endif
