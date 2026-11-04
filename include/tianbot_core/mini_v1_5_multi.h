#ifndef TIANBOT_CORE_MINI_V1_5_MULTI_H
#define TIANBOT_CORE_MINI_V1_5_MULTI_H
#include "core.h"
#include "mini_lidar_scan.hpp"
#include "mini_v1_protocol.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tianbot_core/msg/mini_discovery.hpp>
#include <unordered_map>
#include <vector>

class TianbotMiniV15Multi final : public TianbotCore {
public:
  explicit TianbotMiniV15Multi(const std::shared_ptr<rclcpp::Node> &node);
  ~TianbotMiniV15Multi() override;
  void open() override;

private:
  struct Vehicle {
    std::array<uint8_t, 6> mac{};
    std::string name, ip, ns;
    uint32_t fw{};
    uint8_t state{};
    uint16_t faults{};
    bool selected{}, online{}, found{};
    uint32_t session{}, pending{};
    bool grant_warning_reported{};
    uint32_t cmd_seq{1};
    std::chrono::steady_clock::time_point last_rx{}, last_cmd{},
        last_discovery{}, last_session_attempt{}, last_heartbeat{};
    tianbot_core::mini_v1_5::LidarScanAssembler lidar;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dummy;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr state_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr link;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd;
    geometry_msgs::msg::Quaternion imu_orientation;
    bool have_imu_orientation{false};
  };
  void tianbotDataProc(unsigned char *, int) override {}
  void tick();
  void receive(int fd);
  void handle(Vehicle &, const tianbot_core::mini_v1_5::Frame &, bool);
  void discover();
  void publishDiscovery();
  void command(const std::shared_ptr<Vehicle> &,
               const geometry_msgs::msg::Twist::ConstSharedPtr &);
  void send(const Vehicle &, uint8_t, const std::vector<uint8_t> &,
            uint16_t port);
  void stop(Vehicle &, uint8_t);
  void heartbeat(Vehicle &);
  void createInterfaces(Vehicle &);
  Vehicle *byMac(const std::array<uint8_t, 6> &);
  int mgmt_{-1}, telemetry_{-1}, lidar_{-1};
  uint32_t seq_{1}, request_{1};
  int command_port_{5000}, management_port_{5003}, telemetry_port_{5001},
      lidar_port_{5002};
  int timeout_ms_{500}, max_vehicles_{5};
  double max_v_{1.5}, max_w_{2.0};
  std::string vehicle_ips_, discovery_address_, frame_base_{"base_link"},
      frame_odom_{"odom"}, frame_imu_{"imu_link"}, frame_lidar_{"laser"};
  bool scan_only_{false};
  bool publish_lidar_debug_{false};
  std::chrono::steady_clock::time_point started_{}, next_discovery_{};
  std::set<std::string> missing_reported_;
  std::deque<uint32_t> recent_discovery_requests_;
  std::vector<std::string> ips_;
  std::unordered_map<std::string, std::shared_ptr<Vehicle>> vehicles_;
  rclcpp::Publisher<tianbot_core::msg::MiniDiscovery>::SharedPtr discovery_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif
