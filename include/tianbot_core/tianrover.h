#ifndef TIANBOT_CORE_TIANROVER_H_
#define TIANBOT_CORE_TIANROVER_H_

#include <rclcpp/rclcpp.hpp>

#include "chassis.h"
#include "geometry_msgs/msg/twist.hpp"

/**
 * @brief Tianrover 六轮六转底盘的 ROS 2 通信适配。
 *
 * 仅转发车体速度命令；运动学、电机控制及安全保护由底盘固件负责。
 */
class TianbotTianrover : public TianbotChasis
{
public:
    explicit TianbotTianrover(const std::shared_ptr<rclcpp::Node> &node);

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    void velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);
};

#endif
