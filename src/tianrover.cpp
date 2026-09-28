#include "tianrover.h"

#include "protocol.h"

static_assert(sizeof(struct twist) == 24,
              "Tianrover firmware requires a 24-byte twist payload");

void TianbotTianrover::velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
{
    struct twist twist_cmd = {};
    std::vector<uint8_t> buf;

    // 固件仅使用平面 VX、VY 和 WZ，其他分量固定为零以保证协议语义明确。
    twist_cmd.linear.x = msg->linear.x;
    twist_cmd.linear.y = msg->linear.y;
    twist_cmd.angular.z = msg->angular.z;

    buildCmd(buf, PACK_TYPE_CMD_VEL, reinterpret_cast<uint8_t *>(&twist_cmd), sizeof(twist_cmd));
    if (comm_inf_->send(buf.data(), buf.size()) != 0)
    {
        delete comm_inf_;
        comm_inf_ = nullptr;
        RCLCPP_ERROR(node->get_logger(), "Tianrover communication failed, reopen device");
        heartbeat_timer_->cancel();
        communication_timer_->cancel();
        open();
        communication_timer_->reset();
    }

    // 与现有车型一致：每次控制命令均重置心跳计时。
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}

TianbotTianrover::TianbotTianrover(const std::shared_ptr<rclcpp::Node> &node)
    : TianbotChasis(node)
{
    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1,
        std::bind(&TianbotTianrover::velocityCallback, this, std::placeholders::_1));

    initDone_ = true;
}
