#include "core.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "ackermann.h"
#include "amp.h"
#include "differential.h"
#include "dyn.h"
#include "ndi.h"
#include "omni.h"
#include "rover.h"
#include "mini_v1_5.h"
#include "mini_v1_5_multi.h"
#include "chassis.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tianbot_core");

    std::string type;
    bool type_verify;

    TianbotCore *core = nullptr;

    node->declare_parameter("serial_port", rclcpp::PARAMETER_STRING);
    node->declare_parameter("client_ip", rclcpp::PARAMETER_STRING);
    node->declare_parameter("type",  rclcpp::PARAMETER_STRING);
    node->declare_parameter("type_verify", rclcpp::PARAMETER_BOOL);
    node->declare_parameter("base_frame", rclcpp::PARAMETER_STRING);
    node->declare_parameter("odom_frame", rclcpp::PARAMETER_STRING);
    node->declare_parameter("imu_frame", rclcpp::PARAMETER_STRING);
    node->declare_parameter("publish_tf",rclcpp::PARAMETER_BOOL);
    node->declare_parameter("multi_vehicle_mode", false);

    if (!node->get_parameter("type", type)) {
        type = DEFAULT_TYPE;
    }
    if (!node->get_parameter("type_verify", type_verify)) {
        type_verify = DEFAULT_TYPE_VERIFY;
    }

    if (type == "omni")
    {
        core = new TianbotOmni(node);
    }
    else if (type == "amp")
    {
        core = new TianbotAmp(node);
    }
    else if (type == "ndi")
    {
        core = new TianbotNdi(node);
    }
    else if (type == "ackermann")
    {
        core = new TianbotAckermann(node);
        printf( "new TianbotAckermann(node) success/n");
    }
    else if (type == "diff")
    {
        core = new TianbotDifferential(node);
    }
    else if (type == "dyn")
    {
        core = new TianbotDyn(node);
        type_verify = false;
    }
    else if (type == "rover")
    {
        core = new TianbotRover(node);
    }
    else if (type == "mini_v1_5_multi")
    {
        core = new TianbotMiniV15Multi(node);
        type_verify = false;
    }
    else if (type == "mini_v1_5")
    {
        core = new TianbotMiniV15(node);
        type_verify = false;
    }
    else if (type == "arm")
    {
        // Add arm initialization here
    }

    if (core != nullptr)
    {
        core->open();
        if (type_verify)
        {
            core->checkDevType();
        }
    }

    // rclcpp::Rate loop_rate(10);
    // while (rclcpp::ok())
    // {
    //     rclcpp::spin_some(node);
    //     loop_rate.sleep();
    // }
    if (type == "mini_v1_5" || type == "mini_v1_5_multi") {
        // 全部 Mini UDP 状态在单线程执行，减少执行器调度开销并保持会话状态串行访问。
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node); executor.spin();
    } else {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node); executor.spin();
    }
    delete core;
    rclcpp::shutdown();
    return 0;
}
