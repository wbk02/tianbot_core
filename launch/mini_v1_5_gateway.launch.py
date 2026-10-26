from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot"),
        DeclareLaunchArgument("remote_ip", default_value="192.168.4.1"),
        DeclareLaunchArgument("device_name", default_value=""),
        DeclareLaunchArgument("device_mac", default_value=""),
        DeclareLaunchArgument("command_port", default_value="5000"),
        DeclareLaunchArgument("management_port", default_value="5003"),
        DeclareLaunchArgument("bind_telemetry_port", default_value="5001"),
        DeclareLaunchArgument("bind_lidar_port", default_value="5002"),
        DeclareLaunchArgument("max_linear_velocity", default_value="1.5"),
        DeclareLaunchArgument("max_angular_velocity", default_value="2.0"),
        DeclareLaunchArgument("command_timeout_ms", default_value="500"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="laser"),
        DeclareLaunchArgument("session_id", default_value="0"),
        Node(package="tianbot_core", executable="mini_v1_5_gateway",
             name="mini_v1_5_gateway", namespace=LaunchConfiguration("namespace"),
             parameters=[{"remote_ip": LaunchConfiguration("remote_ip"),
                          "device_name": LaunchConfiguration("device_name"),
                          "device_mac": LaunchConfiguration("device_mac"),
                          "command_port": LaunchConfiguration("command_port"),
                          "management_port": LaunchConfiguration("management_port"),
                          "bind_telemetry_port": LaunchConfiguration("bind_telemetry_port"),
                          "bind_lidar_port": LaunchConfiguration("bind_lidar_port"),
                          "max_linear_velocity": LaunchConfiguration("max_linear_velocity"),
                          "max_angular_velocity": LaunchConfiguration("max_angular_velocity"),
                          "command_timeout_ms": LaunchConfiguration("command_timeout_ms"),
                          "base_frame": LaunchConfiguration("base_frame"),
                          "lidar_frame": LaunchConfiguration("lidar_frame"),
                          "session_id": LaunchConfiguration("session_id")}]),
    ])
