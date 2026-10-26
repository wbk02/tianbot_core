from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot"),
        DeclareLaunchArgument("remote_ip", default_value="192.168.4.1"),
        DeclareLaunchArgument("device_name", default_value=""),
        Node(package="tianbot_core", executable="mini_v1_5_gateway",
             name="mini_v1_5_gateway", namespace=LaunchConfiguration("namespace"),
             parameters=[{"remote_ip": LaunchConfiguration("remote_ip"),
                          "device_name": LaunchConfiguration("device_name"),
                          "session_id": 0}]),
    ])
