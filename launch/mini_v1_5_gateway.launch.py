from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tianbot_core",
            executable="mini_v1_5_gateway",
            name="mini_v1_5_gateway",
            namespace="mini",
            parameters=[{"remote_ip": "192.168.4.1", "session_id": 0}],
        ),
    ])
