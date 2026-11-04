import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    args = [
        ('vehicle_ips', ''), ('discovery_address', ''), ('scan_only', 'false'),
        ('max_vehicles', '5'), ('command_port', '5000'), ('management_port', '5003'),
        ('bind_telemetry_port', '5001'), ('bind_lidar_port', '5002'),
        ('max_linear_velocity', '1.5'), ('max_angular_velocity', '2.0'),
        ('command_timeout_ms', '500'), ('multi_vehicle_mode', 'true'),
        ('publish_lidar_debug', 'false'), ('base_frame', 'base_link'),
        ('odom_frame', 'odom'), ('imu_frame', 'imu_link'), ('lidar_frame', 'laser')]
    return LaunchDescription([
        DeclareLaunchArgument(k, default_value=v) for k,v in args
    ] + [Node(package='tianbot_core', executable='tianbot_core', name='tianbot_core_multi', output='screen', emulate_tty=True,
            additional_env={'RCUTILS_COLORIZED_OUTPUT': '1' if sys.stdout.isatty() and 'NO_COLOR' not in os.environ else '0'},
        parameters=[{'type':'mini_v1_5_multi', **{k: LaunchConfiguration(k) for k,_ in args}}])])
