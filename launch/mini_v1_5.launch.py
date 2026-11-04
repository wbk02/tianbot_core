import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tianbot'),
        DeclareLaunchArgument('remote_ip', default_value='192.168.4.1'),
        DeclareLaunchArgument('device_name', default_value=''),
        DeclareLaunchArgument('device_mac', default_value=''),
        DeclareLaunchArgument('command_port', default_value='5000'),
        DeclareLaunchArgument('management_port', default_value='5003'),
        DeclareLaunchArgument('bind_telemetry_port', default_value='5001'),
        DeclareLaunchArgument('bind_lidar_port', default_value='5002'),
        DeclareLaunchArgument('max_linear_velocity', default_value='1.5'),
        DeclareLaunchArgument('max_angular_velocity', default_value='2.0'),
        DeclareLaunchArgument('command_timeout_ms', default_value='500'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('session_id', default_value='0'),

        Node(
            package='tianbot_core',
            executable='tianbot_core',
            name='tianbot_core',
            output='screen', emulate_tty=True,
            additional_env={'RCUTILS_COLORIZED_OUTPUT': '1' if sys.stdout.isatty() and 'NO_COLOR' not in os.environ else '0'},
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'type': 'mini_v1_5',
                'type_verify': False,
                'remote_ip': LaunchConfiguration('remote_ip'),
                'device_name': LaunchConfiguration('device_name'),
                'device_mac': LaunchConfiguration('device_mac'),
                'command_port': LaunchConfiguration('command_port'),
                'management_port': LaunchConfiguration('management_port'),
                'bind_telemetry_port': LaunchConfiguration('bind_telemetry_port'),
                'bind_lidar_port': LaunchConfiguration('bind_lidar_port'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'command_timeout_ms': LaunchConfiguration('command_timeout_ms'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'imu_frame': LaunchConfiguration('imu_frame'),
                'lidar_frame': LaunchConfiguration('lidar_frame'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'session_id': LaunchConfiguration('session_id'),
            }],
        )
    ])
