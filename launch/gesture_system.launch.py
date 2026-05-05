"""
Gesture System - Real Hardware Launch
Includes MAVROS + all gesture nodes (no test_mode)

Usage: ros2 launch drone_gesture gesture_system.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('drone_gesture'), 'config')
    params_file = os.path.join(config_dir, 'gesture_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('video_source',
            default_value='rtsp://192.168.1.10:554/stream'),
        DeclareLaunchArgument('fcu_url',
            default_value='serial:///dev/ttyTHS1:57600'),

        # MAVROS (飞控通信)
        Node(package='mavros', executable='mavros_node', name='mavros',
             output='screen', parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'namespace': 'mavros',
             }]),

        # 手势识别
        Node(package='drone_gesture', executable='gesture_recognizer',
             name='gesture_recognizer', output='screen',
             parameters=[params_file,
                 {'video_source': LaunchConfiguration('video_source')}]),

        # 离散命令 (起飞/降落)
        Node(package='drone_gesture', executable='gesture_commander',
             name='gesture_commander', output='screen',
             parameters=[params_file, {'test_mode': False}]),

        # 连续速度控制
        Node(package='drone_gesture', executable='gesture_velocity_controller',
             name='gesture_velocity_controller', output='screen',
             parameters=[params_file, {'test_mode': False}]),

        # 安全监控
        Node(package='drone_gesture', executable='safety_monitor',
             name='safety_monitor', output='screen',
             parameters=[params_file]),

        # 诊断
        Node(package='drone_gesture', executable='diagnostics',
             name='diagnostics', output='screen'),
    ])
