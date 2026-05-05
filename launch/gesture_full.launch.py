"""
Gesture Recognition System - Full Mode Launch
All nodes: gesture_recognizer + gesture_commander + velocity_controller + safety + diagnostics

Usage:
  ros2 launch drone_gesture gesture_full.launch.py test_mode:=true
  ros2 launch drone_gesture gesture_full.launch.py test_mode:=false video_source:=rtsp://192.168.1.10:554/stream
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("drone_gesture"), "config")
    params_file = os.path.join(config_dir, "gesture_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("video_source", default_value="0"),
        DeclareLaunchArgument("fcu_url",
            default_value="serial:///dev/ttyTHS1:57600"),
        DeclareLaunchArgument("test_mode", default_value="true"),

        # 1. 手势识别 (摄像头 → 手势)
        Node(package="drone_gesture", executable="gesture_recognizer",
             name="gesture_recognizer", output="screen",
             parameters=[params_file,
                 {"video_source": LaunchConfiguration("video_source")}]),

        # 2. 离散手势命令 (起飞/降落/前进)
        Node(package="drone_gesture", executable="gesture_commander",
             name="gesture_commander", output="screen",
             parameters=[params_file,
                 {"test_mode": LaunchConfiguration("test_mode")}]),

        # 3. 连续手势速度控制 (手部运动 → 速度)
        Node(package="drone_gesture", executable="gesture_velocity_controller",
             name="gesture_velocity_controller", output="screen",
             parameters=[params_file,
                 {"test_mode": LaunchConfiguration("test_mode")}]),

        # 4. 安全监控
        Node(package="drone_gesture", executable="safety_monitor",
             name="safety_monitor", output="screen",
             parameters=[params_file]),

        # 5. 系统诊断
        Node(package="drone_gesture", executable="diagnostics",
             name="diagnostics", output="screen"),
    ])
