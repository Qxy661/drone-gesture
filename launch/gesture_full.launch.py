"""
Gesture Recognition System - Full Mode Launch
All nodes: MAVROS + gesture_recognizer + gesture_commander + safety + diagnostics

Usage: ros2 launch drone_gesture gesture_full.launch.py test_mode:=true
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
        DeclareLaunchArgument("video_source",
            default_value="rtsp://192.168.1.10:554/stream"),
        DeclareLaunchArgument("fcu_url",
            default_value="serial:///dev/ttyTHS1:57600"),
        DeclareLaunchArgument("test_mode", default_value="true"),

        Node(package="drone_gesture", executable="gesture_recognizer",
             name="gesture_recognizer", output="screen",
             parameters=[params_file,
                 {"video_source": LaunchConfiguration("video_source")}]),

        Node(package="drone_gesture", executable="gesture_commander",
             name="gesture_commander", output="screen",
             parameters=[params_file,
                 {"test_mode": LaunchConfiguration("test_mode")}]),

        Node(package="drone_gesture", executable="safety_monitor",
             name="safety_monitor", output="screen",
             parameters=[params_file]),

        Node(package="drone_gesture", executable="diagnostics",
             name="diagnostics", output="screen"),
    ])
