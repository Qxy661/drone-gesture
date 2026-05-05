import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('drone_gesture'), 'config')
    params_file = os.path.join(config_dir, 'gesture_params.yaml')

    return LaunchDescription([
        Node(
            package='drone_gesture',
            executable='gesture_recognizer',
            name='gesture_recognizer',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='drone_gesture',
            executable='gesture_commander',
            name='gesture_commander',
            output='screen',
            parameters=[params_file, {'test_mode': True}],
        ),
    ])
