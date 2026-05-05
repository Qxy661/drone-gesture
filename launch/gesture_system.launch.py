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
        DeclareLaunchArgument('video_source', default_value='rtsp://192.168.1.10:554/stream'),
        DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyTHS1:57600'),

        Node(
            package='mavros', executable='mavros_node', name='mavros', output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'namespace': 'mavros',
            }],
        ),
        Node(
            package='drone_gesture', executable='gesture_recognizer',
            name='gesture_recognizer', output='screen',
            parameters=[params_file, {'video_source': LaunchConfiguration('video_source')}],
        ),
        Node(
            package='drone_gesture', executable='gesture_commander',
            name='gesture_commander', output='screen',
            parameters=[params_file, {'test_mode': False}],
        ),
    ])
