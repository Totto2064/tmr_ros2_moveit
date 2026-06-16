from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('f710_joy_servo')
    config_file = os.path.join(pkg_dir, 'config', 'f710_mapping.yaml')

    return LaunchDescription([
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device_id': 0}],
        ),
        # Joy to Servo converter
        Node(
            package='f710_joy_servo',
            executable='joy_to_servo',
            name='joy_to_servo',
            parameters=[config_file],
            output='screen',
        ),
        
        # Servo trajectory bridge
        Node(
            package='f710_joy_servo',
            executable='servo_trajectory_bridge',
            name='servo_trajectory_bridge',
            output='screen',
        ),
    ])
