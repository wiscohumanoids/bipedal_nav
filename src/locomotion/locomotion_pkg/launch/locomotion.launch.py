import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('locomotion_pkg'),
        'config', 'locomotion_params.yaml')

    return LaunchDescription([
        Node(
            package='locomotion_pkg',
            executable='locomotion_node',
            name='locomotion_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='locomotion_pkg',
            executable='gait_scheduler_node',
            name='gait_scheduler_node',
            output='screen',
            parameters=[config],
        ),
    ])
