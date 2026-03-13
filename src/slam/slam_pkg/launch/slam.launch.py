import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('slam_pkg'), 'config', 'slam_params.yaml')

    return LaunchDescription([
        Node(
            package='slam_pkg',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='slam_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[config],
        ),
    ])
