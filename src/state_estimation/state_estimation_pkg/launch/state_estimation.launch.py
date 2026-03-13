import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('state_estimation_pkg'),
        'config', 'state_estimation_params.yaml')

    return LaunchDescription([
        Node(
            package='state_estimation_pkg',
            executable='state_estimator_node',
            name='state_estimator_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='state_estimation_pkg',
            executable='contact_estimator_node',
            name='contact_estimator_node',
            output='screen',
            parameters=[config],
        ),
    ])
