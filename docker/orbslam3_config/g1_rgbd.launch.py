#!/usr/bin/python3
"""
Launch ORB-SLAM3 in RGBD mode for the Unitree G1 robot.

This connects to the MuJoCo simulation camera topics:
  /head_camera/color  (sensor_msgs/Image, rgb8)
  /head_camera/depth  (sensor_msgs/Image, 32FC1)

Usage:
  ros2 launch orb_slam3_ros2_wrapper g1_rgbd.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    orb_wrapper_pkg = get_package_share_directory('orb_slam3_ros2_wrapper')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation clock if true')

    robot_namespace = LaunchConfiguration('robot_namespace')
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace', default_value='',
        description='The namespace of the robot')

    def all_nodes_launch(context, robot_namespace):
        params_file = LaunchConfiguration('params_file')
        vocabulary_file_path = '/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt'
        config_file_path = '/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/g1_rgbd.yaml'

        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                orb_wrapper_pkg, 'params', 'ros_params', 'g1-rgbd-ros-params.yaml'),
            description='Full path to the ROS2 parameters file')

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=robot_namespace.perform(context),
            param_rewrites={},
            convert_types=True)

        orb_slam3_node = Node(
            package='orb_slam3_ros2_wrapper',
            executable='rgbd',
            output='screen',
            namespace=robot_namespace.perform(context),
            arguments=[vocabulary_file_path, config_file_path],
            parameters=[configured_params])

        return [declare_params_file_cmd, orb_slam3_node]

    opaque_function = OpaqueFunction(
        function=all_nodes_launch, args=[robot_namespace])

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        opaque_function,
    ])
