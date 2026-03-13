"""
Full Stack Launch — Launches the complete humanoid pipeline:
  MuJoCo Sim → SLAM → State Estimation → Locomotion

Use this to test the full integrated system.
Individual teams should use their own launch files during development.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 1. MuJoCo simulation + ros2_control (base layer)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('unitree_ros2_control'),
                'launch', 'unitree_g1.launch.py')))

    # 2. SLAM (delayed to let sim start)
    slam_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_pkg'),
                    'launch', 'slam.launch.py')))])

    # 3. State Estimation (delayed to let sim start)
    state_est_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('state_estimation_pkg'),
                    'launch', 'state_estimation.launch.py')))])

    # 4. Locomotion (delayed to let state estimation start)
    locomotion_launch = TimerAction(
        period=7.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('locomotion_pkg'),
                    'launch', 'locomotion.launch.py')))])

    return LaunchDescription([
        sim_launch,
        slam_launch,
        state_est_launch,
        locomotion_launch,
    ])
