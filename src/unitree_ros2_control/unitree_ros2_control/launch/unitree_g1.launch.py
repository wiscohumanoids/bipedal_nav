import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    g1_description_path = get_package_share_directory('g1_description')
    unitree_ros2_control_path = get_package_share_directory('unitree_ros2_control')

    # Parse URDF via xacro
    urdf_file = os.path.join(g1_description_path, 'g1_23dof_rev_1_0.urdf')
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    controller_config_file = os.path.join(
        unitree_ros2_control_path, 'config', 'g1_position_controllers.yaml')

    mujoco_model_path = os.path.join(
        g1_description_path, 'g1_23dof_rev_1_0.xml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': mujoco_model_path},
        ],
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'g1_position_trajectory_controller'],
        output='screen',
    )

    # Delay controller loading to give MuJoCo time to initialize
    delayed_broadcaster = TimerAction(
        period=3.0,
        actions=[load_joint_state_broadcaster],
    )

    return LaunchDescription([
        node_mujoco_ros2_control,
        node_robot_state_publisher,
        delayed_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
    ])
