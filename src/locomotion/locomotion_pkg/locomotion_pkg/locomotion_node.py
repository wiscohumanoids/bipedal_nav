"""
Locomotion Node — Stub Implementation

Runs the RL locomotion policy: takes RobotState as input, outputs joint commands.

THE BIG PICTURE:
  The locomotion policy is a neural network (trained via RL in MuJoCo) that maps:
    observation = [base orientation, base angular velocity, joint positions,
                   joint velocities, previous action, velocity command]
    ──→ action = [target joint positions for next timestep]

DEVELOPMENT GUIDE (Phased Approach):

Phase 1 — Standing Controller (no RL):
  - Hardcode a PD controller that holds the robot at a default standing pose
  - This validates the full pipeline: state → policy → joint commands → MuJoCo

Phase 2 — Sinusoidal Gait (no RL):
  - Generate walking-like joint trajectories using sine waves
  - Tune frequencies, amplitudes, and phase offsets for each joint
  - Reference: CPG (Central Pattern Generator) approach

Phase 3 — RL Policy (sim-to-sim):
  - Train an RL policy in standalone MuJoCo (no ROS) using PPO/SAC
  - Use IsaacGym, MuJoCo MJX, or stable-baselines3
  - Export the trained policy (e.g., ONNX or PyTorch .pt file)
  - Load and run inference in this ROS node

INPUT TOPICS:
  /state_estimation/robot_state  (humanoid_interfaces/RobotState)
  /cmd_vel                       (geometry_msgs/Twist) — velocity command from planner

OUTPUT TOPICS:
  /locomotion/command            (humanoid_interfaces/LocomotionCommand)
  → Bridged to /joint_trajectory_controller/... by integration layer
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np


# Default standing pose (radians) — matches g1_23dof joint order
STANDING_POSE = {
    'left_hip_pitch_joint': -0.1,
    'left_hip_roll_joint': 0.0,
    'left_hip_yaw_joint': 0.0,
    'left_knee_joint': 0.25,
    'left_ankle_pitch_joint': -0.15,
    'left_ankle_roll_joint': 0.0,
    'right_hip_pitch_joint': -0.1,
    'right_hip_roll_joint': 0.0,
    'right_hip_yaw_joint': 0.0,
    'right_knee_joint': 0.25,
    'right_ankle_pitch_joint': -0.15,
    'right_ankle_roll_joint': 0.0,
    'waist_yaw_joint': 0.0,
    'left_shoulder_pitch_joint': 0.3,
    'left_shoulder_roll_joint': 0.15,
    'left_shoulder_yaw_joint': 0.0,
    'left_elbow_joint': 0.5,
    'left_wrist_roll_joint': 0.0,
    'right_shoulder_pitch_joint': 0.3,
    'right_shoulder_roll_joint': -0.15,
    'right_shoulder_yaw_joint': 0.0,
    'right_elbow_joint': 0.5,
    'right_wrist_roll_joint': 0.0,
}

JOINT_NAMES = list(STANDING_POSE.keys())


class LocomotionNode(Node):
    def __init__(self):
        super().__init__('locomotion_node')
        self.get_logger().info('Locomotion Node initialized (stub — standing controller)')

        # --- Subscribers ---
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # --- Publisher: send trajectory to ros2_control ---
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/g1_position_trajectory_controller/joint_trajectory',
            10)

        self.latest_joint_state = None
        self.cmd_vel = Twist()

        # Policy mode
        self.declare_parameter('mode', 'standing')  # 'standing', 'sinusoidal', 'rl'
        self.mode = self.get_parameter('mode').value

        # Control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        self.t = 0.0

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def control_loop(self):
        """Run the locomotion policy and publish joint commands."""
        self.t += 0.02

        if self.mode == 'standing':
            target_positions = [STANDING_POSE[j] for j in JOINT_NAMES]
        elif self.mode == 'sinusoidal':
            target_positions = self._sinusoidal_gait()
        elif self.mode == 'rl':
            target_positions = self._rl_policy()
        else:
            return

        # Publish as JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = Duration(sec=0, nanosec=20_000_000)  # 20ms
        traj.points = [point]

        self.trajectory_pub.publish(traj)

    def _sinusoidal_gait(self):
        """Generate a simple sinusoidal walking gait (CPG-style)."""
        positions = list(STANDING_POSE.values())

        # Walking frequency
        freq = 1.0  # Hz
        omega = 2.0 * np.pi * freq

        # Hip pitch oscillation (left and right are 180° out of phase)
        hip_amp = 0.3
        positions[0] += hip_amp * np.sin(omega * self.t)        # left hip pitch
        positions[6] += hip_amp * np.sin(omega * self.t + np.pi)  # right hip pitch

        # Knee oscillation (swing leg bends more)
        knee_amp = 0.4
        positions[3] += knee_amp * max(0, np.sin(omega * self.t))        # left knee
        positions[9] += knee_amp * max(0, np.sin(omega * self.t + np.pi))  # right knee

        # Ankle pitch (compensate)
        ankle_amp = 0.15
        positions[4] -= ankle_amp * np.sin(omega * self.t)        # left ankle
        positions[10] -= ankle_amp * np.sin(omega * self.t + np.pi)  # right ankle

        return positions

    def _rl_policy(self):
        """Run a trained RL policy for locomotion."""
        # TODO: Locomotion team implements this
        # 1. Build observation vector from latest_joint_state and cmd_vel
        # 2. Run neural network forward pass (torch, onnxruntime, etc.)
        # 3. Convert action to joint positions
        # 4. Return target positions
        self.get_logger().warn('RL policy not yet implemented, falling back to standing')
        return [STANDING_POSE[j] for j in JOINT_NAMES]


def main(args=None):
    rclpy.init(args=args)
    node = LocomotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
