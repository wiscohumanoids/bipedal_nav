"""
State Estimator Node — Stub Implementation

Fuses sensor data into a unified RobotState message for the locomotion policy.

DATA FLOW:
  IMU (from MuJoCo)     ──┐
  Joint States           ──┼──→ Extended Kalman Filter ──→ RobotState
  SLAM Pose (optional)   ──┘

DEVELOPMENT GUIDE:
1. Start simple: just forward joint_states + IMU data as RobotState
2. Add an EKF/UKF to fuse IMU with joint kinematics for base pose estimation
3. Incorporate SLAM pose as a correction factor
4. Add gait phase estimation based on foot contact patterns

INPUT TOPICS:
  /joint_states          (sensor_msgs/JointState)      — from joint_state_broadcaster
  /imu/data              (sensor_msgs/Imu)             — from MuJoCo IMU sensor
  /slam/state            (humanoid_interfaces/SlamState) — from SLAM team (optional)

OUTPUT TOPICS:
  /state_estimation/robot_state   (humanoid_interfaces/RobotState)
  /state_estimation/contact       (humanoid_interfaces/ContactState)
  tf2: odom → base_link
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
import numpy as np


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__('state_estimator_node')
        self.get_logger().info('State Estimator Node initialized (stub)')

        # --- Subscribers ---
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # --- Publishers ---
        # TODO: Publish humanoid_interfaces/RobotState once interfaces are built
        # self.state_pub = self.create_publisher(RobotState, '/state_estimation/robot_state', 10)

        # --- State variables ---
        self.latest_joint_state = None
        self.latest_imu = None

        # EKF state vector: [pos(3), vel(3), orientation(4), gyro_bias(3), accel_bias(3)]
        self.state = np.zeros(16)
        self.state[6] = 1.0  # quaternion w component
        self.covariance = np.eye(15) * 0.01

        # --- Timer ---
        self.timer = self.create_timer(0.01, self.estimate_state)  # 100 Hz

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def imu_callback(self, msg):
        self.latest_imu = msg

    def estimate_state(self):
        """Main estimation loop — implement your EKF/UKF here."""
        if self.latest_joint_state is None:
            return

        # TODO: State Estimation team implements this
        # Phase 1: Pass-through (just repackage joint_states + IMU)
        # Phase 2: Implement EKF prediction step (IMU integration)
        # Phase 3: Implement EKF update step (joint kinematics + SLAM)
        # Phase 4: Add gait phase estimation
        pass


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
