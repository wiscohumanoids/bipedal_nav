"""
Motion Primitives Node

Publishes deterministic joint trajectories so that the simulated robot moves
and all sensors produce useful data — even before real locomotion policies
exist.  This lets SLAM, state-estimation, and locomotion teams begin
development immediately.

Modes (set via the ``mode`` ROS parameter):
    standing       – hold a stable standing pose
    squat          – bend knees then return to standing (repeating)
    shift_weight   – sway hips left/right
    step_forward   – small single-step motion
    arm_swing      – swing arms back and forth

Usage:
    ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=squat
"""

import math

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Joint ordering must match g1_position_trajectory_controller config
JOINT_NAMES = [
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_hip_yaw_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_hip_yaw_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
    'waist_yaw_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
]

# Indices into the joint list for readability
L_HIP_PITCH = 0
L_HIP_ROLL = 1
L_KNEE = 3
L_ANKLE_PITCH = 4
L_ANKLE_ROLL = 5
R_HIP_PITCH = 6
R_HIP_ROLL = 7
R_KNEE = 9
R_ANKLE_PITCH = 10
R_ANKLE_ROLL = 11
WAIST_YAW = 12
L_SHOULDER_PITCH = 13
L_SHOULDER_ROLL = 14
L_ELBOW = 16
R_SHOULDER_PITCH = 18
R_SHOULDER_ROLL = 19
R_ELBOW = 21

# Stable standing pose — matches the MJCF "stand" keyframe.
STANDING_POSE = [
    -0.2,   # left_hip_pitch
     0.0,   # left_hip_roll
     0.0,   # left_hip_yaw
     0.4,   # left_knee
    -0.2,   # left_ankle_pitch
     0.0,   # left_ankle_roll
    -0.2,   # right_hip_pitch
     0.0,   # right_hip_roll
     0.0,   # right_hip_yaw
     0.4,   # right_knee
    -0.2,   # right_ankle_pitch
     0.0,   # right_ankle_roll
     0.0,   # waist_yaw
     0.3,   # left_shoulder_pitch
     0.15,  # left_shoulder_roll
     0.0,   # left_shoulder_yaw
     0.5,   # left_elbow
     0.0,   # left_wrist_roll
     0.3,   # right_shoulder_pitch
    -0.15,  # right_shoulder_roll
     0.0,   # right_shoulder_yaw
     0.5,   # right_elbow
     0.0,   # right_wrist_roll
]

VALID_MODES = ['standing', 'squat', 'shift_weight', 'step_forward', 'arm_swing']


class MotionPrimitivesNode(Node):
    def __init__(self):
        super().__init__('motion_primitives_node')

        self.declare_parameter('mode', 'standing')
        self.mode = self.get_parameter('mode').value

        if self.mode not in VALID_MODES:
            self.get_logger().error(
                f"Unknown mode '{self.mode}'. Valid modes: {VALID_MODES}")
            raise SystemExit(1)

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/g1_position_trajectory_controller/joint_trajectory',
            10,
        )

        self.t = 0.0
        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(f"Motion primitives node started — mode: {self.mode}")

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        self.t += self.dt

        if self.mode == 'standing':
            positions = self._standing()
        elif self.mode == 'squat':
            positions = self._squat()
        elif self.mode == 'shift_weight':
            positions = self._shift_weight()
        elif self.mode == 'step_forward':
            positions = self._step_forward()
        elif self.mode == 'arm_swing':
            positions = self._arm_swing()
        else:
            return

        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=20_000_000)
        traj.points = [point]
        self.trajectory_pub.publish(traj)

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------

    def _standing(self):
        """Hold a stable standing pose."""
        return list(STANDING_POSE)

    def _squat(self):
        """Bend knees then return to standing (period ~4 s)."""
        pos = list(STANDING_POSE)
        phase = 0.5 * (1.0 - math.cos(2.0 * math.pi * self.t / 4.0))  # 0→1→0

        squat_depth = 0.3   # additional knee bend (rad)
        # Hip must pitch forward MORE than ankle to keep CoM over feet,
        # because the knee link has a backward x-offset (-0.078m).
        hip_comp = squat_depth * 0.8
        ankle_comp = squat_depth * 0.2

        pos[L_HIP_PITCH] -= hip_comp * phase
        pos[R_HIP_PITCH] -= hip_comp * phase
        pos[L_KNEE] += squat_depth * phase
        pos[R_KNEE] += squat_depth * phase
        pos[L_ANKLE_PITCH] -= ankle_comp * phase
        pos[R_ANKLE_PITCH] -= ankle_comp * phase
        return pos

    def _shift_weight(self):
        """Sway hips left/right (period ~5 s)."""
        pos = list(STANDING_POSE)
        phase = math.sin(2.0 * math.pi * self.t / 5.0)

        # Keep roll small — ankle roll range is only ±0.26 rad
        roll_amp = 0.04
        pos[L_HIP_ROLL] += roll_amp * phase
        pos[R_HIP_ROLL] += roll_amp * phase
        pos[L_ANKLE_ROLL] -= roll_amp * phase
        pos[R_ANKLE_ROLL] -= roll_amp * phase
        return pos

    def _step_forward(self):
        """Visible stepping-in-place motion cycling at ~5 s period."""
        pos = list(STANDING_POSE)
        period = 5.0
        t_mod = self.t % period
        half = period / 2.0

        step_len = 0.15   # hip pitch forward swing
        lift = 0.25       # knee lift
        roll_shift = 0.06 # weight shift via hip roll

        if t_mod < half:
            # Shift weight right, swing left leg forward
            phase = 0.5 * (1.0 - math.cos(2.0 * math.pi * t_mod / half))

            pos[L_HIP_PITCH] -= step_len * phase
            pos[L_KNEE] += lift * phase
            pos[L_ANKLE_PITCH] -= step_len * 0.5 * phase

            # lean onto right foot
            pos[L_HIP_ROLL] += roll_shift * phase
            pos[R_HIP_ROLL] += roll_shift * phase
            pos[L_ANKLE_ROLL] -= roll_shift * phase
            pos[R_ANKLE_ROLL] -= roll_shift * phase
        else:
            # Shift weight left, swing right leg forward
            phase = 0.5 * (1.0 - math.cos(2.0 * math.pi * (t_mod - half) / half))

            pos[R_HIP_PITCH] -= step_len * phase
            pos[R_KNEE] += lift * phase
            pos[R_ANKLE_PITCH] -= step_len * 0.5 * phase

            # lean onto left foot
            pos[L_HIP_ROLL] -= roll_shift * phase
            pos[R_HIP_ROLL] -= roll_shift * phase
            pos[L_ANKLE_ROLL] += roll_shift * phase
            pos[R_ANKLE_ROLL] += roll_shift * phase
        return pos

    def _arm_swing(self):
        """Swing arms back and forth (period ~2 s)."""
        pos = list(STANDING_POSE)
        phase = math.sin(2.0 * math.pi * self.t / 2.0)

        swing_amp = 0.5
        pos[L_SHOULDER_PITCH] += swing_amp * phase
        pos[R_SHOULDER_PITCH] -= swing_amp * phase  # opposite arm

        # slight elbow bend variation
        elbow_amp = 0.2
        pos[L_ELBOW] += elbow_amp * max(0.0, phase)
        pos[R_ELBOW] += elbow_amp * max(0.0, -phase)
        return pos


def main(args=None):
    rclpy.init(args=args)
    node = MotionPrimitivesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
