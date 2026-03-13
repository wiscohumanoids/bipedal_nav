"""
Gait Scheduler Node — Stub Implementation

Translates high-level velocity commands (/cmd_vel) into gait parameters
(step frequency, step length, etc.) for the locomotion policy.

DEVELOPMENT GUIDE:
- Maps linear/angular velocity commands to gait parameters
- Manages state machine: IDLE → STARTING → WALKING → STOPPING → IDLE
- Publishes gait phase for the locomotion policy

INPUT TOPICS:
  /cmd_vel  (geometry_msgs/Twist) — from path planner

OUTPUT TOPICS:
  /locomotion/gait_params  (custom, or part of RobotState)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class GaitSchedulerNode(Node):
    def __init__(self):
        super().__init__('gait_scheduler_node')
        self.get_logger().info('Gait Scheduler Node initialized (stub)')

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.gait_phase_pub = self.create_publisher(
            Float64, '/locomotion/gait_phase', 10)

        self.timer = self.create_timer(0.01, self.update_gait)

        self.gait_phase = 0.0
        self.gait_frequency = 1.0  # Hz
        self.is_walking = False

    def cmd_vel_callback(self, msg):
        speed = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)
        self.is_walking = speed > 0.01
        if self.is_walking:
            self.gait_frequency = 0.5 + 1.5 * min(abs(msg.linear.x), 1.0)

    def update_gait(self):
        if self.is_walking:
            self.gait_phase += self.gait_frequency * 0.01
            if self.gait_phase >= 1.0:
                self.gait_phase -= 1.0
        else:
            self.gait_phase = 0.0

        msg = Float64()
        msg.data = self.gait_phase
        self.gait_phase_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
