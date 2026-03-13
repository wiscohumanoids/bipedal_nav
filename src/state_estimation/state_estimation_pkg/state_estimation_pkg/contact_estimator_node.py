"""
Contact Estimator Node — Stub Implementation

Estimates foot contact state from force/torque sensor data or joint torques.

DEVELOPMENT GUIDE:
1. Subscribe to force/torque sensor data from MuJoCo
2. Apply threshold to detect contact (simple approach)
3. Optionally use a probabilistic contact estimator
4. Publish ContactState for the locomotion policy

INPUT TOPICS:
  /joint_states          (sensor_msgs/JointState)  — joint torques
  (future: F/T sensor topics from MuJoCo)

OUTPUT TOPICS:
  /state_estimation/contact  (humanoid_interfaces/ContactState)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ContactEstimatorNode(Node):
    def __init__(self):
        super().__init__('contact_estimator_node')
        self.get_logger().info('Contact Estimator Node initialized (stub)')

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Contact force threshold (Newtons)
        self.declare_parameter('contact_threshold', 10.0)
        self.contact_threshold = self.get_parameter('contact_threshold').value

    def joint_state_callback(self, msg):
        # TODO: State Estimation team implements contact detection
        # Option 1: Use ankle joint torques as a proxy for contact
        # Option 2: Use MuJoCo force/torque sensors (if added to MJCF)
        # Option 3: Use a learned contact estimator
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ContactEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
