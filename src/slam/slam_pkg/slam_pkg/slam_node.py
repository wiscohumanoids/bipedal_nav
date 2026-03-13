"""
SLAM Node — Stub Implementation

This node is the SLAM team's main entry point. It will:
1. Subscribe to camera images from MuJoCo simulation
2. Run visual SLAM (e.g., ORB-SLAM3, or a custom implementation)
3. Publish the robot's estimated pose in the map frame
4. Publish an occupancy grid map

DEVELOPMENT GUIDE:
- Start by subscribing to /camera/color/image_raw (sensor_msgs/Image)
- Use cv_bridge to convert ROS images to OpenCV format
- Implement or integrate a SLAM algorithm (ORB-SLAM3, rtabmap, etc.)
- Publish SlamState messages on /slam/state
- Publish tf2 transform: map → odom

INPUT TOPICS:
  /camera/color/image_raw  (sensor_msgs/Image)         — RGB camera
  /camera/depth/image_raw  (sensor_msgs/Image)         — Depth camera
  /camera/camera_info      (sensor_msgs/CameraInfo)    — Camera intrinsics

OUTPUT TOPICS:
  /slam/state              (humanoid_interfaces/SlamState)
  /slam/map                (nav_msgs/OccupancyGrid)
  tf2: map → odom frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Header


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.get_logger().info('SLAM Node initialized (stub)')

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # --- Publishers ---
        # TODO: Replace with humanoid_interfaces/SlamState once built
        self.map_pub = self.create_publisher(OccupancyGrid, '/slam/map', 10)

        # --- Timer for periodic processing ---
        self.timer = self.create_timer(0.1, self.slam_update)  # 10 Hz

        self.latest_image = None
        self.latest_depth = None

    def image_callback(self, msg):
        self.latest_image = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def slam_update(self):
        """Main SLAM processing loop — implement your SLAM algorithm here."""
        if self.latest_image is None:
            return

        # TODO: SLAM team implements this
        # 1. Extract features from image (ORB, SIFT, etc.)
        # 2. Match features across frames
        # 3. Estimate camera motion (visual odometry)
        # 4. Build/update map
        # 5. Publish SlamState and map
        # 6. Broadcast tf2 transform: map → odom
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
