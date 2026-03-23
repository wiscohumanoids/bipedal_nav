"""
SLAM Node — Proof-of-Concept Implementation

This node demonstrates the SLAM pipeline working end-to-end:
1. Subscribes to camera images from MuJoCo simulation
2. Processes depth images to build a simple occupancy grid
3. Publishes the occupancy grid map
4. Broadcasts tf2 transform: map -> odom

INPUT TOPICS:
  /head_camera/color       (sensor_msgs/Image)         — RGB camera
  /head_camera/depth       (sensor_msgs/Image)         — Depth camera (32FC1)
  /head_camera/camera_info (sensor_msgs/CameraInfo)    — Camera intrinsics

OUTPUT TOPICS:
  /slam/map                (nav_msgs/OccupancyGrid)
  tf2: map -> odom frame
"""

import struct
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, TransformStamped
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        # Declare parameters
        self.declare_parameter('image_topic', '/head_camera/color')
        self.declare_parameter('depth_topic', '/head_camera/depth')
        self.declare_parameter('camera_info_topic', '/head_camera/camera_info')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_size_x', 20.0)
        self.declare_parameter('map_size_y', 20.0)

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        update_rate = self.get_parameter('update_rate').value
        self.map_resolution = self.get_parameter('map_resolution').value
        map_size_x = self.get_parameter('map_size_x').value
        map_size_y = self.get_parameter('map_size_y').value

        # Map dimensions in cells
        self.map_width = int(map_size_x / self.map_resolution)
        self.map_height = int(map_size_y / self.map_resolution)

        # Initialize occupancy grid (-1 = unknown)
        self.occupancy_grid = [-1] * (self.map_width * self.map_height)

        self.get_logger().info(
            f'SLAM Node initialized — subscribing to {image_topic}, {depth_topic}')
        self.get_logger().info(
            f'Map: {self.map_width}x{self.map_height} cells at {self.map_resolution}m resolution')

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # --- Publishers ---
        self.map_pub = self.create_publisher(OccupancyGrid, '/slam/map', 10)

        # --- TF2 broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Timer for periodic processing ---
        self.timer = self.create_timer(1.0 / update_rate, self.slam_update)

        self.latest_image = None
        self.latest_depth = None
        self.camera_info = None
        self.image_count = 0
        self.depth_count = 0

    def image_callback(self, msg):
        self.latest_image = msg
        self.image_count += 1
        if self.image_count == 1:
            self.get_logger().info(
                f'Receiving RGB images: {msg.width}x{msg.height}, encoding={msg.encoding}')

    def depth_callback(self, msg):
        self.latest_depth = msg
        self.depth_count += 1
        if self.depth_count == 1:
            self.get_logger().info(
                f'Receiving depth images: {msg.width}x{msg.height}, encoding={msg.encoding}')

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().info(
                f'Received camera info: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}')
        self.camera_info = msg

    def slam_update(self):
        """Main SLAM processing loop."""
        if self.latest_depth is None:
            return

        # Broadcast identity transform map -> odom (placeholder for real SLAM)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # Build simple occupancy grid from depth data
        if self.latest_depth is not None and self.camera_info is not None:
            self._update_occupancy_from_depth()

        # Publish occupancy grid
        self._publish_map()

    def _update_occupancy_from_depth(self):
        """Project depth image into a simple 2D occupancy grid (top-down view)."""
        depth_msg = self.latest_depth
        width = depth_msg.width
        height = depth_msg.height
        data = depth_msg.data

        fx = self.camera_info.k[0]
        cx = self.camera_info.k[2]

        if fx == 0:
            return

        # Sample center row of depth image for a simple horizontal scan
        center_row = height // 2
        map_cx = self.map_width // 2
        map_cy = self.map_height // 2

        for col in range(0, width, 4):  # Sample every 4th pixel for speed
            # Read float32 depth value
            idx = (center_row * width + col) * 4  # 4 bytes per float
            if idx + 4 > len(data):
                continue
            depth = struct.unpack('f', bytes(data[idx:idx+4]))[0]

            if depth <= 0.1 or depth > 10.0 or math.isnan(depth) or math.isinf(depth):
                continue

            # Project to 3D (camera frame: z=forward, x=right)
            x_3d = (col - cx) / fx * depth
            z_3d = depth

            # Convert to map cell (robot at center, facing +z which maps to +y in map)
            map_x = int(map_cx + x_3d / self.map_resolution)
            map_y = int(map_cy + z_3d / self.map_resolution)

            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                cell_idx = map_y * self.map_width + map_x
                # Mark as occupied if close enough obstacle, free otherwise
                if depth < 5.0:
                    self.occupancy_grid[cell_idx] = 100  # occupied
                # Mark cells between robot and obstacle as free
                for r in range(1, int(depth / self.map_resolution)):
                    free_y = int(map_cy + r)
                    free_x = int(map_cx + (x_3d / depth) * r * self.map_resolution / self.map_resolution)
                    if 0 <= free_x < self.map_width and 0 <= free_y < self.map_height:
                        free_idx = free_y * self.map_width + free_x
                        if self.occupancy_grid[free_idx] == -1:
                            self.occupancy_grid[free_idx] = 0  # free

    def _publish_map(self):
        """Publish the occupancy grid."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin = Pose()
        msg.info.origin.position.x = -(self.map_width * self.map_resolution) / 2.0
        msg.info.origin.position.y = -(self.map_height * self.map_resolution) / 2.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.occupancy_grid
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
