"""
Path Planner Node — Stub Implementation

Takes a NavigationGoal and the current SLAM map, plans a path,
and publishes velocity commands for the robot to follow.

DEVELOPMENT GUIDE:
- Subscribe to /slam/map for occupancy grid
- Subscribe to /navigation/goal for target destinations
- Implement A* or RRT path planning on the occupancy grid
- Publish velocity commands as geometry_msgs/Twist on /cmd_vel

INPUT TOPICS:
  /slam/map           (nav_msgs/OccupancyGrid)
  /navigation/goal    (humanoid_interfaces/NavigationGoal)
  /slam/state         (humanoid_interfaces/SlamState) — for current pose

OUTPUT TOPICS:
  /cmd_vel            (geometry_msgs/Twist) — desired walking velocity
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.get_logger().info('Path Planner Node initialized (stub)')

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/slam/map', self.map_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_map = None

    def map_callback(self, msg):
        self.current_map = msg

    def plan_path(self, start, goal):
        """Implement path planning algorithm (A*, RRT, etc.)."""
        # TODO: SLAM team implements this
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
