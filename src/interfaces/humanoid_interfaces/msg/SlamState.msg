# SlamState.msg
# Published by: SLAM team
# Consumed by: State Estimation team, Integration/Navigation
#
# Represents the SLAM system's output: the robot's estimated pose in the
# world frame and an occupancy grid map of the environment.

std_msgs/Header header

# Robot pose in map frame (from visual/lidar odometry)
geometry_msgs/PoseWithCovariance pose_in_map

# Current velocity estimate from visual odometry
geometry_msgs/TwistWithCovariance velocity

# Occupancy grid map (2D for now, can extend to 3D)
nav_msgs/OccupancyGrid map

# Tracking quality (0.0 = lost, 1.0 = excellent)
float64 tracking_quality

# True if the SLAM system has a valid map and pose
bool is_initialized
