# NavigationGoal.msg
# Published by: High-level planner / user command
# Consumed by: SLAM team (path planning) → State Estimation → Locomotion
#
# A high-level movement command: "walk X meters in direction Y"

std_msgs/Header header

# Target pose in map frame
geometry_msgs/PoseStamped target_pose

# Desired linear velocity magnitude (m/s)
float64 desired_speed

# Command type
uint8 COMMAND_WALK_FORWARD = 0
uint8 COMMAND_WALK_BACKWARD = 1
uint8 COMMAND_TURN_LEFT = 2
uint8 COMMAND_TURN_RIGHT = 3
uint8 COMMAND_STOP = 4
uint8 command_type

# Distance to travel (meters), used with WALK commands
float64 distance
