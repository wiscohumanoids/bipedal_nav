# RobotState.msg
# Published by: State Estimation team
# Consumed by: Locomotion team
#
# Full robot state estimate fusing IMU, joint encoders, and SLAM pose.
# This is THE input to the locomotion policy.

std_msgs/Header header

# --- Base (pelvis) state in world frame ---
geometry_msgs/Pose base_pose
geometry_msgs/Twist base_twist

# --- IMU data (body frame) ---
geometry_msgs/Vector3 angular_velocity    # from gyroscope
geometry_msgs/Vector3 linear_acceleration # from accelerometer
geometry_msgs/Quaternion orientation      # filtered orientation

# --- Joint state (ordered by joint index, same order as controller config) ---
float64[] joint_positions     # radians
float64[] joint_velocities    # rad/s
float64[] joint_efforts       # Nm (torque)

# --- Foot contact state ---
bool left_foot_contact
bool right_foot_contact

# --- Phase variable for gait (0.0 to 1.0) ---
float64 gait_phase
