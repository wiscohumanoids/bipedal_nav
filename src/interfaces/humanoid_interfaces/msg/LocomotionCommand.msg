# LocomotionCommand.msg
# Published by: Locomotion team (RL policy output)
# Consumed by: Joint trajectory controller / MuJoCo simulation
#
# The RL policy outputs target joint positions at each control step.

std_msgs/Header header

# Target joint positions (radians), same ordering as controller config
float64[] joint_positions

# Target joint velocities (rad/s), optional feedforward
float64[] joint_velocities

# Target joint torques (Nm), optional feedforward
float64[] joint_torques

# Which control mode to use
uint8 MODE_POSITION = 0
uint8 MODE_VELOCITY = 1
uint8 MODE_TORQUE = 2
uint8 control_mode
