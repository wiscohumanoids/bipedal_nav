# ContactState.msg
# Published by: State Estimation team
# Consumed by: Locomotion team
#
# Foot contact information for gait control.

std_msgs/Header header

# Contact booleans
bool left_foot_contact
bool right_foot_contact

# Contact forces (z-axis, Newtons)
float64 left_foot_force
float64 right_foot_force

# Center of pressure (CoP) in foot frame
geometry_msgs/Point left_cop
geometry_msgs/Point right_cop
