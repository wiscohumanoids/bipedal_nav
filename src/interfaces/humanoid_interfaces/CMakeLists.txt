cmake_minimum_required(VERSION 3.8)
project(humanoid_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ============================================================
# MESSAGE DEFINITIONS — The contracts between teams
# ============================================================
# Each .msg file defines a data type that crosses team boundaries.
# Teams must agree on these interfaces BEFORE independent development.
# Changes to these files require sign-off from all affected teams.
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  # --- SLAM Team outputs ---
  "msg/SlamState.msg"          # SLAM → State Estimation: map + robot pose
  "msg/NavigationGoal.msg"     # High-level nav command (e.g. "walk 10m forward")

  # --- State Estimation outputs ---
  "msg/RobotState.msg"         # State Est → Locomotion: full body state
  "msg/ContactState.msg"       # State Est → Locomotion: foot contact info

  # --- Locomotion outputs ---
  "msg/LocomotionCommand.msg"  # Locomotion → MuJoCo: joint-level commands

  # --- Action for navigation ---
  "action/Navigate.action"     # End-to-end navigation action

  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

ament_package()
