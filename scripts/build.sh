#!/bin/bash
# =============================================================================
# Build the workspace
# Usage:
#   ./scripts/build.sh              — build everything
#   ./scripts/build.sh slam         — build SLAM packages only
#   ./scripts/build.sh state_est    — build State Estimation packages only
#   ./scripts/build.sh locomotion   — build Locomotion packages only
#   ./scripts/build.sh sim          — build simulation layer only
#   ./scripts/build.sh interfaces   — build shared interfaces only
# =============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

# Source ROS2
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

case "${1:-all}" in
    slam)
        colcon build --symlink-install --packages-select slam_pkg
        ;;
    state_est)
        colcon build --symlink-install --packages-select state_estimation_pkg
        ;;
    locomotion)
        colcon build --symlink-install --packages-select locomotion_pkg
        ;;
    sim)
        colcon build --symlink-install --packages-select g1_description mujoco_ros2_control mujoco_ros2_control_demos unitree_ros2_control
        ;;
    interfaces)
        colcon build --symlink-install --packages-select humanoid_interfaces
        ;;
    all)
        colcon build --symlink-install
        ;;
    *)
        echo "Usage: $0 {all|slam|state_est|locomotion|sim|interfaces}"
        exit 1
        ;;
esac

echo ""
echo "Build complete. Run: source install/setup.bash"
