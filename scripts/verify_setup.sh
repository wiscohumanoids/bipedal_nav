#!/bin/bash
# =============================================================================
# Verify that the workspace is set up correctly
# Run this after building to confirm everything works.
# =============================================================================

set -e

GREEN='\033[1;32m'
RED='\033[1;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

check() {
    if eval "$2" > /dev/null 2>&1; then
        echo -e "  ${GREEN}[PASS]${NC} $1"
        PASS=$((PASS + 1))
    else
        echo -e "  ${RED}[FAIL]${NC} $1"
        FAIL=$((FAIL + 1))
    fi
}

echo "Verifying humanoid_ws setup..."
echo ""

echo "=== System Dependencies ==="
check "ROS2 sourced" "[ -n \"\$ROS_DISTRO\" ]"
check "colcon installed" "which colcon"
check "MuJoCo installed" "[ -n \"\$MUJOCO_DIR\" ] && [ -d \"\$MUJOCO_DIR\" ]"
check "GLFW installed" "dpkg -l libglfw3-dev"
check "Eigen3 installed" "dpkg -l libeigen3-dev"
check "Python3 numpy" "python3 -c 'import numpy'"

echo ""
echo "=== ROS2 Packages ==="
check "ros2_control" "ros2 pkg list 2>/dev/null | grep -q controller_manager"
check "joint_state_broadcaster" "ros2 pkg list 2>/dev/null | grep -q joint_state_broadcaster"
check "joint_trajectory_controller" "ros2 pkg list 2>/dev/null | grep -q joint_trajectory_controller"
check "xacro" "ros2 pkg list 2>/dev/null | grep -q xacro"

echo ""
echo "=== Workspace Packages ==="
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
check "g1_description built" "[ -d \"$WS_DIR/install/g1_description\" ]"
check "mujoco_ros2_control built" "[ -d \"$WS_DIR/install/mujoco_ros2_control\" ]"
check "unitree_ros2_control built" "[ -d \"$WS_DIR/install/unitree_ros2_control\" ]"
check "humanoid_interfaces built" "[ -d \"$WS_DIR/install/humanoid_interfaces\" ]"

echo ""
echo "=== Model Files ==="
check "G1 URDF exists" "[ -f \"$WS_DIR/src/g1_description/g1_23dof_rev_1_0.urdf\" ]"
check "G1 MJCF exists" "[ -f \"$WS_DIR/src/g1_description/g1_23dof_rev_1_0.xml\" ]"
check "Mesh files exist" "ls $WS_DIR/src/g1_description/meshes/*.STL > /dev/null 2>&1"

echo ""
echo "================================"
echo -e "Results: ${GREEN}$PASS passed${NC}, ${RED}$FAIL failed${NC}"
echo "================================"
