#!/bin/bash
# =============================================================================
# Humanoid Robotics Workspace — Environment Setup Script
# =============================================================================
# Run this ONCE after cloning the repo to install all dependencies.
# Assumes Ubuntu 22.04 with ROS2 Humble already installed.
# =============================================================================

set -e

YELLOW='\033[1;33m'
GREEN='\033[1;32m'
RED='\033[1;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} Humanoid Robotics Workspace Setup${NC}"
echo -e "${GREEN}========================================${NC}"

# Check ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ERROR: ROS2 is not sourced. Run: source /opt/ros/humble/setup.bash${NC}"
    echo "If ROS2 is not installed, see STARTUP_GUIDE.md for instructions."
    exit 1
fi
echo -e "${GREEN}[OK]${NC} ROS2 $ROS_DISTRO detected"

# Install ROS2 dependencies
echo -e "${YELLOW}[1/5] Installing ROS2 packages...${NC}"
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-controller-manager \
    python3-colcon-common-extensions \
    python3-rosdep

# Install system dependencies
echo -e "${YELLOW}[2/5] Installing system dependencies...${NC}"
sudo apt install -y \
    libglfw3-dev \
    libeigen3-dev \
    cmake \
    build-essential \
    python3-pip \
    python3-numpy \
    git

# Install MuJoCo
echo -e "${YELLOW}[3/5] Checking MuJoCo installation...${NC}"
if [ -z "$MUJOCO_DIR" ]; then
    MUJOCO_DEFAULT="$HOME/.mujoco/mujoco-3.2.7"
    if [ -d "$MUJOCO_DEFAULT" ]; then
        echo -e "${GREEN}[OK]${NC} MuJoCo found at $MUJOCO_DEFAULT"
        export MUJOCO_DIR="$MUJOCO_DEFAULT"
    else
        echo -e "${YELLOW}MuJoCo not found. Installing MuJoCo 3.2.7...${NC}"
        mkdir -p "$HOME/.mujoco"
        cd "$HOME/.mujoco"
        wget -q "https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz"
        tar -xzf mujoco-3.2.7-linux-x86_64.tar.gz
        rm mujoco-3.2.7-linux-x86_64.tar.gz
        export MUJOCO_DIR="$HOME/.mujoco/mujoco-3.2.7"
        echo -e "${GREEN}[OK]${NC} MuJoCo installed at $MUJOCO_DIR"
    fi
else
    echo -e "${GREEN}[OK]${NC} MUJOCO_DIR is set to $MUJOCO_DIR"
fi

# Add environment variables to .bashrc if not already present
echo -e "${YELLOW}[4/5] Configuring environment variables...${NC}"
BASHRC="$HOME/.bashrc"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

grep -q "MUJOCO_DIR" "$BASHRC" || echo "export MUJOCO_DIR=\"$MUJOCO_DIR\"" >> "$BASHRC"
grep -q "humanoid_ws/install/setup.bash" "$BASHRC" || echo "# Humanoid workspace
source $WS_DIR/install/setup.bash 2>/dev/null || true" >> "$BASHRC"

# Install Python dependencies
echo -e "${YELLOW}[5/5] Installing Python dependencies...${NC}"
pip3 install --user \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    torch --quiet 2>/dev/null || true

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} Setup complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Next steps:"
echo "  1. source ~/.bashrc"
echo "  2. cd $WS_DIR"
echo "  3. colcon build --symlink-install"
echo "  4. source install/setup.bash"
echo "  5. ros2 launch unitree_ros2_control unitree_g1.launch.py"
echo ""
