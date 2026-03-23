# Startup Guide — Humanoid Robotics Workspace

This guide gets you from a fresh machine to running the Unitree G1 humanoid robot in MuJoCo simulation with ROS2. No prior ROS or robotics experience is assumed.

There are **two ways** to set up the workspace:

| Method | Best for | Pros | Cons |
|--------|----------|------|------|
| **Docker (recommended)** | Reproducibility, team onboarding | One command setup, identical environment for everyone | Requires Docker, GUI forwarding can be finicky |
| **Native** | Daily development, fastest iteration | No overhead, direct GPU access | Must install ROS2 + MuJoCo manually |

---

## Table of Contents

1. [Quick Start with Docker (Recommended)](#1-quick-start-with-docker-recommended)
2. [Native Setup](#2-native-setup)
3. [Run the Simulation](#3-run-the-simulation)
4. [ORB-SLAM3 Setup (SLAM Team)](#4-orb-slam3-setup-slam-team)
5. [Understand What's Running](#5-understand-whats-running)
6. [Explore the Codebase](#6-explore-the-codebase)
7. [Common Tasks](#7-common-tasks)
8. [Troubleshooting](#8-troubleshooting)
9. [Background Concepts](#9-background-concepts)

---

## 1. Quick Start with Docker (Recommended)

### Prerequisites

- **Docker:** Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) (Linux) or [Docker Desktop](https://www.docker.com/products/docker-desktop/) (macOS/Windows)
- That's it — no display server, GPU, or native ROS2 install needed. The sim runs headless inside Docker.

### Step 1: Clone the repo

```bash
git clone https://github.com/wiscohumanoids/bipedal_nav
cd humanoid_ws
```

### Step 2: Build the Docker image

```bash
./docker/build.sh
```

This builds an image called `humanoid_ws:latest` containing:
- ROS2 Humble
- MuJoCo 3.2.7 (C library)
- All ROS2 dependencies
- Python venv with numpy, scipy, opencv
- The entire workspace, pre-compiled

Build takes ~5-10 minutes on the first run. The image is ~2 GB.

### Step 3: Launch the simulation

```bash
# Headless mode (works on any OS — no display needed):
./docker/run.sh ros2 launch unitree_ros2_control unitree_g1.launch.py headless:=true

# With GUI (Linux/WSL2 only — requires X11/WSLg):
./docker/run.sh ros2 launch unitree_ros2_control unitree_g1.launch.py
```

### Step 4: Verify in a second terminal

The `run.sh` script auto-detects the running container and `exec`s into it:

```bash
# Check camera is publishing:
./docker/run.sh ros2 topic hz /head_camera/color

# See all active topics:
./docker/run.sh ros2 topic list
```

### Step 5: Run SLAM node (in a second terminal)

```bash
./docker/run.sh ros2 run slam_pkg slam_node --ros-args \
    --params-file install/slam_pkg/share/slam_pkg/config/slam_params.yaml
```

### Docker development workflow

When you edit code in `src/`, you need to rebuild inside the container (or rebuild the image). Two options:

**Option A — Rebuild inside running container (fast iteration):**
```bash
# Mount your source directory into the container for live editing:
docker run --rm -it \
    --network host \
    -e DISPLAY=:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/src:/home/ros2_ws/src:rw \
    humanoid_ws:latest

# Inside the container, rebuild changed packages:
colcon build --packages-select slam_pkg
source install/setup.bash
```

**Option B — Rebuild the image (clean slate):**
```bash
./docker/build.sh
```

---

## 2. Native Setup

Use this if you prefer to install everything directly on your machine.

### Prerequisites

- **OS:** Ubuntu 22.04 LTS (native or WSL2)
- **RAM:** 8 GB minimum (16 GB recommended)
- **Disk:** ~5 GB free space
- **GPU:** Not required for basic sim, helpful for RL training later

If you're on Windows, use WSL2:
```bash
wsl --install -d Ubuntu-22.04
```

### 2.1 Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop (includes RViz, demos, etc.)
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS2 in every new terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip
sudo rosdep init 2>/dev/null || true
rosdep update
```

**Verify:**
```bash
ros2 topic list
# Should print: /parameter_events and /rosout
```

### 2.2 Install MuJoCo

```bash
mkdir -p ~/.mujoco && cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz
tar -xzf mujoco-3.2.7-linux-x86_64.tar.gz
rm mujoco-3.2.7-linux-x86_64.tar.gz

echo 'export MUJOCO_DIR="$HOME/.mujoco/mujoco-3.2.7"' >> ~/.bashrc
source ~/.bashrc

sudo apt install -y libglfw3-dev libeigen3-dev
```

### 2.3 Clone, create venv, and build

```bash
cd ~
git clone <YOUR_GITHUB_URL> humanoid_ws
cd humanoid_ws

# Create and activate a Python virtual environment
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install numpy scipy opencv-python-headless xacro

# Install ROS2 dependencies
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
    ros-humble-effort-controllers ros-humble-robot-state-publisher \
    ros-humble-xacro ros-humble-tf2-ros ros-humble-controller-manager \
    python3-colcon-common-extensions

# Conda users: install ROS2 build deps in your conda env BEFORE building
# (empy must be 3.x — version 4.x is incompatible with ROS2 Humble)
# pip install catkin_pkg 'empy<4' lark

# Build the workspace
colcon build --symlink-install
source install/setup.bash

# Add to .bashrc for convenience
echo 'source ~/humanoid_ws/.venv/bin/activate' >> ~/.bashrc
echo 'source ~/humanoid_ws/install/setup.bash 2>/dev/null || true' >> ~/.bashrc
```

### 2.4 Launch the simulation (native)

```bash
# Make sure everything is sourced
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
export LD_LIBRARY_PATH=$MUJOCO_DIR/lib:$LD_LIBRARY_PATH

# For WSL2, set DISPLAY
export DISPLAY=:0

# Launch
ros2 launch unitree_ros2_control unitree_g1.launch.py
```

---

## 3. Run the Simulation

These commands work identically in Docker or native (just make sure you've sourced the workspace).

### Launch the G1 Robot in MuJoCo

```bash
# With GUI (native or Linux Docker with X11):
ros2 launch unitree_ros2_control unitree_g1.launch.py

# Headless (Docker on any OS):
ros2 launch unitree_ros2_control unitree_g1.launch.py headless:=true
```

You should see:
1. A MuJoCo window opens showing the G1 humanoid robot (GUI mode), or "headless mode initialized" log (headless mode)
2. The robot stands on a ground plane (it will collapse if no controller is sending commands)
3. Terminal output shows controllers being loaded

### Interact with the Simulation

Open new terminals (each needs the workspace sourced):

```bash
# See all active topics
ros2 topic list

# Watch joint states in real-time
ros2 topic echo /joint_states

# Check which controllers are loaded
ros2 control list_controllers
```

### Run the SLAM Node

```bash
ros2 run slam_pkg slam_node --ros-args \
    --params-file install/slam_pkg/share/slam_pkg/config/slam_params.yaml
```

The SLAM node will:
- Receive RGB and depth images from `/head_camera/color` and `/head_camera/depth`
- Build an occupancy grid from depth data
- Publish the map on `/slam/map` at 10 Hz
- Broadcast the `map -> odom` TF transform

### Try the Standing Controller

```bash
ros2 run locomotion_pkg locomotion_node --ros-args -p mode:=standing
```

### Try Motion Primitives

```bash
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=squat
# Other modes: standing, shift_weight, step_forward, arm_swing
```

### Verify Sensors Are Working

```bash
# Camera images (SLAM team)
ros2 topic hz /head_camera/color
ros2 topic hz /head_camera/depth

# IMU data (State Estimation team)
ros2 topic echo /imu/data

# Foot contact sensors (State Est + Locomotion teams)
ros2 topic echo /contact/left_foot
ros2 topic echo /contact/right_foot

# SLAM occupancy grid
ros2 topic hz /slam/map
```

---

## 4. ORB-SLAM3 Setup (SLAM Team)

ORB-SLAM3 runs in its own Docker container and connects to the MuJoCo simulation via ROS2 topics. No GPU required — it runs on CPU.

### Build the ORB-SLAM3 image

```bash
docker build -f docker/Dockerfile.orbslam3 -t orbslam3_ros2:latest .
```

This takes 10-20 minutes on the first build (compiles OpenCV 4.4.0, Pangolin, and ORB-SLAM3 from source). The image is ~4-5 GB.

### Run ORB-SLAM3

You need two containers running simultaneously — they communicate via ROS2 topics over the host network (`--network host`).

**Terminal 1 — MuJoCo simulation:**
```bash
./docker/run.sh ros2 launch unitree_ros2_control unitree_g1.launch.py headless:=true
```

**Terminal 2 — ORB-SLAM3:**
```bash
./docker/run_orbslam3.sh ros2 launch orb_slam3_ros2_wrapper g1_rgbd.launch.py
```

### Verify ORB-SLAM3 is working

Open a third terminal in either container:
```bash
# Check ORB-SLAM3 is publishing pose
ros2 topic echo /robot_pose_slam

# Check point cloud is being built
ros2 topic hz /map_points

# Check TF tree includes map -> odom from ORB-SLAM3
ros2 run tf2_ros tf2_echo map odom
```

### What ORB-SLAM3 publishes

| Topic | Type | Description |
|-------|------|-------------|
| `/robot_pose_slam` | `geometry_msgs/PoseStamped` | Robot pose in map frame |
| `/map_points` | `sensor_msgs/PointCloud2` | Sparse 3D feature point cloud |
| `/map_data` | custom | Full map data |
| `/slam_info` | custom | Tracking status |
| TF: `map -> odom` | tf2 | Localization transform |

### What the SLAM team builds on top

ORB-SLAM3 is the engine — it gives you pose + sparse points. The team builds:

1. **Dense mapping** — combine ORB-SLAM3 pose with depth images to build occupancy grids or 3D voxel maps
2. **Terrain export** — convert the map into MuJoCo MJCF format (heightfields or meshes) so the Unitree can walk on the reconstructed environment
3. **Path planning** — use the map to plan collision-free paths (`path_planner_node.py`)
4. **TF integration** — ensure `map -> odom -> base_link` chain works with the state estimation team

### Camera configuration

The ORB-SLAM3 camera config for the G1 is at `docker/orbslam3_config/g1_rgbd.yaml`. Key values:
- `fx=240, fy=240, cx=320, cy=240` (matches MuJoCo head_camera with fovy=90, 640x480)
- `Camera.fps: 6.0` (MuJoCo publishes at ~6 Hz)
- `Camera.RGB: 1` (MuJoCo publishes rgb8 format)
- `DepthMapFactor: 1.0` (MuJoCo depth is in meters)

When switching to a real camera (e.g. RealSense D435), create a new YAML with the real camera's intrinsics.

---

## 5. Understand What's Running

When you launch `unitree_g1.launch.py`, here's what happens:

```
unitree_g1.launch.py
|
+-- mujoco_ros2_control (C++ node)
|   +-- Loads g1_23dof_rev_1_0.xml into MuJoCo physics engine
|   +-- Opens rendering window (GLFW) or runs headless (OSMesa)
|   +-- Runs physics at 1000 Hz
|   +-- Publishes /clock for simulation time
|   +-- Publishes /head_camera/color and /head_camera/depth (~6 Hz)
|   +-- Publishes /head_camera/camera_info
|   +-- Publishes /imu/data (100 Hz)
|   +-- Publishes /contact/left_foot and /contact/right_foot
|
+-- robot_state_publisher
|   +-- Publishes /tf, /tf_static, /robot_description
|
+-- joint_state_broadcaster
|   +-- Publishes /joint_states at 100 Hz
|
+-- g1_position_trajectory_controller
    +-- Accepts joint trajectory commands
    +-- Sends position commands to MuJoCo
```

### The Control Loop

```
Your code publishes JointTrajectory
    -> g1_position_trajectory_controller receives it
    -> controller_manager calls write()
    -> MujocoSystem::write() sets mj_data->qpos[] in MuJoCo
    -> MuJoCo physics engine steps forward
    -> MujocoSystem::read() reads new joint positions
    -> joint_state_broadcaster publishes /joint_states
    -> Your code reads the new state
    -> (loop)
```

---

## 6. Explore the Codebase

### Where Things Are

| What | Where | Language |
|------|-------|----------|
| Robot 3D model (physics) | `src/g1_description/g1_23dof_rev_1_0.xml` | MJCF/XML |
| Robot description (ROS) | `src/g1_description/g1_23dof_rev_1_0.urdf` | URDF/XML |
| Robot meshes | `src/g1_description/meshes/` | STL |
| MuJoCo-ROS bridge | `src/mujoco_ros2_control/mujoco_ros2_control/src/` | C++ |
| Simulation launch | `src/unitree_ros2_control/.../launch/unitree_g1.launch.py` | Python |
| Controller config | `src/unitree_ros2_control/.../config/g1_position_controllers.yaml` | YAML |
| Team interfaces | `src/interfaces/humanoid_interfaces/msg/` | ROS2 IDL |
| SLAM code | `src/slam/slam_pkg/slam_pkg/` | Python |
| State estimation code | `src/state_estimation/state_estimation_pkg/` | Python |
| Locomotion code | `src/locomotion/locomotion_pkg/locomotion_pkg/` | Python |
| Integration launch | `src/integration/launch/full_stack.launch.py` | Python |
| Motion primitives | `src/integration/motion_primitives_pkg/` | Python |
| **Docker (main)** | `Dockerfile`, `docker/build.sh`, `docker/run.sh` | Shell/Docker |
| **Docker (ORB-SLAM3)** | `docker/Dockerfile.orbslam3`, `docker/run_orbslam3.sh` | Shell/Docker |
| **ORB-SLAM3 config** | `docker/orbslam3_config/` | YAML/Python |

---

## 7. Common Tasks

### Build Only Your Team's Package

```bash
./scripts/build.sh slam         # SLAM team
./scripts/build.sh state_est    # State Estimation team
./scripts/build.sh locomotion   # Locomotion team
./scripts/build.sh interfaces   # Shared interfaces
./scripts/build.sh sim          # Simulation layer
./scripts/build.sh              # Everything
```

### View Camera Images

```bash
ros2 run rqt_image_view rqt_image_view
# Select /head_camera/color from the dropdown
```

### Send a Test Joint Command

```bash
ros2 topic pub --once /g1_position_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['left_hip_pitch_joint', 'right_hip_pitch_joint', 'left_knee_joint', 'right_knee_joint'], \
    points: [{positions: [-0.2, -0.2, 0.4, 0.4], time_from_start: {sec: 1}}]}"
```

### Record and Replay Data

```bash
ros2 bag record -a -o my_recording
ros2 bag play my_recording
```

---

## 8. Troubleshooting

### Docker: MuJoCo window doesn't appear

Use headless mode — it works on any OS without display configuration:
```bash
./docker/run.sh ros2 launch unitree_ros2_control unitree_g1.launch.py headless:=true
```

If you want the GUI (Linux/WSL2 only):
```bash
# Linux — allow Docker to use your X server:
xhost +local:docker
./docker/run.sh ros2 launch unitree_ros2_control unitree_g1.launch.py

# WSL2 — make sure DISPLAY is set:
export DISPLAY=:0
./docker/run.sh ros2 launch unitree_ros2_control unitree_g1.launch.py
```

### Docker: "container name already in use"

The `run.sh` script now auto-detects running containers and `exec`s into them. If you hit this error with an old version of the script, either:
```bash
docker rm -f humanoid_ws_dev   # remove the stale container
# or use docker exec directly:
docker exec -it humanoid_ws_dev bash
```

### Docker: "permission denied" on docker commands

```bash
sudo usermod -aG docker $USER
# Log out and back in
```

### Native: "Package not found" errors

```bash
source ~/humanoid_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Native: MuJoCo window doesn't open

```bash
echo $DISPLAY        # Should be :0 for WSL2
echo $MUJOCO_DIR     # Should point to MuJoCo install

# For WSL2:
export DISPLAY=:0
```

### Native: "Could not find MUJOCO" during build

```bash
export MUJOCO_DIR="$HOME/.mujoco/mujoco-3.2.7"
```

### Native: `libmujoco.so` not found at runtime

```bash
export LD_LIBRARY_PATH=$MUJOCO_DIR/lib:$LD_LIBRARY_PATH
```

### Controllers fail to load

The controllers load 3 seconds after the simulation starts. If you see errors:
```bash
ros2 control list_controllers

# Manually load if needed:
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active g1_position_trajectory_controller
```

### Robot collapses immediately

Expected if no controller is sending commands. Run a locomotion controller:
```bash
ros2 run locomotion_pkg locomotion_node --ros-args -p mode:=standing
```

### Headless visualization with Foxglove (no GUI available)

If you can't get the MuJoCo GUI to display (e.g. remote server, no X11), you can use
[Foxglove](https://foxglove.dev/) to view camera feeds and topics in a web browser:
```bash
sudo apt install -y ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# Open https://app.foxglove.dev and connect to ws://localhost:8765
```

---

## 9. Background Concepts

### For Everyone

**ROS2 Concepts:**
- **Node**: A process that does one thing (e.g., read sensors, plan paths)
- **Topic**: A named channel for data (e.g., `/joint_states`)
- **Publisher/Subscriber**: Nodes publish to and subscribe from topics
- **Message**: The data format on a topic (e.g., `sensor_msgs/JointState`)
- **Launch file**: A Python script that starts multiple nodes together
- **Package**: A directory with code + `package.xml` — the unit of organization in ROS2
- **Workspace**: A directory containing multiple packages, built together with `colcon`

**Simulation Stack:**
- **URDF**: Robot description for ROS (visualization, TF tree)
- **MJCF**: Robot description for MuJoCo (physics simulation)
- **ros2_control**: Framework for controlling robot hardware (or simulated hardware)
- **Controller Manager**: Loads and manages controllers that read/write hardware interfaces
- **Hardware Interface**: Plugin that bridges ros2_control to actual hardware (or MuJoCo)

### For SLAM Team
- **SLAM** = Simultaneous Localization and Mapping: building a map while figuring out where you are in it
- **Visual Odometry**: Estimating camera motion from image sequences
- **Occupancy Grid**: 2D grid where each cell = "occupied", "free", or "unknown"
- Camera topics are `/head_camera/color`, `/head_camera/depth`, `/head_camera/camera_info`

### For State Estimation Team
- **EKF** (Extended Kalman Filter): The workhorse algorithm — predicts state forward, then corrects with measurements
- **IMU**: Inertial Measurement Unit — measures angular velocity (gyro) and linear acceleration (accelerometer)
- **Forward Kinematics**: Joint angles → where each link is in 3D space

### For Locomotion Team
- **RL** (Reinforcement Learning): Train a policy by trial-and-error in simulation
- **PPO**: Proximal Policy Optimization — popular RL algorithm for locomotion
- **Gait cycle**: The repeating pattern of leg movements (stance → swing → stance)

---

## Next Steps

1. **Read ARCHITECTURE.md** — understand the full system design and your team's role
2. **Launch the sim** — see the robot in MuJoCo
3. **Echo some topics** — understand the data flowing through the system
4. **Look at your team's stub code** — see the TODO comments for what to implement
5. **Start with Phase 1** — see the timeline in ARCHITECTURE.md
