# Startup Guide — Humanoid Robotics Workspace

This guide gets you from a fresh Ubuntu machine to running the Unitree G1 humanoid robot in MuJoCo simulation with ROS2. No prior ROS or robotics experience is assumed.

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Install ROS2 Humble](#2-install-ros2-humble)
3. [Install MuJoCo](#3-install-mujoco)
4. [Clone and Build the Workspace](#4-clone-and-build-the-workspace)
5. [Run the Simulation](#5-run-the-simulation)
6. [Understand What's Running](#6-understand-whats-running)
7. [Explore the Codebase](#7-explore-the-codebase)
8. [Common Tasks](#8-common-tasks)
9. [Troubleshooting](#9-troubleshooting)
10. [Background Concepts](#10-background-concepts)

---

## 1. Prerequisites

- **OS:** Ubuntu 22.04 LTS (native or WSL2)
- **RAM:** 8 GB minimum (16 GB recommended)
- **Disk:** ~5 GB free space
- **GPU:** Not required for basic sim, helpful for RL training later

If you're on Windows, use WSL2:
```bash
wsl --install -d Ubuntu-22.04
```

For WSL2 users: MuJoCo rendering requires an X server. Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or use WSLg (Windows 11 has this built-in).

---

## 2. Install ROS2 Humble

ROS2 (Robot Operating System 2) is the middleware that connects all our components. "Humble" is the LTS release for Ubuntu 22.04.

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

**Verify ROS2 works:**
```bash
ros2 topic list
# Should print: /parameter_events and /rosout
```

### What is ROS2? (30-second version)
ROS2 is a framework where independent programs ("nodes") communicate by publishing and subscribing to "topics." Think of topics as named channels:
- Node A publishes joint positions to `/joint_states`
- Node B subscribes to `/joint_states` and uses the data
- Nodes can run in any language, on any machine, and are completely decoupled

---

## 3. Install MuJoCo

MuJoCo is the physics simulator. It simulates the robot's body, joints, gravity, and contact forces.

```bash
# Download MuJoCo 3.2.7
mkdir -p ~/.mujoco
cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz
tar -xzf mujoco-3.2.7-linux-x86_64.tar.gz
rm mujoco-3.2.7-linux-x86_64.tar.gz

# Set environment variable
echo 'export MUJOCO_DIR="$HOME/.mujoco/mujoco-3.2.7"' >> ~/.bashrc
source ~/.bashrc

# Install rendering dependencies
sudo apt install -y libglfw3-dev libeigen3-dev
```

**Verify MuJoCo works:**
```bash
$MUJOCO_DIR/bin/simulate $MUJOCO_DIR/model/humanoid/humanoid.xml
# Should open a window with a humanoid model. Close it with Ctrl+C.
```

### What is MuJoCo? (30-second version)
MuJoCo ("Multi-Joint dynamics with Contact") is a physics engine optimized for robotics. It takes a model of the robot (joints, links, mass, etc.) and simulates what happens when you apply forces/torques. Our robot model is defined in two formats:
- **URDF** — standard ROS robot description (used by ROS2 tools)
- **MJCF** (.xml) — MuJoCo's native format (used by the physics sim)

---

## 4. Clone and Build the Workspace

```bash
# Clone the repository
cd ~
git clone <YOUR_GITHUB_URL> humanoid_ws
cd humanoid_ws

# Install all ROS2 dependencies
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-effort-controllers \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    ros-humble-controller-manager

# OR use the automated setup script:
# ./scripts/setup_environment.sh

# Build the workspace
colcon build --symlink-install
source install/setup.bash

# Add workspace to .bashrc so it's sourced automatically
echo "source ~/humanoid_ws/install/setup.bash" >> ~/.bashrc
```

**Verify the build:**
```bash
./scripts/verify_setup.sh
# Should show all checks passing
```

### What is `colcon build`?
`colcon` is the ROS2 build tool. It finds all packages in `src/`, resolves dependencies, and compiles them. After building:
- Compiled binaries go to `build/`
- Installed files go to `install/`
- Build logs go to `log/`
- `--symlink-install` makes Python files use symlinks so edits take effect without rebuilding

---

## 5. Run the Simulation

### Launch the G1 Robot in MuJoCo

```bash
# Terminal 1: Launch the simulation
ros2 launch unitree_ros2_control unitree_g1.launch.py
```

You should see:
1. A MuJoCo window opens showing the G1 humanoid robot
2. The robot stands on a ground plane (it will likely collapse since no controller is sending commands yet)
3. Terminal output shows controllers being loaded

### Interact with the Simulation

Open new terminals (each needs `source ~/humanoid_ws/install/setup.bash`):

```bash
# Terminal 2: See all active topics
ros2 topic list

# Terminal 3: Watch joint states in real-time
ros2 topic echo /joint_states

# Terminal 4: Check which controllers are loaded
ros2 control list_controllers

# Terminal 5: Launch RViz to visualize the robot model
rviz2 -d src/unitree_ros2_control/unitree_ros2_control/rviz/g1.rviz
```

### Try the Standing Controller

Once the base sim is working, test the locomotion stub:

```bash
# Terminal 2: Launch the locomotion node in standing mode
ros2 run locomotion_pkg locomotion_node --ros-args -p mode:=standing
```

This sends a fixed standing pose to the robot's joints.

### Try the Sinusoidal Gait

```bash
ros2 run locomotion_pkg locomotion_node --ros-args -p mode:=sinusoidal
```

The robot will attempt to walk with sine-wave joint trajectories. It will probably fall — that's expected and normal! The locomotion team's job is to make this work properly.

### Try the Motion Primitives

Motion primitives generate simple, deterministic robot movements so that all sensors (camera, IMU, foot contacts) produce useful data — even before real locomotion policies exist.

```bash
# Terminal 2: Run a squat motion
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=squat

# Or try other modes:
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=shift_weight
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=step_forward
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=arm_swing
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=standing
```

Available modes: `standing`, `squat`, `shift_weight`, `step_forward`, `arm_swing`.

### Verify Sensors Are Working

Once the simulation is running (with or without a motion primitive), check that all sensors are publishing:

```bash
# Camera images (SLAM team)
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw

# IMU data (State Estimation team)
ros2 topic echo /imu/data

# Foot contact sensors (State Est + Locomotion teams)
ros2 topic echo /contact/left_foot
ros2 topic echo /contact/right_foot
```

### Development Workflow with Motion Primitives

This is the recommended workflow for all teams during early development:

1. **Launch the simulation:** `ros2 launch unitree_ros2_control unitree_g1.launch.py`
2. **Start a motion primitive** to get the robot moving (e.g., `mode:=squat`)
3. **Subscribe to sensor topics** in your own nodes — you now have real, dynamic sensor data
4. **Develop your algorithms** against this data

This approach lets every team start development immediately without waiting for other teams to finish their work.

---

## 6. Understand What's Running

When you launch `unitree_g1.launch.py`, here's what happens:

```
unitree_g1.launch.py
│
├── mujoco_ros2_control (C++ node)
│   ├── Loads g1_23dof_rev_1_0.xml into MuJoCo physics engine
│   ├── Loads g1_23dof_rev_1_0.urdf for joint/link info
│   ├── Creates MujocoSystem hardware interface plugin
│   ├── Opens rendering window (GLFW)
│   ├── Runs physics at 1000 Hz (timestep = 0.001s)
│   ├── Runs control loop at 100 Hz
│   ├── Publishes /clock for simulation time
│   ├── Publishes /camera/color/image_raw and /camera/depth/image_raw
│   ├── Publishes /imu/data (orientation, gyro, accelerometer)
│   └── Publishes /contact/left_foot and /contact/right_foot
│
├── robot_state_publisher (standard ROS2 node)
│   ├── Reads URDF
│   ├── Publishes /tf and /tf_static transforms
│   └── Publishes /robot_description
│
├── joint_state_broadcaster (ros2_control controller)
│   ├── Reads joint positions/velocities from hardware interface
│   └── Publishes /joint_states at 100 Hz
│
└── g1_position_trajectory_controller (ros2_control controller)
    ├── Subscribes to joint trajectory commands
    └── Sends position commands to hardware interface → MuJoCo
```

### The Control Loop (How Commands Reach the Robot)

```
Your code publishes JointTrajectory
    → g1_position_trajectory_controller receives it
    → controller_manager calls write()
    → MujocoSystem::write() sets mj_data->qpos[] in MuJoCo
    → MuJoCo physics engine steps forward
    → MujocoSystem::read() reads new joint positions
    → joint_state_broadcaster publishes /joint_states
    → Your code reads the new state
    → (loop)
```

---

## 7. Explore the Codebase

### Where Things Are

| What | Where | Language |
|------|-------|----------|
| Robot 3D model (physics) | `src/g1_description/g1_23dof_rev_1_0.xml` | MJCF/XML |
| Robot description (ROS) | `src/g1_description/g1_23dof_rev_1_0.urdf` | URDF/XML |
| Robot meshes | `src/g1_description/meshes/` | STL |
| MuJoCo↔ROS bridge | `src/mujoco_ros2_control/mujoco_ros2_control/src/` | C++ |
| Simulation launch | `src/unitree_ros2_control/.../launch/unitree_g1.launch.py` | Python |
| Controller config | `src/unitree_ros2_control/.../config/g1_position_controllers.yaml` | YAML |
| Team interfaces | `src/interfaces/humanoid_interfaces/msg/` | ROS2 IDL |
| SLAM code | `src/slam/slam_pkg/slam_pkg/` | Python |
| State estimation code | `src/state_estimation/state_estimation_pkg/` | Python |
| Locomotion code | `src/locomotion/locomotion_pkg/locomotion_pkg/` | Python |
| Integration launch | `src/integration/launch/full_stack.launch.py` | Python |
| Motion primitives | `src/integration/motion_primitives_pkg/` | Python |

### The G1 Robot

The Unitree G1 is a humanoid robot with 23 controllable joints in our configuration:
- **12 leg joints** (6 per leg): hip pitch/roll/yaw, knee, ankle pitch/roll
- **1 waist joint**: waist yaw
- **10 arm joints** (5 per arm): shoulder pitch/roll/yaw, elbow, wrist roll

The MJCF file (`g1_23dof_rev_1_0.xml`) defines the physics:
- Mass and inertia of each body part
- Joint limits and damping
- Contact properties (friction)
- Actuator force limits

### Understanding the URDF

The URDF (Unified Robot Description Format) is an XML file that describes:
```xml
<robot>
  <link name="pelvis">          <!-- A rigid body -->
    <visual>...</visual>        <!-- How it looks -->
    <collision>...</collision>  <!-- Collision shape -->
    <inertial>...</inertial>    <!-- Mass properties -->
  </link>
  <joint name="left_hip_pitch_joint" type="revolute">
    <parent link="pelvis"/>     <!-- Connected to... -->
    <child link="left_hip_pitch_link"/>
    <limit lower="-2.53" upper="2.88" effort="88"/>
  </joint>
</robot>
```

### Understanding the MJCF

MuJoCo's model format defines the physics simulation:
```xml
<mujoco>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pelvis" pos="0 0 0.793">
      <joint name="floating_base_joint" type="free"/>  <!-- 6-DOF floating base -->
      <geom type="mesh" mesh="pelvis"/>
      <body name="left_hip_pitch_link">
        <joint name="left_hip_pitch_joint" axis="0 1 0"
               range="-2.53 2.88" actuatorfrcrange="-88 88"/>
      </body>
    </body>
  </worldbody>
</mujoco>
```

---

## 8. Common Tasks

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
# In a separate terminal
ros2 run rqt_image_view rqt_image_view
# Select /camera/color/image_raw from the dropdown
```

### Send a Test Joint Command

```bash
# Send the robot to a pose via command line
ros2 topic pub --once /g1_position_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['left_hip_pitch_joint', 'right_hip_pitch_joint', 'left_knee_joint', 'right_knee_joint'], \
    points: [{positions: [-0.2, -0.2, 0.4, 0.4], time_from_start: {sec: 1}}]}"
```

### Record and Replay Data

```bash
# Record all topics
ros2 bag record -a -o my_recording

# Replay
ros2 bag play my_recording
```

### Check Simulation Time

```bash
ros2 topic echo /clock
```

---

## 9. Troubleshooting

### "Package not found" errors

```bash
# Make sure you've sourced the workspace
source ~/humanoid_ws/install/setup.bash

# Rebuild
colcon build --symlink-install
source install/setup.bash
```

### MuJoCo window doesn't open

```bash
# Check if DISPLAY is set (needed for rendering)
echo $DISPLAY

# For WSL2, set it:
export DISPLAY=:0

# Or if using WSLg:
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
```

### "Could not find MUJOCO" during build

```bash
# Make sure MUJOCO_DIR is set
echo $MUJOCO_DIR
# Should print something like /home/yourname/.mujoco/mujoco-3.2.7

# If empty:
export MUJOCO_DIR="$HOME/.mujoco/mujoco-3.2.7"
```

### Controllers fail to load

The controllers load 3 seconds after the simulation starts. If you see errors:
```bash
# Check if the controller manager is running
ros2 control list_controllers

# Manually load controllers
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active g1_position_trajectory_controller
```

### Robot collapses immediately

This is expected if no controller is sending commands. The physics simulation applies gravity and the robot falls. Launch a locomotion controller to hold the robot up:
```bash
ros2 run locomotion_pkg locomotion_node --ros-args -p mode:=standing
```

### Build errors in mujoco_ros2_control

```bash
# Make sure all dependencies are installed
sudo apt install -y libglfw3-dev libeigen3-dev
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-controller-manager ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller ros-humble-effort-controllers

# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## 10. Background Concepts

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
- **A\***: Classic shortest-path algorithm on a grid

### For State Estimation Team
- **State Estimation**: Computing the robot's full state (pose, velocity) from noisy sensor data
- **EKF** (Extended Kalman Filter): The workhorse algorithm — predicts state forward, then corrects with measurements
- **IMU**: Inertial Measurement Unit — measures angular velocity (gyro) and linear acceleration (accelerometer)
- **Forward Kinematics**: Joint angles → where each link is in 3D space

### For Locomotion Team
- **RL** (Reinforcement Learning): Train a policy by trial-and-error in simulation
- **PPO**: Proximal Policy Optimization — one of the most popular RL algorithm for locomotion
- **Observation space**: What the policy sees (joint angles, velocities, base orientation, etc.)
- **Action space**: What the policy outputs (target joint positions)
- **Reward function**: Scalar signal that tells the policy how well it's doing
- **Gait cycle**: The repeating pattern of leg movements (stance → swing → stance)
- **CPG**: Central Pattern Generator — produces rhythmic patterns for walking without RL

---

## Next Steps

1. **Read ARCHITECTURE.md** — understand the full system design and your team's role
2. **Launch the sim** — see the robot in MuJoCo
3. **Echo some topics** — understand the data flowing through the system
4. **Look at your team's stub code** — see the TODO comments for what to implement
5. **Start with Phase 1** — see the timeline in ARCHITECTURE.md
