# Architecture Guide — Humanoid Robotics Stack

## End Goal

Get the Unitree G1 humanoid robot to **walk a commanded distance** (e.g., "walk 10m forward") in MuJoCo simulation, with three subsystems working together:

1. **SLAM** — Camera images → map + robot pose → path plan → velocity command
2. **State Estimation** — IMU + joint encoders + SLAM pose → full body state
3. **Locomotion** — Body state + velocity command → joint-level walking commands

---

## Repository Structure

```
humanoid_ws/
├── ARCHITECTURE.md            ← You are here
├── STARTUP_GUIDE.md           ← Setup instructions
├── .gitignore
├── Dockerfile                 ← Main project Docker image (MuJoCo + ROS2 + all packages)
├── docker/
│   ├── build.sh               ← Build main Docker image
│   ├── run.sh                 ← Run/exec into main container
│   ├── Dockerfile.orbslam3    ← ORB-SLAM3 Docker image (separate container)
│   ├── run_orbslam3.sh        ← Run/exec into ORB-SLAM3 container
│   └── orbslam3_config/       ← G1 camera config + launch files for ORB-SLAM3
├── scripts/
│   ├── setup_environment.sh   ← One-time dependency install (native only)
│   ├── build.sh               ← Build workspace (all or per-team)
│   └── verify_setup.sh        ← Check everything is installed
│
└── src/
    ├── g1_description/            ← [Shared] Unitree G1 URDF/MJCF models + meshes
    ├── mujoco_ros2_control/       ← [Shared] MuJoCo ↔ ROS2 bridge (C++)
    ├── unitree_ros2_control/      ← [Shared] G1-specific launch + controller config
    │
    ├── interfaces/
    │   └── humanoid_interfaces/   ← [Shared] Message/action definitions (THE CONTRACTS)
    │
    ├── slam/
    │   └── slam_pkg/              ← [SLAM Team] Visual SLAM + path planning
    │
    ├── state_estimation/
    │   └── state_estimation_pkg/  ← [State Est. Team] EKF sensor fusion
    │
    ├── locomotion/
    │   └── locomotion_pkg/        ← [Locomotion Team] RL walking policy
    │
    └── integration/
        ├── launch/                   ← [All Teams] Full-stack launch files
        └── motion_primitives_pkg/    ← [All Teams] Scripted motions for sensor testing
```

---

## How the Three Teams Work in Parallel

### The Key Idea: Shared Interfaces, Independent Implementation

Each team's package communicates **only through ROS2 topics** defined in `humanoid_interfaces/`. As long as each team publishes and subscribes to the agreed-upon message types, they can develop completely independently.

### Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        MuJoCo Simulation                            │
│  (mujoco_ros2_control + g1_description + unitree_ros2_control)     │
│                                                                     │
│  Publishes:                                                         │
│    /joint_states          (sensor_msgs/JointState)                  │
│    /head_camera/color      (sensor_msgs/Image)                     │
│    /head_camera/depth      (sensor_msgs/Image)                     │
│    /imu/data              (sensor_msgs/Imu)                        │
│    /contact/left_foot     (contact force)                          │
│    /contact/right_foot    (contact force)                          │
│    /tf, /tf_static        (tf2)                                     │
│    /clock                 (rosgraph_msgs/Clock)                     │
│                                                                     │
│  Subscribes:                                                        │
│    /g1_position_trajectory_controller/joint_trajectory              │
└──────────────┬──────────────────────────────┬───────────────────────┘
               │                              │
               ▼                              ▼
┌──────────────────────┐        ┌──────────────────────────┐
│     SLAM Team        │        │  State Estimation Team   │
│                      │        │                          │
│ IN:                  │        │ IN:                      │
│   camera images      │        │   /joint_states          │
│   depth images       │        │   /imu/data              │
│                      │        │   /slam/state (optional) │
│ OUT:                 │        │                          │
│   /slam/state        │───────→│ OUT:                     │
│   /slam/map          │        │   /state_estimation/     │
│   /cmd_vel           │        │     robot_state          │
│   tf: map → odom     │        │   /state_estimation/     │
│                      │        │     contact              │
└──────────────────────┘        │   tf: odom → base_link   │
         │                      └────────────┬─────────────┘
         │                                   │
         │         ┌─────────────────────────┐
         │         ▼                         │
         │  ┌──────────────────────┐         │
         └─→│  Locomotion Team     │←────────┘
            │                      │
            │ IN:                  │
            │   /state_estimation/ │
            │     robot_state      │
            │   /cmd_vel           │
            │                      │
            │ OUT:                 │
            │   JointTrajectory    │──→ MuJoCo (closes the loop)
            └──────────────────────┘
```

### What Each Team Can Do Independently

| Team | Can develop without | Needs from simulation |
|------|--------------------|-----------------------|
| **SLAM** | State est. and locomotion | Camera images (available via head camera) |
| **State Estimation** | SLAM and locomotion | Joint states + IMU + foot contacts (all available) |
| **Locomotion** | SLAM and state estimation | Joint states + foot contacts (all available) |

### Mock/Stub Strategy for Independent Development

- **SLAM team** can test with recorded camera data or static images
- **State Estimation team** can test immediately — joint states are already published by the existing sim
- **Locomotion team** can skip state estimation initially and read `/joint_states` directly. A standing controller stub is already provided.

---

## The Shared Interfaces (humanoid_interfaces)

These are the **contracts** between teams. Changing them requires coordination.

| Message | Publisher | Subscriber | Purpose |
|---------|-----------|------------|---------|
| `SlamState` | SLAM | State Est, Integration | Map + robot pose from SLAM |
| `NavigationGoal` | User/Planner | SLAM | "Walk 10m forward" |
| `RobotState` | State Est | Locomotion | Full body state for RL policy |
| `ContactState` | State Est | Locomotion | Foot contact info |
| `LocomotionCommand` | Locomotion | Integration | Joint-level commands |
| `Navigate.action` | Integration | All | End-to-end navigation action |

### Rules for Interface Changes
1. Propose the change in a GitHub Issue
2. All affected teams review and approve
3. Update the .msg/.action file on `main`
4. All teams pull and rebuild `humanoid_interfaces`

---

## Existing Simulation Layer (What You Get for Free)

### g1_description
- **26 URDF variants** of the Unitree G1 humanoid (23-DOF and 29-DOF configs)
- **MJCF models** (.xml) for MuJoCo physics simulation
- **Mesh files** (.STL) for visualization
- Currently using: `g1_23dof_rev_1_0` (23 degrees of freedom)

### mujoco_ros2_control
- **MuJoCo ↔ ROS2 bridge**: Runs the physics sim and exposes joints via `ros2_control`
- **MujocoSystem plugin**: Hardware interface that reads joint states from MuJoCo and writes commands back
- **Camera support**: Renders camera images via OpenGL and publishes as ROS2 Image messages (~6 Hz)
- **Sensor support**: IMU and force/torque sensors
- **Control modes**: Position, velocity, and effort (torque) control with optional PID
- **Headless mode**: Supports `headless:=true` launch argument for running inside Docker without a display (uses OSMesa software rendering)

### unitree_ros2_control
- **Launch file**: Starts MuJoCo sim + robot state publisher + controllers
- **Controller config**: 23 joints configured for position trajectory control at 100 Hz
- **Joint order** (important for locomotion team):
  ```
  left_hip_pitch, left_hip_roll, left_hip_yaw, left_knee,
  left_ankle_pitch, left_ankle_roll,
  right_hip_pitch, right_hip_roll, right_hip_yaw, right_knee,
  right_ankle_pitch, right_ankle_roll,
  waist_yaw,
  left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw,
  left_elbow, left_wrist_roll,
  right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw,
  right_elbow, right_wrist_roll
  ```

---

## Built-in Simulation Sensors

The MuJoCo simulation provides the following sensors out of the box. No additional configuration is needed — launch the simulation and subscribe.

### Head Camera

A camera is mounted on the torso body in the MJCF model (`g1_23dof_rev_1_0.xml`), providing both color and depth images.

| Topic | Type | Rate | Used by |
|-------|------|------|---------|
| `/head_camera/color` | `sensor_msgs/Image` | ~6 Hz | SLAM (visual odometry, mapping) |
| `/head_camera/depth` | `sensor_msgs/Image` | ~6 Hz | SLAM (depth-based obstacle detection, occupancy grid) |
| `/head_camera/camera_info` | `sensor_msgs/CameraInfo` | ~6 Hz | SLAM (camera intrinsics) |

### IMU

An IMU sensor suite is attached to the pelvis, providing orientation, angular velocity, and linear acceleration.

| Topic | Type | Rate | Used by |
|-------|------|------|---------|
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | State Estimation (EKF prediction, base orientation tracking) |

The MJCF model defines `framequat` (orientation), `gyro` (angular velocity), and `accelerometer` (linear acceleration) sensors on the pelvis body.

### Foot Contact Sensors

Touch sensors on both feet detect ground contact forces.

| Topic | Type | Rate | Used by |
|-------|------|------|---------|
| `/contact/left_foot` | Contact force | 100 Hz | State Estimation + Locomotion (gait phase, stance/swing detection) |
| `/contact/right_foot` | Contact force | 100 Hz | State Estimation + Locomotion (gait phase, stance/swing detection) |

---

## Docker Architecture

The entire stack runs inside Docker containers — no native installs needed. Works on Linux, macOS, and Windows.

```
┌─────────────────────────────────────────────────────────────┐
│                   Host Machine (any OS)                      │
│                                                             │
│  ┌─────────────────────────────┐  ┌───────────────────────┐ │
│  │  humanoid_ws container      │  │  orbslam3 container   │ │
│  │  (./docker/run.sh)          │  │  (./docker/run_       │ │
│  │                             │  │   orbslam3.sh)        │ │
│  │  MuJoCo sim (headless)      │  │                       │ │
│  │  ros2_control bridge        │  │  ORB-SLAM3 C++ lib    │ │
│  │  Robot state publisher      │  │  ROS2 wrapper         │ │
│  │  SLAM/StateEst/Loco nodes   │  │                       │ │
│  │                             │  │  Subscribes:          │ │
│  │  Publishes:                 │  │  /head_camera/color   │ │
│  │  /joint_states              │  │  /head_camera/depth   │ │
│  │  /head_camera/color,depth   │──│                       │ │
│  │  /imu/data                  │  │  Publishes:           │ │
│  │  /contact/*                 │  │  /robot_pose_slam     │ │
│  │                             │  │  /map_points          │ │
│  └─────────────────────────────┘  │  TF: map→odom        │ │
│           ▲                       └───────────────────────┘ │
│           │  All containers share --network host            │
│           │  ROS2 DDS topics flow freely between them       │
│           ▼                                                 │
│  ┌─────────────────────────────┐                            │
│  │  Additional containers      │                            │
│  │  (team-specific tools,      │                            │
│  │   visualization, etc.)      │                            │
│  └─────────────────────────────┘                            │
└─────────────────────────────────────────────────────────────┘
```

Key points:
- **`--network host`** makes all containers share the host's network — ROS2 topics flow between containers automatically
- **Headless mode** (`headless:=true`) uses OSMesa software rendering — no display server or GPU needed
- The `run.sh` scripts auto-detect running containers and `docker exec` into them instead of starting duplicates

---

## Motion Primitives Package

**Purpose:** Generate deterministic robot movements so that all sensors produce useful data before locomotion controllers are implemented.

**Package location:** `src/integration/motion_primitives_pkg/`

### Available Modes

| Mode | Description |
|------|-------------|
| `standing` | Hold a stable standing pose |
| `squat` | Bend knees then return to standing (repeating cycle) |
| `shift_weight` | Sway hips left and right |
| `step_forward` | Small stepping motion alternating legs |
| `arm_swing` | Swing arms back and forth |

### Usage

```bash
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=squat
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=shift_weight
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=step_forward
ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=arm_swing
```

### Development Workflow

Teams can use motion primitives to generate sensor data while developing their own algorithms:

1. **Launch the simulation:** `ros2 launch unitree_ros2_control unitree_g1.launch.py`
2. **Start a motion primitive:** `ros2 run motion_primitives_pkg motion_primitives_node --ros-args -p mode:=squat`
3. **Subscribe to sensor topics** in your own nodes and develop against real, dynamic data

This means:
- **SLAM team** gets camera images with a moving viewpoint (robot moves → camera moves)
- **State Estimation team** gets IMU + contact data with real dynamics (not just a static pose)
- **Locomotion team** can study the joint trajectories and contact patterns as reference

---

## Team Development Guides

### SLAM Team

**Your job:** Take camera images from the simulated robot, build a map, localize the robot, and plan paths.

**Where your code lives:** `src/slam/slam_pkg/` (your Python nodes) + `docker/orbslam3_config/` (ORB-SLAM3 config)

**What's already set up:**
- Head camera is configured in `g1_23dof_rev_1_0.xml` (640x480, fovy=90°, ~6 Hz)
- Camera topics: `/head_camera/color`, `/head_camera/depth`, `/head_camera/camera_info`
- ORB-SLAM3 is Dockerized and configured for the G1 camera (see Section 4 of STARTUP_GUIDE.md)
- A proof-of-concept depth-based SLAM node exists in `slam_pkg/slam_node.py`

**Phase 1 — ORB-SLAM3 integration (Week 1-2)**
- Run ORB-SLAM3 via `docker/run_orbslam3.sh` (already configured)
- Verify pose output on `/robot_pose_slam` and point cloud on `/map_points`
- Write Python nodes in `slam_pkg` that subscribe to ORB-SLAM3's ROS2 outputs
- ORB-SLAM3 runs in C++ inside Docker; your team writes Python code consuming its output topics

**Phase 2 — Dense mapping + terrain reconstruction (Week 2-4)**
- Combine ORB-SLAM3 pose with depth images to build dense occupancy grids or 3D voxel maps
- Convert the map into MuJoCo MJCF format (heightfields or meshes) for terrain reconstruction
- Publish occupancy grid on `/slam/map`

**Phase 3 — Path planning (Week 4-6)**
- Implement A* or RRT path planning on the occupancy grid
- Publish `/cmd_vel` (desired robot velocity) for the locomotion team

**Phase 4 — Integration (Week 6-8)**
- Connect to state estimation for corrected pose
- Test full SLAM → navigation pipeline
- Ensure `map -> odom -> base_link` TF chain works with state estimation team

**Key files to study:**
- `mujoco_ros2_control/src/mujoco_cameras.cpp` — how cameras work in the sim
- `docker/orbslam3_config/g1_rgbd.yaml` — ORB-SLAM3 camera intrinsics for G1
- `docker/orbslam3_config/g1-rgbd-ros-params.yaml` — ORB-SLAM3 ROS2 topic mapping
- `g1_description/g1_23dof_rev_1_0.xml` — MJCF model with head_camera

---

### State Estimation Team

**Your job:** Fuse IMU, joint encoder, and SLAM data into a single, accurate `RobotState` message.

**Where your code lives:** `src/state_estimation/state_estimation_pkg/`

**Phase 1 — Pass-through (Week 1-2)**
- Subscribe to `/joint_states` and repackage as `RobotState`
- This immediately lets the locomotion team start working
- No filtering, no fusion — just data forwarding

**Phase 2 — IMU integration (Week 2-4)**
- IMU sensor is already configured in the MJCF model (framequat, gyro, accelerometer on pelvis)
- The MuJoCo system already reads and publishes IMU data on `/imu/data` — see `mujoco_system.cpp:register_sensors()`
- Implement IMU integration (dead reckoning) for base orientation and velocity

**Phase 3 — EKF fusion (Week 4-6)**
- Implement an Extended Kalman Filter:
  - **Prediction**: IMU integration (gyro → orientation, accel → velocity)
  - **Update**: Joint kinematics (forward kinematics for foot position) + SLAM pose
- State vector: `[position(3), velocity(3), orientation(4), gyro_bias(3), accel_bias(3)]`

**Phase 4 — Contact estimation + gait phase (Week 6-8)**
- Detect foot contacts from ankle torques or MuJoCo contact forces
- Estimate gait phase (0 = start of left swing, 0.5 = start of right swing)
- Publish `ContactState` for locomotion

**Key concepts to learn:**
- Extended Kalman Filter (EKF) for state estimation
- Quaternion-based orientation representation
- Forward kinematics (URDF → joint positions → foot positions)

**Key files to study:**
- `mujoco_ros2_control/src/mujoco_system.cpp` — how sensors are read
- `g1_description/g1_23dof_rev_1_0.urdf` — robot kinematics

---

### Locomotion Team

**Your job:** Make the robot walk. Take the body state and a velocity command, output joint positions that produce stable walking.

**Where your code lives:** `src/locomotion/locomotion_pkg/`

**Phase 1 — Standing controller (Week 1-2)**
- The stub `locomotion_node.py` already has a standing controller
- Run the simulation, verify the robot holds a standing pose
- Tune the standing pose angles so the robot doesn't fall over
- Understand the joint ordering and control interface

**Phase 2 — Sinusoidal gait / CPG (Week 2-4)**
- The stub already has a sinusoidal gait generator
- Tune the oscillation parameters (frequency, amplitude, phase offsets)
- Get the robot to take a few steps (it will likely fall — that's OK)
- Study the gait cycle: stance phase, swing phase, double support

**Phase 3 — RL policy training (Week 4-7)**
- Set up a standalone MuJoCo training environment (no ROS needed for training)
- Use PPO (Proximal Policy Optimization) or SAC
- Observation space: base orientation, angular velocity, joint positions, joint velocities, previous action, velocity command
- Action space: target joint positions (23-DOF)
- Reward function: forward velocity tracking + staying upright + energy penalty
- Frameworks: stable-baselines3, IsaacGym, MuJoCo MJX + JAX

**Phase 4 — Deploy trained policy in ROS (Week 7-8)**
- Export trained model to ONNX or PyTorch
- Load in `locomotion_node.py` and run inference at 50 Hz
- Test with velocity commands from SLAM's path planner

**Key concepts to learn:**
- Reinforcement Learning (PPO algorithm)
- Sim-to-sim transfer
- Central Pattern Generators (CPGs) for locomotion
- Humanoid gait cycle phases

**Key files to study:**
- `locomotion_pkg/locomotion_node.py` — your main node (standing + sinusoidal stubs)
- `unitree_ros2_control/config/g1_position_controllers.yaml` — joint ordering
- `g1_description/g1_23dof_rev_1_0.xml` — physics parameters (joint limits, damping, etc.)

---

## 2-Month Timeline

### Week 1-2: Foundation
| Team | Deliverable |
|------|-------------|
| **All** | Everyone completes STARTUP_GUIDE, can launch MuJoCo sim, sees robot in visualization |
| **SLAM** | Camera added to MJCF, camera images displaying in RViz/OpenCV |
| **State Est.** | Pass-through RobotState published from /joint_states |
| **Locomotion** | Standing controller working, robot holds pose in MuJoCo |

**Integration checkpoint:** Locomotion reads State Est output → robot stands.

### Week 3-4: Core Algorithms
| Team | Deliverable |
|------|-------------|
| **SLAM** | Basic visual odometry working, pose estimates published |
| **State Est.** | IMU sensor added to MJCF, IMU integration for base orientation |
| **Locomotion** | Sinusoidal gait producing steps (may be unstable), RL training environment set up |

**Integration checkpoint:** Robot takes wobbly steps using sinusoidal gait.

### Week 5-6: Refinement
| Team | Deliverable |
|------|-------------|
| **SLAM** | Occupancy grid mapping, A* path planning, /cmd_vel published |
| **State Est.** | EKF fusing IMU + kinematics, accurate base pose estimate |
| **Locomotion** | RL policy trained, walking stably in standalone MuJoCo |

**Integration checkpoint:** RL policy deployed in ROS, stable walking with manual /cmd_vel.

### Week 7-8: Integration & Demo
| Team | Deliverable |
|------|-------------|
| **SLAM** | Full SLAM → path plan → /cmd_vel pipeline working |
| **State Est.** | SLAM pose incorporated into EKF, contact estimation working |
| **Locomotion** | Policy following /cmd_vel commands, walking to waypoints |

**Final demo:** Command "walk 10m forward" → SLAM plans path → State Est tracks robot → Locomotion walks the robot → arrives at target.

---

## Git Workflow

### Branch Strategy
```
main                    ← stable, always builds, integration tested
├── slam/feature-name   ← SLAM team branches
├── state-est/feature   ← State Estimation team branches
├── loco/feature        ← Locomotion team branches
└── integration/test    ← integration testing branches
```

### Rules
1. **Never push directly to `main`** — always use Pull Requests
2. **Branch naming**: `<team>/<short-description>` (e.g., `slam/add-camera-sensor`)
3. **PR reviews**: At least one team member reviews before merge
4. **Interface changes** (humanoid_interfaces): Require approval from all affected teams
5. **Build before merging**: `colcon build` must succeed on the PR branch

### Typical Workflow
```bash
# Start a new feature
git checkout main
git pull
git checkout -b slam/add-orb-features

# Work on your feature
# ... edit code ...
colcon build --packages-select slam_pkg
source install/setup.bash
# ... test ...

# Push and create PR
git add -A
git commit -m "Add ORB feature extraction to SLAM node"
git push -u origin slam/add-orb-features
# Create PR on GitHub
```

---

## ROS2 Topic Reference (Quick Reference)

| Topic | Type | Publisher | Rate |
|-------|------|-----------|------|
| `/joint_states` | JointState | joint_state_broadcaster | 100 Hz |
| `/clock` | Clock | mujoco_ros2_control | sim rate |
| `/tf` | TFMessage | robot_state_publisher | 100 Hz |
| `/head_camera/color` | Image | mujoco_cameras | ~6 Hz |
| `/head_camera/depth` | Image | mujoco_cameras | ~6 Hz |
| `/head_camera/camera_info` | CameraInfo | mujoco_cameras | ~6 Hz |
| `/imu/data` | Imu | mujoco_system | 100 Hz |
| `/contact/left_foot` | Contact force | mujoco_system | 100 Hz |
| `/contact/right_foot` | Contact force | mujoco_system | 100 Hz |
| `/slam/state` | SlamState | slam_node | 10 Hz |
| `/slam/map` | OccupancyGrid | slam_node | 1 Hz |
| `/cmd_vel` | Twist | path_planner_node | 10 Hz |
| `/state_estimation/robot_state` | RobotState | state_estimator_node | 100 Hz |
| `/state_estimation/contact` | ContactState | contact_estimator_node | 100 Hz |
| `/g1_position_trajectory_controller/joint_trajectory` | JointTrajectory | locomotion_node / motion_primitives_node | 50 Hz |

---

## Useful Commands

```bash
# Launch just the simulation
ros2 launch unitree_ros2_control unitree_g1.launch.py

# See what topics are active
ros2 topic list

# Echo a topic
ros2 topic echo /joint_states

# Check controller status
ros2 control list_controllers

# Visualize in RViz
rviz2 -d src/unitree_ros2_control/unitree_ros2_control/rviz/g1.rviz

# Build only your team's package
./scripts/build.sh slam        # or state_est, locomotion

# Launch full stack
ros2 launch humanoid_integration full_stack.launch.py
```
