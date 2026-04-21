# Autonomous Maze Solver — TurtleBot3 (ROS 2 Jazzy / Gazebo Harmonic)

> **Project Repository:** `maze-solver-with-py`  
> **Platform:** ROS 2 Jazzy Jalisco · Gazebo Harmonic · Ubuntu 24.04  
> **Robot:** TurtleBot3 Waffle  
> **Author:** RO-03  

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Hardware & Software Components](#3-hardware--software-components)
4. [Package Structure](#4-package-structure)
5. [How It Works](#5-how-it-works)
6. [Control Flow (End-to-End)](#6-control-flow-end-to-end)
7. [Key Configuration Parameters](#7-key-configuration-parameters)
8. [Multi-Route Operation](#8-multi-route-operation)
9. [ROS 2 Humble → Jazzy Migration](#9-ros-2-humble--jazzy-migration)
10. [Known Limitations & Future Work](#10-known-limitations--future-work)
11. [Quick-Start Commands](#11-quick-start-commands)

---

## 1. Project Overview

The **Autonomous Maze Solver** is a fully autonomous mobile-robot navigation system that plans and executes collision-free paths through a pre-mapped maze environment — entirely without human intervention during navigation.

| Property | Value |
|----------|-------|
| Navigation mode | Map-based (pre-built occupancy grid) |
| Localisation | Particle-filter AMCL |
| Path planning | NavFn (Dijkstra global) + DWB (local) |
| Waypoint execution | `nav2_simple_commander.BasicNavigator` |
| Simulation | Gazebo Harmonic (physics-true, real-time) |
| ROS distribution | **ROS 2 Jazzy Jalisco** |

The robot spawns at a fixed position inside the maze, localises itself against a known map, receives a sequence of (x, y) goal poses, and drives to each one following a dynamically re-planned path while avoiding walls.

---

## 2. System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                          Gazebo Harmonic                             │
│  ┌──────────┐  /scan (LaserScan)   ┌──────────────────────────────┐ │
│  │ GPU LiDAR│─────────────────────▶│                              │ │
│  │ Sensor   │                      │   ros_gz_bridge              │ │
│  ├──────────┤  /odom (Odometry)    │   (parameter_bridge)         │ │
│  │ DiffDrive│─────────────────────▶│                              │ │
│  │ Plugin   │◀── /cmd_vel (Twist) ─│                              │ │
│  └──────────┘  /tf  (odom→base)    └──────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
              ▲                   │
              │ TwistStamped      │ LaserScan / Odometry / TF / Clock
              │                  ▼
┌──────────────────────────────────────────────────────────────────────┐
│                           ROS 2 Graph                                │
│                                                                      │
│  map_server ──/map──▶ amcl ──/tf (map→odom)──▶ Nav2 Costmaps        │
│                                                                      │
│  amcl_initializer ──────────▶ /initialpose + /tf (map→odom seed)    │
│                                                                      │
│  maze_solver ─▶ BasicNavigator ─▶ bt_navigator ─▶ planner_server    │
│                                         │                            │
│                                         ▼                            │
│                               controller_server (DWB)               │
│                                    │                                 │
│                                    ▼                                 │
│                          velocity_smoother ──▶ /cmd_vel ──▶ Bridge   │
│                                                                      │
│  behavior_server (spin/backup/wait) ──▶ /cmd_vel ──▶ Bridge         │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 3. Hardware & Software Components

### 3.1 Robot — TurtleBot3 Waffle

The **TurtleBot3 Waffle** is a compact two-wheeled differential-drive research robot.

| Spec | Value |
|------|-------|
| Drive type | Differential drive (2 wheels + caster) |
| Wheel radius | 0.033 m |
| Wheel separation | 0.287 m |
| Max linear speed | 0.26 m/s |
| Max angular speed | 1.82 rad/s |
| Sensor | 360° LiDAR — 360 beams, 3.5 m range |

In simulation the robot is loaded as a **SDF model** with three plugins:

| Plugin | Role |
|--------|------|
| `gz-sim-diff-drive-system` | Converts `/cmd_vel` Twist into wheel velocities |
| `gz-sim-sensors-system` | Runs the LiDAR sensor simulation |
| `gz-sim-joint-state-publisher-system` | Publishes wheel joint states |

### 3.2 Simulator — Gazebo Harmonic

[Gazebo Harmonic](https://gazebosim.org) (gz-sim 8.x) is the successor to Gazebo Classic. It provides rigid-body physics, sensor noise simulation, and real-time factor control.

| Launch Option | Value | Reason |
|---------------|-------|--------|
| `-r` | Run on start | No manual play button needed |
| `-s` | Server-only (headless) | Saves GPU/CPU resources |
| `-z 1000` | Physics update rate 1000 Hz | Locks RTF to exactly 1.0 |
| World | `empty.sdf` | Minimal inertia, no extra geometry |

> **Why `-z 1000`?**  
> `empty.sdf` uses `type="ignored"` in its physics tag, which causes Gazebo to run at 5–6× real-time on modern hardware. At that speed, TF timestamps race far ahead of wall-clock time and all Nav2 transform lookups silently fail. Setting the update rate to 1000 Hz with `max_step_size = 0.001 s` gives RTF = 0.001 × 1000 = **1.0**.

### 3.3 Middleware — ROS 2 Jazzy

**ROS 2 Jazzy Jalisco** provides:

| Component | Role |
|-----------|------|
| DDS (FastDDS) | Peer-to-peer message passing between all nodes |
| `rclpy` | Python client library |
| `tf2` | Coordinate frame transform tree |
| `lifecycle_manager` | Ordered configure → activate → deactivate for Nav2 nodes |
| `ros_gz_bridge` | Bidirectional ROS 2 ↔ Gazebo gz-transport topic bridge |

Bridge message type conversions:

| Direction | ROS 2 Type | Gazebo Type |
|-----------|-----------|-------------|
| GZ → ROS | `sensor_msgs/LaserScan` | `gz.msgs.LaserScan` |
| GZ → ROS | `nav_msgs/Odometry` | `gz.msgs.Odometry` |
| GZ → ROS | `tf2_msgs/TFMessage` | `gz.msgs.Pose_V` |
| GZ → ROS | `rosgraph_msgs/Clock` | `gz.msgs.Clock` |
| **ROS → GZ** | **`geometry_msgs/TwistStamped`** | **`gz.msgs.Twist`** |

> **Jazzy cmd_vel migration note:** Nav2 moved from `Twist` to `TwistStamped` in Jazzy. The parameter `enable_stamped_cmd_vel: True` **must** be set in `controller_server`, `velocity_smoother`, `spin`, and `backup` nodes. Without it, the bridge silently discards all velocity commands and the robot never moves — this was the hardest bug to diagnose in the migration.

### 3.4 Navigation Stack — Nav2

[Nav2](https://nav2.ros.org) is the complete ROS 2 navigation framework.

#### Nodes managed by `lifecycle_manager_navigation`

| Node | Role |
|------|------|
| `planner_server` | Global path planning via NavFn (Dijkstra) on the global costmap |
| `controller_server` | Local control via DWB — converts global path to per-step `cmd_vel` |
| `smoother_server` | Smooths the global path to remove sharp corners |
| `behavior_server` | Recovery behaviours: `spin`, `backup`, `wait` |
| `bt_navigator` | Behaviour Tree executor — orchestrates all Nav2 actions |
| `waypoint_follower` | Visits a list of goal poses in sequence |
| `velocity_smoother` | Applies acceleration/deceleration limits to raw `cmd_vel` |

#### DWB Local Planner

The **Dynamic Window Approach with B-spline (DWB)** controller:
1. Samples feasible (v_x, v_θ) velocity pairs within dynamic constraints.
2. Scores each trajectory with cost critics (goal alignment, path following, obstacle proximity).
3. Selects and publishes the lowest-cost velocity command at 10 Hz.

| DWB Parameter | Value |
|---------------|-------|
| `max_vel_x` | 0.26 m/s |
| `min_vel_x` | -0.1 m/s |
| `max_vel_theta` | 1.0 rad/s |
| `vx_samples` | 20 |
| `vtheta_samples` | 40 |
| `sim_time` | 1.5 s |

### 3.5 Localisation — AMCL

**Adaptive Monte Carlo Localisation (AMCL)** estimates the robot's pose within a known map using a particle filter.

**Algorithm:**
1. **Init** — Spread 500–2000 particles around the initial pose estimate.
2. **Predict** — Move each particle using the odometry motion model.
3. **Update** — Weight each particle by scan-vs-map likelihood.
4. **Resample** — Duplicate high-weight particles; discard low-weight ones.
5. **Publish** — Broadcast best-estimate `map → odom` transform.

| Parameter | Value | Effect |
|-----------|-------|--------|
| `min_particles` | 500 | Minimum particle count |
| `max_particles` | 2000 | Maximum particle count |
| `update_min_d` | 0.25 m | Update only after 25 cm movement |
| `update_min_a` | 0.2 rad | Update only after 11.5° rotation |
| `laser_model_type` | `likelihood_field` | Faster, smoother scan matching |
| `set_initial_pose` | `True` | Inject pose on startup |
| `transform_tolerance` | 1.0 s | TF timestamp slack for sim jitter |

#### AMCL Bootstrap Fix

AMCL only publishes `map → odom` TF after receiving a laser scan AND a valid initial pose. Nav2's `global_costmap` requires the `map` frame **on activation** — a deadlock.

**Fix:** `amcl_initializer.py` — custom bootstrap node that:
- Starts at t = 28 s (7 s before Nav2 at t = 35 s).
- Broadcasts `map → odom` TF at 5 Hz using the known spawn position.
- Sends `/initialpose` for the **first 2 seconds only** — stopping so AMCL's particle filter can converge without being continuously reset.
- Keeps broadcasting TF indefinitely as a fallback.

### 3.6 Mapping — map_server

Loads a pre-built 2-D occupancy grid from disk.

| File | Description |
|------|-------------|
| `config/maze.yaml` | Map metadata (resolution, origin, thresholds) |
| `config/maze.pgm` | Grayscale image — white = free, black = wall |

| Property | Value |
|----------|-------|
| Size | 338 × 320 pixels |
| Resolution | 0.050 m/pixel |
| Occupied threshold | > 0.65 |
| Free threshold | < 0.196 |

### 3.7 Visualisation — RViz2

| Display | Shows |
|---------|-------|
| **Map** | Occupancy grid from `map_server` |
| **LaserScan** | Real-time 360° LiDAR hits |
| **AMCL Particle Swarm** | Position uncertainty cloud |
| **Global Costmap** | Inflated obstacle map for global planning |
| **Local Costmap** | Rolling-window map for DWB controller |
| **RobotModel** | URDF mesh of TurtleBot3 Waffle |
| **TF** | All coordinate frame axes |

---

## 4. Package Structure

```
maze-solver-with-py/
└── autonomous_tb3/                    ROS 2 ament-python package
    ├── autonomous_tb3/                Python source files
    │   ├── maze_solver.py             Route 1 — original maze path
    │   ├── maze_solver_route2.py      Route 2 — upper-right quadrant
    │   ├── maze_solver_route3.py      Route 3 — middle to top passage
    │   ├── amcl_initializer.py        Bootstrap: TF seed + initialpose
    │   ├── occupancy_grid_pub.py      Utility: publish custom costmaps
    │   └── spawn_entity.py            Utility: programmatic model spawn
    ├── config/
    │   ├── tb3_nav_params.yaml        All Nav2 node parameters
    │   ├── maze.yaml                  Map metadata
    │   ├── maze.pgm                   Occupancy grid image
    │   └── tb3_nav.rviz               RViz2 display configuration
    ├── launch/
    │   └── maze_navigation.launch.py  Master staggered launch file
    ├── world/
    │   └── maze/model.sdf             3-D maze model for Gazebo
    ├── setup.py                       Console script entry points
    └── package.xml                    ROS 2 package dependencies
```

---

## 5. How It Works

### 5.1 Pre-built Map

The maze map was created offline and saved as a PGM image + YAML. This is a **known-map** approach — the robot does not build the map at runtime; it localises against a fixed reference.

### 5.2 Localisation Pipeline

```
LiDAR scan (/scan)
      │
      ▼
  AMCL node
  ┌──────────────────────────────┐
  │ 1. Motion model (odometry)   │
  │    Move particles forward    │
  │ 2. Sensor model (scan match) │
  │    Weight each particle      │
  │ 3. Resample                  │
  │    Concentrate near truth    │
  │ 4. Estimate robot pose       │
  └──────────────────────────────┘
      │
      ├── /amcl_pose   (pose estimate, for maze_solver feedback)
      └── /tf          (map → odom transform, consumed by Nav2)
```

### 5.3 Navigation Pipeline

```
maze_solver.py
  └── BasicNavigator.goThroughPoses([goal1, goal2])
          │
          ▼
     bt_navigator  (Behaviour Tree)
          │
          ├─▶ planner_server.ComputePathToPose
          │       └── NavFn (Dijkstra) on global_costmap
          │               └── Returns global path (sequence of poses)
          │
          └─▶ controller_server.FollowPath
                  └── DWB LocalPlanner
                          ├── Sample (v_x, v_θ) pairs
                          ├── Score each trajectory
                          └── Publish best cmd_vel_nav
```

**Costmaps:**

| Costmap | Scope | Purpose |
|---------|-------|---------|
| **Global costmap** | Full map | Dijkstra plans long-range paths |
| **Local costmap** | 3 × 3 m rolling window | DWB avoids real-time obstacles |

Both costmaps use: Static layer (map) + Obstacle layer (scan) + Inflation layer (0.55 m buffer).

### 5.4 Velocity Command Pipeline

```
controller_server  ─── TwistStamped ──▶ cmd_vel_nav
                                              │
                                              ▼
                                    velocity_smoother
                                    (accel/decel limits)
                                              │
                                              ▼  TwistStamped
                                          /cmd_vel
                                              │
                                              ▼
                                       ros_gz_bridge
                                    (TwistStamped → Twist)
                                              │
                                              ▼
                                   Gazebo DiffDrive plugin
                                              │
                                              ▼
                                       Robot moves
```

Recovery behaviours (`spin`, `backup`) publish **directly** to `/cmd_vel`, bypassing the smoother.

### 5.5 TF Frame Tree

```
map
 └── odom               ← AMCL (map→odom) / seeded by amcl_initializer
      └── base_footprint ← DiffDrive plugin (odom→base_footprint)
           └── base_link  ← robot_state_publisher (URDF fixed joint)
                ├── base_scan       ← LiDAR frame (URDF fixed)
                ├── wheel_left_link
                └── wheel_right_link
```

---

## 6. Control Flow (End-to-End)

```
t =  0 s  Gazebo starts — physics at 1000 Hz (RTF = 1.0)
          Maze SDF model loads into Gazebo world

t = 20 s  TurtleBot3 Waffle spawns at  (-5.18, -6.58)
          ros_gz_bridge starts  →  /scan, /odom, /tf, /clock bridged
          robot_state_publisher →  static TF published (/tf_static)

t = 25 s  lifecycle_manager_localization starts
          map_server:  loads maze.pgm  →  publishes /map
          amcl:        configures & activates
                       set_initial_pose injects (-5.18, -6.58)

t = 28 s  amcl_initializer starts
          →  broadcasts map→odom TF at 5 Hz
          →  sends /initialpose for first 2 s (10 ticks) then STOPS
          →  AMCL particle filter converges without constant resets

t = 35 s  lifecycle_manager_navigation starts
          All Nav2 nodes:  configure  →  activate  (in order)
          global_costmap finds map frame  ✓  (seeded by amcl_initializer)
          Navigation stack is ACTIVE
          RViz2 opens — map, scans, costmaps, particles visible

t = 35+   USER runs:  ros2 run autonomous_tb3 maze_solver

          maze_solver.py steps:
            1. Publish /initialpose (15 s window, non-fatal on timeout)
            2. Create BasicNavigator
            3. Call waitUntilNav2Active()
            4. Build list of PoseStamped goals
            5. Call navigator.goThroughPoses([goal1, goal2])

          bt_navigator receives NavigateThroughPoses action
          → planner_server computes Dijkstra global path
          → controller_server runs DWB at 10 Hz
          → velocity_smoother applies accel/decel limits
          → ros_gz_bridge converts TwistStamped → Twist → Gazebo
          → DiffDrive drives robot wheels

          [If stuck / no progress]:
          → behavior_server: spin → wait → backup → retry

          [Goal within xy_tolerance = 0.25 m]:
          → advance to next waypoint

FINAL     maze_solver prints:  "Goal succeeded!"
          navigator.lifecycleShutdown()  — graceful stop
```

---

## 7. Key Configuration Parameters

**File:** `config/tb3_nav_params.yaml`

### AMCL
```yaml
set_initial_pose: True      # Inject pose on activation (avoids TF deadlock)
transform_tolerance: 1.0    # Allow 1 s TF timestamp slack
update_min_d: 0.25          # Update filter every 25 cm
update_min_a: 0.2           # Update filter every 11.5°
initial_pose:
  x: -5.18
  y: -6.58
```

### Controller Server
```yaml
enable_stamped_cmd_vel: True  # CRITICAL — publish TwistStamped for bridge
controller_frequency: 10.0
movement_time_allowance: 10.0
```

### Velocity Smoother
```yaml
enable_stamped_cmd_vel: True  # CRITICAL — same reason
smoothing_frequency: 20.0
max_velocity: [0.22, 0.0, 1.0]   # linear / lateral / angular
max_accel:    [2.5,  0.0, 3.2]
```

### Behavior Server
```yaml
enable_stamped_cmd_vel: True  # CRITICAL — spin/backup need TwistStamped
max_rotational_vel: 1.0
min_rotational_vel: 0.4
```

### Global Costmap
```yaml
inflation_radius: 0.55   # Wall buffer (robot safety margin)
robot_radius: 0.22       # TurtleBot3 Waffle effective radius
```

---

## 8. Multi-Route Operation

Three independent solver scripts share the same launch — only Terminal 2 command changes.

| Script | Waypoints | Description |
|--------|-----------|-------------|
| `maze_solver` | `(-1.23,-2.1)` → `(-7.4,-1.17)` | Original cross-maze path |
| `maze_solver_route2` | `(-2.50,-5.00)` → `(0.00,-2.50)` | Right side to upper-right |
| `maze_solver_route3` | `(0.804,0.577)` → `(4.66,-1.41)` | Middle corridor to top passage |

**Adding custom routes:** Edit `WAYPOINTS` in any solver file. Use RViz2 **Publish Point** tool to read corridor coordinates — the z value returned is floor height and is ignored for 2-D navigation.

---

## 9. ROS 2 Humble → Jazzy Migration

| # | Category | Problem | Fix Applied |
|---|----------|---------|-------------|
| 1 | Simulator | Gazebo Classic deprecated | Replaced with Gazebo Harmonic via `ros_gz_sim` |
| 2 | bt_navigator | FATAL crash — plugin already registered | Removed `plugin_lib_names` (Jazzy auto-loads built-ins) |
| 3 | bt_navigator | Missing `error_code_names` parameter | Added mandatory Jazzy parameter |
| 4 | bringup | `bringup_launch.py` hardcodes `docking_server` | Custom per-node launch file |
| 5 | map_server | `RewrittenYaml` path injection silently failed | Direct `map_yaml_file` parameter injection |
| 6 | Gazebo RTF | Sim ran at 5.75× real-time → all TF failures | Added `-z 1000` to Gazebo server args |
| 7 | AMCL startup | `map` frame missing when Nav2 activated | `amcl_initializer.py` seeds TF 7 s before Nav2 |
| 8 | AMCL reset | `/initialpose` flood reset particle filter repeatedly | Cap to 10 ticks (2 s) then stop sending |
| 9 | **cmd_vel type** | **All velocity commands silently dropped** | **`enable_stamped_cmd_vel: True` in all velocity nodes** |
| 10 | Clock race | AMCL set pose at sim-time = 0 | `set_initial_pose` YAML param instead of runtime message |

---

## 10. Known Limitations & Future Work

| Limitation | Description | Potential Fix |
|------------|-------------|---------------|
| Fixed spawn | All routes start at `(-5.18, -6.58)` | Parameterise `x_pose`, `y_pose` as launch args and update `amcl_initializer` |
| AMCL convergence | Maze solver proceeds without confirmed AMCL convergence | Subscribe to `/amcl_pose` with convergence threshold check |
| Static map | Cannot handle new dynamic obstacles | Integrate SLAM Toolbox for live re-mapping |
| No multi-robot | Single TurtleBot3 only | Add ROS 2 namespaces for multi-robot operation |
| Real hardware untested | Validated in Gazebo Harmonic only | Port to physical TurtleBot3 with TF calibration |
| Recovery sensitivity | Spin/backup timeouts are strict | Tune `behavior_server` time allowances per maze complexity |

---

## 11. Quick-Start Commands

### Terminal 1 (same for all routes):
```bash
pkill -f "gz sim" 2>/dev/null; pkill -f "gz_sim" 2>/dev/null; sleep 2
cd ~/Documents/MAR_LAST && source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch autonomous_tb3 maze_navigation.launch.py
```

### Terminal 2 — Route 1 (original):
```bash
cd ~/Documents/MAR_LAST && source install/setup.bash
ros2 run autonomous_tb3 maze_solver
```

### Terminal 2 — Route 2 (upper-right):
```bash
cd ~/Documents/MAR_LAST && source install/setup.bash
ros2 run autonomous_tb3 maze_solver_route2
```

### Terminal 2 — Route 3 (3-waypoint loop):
```bash
cd ~/Documents/MAR_LAST && source install/setup.bash
ros2 run autonomous_tb3 maze_solver_route3
```

### Build the package
```bash
cd ~/Documents/MAR_LAST
colcon build --symlink-install --packages-select autonomous_tb3
source install/setup.bash
```

### Diagnostic commands
```bash
ros2 run tf2_tools view_frames      # Visualise full TF tree
ros2 topic hz /cmd_vel              # Verify velocity commands are flowing
ros2 topic hz /scan                 # Verify LiDAR data from bridge
ros2 topic echo /amcl_pose          # Watch AMCL localisation output
```

---

*Report for `maze-solver-with-py` · branch `ro4`  
ROS 2 Jazzy Jalisco · Gazebo Harmonic · Nav2 · TurtleBot3 Waffle*
