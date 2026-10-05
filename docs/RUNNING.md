# DRISTHI — Running the System

## Prerequisites

- Ubuntu 24.04 (or compatible)
- ROS 2 Jazzy
- Gazebo Harmonic (or compatible)
- `colcon`, `rosdep`

## Build

```bash
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Primary Launch Command

```bash
ros2 launch sih26126_bringup full_stack.launch.py \
  slam:=true \
  fusion:=true \
  start_dashboard:=true \
  gui:=false \
  scenario:=showcase
```

This starts:
- Gazebo Sim (headless)
- All ROS–Gazebo topic bridges (including LiDAR PointCloud2 rename bridge)
- slam_toolbox (async SLAM)
- robot_localization EKF (wheel odometry + IMU fusion)
- Perception, terrain, traversability pipeline
- Frontier explorer and mission manager
- Safety supervisor + stuck recovery
- Live dashboard on http://localhost:8080

## Launch Arguments

| Argument              | Default     | Description                                              |
|-----------------------|-------------|----------------------------------------------------------|
| `slam`                | `false`     | Enable LiDAR SLAM (slam_toolbox)                         |
| `fusion`              | `false`     | Enable EKF wheel+IMU fusion (robot_localization)         |
| `gui`                 | `false`     | Show Gazebo GUI (use `false` in headless/WSL environments) |
| `start_dashboard`     | `true`      | Start the live HTTP dashboard                            |
| `dashboard_port`      | `8080`      | HTTP port (auto-advances up to +10 if occupied)          |
| `scenario`            | `showcase`  | `showcase` (default) or `dynamic_obstacle` (experimental) |
| `world`               | showcase SDF| Absolute path to Gazebo world SDF                        |
| `exploration_radius_m`| `1000.0`   | Max frontier exploration radius from home                |
| `mission_timeout_sec` | `1800.0`   | Mission failure cutoff (not exploration completion)      |

## Dashboard Access

Once launched, the dashboard URL is printed to the terminal:

```
[dashboard] DRISTHI dashboard running at http://localhost:8080
```

Open it in any browser. The dashboard is **read-only** — it only subscribes to
ROS topics and never publishes.

### What the Dashboard Shows

| Question                    | Location                               |
|-----------------------------|----------------------------------------|
| Is the robot healthy?       | Top bar — SYSTEM ONLINE/DEGRADED/OFFLINE |
| Is localization working?    | DRISTHI Status panel → Localization confidence |
| Is there a hazard?          | Top bar — RISK badge + Active Risk panel |
| Why is it stopped?          | DRISTHI Status panel → WHY DID THE ROBOT STOP? banner |
| What is it doing?           | Mission Intelligence panel + top bar mission badge |
| What happened recently?     | Mission Intelligence Events timeline   |

Technical details (ROS topics, TF, sensor rates) are in the collapsible
"Technical Details — ROS Topic Status" section at the bottom of the left column.

## Scenarios

### `showcase` (default, validated)

Locally generated heightmap terrain with drivable valley, rocks, and obstacles.
The robot starts with an all-unknown map and autonomously explores via frontier
selection, terrain-aware traversability, and safety-supervised motion.

### `dynamic_obstacle` (experimental, not fully validated)

Spawns a moving obstacle in a separate world. The obstacle controller moves
deterministically across the robot's path. Sensor-based detection via LiDAR
is not yet validated for reliable obstacle encounters.

## Shutdown

Press `Ctrl+C` in the launch terminal. All nodes will shut down cleanly.
A mission result JSON is written to `artifacts/mission_results/` on completion.

## Verifying the Stack

After launching (wait ~30 seconds for SLAM to activate):

```bash
# LiDAR point cloud
ros2 topic hz /lidar/points

# 2D scan (from LiDAR processor)
ros2 topic hz /lidar/scan_2d

# IMU
ros2 topic hz /imu

# EKF-fused odometry
ros2 topic hz /odometry/filtered

# SLAM TF (should appear after SLAM activates)
ros2 run tf2_ros tf2_echo map odom
```

## Running Tests

```bash
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test
colcon test-result --all
```

Expected: all tests pass (210 unit tests as of submission).
