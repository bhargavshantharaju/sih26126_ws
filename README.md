# DRISTHI — Autonomous Terrain-Aware UGV Navigation

ROS 2 Jazzy and Gazebo Sim stack for GPS-denied outdoor navigation with live
perception, depth-based mapping, LiDAR SLAM, wheel+IMU EKF fusion, frontier
exploration, local planning, stuck detection/recovery, and safety-supervised
motion.

## System Architecture

```text
Gazebo RGB/depth/IMU/LiDAR/odometry
        ↓
ROS-Gazebo bridge
        ├── RGB image   → perception baseline → traversability score image
        ├── depth cloud → terrain geometry / classification → terrain-aware score
        ├── depth cloud + odometry → online occupancy mapper → /map (when SLAM disabled)
        ├── /scan       → slam_toolbox → map → odom TF  (when slam:=true)
        └── IMU + wheel odometry → robot_localization EKF → odom → base_link TF
                                                         ↓
                                            /odometry/filtered
                                                         ↓
                                         sensor health / contracts
                                                         ↓
                                      reachable frontier explorer
                                        ├── /exploration/goal
                                        └── /planner/global_path
                                                         ↓
                               goal advisory → local path selector
                                                         ↓
                                   safety_supervisor → /cmd_vel → Gazebo
                                        ↑
                             stuck_recovery (advisory /recovery/cmd_vel_suggestion)
```

### TF Authority

| Transform       | Authority                 | Condition              |
|-----------------|---------------------------|------------------------|
| map → odom      | slam_toolbox              | slam:=true             |
| odom → base_link| robot_localization EKF    | fusion:=true           |
| odom → base_link| odom_tf_publisher         | slam:=true, fusion:=false |
| odom → base_link| Gazebo TF bridge          | slam:=false            |

**Exactly one authority** per TF edge is active at runtime.

### Key Design Constraints

- `safety_supervisor` is the **sole** `/cmd_vel` publisher. All planner and
  recovery outputs are advisory.
- `/map` has exactly one publisher (either slam_toolbox or online_mapper,
  never both).
- The dashboard is **read-only** — it derives all data from ROS topics.

## Build

```bash
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## One-Command Startup (DRISTHI full stack)

```bash
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch sih26126_bringup full_stack.launch.py \
  slam:=true \
  fusion:=true \
  start_dashboard:=true \
  gui:=true \
  scenario:=showcase
```

Open the `localhost` URL printed by the dashboard at startup.

### Launch Arguments

| Argument           | Default   | Description                                      |
|--------------------|-----------|--------------------------------------------------|
| `slam`             | `false`   | Enable LiDAR SLAM (slam_toolbox)                 |
| `fusion`           | `false`   | Enable wheel+IMU EKF fusion (robot_localization) |
| `gui`              | `false`   | Show Gazebo GUI                                  |
| `start_dashboard`  | `true`    | Start the DRISTHI live dashboard                 |
| `dashboard_port`   | `8080`    | Preferred HTTP port (advances up to +10)         |
| `scenario`         | `showcase`| Scenario: `showcase` or `dynamic_obstacle`       |
| `world`            | showcase  | Absolute path to Gazebo world SDF                |

## Available Scenarios

### `scenario:=showcase` (default)

Locally generated heightmap terrain with a central drivable valley, trees,
rocks, and fallen logs. No supplied map or hidden route. The robot starts
with an all-unknown occupancy grid and autonomously explores via frontier
selection, terrain-aware traversability scoring, and safety-supervised motion.

### `scenario:=dynamic_obstacle` (experimental / incomplete)

**Status: NOT COMPLETE.** This scenario attempts a dynamic obstacle encounter
but has not demonstrated:
- Valid forward LiDAR returns from a moving obstacle
- UGV–obstacle encounter during navigation
- Obstacle-triggered planner or safety response
- Collision-free evasive reaction

It is preserved for future development. Do not claim it works.

## Runtime Topics

### Sensors
- `/camera`, `/camera_info` — RGB camera
- `/depth_camera/points` — depth point cloud
- `/imu` — IMU
- `/lidar` (bridged as `/scan`) — 2D LiDAR

### Localization Contract
- `/localization/pose` — active pose for navigation
- `/localization/source` — origin of pose (e.g. `slam_toolbox`, `gazebo_odometry`)
- `/localization/confidence` — health signal (not a visual odometry accuracy estimate)
- `/odometry/filtered` — EKF-fused odometry (when fusion:=true)

### Mapping / Exploration
- `/map` — occupancy grid
- `/mapping/state`, `/exploration/state`, `/exploration/frontiers`, `/exploration/goal`

### Planning
- `/planner/global_path`, `/planner/local_path`
- `/planner/local_recommended_cmd_vel` — advisory (not actuation)
- `/planner/local_obstacle_clearance`, `/planner/local_obstacle_score`

### Safety / Motion
- `/safety/state`, `/safety/reason`
- `/cmd_vel` — **sole authority: safety_supervisor only**

### Recovery
- `/recovery/state` — NORMAL / STUCK_SUSPECTED / RECOVERY_ACTIVE / RECOVERY_SUCCESS / RECOVERY_FAILED / STOPPED
- `/recovery/cmd_vel_suggestion` — advisory (safety_supervisor decides whether to use it)
- `/recovery/reason` — human-readable recovery status

## Stuck Detection and Recovery

The `stuck_recovery_node` (in `sih26126_safety`) detects when the UGV is
commanded to move but measured velocity remains near zero:

**State machine:**
```
NORMAL → STUCK_SUSPECTED → RECOVERY_ACTIVE → RECOVERY_SUCCESS → NORMAL
                        ↘                  ↘
                         NORMAL              RECOVERY_FAILED → STOPPED
```

**Guard conditions** (recovery is blocked if any are true):
- Localization confidence < 0.1
- Safety supervisor state is STOPPED or E_STOP
- No planner command received
- Commanded speed effectively zero

**Recovery sequence:** Stop → Reverse → Rotate → Stop → Re-evaluate

If obstacle clearance < 0.3 m during reverse: abort reverse, proceed to
rotate or stop. After `max_recovery_attempts` (default 3) the node enters
terminal STOPPED state.

**Architecture:** The recovery node does NOT publish to `/cmd_vel`. It
publishes advisory suggestions to `/recovery/cmd_vel_suggestion` only.
The safety_supervisor remains the sole `/cmd_vel` authority.

### Configurable Parameters

| Parameter                      | Default | Description                         |
|--------------------------------|---------|-------------------------------------|
| `stuck_command_speed_threshold`| 0.05 m/s| Min commanded speed to suspect stuck |
| `stuck_measured_speed_threshold`| 0.02 m/s| Max measured speed to consider stuck |
| `stuck_timeout_sec`            | 3.0 s   | Duration before STUCK_SUSPECTED      |
| `stuck_suspect_timeout_sec`    | 2.0 s   | Hysteresis before RECOVERY_ACTIVE    |
| `recovery_reverse_speed`       | 0.10 m/s| Reverse speed during recovery        |
| `recovery_reverse_time_sec`    | 2.0 s   | Duration of reverse phase            |
| `recovery_rotate_speed`        | 0.3 r/s | Rotation speed during recovery       |
| `recovery_rotate_time_sec`     | 2.0 s   | Duration of rotation phase           |
| `max_recovery_attempts`        | 3       | Attempts before terminal STOPPED     |
| `min_clearance_for_reverse`    | 0.3 m   | Minimum clearance to allow reverse   |

## Tests

```bash
colcon test
colcon test-result --verbose
```

The current suite contains 321 tests: 316 passed, 5 skipped, 0 failures, and
0 errors. The stuck detection state machine has deterministic unit tests
covering guard conditions, FSM transitions, displacement calculation, and
architecture constraints (no direct `/cmd_vel` publishing from recovery).

Live Gazebo end-to-end recovery testing (vehicle physically gets stuck and
autonomously recovers) is future work requiring a controlled test scenario.

## Packages

| ROS Package                  | Role                                        |
|------------------------------|---------------------------------------------|
| `sih26126_bringup`           | Launch files — `full_stack.launch.py` is the entry point |
| `sih26126_autonomy`          | Frontier explorer, mission manager, online mapper |
| `sih26126_safety`            | Safety supervisor (sole /cmd_vel), stuck recovery |
| `sih26126_planner`           | Global + local planners (advisory)          |
| `sih26126_localization`      | SLAM adapter, EKF adapter, TF publishers    |
| `sih26126_perception`        | RGB perception baseline                     |
| `sih26126_traversability`    | Traversability scoring                      |
| `sih26126_geometry`          | Terrain geometry / RANSAC slope estimation  |
| `sih26126_sensor_contract`   | Sensor health, LiDAR monitor, scan relay    |
| `sih26126_dashboard`         | Read-only live autonomy dashboard           |
| `sih26126_sim`               | Simulation worlds and UGV model             |
| `sih26126_msgs`              | Custom ROS message definitions              |

## Localization Scope

This is **ODOMETRY-ASSISTED ONLINE MAPPING** with optional LiDAR SLAM — not
visual odometry and not GPS-fused navigation. When `slam:=true`, `slam_toolbox`
provides loop-closure-capable map-to-odom estimation from 2D LiDAR. When
`fusion:=true`, `robot_localization` fuses wheel odometry and IMU for
odom-to-base_link. No loop closure is claimed in baseline (`slam:=false`) mode.

The global explorer's `NO_FRONTIER` status means no reachable frontier is
currently available in the observed free map — it does not mean the whole
world has been explored.

## Validation Summary (2026-10-05)

Verified live on this stack with `slam:=true fusion:=true scenario:=showcase`:

| Topic / Check              | Result              |
|----------------------------|---------------------|
| `/lidar/points`            | ~5.6 Hz ✅          |
| `/lidar/scan_2d`           | ~5.6 Hz ✅          |
| `/imu`                     | ~100 Hz ✅          |
| `/odometry/filtered` (EKF) | ~17 Hz ✅           |
| `map → odom` TF (SLAM)     | Published ✅        |
| `fusion_health_monitor`    | HEALTHY ✅          |
| Unit test suite            | 321 total; 316 passed; 5 skipped; 0 failures/errors ✅ |

See `docs/VALIDATION.md` for full details.

## Current Limitations

- Dynamic obstacle detection/avoidance is **not complete** (scenario preserved, not validated)
- Live Gazebo recovery test scenario not yet implemented
- Terrain confidence / temporal memory: planned, not implemented
- Information-gain-aware frontier selection: planned, not implemented
- Risk-aware planning (global): planned, not implemented

See `docs/KNOWN_LIMITATIONS.md` for full details.

## Next Planned Features

1. Dynamic obstacle handling (complete the sensor detection pipeline)
2. Terrain confidence / temporal memory
3. Information-gain-aware frontier selection
4. Risk-aware planning
5. Scenario-driven integration tests
6. Degraded-autonomy mode
