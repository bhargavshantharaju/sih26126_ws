# DRISTHI — Autonomous Terrain-Aware UGV Navigation

> **SIH 2026 — Problem Statement 26126**
>
> GPS-denied outdoor navigation with live LiDAR SLAM, sensor-fused odometry,
> depth-based traversability scoring, frontier exploration, and safety-supervised
> autonomous motion — all running on a Gazebo-simulated UGV under ROS 2 Jazzy.

---

## Status

| Capability | State |
|---|---|
| Build (12 packages, ROS 2 Jazzy) | ✅ Verified |
| Unit + lint test suite | ✅ 324 passed, 0 failures, 5 skipped (329 total) |
| Simulation clock and sensor pipeline | ✅ Verified live |
| LiDAR SLAM (`slam_toolbox`) | ✅ Map grows from 7 × 7 → 172 × 145 cells |
| EKF wheel + IMU fusion | ✅ Sustained HEALTHY |
| `odom → base_link` TF (EKF) | ✅ Verified live |
| `map → odom` TF (SLAM) | ✅ Verified live |
| Frontier detection and selection | ✅ 622 frontiers detected, 2+ completed |
| Autonomous motion (unsupervised) | ✅ Verified: changing poses in map frame, 0.09–0.12 m/s |
| Safety supervisor sole `/cmd_vel` authority | ✅ 1 publisher confirmed |
| Dynamic obstacle scenario | ⚠️ Incomplete — preserved, not validated |
| Terrain confidence / temporal memory | 🔲 Planned |
| Information-gain-aware frontier selection | 🔲 Planned |

---

## System Architecture

```mermaid
flowchart TD
    GZ([Gazebo Harmonic\nminimal_stable.sdf])

    GZ -->|PointCloud2\n/lidar/points| LP[lidar_processor\n3D→2D LaserScan\n+ health gate]
    GZ -->|Odometry\n/model/sih_ugv/odometry| EKF_IN[ekf_sensor_adapter\ncovariance injection]
    GZ -->|IMU\n/imu| EKF_IN
    GZ -->|RGB\n/camera| PERC[perception_baseline\nRGB traversability]
    GZ -->|DepthCloud\n/depth_camera/points| TG[terrain_geometry\nRANSAC slope]
    GZ -->|/clock| CLK[/clock bridge\nsim time]

    CLK --> ALL_NODES(all nodes\nuse_sim_time=true)

    LP -->|/lidar/scan_2d| SLAM[slam_toolbox\nasync SLAM\nmin_travel=0.0]
    LP -->|/sensor_health/lidar| DA[degraded_autonomy_manager]
    EKF_IN -->|/fusion/wheel_odometry\n/fusion/imu| EKF[robot_localization EKF\n20 Hz]
    EKF -->|odom→base_link TF| TFEDGE1[[TF: odom→base_link]]
    SLAM -->|map→odom TF| TFEDGE2[[TF: map→odom]]
    SLAM -->|/map OccupancyGrid| FE[frontier_explorer\nDijkstra + bootstrap]

    PERC -->|/traversability/score_map| TAT[terrain_aware_traversability]
    TG -->|terrain scores| TAT
    TAT -->|/traversability/terrain_aware_mean_score| GPP[goal_path_planner\nmap-frame\nterrain fallback]

    EKF -->|/odometry/filtered| GPP
    FE -->|/exploration/goal| GPP
    FE -->|/planner/global_path| GPP

    GPP -->|/planner/navigation_state| LPS[local_path_selector\ncandidates + scoring]
    GPP -->|/planner/path| LPS
    LPS -->|/planner/local_navigation_state\n/planner/local_recommended_cmd_vel| SS

    DA -->|/autonomy/mode| SS[safety_supervisor\nSOLE /cmd_vel publisher]
    SR[stuck_recovery_node\nadvisory only] -->|/recovery/cmd_vel_suggestion| SS
    SS -->|/cmd_vel| GZ
```

### TF Authority

| Transform | Authority | Condition |
|---|---|---|
| `map → odom` | `slam_toolbox` | `slam:=true` |
| `odom → base_link` | `ekf_filter_node` | `fusion:=true` |
| `odom → base_link` | `odom_tf_publisher` | `slam:=true`, `fusion:=false` |
| `odom → base_link` | Gazebo TF bridge | `slam:=false` |
| `base_link → lidar_3d` | `lidar_static_tf_publisher` | `slam:=true` (static) |
| `base_link → imu_sensor` | `imu_static_tf_publisher` | `fusion:=true` (static) |

**Exactly one authority per TF edge is active at runtime.**

---

## Package Map

| Package | Role |
|---|---|
| `sih26126_bringup` | Launch files — `full_stack.launch.py` is the single entry point |
| `sih26126_sim` | Gazebo worlds, UGV SDF models, heightmap assets |
| `sih26126_msgs` | Custom ROS 2 message types (`SensorHealth`, `AutonomyMode`, `MissionEvent`) |
| `sih26126_localization` | SLAM activator, EKF adapters, static TF publishers, slam/odom pose adapters |
| `sih26126_sensor_contract` | LiDAR processor (3D→2D), health monitor, scan relay, obstacle extractor/tracker |
| `sih26126_perception` | RGB camera traversability baseline |
| `sih26126_geometry` | RANSAC terrain-slope geometry, terrain classifier |
| `sih26126_traversability` | Terrain-aware traversability scoring |
| `sih26126_autonomy` | Frontier explorer, online mapper, mission manager, mission intelligence |
| `sih26126_planner` | Global goal planner (map-frame), local path selector (advisory) |
| `sih26126_safety` | Safety supervisor (sole `/cmd_vel`), stuck recovery (advisory), degraded-autonomy FSM |
| `sih26126_dashboard` | Read-only live autonomy dashboard (HTTP) |

---

## Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 or 24.04 (tested under WSL2) |
| ROS 2 | **Jazzy** |
| Gazebo | **Harmonic** (`gz-harmonic`) |
| `ros_gz_bridge` | Jazzy-compatible |
| `slam_toolbox` | ROS 2 Jazzy |
| `robot_localization` | ROS 2 Jazzy |
| Python | 3.10+ |

---

## Quick Start

### 1. Clone and install dependencies

```bash
mkdir -p ~/sih26126_ws/src
cd ~/sih26126_ws/src
# (place this repository here, or clone it)
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Build

```bash
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Expected output: `Summary: 12 packages finished`

### 3. Launch full autonomous stack

```bash
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch sih26126_bringup full_stack.launch.py \
  slam:=true \
  fusion:=true \
  gui:=false \
  start_dashboard:=true \
  scenario:=showcase
```

The dashboard URL is printed at startup (default `http://localhost:8080`).

### 4. Launch arguments

| Argument | Default | Description |
|---|---|---|
| `slam` | `false` | Enable LiDAR SLAM (`slam_toolbox`) |
| `fusion` | `false` | Enable wheel + IMU EKF fusion (`robot_localization`) |
| `gui` | `false` | Show Gazebo GUI |
| `start_dashboard` | `true` | Start the live read-only dashboard |
| `dashboard_port` | `8080` | Preferred HTTP port (tries up to +10) |
| `scenario` | `showcase` | `showcase` (default) or `dynamic_obstacle` (experimental) |
| `world` | showcase SDF | Absolute path to a Gazebo world SDF |

---

## Verifying a Successful Startup

After launching with `slam:=true fusion:=true`, the following indicators confirm
the stack is healthy (check within ~30 s of launch):

```bash
# Simulation clock advancing
ros2 topic echo /clock --once

# LiDAR scan flowing
ros2 topic hz /lidar/scan_2d

# EKF fused odometry
ros2 topic echo /odometry/filtered --once

# TF chain: map → odom → base_link
ros2 run tf2_tools view_frames        # or:
ros2 topic echo /tf --once | grep frame_id

# Map growing (width/height should exceed 7×7 within 20 s)
ros2 topic echo /map --once | grep -E 'width|height'

# Exploration active
ros2 topic echo /exploration/state --once

# Safety supervisor is the only /cmd_vel publisher
ros2 topic info -v /cmd_vel | grep 'Node name'
```

Expected healthy indicators:

```
/clock         → advancing sim-time
/odometry/filtered → frame_id: odom, child_frame_id: base_link
/map           → width: >7, height: >7, frame_id: map
/exploration/state → EXPLORING  (after ~20 s)
/safety/state  → SAFE  (once a frontier goal is issued)
/cmd_vel publisher → safety_supervisor (count: 1)
```

---

## Scenarios

### `scenario:=showcase` (default, validated)

Locally generated heightmap terrain with a central drivable valley, trees,
rocks, and fallen logs. The robot starts with an all-unknown map and
explores autonomously using frontier selection, terrain traversability
scoring, and safety-supervised motion.

**Validated behaviour (2026-10-12, `minimal_stable.sdf`):**

- Map: 7 × 7 → 172 × 145 cells within 60 s
- Frontiers: 622 cells detected; 2+ goals reached
- Robot motion: changing poses in map frame, 0.32 m net displacement observed
- Sustained speed: 0.09–0.12 m/s
- EKF fusion: HEALTHY throughout

### `scenario:=dynamic_obstacle` ⚠️ (experimental, not validated)

A moving obstacle is spawned. **This scenario has not demonstrated:**
- Valid forward LiDAR returns from a moving obstacle
- UGV–obstacle encounter during navigation
- Obstacle-triggered evasive response

It is preserved for future development. Do not claim it works.

---

## Runtime Topics

### Sensors
| Topic | Type | Source |
|---|---|---|
| `/lidar/points` | `PointCloud2` | Gazebo LiDAR bridge |
| `/lidar/scan_2d` | `LaserScan` | `lidar_processor` |
| `/sensor_health/lidar` | `SensorHealth` | `lidar_processor` |
| `/imu` | `Imu` | Gazebo IMU bridge |
| `/camera` | `Image` | Gazebo camera bridge |
| `/depth_camera/points` | `PointCloud2` | Gazebo depth camera bridge |

### Localization
| Topic | Description |
|---|---|
| `/localization/pose` | Active pose for navigation (map frame when SLAM active) |
| `/localization/source` | Origin of pose (e.g. `slam_toolbox`) |
| `/localization/confidence` | Health signal [0–1] |
| `/odometry/filtered` | EKF-fused odometry (`fusion:=true`) |

### Mapping and Exploration
| Topic | Description |
|---|---|
| `/map` | OccupancyGrid — sole publisher: `slam_toolbox` when `slam:=true` |
| `/mapping/state` | Mapper FSM state |
| `/exploration/state` | Frontier explorer state (`MAPPING` → `EXPLORING` → …) |
| `/exploration/frontiers` | Raw frontier cells |
| `/exploration/goal` | Current selected frontier goal (map frame) |
| `/exploration/frontier_count` | Number of raw frontier cells |
| `/exploration/frontiers_completed` | Reached frontier count |
| `/planner/global_path` | Global path from explorer to goal |

### Planning and Safety
| Topic | Description |
|---|---|
| `/planner/navigation_state` | Planner FSM state (`NAVIGATE`, `WAITING_FOR_EXPLORATION_GOAL`, …) |
| `/planner/local_navigation_state` | Local selector state (`LOCAL_NAVIGATE`, `LOCAL_STOP_OBSTACLE`) |
| `/planner/local_recommended_cmd_vel` | Advisory velocity — **not actuation** |
| `/planner/local_obstacle_clearance` | Forward clearance in metres |
| `/safety/state` | Safety supervisor state (`SAFE`, `STOPPED`, …) |
| `/safety/reason` | Human-readable reason for current state |
| `/cmd_vel` | **Sole authority: `safety_supervisor` only** |

### Recovery and Autonomy Mode
| Topic | Description |
|---|---|
| `/recovery/state` | Stuck recovery FSM: `NORMAL` → `STUCK_SUSPECTED` → `RECOVERY_ACTIVE` → … |
| `/recovery/cmd_vel_suggestion` | Advisory only — supervisor decides whether to use it |
| `/recovery/reason` | Human-readable recovery status |
| `/autonomy/mode` | `AutonomyMode`: `FULL_AUTONOMY`, `DEGRADED_SENSOR`, `SAFE_HOLD`, … |
| `/autonomy/capability_score` | [0–1] capability score |

---

## Stuck Detection and Recovery

`stuck_recovery_node` (in `sih26126_safety`) detects commanded-but-zero measured velocity:

```
NORMAL → STUCK_SUSPECTED → RECOVERY_ACTIVE → RECOVERY_SUCCESS → NORMAL
                        ↘                  ↘
                       NORMAL            RECOVERY_FAILED → STOPPED
```

**Guard conditions** (recovery blocked if any are true):
- Localization confidence < 0.1
- Safety supervisor in `STOPPED` or `E_STOP`
- No planner command received
- Commanded speed effectively zero

**Recovery sequence:** Stop → Reverse → Rotate → Stop → Re-evaluate

The recovery node publishes only advisory `/recovery/cmd_vel_suggestion`.
`safety_supervisor` remains the sole `/cmd_vel` publisher.

| Parameter | Default | Description |
|---|---|---|
| `stuck_timeout_sec` | 3.0 s | Duration before `STUCK_SUSPECTED` |
| `stuck_suspect_timeout_sec` | 2.0 s | Hysteresis to `RECOVERY_ACTIVE` |
| `recovery_reverse_speed` | 0.10 m/s | Reverse speed |
| `recovery_reverse_time_sec` | 2.0 s | Duration of reverse phase |
| `recovery_rotate_speed` | 0.3 r/s | Rotation speed |
| `recovery_rotate_time_sec` | 2.0 s | Rotation duration |
| `max_recovery_attempts` | 3 | Attempts before terminal `STOPPED` |
| `min_clearance_for_reverse` | 0.3 m | Minimum clearance to allow reverse |

---

## Startup Timing

Nodes launch in staggered tiers to prevent race conditions on `/clock` and TF:

| Timer | Tier | Nodes |
|---|---|---|
| T+0 s | Simulator | Gazebo headless/GUI |
| T+5 s | Bridges | `ros_gz_bridge`, `lidar_bridge`, optional TF bridge |
| T+6 s | Localization | `slam_toolbox`, EKF, static TF publishers, `slam_pose_adapter`, planner |
| T+7 s | Sensors | `lidar_processor`, sensor contract, terrain nodes, online mapper, `slam_activator` |
| T+8 s | Safety | `local_path_selector`, `safety_supervisor`, `stuck_recovery`, `degraded_autonomy_manager` |
| T+9 s | Mission | `frontier_explorer`, `mission_manager`, `mission_intelligence` |

`lidar_processor` holds scans until `odom→base_link` TF is confirmed present, preventing
SLAM's MessageFilter queue from filling before the EKF is ready.

---

## Tests

```bash
colcon test
colcon test-result --all --verbose
```

**Current results (2026-10-12):** 329 tests, 0 failures, 0 errors, 5 skipped.

The test suite covers:
- Stuck-detection FSM transitions and guard conditions
- Safety supervisor `safety_decision()` policy
- Frontier exploration algorithms (Dijkstra reachability, cluster selection, bootstrap)
- Goal path planner (terrain scoring, path-frame consistency)
- Local path selector (obstacle scoring, heading logic)
- Online mapper (ray casting, occupancy update)
- Sensor contract (LiDAR health, validity)
- Traversability and terrain geometry
- Lint (flake8, pep257) and package manifest checks

Live Gazebo end-to-end recovery testing (vehicle physically gets stuck and recovers)
requires a controlled scenario and is future work.

---

## Localization Scope

This is **odometry-assisted online LiDAR SLAM** — not visual odometry and not GPS-fused
navigation.

- `slam:=true` — `slam_toolbox` provides loop-closure-capable `map → odom` from 2D LiDAR.
- `fusion:=true` — `robot_localization` EKF fuses wheel odometry + IMU yaw for `odom → base_link`.
- `slam:=false` — Gazebo ground-truth odometry relayed directly; no loop closure.

`/localization/confidence` is a sensor-health signal, not a visual odometry accuracy estimate.

---

## Troubleshooting

### Clock stuck / nodes not starting
```bash
# Verify /clock is flowing
ros2 topic hz /clock
# If silent, the Gazebo bridge is not running. Check:
ros2 node list | grep bridge
```

### Map stays 7 × 7 and never grows
Cause: `minimum_travel_distance > 0` in `slam_params.yaml` prevents SLAM from
recording new keyframes while the robot is stationary — the full startup deadlock.

Fix already applied: `minimum_travel_distance: 0.0`, `minimum_travel_heading: 0.0`.
If you see a 7×7 map again, confirm these values in:
`src/sih26126_localization/config/slam_params.yaml`

### SLAM message filter drops
```
[slam_toolbox]: Message Filter dropping message: frame '...' ... queue is full
```
Reduce `throttle_scans` (currently `2` = process every other scan at 5 Hz).
Too many drops indicate WSL2 CPU pressure; try closing other heavy processes.

### Safety supervisor stays STOPPED
```bash
ros2 topic echo /safety/reason --once
ros2 topic echo /planner/navigation_state --once
ros2 topic echo /exploration/state --once
```
Common reasons and resolution:
- `NAVIGATION_NOT_ACTIVE` — frontier explorer has not issued a goal yet; wait for map to grow
- `WAITING_FOR_NAVIGATION_STATE` — local_path_selector not yet publishing; check T+8 s tier
- `STALE_LOCALIZATION` — EKF not publishing `/localization/pose`; check fusion health
- `LOW_LOCALIZATION_CONFIDENCE` — EKF failed; restart stack

### EKF "Failed to meet update rate"
Reduce EKF frequency: `frequency: 20.0` in `src/sih26126_localization/config/ekf_params.yaml`
(already applied). Under heavy WSL2 load, reduce to 15.0 if errors persist.

### Missing TF: "odom does not exist"
Static TF publishers start at T+6 s. If TF is missing after 15 s, check:
```bash
ros2 node list | grep tf_publisher
ros2 topic echo /tf_static --once
```

### DDS discovery issues (WSL2)
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
```

---

## Known Limitations

- **Dynamic obstacle scenario** is incomplete: no validated LiDAR detection of moving obstacle,
  no confirmed evasive response.
- **Live recovery scenario**: stuck detection FSM has deterministic unit tests, but
  an end-to-end Gazebo test where the UGV physically gets stuck and autonomously recovers
  has not been demonstrated.
- **Terrain confidence / temporal memory**: planned, not implemented.
- **Information-gain-aware frontier selection**: planned, not implemented.
- **Risk-aware global planning**: planned, not implemented.

See `docs/KNOWN_LIMITATIONS.md` for the full list.

---

## Documentation

| Document | Contents |
|---|---|
| `SLAM_LOCALIZATION_VALIDATION.md` | Full TF chain verification and autonomous run record |
| `docs/KNOWN_LIMITATIONS.md` | Detailed limitations per capability area |
| `COMPLETION_SUMMARY.md` | End-to-end integration summary |

---

## Future Work

1. Complete dynamic obstacle detection and evasive response
2. Terrain confidence and temporal map memory
3. Information-gain-weighted frontier selection
4. Risk-aware global path planning
5. Controlled Gazebo scenario for physical stuck-and-recover testing
6. Degraded-autonomy mode: RTB on persistent sensor failure
