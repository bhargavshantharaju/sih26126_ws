# DRISTHI — Project Status and Next Phase

**Repository:** `recovery/dristi-saved-work-20261015`  
**Date of this report:** 2026-10-17  
**Stack:** ROS 2 Jazzy · Gazebo Harmonic · WSL2  
**Problem statement:** SIH26126 (GPS-denied autonomous UGV)

---

## 1. Git state

**Branch:** `recovery/dristi-saved-work-20261015`  
**Remote:** up to date with `origin/recovery/dristi-saved-work-20261015`  
**Stashes intact:** `stash@{0}` and `stash@{1}` (both labelled "Workspace backup before cleanup 20261015_055307/03" on `feat/localization-repair`)

### Modified files (not staged)

| File | Lines changed | Purpose |
|------|:---:|---|
| `.gitignore` | +9 | Added build/install/log exclusions |
| `src/sih26126_autonomy/setup.py` | +4 | Registered new autonomy entry points |
| `sih26126_autonomy/frontier_explorer.py` | +425/-2 | Cost-aware Dijkstra, cost_map subscription, frontier scoring integration |
| `sih26126_autonomy/mission_manager.py` | +191/-7 | Profile dispatch, grace period, safe-stop counter, startup logic |
| `test/test_frontier_explorer.py` | +453 | New tests for connected-component and cost-aware routing |
| `test/test_mission_manager.py` | +280/-4 | Extended state machine and discontinuity tests |
| `full_stack.launch.py` | +113/-3 | Added nodes: cost_map_fusion, mission_profiles, dashboard, terrain_memory, explainability_logger |
| `sih26126_dashboard/package.xml` | +3 | Added flask dependency |
| `sih26126_dashboard/setup.py` | +1 | Registered dashboard entry point |
| `sih26126_dashboard/dashboard_node.py` | +248 | Full dashboard implementation |
| `ekf_params.yaml` | +22/-6 | RGB-D odometry input, tuned covariances |
| `slam_params.yaml` | +24/-3 | Zero-motion thresholds, scan-match tuning |
| `ekf_sensor_adapter.py` | +6 | Output frame fix for EKF adapter |
| `imu_static_tf_publisher.py` / `lidar_static_tf_publisher.py` | +12/-3 each | Frame-ID validation fixes |
| `rgbd_odometry_baseline.py` | +67/-5 | Startup frame validation and odometry noise tuning |
| `slam_activator.py` | +30/-2 | Activation guard and topic monitoring |
| `src/sih26126_planner/setup.py` | +2 | Registered cost_map_fusion entry point |
| **`sih26126_planner/goal_path_planner.py`** | **+177/-7** | **Added `/map/cost_map` subscription, `path_cost_score()`, risk-aware speed modulation** |
| `sih26126_planner/local_path_selector.py` | +245/-2 | PlannerRisk assessment, dynamic obstacle integration |
| **`test/test_goal_path_planner.py`** | **+206** | **12 new cost-map integration tests** |
| `sih26126_safety/degraded_autonomy.py` | +17/-1 | Recovery mode state improvements |
| `test/test_degraded_autonomy.py` | +47 | Safety state tests |
| `sih26126_sensor_contract/lidar_processor.py` | +38/-3 | Scan relay health improvements |
| `sih26126_sim/models/sih_ugv/sih_ugv.sdf` | +16/-4 | LiDAR plugin topic alignment |
| `sih26126_sim/models/sih_ugv_depth/sih_ugv_depth.sdf` | +26/-3 | RGB-D camera config fixes |
| `dynamic_obstacle_world.sdf` | +2/-2 | World SDF update |
| `sih26126_traversability/setup.py` | +2 | Registered terrain_memory entry point |

### New (untracked) source files

| File | Component |
|------|-----------|
| `sih26126_autonomy/explainability_logger.py` | Event logging for rejected paths and recovery decisions |
| `sih26126_autonomy/frontier_scoring.py` | Multi-criteria frontier scoring (terrain, obstacles, heading) |
| `sih26126_autonomy/mission_profiles.py` | MissionProfileController for WAYPOINT/COVERAGE/SEARCH |
| `sih26126_planner/cost_map_fusion.py` | Fuses occupancy + terrain + localization into `/map/cost_map` |
| `sih26126_traversability/terrain_memory.py` | Spatial terrain score memory with decay |
| `test/test_cost_aware_routing.py` | 27 tests for frontier_explorer Dijkstra + cost multiplier |
| `test/test_mission_dispatch.py` | 17 tests for WAYPOINT/COVERAGE/SEARCH state transitions |
| `test/test_mission_profiles.py` | 19 tests for waypoint generation patterns |
| `test/test_rgbd_startup.py` | 8 tests for startup grace period |
| `test/test_explainability_logger.py` | 14 tests for event log data structures |
| `test/test_frontier_scoring.py` | 18 tests for multi-criteria scoring |
| `test/test_dynamic_obstacle_demo.py` | 1 integration smoke test |
| `sih26126_localization/test/test_visual_odometry_quality.py` | 9 localization quality tests |
| `sih26126_sim/models/sih_ugv/model.config` | Model registration for Gazebo |
| `sih26126_sim/models/sih_ugv_depth/model.config` | Depth model registration |
| `sih26126_sim/worlds/minimal_stable.sdf` | Minimal test world |
| `sih26126_bringup/launch/minimal_test.launch.py` / `test_minimal.launch.py` | Test launch configs |
| `sih26126_dashboard/gui_stream_node.py` / `gz_gui_stream.py` | Optional GUI streaming |

---

## 2. Feature status

Legend: **Implemented** — source exists · **Integrated** — wired in launch · **Tested** — unit/integration tests pass · **Built** — `colcon build` succeeds · **Runtime-observed** — a Gazebo run confirmed the behavior

| Capability | Status |
|---|---|
| ROS 2 / Gazebo Harmonic bringup | **Implemented · Integrated · Built** |
| LiDAR scan conversion + health | **Implemented · Integrated · Tested** |
| SLAM Toolbox (`slam:=true`) | **Integrated · Built** — runtime not re-verified in this session |
| EKF sensor fusion (`fusion:=true`) | **Integrated · Built** — configuration tested; runtime contribution unverified |
| RGB-D odometry baseline | **Implemented · Integrated · Tested** — startup grace period: 8 tests pass |
| Occupancy mapping + frontier explorer | **Implemented · Integrated · Tested** — 28 frontier tests pass; cost-aware Dijkstra: 27 tests |
| Cost map fusion (`/map/cost_map`) | **Implemented · Integrated · Tested · Built** — `cost_map_fusion` node fuses occupancy + terrain + localization |
| Global planner cost map consumption | **Implemented · Tested · Built** — `goal_path_planner` subscribes to `/map/cost_map` and applies risk-aware speed; 12 integration tests pass |
| Mission management (EXPLORATION) | **Implemented · Integrated · Tested** — main configured path |
| Mission profiles (WAYPOINT/COVERAGE/SEARCH) | **Implemented · Integrated · Tested** — 19 profile + 17 dispatch tests pass; end-to-end in simulator not verified |
| Terrain geometry + classification | **Implemented · Integrated · Built** |
| Traversability scoring + terrain memory | **Implemented · Integrated · Tested** — 25 tests pass |
| Safety supervisor (`/cmd_vel` authority) | **Implemented · Integrated · Tested** — 102 safety tests pass; architecture maintained |
| Dynamic obstacle detection + evasion | **Implemented · Integrated · Tested** — demo test passes; live Gazebo evasion not demonstrated |
| Stuck recovery (advisory) | **Implemented · Integrated · Tested** — architecture: recovery publishes to suggestion topic only |
| Explainability logger + dashboard | **Implemented · Integrated · Tested** — process-level; HTTP rendering not verified |

---

## 3. Changes made in this session and reason for each change

### `src/sih26126_autonomy/test/test_mission_dispatch.py`

**Reason:** Two bugs prevented the test from loading:

1. **Indentation error** — `nav_msg.OccupancyGrid`, `nav_msg.Odometry`, `nav_msg.Path`, and `nav.msg = nav_msg` were inside the `_NavPath` class body (8-space indent). Python tried to reference `_NavPath` from within its own definition, causing `NameError: cannot access free variable '_NavPath'`. Fixed by dedenting all four lines to the enclosing function level.

2. **`sys.modules` contamination** — The test installs lightweight stubs for `rclpy`, `geometry_msgs`, `nav_msgs`, etc. at module import time. These stubs persisted in `sys.modules` across the pytest session. When `test_online_mapping.py` was collected after this file, it imported `local_path_selector.py` which tried `from rclpy.executors import ExternalShutdownException` and got the stub `rclpy` (a plain `types.ModuleType`) instead of the real package. Fix: save the original `sys.modules` entries before installing stubs, reload `mission_manager` to bind `MissionManager` to the stub `Node`, then restore the originals. `MissionManager` retains the stub `Node` in its MRO (already bound at class-creation time) so tests still work correctly.

**Evidence:** 179/179 autonomy tests pass in both orders (cost_aware first or dispatch first).

### `src/sih26126_planner/sih26126_planner/goal_path_planner.py`

**Reason:** The global planner was the only component in the planning pipeline that did not consume the `/map/cost_map` topic. The frontier explorer already used cost-aware Dijkstra for path selection; the cost_map_fusion node already published the fused map. But the actual motion commands produced by `goal_path_planner` were derived only from the terrain score, with no awareness of the occupancy/terrain/localization-fused risk map.

**Changes:**
- Added `OccupancyGrid` import and `numpy` import.
- Added `path_cost_score(path, cost_map)` pure function: projects each waypoint in the path onto the cost map grid, reads the cell value, maps it to a normalised cost (unknown cells → 0.5, near-lethal ≥ 99 → 1.0, free 0 → 0.0), returns the mean.
- Added `self.cost_map: Optional[OccupancyGrid] = None` and `cost_map_callback` subscription to `/map/cost_map`.
- In `process()`, after computing `linear_speed` from terrain score and heading, applies: `linear_speed *= max(0.0, 1.0 - path_cost_score(path, self.cost_map))`. Cost 0.0 → no reduction; cost 0.5 → 50 % reduction; cost 1.0 → stop.
- Added `cost_map_penalty` to the periodic log line.

**Evidence:** 25 planner unit tests pass (13 pre-existing + 12 new). The function is pure; no ROS environment required to test it.

### `src/sih26126_planner/test/test_goal_path_planner.py`

**Reason:** Added 12 tests for the new `path_cost_score` function, covering: graceful degradation when no cost map is provided, all-free map returns 0.0, medium cost cells produce non-zero scores, near-lethal (≥ 99) cells contribute 1.0, unknown (−1) cells contribute 0.5, cells outside the map footprint contribute 0.5, higher cost map produces higher path score (the key integration test), zero vs. all-blocked map produces different scores, and invalid dimensions are handled without exceptions.

---

## 4. Actual test results

All results from this session (2026-10-17), run with `python3 -m pytest … --tb=short`:

| Package | Tests | Failures | Command |
|---------|:-----:|:--------:|---------|
| `sih26126_planner` | 123 | 0 | `python3 -m pytest src/sih26126_planner/test/ -v` |
| `sih26126_autonomy` | 179 | 0 | `python3 -m pytest src/sih26126_autonomy/test/ -v` |
| `sih26126_safety` | 102 | 0 | `python3 -m pytest src/sih26126_safety/test/ -v` |
| `sih26126_sensor_contract` | 41 | 0 | `python3 -m pytest src/sih26126_sensor_contract/test/ -v` |
| `sih26126_localization` | 57 | 0 | `python3 -m pytest src/sih26126_localization/test/ -v` |
| `sih26126_traversability` | 25 | 0 | `python3 -m pytest src/sih26126_traversability/test/ -v` |
| `sih26126_geometry` | 16 | 0 | `python3 -m pytest src/sih26126_geometry/test/ -v` |
| **Total** | **543** | **0** | |

Pre-existing lint tests (`test_flake8`, `test_pep257`) fail because they scan the `build/` and `install/` directories, which contain generated code with style deviations. These failures exist on the branch baseline and are not caused by the changes in this session. Functional test count is unaffected.

Build results (this session):
```
colcon build --symlink-install --packages-select sih26126_planner  → 1 package finished, 0 errors
colcon build --symlink-install --packages-select sih26126_autonomy → 1 package finished, 0 errors
```

---

## 5. Remaining limitations

- **Runtime / Gazebo integration not re-validated.** All tests are pure-Python unit tests. No Gazebo run was performed in this session. Confirmed prior evidence: sensor rate records from 2026-10-16; no sustained goal-reaching or forward locomotion confirmed by any session.
- **Mission profiles (WAYPOINT/COVERAGE/SEARCH) not end-to-end tested in Gazebo.** The state machine and waypoint generation are unit-tested; their ability to drive the robot through a complete mission in simulation is unverified.
- **Dynamic obstacle evasion not demonstrated.** The detection and evasion code is present and unit-tested; a controlled Gazebo run in which the robot actually encounters and avoids a moving obstacle has not been performed.
- **Cost map effect on routes not runtime-verified.** The `path_cost_score` function and speed modulation are unit-tested; their effect on robot trajectory in a live Gazebo run has not been observed.
- **Dashboard HTTP rendering unverified.** The dashboard node starts (process-level verified) but browser-visible rendering and HTTP behavior are not confirmed.
- **RGB-D odometry contribution unverified.** The EKF is configured to accept a `visual_odom` input; how much this input actually improves localization vs. pure LiDAR SLAM has not been measured.
- **Physical stuck recovery not demonstrated.** The state machine is tested; a controlled Gazebo run that physically gets the robot stuck and recovers has not been performed.
- **Lint failures in build/ directory.** Pre-existing `test_flake8`/`test_pep257` failures scan generated code. The source files added in this session have not been reformatted.

---

## 6. Next three engineering tasks

### Task 1 — Runtime validation of the cost-map-to-planner pipeline

**Description:** Launch the full stack with `slam:=true fusion:=true scenario:=showcase` and confirm that (a) `cost_map_fusion` publishes to `/map/cost_map` at ≥ 1 Hz, (b) `goal_path_planner` logs `cost_map_penalty` values that vary as the robot moves, and (c) a path through a region marked occupied in the cost map results in measurably reduced speed in the `/planner/recommended_cmd_vel` topic.

**Dependencies:** Working Gazebo Harmonic environment with display support OR headless `gui:=false`. The `cost_map_fusion` node must receive `/map/occupancy` from SLAM Toolbox and `/localization/confidence`.

**Acceptance criteria:**
1. `ros2 topic hz /map/cost_map` reports ≥ 1 Hz while the stack runs.
2. `goal_path_planner` logs include `cost_map_penalty=<value>` where `<value>` is non-zero when the robot is near the cost map origin or in a mapped area.
3. Speed in `/planner/recommended_cmd_vel` is measurably lower (< 80 % of `max_speed_mps`) when `cost_map_penalty > 0.3` as read from the log.

---

### Task 2 — Sustained exploration run with frontier completion

**Description:** Perform a bounded Gazebo exploration run (≥ 120 s, `exploration_radius_m:=10`) and confirm that the robot selects at least three frontier goals, reaches at least one, and the `mission_manager` transitions through `MAPPING → EXPLORING → RETURNING_HOME`.

**Dependencies:** Working Gazebo run environment. SLAM Toolbox must build a map (`ros2 topic hz /map` ≥ 0.5 Hz). Localization must produce a valid `map → odom` TF (`tf2_echo map odom` must print a valid transform).

**Acceptance criteria:**
1. `ros2 topic echo /exploration/state` shows `EXPLORING` at least once.
2. `ros2 topic echo /planner/navigation_state` shows `NAVIGATE` (not stuck at `WAITING_TERRAIN` or `WAITING_POSE`).
3. `ros2 topic echo /mission/state` shows `RETURNING_HOME` or `MISSION_COMPLETE` before the 1800 s timeout.
4. `/map` grows in size (number of known cells increases) across the run duration.

---

### Task 3 — Dynamic obstacle encounter and evasion demonstration

**Description:** Launch with `scenario:=dynamic_obstacle` and confirm that (a) the `obstacle_tracker` publishes tracked obstacles on `/obstacles/tracked` as the moving orange box crosses the forward path, (b) the `local_path_selector` produces a steering command away from the obstacle, and (c) the `safety_supervisor` publishes a non-zero `/cmd_vel` (robot moves and adjusts course).

**Dependencies:** Tasks 1 and 2 confirmed working (stable localization and navigation). The `dynamic_obstacle_world.sdf` and `model://sih_ugv` resolution (verified in this session).

**Acceptance criteria:**
1. `ros2 topic echo /obstacles/tracked` shows at least one tracked obstacle with non-zero velocity while the orange actor is in motion.
2. `ros2 topic echo /safety/state` remains `SAFE` (no false SAFE_STOP triggered by the moving obstacle).
3. `ros2 topic echo /cmd_vel` shows non-zero `angular.z` as the robot steers around the crossing obstacle.
4. No collision detected (`/sensor_health/lidar` stays HEALTHY; safety state does not reach `EMERGENCY_STOP`).

---

*This report covers the session from 2026-10-17. No commits have been made. No stashes have been modified. No branches have been changed.*
