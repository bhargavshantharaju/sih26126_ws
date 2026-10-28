# DRISTHI Phase 2 — Integration Checkpoint

**Date:** 2026-10-16T14:00:00Z
**Branch:** recovery/dristi-saved-work-20261015
**Status:** Phase 2 integration and smoke-test COMPLETE

---

## What Was Checked

### Repository State
- Branch: `recovery/dristi-saved-work-20261015` ✓
- Stashes: `stash@{0}` and `stash@{1}` both preserved ✓
- No commits made, no stashes dropped ✓
- 27 tracked modified files, 52 untracked files (including report documents)

### Phase 1 Claims vs Reality

| Feature | Files Exist | Entry Point | In Launch | Topic Active | Scoring/Logic Wired |
|---------|------------|-------------|-----------|--------------|----------------------|
| Terrain Memory | ✓ | ✓ | ✗→✓ | ✓ (post-fix) | Partial — score published, grid not spatial |
| Frontier Scoring | ✓ | N/A (lib) | N/A | N/A | ✗→✓ (post-fix) |
| Cost Map Fusion | ✓ | ✓ | ✗→✓ | Publishes when map available | Partial — planner does not consume |
| Mission Profiles | ✓ | ✓ | ✗→✓ | ✓ (post-fix) | Partial — mission_manager doesn't dispatch |
| Explainability Logger | ✓ | ✓ | ✗→✓ | ✓ (post-fix) | ✓ — dual-publisher bug fixed |
| Dynamic Obstacle Avoidance | ✓ | ✓ | ✓ (pre-existing) | ✓ | ✓ — local_path_selector uses /obstacles/tracked |
| RGB-D Visual Localization | ✓ | ✓ | ✓ (pre-existing) | ✓ | ✓ — EKF wired in ekf_params.yaml as odom1 |
| Advanced Perception | Doc only | N/A | N/A | N/A | Infrastructure only; no models |

---

## Integration Defects Found and Fixed

### 1. All 4 New Nodes Missing from Launch File
**Defect:** `terrain_memory`, `cost_map_fusion`, `mission_profiles`, and
`explainability_logger` had entry points but were never launched.
**Fix:** Added all 4 nodes to `full_stack.launch.py` — `terrain_memory` and
`cost_map_fusion` at T+8s, `mission_profiles` and `explainability_logger` at T+9s.

### 2. Frontier Scoring Not Used in Frontier Selection
**Defect:** `frontier_scoring.py` existed as a library but `frontier_explorer.py`
never imported or called it. `select_frontier()` used only `gain / (distance + heading + 1.0)`.
**Fix:** Added `/terrain/memory_score` and `/obstacles/tracked` subscriptions to
`FrontierExplorer`. Updated `select_frontier()` signature to accept
`terrain_score`, `obstacle_positions`, `map_origin_x`, `map_origin_y`.
Now calls `score_frontier_with_terrain_and_obstacles()` from `frontier_scoring`
for each candidate. Added `terrain_score_to_cost_scalar()` helper to `frontier_scoring.py`.
Both `select_frontier` call sites in `process()` updated.

### 3. Dual-Publisher Bug in ExplainabilityLogger
**Defect:** `rejection_pub` and `recovery_outcome_pub` both pointed to
`/mission/events` — two `create_publisher` calls on the same topic from the
same node causes ROS 2 resource management issues.
**Fix:** Merged into a single `mission_event_pub`. Both `log_path_rejection()`
and `log_recovery_outcome()` now use `mission_event_pub`.

### 4. Trivial Enum Tests (Test Quality)
**Defect:** `test_mission_profiles.py` used `assert MissionType.EXPLORATION` —
truthy check on a non-None enum member, always passes regardless of value.
Same pattern in `test_explainability_logger.py`.
**Fix:** Replaced with `assert MissionType.EXPLORATION.value == "EXPLORATION"` etc.
All enum existence tests now validate the actual string value.

---

## Remaining Integration Gaps (Require Larger Changes)

### cost_map_fusion: Published but Not Consumed
- `cost_map_fusion` publishes `/map/cost_map` (OccupancyGrid with fused costs)
- `goal_path_planner` routes on the `global_path` from `frontier_explorer`
  (Dijkstra over raw occupancy only) — does not subscribe to `/map/cost_map`
- `local_path_selector` makes reactive steering decisions from LiDAR obstacles
  and terrain score — does not read `/map/cost_map`
- **To fully fix**: `goal_path_planner` or `frontier_explorer` would need to
  run Dijkstra over the cost map rather than the raw occupancy grid — a
  significant rewrite of the path planning loop.
- **Current status**: `cost_map_fusion` is in the live graph, computes and
  publishes the map, and the topic is available for dashboard display and
  future planner integration.

### mission_profiles: Launched but mission_manager Does Not Dispatch
- `MissionProfileController` receives home pose, selects EXPLORATION mode,
  publishes `/mission/profile_state`, `/mission/total_waypoints`,
  `/mission/waypoint_index`.
- For EXPLORATION mode it correctly defers to `frontier_explorer` (correct behavior).
- `mission_manager` has no `mission_type` parameter and does not subscribe to
  `/mission/profile_state` — the two nodes run in parallel but are not
  tightly coupled for WAYPOINT/COVERAGE/SEARCH modes.
- **To fully fix**: `mission_manager` would need a `mission_type` parameter and
  code to forward goals from `MissionProfileController` to the planner.
- **Current status**: EXPLORATION mode works end-to-end. Other modes generate
  waypoints but are not consumed by the mission manager.

---

## Actual Files Modified

```
src/sih26126_autonomy/sih26126_autonomy/frontier_explorer.py
src/sih26126_autonomy/sih26126_autonomy/frontier_scoring.py
src/sih26126_autonomy/sih26126_autonomy/explainability_logger.py
src/sih26126_bringup/launch/full_stack.launch.py
src/sih26126_autonomy/test/test_mission_profiles.py
src/sih26126_autonomy/test/test_explainability_logger.py
```

---

## Test Results

### Focused Tests (Changed Paths)
```
120 passed in 0.85s
```
- `test_terrain_memory.py`:        18/18 ✓
- `test_cost_map_fusion.py`:       22/22 ✓
- `test_frontier_scoring.py`:      18/18 ✓
- `test_mission_profiles.py`:      19/19 ✓ (fixed enum tests)
- `test_explainability_logger.py`: 15/15 ✓ (fixed enum tests, dual-publisher)
- `test_frontier_explorer.py`:     28/28 ✓ (enhanced scoring does not break existing tests)

### Package Builds
- Affected packages (5): `sih26126_msgs`, `sih26126_traversability`, `sih26126_planner`,
  `sih26126_autonomy`, `sih26126_bringup` — **SUCCESS (13.8s, 0 errors)**
- Full workspace (12): All packages — **SUCCESS (12.1s, 0 errors)**

---

## Smoke Test Results

### Baseline Scenario (`slam:=false fusion:=false`)
- **Result: PASS** — all 29 nodes started cleanly, no errors
- All 4 new nodes confirmed in live graph: terrain_memory ✓, cost_map_fusion ✓,
  mission_profiles ✓, explainability_logger ✓
- Active topics confirmed: `/terrain/memory_score`, `/mission/events`,
  `/mission/decision_rationale`, `/mission/profile_state`, `/mission/total_waypoints`
- Frontier explorer selected first frontier (2.65, -1.25) at T+13s distance=2.84m
- goal_path_planner: terrain_score=0.600, state=NAVIGATE, speed=0.101 m/s
- obstacle_extractor: 5 clusters from 3976 pts (dynamic obstacle pipeline active)
- terrain_classifier: FLAT class_confidence=0.988
- WARN: `lidar_unhealthy` triggered at startup (known latency issue, non-blocking)

### Dynamic Obstacle Scenario (`scenario:=dynamic_obstacle`)
- **Result: PARTIAL PASS**
- objective4_obstacle_controller, obstacle_extractor, obstacle_tracker started ✓
- objective4_pose_service_bridge established ✓
- Gazebo mesh error: `__default__` mesh not found (visual model missing in SDF)
- Physics ran; obstacle pipeline initialized correctly

### RGB-D Fusion Scenario (`slam:=true fusion:=true`)
- **Result: PARTIAL PASS — node graph correct, known startup blocker**
- All fusion nodes started and initialized ✓
- slam_toolbox reached ACTIVE state via slam_activator ✓
- fusion_health_monitor: HEALTHY (imu=True odom=True ekf=True) ✓
- rgbd_odometry_baseline: pair=10–110, features=99, inliers=64, reproj=0.45 ✓
- /fusion/visual_odometry confirmed publishing (x=-0.003, y=-0.004) ✓
- /odometry/filtered EKF output confirmed ✓
- **Blocker**: mission_manager SAFE_STOP from FRAME_CHANGED on `visual_odom` frame
  at startup before SLAM map frame settles. Robot stationary, delta≈0.001m.
  This is pre-existing behavior; system correctly rejects bad frames.

---

## Architecture Constraints Verified

| Constraint | Status |
|-----------|--------|
| SLAM Toolbox owns `map → odom` | ✓ verified in launch config |
| EKF owns `odom → base_link` | ✓ `publish_tf: true` in ekf_params.yaml |
| Safety supervisor is sole `/cmd_vel` publisher | ✓ confirmed — no other node publishes cmd_vel |
| Ground truth not used in autonomy pipeline | ✓ confirmed — Gazebo TF bridge disabled in SLAM/fusion modes |

---

## Next Task to Resume

**Phase 3 starting point:**
1. Fix RGB-D fusion startup-sequence SAFE_STOP: mission_manager's `FRAME_CHANGED` check
   is too strict for the first few seconds of visual odometry initialization.
   Options: (a) add startup grace period to pose discontinuity check,
   (b) delay visual_odom publication until SLAM map frame is established.
2. Wire `cost_map_fusion` output into `frontier_explorer`'s Dijkstra search:
   run shortest paths over `/map/cost_map` rather than raw occupancy for
   risk-aware global routing.
3. Integrate `mission_manager` with `MissionProfileController` for WAYPOINT/
   COVERAGE/SEARCH modes: add `mission_type` parameter and goal-forwarding logic.
4. Fix `dynamic_obstacle_world.sdf` mesh reference (`__default__`) so the
   visual obstacle model loads correctly.
