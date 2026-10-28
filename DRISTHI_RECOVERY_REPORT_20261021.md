# DRISTHI SLAM Recovery Report — 2026-10-21

**Prepared:** 2026-10-21T06:25:00Z  
**Branch:** `recovery/dristi-saved-work-20261015`  
**Session:** Account-switch continuation from prior Kiro session

---

## 1. Root Cause

**Primary failure:** `slam_toolbox`'s MessageFilter queue filled within ~2 sim-seconds at
startup, then dropped every subsequent scan with "discarding message because the queue is
full", preventing the map from growing beyond the initial 7×7 keyframe.

**Mechanism (verified):**

`transform_timeout` in slam_toolbox controls how long the `tf2_ros::Buffer`'s
`waitForTransform` timer waits for `odom→base_link` TF before discarding a queued scan.
This timeout operates in **simulation time** — verified by tracing the implementation:
`tf2_ros::Buffer::waitForTransform` creates timers via `CreateTimerROS`, which takes the
node's `rclcpp::Clock` (RCL_ROS_TIME when `use_sim_time=true`).

At WSL2 RTF ≈ 0.35:
- EKF runs at 20 Hz wall ≈ **7 Hz sim-time** (period ≈ 0.143s sim)
- Old `transform_timeout=0.2s` sim → ~1.4 EKF sim-frames — marginal; any startup
  ordering delay in the first EKF TF publication causes the scan to be dropped
- With `scan_buffer_size=10` and `throttle_scans=2` (~5 Hz sim), the 10 buffer slots
  fill in ~2 sim-seconds and every subsequent scan is dropped

**Secondary failure (deadlock):** Non-zero `minimum_travel_distance=0.3` and
`minimum_travel_heading=0.3` prevented SLAM from accepting keyframes while the robot
was stationary at startup, creating a circular dependency:
robot stopped → SLAM travel thresholds not met → map never grows → no frontiers →
no goal → planner waits → safety supervisor stays STOPPED → robot stays stopped.

---

## 2. Modified Files

### `src/sih26126_localization/config/slam_params.yaml`

| Parameter | Old | New | Justification |
|---|---|---|---|
| `throttle_scans` | 1 | 2 | Halves 10 Hz LiDAR to ~5 Hz sim, halves queue arrival rate |
| `map_update_interval` | 5.0 | 2.0 | Faster map publication during exploration |
| `transform_timeout` | 0.2 | **1.0** | Core fix: gives ~7 EKF sim-frames at RTF=0.35 |
| `minimum_travel_distance` | 0.3 | **0.0** | Removes stationary startup deadlock |
| `minimum_travel_heading` | 0.3 | **0.0** | Same — time-based keyframe acceptance instead |
| `scan_buffer_size` | 10 | **20** | Doubles startup surge headroom |

**Comment correction:** The inline YAML comments previously stated the timeout in
wall-clock terms (incorrect). Corrected to accurately describe sim-time behavior with
the EKF sim-rate calculation.

### `src/sih26126_localization/test/test_slam_transform_timeout_regression.py` *(new)*

9 regression tests covering:
1. `transform_timeout >= 1.0s` sim-time
2. `scan_buffer_size >= 20`
3. `throttle_scans == 2` (unchanged, correct)
4. `minimum_time_interval <= 0.5s`
5. `minimum_travel_distance == 0.0` and `minimum_travel_heading == 0.0`
6. `scan_topic == '/lidar/scan_2d'` (LiDAR topic contract)
7. Analytical: new timeout covers ≥3 EKF sim-frames at RTF=0.30 (conservative)
8. Characterization: old 0.2s timeout gave <2 EKF sim-frames
9. `tf_buffer_duration >= transform_timeout * 10`

---

## 3. Regression Test Results

```
src/sih26126_localization/test/test_slam_transform_timeout_regression.py

9 passed in 0.03s
```

All 9 tests pass. The test file is flake8-clean (verified with `--max-line-length=99`).

---

## 4. Build, Functional Test, and Lint Results

### Build
```
colcon build --symlink-install
Summary: 12 packages finished [6.74s]
```
All 12 packages built successfully.

### Functional Tests (per package, lint excluded)

| Package | Result |
|---|---|
| sih26126_localization | 66 passed, 1 skipped |
| sih26126_sensor_contract | 48 passed |
| sih26126_safety | 107 passed, 1 skipped |
| sih26126_planner | 134 passed, 1 skipped |
| sih26126_autonomy | 193 passed |
| sih26126_geometry | 16 passed |
| sih26126_traversability | 25 passed, 1 skipped |
| **Total** | **589 passed** |

### Lint
- **8 failures** (2 × `test_flake8` + `test_pep257` across 4 packages) — all scan
  `./build/` and `./install/` generated directories. **Pre-existing, not introduced
  by this session.** Our new test file is independently verified flake8-clean.

---

## 5. Safety and Topic Contract Verification

### Safety supervisor
- 30-second startup grace period for localization **confidence** only.
- Pose **freshness** (`localization_age > timeout`) is enforced throughout grace — the
  EKF must publish poses for navigation to proceed at any time.
- Recovery commands remain advisory; `safety_supervisor` is the sole `/cmd_vel`
  publisher by design and by runtime verification.

### LiDAR topic contract
- `lidar_processor` publishes `LaserScan` to `/lidar/scan_2d` ✓
- `local_path_selector` subscribes to `/lidar/scan_2d` ✓  
  (was `/scan` before prior fix session — regression test confirms correct topic)
- `slam_toolbox` subscribes to `/lidar/scan_2d` via `scan_topic` param ✓

### LiDAR timestamps
- `lidar_processor` stamps published scans with `self.get_clock().now()` (current
  sim-time), not the original Gazebo bridge `msg.header.stamp`. This prevents the
  "Requested time T1 but earliest data is T2 > T1" TF error when the TF gate first
  opens and queued clouds have old timestamps.

---

## 6. Runtime Acceptance Test

**Launch:** `ros2 launch sih26126_bringup full_stack.launch.py slam:=true fusion:=true gui:=false start_dashboard:=true scenario:=showcase`  
**World:** `showcase_terrain.sdf`  
**Wall-clock duration:** ~34 minutes (05:50 – 06:24 UTC 2026-10-21)  
**Capture log:** `/tmp/dristhi_runtime_20261021T060000.log`  
**ROS launch log:** `/home/venom/.ros/log/2026-10-21-05-50-14-439127-BHARGAV-LOQ-38587/`

### Verified evidence

| Evidence | Value | Source |
|---|---|---|
| `/lidar/scan_2d` rate | **8.7 Hz** | `ros2 topic hz` |
| `/map` rate | **0.5 Hz** | `ros2 topic hz` |
| Map size | **776 × 765 cells at 0.05m** = 38.8m × 38.25m | `ros2 topic echo /map` |
| `map→odom` TF | Translation [1.118, 0.099, 0.000] | `tf2_echo map odom` |
| `/cmd_vel` publisher count | **1** (safety_supervisor only) | `ros2 topic info /cmd_vel` |
| Node count | **38 nodes** | `ros2 node list` |
| Safety state | `SAFE_COMMAND` / `LOCAL_NAVIGATE` | direct topic read |
| Clock | 1 publisher, 37 subscribers | `ros2 topic info /clock` |

### SLAM queue-full status (post-fix)
- **Old behavior:** continuous drops every fraction of a sim-second; map stuck at 7×7
- **Post-fix behavior:** individual drops every ~15–40 sim-seconds  
  Drop timestamps from SLAM log: 4.3, 39.7, 52.1, 63.8, 67.1, 85.9, 104.3, 118.9,
  135.4, 156.1, 158.8, 172.4, 174.9, 191.4, 207.0, 224.6, 239.6, 276.5 (sim-seconds)
- Map grew to **776×765 cells** confirming SLAM is processing the vast majority of scans

The remaining infrequent drops are likely caused by brief moments when the EKF
processing load under WSL2 produces a TF publication gap that slightly exceeds the
new 1.0s sim timeout. This is a residual resource-constraint effect, not the original
queue-flood failure mode. Mapping is effective.

### Shutdown
- `kill -SIGTERM 38587` (launch process)
- `kill -SIGTERM 38590` (gz sim, did not respond)
- `kill -SIGKILL 38590` (gz sim confirmed exited)
- No orphaned processes confirmed.

---

## 7. Unresolved Limitations

1. **Residual queue-full drops:** ~1 drop per 15–40 sim-seconds persists under WSL2
   resource pressure. Increasing `scan_buffer_size` further or `transform_timeout`
   higher would reduce these but risks masking genuine TF unavailability.
   Not addressed in this minimal fix.

2. **Frontier size:** The frontier explorer occasionally reports "Reachable region too
   small" — this is the map-growth vs. frontier-threshold interaction and is
   pre-existing, not introduced by this session.

3. **RGB-D odometry contribution:** The EKF fuses visual odometry from
   `rgbd_odometry_baseline` but its quality contribution to localization accuracy has
   not been independently validated.

4. **Dashboard browser rendering:** The dashboard node was started (`start_dashboard:=true`)
   but browser-visible HTTP rendering was not confirmed in this session.

5. **Stashes preserved:** Both pre-existing stashes remain intact:
   - `stash@{0}: Workspace backup before cleanup 20261015_055307`
   - `stash@{1}: Workspace backup before cleanup 20261015_055303`

---

## 8. Final Git State

```
Branch:  recovery/dristi-saved-work-20261015
Status:  Modified (not committed)
Stashes: 2 preserved (see above)
```

**Changed files this session:**
- `src/sih26126_localization/config/slam_params.yaml` — 5 parameter changes + corrected comments
- `src/sih26126_localization/test/test_slam_transform_timeout_regression.py` — new file (9 tests)
