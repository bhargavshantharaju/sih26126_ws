# DRISTHI Final Validation Follow-Up Report

**Date:** October 23, 2026, 20:18–20:22 UTC (session continuation)
**Branch:** `recovery/dristi-saved-work-20261015`
**Stashes preserved:** `stash@{0}: Workspace backup before cleanup 20261015_055307`, `stash@{1}: Workspace backup before cleanup 20261015_055303`
**Evidence directory:** `clean_acceptance_20261023T201848/`
**Previous evidence:** `clean_acceptance_20261023T193407/` (preserved, not modified)

---

## Verdict: ACCEPTANCE INCOMPLETE

The system demonstrated continuous, safety-supervised autonomous navigation for 120 wall-clock seconds (57.4 sim seconds), with the safety supervisor maintaining SAFE/SAFE_COMMAND throughout and verified robot motion of 1.82 m (Gazebo ground truth). However, three critical gaps remain:

1. **Sim time only 57.4 seconds** — falls short of the 60-second threshold if measured in sim time.
2. **Autonomy was in DEGRADED\_SENSOR mode** (lidar\_unhealthy) for ~99 of 120 seconds — the navigation was degraded, not fully healthy.
3. **EKF odometry diverges from Gazebo ground truth** — EKF net displacement 0.19 m vs Gazebo 1.00 m, indicating localization quality concerns.

---

## 1. Process Contamination (Confirmed)

The earlier test (`acceptance_logs_20261023T191941/`) was contaminated by two complete DRISTHI stacks running simultaneously (63 orphaned processes). This is confirmed by `PROCESS_CONTAMINATION_DISCOVERY.md` and the process-tree evidence. The clean test at 19:34 operated with a single stack, verified by the process tree in `ros_output.log` showing only one set of PIDs.

**Effect:** The contaminated test's results are invalid. The clean test's results are the basis for this validation. However, the absence of contamination does not by itself validate every subsystem — the subsystems must be independently verified.

---

## 2. Previous Clean-Run Evidence Audit (19:34 Test)

I independently inspected all artifacts in `clean_acceptance_20261023T193407/` against the `CLEAN_TEST_REPORT_20261023T193407.md` claims. Findings:

| Claim in report | Actual evidence | Verdict |
|---|---|---|
| 90 seconds healthy navigation | Sim clock advanced 12.5→68.0s = **55.5s sim time**; wall-clock ~141s | **Overstated** — sim time < 60s |
| Fusion HEALTHY throughout | 24 HEALTHY, 2 DEGRADED (odom=False), 1 FAILED | **False** — not "throughout" |
| Safety SAFE throughout | `safety_timeline.txt` is garbled: mostly `timeout` entries with interleaved state/reason | **Not verified** — collector was broken |
| Zero pose discontinuities | 0 matches in log (correct thresholds) | **Confirmed** |
| 17 reachable frontiers | `final_frontier_count.txt`: data: 17 | **Confirmed** (single snapshot) |
| Distance 4.03 m | `final_metrics.txt` truncated; Gazebo odometry shows max ~0.6 m displacement | **Not verified** — metric overstates actual motion |
| 90-second runtime | Wall-clock observation was 90s, sim time 55.5s | **Misleading** — wall ≠ sim time |

**Evidence-collection defects found:**
- `safety_timeline.txt`: Two independent `timeout 2 ros2 topic echo --once` calls for `/safety/state` and `/safety/reason` produced garbled interleaving (e.g., `state=timeout reason= SAFE_COMMAND`). Most samples timed out.
- `mission_state_timeline.txt`: `--once` on an event-driven topic mostly timed out (only 3 real samples).
- `final_metrics.txt`: `ros2 topic echo` truncated the long JSON string field (`...exploration_comple...'`).
- No odometry, TF, sensor-rate, or `/cmd_vel` publisher evidence was collected.
- Shutdown missed `async_slam_toolbox_node` (C++ binary, not matched by `python3` filter) — PID 13374 survived and was SIGKILLed manually.

---

## 3. Fixes Applied

### 3a. Threshold correction (mission\_manager.py)

**Change:** `pose_jump_margin_m` 0.20 → 0.10 (restored to original baseline in HEAD).

**Rationale:** The robot's physical max speed is 1.5 m/s (from `sih_ugv.sdf`: `max_linear_velocity`). At the observed 0.03–0.05 m/s navigation speed, a 0.20 m margin represents 4–6.7 seconds of motion — not a noise floor. The original 0.10 m (3.3 s of motion) was already generous. `max_pose_speed_mps=2.5` (1.67× physical max) is retained — it has a defensible physical basis.

**Verification:** 18 mission\_manager tests + 8 rgbd\_startup tests passed. 0 pose discontinuities in the new acceptance run.

### 3b. Evidence collector (evidence\_collector.py — new file)

Replaced the broken `ros2 topic echo --once` polling with a Python `rclpy` subscriber node that atomically captures all critical topics:
- `/clock` (51,827 samples, monotonicity checked)
- `/mission/state` (114 samples — no timeouts)
- `/mission/metrics` (full JSON, not truncated — 114 messages)
- `/safety/state` + `/safety/reason` (694 atomic pairs — no garbling)
- `/fusion/state` (282 samples)
- `/model/sih_ugv/odometry` (1,652 Gazebo ground-truth samples, cumulative path + net displacement computed)
- `/odometry/filtered` (1,138 EKF samples)
- `/localization/confidence` (567 samples)
- `/lidar/points` + `/lidar/scan_2d` (rate measurement)
- `/cmd_vel` (message count + publisher verification via graph API)
- TF `map→odom→base_link` (60 checks)

### 3c. Test script (run\_clean\_acceptance\_test.sh — rewritten)

- Uses `setsid` to launch in a new process group for reliable child tracking.
- Preflight checks for slam\_toolbox orphans (was missing).
- Shutdown escalates SIGINT → SIGTERM → SIGKILL on the process **group**, catching slam\_toolbox, gz, and all ROS nodes.
- Records process trees before/after shutdown.
- No slam\_toolbox orphan in the new run (confirmed: `process_tree_post_shutdown.txt` = "none").

### 3d. Emergency cleanup (emergency\_cleanup.sh — rewritten)

- Removed hardcoded parent PID 367 (would not work in any other session).
- Identifies orphans by checking if parent has exited (PPID=1 or parent PID not running).
- Added slam\_toolbox to the scan patterns.

---

## 4. New Acceptance Run Evidence (20:18–20:22 UTC)

**Launch:** `ros2 launch sih26126_bringup autonomous_mission.launch.py gui:=false start_dashboard:=false` (default world: `autonomous_forest.sdf`)

### CONFIRMED

| Criterion | Evidence | Value |
|---|---|---|
| `/clock` advancing | 51,827 samples, 0 non-monotonic | Sim 10.19→67.60s (57.41s) |
| Gazebo and bridges healthy | Processes running throughout, TF available | 60/60 checks |
| `/lidar/points` publishing | 561 messages, 4.4 Hz (last 10s) | Active |
| `/lidar/scan_2d` publishing | 574 messages, 4.3 Hz (last 10s) | Active |
| TF map→odom→base\_link | 60/60 checks passed | Resolved throughout |
| EKF odometry updating | 1,138 samples | Continuous |
| Fusion health | 282 samples: 281 HEALTHY, 1 DEGRADED | ~99.6% healthy |
| SLAM consuming scans | 4 queue-full drops (1 during startup) | Improved from 14 |
| Map updating | known\_area\_m2 = 50.81, 20,325 cells | Growing |
| Reachable frontiers | 19 at test end, goal at (4.11, -2.49) | Valid |
| Mission state coherent | 114 samples, all EXPLORING | No transitions, no failures |
| Safety supervisor state | 694 samples, all SAFE/SAFE\_COMMAND | Valid throughout |
| `/cmd_vel` sole publisher | 1 publisher (safety\_supervisor), 24 checks | Confirmed |
| Autonomous motion | Gazebo ground truth: 1.82 m path, 1.00 m net displacement | Independently verified |
| Pose discontinuities | 0 with corrected threshold (2.5 m/s, 0.10 m) | None |
| Safety stops | 0 | None |
| Localization confidence | 567 samples, all 1.0 | Stable |
| Shutdown: no orphans | `process_tree_post_shutdown.txt` = "none" | All terminated |

### NOT VERIFIED

| Item | Status | Detail |
|---|---|---|
| Dashboard health | Not tested | `start_dashboard:=false` — no dashboard node launched |
| Collision detection | Not measured | `collision_detected: null` in metrics — sensor not active |
| Contact sensor stream | Not available | `contact_sensor_stream_seen: false` |
| Browser-rendered dashboard | Not tested | No dashboard was running |
| Autonomy mode (topic) | Not captured | `/autonomy/mode` uses custom `AutonomyMode` msg type; collector subscribed as `String` — no data received |

### FAILED / CONCERNS

| Issue | Evidence | Impact |
|---|---|---|
| Autonomy DEGRADED\_SENSOR | `lidar_unhealthy` for ~99/120s; local\_path\_selector logged `mode=DEGRADED_SENSOR` throughout | Navigation was degraded, not fully healthy |
| Sim time < 60s | 57.41s sim time in 120.07s wall time (RTF ~0.48) | Falls short of 60s if sim-time is the metric |
| EKF divergence | EKF net displacement 0.19 m vs Gazebo 1.00 m; EKF path 2.64 m vs Gazebo 1.82 m | Localization quality concern |
| 0 frontiers completed | `frontiers_completed: 0` in metrics | No goals reached |
| Distance metric overstates | `distance_travelled_m=4.11` vs Gazebo 1.82 m (2.25×) | SLAM pose accumulation includes jitter/corrections |
| Unclean shutdown | 20 Python nodes exit code 1 (ExternalShutdownException), gz sim needed SIGKILL | Not graceful |

---

## 5. Odometry Validation

### Gazebo Ground Truth (authoritative)
- **Source:** `/model/sih_ugv/odometry` (Gazebo bridge, 1,652 samples)
- **Cumulative path length:** 1.82 m
- **Net displacement:** 1.00 m (from (0.034, 0.016) to (0.804, 0.649))
- **Yaw change:** 0.607 → -1.385 rad (114° rotation)

### EKF Filtered Odometry
- **Source:** `/odometry/filtered` (robot\_localization EKF, 1,138 samples)
- **Cumulative path length:** 2.64 m
- **Net displacement:** 0.19 m (from (0.293, -0.015) to (0.443, -0.127))

### Mission Manager Metric
- **Source:** `/mission/metrics` `distance_travelled_m` (SLAM pose accumulation)
- **Value:** 4.11 m
- **Assessment:** Overstates actual motion by 2.25× vs Gazebo ground truth. The SLAM pose accumulates per-sample deltas including SLAM corrections and pose jitter, inflating the cumulative path beyond the physical motion. This is cumulative path length, not net displacement.

**Limitation:** The EKF and SLAM poses diverge from the Gazebo ground truth, suggesting the localization chain (SLAM map→odom + EKF odom→base\_link) is not accurately tracking the robot's true position. The Gazebo ground truth is the only authoritative measurement.

---

## 6. Safety Threshold Audit

| Parameter | Original (HEAD) | Previous session | Current (this session) | Justification |
|---|---|---|---|---|
| `max_pose_speed_mps` | 2.0 | 5.0 → 2.5 | **2.5** | 1.67× robot's 1.5 m/s physical max — defensible |
| `pose_jump_margin_m` | 0.10 | 0.50 → 0.20 | **0.10** (restored) | 0.20 represented 4–6.7s of motion — not a noise floor |
| `pose_discontinuity_safe_stop_count` | (new) | 3 | **3** | Requires 3 consecutive before SAFE\_STOP |
| `pose_startup_grace_sec` | (new) | 15.0 | **15.0** | Suppresses frame-change faults during init |

The safety supervisor (`safety_supervisor.py`) does NOT implement pose-discontinuity detection — that logic lives in `mission_manager.py`. The safety supervisor IS the sole `/cmd_vel` publisher (confirmed: 1 publisher in 24 checks). It enforces command clamping (max\_linear=0.30, max\_angular=0.60), obstacle-score gating, localization-age timeout, and confidence threshold.

---

## 7. Shutdown Lifecycle

### Previous run (19:34)
- 27 processes died: 20 Python (exit code 1, ExternalShutdownException), 5 C++ (exit code -9, SIGKILL including slam\_toolbox PID 13374)
- SLAM Toolbox survived as orphan — manually SIGKILLed

### New run (20:18)
- 20 Python nodes exit code 1 (ExternalShutdownException during shutdown — same rclpy issue)
- 14 nodes finished cleanly
- gz sim survived SIGINT and SIGTERM, required SIGKILL (process group)
- **No orphans:** `process_tree_post_shutdown.txt` = "none" — all test processes terminated

**Root cause of Python crashes:** `rclpy.spin()` raises `ExternalShutdownException` when the context is shut down externally (by launch SIGINT). This is a known rclpy behavior, not a DRISTHI bug. The `except (KeyboardInterrupt, ExternalShutdownException): pass` handler in most nodes catches it, but some nodes also call `rclpy.shutdown()` in their `finally` block, which raises a secondary `RCLError: rcl_shutdown already called`.

**Root cause of gz sim not responding to SIGINT:** Gazebo's `gz sim -s` (server-only mode) does not handle SIGINT in all cases. The process-group SIGKILL is the appropriate last resort.

---

## 8. Tests, Build, and Lint

### Build
All 12 packages built successfully: `colcon build --symlink-install` — 0 errors, 12 finished.

### Functional tests (excluding lint)

| Package | Passed | Skipped | Deselected (lint) |
|---|---|---|---|
| sih26126\_autonomy | 193 | 0 | — |
| sih26126\_safety | 107 | 1 | 3 |
| sih26126\_planner | 134 | 0 | 3 |
| sih26126\_localization | 66 | 0 | 3 |
| sih26126\_sensor\_contract | 48 | 0 | — |
| sih26126\_traversability | 25 | 0 | 3 |
| sih26126\_dashboard | 6 | 0 | — |
| sih26126\_bringup (launch regression) | 8 | 0 | — |
| **Total** | **587** | **1** | **12** |

**Focused regression tests:**
- Mission manager threshold: 18 passed
- RGBD startup: 8 passed
- SLAM transform timeout: 9 passed

### Lint (run separately, not suppressed)

| Package | flake8 | pep257 | Error count |
|---|---|---|---|
| sih26126\_safety | FAILED | FAILED | 339 (D213, D403, etc.) |
| sih26126\_localization | FAILED | FAILED | 1826 flake8 errors |

**Assessment:** All lint failures are pre-existing — they span files not modified in this session (e.g., `lidar_monitor.py`, `obstacle_extractor.py`, `scan_relay.py`). The threshold change (`pose_jump_margin_m` 0.20→0.10) is a single numeric value and introduces no lint issues. The safety package's pep257 errors include D213 violations in `test_safety_supervisor.py` which was modified by the previous session, not this one.

---

## 9. World Validation

| World | Exists | Valid XML | Launch support |
|---|---|---|---|
| `autonomous_forest.sdf` | ✓ (22,900 bytes) | ✓ | Default world for `autonomous_mission.launch.py` |
| `showcase_terrain.sdf` | ✓ (27,883 bytes) | ✓ | Default for `full_stack.launch.py` |
| `forest_hills.sdf` | ✓ (11,686 bytes) | ✓ | Available via `world:=` argument |
| `minimal_stable.sdf` | ✓ (6,493 bytes) | ✓ | Available via `world:=` argument |

No concurrent world testing was performed. The acceptance run used the default `autonomous_forest.sdf`.

---

## 10. Dashboard

Not tested. The acceptance run launched with `start_dashboard:=false`. The dashboard node (`dashboard_node.py`) was not running. Dashboard health verification requires a separate run with `start_dashboard:=true` and either browser-rendered or API-endpoint verification.

---

## 11. Remaining Defects and Next Steps

### Defects
1. **Lidar health:** The sensor contract flags the lidar as unhealthy (4.4 Hz vs expected rate), keeping autonomy in DEGRADED\_SENSOR. Investigate the lidar publishing rate and the sensor-contract threshold.
2. **EKF localization divergence:** EKF net displacement (0.19 m) vs Gazebo (1.00 m) — the EKF is not accurately tracking motion. Investigate EKF parameters, sensor inputs, and TF chain.
3. **Distance metric inflation:** `distance_travelled_m` (4.11 m) overstates actual motion (1.82 m Gazebo) by 2.25× due to SLAM pose jitter accumulation. Consider using Gazebo ground-truth odometry or filtering the SLAM pose before accumulating.
4. **Sim-time RTF:** Gazebo runs at ~0.48× real-time, causing sim time to lag wall time. Longer wall-clock observation or performance optimization is needed to achieve 60s sim time.
5. **Shutdown cleanliness:** Python nodes crash with `ExternalShutdownException` during shutdown. While not a DRISTHI bug (it's an rclpy behavior), the secondary `RCLError` from calling `rclpy.shutdown()` in `finally` blocks should be guarded.
6. **Lint failures:** 339 (safety) + 1826 (localization) pre-existing style violations. Not introduced by this session's changes.

### Next steps
1. Investigate lidar publishing rate (4.4 Hz) vs sensor-contract expected rate — fix the root cause of DEGRADED\_SENSOR.
2. Investigate EKF odometry divergence from Gazebo ground truth — verify EKF input sensors and parameters.
3. Run a longer acceptance test (≥180s wall) to achieve ≥60s sim time, or optimize Gazebo RTF.
4. Run a dashboard-enabled acceptance test and verify dashboard API endpoints.
5. Fix pre-existing lint violations (D213, D403) in safety and localization packages.
6. Guard `rclpy.shutdown()` calls in `finally` blocks with `if rclpy.ok()` checks to eliminate secondary RCLErrors.

---

## 12. Modified Files

| File | Change | Why |
|---|---|---|
| `src/sih26126_autonomy/sih26126_autonomy/mission_manager.py` | `pose_jump_margin_m` 0.20→0.10 | Restored to original baseline; 0.20 was not a defensible noise floor |
| `run_clean_acceptance_test.sh` | Rewritten | Fix evidence collection, add slam\_toolbox cleanup, use process groups |
| `emergency_cleanup.sh` | Rewritten | Remove hardcoded PID 367, add slam\_toolbox, use orphan detection by parent status |
| `evidence_collector.py` | New file | Comprehensive timestamped ROS 2 subscriber replacing broken `ros2 topic echo` polling |

---

**Report generated:** October 23, 2026, 20:25 UTC
**Evidence directory:** `clean_acceptance_20261023T201848/`
**All previous reports and the continuation checkpoint are preserved and unmodified.**
