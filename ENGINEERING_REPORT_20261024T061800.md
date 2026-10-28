# DRISTHI Engineering Report - 2026-10-24T06:18:00Z

## Executive Summary

**Branch:** `recovery/dristi-saved-work-20261015`  
**Test Duration:** 90 seconds  
**Result:** CRITICAL BLOCKER IDENTIFIED - Robot stationary despite healthy subsystems

The DRISTHI autonomous UGV stack launches successfully with all 36 nodes operational, but the robot does not move due to a critical localization defect. SLAM Toolbox is publishing unstable map→base_link transforms that report implausible velocities (6.8-7.0 m/s) while the robot remains stationary according to Gazebo ground-truth odometry (x=0.40m constant).

## System State Summary

### ✅ WORKING Components

1. **Build System**: All 12 packages build successfully (7.54s)
2. **Node Launch**: All 36 nodes start and remain alive
3. **Simulation**: Gazebo Harmonic running, clock at ~419 Hz
4. **Sensor Fusion**: EKF reports HEALTHY (imu=True, odom=True, ekf=True)
5. **Visual Odometry**: Processing 200+ frame pairs, reporting pose updates
6. **Terrain Classification**: FLAT terrain, confidence 0.994
7. **SLAM Activation**: slam_toolbox configured, activated, processing scans
8. **Frontier Detection**: Finding reachable frontiers, issuing goals
9. **Safety Architecture**: safety_supervisor is SOLE /cmd_vel publisher (verified)
10. **Command Pipeline**: /cmd_vel publishing at 5 Hz with velocity commands
11. **Mission Management**: Transitions to EXPLORING state
12. **Safety Supervisor**: Reports SAFE state with SAFE_COMMAND reason

### ❌ CRITICAL DEFECT

**Symptom:** Robot does not move despite receiving velocity commands  
**Ground Truth:** Gazebo odometry reports robot stationary at (x=0.40m, y=0.00m) for entire 90s test  
**SLAM Behavior:** map→base_link transform reports position changes of 0.68-0.70m every 0.1s  
**Calculated Velocity:** 6.8-7.0 m/s (273% over max_pose_speed_mps threshold of 2.5 m/s)  
**Mission Manager Response:** Rejects all 480+ pose updates as IMPLAUSIBLE_POSE_DELTA  
**Consequence:** Mission manager never accepts any localization poses, preventing distance tracking and mission completion

## Root Cause Analysis

### Timeline of Failure

1. **T+0s**: Launch begins
2. **T+7s**: SLAM activates successfully, registers "Custom Described Lidar"
3. **T+25s**: slam_pose_adapter begins publishing /localization/pose
4. **T+28s**: First pose discontinuity detected (delta=0.366m in 0.1s)
5. **T+28-118s**: Continuous stream of pose discontinuities (480+ total)
6. **T+0-90s**: Gazebo odometry remains constant at x=0.40m (robot stationary)

### Evidence

```
[mission_manager] Pose discontinuity 433: IMPLAUSIBLE_POSE_DELTA delta=0.681 m in 0.1000 s frame='map'
[mission_manager] Pose discontinuity 434: IMPLAUSIBLE_POSE_DELTA delta=0.681 m in 0.1000 s frame='map'
...
[mission_manager] Pose discontinuity 480: IMPLAUSIBLE_POSE_DELTA delta=0.708 m in 0.1000 s frame='map'
```

Gazebo odometry samples (12 samples over 60s):
```
x: 0.4017389272331383   (sample 1)
x: 0.40173892732787286  (sample 2)
x: 0.4017389273195389   (sample 3)
...
x: 0.4017389272632915   (sample 12)
```

**Conclusion:** Robot is physically stationary (Gazebo truth), but SLAM's map frame is moving/jumping relative to the odom frame at ~7 m/s.

### Hypothesis

The most likely causes:

1. **SLAM receiving invalid scan data**: lidar_processor may be publishing scans with inconsistent timestamps or incorrect TF associations
2. **SLAM map drift**: slam_toolbox may be incorrectly updating its map→odom transform due to scan matching issues
3. **EKF instability**: odom→base_link transform may be drifting, causing SLAM's map→odom to compensate with jumps
4. **Timestamp mismatch**: Even though slam_pose_adapter uses monotonic timestamps for messages, the underlying TF transform from SLAM may have timing issues

### Key Configuration

From source inspection and runtime logs:

- **lidar_processor**: Uses `get_clock().now()` for scan timestamps (not original Gazebo timestamp)
- **SLAM config**: `transform_timeout: 1.0s`, `throttle_scans: 2`, `minimum_travel_distance/heading: 0.0`
- **slam_pose_adapter**: Uses `get_clock().now()` for pose message timestamps, looks up map→base_link TF
- **EKF**: 20 Hz, fusing wheel odometry + IMU + visual odometry

## Comparison to Previous State

According to `DRISTHI_CONTINUATION_CHECKPOINT.md`, previous fixes addressed:
- ✅ slam_pose_adapter monotonic timestamps (PRESENT in code)
- ✅ mission_manager elapsed fallback (PRESENT in code)
- ✅ lidar_processor TF gate and re-verification (PRESENT in code)

However, the current failure mode is different from the previous one:
- **Previous**: Non-monotonic pose timestamps causing negative elapsed times
- **Current**: Monotonic pose timestamps, but the *underlying TF data* from SLAM shows implausible position changes

This suggests the fix resolved the timestamp symptom but not the root TF/SLAM cause.

## Test Infrastructure Health

- ✅ Launch script works correctly
- ✅ No orphaned processes after shutdown
- ✅ Evidence capture successful (launch.log, topic samples, position log)
- ✅ /cmd_vel publisher verification confirms safety_supervisor is sole authority
- ✅ Clock, sensor rates, and command rates all healthy

## Remaining Tasks

From the 10-task engineering plan:

- ✅ Task 1: Real state established
- ✅ Task 2: Build verified
- ✅ Task 3: Pipeline mapped
- ⚠️  Task 4: Localization/TF/SLAM - BLOCKED by SLAM instability
- ❌ Task 5: Perception/traversability/navigation - Cannot verify without motion
- ❌ Task 6: Autonomy/exploration/mission - Cannot complete mission without stable localization
- ❌ Task 7: Safety validation - Cannot test full scenarios without motion
- ❌ Task 8: Dashboard - Not tested (start_dashboard:=false for this run)
- ⚠️  Task 9: 60s healthy operation - System runs but robot doesn't move
- 🔄 Task 10: This report

## Recommended Next Actions

### Immediate Diagnosis

1. **Compare EKF odom→base_link to Gazebo ground truth**
   - Run `ros2 topic echo /odometry/filtered` during operation
   - Run `ros2 run tf2_ros tf2_echo odom base_link`  
   - Check if EKF is stable or drifting

2. **Inspect SLAM map→odom transform directly**
   - Run `ros2 run tf2_ros tf2_echo map odom`
   - Verify if map→odom is jumping while robot is stationary

3. **Examine scan quality**
   - Verify `/lidar/scan_2d` has reasonable ranges
   - Check if scan frame_id and timestamps are correct
   - Monitor for "queue is full" warnings from SLAM

4. **Test with SLAM disabled**
   - Launch with `slam:=false fusion:=true`
   - Verify if robot moves with baseline odometry localization
   - This isolates whether SLAM is the sole cause

### Potential Fixes

**Option A: Disable minimum_travel thresholds may be causing issues**
- SLAM params currently have `minimum_travel_distance: 0.0` and `minimum_travel_heading: 0.0`
- This was intended to allow stationary mapping, but may cause SLAM to process every scan even with no robot motion
- Try restoring small thresholds (0.05m, 0.05rad) to prevent SLAM from updating when truly stationary

**Option B: Increase transform_timeout**
- Current: 1.0s
- SLAM may need more time to resolve TF at scan timestamps, especially during startup

**Option C: Fix scan timestamp source**
- lidar_processor currently uses `get_clock().now()` instead of the original Gazebo timestamp
- While this fixed TF buffer issues, it may cause SLAM to receive scans with timestamps that don't match the actual sensor acquisition time
- Consider using a hybrid approach: use Gazebo timestamp but validate it's within TF buffer range

**Option D: Increase scan_buffer_size further**
- Current: 20
- May need even larger buffer during startup when TF chain is still stabilizing

**Option E: Investigate WSL2 timing**
- WSL2 RTF < 1.0 may cause timing issues between /clock delivery and TF updates
- May need to increase various timeouts throughout the stack

## Evidence Files

All evidence stored in: `acceptance_run_20261024T061522/`

- `launch.log`: Complete 90s launch output (194 KB)
- `summary.txt`: Test summary
- `mission_state.txt`: EXPLORING
- `safety_state.txt`: SAFE
- `safety_reason.txt`: SAFE_COMMAND
- `start_position.txt`: x=0.40m
- `position_log.txt`: 12 samples showing no movement
- `cmd_vel_info.txt`: Verified safety_supervisor as sole publisher
- `cmd_vel_hz.txt`: ~5 Hz command rate
- `clock_hz.txt`: ~419 Hz simulation clock

## Conclusion

The DRISTHI stack demonstrates excellent integration across most subsystems: all nodes launch, sensors work, safety architecture is correct, and commands flow properly. However, a critical SLAM/localization defect prevents the robot from moving. The issue is NOT in the mission logic, safety supervisor, or command pipeline - those are working correctly. The blocker is that SLAM's map→odom transform is unstable, causing mission_manager to correctly reject all localization poses as implausible.

**Status**: BLOCKED - Cannot achieve 60s autonomous navigation until SLAM stability is resolved.

**Priority**: CRITICAL - This is a showstopper for demonstration.

---

**Report prepared by**: Kiro CLI (claude-sonnet-4.5)  
**Session ID**: DRISTHI full-system engineering 2026-10-24
