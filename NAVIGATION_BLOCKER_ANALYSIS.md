# NAVIGATION BLOCKER ROOT CAUSE ANALYSIS

## EXECUTIVE SUMMARY

**STATUS: PARTIAL — Robot rotates but does not translate**

The robot IS physically moving (rotation confirmed), but navigation is blocked by **SYSTEM OVERLOAD** causing:
1. Simulation running at only 61.5% real-time
2. SLAM dropping messages due to queue overflow
3. Erratic goal angle calculations (oscillating ±160-180 degrees)
4. Rotation rate only 15% of commanded speed
5. Robot stuck in perpetual rotate-in-place mode

## VERIFIED MOTION EVIDENCE

### Measurement Period: 30 Wall Seconds (18.45 Sim Seconds)

**T0 (wall time 1791260632):**
- Position: x=0.4126, y=-0.0175
- Yaw: 0.00 deg
- Commanded: angular.z = 0.45 rad/s

**T1 (30 seconds later):**
- Position: x=0.4126, y=-0.0175 (NO CHANGE)
- Yaw: -73.73 deg (**73.7 degrees rotation**)
- Commanded: angular.z = 0.45 rad/s

###Analysis:
- ✅ **Translation**: 0.00 mm (NONE)
- ✅ **Rotation**: 73.7 deg measured
- ⚠️  **Rotation rate**: 4.0 deg/s (sim time) vs 25.8 deg/s commanded
- **Efficiency**: 15% of commanded rate

## ROOT CAUSE: SYSTEM OVERLOAD

### Evidence Chain

#### 1. Sim Time Running Slow
```
Measurement over 12 wall seconds:
  Δ wall time: 12.00 s
  Δ sim time: 7.38 s
  Ratio: 0.615 (61.5% real-time)
```

**This is the PRIMARY indicator of system overload.**

#### 2. SLAM Message Drops
```log
[slam_toolbox]: Message Filter dropping message: frame 'sih_ugv/base_link/lidar_3d' 
   at time 217.800 for reason 'discarding message because the queue is full'

[slam_toolbox]: Message Filter dropping message: frame 'sih_ugv/base_link/lidar_3d' 
   at time 276.600 for reason 'the timestamp on the message is earlier than all 
   the data in the transform cache'
```

**Consequence**: SLAM cannot maintain consistent state, TF may be unreliable.

#### 3. EKF Update Rate Failure
```log
[ekf_filter_node]: Failed to meet update rate! Took 0.041 seconds. 
   Try decreasing the rate, limiting sensor output frequency, or limiting the 
   number of sensors.
```

**Consequence**: Localization fusion cannot keep up with configured 30 Hz rate.

#### 4. Erratic Goal Angle Oscillation
```
Recent goal angles from logs:
goal=-155.2  goal=163.9  goal=171.5  goal=179.6  goal=178.6  
goal=-175.0  goal=-166.7 goal=-163.5 goal=-160.2 goal=178.2  
goal=-173.4  goal=-168.4 goal=-152.8 goal=158.7  goal=-179.6
```

The goal angle jumps between +160 to +180 and -160 to -180, indicating:
- Robot is rotating near the 180-degree wrap point
- OR goal/pose frames are mismatched
- OR localization pose updates are inconsistent

#### 5. No Forward Motion Command
```
Local planner logs consistently show:
  speed=0.000
```

**Reason**: `waypoint_command()` only permits forward motion when `abs(goal_angle) <= 30 deg`.
Current goal angle is 160-180 deg, so robot remains in rotate-in-place mode.

## SECONDARY FINDINGS

### 1. DEGRADED_SENSOR Mode Active
```log
mode=DEGRADED_SENSOR
```

This may be reducing commanded velocities or affecting planning, but is NOT the primary blocker.

### 2. Slow Physical Rotation
Even accounting for 61.5% sim speed, the robot rotates at only 4 deg/s (sim time) vs 25.8 deg/s commanded.

**Possible causes**:
- High wheel friction/damping in SDF
- Insufficient motor torque
- Mass/inertia too high
- Joint limits

**However**, this is a PERFORMANCE issue, not a BLOCKER. If the goal angle were reasonable, the robot would eventually rotate to face it.

### 3. Frame Configuration Appears Correct
- SLAM configured with: map_frame=map, odom_frame=odom, base_frame=base_link
- SLAM lifecycle state: active (id=3)
- SLAM mode: mapping
- Local planner subscribes to `/localization/pose` (published at ~6 Hz)
- Global path frame_id: map

### 4. Goal Selection
- Current pose: (0.41, 0.01)
- Goal from path: (1.93, -0.65)
- Distance: ~1.66 m
- World bearing: should be ahead/right, not behind

**The 160-180 deg goal angle does NOT match the geometric relationship between current pose and goal.**

This indicates:
- Pose used for goal angle calculation is wrong (wrong frame or stale)
- OR yaw value is inverted/incorrect
- OR goal coordinates are in a different frame than pose

## THE ACTUAL BLOCKER

**The robot cannot navigate because**:

1. **System overload** causes sim to run at 61.5% speed
2. **SLAM drops messages** and cannot maintain reliable TF
3. **Goal angle calculation** produces nonsensical 160-180 deg bearing
4. **Waypoint command logic** correctly refuses to move forward when goal is "behind"
5. **Robot rotates slowly** in place, never reducing goal angle below 30 deg threshold

**The rotate-in-place behavior is CORRECT given the perceived goal angle.**

**The goal angle calculation is WRONG, likely due to frame mismatch or stale pose.**

## NOT THE BLOCKER

- ❌ cmd_vel not reaching Gazebo (it is reaching it)
- ❌ Safety supervisor blocking motion (it passes commands through)
- ❌ No rotation at all (rotation is occurring)
- ❌ SLAM not active (it is active, but overloaded)
- ❌ Missing TF publishers (TF is being published at 42 Hz)

## FIXES REQUIRED

### Priority 1: Reduce System Load

**The system cannot maintain real-time performance.**

Options:
1. **Reduce sensor rates**
   - Depth camera: 30 Hz → 10 Hz
   - LiDAR: no change (already ~5.6 Hz)
   - IMU: 100 Hz → 50 Hz

2. **Reduce planner rates**
   - Local planner: 5 Hz → 2 Hz
   - Frontier explorer: ? Hz → 1 Hz

3. **Simplify world**
   - Reduce terrain complexity
   - Remove unnecessary visual elements
   - Use simpler collision geometry

4. **Increase EKF sensor_timeout** (already done: 0.1 → 0.5)
   - But this must match actual sensor timing

5. **Reduce SLAM update rate or resolution**

### Priority 2: Fix Goal Angle Calculation

**Diagnose frame mismatch in local_path_selector:**

1. Log all inputs to goal_angle_from_pose():
   - pose_x, pose_y (value and frame)
   - yaw (value and source)
   - goal_x, goal_y (value and frame)
   - computed world_angle
   - computed goal_angle

2. Verify `effective_pose` selection logic:
   - Is `use_localization_pose` true?
   - Is `/localization/pose` being received?
   - Is frame matching working correctly?
   - Are coordinates actually in the same frame?

3. Check if yaw has correct sign/convention

4. Verify quaternion_to_yaw() implementation

### Priority 3: Rotation Performance

**After** system load and goal angle are fixed, investigate why rotation is only 15% of commanded rate:
- Check SDF wheel/joint damping
- Check mass/inertia
- Check motor torque limits
- Consider PID tuning for diff_drive controller

## LEGITIMATE VS QUESTIONABLE CHANGES

### LEGITIMATE
1. ✅ slam_activator retry logic (handles clock initialization)
2. ✅ local_path_selector map-frame pose handling (fixes frame mismatch)
3. ✅ local_path_selector frame_id parameter (ensures consistency)

### NEEDS VERIFICATION
1. ⚠️  ekf sensor_timeout 0.1 → 0.5
   - **Measured sensor rates needed**
   - May be masking real timing issue OR may be correct

2. ⚠️  local_path_selector x_min 0.20 → 0.45
   - **LiDAR geometry analysis needed**
   - May be ignoring real obstacles OR may be filtering self-hits

3. ⚠️  mission_manager frontier_exhaustion_grace_sec 3.0 → 15.0
   - **Frontier detection rate measurement needed**
   - May be hiding exploration failure OR may be accommodating real dynamics

## NEXT STEPS

1. **Profile the system** to identify CPU bottleneck:
   - Is it Gazebo physics?
   - Is it SLAM processing?
   - Is it sensor data serialization/bridging?

2. **Reduce sensor/planner rates** systematically until sim achieves >90% real-time

3. **Add detailed logging** to local_path_selector goal angle calculation

4. **Run minimal test**:
   - Disable exploration
   - Send manual goal AHEAD of robot
   - Verify goal angle is computed correctly
   - Verify forward motion occurs

5. **Only after motion works**, re-enable full autonomy

## ACCEPTANCE CRITERIA REMINDERS

- ❌ Two autonomous navigation cycles: NOT ACHIEVED
- ⏸️  Robot physically moves: PARTIALLY (rotation only)
- ❌ Translation occurs: NOT ACHIEVED
- ❌ Map grows from exploration: NOT MEASURED
- ❌ Second frontier selected: NOT ACHIEVED

**VERDICT: PARTIAL**

**EXACT REMAINING BLOCKER:**

**System overload (sim 61.5% real-time) → SLAM drops messages → unreliable pose/TF → incorrect goal angle (160-180 deg) → waypoint_command correctly refuses forward motion → robot rotates slowly in place forever**

**The navigation logic is working correctly. The input data is wrong due to system overload.**
