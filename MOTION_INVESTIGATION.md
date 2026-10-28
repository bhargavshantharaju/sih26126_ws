# MOTION INVESTIGATION REPORT

## CURRENT LEGITIMATE FIXES

1. **slam_activator.py retry logic**: Increased timeouts and retries to handle sim_time clock initialization race
   - `service_timeout_sec`: 30.0 → 60.0
   - `call_timeout_sec`: 10.0 → 15.0
   - `retry_interval_sec`: 1.0 → 2.0
   - `max_retries`: 10 → 30
   - Added retry loop for initial state read
   - **Assessment**: LEGITIMATE - addresses documented sim_time clock race

2. **local_path_selector.py map-frame pose handling**: Added `/localization/pose` subscription and frame matching
   - Added localization pose callback
   - Switched to map-frame pose when path_frame == localization_frame == 'map'
   - **Assessment**: LEGITIMATE - fixes frame mismatch between SLAM map-frame path and odom-frame odometry

3. **local_path_selector.py path building frame_id**: Made frame_id parameter explicit
   - Path now built in the same frame as effective_pose
   - **Assessment**: LEGITIMATE - ensures frame consistency

## CURRENT QUESTIONABLE CHANGES

1. **ekf_params.yaml sensor_timeout**: 0.1 → 0.5
   - **Question**: Is this masking a real sensor timing issue or is it matching actual sensor rates?
   - **Action Required**: Measure actual `/imu` and `/model/sih_ugv/odometry` rates

2. **local_path_selector.py x_min**: 0.20 → 0.45
   - **Question**: Are we ignoring real near-field obstacles or filtering genuine chassis/ground returns?
   - **Documented Rationale**: "scan frequently hits the robot's own chassis or the ground plane"
   - **Action Required**: Verify LiDAR mounting geometry and determine if 0.20-0.44m returns are genuinely self-hits

3. **mission_manager.py frontier_exhaustion_grace_sec**: 3.0 → 15.0
   - **Question**: Is this hiding an inability to find frontiers or accommodating genuine exploration dynamics?
   - **Action Required**: Verify frontier detection rate during actual exploration

## CURRENT RUNTIME BLOCKER

**UNKNOWN: Robot commanded to rotate (angular=0.45) but unknown if physical motion occurs.**

Previous session reported:
- pose around (0.42, -0.06)
- goal distance around 4.08 m
- local angle around 28 deg
- goal angle around 169 deg (GOAL IS BEHIND ROBOT)
- obstacle around 3.00 m
- obstacle score around 0.92
- safety passing angular command around 0.45 rad/s
- linear command 0.000

After ~60 seconds, pose was still approximately unchanged.

## HYPOTHESES TO TEST

### Hypothesis A: Robot is rotating but not translating
- Planner logic keeps robot in rotate-in-place mode
- `waypoint_command()` threshold prevents forward motion
- Yaw should change in odometry

### Hypothesis B: Robot is not physically responding to cmd_vel at all
- Command reaches safety_supervisor
- But Gazebo or controller does not respond
- Neither translation nor rotation occurs

### Hypothesis C: Goal angle calculation error
- Goal appears to be at 169 deg (almost directly behind)
- Maximum steering angle is 28 deg
- Robot must rotate in place to face goal
- But rotation may not be occurring

## INVESTIGATION PLAN

1. Launch stack
2. Simultaneously monitor:
   - `/cmd_vel` (commanded motion)
   - `/odom` or `/model/sih_ugv/odometry` (actual motion)
   - `/odometry/filtered` (EKF fusion output)
3. Record timestamps and values at T0 and T1 (30 seconds apart)
4. Calculate:
   - Δx, Δy (translation)
   - Δyaw (rotation)
   - Commanded linear/angular
5. Determine which case applies
