# Dynamic Obstacle Scenario Documentation

## Overview
The `dynamic_obstacle_demo` world provides a controlled environment for testing predictive dynamic obstacle avoidance.

## World Configuration
**File:** `src/sih26126_sim/worlds/dynamic_obstacle_demo.sdf`

### Entities
- **Robot:** UGV at origin (0, 0, 0)
- **Ground:** 30m x 20m flat terrain
- **Dynamic Obstacle:** 0.65m x 0.65m x 1.1m red box
  - Initial position: (3.2, -2.0, 0.55)
  - Kinematic: true (position controlled via Gazebo service)
  - Gravity: false

## Test Controller
**File:** `src/sih26126_autonomy/sih26126_autonomy/dynamic_obstacle_demo.py`

The `DynamicObstacleController` node moves the obstacle through a programmed trajectory:

```python
def obstacle_y(elapsed: float) -> float:
    """Obstacle crosses the UGV's path at ~y=0."""
    # Moves from y=-2.0 to y=+2.0 over 8 seconds
    # Crosses y=0 at t=4s
```

### Motion Profile
- Start: y = -2.0m
- End: y = +2.0m
- Duration: ~8 seconds
- Crossing point: y = 0 at t = 4s
- Average velocity: ~0.5 m/s in +y direction

## Expected Behaviors

### 1. Obstacle Detection
**Component:** `obstacle_extractor.py` + `obstacle_tracker.py`
- LiDAR detects obstacle as point cloud cluster
- Obstacle extractor publishes to `/obstacles/raw`
- Obstacle tracker:
  - Associates detections across frames
  - Estimates velocity via exponential smoothing (alpha=0.3)
  - Classifies as DYNAMIC when speed > 0.15 m/s
  - Publishes to `/obstacles/tracked` with:
    - `position`: current (x, y, z)
    - `velocity`: estimated (vx, vy, vz)
    - `is_dynamic`: true
    - `confidence`: increases with track age
    - `predicted_position`: position + velocity * predict_horizon

### 2. Collision Risk Prediction
**Component:** `local_path_selector.py::predict_trajectory_collision_risk()`
- Receives `/obstacles/tracked`
- For each dynamic obstacle (confidence > 0.3):
  - Projects robot position along candidate steering angle
  - Projects obstacle position using its velocity
  - Computes minimum distance over 2-second horizon (8 time steps)
  - Returns risk ∈ [0, 1]:
    - 0.0: distance > safe_distance (1.2m)
    - 1.0: distance ≤ critical_distance (0.6m)
    - Linear interpolation between

### 3. Trajectory Selection
**Component:** `local_path_selector.py::select_best_candidate()`
- Integrates collision risk into candidate scoring
- `obstacle_score = obstacle_score * (1.0 - collision_risk)`
- High collision risk significantly reduces candidate score
- Robot should prefer trajectories that avoid predicted collision

### 4. Safety Supervisor
**Component:** `safety_supervisor.py`
- Monitors `/obstacles/dynamic_risk` (max risk across all obstacles)
- Applies speed scaling when dynamic_risk > threshold
- Final authority on `/cmd_vel` publication

## Expected Outcomes

### Scenario A: Robot Stationary
- Obstacle moves through robot's vicinity
- Risk increases as obstacle approaches
- Robot remains stopped (no goal set)
- Tracker maintains velocity estimate ~0.5 m/s

### Scenario B: Robot Navigating Toward Goal
- Robot has goal at (5, 0, 0) requiring forward motion
- Obstacle crosses path at y=0
- **Expected behavior:**
  1. Robot moves forward initially (low risk)
  2. As obstacle approaches crossing point:
     - Collision risk increases
     - Straight-ahead trajectory penalized
     - Robot selects evasive trajectory (turn left/right) OR
     - Robot slows/stops if no safe trajectory exists
  3. After obstacle clears:
     - Risk decreases
     - Robot resumes goal-directed navigation

### Scenario C: Blocked Corridor
- Obstacle positioned to block all safe trajectories
- **Expected behavior:**
  - All candidates have high collision risk
  - Robot stops (safety supervisor blocks unsafe command)
  - State: `OBSTACLE_STOP` or similar

## Test Validation Criteria

✅ **Detection:** Obstacle appears in `/obstacles/tracked` within 2 seconds
✅ **Velocity Estimation:** Estimated vy ≈ 0.5 m/s (±0.2 m/s)
✅ **Dynamic Classification:** `is_dynamic = true` once velocity estimate stabilizes
✅ **Risk Computation:** `collision_risk > 0.5` when obstacle within 1.5m of predicted robot path
✅ **Evasive Action:** Robot selects non-zero steering angle when straight-ahead has high risk
✅ **Safe Stop:** Robot velocity → 0 when all trajectories blocked
✅ **Recovery:** Robot resumes navigation after obstacle clears

## Failure Modes to Check

❌ **Ignoring Dynamic Obstacles:** Robot treats moving obstacle like static map feature
❌ **Late Reaction:** Robot doesn't predict collision until already too close
❌ **False Positives:** Static objects incorrectly classified as dynamic
❌ **Stale Tracks:** Obstacle disappears but stale track continues to block path
❌ **Duplicate TF Authority:** Multiple nodes publishing conflicting transforms
❌ **Safety Bypass:** Local planner commands unsafe velocity despite high risk

## Launch Command

```bash
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Start simulation with dynamic obstacle world
ros2 launch sih26126_bringup full_stack.launch.py \
  world:=dynamic_obstacle_demo \
  slam:=true \
  fusion:=true \
  gui:=false

# In separate terminal, start obstacle controller
ros2 run sih26126_autonomy dynamic_obstacle_controller
```

## Data Collection Topics

Monitor these topics during test:
- `/obstacles/raw` - Raw detections
- `/obstacles/tracked` - Velocity-estimated tracks
- `/obstacles/dynamic_risk` - Scalar risk [0,1]
- `/local_path_selector/state` - Planner state
- `/planner/risk_assessment` - Detailed risk breakdown
- `/safety/state` - Safety supervisor decision
- `/cmd_vel` - Final commanded velocity
- `/model/sih_ugv/odometry` - Robot ground truth
