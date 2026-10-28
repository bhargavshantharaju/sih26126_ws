# DRISTHI SYSTEM REDESIGN PLAN
## Engineering Restart — 2026-10-06

---

## MISSION STATEMENT

**Actual Problem:**
Build an autonomous UGV that explores and maps an unknown GPS-denied outdoor environment without prior maps, using vision and depth sensing for terrain-aware navigation, LiDAR for localization and obstacle detection, and frontier-based autonomous exploration.

**Required Capabilities:**
1. Online SLAM-based localization in unknown environment
2. Occupancy mapping from sensors
3. Terrain traversability assessment from depth/vision
4. Autonomous frontier-based exploration
5. Global path planning through discovered space
6. Local obstacle avoidance and path following
7. Safety-supervised motion control
8. Stuck detection and recovery
9. Continuous autonomous operation through multiple exploration cycles

**NOT Required:**
- GPS integration
- Visual odometry (wheel+IMU+SLAM is sufficient)
- Pre-existing maps
- Manual waypoint navigation (exploration is autonomous)
- ML/AI features beyond classical perception

---

## CURRENT ARCHITECTURE ASSESSMENT

### What Is Working ✅

1. **Package Structure** — Clean ROS 2 modularity
2. **SLAM Integration** — slam_toolbox works when environment is stable
3. **EKF Fusion** — robot_localization fuses wheel+IMU
4. **Sensor Streams** — RGB, depth, LiDAR, IMU all publish
5. **Terrain Pipeline** — Depth → geometry → traversability scoring exists
6. **Frontier Detection** — Identifies unexplored regions
7. **Global Planner** — A* path through occupancy grid
8. **Local Planner** — Terrain-aware candidate selection
9. **Safety Architecture** — Sole `/cmd_vel` authority pattern is correct
10. **Stuck Recovery** — FSM logic and guard conditions are sound
11. **Unit Tests** — 316/321 passing, good coverage
12. **Dashboard** — Read-only visualization exists

### What Is Fragile ❌

1. **Simulation Performance** — Showcase world runs at 29-62% real-time, clock jumps backwards
2. **Robot Dynamics** — Rotation at 15% of commanded rate
3. **Goal Behind Robot** — Navigation stuck rotating toward goal that ended up 155° behind
4. **SLAM Message Drops** — Queue overflows, "timestamp earlier than cache"
5. **Frame Coordination** — Multiple localization sources, complex frame matching
6. **Sensor Rate Configuration** — May not match actual capabilities
7. **Frontier Distance Calculation** — Strange values (37m for nearby frontiers)
8. **DEGRADED_SENSOR Mode** — Triggers unnecessarily during normal operation
9. **Mission State Logic** — Overly complex transitions, grace periods
10. **Multiple Workarounds** — Accumulated patches hiding root causes

### Root Causes Identified

**Primary**: Gazebo showcase world is computationally too expensive
- Heightmap terrain generation
- Dense tree/rock decoration
- Complex collision geometry
- Dynamic lighting
- Result: Cannot maintain >90% real-time, clock becomes unstable

**Secondary**: Robot rotation dynamics are incorrect
- Commanded 0.45 rad/s → measured 0.07 rad/s (in sim time)
- Possible sign inversion in rotation direction
- Robot rotated 132° when goal was at -22.7° (only needed +22.7°)

**Tertiary**: Accumulated complexity
- Too many competing state sources
- Too many timeouts and grace periods
- Too many special cases
- System became hard to reason about

---

## REDESIGN PRINCIPLES

1. **Simulation Must Be Stable First**
   - Target: >95% real-time with zero clock discontinuities
   - Approach: Simpler world that still represents outdoor challenge

2. **Robot Must Respond Predictably**
   - Fix rotation dynamics
   - Verify rotation direction
   - Test in minimal environment before full autonomy

3. **One Clear Data Flow**
   - Sensors → Health → State Estimation → Mapping → Exploration → Planning → Safety → Motion
   - No hidden ground truth injection
   - No competing localization sources

4. **Robust Not Complex**
   - Prefer simple working system over complex fragile system
   - Remove workarounds once root cause is fixed
   - Clear error handling, not endless timeouts

5. **Verifiable at Each Stage**
   - Stage A: Stable sim + sensor streams
   - Stage B: SLAM + TF + EKF healthy
   - Stage C: Map grows, frontiers detected
   - Stage D: One navigation cycle works
   - Stage E: Two navigation cycles work (proves closed loop)

---

## PROPOSED FINAL ARCHITECTURE

### Robot Configuration
```
Differential-drive UGV
- Wheel separation: 0.96 m
- Wheel radius: 0.28 m  
- Mass: 28 kg (verify/tune for responsive dynamics)
- Wheel friction: 1.5 (verify sufficient for rotation)
- Joint damping: 0.05 (verify not over-damped)
- Sensors mounted at known, tested heights
```

### Sensor Suite & Responsibilities
```
3D LiDAR (5.6 Hz)
  → 2D horizontal scan projection
  → slam_toolbox (map ← odom)
  → Obstacle detection (forward field)
  
RGB-D Camera (8-10 Hz)
  → Depth point cloud
  → Terrain geometry estimation
  → Traversability scoring
  → Local obstacle avoidance
  
IMU (50 Hz)
  → Orientation
  → Angular velocity
  → EKF input
  
Wheel Odometry (30 Hz)
  → Linear velocity
  → EKF input
```

**Rationale**: Each sensor has ONE primary role. LiDAR is most reliable for SLAM. Depth is best for local terrain. IMU+wheels provide continuous local motion estimate.

### Localization Architecture
```
slam_toolbox
  ↓
map → odom TF
  
robot_localization EKF
  ← IMU
  ← wheel odometry
  ↓
odom → base_link TF
  ↓
/odometry/filtered

slam_pose_adapter
  ← map → odom TF
  ← odom → base_link TF
  ↓
/localization/pose (map frame)
  ↓
Used by: frontier explorer, global planner, local planner
```

**No competing sources. One authoritative chain.**

### Mapping Architecture
```
LiDAR scan
  ↓
slam_toolbox
  ↓
/map (occupancy grid, map frame)
  ↓
Used by: frontier explorer, global planner
```

**When slam:=false, online_mapper publishes /map from depth+odometry. Never both.**

### Exploration Architecture
```
/map + /localization/pose
  ↓
frontier_explorer
  ↓
Evaluate each frontier:
  - Information gain (unexplored area reachable)
  - Distance cost (path length)
  - Terrain risk (from traversability)
  - Obstacle risk (from depth obstacles)
  - Localization confidence
  ↓
Frontier utility score
  ↓
Select best frontier
  ↓
/exploration/goal
```

**This is the innovation: risk-aware information-gain exploration, not just nearest frontier.**

### Planning Architecture
```
/exploration/goal + /map + /localization/pose
  ↓
goal_path_planner (A*)
  ↓
/planner/global_path (map frame)
  
/planner/global_path + /localization/pose + depth + LiDAR + traversability
  ↓
local_path_selector
  ├─ Terrain-aware corridor scoring
  ├─ Obstacle clearance (depth + LiDAR)
  ├─ Goal alignment
  ├─ Smoothness
  ↓
Selected steering angle
  ↓
waypoint_command()
  ├─ Rotate if goal > 30° off
  ├─ Translate + steer if aligned
  ↓
/planner/local_recommended_cmd_vel (advisory)
```

### Safety Architecture
```
/planner/local_recommended_cmd_vel (advisory)
/recovery/cmd_vel_suggestion (advisory)
Sensor health
Localization health
Obstacle proximity
Motion limits
  ↓
safety_supervisor (SOLE AUTHORITY)
  ↓
/cmd_vel
  ↓
Gazebo UGV
```

**All command generation is advisory. Safety has final veto.**

### Recovery Architecture
```
Commanded velocity vs measured velocity
  ↓
stuck_recovery_node
State machine: NORMAL → STUCK_SUSPECTED → RECOVERY_ACTIVE → SUCCESS/FAILED
  ↓
/recovery/cmd_vel_suggestion (advisory)
  ↓
safety_supervisor decides whether to use
```

**Recovery never bypasses safety.**

### Gazebo World Design
```
Minimal Stable World:
- Flat ground plane (no heightmap)
- 4-6 simple box obstacles (varied sizes)
- 2-3 narrow passages
- Open areas requiring exploration
- Spawn point with clear initial view
- No complex collision geometry
- Basic directional lighting
- Target: >95% real-time, zero clock jumps

Representative World (after minimal works):
- Gentle terrain variation (not extreme heightmap)
- Scattered obstacles (rocks, simple shapes)
- Some vegetation (low-complexity)
- Still achieves >90% real-time
```

---

## IMPLEMENTATION STAGES

### STAGE A: Stable Simulation Foundation

**Goal**: Gazebo runs reliably >95% real-time with stable clock

**Tasks**:
1. Create `minimal_stable.world`
   - Flat plane
   - 5 box obstacles
   - Simple lighting
   - No decorations

2. Test robot spawn and basic motion
   - Verify cmd_vel → actual motion
   - Verify rotation direction (positive angular.z = counter-clockwise)
   - Measure actual rotation rate vs commanded

3. Fix robot dynamics if needed
   - Adjust mass/inertia
   - Adjust wheel friction
   - Adjust joint damping
   - Target: commanded 0.45 rad/s → measured 0.40-0.45 rad/s

4. Measure sim performance
   - `ros2 topic hz /clock`
   - Real-time factor >95%
   - Zero "Detected jump back in time" messages
   - Sensor streams stable

**Acceptance**: Robot moves predictably, sim is stable

### STAGE B: Localization Health

**Goal**: SLAM + EKF provide reliable pose

**Tasks**:
1. Launch slam_toolbox in minimal world
   - Verify /map publication
   - Verify map → odom TF
   - Verify no message drops
   - Verify no clock issues

2. Launch EKF fusion
   - Verify odom → base_link TF
   - Verify /odometry/filtered
   - Verify sensor timing is appropriate
   - Measure actual IMU, wheel odom rates
   - Set sensor_timeout based on measurements

3. Verify slam_pose_adapter
   - Publishes /localization/pose in map frame
   - Correct transformation chain
   - Fresh updates

4. Drive robot manually with teleop
   - Verify map grows
   - Verify pose tracks correctly
   - Verify TF tree is complete

**Acceptance**: Manual driving builds map, pose is accurate

### STAGE C: Autonomous Mapping

**Goal**: Frontier detection and goal selection work

**Tasks**:
1. Launch full stack in minimal world (no obstacles initially)
   - Frontier explorer detects unknown regions
   - Selects a frontier
   - Generates exploration goal
   - Publishes /exploration/goal

2. Implement/verify frontier utility scoring
   ```python
   utility = (
       0.30 * information_gain_score
       + 0.25 * (1 - distance_cost)
       + 0.20 * (1 - terrain_risk)
       + 0.15 * (1 - obstacle_risk)
       + 0.10 * localization_confidence
   )
   ```

3. Add obstacles to world
   - Verify frontier selection avoids obstacles
   - Verify utility scores published
   - Verify reasoning is visible

**Acceptance**: Frontier selection is intelligent and explainable

### STAGE D: First Navigation Cycle

**Goal**: Robot autonomously navigates to one frontier

**Tasks**:
1. Goal → global path works
   - A* through occupancy grid
   - Path is geometrically valid
   - Path published in map frame

2. Local planner follows path
   - Uses map-frame localization pose
   - Computes correct goal angle
   - Selects terrain-aware corridor
   - Commands rotation when goal > 30° off
   - Commands forward motion when aligned
   - Publishes obstacle evidence

3. Safety passes commands
   - No false stops
   - Handles stale data correctly
   - Respects obstacle proximity

4. Robot physically moves
   - Rotates toward goal
   - Translates forward
   - Reaches frontier vicinity
   - Map grows
   - New frontiers appear

**Acceptance**: Robot completes one full exploration cycle

### STAGE E: Second Navigation Cycle

**Goal**: Continuous autonomous exploration

**Tasks**:
1. After reaching first frontier
   - New frontier detected
   - New goal selected
   - New path planned
   - Robot continues moving
   - No manual intervention

2. Handle edge cases
   - No reachable frontier → stop gracefully
   - Obstacle blocks path → replan
   - Stuck detected → recovery → continue

**Acceptance**: Robot completes TWO autonomous cycles

### STAGE F: Risk-Aware Intelligence

**Goal**: Exploration quality is visibly better than "nearest frontier"

**Tasks**:
1. Add terrain variation to world
   - Sloped ramp
   - Rough area
   - Smooth area

2. Verify traversability affects frontier selection
   - Robot prefers smoother paths
   - Avoids high-risk terrain when alternatives exist
   - Still explores necessary rough areas

3. Publish frontier scores
   ```
   /exploration/frontier_scores
   [
     {id: 0, utility: 0.82, info_gain: 0.75, distance_cost: 0.20, terrain_risk: 0.10, ...},
     {id: 1, utility: 0.64, info_gain: 0.65, distance_cost: 0.45, terrain_risk: 0.35, ...},
     ...
   ]
   ```

**Acceptance**: System behavior is explainable and intelligent

### STAGE G: Presentation Quality

**Goal**: Demo is convincing

**Tasks**:
1. Clean world that shows capabilities
   - Multiple rooms/regions
   - Varied terrain
   - Obstacles requiring avoidance
   - Clear exploration progression

2. Dashboard shows key information
   - Map with frontiers
   - Selected frontier highlighted
   - Frontier utility scores
   - Path visualization
   - Terrain/obstacle risk
   - Exploration progress

3. Documentation
   - Clear README
   - Architecture diagram
   - Capability list
   - Limitations stated honestly

**Acceptance**: 5-minute demo conveys intelligence

---

## CHANGES REQUIRED

### World Files
- **Create**: `src/sih26126_sim/worlds/minimal_stable.world`
- **Modify**: Launch files to support new world
- **Later**: Create representative world after minimal works

### Robot Model
- **Investigate**: Current rotation dynamics
- **Fix**: If rotation rate or direction is wrong
- **Verify**: Mass, inertia, friction, damping

### Localization
- **Keep**: Current architecture (SLAM + EKF + adapter)
- **Fix**: Frame matching issues in local planner
- **Remove**: Competing localization sources if any exist

### Frontier Explorer
- **Add**: Utility scoring with information gain + risk
- **Add**: Publish detailed frontier scores
- **Fix**: Distance calculation if buggy
- **Remove**: Overly long grace periods if unjustified

### Planning
- **Keep**: Global A* + local terrain-aware selection
- **Fix**: Goal angle calculation (verify not inverted)
- **Add**: Better logging of goal bearing math
- **Fix**: Rotate-in-place logic (verify threshold)

### Safety
- **Keep**: Current architecture
- **Remove**: Unnecessary sensor timeouts if too conservative
- **Verify**: Actual sensor timing, adjust timeouts accordingly

### Configuration
- **Remove**: Sensor rate changes that made performance worse
- **Measure**: Actual rates, configure appropriately
- **Document**: Why each timeout value was chosen

### Code Cleanup
- **Remove**: Temporary diagnostic logging after issues fixed
- **Remove**: Workarounds after root cause fixed
- **Consolidate**: Parameter files
- **Document**: Frame ownership clearly

---

## SUCCESS CRITERIA

### Minimum Viable
1. ✅ Simulation runs >95% real-time, clock stable
2. ✅ Robot responds to commands correctly
3. ✅ SLAM + EKF healthy
4. ✅ Map grows from observations
5. ✅ Frontiers detected
6. ✅ One frontier selected intelligently
7. ✅ Path planned and followed
8. ✅ Robot reaches first goal
9. ✅ Second frontier detected
10. ✅ Robot continues to second goal
11. ✅ **TWO COMPLETE AUTONOMOUS CYCLES**

### Desired
12. Terrain risk visibly affects decisions
13. Obstacle avoidance works live
14. Stuck recovery demonstrated
15. Dashboard shows reasoning
16. Clean, explainable architecture

### Documentation
17. README explains mission
18. Architecture documented
19. Limitations stated honestly
20. No fake claims

---

## WHAT NOT TO DO

❌ Don't keep showcase world if it cannot run stably
❌ Don't optimize for screenshots over function
❌ Don't add ML/AI buzzwords without implementation
❌ Don't claim visual odometry if only using wheel+IMU
❌ Don't fake ground truth as localization
❌ Don't bypass safety supervisor
❌ Don't make dashboard mandatory
❌ Don't remove obstacle avoidance to hide failures
❌ Don't use 15-second grace periods to hide exploration failures
❌ Don't accumulate more workarounds
❌ Don't preserve fragile code just because it exists

✅ Do fix root causes
✅ Do verify at each stage
✅ Do simplify where appropriate
✅ Do document honestly
✅ Do make behavior explainable
✅ Do prove with real runtime
✅ Do build technical story worth presenting

---

## ESTIMATED EFFORT

| Stage | Time | Risk |
|-------|------|------|
| A: Stable sim | 2-3 hours | Low (straightforward) |
| B: Localization | 1-2 hours | Low (architecture exists) |
| C: Mapping | 2-3 hours | Medium (frontier scoring) |
| D: First cycle | 3-4 hours | Medium (integration) |
| E: Second cycle | 1-2 hours | Low (should work if D works) |
| F: Intelligence | 2-3 hours | Medium (tuning) |
| G: Polish | 1-2 hours | Low (visualization) |

**Total: 12-19 hours of focused engineering**

---

## NEXT IMMEDIATE ACTIONS

1. Create minimal_stable.world
2. Test robot dynamics in minimal world
3. Fix rotation if broken
4. Verify sim stability
5. Launch SLAM in minimal world
6. Verify localization health
7. Proceed through stages systematically

**Do not skip ahead. Each stage must work before the next.**

---

## FINAL SYSTEM STORY

> **DRISTHI** is an autonomous UGV that explores unknown GPS-denied environments using risk-aware frontier-based exploration.
>
> It uses 3D LiDAR for SLAM-based localization, RGB-D cameras for terrain-aware traversability assessment, and an IMU+wheel odometry fusion for continuous state estimation.
>
> Unlike simple nearest-frontier approaches, DRISTHI evaluates exploration targets using information gain, path cost, terrain risk, obstacle risk, and localization confidence—choosing targets that maximize mapping progress while maintaining safety.
>
> The system demonstrates closed-loop autonomy: sensing → mapping → intelligent exploration → planning → safety-supervised motion → updated sensing. 
>
> It handles obstacles dynamically, recovers from being stuck, and operates continuously through multiple exploration cycles without human intervention.

**This is technically honest, architecturally sound, and demonstrably real.**

