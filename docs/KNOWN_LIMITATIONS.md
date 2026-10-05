# DRISTHI — Known Limitations

## Architecture Limitations

### Dynamic Obstacle Detection — NOT VALIDATED
The `scenario:=dynamic_obstacle` launch mode and `objective4_obstacle_controller`
node exist in the codebase. However, the following have **not** been demonstrated
in runtime:

- Valid forward LiDAR returns from a moving obstacle in Gazebo
- A UGV–obstacle physical encounter during navigation
- Obstacle-triggered planner or safety response
- Collision-free evasive reaction

The obstacle tracker and extractor nodes (`obstacle_extractor`, `obstacle_tracker`)
run in the showcase scenario but are processing static terrain clusters, not a
confirmed moving obstacle. Do not claim Objective 4 is complete.

### Terrain Confidence / Temporal Memory — Not Implemented
The system scores terrain traversability per-frame. There is no accumulation of
terrain observations over time or confidence weighting from revisited locations.
This is noted as a planned future feature.

### Information-Gain-Aware Frontier Selection — Not Implemented
Frontiers are selected by reachability and distance. Information gain
(how much new map area a frontier would reveal) is not computed. This is a
planned improvement.

### Risk-Aware Planning — Partial
The `local_path_selector` computes a `PlannerRisk` assessment using terrain,
obstacle, localization, and dynamic risk components. However, the global planner
does not yet use this risk signal for path modification — it affects local path
selection only.

### Live Stuck-Recovery Test Scenario — Not Implemented
The `stuck_recovery_node` has deterministic unit tests covering its state machine.
A Gazebo integration test (robot physically gets stuck and autonomously recovers
in simulation) has not been implemented.

## Simulation / Environment Limitations

### Gazebo GUI in WSL
The Gazebo GUI (`gui:=true`) may not render correctly in WSL2 environments due
to GPU/display limitations. Use `gui:=false` for headless operation.

### SLAM Cold-Start Timing
The `slam_activator` node calls the slam_toolbox lifecycle service to configure
and activate SLAM. Occasionally the first `GetState` service call times out
(observed: `Failed to read slam_toolbox initial state`). On retry or relaunch,
SLAM activates normally. This is a startup race condition in the lifecycle
service, not a logic error.

### `/lidar/scan_2d` → `/scan` Relay
The `scan_relay` node republishes `/lidar/scan_2d` to the legacy `/scan` topic
for backward compatibility. This relay is deprecated; components should subscribe
to `/lidar/scan_2d` directly. The relay will be removed in a future cleanup.

### Localization in Baseline Mode (`slam:=false`)
Without SLAM, the system uses raw Gazebo odometry for the `odom → base_link`
transform and has no loop closure. Drift accumulates over long runs. The baseline
mode is provided for comparison; the recommended operational mode is
`slam:=true fusion:=true`.

### No GPS / No External Positioning
DRISTHI is designed for GPS-denied environments. There is no integration with
GPS, RTK, or any external positioning system. All navigation is relative to the
robot's starting position.
