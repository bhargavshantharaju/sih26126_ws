# SIH26126 UGV autonomy demo

ROS 2 Jazzy and Gazebo Sim stack for live perception, depth-based mapping, frontier exploration, local planning, and safety-supervised motion.

## Runtime architecture

```text
Gazebo RGB/depth/IMU/odometry
        ↓
ROS-Gazebo bridge
        ├── RGB image → perception baseline → traversability score image
        ├── depth cloud → terrain geometry/classification → terrain-aware score
        ├── depth cloud + Gazebo odometry → online occupancy mapper → /map
        └── odometry → pose adapter → /localization/pose
                                  ↓
                       reachable frontier explorer
                         ├── /exploration/goal
                         └── /planner/global_path
                                  ↓
                 goal advisory → local path selector
                                  ↓
                     safety_supervisor → /cmd_vel → Gazebo
```

The robot starts with an all-unknown bounded occupancy grid. `/map` is updated from live depth point-cloud rays in the odometry coordinate frame; the forest SDF is never read by the mapper. Occupancy values use `-1` unknown, `0` free, and `100` occupied. The frontier explorer selects reachable known-free cells beside unknown space and plans over known-free cells only.

Terrain slope is estimated from a deterministic RANSAC plane fit over the central lower-image driving corridor. Candidate planes must have a near-vertical normal (maximum configurable ground slope defaults to 50°); vertical tree and obstacle faces are rejected. The fitted ground inliers alone contribute to slope and roughness classification. This replaces the earlier average over all lower-image surface normals, which included forest objects and incorrectly classified the nominally flat forest floor as steep. Ground fit distance, corridor bounds, and slope limit are ROS parameters on `terrain_geometry`.

**Localization and mapping scope:** this is **ODOMETRY-ASSISTED ONLINE MAPPING**, not SLAM. The current map pose source is Gazebo simulation odometry, and the map is expressed in its `odom` frame. `rgbd_odometry_baseline` remains a separate sensor-derived baseline and is not fused into the navigation/map pose. No loop closure is claimed.

`safety_supervisor` is the sole authoritative ROS publisher of `/cmd_vel`. Planner outputs remain advisory. The dashboard is read-only and derives its map, images, state, health, and event log from ROS messages.

The local selector checks receive age for depth, terrain score image, and odometry before generating motion. Missing or stale input publishes a zero local command and a non-navigation state, which the safety supervisor rejects. Its age limit is configurable with `max_sensor_age_s` (default 0.75 seconds).

## Build and test

```bash
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

## Showcase terrain demo

The showcase uses a locally generated grayscale heightmap plus local SDF tree, rock, and fallen-log geometry. A recessed basin has visual-only water; the central valley corridor remains drivable. It has no supplied occupancy map or hidden route. The full-stack launch takes an exclusive lock so a second launch cannot create competing Gazebo clocks, publishers, and mission managers.

Launch Gazebo, autonomy, and the read-only dashboard together:

```bash
cd ~/sih26126_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sih26126_bringup full_stack.launch.py \
  gui:=true \
  start_dashboard:=true \
  dashboard_port:=8101 \
  world:=$(ros2 pkg prefix sih26126_sim)/share/sih26126_sim/worlds/showcase_terrain.sdf
```

Open the localhost URL printed by the dashboard; the requested port can advance by up to ten if occupied.

## Key live topics

- Sensors: `/camera`, `/camera_info`, `/depth_camera/points`, `/imu`.
- Pose: `/model/sih_ugv/odometry` and the explicitly labeled `/localization/pose` adapter output.
- Localization contract: `/localization/pose`, `/localization/source`, and `/localization/confidence`. The current confidence value indicates that the Gazebo pose source is available; it is not a visual odometry accuracy estimate.
- Mapping/exploration: `/map`, `/mapping/state`, `/exploration/state`, `/exploration/frontiers`, `/exploration/goal`.
- Planning: `/planner/global_path`, `/planner/local_path`, `/planner/local_steering_angle`, `/planner/local_obstacle_clearance`, `/planner/local_obstacle_score`.
- Velocity advisory: `/planner/recommended_speed` is terrain- and heading-aware; `/planner/local_recommended_cmd_vel` applies the selected corridor's obstacle factor.
- Safety/motion: `/safety/state`, `/safety/reason`, `/cmd_vel`.

The safety supervisor independently requires fresh `/localization/confidence` as well as a fresh planner state, command, and obstacle score. Missing, stale, non-finite, or low localization confidence results in a zero `/cmd_vel`. This protects the simulation adapter contract; it does not convert Gazebo odometry into visual localization. ROS nodes currently use their default clock behavior; `/clock` is bridged, but the full launch does not enable `use_sim_time` across nodes.

The global explorer’s `NO_FRONTIER` status means no reachable frontier is currently available in the observed free map; it does not certify that the whole world has been explored.
