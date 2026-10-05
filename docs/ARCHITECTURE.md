# DRISTHI — System Architecture

## Overview

DRISTHI is a ROS 2 Jazzy / Gazebo Sim stack for GPS-denied outdoor autonomous
navigation. The design is layered: sensing feeds perception, perception feeds
localization and mapping, mapping feeds exploration, exploration feeds planning,
and planning feeds a safety-supervised actuator.

## Pipeline

```
Sensors (Gazebo Sim)
  /camera                    RGB image (sensor_msgs/Image)
  /depth_camera/points       Depth point cloud (PointCloud2)
  /imu                       IMU (sensor_msgs/Imu)
  /lidar/points              3-D LiDAR point cloud (PointCloud2)
         |
         ├── perception_baseline      → /traversability/image
         ├── traversability_baseline  → /traversability/score
         ├── terrain_geometry         → slope / roughness estimates
         ├── terrain_classifier       → terrain class (FLAT / SLOPE / ROUGH)
         └── terrain_aware_traversability → combined traversability score
         |
  lidar_processor
         /lidar/points → /lidar/scan_2d (LaserScan)
         |
  slam_toolbox  (when slam:=true)
         /lidar/scan_2d → map → odom TF
         /map (OccupancyGrid)
         |
  robot_localization EKF  (when fusion:=true)
         /model/sih_ugv/odometry + /imu → /odometry/filtered
         odom → base_link TF
         |
  online_mapper
         /depth_camera/points + odometry → /map  (when slam:=false)
         |
  frontier_explorer
         /map + /localization/pose → /exploration/goal, /exploration/frontiers
         |
  goal_path_planner
         /exploration/goal + /map → /planner/global_path
         |
  local_path_selector
         /planner/global_path + obstacle data + dynamic_risk
         → /planner/local_path, /planner/local_recommended_cmd_vel (advisory)
         → /planner/risk_assessment (PlannerRisk)
         |
  safety_supervisor                ← sole /cmd_vel publisher
         /planner/local_recommended_cmd_vel (advisory)
         /recovery/cmd_vel_suggestion (advisory)
         → /cmd_vel → Gazebo
         |
  stuck_recovery  (advisory only)
         → /recovery/cmd_vel_suggestion
         → /recovery/state
```

## TF Tree

| Transform         | Authority                   | Condition                      |
|-------------------|-----------------------------|-------------------------------|
| `map → odom`      | **slam_toolbox**            | `slam:=true`                  |
| `odom → base_link`| **robot_localization EKF**  | `fusion:=true`                |
| `odom → base_link`| odom_tf_publisher           | `slam:=true, fusion:=false`   |
| `odom → base_link`| Gazebo TF bridge            | `slam:=false`                 |
| `base_link → lidar_3d` | lidar_static_tf_publisher | `slam:=true`             |
| `base_link → imu` | imu_static_tf_publisher     | `fusion:=true`                |

**Exactly one authority** per TF edge is active at runtime. The launch file
enforces mutual exclusion via `IfCondition` / `UnlessCondition`.

## Key Architectural Invariants

- **`safety_supervisor` is the sole `/cmd_vel` publisher.** All planner and
  recovery outputs are advisory topics. No other node writes to `/cmd_vel`.

- **`/map` has exactly one publisher.** Either `slam_toolbox` (when `slam:=true`)
  or `online_mapper` (when `slam:=false`), never both simultaneously.

- **No ground truth is used as an autonomy input.** The localization pipeline
  uses only wheel odometry, IMU, and 2D LiDAR — no Gazebo pose ground-truth
  is fed into any navigation decision.

- **`/localization/confidence`** is a health signal, not a navigation accuracy
  estimate. It indicates whether the active localization source is healthy.

- **The dashboard is read-only.** It subscribes to topics and serves a live
  HTTP interface. It never publishes to any actuator topic.

## Package Responsibilities

| Package                    | Role                                                        |
|----------------------------|-------------------------------------------------------------|
| `sih26126_bringup`         | Launch files — `full_stack.launch.py` is the entry point    |
| `sih26126_autonomy`        | Frontier explorer, mission manager, online mapper, mission intelligence |
| `sih26126_safety`          | Safety supervisor (sole /cmd_vel), stuck recovery, degraded autonomy |
| `sih26126_planner`         | Global + local planners, risk assessment (advisory)         |
| `sih26126_localization`    | SLAM adapter, EKF adapter, TF publishers, fusion health monitor |
| `sih26126_perception`      | RGB perception baseline → traversability mask               |
| `sih26126_traversability`  | Traversability scoring (RGB + terrain-aware)                |
| `sih26126_geometry`        | Terrain geometry estimation, RANSAC slope, terrain classifier |
| `sih26126_sensor_contract` | Sensor health, LiDAR processor, scan relay, obstacle extractor/tracker |
| `sih26126_dashboard`       | Read-only live autonomy dashboard (HTTP)                    |
| `sih26126_sim`             | Simulation worlds and UGV model (SDF)                       |
| `sih26126_msgs`            | Custom ROS message definitions                              |

## Sensor Integration

| Sensor         | GZ topic                    | ROS topic                   | Type                        |
|----------------|-----------------------------|-----------------------------|------------------------------|
| RGB camera     | `/camera`                   | `/camera`                   | `sensor_msgs/Image`          |
| Depth camera   | `/depth_camera/points`      | `/depth_camera/points`      | `sensor_msgs/PointCloud2`    |
| IMU            | `/imu`                      | `/imu`                      | `sensor_msgs/Imu`            |
| 3-D LiDAR      | `/lidar/points/points`      | `/lidar/points`             | `sensor_msgs/PointCloud2`    |
| Odometry       | `/model/sih_ugv/odometry`   | `/model/sih_ugv/odometry`   | `nav_msgs/Odometry`          |

The LiDAR bridge uses a dedicated `parameter_bridge` node with `config_file`
(`sih26126_bringup/config/bridge.yaml`) because the GZ topic name differs from
the ROS topic name.

## Degraded Autonomy

`degraded_autonomy_manager` monitors localization confidence, LiDAR health, and
recovery state. It publishes `/autonomy/mode` (AutonomyMode) with transitions:

```
FULL_AUTONOMY → DEGRADED_SENSOR | DEGRADED_LOCALIZATION | DEGRADED_PERCEPTION
                              → RECOVERY → FULL_AUTONOMY | SAFE_HOLD
```

The safety supervisor consults autonomy mode but retains full veto authority
over `/cmd_vel`.
