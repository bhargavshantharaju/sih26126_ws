# DRISTHI — Validation Results

## Current Session Validation (2026-10-05)

These results were directly observed in this session using `ros2 topic hz`,
`gz topic -i`, and `ros2 run tf2_ros tf2_echo`. All measurements from a live
running stack with `slam:=true fusion:=true scenario:=showcase`.

### Sensor Pipeline

| Topic                 | Type                          | Measured Hz | Status    |
|-----------------------|-------------------------------|-------------|-----------|
| `/lidar/points`       | `sensor_msgs/PointCloud2`     | ~5.6 Hz     | ✅ PASS   |
| `/lidar/scan_2d`      | `sensor_msgs/LaserScan`       | ~5.6 Hz     | ✅ PASS   |
| `/imu`                | `sensor_msgs/Imu`             | ~100 Hz     | ✅ PASS   |
| `/odometry/filtered`  | `nav_msgs/Odometry`           | ~17 Hz      | ✅ PASS   |

### LiDAR Bridge

- GZ topic: `/lidar/points/points` — `gz.msgs.PointCloudPacked`, publisher confirmed
- ROS topic: `/lidar/points` — `sensor_msgs/PointCloud2`, publisher: `sih26126_lidar_bridge`
- Mapping: GZ → ROS rename via `bridge.yaml` config_file bridge ✅

### SLAM

- `slam_toolbox` activated via `slam_activator` lifecycle service
- `map → odom` TF published at sim time 41s+ ✅
- scan_topic: `/lidar/scan_2d` ✅

### Sensor Fusion

- `robot_localization` EKF node active with `fusion:=true`
- Inputs: `/model/sih_ugv/odometry` + `/imu`
- Output: `/odometry/filtered` at ~17 Hz ✅
- `fusion_health_monitor`: state=HEALTHY, imu=True, odom=True, ekf=True ✅

### Localization Pipeline

- `odom → base_link`: robot_localization EKF (fusion mode) ✅
- `map → odom`: slam_toolbox ✅
- `base_link → lidar_3d`: lidar_static_tf_publisher ✅

## Previously Validated (Prior Sessions)

Multiple successful autonomous exploration runs were recorded prior to this
session. Mission result artifacts are in `artifacts/mission_results/`. These
runs demonstrated:

- Frontier-based autonomous exploration
- Occupancy map growth over time
- Terrain-aware traversability scoring in motion
- Safety supervisor stopping on localization loss
- Stuck recovery activation and recovery

Specific metrics from those runs are in the JSON artifacts — not reproduced
here to avoid fabrication.

## Test Suite

```
colcon test
colcon test-result --all
```

**321 tests total — 316 passed, 5 skipped, 0 failures.**

Coverage includes:
- Stuck detection state machine (all transitions, guard conditions)
- Safety supervisor logic (SAFE / STOPPED / E_STOP transitions)
- Frontier explorer (reachability, boundary conditions)
- Mission manager (state transitions, completion criteria)
- Local path selector (risk integration, obstacle avoidance)
- Sensor contract (health monitoring)
- Degraded autonomy state machine
- Mission intelligence event generation

## Not Yet Validated

| Capability                              | Status                    |
|-----------------------------------------|---------------------------|
| Dynamic obstacle sensor detection       | NOT VALIDATED in runtime  |
| UGV–obstacle physical encounter         | NOT DEMONSTRATED          |
| Obstacle-triggered evasive maneuver     | NOT DEMONSTRATED          |
| Live stuck-recovery in Gazebo           | NOT DEMONSTRATED          |

The dynamic obstacle scenario (`scenario:=dynamic_obstacle`) is preserved for
future development. Do not interpret its presence as evidence of working
runtime behavior.
