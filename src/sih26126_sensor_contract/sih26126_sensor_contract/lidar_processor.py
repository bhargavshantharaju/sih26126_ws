#!/usr/bin/env python3
# Copyright 2026 DRISTHI Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""DRISTHI 3D LiDAR processor.

Converts /lidar/points (PointCloud2) to /lidar/scan_2d (LaserScan)
by projecting the horizontal plane slice.
Also publishes LiDAR health on /sensor_health/lidar.
"""

from __future__ import annotations  # Enable list[type] syntax for Python 3.9+

import math
import struct
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sih26126_msgs.msg import SensorHealth
import tf2_ros


def pointcloud2_to_xyz(msg: PointCloud2):
    """Extract XYZ points from a PointCloud2 message."""
    # Parse field offsets
    field_map = {f.name: f.offset for f in msg.fields}
    x_off = field_map.get('x', 0)
    y_off = field_map.get('y', 4)
    z_off = field_map.get('z', 8)
    point_step = msg.point_step
    n_points = msg.width * msg.height
    if n_points == 0:
        return np.empty((0, 3), dtype=np.float32)
    data = np.frombuffer(msg.data, dtype=np.uint8)
    xs = np.frombuffer(data[x_off::point_step][:n_points * 4].tobytes(),
                       dtype=np.float32)
    ys = np.frombuffer(data[y_off::point_step][:n_points * 4].tobytes(),
                       dtype=np.float32)
    zs = np.frombuffer(data[z_off::point_step][:n_points * 4].tobytes(),
                       dtype=np.float32)
    return np.column_stack([xs, ys, zs])


def project_to_scan_2d(
    points: np.ndarray,
    angle_min: float,
    angle_max: float,
    n_bins: int,
    range_min: float,
    range_max: float,
    vertical_tolerance_rad: float = 0.0873,  # ~5 degrees
) -> np.ndarray:
    """Project 3D XYZ points to a 2D range array via horizontal slice."""
    if len(points) == 0:
        return np.full(n_bins, float('inf'))
    # Filter by vertical angle (elevation)
    xy_dist = np.hypot(points[:, 0], points[:, 1])
    elevation = np.arctan2(points[:, 2], np.maximum(xy_dist, 1e-6))
    v_mask = np.abs(elevation) <= vertical_tolerance_rad
    pts = points[v_mask]
    if len(pts) == 0:
        return np.full(n_bins, float('inf'))
    # Filter by range
    ranges = np.hypot(pts[:, 0], pts[:, 1])
    r_mask = (ranges >= range_min) & (ranges <= range_max)
    pts = pts[r_mask]
    ranges = ranges[r_mask]
    if len(pts) == 0:
        return np.full(n_bins, float('inf'))
    # Compute azimuth and bin index
    azimuths = np.arctan2(pts[:, 1], pts[:, 0])
    a_span = angle_max - angle_min
    bin_indices = ((azimuths - angle_min) / a_span * n_bins).astype(int)
    valid = (bin_indices >= 0) & (bin_indices < n_bins)
    ranges_out = np.full(n_bins, float('inf'))
    for i, r, v in zip(bin_indices[valid], ranges[valid], valid[valid]):
        if r < ranges_out[i]:
            ranges_out[i] = r
    return ranges_out


class LidarProcessor(Node):
    """3D LiDAR processor: PointCloud2 -> 2D LaserScan + health monitoring."""

    def __init__(self) -> None:
        """Initialize the LiDAR processor."""
        super().__init__('lidar_processor')

        # Parameters
        self.declare_parameter('lidar_frame', 'sih_ugv/base_link/lidar_3d')
        self.declare_parameter('angle_min', -2.35619)
        self.declare_parameter('angle_max', 2.35619)
        self.declare_parameter('scan_bins', 720)
        self.declare_parameter('range_min', 0.2)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('vertical_tolerance_deg', 5.0)
        # Stale detection: report LiDAR as stale if no PointCloud2 received
        # within this wall-clock interval.  Default 2.0s accommodates WSL2
        # simulation where Gazebo may run at RTF < 1.0 and the ROS bridge
        # delivery can lag.  A 1.0s timeout produced frequent false DEGRADED_SENSOR
        # transitions even while the system was actively navigating.
        self.declare_parameter('stale_timeout_sec', 2.0)
        self.declare_parameter('min_valid_ratio', 0.05)

        self._lidar_frame = str(self.get_parameter('lidar_frame').value)
        self._angle_min = float(self.get_parameter('angle_min').value)
        self._angle_max = float(self.get_parameter('angle_max').value)
        self._scan_bins = int(self.get_parameter('scan_bins').value)
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)
        self._vertical_tol = math.radians(
            float(self.get_parameter('vertical_tolerance_deg').value))
        self._stale_timeout = float(self.get_parameter('stale_timeout_sec').value)
        self._min_valid_ratio = float(self.get_parameter('min_valid_ratio').value)

        self._last_cloud_time: float | None = None
        self._sample_count: int = 0
        self._last_valid_ratio: float = 0.0

        # TF buffer for checking odom->base_link availability at scan timestamps.
        # SLAM's MessageFilter requires odom->base_link TF at the scan's exact
        # sim-timestamp.  We check this transform at each cloud's original
        # timestamp before publishing the scan.  Clouds captured before the EKF
        # starts publishing odom->base_link are dropped — they are genuinely
        # stale and restamping them would break temporal consistency.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscribers
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self._cloud_callback,
            10,
        )

        # Publishers
        self._scan_pub = self.create_publisher(LaserScan, '/lidar/scan_2d', 10)
        self._health_pub = self.create_publisher(SensorHealth, '/sensor_health/lidar', 10)

        # Health timer at 5 Hz
        self._health_timer = self.create_timer(0.2, self._publish_health)

        self.get_logger().info(
            f'LiDAR processor initialized: {self._scan_bins} bins, '
            f'VFOV±{math.degrees(self._vertical_tol):.0f}°'
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        """Process incoming 3D point cloud and publish 2D scan.

        SLAM's MessageFilter requires odom->base_link TF at the scan's exact
        sim-timestamp.  We check this transform at the cloud's original
        timestamp before publishing.  Clouds captured before the EKF starts
        are dropped — they are genuinely stale.  The original cloud timestamp
        is always preserved on the published scan to maintain temporal
        consistency between scan data and robot pose.
        """
        self._last_cloud_time = time.monotonic()
        self._sample_count += 1

        # Check if odom->base_link TF exists at this cloud's timestamp.
        # This is the same transform SLAM's MessageFilter will look up.
        # Using the cloud's original timestamp ensures we only publish
        # scans whose TF is available, preventing queue-full drops in SLAM.
        cloud_time = rclpy.time.Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
        )
        try:
            self._tf_buffer.lookup_transform(
                'odom', 'base_link',
                cloud_time,
                timeout=rclpy.duration.Duration(seconds=0.0),
            )
        except tf2_ros.ExtrapolationException:
            # TF not available at this timestamp yet (EKF hasn't caught up
            # or cloud predates EKF startup).  Drop the cloud — restamping
            # would break temporal consistency.
            self.get_logger().debug(
                f'Dropping cloud at t={msg.header.stamp.sec}.'
                f'{msg.header.stamp.nanosec:09d}s: '
                f'odom->base_link TF not available at this timestamp',
                throttle_duration_sec=5.0,
            )
            return
        except tf2_ros.LookupException:
            # TF chain does not exist at all — EKF not started yet.
            self.get_logger().debug(
                'Dropping cloud: odom->base_link TF chain not available',
                throttle_duration_sec=5.0,
            )
            return
        except tf2_ros.ConnectivityException:
            # TF chain is broken (missing intermediate frame).
            self.get_logger().debug(
                'Dropping cloud: odom->base_link TF chain broken',
                throttle_duration_sec=5.0,
            )
            return

        # TF is available at the cloud's timestamp — process and publish.
        self._process_cloud(msg)

    def _process_cloud(self, msg: PointCloud2) -> None:
        """Process a point cloud and publish the corresponding scan."""
        try:
            pts = self._fast_extract_xyz(msg)
        except Exception as exc:
            self.get_logger().warn(f'PointCloud2 parse error: {exc}',
                                   throttle_duration_sec=5.0)
            return

        n_total = len(pts)
        ranges = project_to_scan_2d(
            pts,
            self._angle_min,
            self._angle_max,
            self._scan_bins,
            self._range_min,
            self._range_max,
            self._vertical_tol,
        )

        valid = np.isfinite(ranges)
        self._last_valid_ratio = float(np.sum(valid)) / max(1, self._scan_bins)

        scan = LaserScan()
        # Preserve the original cloud timestamp (actual measurement time).
        # We only publish scans whose odom->base_link TF is available at this
        # timestamp, so SLAM's MessageFilter will succeed when it looks up the
        # transform.  This maintains temporal consistency: the scan data and
        # robot pose correspond to the same instant in simulation time.
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self._lidar_frame
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = (self._angle_max - self._angle_min) / max(
            1, self._scan_bins - 1)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = ranges.tolist()
        self._scan_pub.publish(scan)

        self.get_logger().debug(
            f'scan_2d: bins={self._scan_bins} valid_ratio={self._last_valid_ratio:.2f} '
            f'pts={n_total}',
            throttle_duration_sec=5.0,
        )

    def _fast_extract_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Extract XYZ from PointCloud2 using struct field offsets."""
        field_map = {f.name: f.offset for f in msg.fields}
        if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
            return np.empty((0, 3), dtype=np.float32)
        n_points = msg.width * msg.height
        if n_points == 0:
            return np.empty((0, 3), dtype=np.float32)
        ps = msg.point_step
        buf = bytes(msg.data)
        x_o, y_o, z_o = field_map['x'], field_map['y'], field_map['z']
        # Vectorized extraction using numpy strides
        data = np.frombuffer(buf, dtype=np.uint8)
        if len(data) < n_points * ps:
            return np.empty((0, 3), dtype=np.float32)
        reshaped = data[:n_points * ps].reshape(n_points, ps)
        xs = reshaped[:, x_o:x_o + 4].copy().view(np.float32).flatten()
        ys = reshaped[:, y_o:y_o + 4].copy().view(np.float32).flatten()
        zs = reshaped[:, z_o:z_o + 4].copy().view(np.float32).flatten()
        pts = np.column_stack([xs, ys, zs])
        finite = np.isfinite(pts).all(axis=1)
        return pts[finite]

    def _publish_health(self) -> None:
        """Publish LiDAR sensor health."""
        health = SensorHealth()
        health.stamp = self.get_clock().now().to_msg()
        health.sensor_name = 'lidar_3d'
        health.sample_count = self._sample_count

        if self._last_cloud_time is None:
            health.data_valid = False
            health.stale = True
            health.healthy = False
            health.age_sec = -1.0
            health.valid_ratio = 0.0
            health.reason = 'no data received'
        else:
            age = time.monotonic() - self._last_cloud_time
            health.age_sec = float(age)
            health.stale = age > self._stale_timeout
            health.valid_ratio = self._last_valid_ratio
            health.data_valid = self._last_valid_ratio >= self._min_valid_ratio
            health.healthy = health.data_valid and not health.stale
            if health.stale:
                health.reason = f'stale: {age:.1f}s > {self._stale_timeout}s'
            elif not health.data_valid:
                health.reason = f'low valid ratio: {self._last_valid_ratio:.2f}'
            else:
                health.reason = 'healthy'

        self._health_pub.publish(health)


def main(args=None) -> None:
    """Run the LiDAR processor node."""
    rclpy.init(args=args)
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
