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
"""DRISTHI 3D obstacle extractor.

Extracts non-ground obstacle clusters from /lidar/points (PointCloud2).
Publishes TrackedObstacleArray to /obstacles/raw (pre-tracking detections).
Uses only sensed LiDAR data — no Gazebo ground-truth.
"""

import math
import time
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sih26126_msgs.msg import TrackedObstacle, TrackedObstacleArray
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def extract_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract finite XYZ points from a PointCloud2 message."""
    field_map = {f.name: f.offset for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32)
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    ps = msg.point_step
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if len(buf) < n * ps:
        return np.empty((0, 3), dtype=np.float32)
    flat = buf[:n * ps].reshape(n, ps)
    xs = flat[:, field_map['x']:field_map['x'] + 4].copy().view(np.float32).flatten()
    ys = flat[:, field_map['y']:field_map['y'] + 4].copy().view(np.float32).flatten()
    zs = flat[:, field_map['z']:field_map['z'] + 4].copy().view(np.float32).flatten()
    pts = np.column_stack([xs, ys, zs])
    return pts[np.isfinite(pts).all(axis=1)]


def remove_ground(
    pts: np.ndarray,
    ground_threshold: float = 0.25,
    n_sectors: int = 36,
) -> np.ndarray:
    """Remove ground points using per-sector minimum-Z estimation."""
    if len(pts) == 0:
        return pts
    azimuths = np.arctan2(pts[:, 1], pts[:, 0])
    sector_width = 2.0 * math.pi / n_sectors
    sector_idx = ((azimuths + math.pi) / sector_width).astype(int) % n_sectors
    obstacle_mask = np.ones(len(pts), dtype=bool)
    for s in range(n_sectors):
        in_sector = sector_idx == s
        if not np.any(in_sector):
            continue
        sector_z = pts[in_sector, 2]
        ground_z = float(np.percentile(sector_z, 10))
        obstacle_mask[in_sector] = (pts[in_sector, 2] - ground_z) > ground_threshold
    return pts[obstacle_mask]


def voxel_cluster(
    pts: np.ndarray,
    voxel_size: float = 0.25,
    min_pts: int = 5,
) -> List[np.ndarray]:
    """Cluster obstacle points using voxel grid + connected components."""
    if len(pts) < min_pts:
        return []
    voxel_indices = (pts[:, :3] / voxel_size).astype(int)
    voxel_map: dict = {}
    for i, vi in enumerate(map(tuple, voxel_indices)):
        voxel_map.setdefault(vi, []).append(i)
    visited: set = set()
    clusters: List[np.ndarray] = []
    for seed in list(voxel_map.keys()):
        if seed in visited:
            continue
        cluster_voxels = [seed]
        queue = [seed]
        visited.add(seed)
        while queue:
            v = queue.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == dy == dz == 0:
                            continue
                        nb = (v[0] + dx, v[1] + dy, v[2] + dz)
                        if nb in voxel_map and nb not in visited:
                            visited.add(nb)
                            cluster_voxels.append(nb)
                            queue.append(nb)
        pt_indices: List[int] = []
        for vx in cluster_voxels:
            pt_indices.extend(voxel_map[vx])
        if len(pt_indices) >= min_pts:
            clusters.append(pts[pt_indices])
    return clusters


def cluster_to_detection(cluster: np.ndarray, frame_id: str) -> TrackedObstacle:
    """Convert a point cluster to a TrackedObstacle detection."""
    centroid = cluster.mean(axis=0)
    mn = cluster.min(axis=0)
    mx = cluster.max(axis=0)
    dims = mx - mn
    n_pts = len(cluster)
    confidence = min(1.0, n_pts / 50.0)
    cx_r = round(float(centroid[0]), 1)
    cy_r = round(float(centroid[1]), 1)
    cz_r = round(float(centroid[2]), 1)
    track_id = f'raw_{hash((cx_r, cy_r, cz_r)) & 0xFFFF:04x}'
    obs = TrackedObstacle()
    obs.id = track_id
    obs.position = Point(x=float(centroid[0]), y=float(centroid[1]),
                         z=float(centroid[2]))
    obs.velocity = Vector3(x=0.0, y=0.0, z=0.0)
    obs.predicted_position = Point(x=float(centroid[0]),
                                   y=float(centroid[1]),
                                   z=float(centroid[2]))
    obs.dimensions = Vector3(x=float(max(dims[0], 0.1)),
                             y=float(max(dims[1], 0.1)),
                             z=float(max(dims[2], 0.1)))
    obs.confidence = float(confidence)
    obs.age_sec = 0.0
    obs.last_seen_sec = 0.0
    obs.is_dynamic = False
    obs.classification = 'UNKNOWN'
    obs.frame_id = frame_id
    return obs


class ObstacleExtractor(Node):
    """Extract obstacle clusters from 3D LiDAR for the DRISTHI tracker."""

    def __init__(self) -> None:
        """Initialize the obstacle extractor."""
        super().__init__('obstacle_extractor')

        self.declare_parameter('ground_threshold', 0.25)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('voxel_size', 0.25)
        self.declare_parameter('min_cluster_pts', 5)
        self.declare_parameter('max_obstacle_height', 3.0)
        self.declare_parameter('lidar_frame', 'sih_ugv/base_link/lidar_3d')

        self._ground_thresh = float(self.get_parameter('ground_threshold').value)
        self._min_range = float(self.get_parameter('min_range').value)
        self._max_range = float(self.get_parameter('max_range').value)
        self._voxel_size = float(self.get_parameter('voxel_size').value)
        self._min_pts = int(self.get_parameter('min_cluster_pts').value)
        self._max_height = float(self.get_parameter('max_obstacle_height').value)
        self._lidar_frame = str(self.get_parameter('lidar_frame').value)

        self._cloud_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self._cloud_cb, 5)
        self._raw_pub = self.create_publisher(
            TrackedObstacleArray, '/obstacles/raw', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/obstacles/markers', 10)

        self._frame_count = 0
        self.get_logger().info(
            f'Obstacle extractor initialized: ground_thresh={self._ground_thresh}m '
            f'range={self._min_range}-{self._max_range}m'
        )

    def _cloud_cb(self, msg: PointCloud2) -> None:
        """Process incoming point cloud and publish detections."""
        self._frame_count += 1
        pts = extract_xyz(msg)
        if len(pts) == 0:
            self._publish_empty(msg.header)
            return
        ranges = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[(ranges >= self._min_range) & (ranges <= self._max_range)]
        pts = pts[pts[:, 2] <= self._max_height]
        if len(pts) < self._min_pts:
            self._publish_empty(msg.header)
            return
        obstacle_pts = remove_ground(pts, self._ground_thresh)
        if len(obstacle_pts) < self._min_pts:
            self._publish_empty(msg.header)
            return
        clusters = voxel_cluster(obstacle_pts, self._voxel_size, self._min_pts)
        detections = [cluster_to_detection(c, self._lidar_frame) for c in clusters]
        for det in detections:
            det.stamp = msg.header.stamp
        arr = TrackedObstacleArray()
        arr.header = msg.header
        arr.obstacles = detections
        arr.total_active_tracks = len(detections)
        self._raw_pub.publish(arr)
        self._publish_markers(detections, msg.header)
        if self._frame_count % 50 == 0:
            self.get_logger().info(
                f'extractor: frame={self._frame_count} '
                f'pts={len(pts)} obs_pts={len(obstacle_pts)} '
                f'clusters={len(clusters)}',
                throttle_duration_sec=5.0,
            )

    def _publish_empty(self, header) -> None:
        """Publish empty detection array."""
        arr = TrackedObstacleArray()
        arr.header = header
        arr.obstacles = []
        arr.total_active_tracks = 0
        self._raw_pub.publish(arr)

    def _publish_markers(
        self,
        detections: List[TrackedObstacle],
        header,
    ) -> None:
        """Publish RViz markers for detected obstacle clusters."""
        markers = []
        del_m = Marker()
        del_m.header = header
        del_m.ns = 'obstacles_raw'
        del_m.id = 9999
        del_m.action = Marker.DELETEALL
        markers.append(del_m)
        for i, det in enumerate(detections):
            m = Marker()
            m.header = header
            m.ns = 'obstacles_raw'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = det.position
            m.pose.orientation.w = 1.0
            m.scale.x = max(0.2, det.dimensions.x)
            m.scale.y = max(0.2, det.dimensions.y)
            m.scale.z = max(0.2, det.dimensions.z)
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.6
            m.lifetime.sec = 1
            markers.append(m)
        ma = MarkerArray()
        ma.markers = markers
        self._marker_pub.publish(ma)


def main(args=None) -> None:
    """Run the obstacle extractor node."""
    rclpy.init(args=args)
    node = ObstacleExtractor()
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
