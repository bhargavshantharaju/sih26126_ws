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
"""DRISTHI dynamic obstacle tracker.

Maintains tracks of detected obstacles, estimates velocity, classifies
static vs dynamic, and publishes risk to /obstacles/dynamic_risk.
Consumes /obstacles/raw (detections) → publishes /obstacles/tracked.
"""

import math
import time
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sih26126_msgs.msg import TrackedObstacle, TrackedObstacleArray
from std_msgs.msg import Float32, Header


class _Track:
    """Internal mutable track state for one obstacle."""

    __slots__ = (
        'track_id', 'x', 'y', 'z',
        'vx', 'vy', 'vz',
        'dim_x', 'dim_y', 'dim_z',
        'confidence', 'is_dynamic', 'classification',
        'frame_id', 'created_at', 'last_seen', 'update_count', 'alpha',
    )

    def __init__(
        self,
        track_id: str,
        x: float, y: float, z: float,
        dim_x: float, dim_y: float, dim_z: float,
        confidence: float,
        frame_id: str,
        alpha: float = 0.3,
    ) -> None:
        """Create a new track from a first detection."""
        now = time.monotonic()
        self.track_id = track_id
        self.x, self.y, self.z = x, y, z
        self.vx = self.vy = self.vz = 0.0
        self.dim_x, self.dim_y, self.dim_z = dim_x, dim_y, dim_z
        self.confidence = confidence
        self.is_dynamic = False
        self.classification = 'UNKNOWN'
        self.frame_id = frame_id
        self.created_at = now
        self.last_seen = now
        self.update_count = 0
        self.alpha = alpha

    def update(
        self,
        x: float, y: float, z: float,
        dim_x: float, dim_y: float, dim_z: float,
        confidence: float,
        dt: float,
        dyn_thresh: float,
    ) -> None:
        """Update track state from a new detection."""
        if dt > 0.05:
            ax, ay, az = self.alpha, self.alpha, self.alpha
            self.vx = ax * ((x - self.x) / dt) + (1.0 - ax) * self.vx
            self.vy = ay * ((y - self.y) / dt) + (1.0 - ay) * self.vy
            self.vz = az * ((z - self.z) / dt) + (1.0 - az) * self.vz
        self.x, self.y, self.z = x, y, z
        self.dim_x = 0.7 * self.dim_x + 0.3 * dim_x
        self.dim_y = 0.7 * self.dim_y + 0.3 * dim_y
        self.dim_z = 0.7 * self.dim_z + 0.3 * dim_z
        self.confidence = min(
            1.0,
            0.8 * self.confidence + 0.2 * confidence
            + 0.01 * min(self.update_count, 20),
        )
        self.last_seen = time.monotonic()
        self.update_count += 1
        speed = math.hypot(self.vx, self.vy)
        self.is_dynamic = speed > dyn_thresh
        self.classification = 'DYNAMIC' if self.is_dynamic else 'STATIC'

    def to_msg(self, stamp, predict_h: float) -> TrackedObstacle:
        """Convert track to TrackedObstacle ROS message."""
        now = time.monotonic()
        msg = TrackedObstacle()
        msg.stamp = stamp
        msg.id = self.track_id
        msg.position = Point(x=self.x, y=self.y, z=self.z)
        msg.velocity = Vector3(x=self.vx, y=self.vy, z=self.vz)
        msg.predicted_position = Point(
            x=self.x + self.vx * predict_h,
            y=self.y + self.vy * predict_h,
            z=self.z + self.vz * predict_h,
        )
        msg.dimensions = Vector3(x=self.dim_x, y=self.dim_y, z=self.dim_z)
        msg.confidence = float(self.confidence)
        msg.age_sec = float(now - self.created_at)
        msg.last_seen_sec = float(now - self.last_seen)
        msg.is_dynamic = self.is_dynamic
        msg.classification = self.classification
        msg.frame_id = self.frame_id
        return msg


class ObstacleTracker(Node):
    """Track obstacles from /obstacles/raw → /obstacles/tracked."""

    def __init__(self) -> None:
        """Initialize the obstacle tracker with configurable parameters."""
        super().__init__('obstacle_tracker')

        self.declare_parameter('track_timeout_sec', 3.0)
        self.declare_parameter('association_max_dist', 1.5)
        self.declare_parameter('dynamic_velocity_threshold', 0.15)
        self.declare_parameter('predict_horizon_sec', 1.0)
        self.declare_parameter('max_tracks', 50)
        self.declare_parameter('velocity_alpha', 0.3)

        self._timeout = float(self.get_parameter('track_timeout_sec').value)
        self._assoc_dist = float(self.get_parameter('association_max_dist').value)
        self._dyn_thresh = float(
            self.get_parameter('dynamic_velocity_threshold').value)
        self._predict_h = float(self.get_parameter('predict_horizon_sec').value)
        self._max_tracks = int(self.get_parameter('max_tracks').value)
        self._vel_alpha = float(self.get_parameter('velocity_alpha').value)

        self._tracks: Dict[str, _Track] = {}
        self._last_det_times: Dict[str, float] = {}
        self._next_id = 0

        self._raw_sub = self.create_subscription(
            TrackedObstacleArray, '/obstacles/raw', self._raw_cb, 5)
        self._tracked_pub = self.create_publisher(
            TrackedObstacleArray, '/obstacles/tracked', 10)
        self._risk_pub = self.create_publisher(
            Float32, '/obstacles/dynamic_risk', 10)

        # Periodic housekeeping: remove stale tracks
        self._timer = self.create_timer(0.2, self._housekeeping)
        self.get_logger().info(
            f'Obstacle tracker initialized: timeout={self._timeout}s '
            f'dyn_thresh={self._dyn_thresh}m/s'
        )

    def _raw_cb(self, msg: TrackedObstacleArray) -> None:
        """Process raw detections and update track dictionary."""
        now = time.monotonic()
        used_tracks: set = set()
        unmatched: List[TrackedObstacle] = []

        for det in msg.obstacles:
            best_id: Optional[str] = None
            best_dist = self._assoc_dist
            for tid, track in self._tracks.items():
                if tid in used_tracks:
                    continue
                dist = math.sqrt(
                    (det.position.x - track.x) ** 2
                    + (det.position.y - track.y) ** 2
                    + (det.position.z - track.z) ** 2
                )
                if dist < best_dist:
                    best_dist = dist
                    best_id = tid

            if best_id is not None:
                track = self._tracks[best_id]
                dt = now - self._last_det_times.get(best_id, now)
                track.update(
                    det.position.x, det.position.y, det.position.z,
                    det.dimensions.x, det.dimensions.y, det.dimensions.z,
                    det.confidence, dt, self._dyn_thresh,
                )
                track.frame_id = det.frame_id
                self._last_det_times[best_id] = now
                used_tracks.add(best_id)
            else:
                unmatched.append(det)

        for det in unmatched:
            if len(self._tracks) >= self._max_tracks:
                break
            new_id = f'obs_{self._next_id:04d}'
            self._next_id += 1
            t = _Track(
                track_id=new_id,
                x=det.position.x, y=det.position.y, z=det.position.z,
                dim_x=det.dimensions.x, dim_y=det.dimensions.y,
                dim_z=det.dimensions.z,
                confidence=det.confidence,
                frame_id=det.frame_id,
                alpha=self._vel_alpha,
            )
            self._tracks[new_id] = t
            self._last_det_times[new_id] = now

        self._publish_tracks(msg.header)

    def _housekeeping(self) -> None:
        """Remove timed-out tracks and publish current state."""
        now = time.monotonic()
        stale = [tid for tid, t in self._tracks.items()
                 if (now - t.last_seen) > self._timeout]
        for tid in stale:
            del self._tracks[tid]
            self._last_det_times.pop(tid, None)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'sih_ugv/base_link/lidar_3d'
        self._publish_tracks(header)

    def _publish_tracks(self, header) -> None:
        """Publish tracked obstacles and dynamic risk score."""
        stamp = header.stamp
        msgs = [t.to_msg(stamp, self._predict_h) for t in self._tracks.values()]

        arr = TrackedObstacleArray()
        arr.header = header
        arr.obstacles = msgs
        arr.total_active_tracks = len(msgs)
        self._tracked_pub.publish(arr)

        dynamic_risk = 0.0
        for t in self._tracks.values():
            if t.is_dynamic:
                dist = math.hypot(t.x, t.y)
                risk = 1.0 / (1.0 + max(dist, 0.1))
                dynamic_risk = max(dynamic_risk, risk)

        risk_msg = Float32()
        risk_msg.data = float(min(1.0, dynamic_risk))
        self._risk_pub.publish(risk_msg)


def main(args=None) -> None:
    """Run the obstacle tracker node."""
    rclpy.init(args=args)
    node = ObstacleTracker()
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
