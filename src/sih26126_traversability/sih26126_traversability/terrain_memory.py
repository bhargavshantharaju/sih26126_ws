#!/usr/bin/env python3
"""
Terrain Memory node for DRISTHI.

Maintains a grid-based memory of terrain classifications with
time-decay, distinguishing unknown cells from confirmed hazards.
New safe observations can override stale unsafe assessments.

Publishes a per-cell traversability score on /terrain/memory_score
and an optional memory image on /terrain/memory_map.
"""

import math
import time
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from std_msgs.msg import Float32, String

TERRAIN_UNKNOWN = 0
TERRAIN_FLAT = 1
TERRAIN_SLOPED = 2
TERRAIN_ROUGH = 3
TERRAIN_STEEP = 4


def terrain_class_to_value(class_str: str) -> int:
    """Convert a terrain-class string to its integer constant."""
    mapping = {
        'UNKNOWN': TERRAIN_UNKNOWN,
        'FLAT': TERRAIN_FLAT,
        'SLOPED': TERRAIN_SLOPED,
        'ROUGH': TERRAIN_ROUGH,
        'STEEP': TERRAIN_STEEP,
    }
    return mapping.get(class_str, TERRAIN_UNKNOWN)


def terrain_value_to_traversability(
    terrain_value: int,
    confidence: float,
) -> float:
    """
    Convert a terrain class value and confidence to a traversability score.

    Returns 0.5 (neutral) when confidence is below 0.01.  Otherwise,
    blends a class-specific base score with 0.5 (neutral) weighted by
    confidence, so low-confidence observations approach neutral.
    """
    if confidence < 0.01:
        return 0.5

    base_scores = {
        TERRAIN_UNKNOWN: 0.5,
        TERRAIN_FLAT: 1.0,
        TERRAIN_SLOPED: 0.7,
        TERRAIN_ROUGH: 0.4,
        TERRAIN_STEEP: 0.1,
    }

    base = base_scores.get(terrain_value, 0.5)
    return 0.5 * (1.0 - confidence) + base * confidence


class TerrainCell:
    """Single cell in the terrain memory grid."""

    def __init__(self):
        """Initialise cell with unknown terrain and zero confidence."""
        self.terrain_class: int = TERRAIN_UNKNOWN
        self.confidence: float = 0.0
        self.last_updated: float = 0.0
        self.update_count: int = 0

    def update(
        self,
        terrain_class: int,
        confidence: float,
        timestamp: float,
    ):
        """Record a new terrain observation for this cell."""
        self.terrain_class = terrain_class
        self.confidence = max(0.0, min(1.0, confidence))
        self.last_updated = timestamp
        self.update_count += 1

    def get_decayed_confidence(
        self,
        current_time: float,
        half_life_sec: float,
    ) -> float:
        """Return confidence after applying exponential time decay."""
        if self.confidence < 0.01:
            return 0.0

        age = current_time - self.last_updated
        if age < 0.0:
            return self.confidence

        if age > 5.0 * half_life_sec:
            return 0.0

        decay_factor = math.exp(-age * 0.693147 / half_life_sec)
        return self.confidence * decay_factor


class TerrainMemory(Node):
    """Maintain a spatial-temporal terrain confidence map with decay."""

    def __init__(self):
        """Initialise subscriptions, publisher, and periodic timer."""
        super().__init__('terrain_memory')

        self.declare_parameter('cell_size_m', 0.1)
        self.declare_parameter('confidence_half_life_sec', 30.0)
        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('map_timeout_sec', 5.0)
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('terrain_timeout_sec', 2.0)

        self.cells: Dict[Tuple[int, int], TerrainCell] = {}
        self.pose: Optional[PoseStamped] = None
        self.map: Optional[OccupancyGrid] = None
        self.current_terrain_class: str = 'UNKNOWN'
        self.current_terrain_confidence: float = 0.0

        self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.pose_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10,
        )
        self.create_subscription(
            String,
            '/terrain/class',
            self.terrain_class_callback,
            10,
        )
        self.create_subscription(
            Float32,
            '/terrain/class_confidence',
            self.terrain_confidence_callback,
            10,
        )

        self.terrain_memory_pub = self.create_publisher(
            Image,
            '/terrain/memory_map',
            10,
        )
        self.terrain_score_pub = self.create_publisher(
            Float32,
            '/terrain/memory_score',
            10,
        )

        update_rate = self.get_parameter('update_rate_hz').value
        self.create_timer(1.0 / update_rate, self.process)

        self.get_logger().info('Terrain memory node started')

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to integer cell indices."""
        cell_size = self.get_parameter('cell_size_m').value
        return (
            int(math.floor(x / cell_size)),
            int(math.floor(y / cell_size)),
        )

    def pose_callback(self, msg: PoseStamped):
        """Track the current robot pose."""
        self.pose = msg

    def map_callback(self, msg: OccupancyGrid):
        """Track the current occupancy map."""
        self.map = msg

    def terrain_class_callback(self, msg: String):
        """Update the current terrain classification string."""
        self.current_terrain_class = msg.data

    def terrain_confidence_callback(self, msg: Float32):
        """Update the current terrain classification confidence."""
        self.current_terrain_confidence = msg.data

    def process(self):
        """Update terrain memory and publish the current cell score."""
        current_time = time.time()

        if not self.data_is_fresh(current_time):
            return

        if self.pose is not None:
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            cell_key = self.world_to_cell(x, y)

            terrain_value = terrain_class_to_value(
                self.current_terrain_class
            )

            if cell_key not in self.cells:
                self.cells[cell_key] = TerrainCell()

            self.cells[cell_key].update(
                terrain_value,
                self.current_terrain_confidence,
                current_time,
            )

        score = self.get_current_terrain_score(current_time)
        if score is not None:
            score_msg = Float32()
            score_msg.data = float(score)
            self.terrain_score_pub.publish(score_msg)

    def data_is_fresh(self, current_time: float) -> bool:
        """Return True only when pose data is recent enough."""
        if self.pose is None:
            return False

        pose_age = (
            current_time
            - self.pose.header.stamp.sec
            - self.pose.header.stamp.nanosec * 1e-9
        )
        pose_timeout = self.get_parameter('pose_timeout_sec').value

        if pose_age > pose_timeout or pose_age < 0.0:
            return False

        return True

    def get_current_terrain_score(
        self,
        current_time: float,
    ) -> Optional[float]:
        """Return 0.0-1.0 traversability score at current pose, or None."""
        if self.pose is None:
            return None

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        cell_key = self.world_to_cell(x, y)

        if cell_key not in self.cells:
            return 0.5

        cell = self.cells[cell_key]
        half_life = self.get_parameter('confidence_half_life_sec').value
        decayed_conf = cell.get_decayed_confidence(current_time, half_life)

        return terrain_value_to_traversability(
            cell.terrain_class, decayed_conf
        )

    def get_cell_score(
        self,
        x: float,
        y: float,
        current_time: float,
    ) -> float:
        """
        Return terrain score for an arbitrary world position.

        Returns 0.5 (neutral) for cells with no recorded data.
        """
        cell_key = self.world_to_cell(x, y)

        if cell_key not in self.cells:
            return 0.5

        cell = self.cells[cell_key]
        half_life = self.get_parameter('confidence_half_life_sec').value
        decayed_conf = cell.get_decayed_confidence(current_time, half_life)

        return terrain_value_to_traversability(
            cell.terrain_class, decayed_conf
        )


def main(args=None):
    """Spin the TerrainMemory node."""
    rclpy.init(args=args)
    node = TerrainMemory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
