#!/usr/bin/env python3
"""
Cost Map Fusion node for DRISTHI.

Combines occupancy grid, terrain memory scores, and localization
confidence into a unified /map/cost_map for risk-aware global path
planning.  Cell values in the output OccupancyGrid represent traversal
cost scaled to [0, 100].
"""

from typing import Optional

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32


def occupancy_to_cost(occupancy_value: int) -> float:
    """
    Convert an occupancy grid cell value to a cost in [0.0, 1.0].

    -1 (unknown) maps to 0.5 (cautious), values 0-49 map
    proportionally to [0.0, 0.49] (free), and values 50-100 map
    to 1.0 (occupied/blocked).
    """
    if occupancy_value < 0:
        return 0.5
    elif occupancy_value >= 50:
        return 1.0
    else:
        return float(occupancy_value) / 100.0


def terrain_score_to_cost(terrain_score: float) -> float:
    """
    Convert a terrain traversability score to a cost in [0.0, 1.0].

    Inverts the score so that 1.0 (safe terrain) becomes 0.0 (low
    cost) and 0.0 (unsafe terrain) becomes 1.0 (high cost).
    """
    return 1.0 - terrain_score


def localization_confidence_to_cost(confidence: float) -> float:
    """
    Convert localisation confidence to an uncertainty cost multiplier.

    Returns 1.0 when confidence is 1.0 (no penalty) and 2.0 when
    confidence is 0.0 (double cost due to poor localisation).
    """
    return 1.0 + (1.0 - confidence)


def fuse_costs(
    occupancy_cost: float,
    terrain_cost: float,
    localization_multiplier: float,
    occupancy_weight: float = 0.5,
    terrain_weight: float = 0.3,
    localization_weight: float = 0.2,
) -> float:
    """
    Fuse multiple cost components into a unified cost value in [0, 1].

    Computes a weighted combination of occupancy and terrain costs,
    then applies a localisation-uncertainty multiplier.  Weights should
    sum to 1.0 for a balanced fusion.  The result is clamped to [0, 1].
    """
    base_cost = (
        occupancy_weight * occupancy_cost
        + terrain_weight * terrain_cost
    )
    fused = base_cost * (
        1.0 + localization_weight * (localization_multiplier - 1.0)
    )
    return max(0.0, min(1.0, fused))


class CostMapFusion(Node):
    """Fuse occupancy, terrain, and localisation into a unified cost map."""

    def __init__(self):
        """Initialise subscriptions, publisher, and periodic timer."""
        super().__init__('cost_map_fusion')

        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('occupancy_weight', 0.5)
        self.declare_parameter('terrain_weight', 0.3)
        self.declare_parameter('localization_weight', 0.2)
        self.declare_parameter('map_timeout_sec', 5.0)
        self.declare_parameter('terrain_timeout_sec', 10.0)
        self.declare_parameter('confidence_timeout_sec', 2.0)
        self.declare_parameter('default_terrain_score', 0.5)
        self.declare_parameter('default_localization_confidence', 0.8)

        self.occupancy_map: Optional[OccupancyGrid] = None
        self.terrain_scores: dict = {}
        self.latest_terrain_score: float = self.get_parameter(
            'default_terrain_score').value
        self.last_terrain_update: float = 0.0
        self.localization_confidence: float = self.get_parameter(
            'default_localization_confidence'
        ).value
        self.last_confidence_update: float = 0.0
        self.pose: Optional[PoseStamped] = None

        # Use TRANSIENT_LOCAL to match slam_toolbox's /map publication QoS
        # and frontier_explorer's /map/cost_map subscription QoS.
        map_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_callback,
            map_qos,
        )
        self.create_subscription(
            Float32,
            '/localization/confidence',
            self.confidence_callback,
            10,
        )
        self.create_subscription(
            Float32,
            '/terrain/memory_score',
            self.terrain_score_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.pose_callback,
            10,
        )

        self.cost_map_pub = self.create_publisher(
            OccupancyGrid,
            '/map/cost_map',
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        update_rate = self.get_parameter('update_rate_hz').value
        self.create_timer(1.0 / update_rate, self.process)

        self.get_logger().info('Cost map fusion node started')

    def occupancy_callback(self, msg: OccupancyGrid):
        """Update stored occupancy map and trigger a cost map update."""
        old_dims = (
            self.occupancy_map.info.width,
            self.occupancy_map.info.height,
        ) if self.occupancy_map is not None else (0, 0)
        self.occupancy_map = msg
        new_dims = (msg.info.width, msg.info.height)
        if new_dims != old_dims:
            # Map dimensions changed (SLAM grew the map): publish the updated
            # cost map immediately so frontier_explorer sees matching dimensions
            # on the next cycle rather than having to wait for the next timer tick.
            self.process()

    def confidence_callback(self, msg: Float32):
        """Update localisation confidence and record the timestamp."""
        self.localization_confidence = max(0.0, min(1.0, msg.data))
        self.last_confidence_update = (
            self.get_clock().now().nanoseconds * 1e-9
        )

    def terrain_score_callback(self, msg: Float32):
        """Store the latest terrain traversability score for cost fusion."""
        score = float(msg.data)
        if not (score == score):  # NaN guard
            return
        self.latest_terrain_score = max(0.0, min(1.0, score))
        self.last_terrain_update = (
            self.get_clock().now().nanoseconds * 1e-9
        )

    def pose_callback(self, msg: PoseStamped):
        """Track the current robot pose."""
        self.pose = msg

    def process(self):
        """Generate and publish the fused cost map."""
        if self.occupancy_map is None:
            return

        current_time = self.get_clock().now().nanoseconds * 1e-9
        confidence_timeout = self.get_parameter(
            'confidence_timeout_sec'
        ).value
        confidence_age = current_time - self.last_confidence_update

        if confidence_age > confidence_timeout:
            loc_confidence = self.get_parameter(
                'default_localization_confidence'
            ).value
        else:
            loc_confidence = self.localization_confidence

        loc_multiplier = localization_confidence_to_cost(loc_confidence)

        occupancy_weight = self.get_parameter('occupancy_weight').value
        terrain_weight = self.get_parameter('terrain_weight').value
        localization_weight = self.get_parameter(
            'localization_weight'
        ).value
        default_terrain_score = self.get_parameter(
            'default_terrain_score'
        ).value

        # Use the stored terrain score when it is fresh; fall back to the
        # default (0.5 = cautious) when terrain input has timed out.
        terrain_timeout = self.get_parameter('terrain_timeout_sec').value
        terrain_age = current_time - self.last_terrain_update
        if self.last_terrain_update > 0.0 and terrain_age <= terrain_timeout:
            effective_terrain_score = self.latest_terrain_score
        else:
            if self.last_terrain_update > 0.0:
                self.get_logger().warn(
                    f'Terrain score stale '
                    f'({terrain_age:.1f}s > {terrain_timeout:.1f}s);'
                    f' using default {default_terrain_score}',
                    throttle_duration_sec=10.0,
                )
            effective_terrain_score = default_terrain_score

        occupancy_data = np.array(
            self.occupancy_map.data, dtype=np.int8
        )
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height

        # Vectorized cost computation (replaces the slow per-cell Python loop).
        # Occupancy cost: -1 → 0.5, 0-49 → 0.0-0.49, 50-100 → 1.0
        occ_float = occupancy_data.astype(np.float32)
        occ_cost = np.where(
            occ_float < 0,
            0.5,
            np.where(occ_float >= 50, 1.0, occ_float / 100.0)
        )

        # Terrain cost: use the received terrain score (spatial terrain integrated)
        terr_cost = np.full(width * height, 1.0 - effective_terrain_score,
                            dtype=np.float32)

        # Fused cost (vectorized version of fuse_costs)
        base_cost = occupancy_weight * occ_cost + terrain_weight * terr_cost
        fused = base_cost * (1.0 + localization_weight * (loc_multiplier - 1.0))
        fused = np.clip(fused, 0.0, 1.0)
        cost_data = (fused * 100.0).astype(np.int8)

        cost_map = OccupancyGrid()
        cost_map.header = self.occupancy_map.header
        cost_map.header.stamp = self.get_clock().now().to_msg()
        cost_map.info = self.occupancy_map.info
        cost_map.data = cost_data.tolist()
        self.cost_map_pub.publish(cost_map)


def main(args=None):
    """Spin the CostMapFusion node."""
    rclpy.init(args=args)
    node = CostMapFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
