#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


CLASS_FACTORS = {
    'FLAT': 1.00,
    'SLOPED': 0.80,
    'ROUGH': 0.60,
    'STEEP': 0.30,
    'UNKNOWN': 0.50,
}


def clamp01(value: float) -> float:
    return float(np.clip(value, 0.0, 1.0))


def terrain_factor(
    terrain_class: str,
    class_confidence: float,
    geometry_confidence: float,
    valid_ratio: float,
) -> float:
    """
    Convert terrain state into a multiplicative traversability factor.

    Confidence controls how strongly terrain classification modifies the
    camera-only baseline. At zero reliability the baseline is preserved.
    """
    base = CLASS_FACTORS.get(str(terrain_class).upper(), 0.50)

    reliability = (
        clamp01(class_confidence)
        * clamp01(geometry_confidence)
        * clamp01(valid_ratio)
    )

    factor = 1.0 - reliability * (1.0 - base)
    return float(np.clip(factor, 0.0, 1.0))


def apply_terrain_factor(
    baseline: np.ndarray,
    factor: float,
) -> np.ndarray:
    """Apply a scalar terrain factor to a mono8 0..255 score image."""
    factor = clamp01(factor)

    values = baseline.astype(np.float32) / 255.0
    adjusted = np.clip(values * factor, 0.0, 1.0)

    return np.rint(adjusted * 255.0).astype(np.uint8)


class TerrainAwareTraversability(Node):
    """
    Fuse the existing camera traversability baseline with P2.3 terrain state.

    Inputs:
      /traversability/score_map       sensor_msgs/msg/Image, mono8
      /terrain/class                  std_msgs/msg/String
      /terrain/class_confidence      std_msgs/msg/Float32
      /geometry/geometry_confidence   std_msgs/msg/Float32
      /geometry/valid_point_ratio     std_msgs/msg/Float32

    Outputs:
      /traversability/terrain_aware_score_map
      /traversability/terrain_aware_mean_score
      /traversability/terrain_factor
    """

    def __init__(self) -> None:
        super().__init__('terrain_aware_traversability')

        self.declare_parameter(
            'input_topic',
            '/traversability/score_map',
        )
        self.declare_parameter(
            'publish_hz',
            5.0,
        )
        self.declare_parameter(
            'max_input_age_s',
            1.0,
        )

        input_topic = str(
            self.get_parameter('input_topic').value
        )
        publish_hz = float(
            self.get_parameter('publish_hz').value
        )

        self.max_input_age_s = float(
            self.get_parameter('max_input_age_s').value
        )

        self.latest_score_map: Optional[Image] = None
        self.latest_class = 'UNKNOWN'
        self.latest_class_confidence = 0.0
        self.latest_geometry_confidence = 0.0
        self.latest_valid_ratio = 0.0

        self.score_stamp_ns = 0
        self.class_stamp_ns = 0
        self.class_conf_stamp_ns = 0
        self.geometry_conf_stamp_ns = 0
        self.valid_ratio_stamp_ns = 0

        self.frames = 0

        self.sub_score_map = self.create_subscription(
            Image,
            input_topic,
            self.score_callback,
            10,
        )

        self.sub_class = self.create_subscription(
            String,
            '/terrain/class',
            self.class_callback,
            10,
        )

        self.sub_class_confidence = self.create_subscription(
            Float32,
            '/terrain/class_confidence',
            self.class_confidence_callback,
            10,
        )

        self.sub_geometry_confidence = self.create_subscription(
            Float32,
            '/geometry/geometry_confidence',
            self.geometry_confidence_callback,
            10,
        )

        self.sub_valid_ratio = self.create_subscription(
            Float32,
            '/geometry/valid_point_ratio',
            self.valid_ratio_callback,
            10,
        )

        self.pub_score_map = self.create_publisher(
            Image,
            '/traversability/terrain_aware_score_map',
            10,
        )

        self.pub_mean_score = self.create_publisher(
            Float32,
            '/traversability/terrain_aware_mean_score',
            10,
        )

        self.pub_factor = self.create_publisher(
            Float32,
            '/traversability/terrain_factor',
            10,
        )

        period = 1.0 / max(publish_hz, 0.1)
        self.timer = self.create_timer(
            period,
            self.process,
        )

        self.get_logger().info(
            'Terrain-aware traversability started. '
            f'Input={input_topic} publish_hz={publish_hz:.2f}'
        )

    def now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def score_callback(self, msg: Image) -> None:
        self.latest_score_map = msg
        self.score_stamp_ns = self.now_ns()

    def class_callback(self, msg: String) -> None:
        self.latest_class = str(msg.data).strip().upper()
        self.class_stamp_ns = self.now_ns()

    def class_confidence_callback(self, msg: Float32) -> None:
        self.latest_class_confidence = clamp01(float(msg.data))
        self.class_conf_stamp_ns = self.now_ns()

    def geometry_confidence_callback(self, msg: Float32) -> None:
        self.latest_geometry_confidence = clamp01(float(msg.data))
        self.geometry_conf_stamp_ns = self.now_ns()

    def valid_ratio_callback(self, msg: Float32) -> None:
        self.latest_valid_ratio = clamp01(float(msg.data))
        self.valid_ratio_stamp_ns = self.now_ns()

    def fresh(self, stamp_ns: int) -> bool:
        if stamp_ns <= 0:
            return False

        age_s = (
            self.now_ns() - stamp_ns
        ) / 1e9

        return 0.0 <= age_s <= self.max_input_age_s

    def process(self) -> None:
        if self.latest_score_map is None:
            return

        required_fresh = all(
            (
                self.fresh(self.score_stamp_ns),
                self.fresh(self.class_stamp_ns),
                self.fresh(self.class_conf_stamp_ns),
                self.fresh(self.geometry_conf_stamp_ns),
                self.fresh(self.valid_ratio_stamp_ns),
            )
        )

        # A stale or incomplete terrain signal must not invent a terrain
        # penalty. The baseline is preserved until all upstream values are
        # current again.
        if required_fresh:
            factor = terrain_factor(
                self.latest_class,
                self.latest_class_confidence,
                self.latest_geometry_confidence,
                self.latest_valid_ratio,
            )
            terrain_state = self.latest_class
        else:
            factor = 1.0
            terrain_state = 'WAITING'

        msg = self.latest_score_map

        if msg.encoding.lower() not in ('mono8', '8uc1'):
            self.get_logger().warning(
                f'Unsupported score-map encoding: {msg.encoding}'
            )
            return

        expected = int(msg.height) * int(msg.step)

        raw = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )

        if raw.size < expected:
            self.get_logger().warning(
                'Score-map buffer is smaller than expected.'
            )
            return

        image = raw[:expected].reshape(
            int(msg.height),
            int(msg.step),
        )[:, : int(msg.width)]

        adjusted = apply_terrain_factor(
            image,
            factor,
        )

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = 'mono8'
        out.is_bigendian = msg.is_bigendian
        out.step = msg.width
        out.data = adjusted.tobytes()

        self.pub_score_map.publish(out)

        valid = adjusted.astype(np.float32) / 255.0
        nonzero = valid > 0.0

        mean_score = (
            float(np.mean(valid[nonzero]))
            if np.any(nonzero)
            else 0.0
        )

        mean_msg = Float32()
        mean_msg.data = clamp01(mean_score)
        self.pub_mean_score.publish(mean_msg)

        factor_msg = Float32()
        factor_msg.data = factor
        self.pub_factor.publish(factor_msg)

        self.frames += 1

        if self.frames % 10 == 0:
            self.get_logger().info(
                f'frames={self.frames} '
                f'class={terrain_state} '
                f'class_confidence={self.latest_class_confidence:.3f} '
                f'geometry_confidence={self.latest_geometry_confidence:.3f} '
                f'valid_ratio={self.latest_valid_ratio:.3f} '
                f'terrain_factor={factor:.3f} '
                f'mean_score={mean_score:.3f}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TerrainAwareTraversability()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
