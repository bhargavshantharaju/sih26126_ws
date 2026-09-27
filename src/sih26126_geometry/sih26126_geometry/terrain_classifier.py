#!/usr/bin/env python3

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import String


# ============================================================
# Classification constants
# ============================================================

FLAT_MAX_SLOPE_DEG = 5.0
SLOPED_MAX_SLOPE_DEG = 30.0

ROUGHNESS_THRESHOLD_M = 0.05

MIN_GEOMETRY_CONFIDENCE = 0.30
MIN_VALID_POINT_RATIO = 0.20


CLASS_FLAT = "FLAT"
CLASS_SLOPED = "SLOPED"
CLASS_ROUGH = "ROUGH"
CLASS_STEEP = "STEEP"
CLASS_UNKNOWN = "UNKNOWN"


# ============================================================
# Pure classification logic
# ============================================================

def _closeness(value, threshold):
    """Return confidence-like closeness to a threshold."""
    if threshold <= 0.0:
        return 0.0

    return float(
        max(
            0.0,
            min(
                1.0,
                abs(value - threshold) / threshold,
            ),
        )
    )


def classify_terrain(
    slope_deg,
    roughness_m,
    geometry_confidence,
    valid_ratio,
):
    """
    Deterministically classify terrain geometry.

    Priority:
      1. invalid / low confidence -> UNKNOWN
      2. steep slope              -> STEEP
      3. high roughness           -> ROUGH
      4. moderate slope           -> SLOPED
      5. otherwise                -> FLAT

    Returns:
        (label, confidence)
    """

    values = (
        slope_deg,
        roughness_m,
        geometry_confidence,
        valid_ratio,
    )

    if not all(math.isfinite(float(v)) for v in values):
        return CLASS_UNKNOWN, 0.0

    if geometry_confidence < MIN_GEOMETRY_CONFIDENCE:
        return CLASS_UNKNOWN, float(
            max(0.0, geometry_confidence)
        )

    if valid_ratio < MIN_VALID_POINT_RATIO:
        return CLASS_UNKNOWN, float(
            max(0.0, valid_ratio)
        )

    slope_deg = max(0.0, float(slope_deg))
    roughness_m = max(0.0, float(roughness_m))
    geometry_confidence = float(
        max(0.0, min(1.0, geometry_confidence))
    )
    valid_ratio = float(
        max(0.0, min(1.0, valid_ratio))
    )

    # --------------------------------------------------------
    # STEEP
    # --------------------------------------------------------

    if slope_deg >= SLOPED_MAX_SLOPE_DEG:
        class_margin = min(
            1.0,
            slope_deg / 60.0,
        )

        confidence = (
            0.60 * class_margin
            + 0.40 * geometry_confidence
        )

        return CLASS_STEEP, float(
            max(0.0, min(1.0, confidence))
        )

    # --------------------------------------------------------
    # ROUGH
    # --------------------------------------------------------

    if roughness_m >= ROUGHNESS_THRESHOLD_M:
        rough_margin = min(
            1.0,
            roughness_m / 0.20,
        )

        confidence = (
            0.60 * rough_margin
            + 0.40 * geometry_confidence
        )

        return CLASS_ROUGH, float(
            max(0.0, min(1.0, confidence))
        )

    # --------------------------------------------------------
    # SLOPED
    # --------------------------------------------------------

    if slope_deg >= FLAT_MAX_SLOPE_DEG:
        slope_margin = (
            (slope_deg - FLAT_MAX_SLOPE_DEG)
            / (
                SLOPED_MAX_SLOPE_DEG
                - FLAT_MAX_SLOPE_DEG
            )
        )

        confidence = (
            0.60 * max(0.0, min(1.0, slope_margin))
            + 0.40 * geometry_confidence
        )

        return CLASS_SLOPED, float(
            max(0.0, min(1.0, confidence))
        )

    # --------------------------------------------------------
    # FLAT
    # --------------------------------------------------------

    flat_margin = 1.0 - (
        slope_deg / FLAT_MAX_SLOPE_DEG
    )

    rough_margin = 1.0 - (
        roughness_m / ROUGHNESS_THRESHOLD_M
    )

    confidence = (
        0.40 * max(0.0, min(1.0, flat_margin))
        + 0.20 * max(0.0, min(1.0, rough_margin))
        + 0.40 * geometry_confidence
    )

    return CLASS_FLAT, float(
        max(0.0, min(1.0, confidence))
    )


# ============================================================
# Image decoder
# ============================================================

def decode_float_image(msg):
    """
    Decode a ROS 32FC1 Image into a float32 numpy array.
    """
    if msg.encoding != "32FC1":
        raise ValueError(
            f"expected 32FC1, got {msg.encoding}"
        )

    expected_bytes = (
        msg.height
        * msg.step
    )

    if len(msg.data) < expected_bytes:
        raise ValueError(
            "image data shorter than declared step/height"
        )

    # step can be larger than width * 4 due to row padding.
    row_bytes = msg.step

    raw = np.frombuffer(
        msg.data,
        dtype=np.uint8,
        count=expected_bytes,
    )

    rows = raw.reshape(
        msg.height,
        row_bytes,
    )

    float_width_bytes = msg.width * 4

    values = np.frombuffer(
        rows[:, :float_width_bytes].tobytes(),
        dtype=np.float32,
    )

    return values.reshape(
        msg.height,
        msg.width,
    )


# ============================================================
# ROS 2 node
# ============================================================

class TerrainClassifierNode(Node):

    def __init__(self):
        super().__init__("terrain_classifier")

        self.declare_parameter(
            "slope_topic",
            "/geometry/slope_map",
        )

        self.declare_parameter(
            "roughness_topic",
            "/geometry/roughness_map",
        )

        self.declare_parameter(
            "confidence_topic",
            "/geometry/geometry_confidence",
        )

        self.declare_parameter(
            "valid_ratio_topic",
            "/geometry/valid_point_ratio",
        )

        self.declare_parameter(
            "publish_hz",
            5.0,
        )

        slope_topic = self.get_parameter(
            "slope_topic"
        ).value

        roughness_topic = self.get_parameter(
            "roughness_topic"
        ).value

        confidence_topic = self.get_parameter(
            "confidence_topic"
        ).value

        valid_ratio_topic = self.get_parameter(
            "valid_ratio_topic"
        ).value

        publish_hz = float(
            self.get_parameter("publish_hz").value
        )

        self.slope_map = None
        self.roughness_map = None
        self.geometry_confidence = None
        self.valid_ratio = None

        self.frames = 0

        self.slope_sub = self.create_subscription(
            Image,
            slope_topic,
            self.slope_callback,
            10,
        )

        self.roughness_sub = self.create_subscription(
            Image,
            roughness_topic,
            self.roughness_callback,
            10,
        )

        self.confidence_sub = self.create_subscription(
            Float32,
            confidence_topic,
            self.confidence_callback,
            10,
        )

        self.valid_ratio_sub = self.create_subscription(
            Float32,
            valid_ratio_topic,
            self.valid_ratio_callback,
            10,
        )

        self.class_pub = self.create_publisher(
            String,
            "/terrain/class",
            10,
        )

        self.confidence_pub = self.create_publisher(
            Float32,
            "/terrain/class_confidence",
            10,
        )

        period = 1.0 / max(
            0.1,
            publish_hz,
        )

        self.timer = self.create_timer(
            period,
            self.process,
        )

        self.get_logger().info(
            "Terrain classifier started"
        )

    def slope_callback(self, msg):
        try:
            self.slope_map = decode_float_image(msg)
        except Exception as exc:
            self.get_logger().warning(
                f"slope decode failed: {exc}"
            )

    def roughness_callback(self, msg):
        try:
            self.roughness_map = decode_float_image(msg)
        except Exception as exc:
            self.get_logger().warning(
                f"roughness decode failed: {exc}"
            )

    def confidence_callback(self, msg):
        self.geometry_confidence = float(
            msg.data
        )

    def valid_ratio_callback(self, msg):
        self.valid_ratio = float(
            msg.data
        )

    def _finite_mean(self, values):
        if values is None:
            return None

        finite = values[
            np.isfinite(values)
        ]

        if finite.size == 0:
            return None

        return float(
            np.mean(finite)
        )

    def process(self):
        self.frames += 1

        if (
            self.slope_map is None
            or self.roughness_map is None
            or self.geometry_confidence is None
            or self.valid_ratio is None
        ):
            return

        slope_mean = self._finite_mean(
            self.slope_map
        )

        roughness_mean = self._finite_mean(
            self.roughness_map
        )

        if (
            slope_mean is None
            or roughness_mean is None
        ):
            label = CLASS_UNKNOWN
            class_confidence = 0.0

        else:
            label, class_confidence = classify_terrain(
                slope_mean,
                roughness_mean,
                self.geometry_confidence,
                self.valid_ratio,
            )

        class_msg = String()
        class_msg.data = label

        confidence_msg = Float32()
        confidence_msg.data = float(
            class_confidence
        )

        self.class_pub.publish(
            class_msg
        )

        self.confidence_pub.publish(
            confidence_msg
        )

        if self.frames % 10 == 0:
            self.get_logger().info(
                f"frames={self.frames} "
                f"class={label} "
                f"class_confidence="
                f"{class_confidence:.3f} "
                f"slope_deg="
                f"{slope_mean if slope_mean is not None else float('nan'):.3f} "
                f"roughness_m="
                f"{roughness_mean if roughness_mean is not None else float('nan'):.5f} "
                f"geometry_confidence="
                f"{self.geometry_confidence:.3f} "
                f"valid_ratio="
                f"{self.valid_ratio:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = TerrainClassifierNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
