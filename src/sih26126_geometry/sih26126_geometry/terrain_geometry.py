#!/usr/bin/env python3

import math
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32
from sensor_msgs_py import point_cloud2


def pointcloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Decode organized FLOAT32 XYZ PointCloud2 into HxWx3."""
    if msg.height <= 0 or msg.width <= 0:
        raise ValueError("PointCloud2 has invalid dimensions")

    field_map = {f.name: f for f in msg.fields}

    for name in ("x", "y", "z"):
        if name not in field_map:
            raise ValueError(f"PointCloud2 missing {name} field")
        if field_map[name].datatype != PointField.FLOAT32:
            raise ValueError(f"PointCloud2 field {name} must be FLOAT32")

    dtype = np.uint8
    raw = np.frombuffer(msg.data, dtype=dtype)

    expected = msg.row_step * msg.height
    if raw.size < expected:
        raise ValueError("PointCloud2 data shorter than row_step*height")

    endian = ">" if msg.is_bigendian else "<"
    out = np.full(
        (msg.height, msg.width, 3),
        np.nan,
        dtype=np.float32,
    )

    for channel, name in enumerate(("x", "y", "z")):
        offset = field_map[name].offset
        typed = np.dtype(endian + "f4")

        out[:, :, channel] = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=typed,
            buffer=raw,
            offset=offset,
            strides=(msg.row_step, msg.point_step),
        )

    return out


def finite_mask(xyz: np.ndarray, min_range: float, max_range: float) -> np.ndarray:
    """Return finite XYZ points within Euclidean range bounds."""
    x = xyz[:, :, 0]
    y = xyz[:, :, 1]
    z = xyz[:, :, 2]

    rng = np.sqrt(x * x + y * y + z * z)

    return (
        np.isfinite(x)
        & np.isfinite(y)
        & np.isfinite(z)
        & np.isfinite(rng)
        & (rng >= min_range)
        & (rng <= max_range)
    )


def compute_surface_normals(
    xyz: np.ndarray,
    valid: np.ndarray,
) -> np.ndarray:
    """Estimate organized-cloud normals from central 3-D differences."""
    h, w, _ = xyz.shape
    normals = np.full((h, w, 3), np.nan, dtype=np.float32)

    if h < 3 or w < 3:
        return normals

    left = xyz[1:-1, :-2]
    right = xyz[1:-1, 2:]
    up = xyz[:-2, 1:-1]
    down = xyz[2:, 1:-1]

    valid_center = (
        valid[1:-1, 1:-1]
        & valid[1:-1, :-2]
        & valid[1:-1, 2:]
        & valid[:-2, 1:-1]
        & valid[2:, 1:-1]
    )

    valid_x = valid[1:-1, :-2] & valid[1:-1, 2:]
    valid_y = valid[:-2, 1:-1] & valid[2:, 1:-1]

    tangent_x = np.zeros_like(right, dtype=np.float32)
    tangent_y = np.zeros_like(down, dtype=np.float32)

    tangent_x[valid_x] = right[valid_x] - left[valid_x]
    tangent_y[valid_y] = down[valid_y] - up[valid_y]

    normal = np.cross(tangent_x, tangent_y)
    magnitude = np.linalg.norm(normal, axis=2)

    good = valid_center & np.isfinite(magnitude) & (magnitude > 1e-6)

    normal_good = normal[good] / magnitude[good, None]

    # The Gazebo depth cloud uses z as the approximately vertical axis
    # for the current UGV camera configuration. Orient normals upward.
    flip = normal_good[:, 2] < 0.0
    normal_good[flip] *= -1.0

    target = normals[1:-1, 1:-1]
    target[good] = normal_good.astype(np.float32)

    return normals


def compute_slope_degrees(
    normals: np.ndarray,
    valid: np.ndarray,
) -> np.ndarray:
    """Compute slope angle relative to the +Z vertical axis."""
    nz = np.abs(normals[:, :, 2])
    nz = np.clip(nz, 0.0, 1.0)

    slope = np.full(normals.shape[:2], np.nan, dtype=np.float32)
    good = valid & np.isfinite(nz)

    slope[good] = np.degrees(np.arccos(nz[good])).astype(np.float32)
    return slope


def box_mean(values: np.ndarray, valid: np.ndarray, radius: int) -> np.ndarray:
    """NaN-aware box mean using integral images."""
    h, w = values.shape
    mask = valid & np.isfinite(values)

    v = np.where(mask, values, 0.0).astype(np.float64)
    m = mask.astype(np.float64)

    integral_v = np.pad(v, ((1, 1), (1, 1)), mode="constant")
    integral_m = np.pad(m, ((1, 1), (1, 1)), mode="constant")

    integral_v = integral_v.cumsum(axis=0).cumsum(axis=1)
    integral_m = integral_m.cumsum(axis=0).cumsum(axis=1)

    y0 = np.arange(h) - radius
    y1 = np.arange(h) + radius + 1
    x0 = np.arange(w) - radius
    x1 = np.arange(w) + radius + 1

    y0 = np.clip(y0, 0, h)
    y1 = np.clip(y1, 0, h)
    x0 = np.clip(x0, 0, w)
    x1 = np.clip(x1, 0, w)

    sum_v = (
        integral_v[y1[:, None], x1[None, :]]
        - integral_v[y0[:, None], x1[None, :]]
        - integral_v[y1[:, None], x0[None, :]]
        + integral_v[y0[:, None], x0[None, :]]
    )

    count = (
        integral_m[y1[:, None], x1[None, :]]
        - integral_m[y0[:, None], x1[None, :]]
        - integral_m[y1[:, None], x0[None, :]]
        + integral_m[y0[:, None], x0[None, :]]
    )

    out = np.full((h, w), np.nan, dtype=np.float32)
    good = count >= 3
    out[good] = (sum_v[good] / count[good]).astype(np.float32)
    return out


def compute_roughness_m(
    xyz: np.ndarray,
    valid: np.ndarray,
    radius: int = 3,
) -> np.ndarray:
    """
    Estimate local surface roughness as RMS high-frequency height residual.

    A local mean removes most of the first-order slope component, while
    preserving small-scale terrain variation.
    """
    z = xyz[:, :, 2]

    mean = box_mean(z, valid, radius)
    residual = z - mean

    residual_sq = residual * residual
    mean_sq = box_mean(residual_sq, valid & np.isfinite(residual), radius)

    roughness = np.full(z.shape, np.nan, dtype=np.float32)
    good = np.isfinite(mean_sq) & (mean_sq >= 0.0)
    roughness[good] = np.sqrt(mean_sq[good]).astype(np.float32)

    return roughness


def confidence_from_geometry(
    valid: np.ndarray,
    normals: np.ndarray,
    roughness: np.ndarray,
) -> float:
    """Summarize fraction of valid navigation pixels with usable geometry."""
    usable = (
        valid
        & np.isfinite(normals).all(axis=2)
        & np.isfinite(roughness)
    )
    total = int(np.count_nonzero(valid))
    if total == 0:
        return 0.0
    return float(np.count_nonzero(usable) / total)


def image_from_float(
    array: np.ndarray,
    header,
    frame_id: str,
) -> Image:
    """Create a ROS 32FC1 image from a float32 array."""
    msg = Image()
    msg.header = header
    msg.header.frame_id = frame_id
    msg.height = array.shape[0]
    msg.width = array.shape[1]
    msg.encoding = "32FC1"
    msg.is_bigendian = 0
    msg.step = array.shape[1] * 4
    msg.data = np.asarray(array, dtype=np.float32).tobytes()
    return msg


class TerrainGeometryNode(Node):
    """ROS node producing slope, normals, and roughness from depth geometry."""

    def __init__(self):
        super().__init__("terrain_geometry")

        self.declare_parameter("input_topic", "/depth_camera/points")
        self.declare_parameter("min_range_m", 0.1)
        self.declare_parameter("max_range_m", 15.0)
        self.declare_parameter("roi_y_start", 0.50)
        self.declare_parameter("roughness_radius", 3)
        self.declare_parameter("publish_hz", 5.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.min_range = float(self.get_parameter("min_range_m").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.roi_y_start = float(self.get_parameter("roi_y_start").value)
        self.roughness_radius = int(
            self.get_parameter("roughness_radius").value
        )

        self.latest_cloud = None
        self.latest_cloud_stamp = None

        from rclpy.qos import qos_profile_sensor_data

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.normals_pub = self.create_publisher(
            PointCloud2,
            "/geometry/surface_normals",
            10,
        )
        self.slope_pub = self.create_publisher(
            Image,
            "/geometry/slope_map",
            10,
        )
        self.roughness_pub = self.create_publisher(
            Image,
            "/geometry/roughness_map",
            10,
        )
        self.confidence_pub = self.create_publisher(
            Float32,
            "/geometry/geometry_confidence",
            10,
        )

        period = max(0.05, 1.0 / float(
            self.get_parameter("publish_hz").value
        ))
        self.timer = self.create_timer(period, self.process)

        self.frames = 0
        self.get_logger().info(
            "Terrain geometry started. Waiting for depth cloud..."
        )

    def cloud_callback(self, msg: PointCloud2):
        self.latest_cloud = msg
        self.latest_cloud_stamp = msg.header.stamp

    def process(self):
        msg = self.latest_cloud
        if msg is None:
            return

        self.latest_cloud = None

        try:
            xyz = pointcloud_to_xyz(msg)
            valid = finite_mask(
                xyz,
                self.min_range,
                self.max_range,
            )

            h = xyz.shape[0]
            y0 = int(round(h * self.roi_y_start))
            roi = np.zeros_like(valid, dtype=bool)
            roi[y0:, :] = True

            valid &= roi

            normals = compute_surface_normals(xyz, valid)
            slope = compute_slope_degrees(normals, valid)
            roughness = compute_roughness_m(
                xyz,
                valid,
                radius=self.roughness_radius,
            )

            confidence = confidence_from_geometry(
                valid,
                normals,
                roughness,
            )

            slope_msg = image_from_float(
                slope,
                msg.header,
                msg.header.frame_id,
            )
            roughness_msg = image_from_float(
                roughness,
                msg.header,
                msg.header.frame_id,
            )

            self.slope_pub.publish(slope_msg)
            self.roughness_pub.publish(roughness_msg)

            confidence_msg = Float32()
            confidence_msg.data = confidence
            self.confidence_pub.publish(confidence_msg)

            finite_normals = normals[
                np.isfinite(normals).all(axis=2)
            ]

            if finite_normals.size:
                fields = [
                    PointField(
                        name="nx",
                        offset=0,
                        datatype=PointField.FLOAT32,
                        count=1,
                    ),
                    PointField(
                        name="ny",
                        offset=4,
                        datatype=PointField.FLOAT32,
                        count=1,
                    ),
                    PointField(
                        name="nz",
                        offset=8,
                        datatype=PointField.FLOAT32,
                        count=1,
                    ),
                ]

                normal_points = finite_normals.astype(
                    np.float32
                ).tolist()

                normals_msg = point_cloud2.create_cloud(
                    msg.header,
                    fields,
                    normal_points,
                )
                self.normals_pub.publish(normals_msg)

            self.frames += 1

            if self.frames % 10 == 0:
                finite_count = int(np.count_nonzero(valid))
                slope_values = slope[np.isfinite(slope)]
                rough_values = roughness[np.isfinite(roughness)]

                slope_mean = (
                    float(np.mean(slope_values))
                    if slope_values.size
                    else float("nan")
                )
                rough_mean = (
                    float(np.mean(rough_values))
                    if rough_values.size
                    else float("nan")
                )

                self.get_logger().info(
                    f"frames={self.frames} "
                    f"valid={finite_count} "
                    f"confidence={confidence:.3f} "
                    f"mean_slope_deg={slope_mean:.3f} "
                    f"mean_roughness_m={rough_mean:.5f}"
                )

        except Exception as exc:
            self.get_logger().error(
                f"Terrain geometry processing failed: {exc}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TerrainGeometryNode()

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
