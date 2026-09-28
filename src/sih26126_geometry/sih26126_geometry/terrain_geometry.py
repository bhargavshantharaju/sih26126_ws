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


def fit_ground_plane(
    xyz: np.ndarray,
    valid: np.ndarray,
    fit_region: np.ndarray,
    iterations: int = 100,
    distance_threshold: float = 0.05,
    max_slope_degrees: float = 50.0,
    seed: int = 26126,
):
    """Fit the dominant near-horizontal plane and return its inlier mask.

    RANSAC is restricted to a central, lower-image corridor by the caller.
    Near-vertical planes are rejected so tree trunks and obstacle faces cannot
    be interpreted as ground. The winning plane is refined with SVD over its
    inliers to reduce sampling noise.
    """
    if xyz.ndim != 3 or xyz.shape[2] != 3:
        raise ValueError("xyz must have shape HxWx3")
    if valid.shape != xyz.shape[:2] or fit_region.shape != valid.shape:
        raise ValueError("valid and fit_region must match xyz image dimensions")

    candidate_mask = valid & fit_region
    candidate_pixels = np.flatnonzero(candidate_mask)
    if candidate_pixels.size < 100:
        return None

    points_all = xyz.reshape(-1, 3)[candidate_pixels].astype(np.float64)
    finite = np.isfinite(points_all).all(axis=1)
    candidate_pixels = candidate_pixels[finite]
    points_all = points_all[finite]
    if points_all.shape[0] < 100:
        return None

    # Bound per-frame work while sampling uniformly across the image region.
    rng = np.random.default_rng(seed)
    if points_all.shape[0] > 6000:
        sample_indices = np.sort(rng.choice(points_all.shape[0], 6000, replace=False))
        points = points_all[sample_indices]
    else:
        points = points_all

    min_vertical_normal = math.cos(math.radians(max_slope_degrees))
    best_inliers = None
    best_count = 0
    best_rmse = float("inf")

    for _ in range(max(1, int(iterations))):
        sample = points[rng.choice(points.shape[0], 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-8:
            continue
        normal /= norm
        if abs(float(normal[2])) < min_vertical_normal:
            continue
        if normal[2] < 0.0:
            normal *= -1.0

        offset = -float(np.dot(normal, sample[0]))
        residual = np.abs(points @ normal + offset)
        inliers = residual <= distance_threshold
        count = int(np.count_nonzero(inliers))
        if count < best_count or count < 3:
            continue
        rmse = float(np.sqrt(np.mean(residual[inliers] ** 2)))
        if count > best_count or rmse < best_rmse:
            best_inliers = inliers
            best_count = count
            best_rmse = rmse

    if best_inliers is None or best_count < 100:
        return None

    inlier_points = points[best_inliers]
    center = np.mean(inlier_points, axis=0)
    _, _, vh = np.linalg.svd(inlier_points - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal *= -1.0
    if abs(float(normal[2])) < min_vertical_normal:
        return None
    offset = -float(np.dot(normal, center))

    # Re-evaluate the refined model on all candidate pixels.
    residual_all = np.abs(points_all @ normal + offset)
    inlier_all = residual_all <= distance_threshold
    if int(np.count_nonzero(inlier_all)) < 100:
        return None
    rmse = float(np.sqrt(np.mean(residual_all[inlier_all] ** 2)))

    inlier_mask = np.zeros(valid.size, dtype=bool)
    inlier_mask[candidate_pixels[inlier_all]] = True
    return (
        np.asarray((*normal, offset), dtype=np.float32),
        inlier_mask.reshape(valid.shape),
        rmse,
        int(points_all.shape[0]),
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
        self.declare_parameter("roi_x_margin", 0.15)
        self.declare_parameter("ground_plane_threshold_m", 0.05)
        self.declare_parameter("ground_plane_max_slope_deg", 50.0)
        self.declare_parameter("roughness_radius", 3)
        self.declare_parameter("publish_hz", 5.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.min_range = float(self.get_parameter("min_range_m").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.roi_y_start = float(self.get_parameter("roi_y_start").value)
        self.roi_x_margin = float(self.get_parameter("roi_x_margin").value)
        self.ground_plane_threshold = float(
            self.get_parameter("ground_plane_threshold_m").value
        )
        self.ground_plane_max_slope = float(
            self.get_parameter("ground_plane_max_slope_deg").value
        )
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
            fit_region = np.zeros_like(valid, dtype=bool)
            x_margin = int(round(xyz.shape[1] * self.roi_x_margin))
            fit_region[y0:, x_margin:xyz.shape[1] - x_margin] = True
            plane_result = fit_ground_plane(
                xyz,
                valid,
                fit_region,
                distance_threshold=self.ground_plane_threshold,
                max_slope_degrees=self.ground_plane_max_slope,
            )
            if plane_result is None:
                self.get_logger().warning(
                    "Ground-plane estimation failed; terrain geometry is unavailable."
                )
                return

            plane, ground_inliers, plane_rmse, candidate_count = plane_result
            slope_degrees = math.degrees(
                math.acos(float(np.clip(plane[2], -1.0, 1.0)))
            )
            residual = np.abs(
                np.einsum("ijk,k->ij", xyz, plane[:3]) + float(plane[3])
            )
            slope = np.full(valid.shape, np.nan, dtype=np.float32)
            roughness = np.full(valid.shape, np.nan, dtype=np.float32)
            slope[ground_inliers] = slope_degrees
            roughness[ground_inliers] = residual[ground_inliers]

            inlier_count = int(np.count_nonzero(ground_inliers))
            inlier_ratio = inlier_count / float(max(1, candidate_count))
            support = min(1.0, inlier_count / 1500.0)
            consistency = max(
                0.0,
                1.0 - plane_rmse / max(self.ground_plane_threshold, 1e-6),
            )
            confidence = float(
                np.clip(
                    0.50 * support
                    + 0.30 * min(1.0, inlier_ratio / 0.40)
                    + 0.20 * consistency,
                    0.0,
                    1.0,
                )
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

            if inlier_count:
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

                normal_points = np.broadcast_to(
                    plane[:3],
                    (inlier_count, 3),
                ).astype(np.float32).tolist()

                normals_msg = point_cloud2.create_cloud(
                    msg.header,
                    fields,
                    normal_points,
                )
                self.normals_pub.publish(normals_msg)

            self.frames += 1

            if self.frames % 10 == 0:
                finite_count = int(np.count_nonzero(ground_inliers))
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
                    f"ground_inliers={finite_count} "
                    f"confidence={confidence:.3f} "
                    f"ground_slope_deg={slope_degrees:.3f} "
                    f"ground_ratio={inlier_ratio:.3f} "
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
