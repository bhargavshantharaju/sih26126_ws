#!/usr/bin/env python3

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32


def pointcloud_to_xyz(msg):
    """
    Convert an organized PointCloud2 message into an HxWx3 float32 array.

    Supports standard FLOAT32 x/y/z fields and both endian modes.
    Invalid points remain NaN.
    """

    field_map = {
        field.name: field
        for field in msg.fields
    }

    for name in ("x", "y", "z"):
        if name not in field_map:
            raise ValueError(
                f"PointCloud2 missing {name} field"
            )

        if field_map[name].datatype != PointField.FLOAT32:
            raise ValueError(
                f"PointCloud2 field {name} is not FLOAT32"
            )

    width = msg.width
    height = msg.height

    if width <= 0 or height <= 0:
        raise ValueError("Invalid PointCloud2 dimensions")

    byte_order = ">" if msg.is_bigendian else "<"

    dtype = np.dtype(
        [
            (
                "x",
                f"{byte_order}f4",
            ),
            (
                "y",
                f"{byte_order}f4",
            ),
            (
                "z",
                f"{byte_order}f4",
            ),
        ]
    )

    offsets = [
        field_map["x"].offset,
        field_map["y"].offset,
        field_map["z"].offset,
    ]

    min_needed = max(offsets) + 4

    if msg.point_step < min_needed:
        raise ValueError(
            "Point step is too small for x/y/z fields"
        )

    # Build the exact structured dtype using the real offsets.
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [
                f"{byte_order}f4",
                f"{byte_order}f4",
                f"{byte_order}f4",
            ],
            "offsets": offsets,
            "itemsize": msg.point_step,
        }
    )

    required_bytes = msg.row_step * msg.height

    if len(msg.data) < required_bytes:
        raise ValueError(
            "PointCloud2 data shorter than row_step*height"
        )

    raw = np.frombuffer(
        msg.data,
        dtype=dtype,
        count=(
            msg.row_step // msg.point_step
        ) * msg.height,
    )

    points = np.full(
        (height, width, 3),
        np.nan,
        dtype=np.float32,
    )

    rows = raw.reshape(
        height,
        msg.row_step // msg.point_step,
    )

    usable = rows[:, :width]

    points[:, :, 0] = usable["x"]
    points[:, :, 1] = usable["y"]
    points[:, :, 2] = usable["z"]

    return points


def fit_plane_ransac(
    points,
    iterations=120,
    distance_threshold=0.08,
    seed=26126,
):
    """
    Fit plane ax+by+cz+d=0 with deterministic RANSAC.
    """

    if points.shape[0] < 30:
        return None

    rng = np.random.default_rng(seed)

    best_plane = None
    best_count = 0
    best_error = float("inf")

    n_points = points.shape[0]

    for _ in range(iterations):

        indices = rng.choice(
            n_points,
            size=3,
            replace=False,
        )

        p1, p2, p3 = points[indices]

        v1 = p2 - p1
        v2 = p3 - p1

        normal = np.cross(
            v1,
            v2,
        )

        norm = np.linalg.norm(normal)

        if norm < 1e-6:
            continue

        normal = normal / norm

        d = -float(
            np.dot(normal, p1)
        )

        distances = np.abs(
            points @ normal + d
        )

        inliers = distances <= distance_threshold

        count = int(inliers.sum())

        if count < best_count:
            continue

        if count == 0:
            continue

        error = float(
            np.sqrt(
                np.mean(
                    distances[inliers] ** 2
                )
            )
        )

        if (
            count > best_count
            or error < best_error
        ):
            best_plane = (
                normal[0],
                normal[1],
                normal[2],
                d,
            )

            best_count = count
            best_error = error

    if best_plane is None:
        return None

    return (
        np.asarray(best_plane, dtype=np.float32),
        best_count,
        best_error,
    )


class GeometryBaseline(Node):

    def __init__(self):

        super().__init__(
            "sih26126_geometry_baseline"
        )

        self.declare_parameter(
            "min_range",
            0.15,
        )

        self.declare_parameter(
            "max_range",
            20.0,
        )

        self.declare_parameter(
            "positive_obstacle_margin",
            0.15,
        )

        self.declare_parameter(
            "negative_obstacle_margin",
            0.30,
        )

        self.declare_parameter(
            "plane_threshold",
            0.08,
        )

        self.obstacle_pub = self.create_publisher(
            Image,
            "/geometry/obstacle_mask",
            10,
        )

        self.negative_pub = self.create_publisher(
            Image,
            "/geometry/negative_obstacle_mask",
            10,
        )

        self.ground_fraction_pub = self.create_publisher(
            Float32,
            "/geometry/ground_fraction",
            10,
        )

        self.obstacle_fraction_pub = self.create_publisher(
            Float32,
            "/geometry/obstacle_fraction",
            10,
        )

        self.negative_fraction_pub = self.create_publisher(
            Float32,
            "/geometry/negative_obstacle_fraction",
            10,
        )

        self.rmse_pub = self.create_publisher(
            Float32,
            "/geometry/plane_rmse",
            10,
        )

        self.confidence_pub = self.create_publisher(
            Float32,
            "/geometry/confidence",
            10,
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            "/depth_camera/points",
            self.cloud_callback,
            10,
        )

        self.frames = 0

        self.get_logger().info(
            "3D geometry baseline started."
        )

    def publish_image(
        self,
        image,
        publisher,
        frame_id,
        stamp,
    ):

        msg = Image()

        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = image.shape[1]
        msg.data = image.tobytes()

        publisher.publish(msg)

    def cloud_callback(self, msg):

        try:
            points = pointcloud_to_xyz(msg)
        except Exception as exc:
            self.get_logger().warning(
                f"PointCloud conversion failed: {exc}"
            )
            return

        h, w, _ = points.shape

        x = points[:, :, 0]
        y = points[:, :, 1]
        z = points[:, :, 2]

        range_m = np.sqrt(
            x * x +
            y * y +
            z * z
        )

        valid = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
            & np.isfinite(range_m)
            & (range_m >=
               self.get_parameter(
                   "min_range"
               ).value)
            & (range_m <=
               self.get_parameter(
                   "max_range"
               ).value)
        )

        # Use the lower 65% of the image to estimate terrain.
        fit_region = np.zeros(
            (h, w),
            dtype=bool,
        )

        fit_region[
            int(h * 0.35):
            int(h * 0.92),
            int(w * 0.05):
            int(w * 0.95),
        ] = True

        fit_mask = valid & fit_region

        fit_points = points[fit_mask]

        if fit_points.shape[0] < 100:

            self.get_logger().warning(
                "Not enough valid points for ground-plane fitting."
            )

            return

        result = fit_plane_ransac(
            fit_points,
            iterations=120,
            distance_threshold=float(
                self.get_parameter(
                    "plane_threshold"
                ).value
            ),
        )

        if result is None:

            self.get_logger().warning(
                "Ground-plane estimation failed."
            )

            return

        plane, inlier_count, plane_rmse = result

        nx, ny, nz, d = plane

        # Ray from camera origin through each valid point.
        #
        # The organized point cloud legitimately contains NaN
        # values outside the visible/valid region. Do not perform
        # arithmetic or division on those pixels.

        ray_norm = np.sqrt(
            x * x +
            y * y +
            z * z
        )

        ray_valid = (
            valid
            & np.isfinite(ray_norm)
            & (ray_norm > 1e-6)
        )

        rx = np.zeros_like(x)
        ry = np.zeros_like(y)
        rz = np.zeros_like(z)

        np.divide(
            x,
            ray_norm,
            out=rx,
            where=ray_valid,
        )

        np.divide(
            y,
            ray_norm,
            out=ry,
            where=ray_valid,
        )

        np.divide(
            z,
            ray_norm,
            out=rz,
            where=ray_valid,
        )

        ray_plane_denominator = (
            nx * rx +
            ny * ry +
            nz * rz
        )

        ground_depth = np.full(
            (h, w),
            np.nan,
            dtype=np.float32,
        )

        plane_valid = (
            ray_valid
            & np.isfinite(
                ray_plane_denominator
            )
            & (
                np.abs(
                    ray_plane_denominator
                ) > 1e-4
            )
        )

        ground_depth[plane_valid] = (
            -d /
            ray_plane_denominator[
                plane_valid
            ]
        )

        positive_margin = float(
            self.get_parameter(
                "positive_obstacle_margin"
            ).value
        )

        negative_margin = float(
            self.get_parameter(
                "negative_obstacle_margin"
            ).value
        )

        # Only evaluate the useful lower image region.
        evaluation_region = np.zeros(
            (h, w),
            dtype=bool,
        )

        evaluation_region[
            int(h * 0.25):
            int(h * 0.92),
            int(w * 0.05):
            int(w * 0.95),
        ] = True

        eval_valid = (
            valid
            & plane_valid
            & evaluation_region
            & np.isfinite(ground_depth)
            & (ground_depth > 0.10)
        )

        positive = (
            eval_valid
            & (
                range_m
                <
                ground_depth
                - positive_margin
            )
        )

        negative = (
            eval_valid
            & (
                range_m
                >
                ground_depth
                + negative_margin
            )
        )

        # Remove isolated noise using a simple neighborhood rule.
        # This keeps the baseline dependency-free.
        def neighborhood_count(mask):
            padded = np.pad(
                mask.astype(np.uint8),
                1,
                mode="constant",
            )

            total = (
                padded[:-2, :-2]
                + padded[:-2, 1:-1]
                + padded[:-2, 2:]
                + padded[1:-1, :-2]
                + padded[1:-1, 1:-1]
                + padded[1:-1, 2:]
                + padded[2:, :-2]
                + padded[2:, 1:-1]
                + padded[2:, 2:]
            )

            return total

        positive = (
            positive
            & (
                neighborhood_count(positive)
                >= 3
            )
        )

        negative = (
            negative
            & (
                neighborhood_count(negative)
                >= 3
            )
        )

        ground = (
            eval_valid
            & ~positive
            & ~negative
        )

        obstacle_mask = np.zeros(
            (h, w),
            dtype=np.uint8,
        )

        negative_mask = np.zeros(
            (h, w),
            dtype=np.uint8,
        )

        obstacle_mask[positive] = 255
        negative_mask[negative] = 255

        ground_fraction = (
            float(ground.sum())
            /
            float(eval_valid.sum())
            if eval_valid.any()
            else 0.0
        )

        obstacle_fraction = (
            float(positive.sum())
            /
            float(eval_valid.sum())
            if eval_valid.any()
            else 0.0
        )

        negative_fraction = (
            float(negative.sum())
            /
            float(eval_valid.sum())
            if eval_valid.any()
            else 0.0
        )

        valid_fraction = float(
            eval_valid.mean()
        )

        plane_fraction = (
            float(inlier_count)
            /
            float(fit_points.shape[0])
        )

        confidence = float(
            np.clip(
                0.50 * min(
                    plane_fraction / 0.70,
                    1.0,
                )
                + 0.30 * min(
                    valid_fraction / 0.50,
                    1.0,
                )
                + 0.20 * max(
                    0.0,
                    1.0 -
                    min(
                        plane_rmse / 0.15,
                        1.0,
                    ),
                ),
                0.0,
                1.0,
            )
        )

        self.publish_image(
            obstacle_mask,
            self.obstacle_pub,
            msg.header.frame_id,
            msg.header.stamp,
        )

        self.publish_image(
            negative_mask,
            self.negative_pub,
            msg.header.frame_id,
            msg.header.stamp,
        )

        for value, publisher in (
            (
                ground_fraction,
                self.ground_fraction_pub,
            ),
            (
                obstacle_fraction,
                self.obstacle_fraction_pub,
            ),
            (
                negative_fraction,
                self.negative_fraction_pub,
            ),
            (
                plane_rmse,
                self.rmse_pub,
            ),
            (
                confidence,
                self.confidence_pub,
            ),
        ):

            out = Float32()
            out.data = float(value)
            publisher.publish(out)

        self.frames += 1

        if self.frames % 30 == 0:

            self.get_logger().info(
                "geometry frame=%d "
                "valid=%.3f "
                "ground=%.3f "
                "obstacle=%.3f "
                "negative=%.3f "
                "rmse=%.4f "
                "confidence=%.3f"
                % (
                    self.frames,
                    valid_fraction,
                    ground_fraction,
                    obstacle_fraction,
                    negative_fraction,
                    plane_rmse,
                    confidence,
                )
            )


def main():

    rclpy.init()

    node = GeometryBaseline()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
