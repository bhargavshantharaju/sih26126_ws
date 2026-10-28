#!/usr/bin/env python3

"""Timestamp-synchronized and motion-gated RGB-D visual odometry."""

from __future__ import annotations

from collections import deque
import math
from typing import Optional

import cv2

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def stamp_to_seconds(stamp) -> float:
    """Convert a ROS time message into seconds."""
    return (
        float(stamp.sec)
        + float(stamp.nanosec) * 1e-9
    )


def extract_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract organized XYZ point cloud as HxWx3 float32 array."""
    field_map = {
        field.name: field
        for field in msg.fields
    }

    for name in ('x', 'y', 'z'):
        if name not in field_map:
            raise ValueError(
                f'PointCloud2 is missing field {name!r}'
            )

    height = int(msg.height)
    width = int(msg.width)

    if height <= 0 or width <= 0:
        raise ValueError(
            'PointCloud2 has invalid dimensions'
        )

    endian = '>' if msg.is_bigendian else '<'

    xyz = np.full(
        (height, width, 3),
        np.nan,
        dtype=np.float32,
    )

    for channel, name in enumerate(('x', 'y', 'z')):
        field = field_map[name]

        if field.datatype != 7:
            raise ValueError(
                f'PointCloud2 field {name!r} is not FLOAT32'
            )

        values = np.ndarray(
            shape=(height, width),
            dtype=np.dtype(endian + 'f4'),
            buffer=msg.data,
            offset=int(field.offset),
            strides=(
                int(msg.row_step),
                int(msg.point_step),
            ),
        )

        xyz[:, :, channel] = values.astype(
            np.float32,
            copy=False,
        )

    return xyz


def decode_rgb(msg: Image) -> np.ndarray:
    """Decode ROS image into grayscale uint8 image."""
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)

    raw = np.frombuffer(
        msg.data,
        dtype=np.uint8,
    )

    encoding = msg.encoding.lower()

    if encoding in ('rgb8', 'bgr8'):
        image = raw.reshape(
            height,
            step,
        )[:, : width * 3]

        image = image.reshape(
            height,
            width,
            3,
        )

        if encoding == 'rgb8':
            return cv2.cvtColor(
                image,
                cv2.COLOR_RGB2GRAY,
            )

        return cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY,
        )

    if encoding in ('mono8', '8uc1'):
        return raw.reshape(
            height,
            step,
        )[:, :width].copy()

    raise ValueError(
        f'Unsupported image encoding: {msg.encoding}'
    )


def rigid_transform(
    source: np.ndarray,
    target: np.ndarray,
) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """Compute target ~= R @ source + t using SVD."""
    if source.shape != target.shape:
        raise ValueError(
            'Source and target shapes differ'
        )

    if source.shape[0] < 3:
        return None

    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)

    source_centered = source - source_centroid
    target_centered = target - target_centroid

    covariance = (
        source_centered.T @ target_centered
    )

    try:
        u, _, vt = np.linalg.svd(covariance)
    except np.linalg.LinAlgError:
        return None

    rotation = vt.T @ u.T

    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    translation = (
        target_centroid
        - rotation @ source_centroid
    )

    return (
        rotation.astype(np.float32),
        translation.astype(np.float32),
    )


def ransac_rigid_transform(
    source: np.ndarray,
    target: np.ndarray,
    iterations: int = 180,
    threshold_m: float = 0.04,
) -> tuple[
    Optional[np.ndarray],
    Optional[np.ndarray],
    np.ndarray,
    np.ndarray,
]:
    """Robustly estimate a 3D rigid transform."""
    count = source.shape[0]

    if count < 6:
        return (
            None,
            None,
            np.zeros(count, dtype=bool),
            np.full(
                count,
                np.inf,
                dtype=np.float32,
            ),
        )

    rng = np.random.default_rng(26126)

    best_inliers = np.zeros(
        count,
        dtype=bool,
    )

    best_r = None
    best_t = None

    for _ in range(iterations):
        sample_indices = rng.choice(
            count,
            size=3,
            replace=False,
        )

        result = rigid_transform(
            source[sample_indices],
            target[sample_indices],
        )

        if result is None:
            continue

        rotation, translation = result

        predicted = (
            source @ rotation.T
            + translation
        )

        residuals = np.linalg.norm(
            predicted - target,
            axis=1,
        )

        inliers = residuals < threshold_m
        inlier_count = int(inliers.sum())

        if inlier_count > int(best_inliers.sum()):
            best_inliers = inliers
            best_r = rotation
            best_t = translation

    if (
        best_r is not None
        and int(best_inliers.sum()) >= 6
    ):
        refined = rigid_transform(
            source[best_inliers],
            target[best_inliers],
        )

        if refined is not None:
            best_r, best_t = refined

    if best_r is None or best_t is None:
        return (
            None,
            None,
            best_inliers,
            np.full(
                count,
                np.inf,
                dtype=np.float32,
            ),
        )

    predicted = (
        source @ best_r.T
        + best_t
    )

    residuals = np.linalg.norm(
        predicted - target,
        axis=1,
    )

    final_inliers = residuals < threshold_m

    return (
        best_r,
        best_t,
        final_inliers,
        residuals.astype(np.float32),
    )


def rotation_angle(rotation: np.ndarray) -> float:
    """Return the angle represented by a rotation matrix."""
    cosine = (
        float(np.trace(rotation)) - 1.0
    ) / 2.0

    return math.acos(
        max(-1.0, min(1.0, cosine))
    )


def rotation_to_yaw(rotation: np.ndarray) -> float:
    """Return planar yaw for X-forward, Y-left, Z-up frame."""
    return math.atan2(
        float(rotation[1, 0]),
        float(rotation[0, 0]),
    )


def transform_is_credible(
    rotation: np.ndarray,
    translation: np.ndarray,
    inliers: np.ndarray,
    residuals: np.ndarray,
    dt: float,
) -> tuple[bool, str]:
    """Reject geometrically or physically implausible transforms."""
    inlier_count = int(inliers.sum())
    total = int(inliers.size)

    if inlier_count < 8:
        return False, 'too_few_inliers'

    if total <= 0:
        return False, 'no_correspondences'

    inlier_ratio = (
        float(inlier_count)
        / float(total)
    )

    if inlier_ratio < 0.65:
        return False, 'low_inlier_ratio'

    inlier_residuals = residuals[inliers]

    if inlier_residuals.size == 0:
        return False, 'no_inlier_residuals'

    median_residual = float(
        np.median(inlier_residuals)
    )

    if not np.isfinite(median_residual):
        return False, 'invalid_residual'

    if median_residual > 0.05:
        return False, 'high_residual'

    translation_m = float(
        np.linalg.norm(translation)
    )

    if not np.isfinite(translation_m):
        return False, 'invalid_translation'

    allowed_translation = (
        1.8 * max(dt, 0.0)
        + 0.05
    )

    if translation_m > allowed_translation:
        return False, 'motion_too_large'

    angle = rotation_angle(rotation)

    if not np.isfinite(angle):
        return False, 'invalid_rotation'

    allowed_rotation = min(
        0.40,
        2.0 * max(dt, 0.0) + 0.12,
    )

    if angle > allowed_rotation:
        return False, 'rotation_too_large'

    return True, 'accepted'


def nearest_buffer_index(
    buffer: deque,
    stamp: float,
) -> Optional[int]:
    """Return nearest buffered image index for a timestamp."""
    if not buffer:
        return None

    return min(
        range(len(buffer)),
        key=lambda index: abs(
            buffer[index][0] - stamp
        ),
    )


def sample_xyz(cloud: np.ndarray, u: float, v: float) -> Optional[np.ndarray]:
    """Return a valid metric XYZ sample near a pixel."""
    height, width = cloud.shape[:2]
    x = int(round(float(u)))
    y = int(round(float(v)))

    if x < 0 or x >= width or y < 0 or y >= height:
        return None

    x0, x1 = max(0, x - 1), min(width, x + 2)
    y0, y1 = max(0, y - 1), min(height, y + 2)

    points = cloud[y0:y1, x0:x1].reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]

    if points.size == 0:
        return None

    ranges = np.linalg.norm(points, axis=1)
    keep = (
        np.isfinite(ranges)
        & (ranges >= 0.2)
        & (ranges <= 20.0)
    )

    points = points[keep]
    ranges = ranges[keep]

    if points.size == 0:
        return None

    center = cloud[y, x]
    if np.isfinite(center).all():
        center_range = float(np.linalg.norm(center))
        if 0.2 <= center_range <= 20.0:
            return center.astype(np.float32, copy=True)

    index = int(
        np.argmin(
            np.abs(
                ranges - np.median(ranges)
            )
        )
    )

    return points[index].astype(
        np.float32,
        copy=True,
    )


GAZEBO_TO_OPTICAL = np.array(
    [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)


def gazebo_xyz_to_optical(
    points: np.ndarray,
) -> np.ndarray:
    """Convert Gazebo XYZ to OpenCV optical coordinates."""
    return points @ GAZEBO_TO_OPTICAL.T


def estimate_pnp(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
) -> Optional[
    tuple[np.ndarray, np.ndarray, np.ndarray, float]
]:
    """Estimate p_curr = R p_prev + t from 3D-to-2D matches."""
    if (
        len(object_points) < 8
        or len(object_points) != len(image_points)
    ):
        return None

    flag = (
        cv2.SOLVEPNP_SQPNP
        if hasattr(cv2, 'SOLVEPNP_SQPNP')
        else cv2.SOLVEPNP_EPNP
    )

    try:
        ok, rvec, tvec, indices = cv2.solvePnPRansac(
            object_points.astype(np.float64),
            image_points.astype(np.float64),
            camera_matrix.astype(np.float64),
            None,
            iterationsCount=250,
            reprojectionError=2.5,
            confidence=0.995,
            flags=flag,
        )
    except cv2.error:
        return None

    if (
        not ok
        or indices is None
        or len(indices) < 8
    ):
        return None

    indices = indices.reshape(-1)

    try:
        if hasattr(cv2, 'solvePnPRefineLM'):
            rvec, tvec = cv2.solvePnPRefineLM(
                object_points[indices].astype(
                    np.float64
                ),
                image_points[indices].astype(
                    np.float64
                ),
                camera_matrix.astype(
                    np.float64
                ),
                None,
                rvec,
                tvec,
            )

        rotation, _ = cv2.Rodrigues(rvec)

        projected, _ = cv2.projectPoints(
            object_points.astype(np.float64),
            rvec,
            tvec,
            camera_matrix.astype(np.float64),
            None,
        )
    except cv2.error:
        return None

    projected = projected.reshape(-1, 2)

    residuals = np.linalg.norm(
        projected - image_points,
        axis=1,
    )

    inliers = residuals <= 3.5

    if int(inliers.sum()) < 8:
        return None

    return (
        rotation.astype(np.float64),
        tvec.reshape(3).astype(np.float64),
        inliers,
        float(
            np.median(
                residuals[inliers]
            )
        ),
    )


class RGBDOdometryBaseline(Node):
    """Feature-based metric visual odometry using RGB-D observations."""

    def __init__(self) -> None:

        super().__init__(
            'rgbd_odometry_baseline'
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10,
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.cloud_callback,
            10,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10,
        )

        # Visual odometry publishes ONLY to /fusion/visual_odometry for EKF input.
        # The authoritative /localization/pose comes from slam_pose_adapter (when slam:=true)
        # or online_mapping pose_adapter (when slam:=false), not from RGB-D odometry.
        # Publishing a second conflicting pose to /localization/pose caused frame-change
        # discontinuities because visual_odom != map frames.
        self.odom_pub = self.create_publisher(
            Odometry,
            '/fusion/visual_odometry',
            10,
        )

        self.orb = cv2.ORB_create(
            nfeatures=1200,
            scaleFactor=1.2,
            nlevels=8,
            fastThreshold=5,
        )

        self.matcher = cv2.BFMatcher(
            cv2.NORM_HAMMING,
            crossCheck=False,
        )

        self.image_buffer = deque(
            maxlen=12
        )

        self.max_sync_delta = 0.020
        self.camera_k = None
        self.camera_size = None

        self.previous_gray = None
        self.previous_cloud = None
        self.previous_keypoints = None
        self.previous_descriptors = None
        self.previous_stamp = None

        self.pose_rotation = np.eye(
            3,
            dtype=np.float32,
        )

        self.pose_translation = np.zeros(
            3,
            dtype=np.float32,
        )

        self.pair_count = 0
        self.accepted_count = 0
        self.rejected_count = 0
        self.last_log_pair = 0

        self.last_inlier_count = 0
        self.last_inlier_ratio = 0.0
        self.last_reprojection_error = float('inf')

        self.get_logger().info(
            'Timestamp-synchronized RGB-D visual '
            'odometry baseline started.'
        )

    def camera_info_callback(
        self,
        msg: CameraInfo,
    ) -> None:
        """Store RGB camera intrinsics."""
        camera_matrix = np.asarray(
            msg.k,
            dtype=np.float64,
        ).reshape(3, 3)

        if (
            np.isfinite(camera_matrix).all()
            and camera_matrix[0, 0] > 0.0
            and camera_matrix[1, 1] > 0.0
        ):
            self.camera_k = camera_matrix
            self.camera_size = (
                int(msg.width),
                int(msg.height),
            )

    def image_callback(self, msg: Image) -> None:
        """Buffer RGB frames for timestamp pairing."""
        try:
            gray = decode_rgb(msg)
        except ValueError as exc:
            self.get_logger().warning(
                f'Image decode failed: {exc}'
            )
            return

        stamp = stamp_to_seconds(
            msg.header.stamp
        )

        self.image_buffer.append(
            (stamp, gray)
        )

        cutoff = (
            stamp - 0.50
        )

        while (
            self.image_buffer
            and self.image_buffer[0][0] < cutoff
        ):
            self.image_buffer.popleft()

    def cloud_callback(self, msg: PointCloud2) -> None:
        """Pair a depth cloud with its nearest RGB frame."""
        try:
            cloud = extract_xyz(msg)
        except (ValueError, BufferError) as exc:
            self.get_logger().warning(
                f'Point cloud decode failed: {exc}'
            )
            return

        stamp = stamp_to_seconds(
            msg.header.stamp
        )

        index = nearest_buffer_index(
            self.image_buffer,
            stamp,
        )

        if index is None:
            return

        sync_delta = abs(
            self.image_buffer[index][0]
            - stamp
        )

        if sync_delta > self.max_sync_delta:
            return

        _, gray = self.image_buffer[index]

        del self.image_buffer[index]

        self.process_pair(
            gray,
            cloud,
            stamp,
            sync_delta,
        )

    def process_pair(
        self,
        gray: np.ndarray,
        cloud: np.ndarray,
        stamp: float,
        sync_delta: float,
    ) -> None:
        """Estimate metric RGB-D motion and publish the pose."""
        if gray.shape != cloud.shape[:2]:
            gray = cv2.resize(
                gray,
                (
                    cloud.shape[1],
                    cloud.shape[0],
                ),
                interpolation=cv2.INTER_AREA,
            )

        keypoints, descriptors = (
            self.orb.detectAndCompute(
                gray,
                None,
            )
        )

        if (
            descriptors is None
            or len(keypoints) < 12
        ):
            self.previous_gray = gray
            self.previous_cloud = cloud.copy()
            self.previous_keypoints = keypoints
            self.previous_descriptors = descriptors
            self.previous_stamp = stamp
            self.publish_pose(stamp)
            return

        if self.previous_gray is None:
            self.previous_gray = gray
            self.previous_cloud = cloud.copy()
            self.previous_keypoints = keypoints
            self.previous_descriptors = descriptors
            self.previous_stamp = stamp
            self.publish_pose(stamp)
            return

        dt = (
            stamp
            - float(self.previous_stamp)
        )

        if (
            dt <= 0.0
            or dt > 0.25
        ):
            self.previous_gray = gray
            self.previous_cloud = cloud.copy()
            self.previous_keypoints = keypoints
            self.previous_descriptors = descriptors
            self.previous_stamp = stamp
            self.publish_pose(stamp)
            return

        self.pair_count += 1

        matches = self.matcher.knnMatch(
            self.previous_descriptors,
            descriptors,
            k=2,
        )

        previous_3d = []
        current_2d = []

        fallback_source = []
        fallback_target = []

        for pair in matches:
            if len(pair) != 2:
                continue

            first, second = pair

            if (
                first.distance
                >= 0.72 * second.distance
            ):
                continue

            previous_pixel = (
                self.previous_keypoints[
                    first.queryIdx
                ].pt
            )

            current_pixel = (
                keypoints[
                    first.trainIdx
                ].pt
            )

            pixel_jump = np.hypot(
                current_pixel[0]
                - previous_pixel[0],
                current_pixel[1]
                - previous_pixel[1],
            )

            if pixel_jump > 80.0:
                continue

            previous_point = sample_xyz(
                self.previous_cloud,
                previous_pixel[0],
                previous_pixel[1],
            )

            if previous_point is None:
                continue

            previous_3d.append(
                previous_point
            )

            current_2d.append(
                current_pixel
            )

            current_point = sample_xyz(
                cloud,
                current_pixel[0],
                current_pixel[1],
            )

            if current_point is not None:
                fallback_source.append(
                    previous_point
                )
                fallback_target.append(
                    current_point
                )

        accepted = False
        method = 'none'
        inlier_count = 0
        motion = 0.0
        reprojection_error = float('inf')

        # -------------------------------------------------------
        # Camera intrinsics.
        # -------------------------------------------------------
        camera_k = getattr(
            self,
            'camera_k',
            None,
        )

        camera_size = getattr(
            self,
            'camera_size',
            None,
        )

        height, width = gray.shape

        if (
            camera_k is not None
            and camera_size is not None
        ):
            camera_matrix = camera_k.copy()

            source_width, source_height = (
                camera_size
            )

            if (
                source_width > 0
                and source_height > 0
            ):
                scale_x = (
                    width
                    / float(source_width)
                )

                scale_y = (
                    height
                    / float(source_height)
                )

                camera_matrix[0, 0] *= scale_x
                camera_matrix[0, 2] *= scale_x
                camera_matrix[1, 1] *= scale_y
                camera_matrix[1, 2] *= scale_y
        else:
            focal = (
                width
                / (
                    2.0
                    * np.tan(
                        1.0472
                        * 0.5
                    )
                )
            )

            camera_matrix = np.array(
                [
                    [
                        focal,
                        0.0,
                        (width - 1.0)
                        * 0.5,
                    ],
                    [
                        0.0,
                        focal,
                        (height - 1.0)
                        * 0.5,
                    ],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )

        # -------------------------------------------------------
        # PRIMARY: metric 3D -> current 2D PnP.
        # -------------------------------------------------------
        if len(previous_3d) >= 12:
            object_points = (
                gazebo_xyz_to_optical(
                    np.asarray(
                        previous_3d,
                        dtype=np.float64,
                    )
                )
            )

            pnp_result = estimate_pnp(
                object_points,
                np.asarray(
                    current_2d,
                    dtype=np.float64,
                ),
                camera_matrix,
            )

            if pnp_result is not None:
                (
                    optical_rotation,
                    optical_translation,
                    inliers,
                    reprojection_error,
                ) = pnp_result

                rotation = (
                    GAZEBO_TO_OPTICAL.T
                    @ optical_rotation
                    @ GAZEBO_TO_OPTICAL
                )

                translation = (
                    GAZEBO_TO_OPTICAL.T
                    @ optical_translation
                )

                inlier_count = int(
                    inliers.sum()
                )

                inlier_ratio = (
                    inlier_count
                    / float(len(inliers))
                )

                motion = float(
                    np.linalg.norm(
                        translation
                    )
                )

                credible = (
                    inlier_count >= 12
                    and inlier_ratio >= 0.50
                    and reprojection_error <= 3.5
                    and motion
                    <= 1.6 * dt + 0.12
                    and rotation_angle(
                        rotation
                    )
                    <= min(
                        0.35,
                        2.0 * dt + 0.12,
                    )
                )

                if credible:
                    inverse_rotation = (
                        rotation.T
                    )

                    inverse_translation = (
                        -inverse_rotation
                        @ translation
                    )

                    self.pose_translation += (
                        self.pose_rotation
                        @ inverse_translation
                    )

                    self.pose_rotation = (
                        self.pose_rotation
                        @ inverse_rotation
                    )

                    u, _, vt = np.linalg.svd(
                        self.pose_rotation
                    )

                    self.pose_rotation = (
                        u @ vt
                    )

                    if (
                        np.linalg.det(
                            self.pose_rotation
                        )
                        < 0.0
                    ):
                        u[:, -1] *= -1.0
                        self.pose_rotation = (
                            u @ vt
                        )

                    self.accepted_count += 1
                    accepted = True
                    method = 'pnp'
                    motion = float(
                        np.linalg.norm(translation)
                    )
                    self.last_inlier_count = inlier_count
                    self.last_inlier_ratio = (
                        float(inlier_count) / len(previous_3d)
                    )
                    self.last_reprojection_error = reprojection_error

        # -------------------------------------------------------
        # FALLBACK: metric 3D -> 3D.
        # -------------------------------------------------------
        if (
            not accepted
            and len(fallback_source) >= 12
        ):
            (
                rotation,
                translation,
                inliers,
                residuals,
            ) = ransac_rigid_transform(
                np.asarray(
                    fallback_source,
                    dtype=np.float32,
                ),
                np.asarray(
                    fallback_target,
                    dtype=np.float32,
                ),
                iterations=180,
                threshold_m=0.05,
            )

            if (
                rotation is not None
                and translation is not None
            ):
                credible, _ = (
                    transform_is_credible(
                        rotation,
                        translation,
                        inliers,
                        residuals,
                        dt,
                    )
                )

                if credible:
                    inverse_rotation = (
                        rotation.T
                    )

                    inverse_translation = (
                        -inverse_rotation
                        @ translation
                    )

                    self.pose_translation += (
                        self.pose_rotation
                        @ inverse_translation
                    )

                    self.pose_rotation = (
                        self.pose_rotation
                        @ inverse_rotation
                    )

                    u, _, vt = np.linalg.svd(
                        self.pose_rotation
                    )

                    self.pose_rotation = (
                        u @ vt
                    )

                    if (
                        np.linalg.det(
                            self.pose_rotation
                        )
                        < 0.0
                    ):
                        u[:, -1] *= -1.0
                        self.pose_rotation = (
                            u @ vt
                        )

                    self.accepted_count += 1
                    accepted = True
                    method = '3d'
                    inlier_count = int(
                        inliers.sum()
                    )
                    motion = float(
                        np.linalg.norm(
                            translation
                        )
                    )
                    self.last_inlier_count = inlier_count
                    self.last_inlier_ratio = (
                        float(inlier_count) / len(fallback_source)
                    )
                    self.last_reprojection_error = float(
                        np.median(residuals[inliers])
                    ) if inliers.any() else 999.0

        if not accepted:
            self.rejected_count += 1

        if (
            self.pair_count
            - self.last_log_pair
            >= 10
        ):
            self.get_logger().info(
                'pair=%d features=%d corr=%d '
                'inliers=%d accepted=%d '
                'rejected=%d method=%s '
                'sync=%.3f motion=%.3f '
                'pose_x=%.3f pose_y=%.3f '
                'yaw=%.3f reproj=%.2f'
                % (
                    self.pair_count,
                    len(keypoints),
                    len(previous_3d),
                    inlier_count,
                    self.accepted_count,
                    self.rejected_count,
                    method,
                    sync_delta,
                    motion,
                    float(
                        self.pose_translation[0]
                    ),
                    float(
                        self.pose_translation[1]
                    ),
                    rotation_to_yaw(
                        self.pose_rotation
                    ),
                    reprojection_error,
                )
            )

            self.last_log_pair = (
                self.pair_count
            )

        self.previous_gray = gray
        self.previous_cloud = cloud.copy()
        self.previous_keypoints = keypoints
        self.previous_descriptors = descriptors
        self.previous_stamp = stamp
        self.publish_pose(stamp)

    def publish_pose(self, stamp: float) -> None:
        """Publish current planar visual odometry pose and odometry."""
        pose = PoseStamped()

        seconds = int(stamp)
        nanoseconds = int(
            round(
                (stamp - seconds)
                * 1_000_000_000
            )
        )

        pose.header.stamp.sec = seconds
        pose.header.stamp.nanosec = (
            nanoseconds
        )

        pose.header.frame_id = 'visual_odom'

        # Gazebo depth-camera coordinates are:
        #   +X = forward
        #   +Y = lateral
        #   +Z = vertical
        #
        # The UGV planar frame uses the same horizontal axes.
        pose.pose.position.x = float(
            self.pose_translation[0]
        )

        pose.pose.position.y = float(
            self.pose_translation[1]
        )

        pose.pose.position.z = 0.0

        yaw = rotation_to_yaw(
            self.pose_rotation
        )

        half_yaw = 0.5 * yaw

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(
            math.sin(half_yaw)
        )
        # Visual odometry pose is embedded in the Odometry message for EKF fusion.
        # We do not publish a separate PoseStamped to /localization/pose because
        # that topic is owned by slam_pose_adapter (slam:=true) or the baseline
        # pose_adapter (slam:=false). Publishing here caused frame conflicts.

        # Publish Odometry message for EKF fusion
        odom = Odometry()
        odom.header = pose.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose = pose.pose

        # Visual odometry covariance.
        #
        # The RGB-D PnP estimator in this simulation accumulates a biased
        # planar pose — it underestimates forward motion by ~3x and tracks
        # lateral motion in the wrong direction (observed: visual pose
        # 0.30 m vs Gazebo ground truth 1.00 m; y goes negative while
        # Gazebo y goes positive).  The PnP reports 100% inliers with low
        # reprojection error, so the quality scaling alone cannot catch
        # the systematic bias.
        #
        # The EKF also receives wheel-odometry velocities (vx, yaw_rate)
        # from Gazebo ground truth via ekf_sensor_adapter, plus IMU yaw —
        # both accurate.  A tight 0.01 (1 cm) covariance caused the EKF to
        # trust the biased visual pose over the accurate wheel-odometry
        # integration, producing odom->base_link divergence (EKF net
        # 0.19 m vs Gazebo 1.00 m).
        #
        # Raising the baseline to 10.0 m (position) and 1.0 rad (rotation)
        # lets the EKF weight the visual odometry weakly, relying primarily
        # on the accurate wheel-odometry + IMU fusion.  The visual odometry
        # still contributes a correction signal but cannot dominate the
        # estimate.  When no motion has ever been accepted, the pose has no
        # measurement behind it and gets an effectively-infinite covariance.
        base_pos_cov = 10.0
        base_rot_cov = 1.0

        if self.accepted_count == 0:
            # No motion has ever been accepted — the accumulated pose is
            # just the initial (0, 0, 0) with no measurement behind it.
            pos_cov = 10000.0
            rot_cov = 10000.0
        else:
            quality_factor = 1.0
            if self.last_inlier_ratio > 0.0:
                # Inverse relationship: fewer inliers = higher covariance
                quality_factor = max(1.0, 5.0 * (1.0 - self.last_inlier_ratio))

            if self.last_reprojection_error < 10.0:
                # High reprojection error increases uncertainty
                quality_factor *= (1.0 + self.last_reprojection_error / 2.0)

            pos_cov = base_pos_cov * quality_factor
            rot_cov = base_rot_cov * quality_factor

        # 6x6 covariance matrix [x, y, z, roll, pitch, yaw]
        # Only populate x, y, yaw (planar motion)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = pos_cov      # x
        odom.pose.covariance[7] = pos_cov      # y
        odom.pose.covariance[14] = 99999.0     # z (not used)
        odom.pose.covariance[21] = 99999.0     # roll (not used)
        odom.pose.covariance[28] = 99999.0     # pitch (not used)
        odom.pose.covariance[35] = rot_cov     # yaw

        self.odom_pub.publish(odom)


def main(args=None) -> None:
    """Run the RGB-D odometry baseline node."""
    rclpy.init(args=args)

    node = RGBDOdometryBaseline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
