import math

import numpy as np

from sih26126_localization.rgbd_odometry_baseline import (
    nearest_buffer_index,
    ransac_rigid_transform,
    rigid_transform,
    rotation_to_yaw,
    transform_is_credible,
)


def test_rigid_transform_recovers_known_motion():
    rng = np.random.default_rng(1234)

    source = rng.normal(
        size=(30, 3)
    ).astype(np.float32)

    yaw = math.radians(12.0)

    rotation_true = np.array(
        [
            [math.cos(yaw), 0.0, math.sin(yaw)],
            [0.0, 1.0, 0.0],
            [-math.sin(yaw), 0.0, math.cos(yaw)],
        ],
        dtype=np.float32,
    )

    translation_true = np.array(
        [0.15, -0.04, 0.22],
        dtype=np.float32,
    )

    target = (
        source @ rotation_true.T
        + translation_true
    )

    rotation, translation = rigid_transform(
        source,
        target,
    )

    assert rotation is not None
    assert translation is not None

    assert np.allclose(
        rotation,
        rotation_true,
        atol=1e-5,
    )

    assert np.allclose(
        translation,
        translation_true,
        atol=1e-5,
    )


def test_ransac_rejects_outliers():
    rng = np.random.default_rng(5678)

    source = rng.normal(
        size=(40, 3)
    ).astype(np.float32)

    translation_true = np.array(
        [0.10, 0.00, 0.20],
        dtype=np.float32,
    )

    target = (
        source
        + translation_true
    )

    target[::5] += np.array(
        [2.0, -1.5, 1.0],
        dtype=np.float32,
    )

    (
        rotation,
        translation,
        inliers,
        residuals,
    ) = ransac_rigid_transform(
        source,
        target,
        iterations=120,
        threshold_m=0.05,
    )

    assert rotation is not None
    assert translation is not None
    assert int(inliers.sum()) >= 30

    assert np.all(
        np.isfinite(residuals)
    )

    assert np.allclose(
        translation,
        translation_true,
        atol=0.03,
    )


def test_yaw_conversion():
    yaw = math.radians(20.0)

    rotation = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )

    assert math.isclose(
        rotation_to_yaw(rotation),
        yaw,
        abs_tol=1e-6,
    )


def test_nearest_timestamp_index():
    buffer = [
        (1.00, None),
        (1.10, None),
        (1.21, None),
    ]

    assert nearest_buffer_index(
        buffer,
        1.09,
    ) == 1


def test_transform_gate_accepts_small_motion():
    rotation = np.eye(
        3,
        dtype=np.float32,
    )

    translation = np.array(
        [0.0, 0.0, 0.025],
        dtype=np.float32,
    )

    inliers = np.ones(
        12,
        dtype=bool,
    )

    residuals = np.full(
        12,
        0.01,
        dtype=np.float32,
    )

    accepted, reason = transform_is_credible(
        rotation,
        translation,
        inliers,
        residuals,
        0.10,
    )

    assert accepted
    assert reason == 'accepted'


def test_transform_gate_rejects_large_rotation():
    yaw = math.radians(90.0)

    rotation = np.array(
        [
            [math.cos(yaw), 0.0, math.sin(yaw)],
            [0.0, 1.0, 0.0],
            [-math.sin(yaw), 0.0, math.cos(yaw)],
        ],
        dtype=np.float32,
    )

    translation = np.zeros(
        3,
        dtype=np.float32,
    )

    inliers = np.ones(
        12,
        dtype=bool,
    )

    residuals = np.full(
        12,
        0.01,
        dtype=np.float32,
    )

    accepted, reason = transform_is_credible(
        rotation,
        translation,
        inliers,
        residuals,
        0.10,
    )

    assert not accepted
    assert reason == 'rotation_too_large'


def test_transform_gate_rejects_large_translation():
    rotation = np.eye(
        3,
        dtype=np.float32,
    )

    translation = np.array(
        [1.0, 0.0, 0.0],
        dtype=np.float32,
    )

    inliers = np.ones(
        12,
        dtype=bool,
    )

    residuals = np.full(
        12,
        0.01,
        dtype=np.float32,
    )

    accepted, reason = transform_is_credible(
        rotation,
        translation,
        inliers,
        residuals,
        0.10,
    )

    assert not accepted
    assert reason == 'motion_too_large'
