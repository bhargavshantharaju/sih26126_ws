import numpy as np

from sih26126_geometry.geometry_baseline import (
    fit_plane_ransac,
)


def test_flat_ground_plane():
    rng = np.random.default_rng(26126)

    x = rng.uniform(
        -3,
        3,
        800,
    )

    y = rng.uniform(
        -2,
        2,
        800,
    )

    z = np.full_like(x, 1.0)

    points = np.column_stack(
        [x, y, z]
    ).astype(np.float32)

    result = fit_plane_ransac(
        points,
        iterations=80,
        distance_threshold=0.03,
    )

    assert result is not None

    plane, inliers, rmse = result

    assert inliers > 700
    assert rmse < 0.02


def test_plane_with_outliers():
    rng = np.random.default_rng(26126)

    x = rng.uniform(
        -3,
        3,
        700,
    )

    y = rng.uniform(
        -2,
        2,
        700,
    )

    z = 1.0 + rng.normal(
        0,
        0.005,
        700,
    )

    ground = np.column_stack(
        [x, y, z]
    )

    outliers = rng.uniform(
        -4,
        4,
        (120, 3),
    )

    points = np.vstack(
        [ground, outliers]
    ).astype(np.float32)

    result = fit_plane_ransac(
        points,
        iterations=120,
        distance_threshold=0.04,
    )

    assert result is not None

    plane, inliers, rmse = result

    assert inliers > 600
    assert rmse < 0.03
