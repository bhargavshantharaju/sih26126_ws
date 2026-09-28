import math

import numpy as np

from sih26126_geometry.terrain_geometry import (
    compute_roughness_m,
    compute_slope_degrees,
    compute_surface_normals,
    finite_mask,
    fit_ground_plane,
)


def make_ground(h=80, w=120):
    x = np.linspace(1.0, 8.0, w, dtype=np.float32)
    y = np.linspace(-3.0, 3.0, h, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)

    xyz = np.stack(
        (
            xx,
            yy,
            np.full_like(xx, -1.12),
        ),
        axis=2,
    ).astype(np.float32)

    return xyz


def make_ramp(angle_deg, h=80, w=120):
    xyz = make_ground(h, w)
    slope = math.tan(math.radians(angle_deg))
    xyz[:, :, 2] += slope * (
        xyz[:, :, 0] - np.mean(xyz[:, :, 0])
    )
    return xyz


def test_flat_ground_slope_near_zero():
    xyz = make_ground()
    valid = finite_mask(xyz, 0.1, 15.0)

    normals = compute_surface_normals(xyz, valid)
    slope = compute_slope_degrees(normals, valid)

    values = slope[np.isfinite(slope)]

    assert values.size > 1000
    assert float(np.median(values)) < 0.5


def test_twenty_degree_ramp():
    xyz = make_ramp(20.0)
    valid = finite_mask(xyz, 0.1, 15.0)

    normals = compute_surface_normals(xyz, valid)
    slope = compute_slope_degrees(normals, valid)

    values = slope[np.isfinite(slope)]

    assert values.size > 1000
    assert abs(float(np.median(values)) - 20.0) < 2.0


def test_ground_plane_fit_rejects_vertical_object_surfaces():
    xyz = make_ground()
    height, width, _ = xyz.shape

    # Simulate a broad visible tree trunk patch as a vertical plane.
    rows = slice(30, 62)
    cols = slice(48, 74)
    xyz[rows, cols, 0] = 3.0
    xyz[rows, cols, 1] = np.linspace(
        -0.5,
        0.5,
        cols.stop - cols.start,
        dtype=np.float32,
    )[None, :]
    xyz[rows, cols, 2] = np.linspace(
        -1.1,
        1.8,
        rows.stop - rows.start,
        dtype=np.float32,
    )[:, None]

    valid = finite_mask(xyz, 0.1, 15.0)
    fit_region = np.ones((height, width), dtype=bool)
    result = fit_ground_plane(xyz, valid, fit_region)

    assert result is not None
    plane, ground_inliers, rmse, candidate_count = result
    slope = math.degrees(math.acos(float(plane[2])))

    assert slope < 0.5
    assert ground_inliers[rows, cols].mean() < 0.1
    assert int(ground_inliers.sum()) > 1000
    assert candidate_count >= int(ground_inliers.sum())
    assert rmse < 0.002


def test_ground_plane_fit_preserves_real_slope():
    xyz = make_ramp(20.0)
    valid = finite_mask(xyz, 0.1, 15.0)
    fit_region = np.ones(valid.shape, dtype=bool)

    result = fit_ground_plane(xyz, valid, fit_region)

    assert result is not None
    plane, _, _, _ = result
    slope = math.degrees(math.acos(float(plane[2])))
    assert abs(slope - 20.0) < 1.0


def test_roughness_distinguishes_flat_and_rough():
    flat = make_ground()
    rough = flat.copy()

    rng = np.random.default_rng(42)
    rough[:, :, 2] += rng.normal(
        0.0,
        0.04,
        size=rough.shape[:2],
    ).astype(np.float32)

    flat_valid = finite_mask(flat, 0.1, 15.0)
    rough_valid = finite_mask(rough, 0.1, 15.0)

    flat_r = compute_roughness_m(flat, flat_valid)
    rough_r = compute_roughness_m(rough, rough_valid)

    flat_values = flat_r[np.isfinite(flat_r)]
    rough_values = rough_r[np.isfinite(rough_r)]

    assert float(np.median(flat_values)) < 1e-3
    assert float(np.median(rough_values)) > 0.005
