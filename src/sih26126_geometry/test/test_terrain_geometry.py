import math

import numpy as np

from sih26126_geometry.terrain_geometry import (
    compute_roughness_m,
    compute_slope_degrees,
    compute_surface_normals,
    finite_mask,
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
