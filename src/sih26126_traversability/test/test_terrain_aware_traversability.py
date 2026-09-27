import numpy as np

from sih26126_traversability.terrain_aware_traversability import (
    apply_terrain_factor,
    terrain_factor,
)


def test_flat_high_confidence_preserves_score():
    factor = terrain_factor(
        'FLAT',
        1.0,
        1.0,
        1.0,
    )
    assert factor == 1.0


def test_sloped_reduces_score():
    factor = terrain_factor(
        'SLOPED',
        1.0,
        1.0,
        1.0,
    )
    assert 0.0 < factor < 1.0


def test_steep_penalizes_more_than_sloped():
    sloped = terrain_factor(
        'SLOPED',
        1.0,
        1.0,
        1.0,
    )
    steep = terrain_factor(
        'STEEP',
        1.0,
        1.0,
        1.0,
    )
    assert steep < sloped


def test_rough_penalizes_more_than_sloped():
    rough = terrain_factor(
        'ROUGH',
        1.0,
        1.0,
        1.0,
    )
    sloped = terrain_factor(
        'SLOPED',
        1.0,
        1.0,
        1.0,
    )
    assert rough < sloped


def test_confidence_blends_toward_baseline():
    high = terrain_factor(
        'STEEP',
        1.0,
        1.0,
        1.0,
    )
    low = terrain_factor(
        'STEEP',
        0.2,
        1.0,
        1.0,
    )

    assert 0.0 < high < low < 1.0


def test_zero_reliability_preserves_baseline():
    factor = terrain_factor(
        'STEEP',
        0.0,
        1.0,
        1.0,
    )
    assert factor == 1.0


def test_apply_factor_reduces_pixels():
    image = np.array(
        [[0, 64, 128, 255]],
        dtype=np.uint8,
    )

    result = apply_terrain_factor(
        image,
        0.5,
    )

    assert result.tolist() == [[0, 32, 64, 128]]
