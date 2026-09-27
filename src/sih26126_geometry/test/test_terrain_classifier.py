import math

from sih26126_geometry.terrain_classifier import (
    CLASS_FLAT,
    CLASS_ROUGH,
    CLASS_SLOPED,
    CLASS_STEEP,
    CLASS_UNKNOWN,
    classify_terrain,
)


def test_flat_terrain():
    label, confidence = classify_terrain(
        1.0,
        0.01,
        0.98,
        0.80,
    )

    assert label == CLASS_FLAT
    assert 0.0 <= confidence <= 1.0


def test_sloped_terrain():
    label, confidence = classify_terrain(
        13.862,
        0.01359,
        0.987,
        0.8175,
    )

    assert label == CLASS_SLOPED
    assert confidence > 0.0


def test_rough_terrain():
    label, confidence = classify_terrain(
        3.0,
        0.08,
        0.95,
        0.80,
    )

    assert label == CLASS_ROUGH
    assert confidence > 0.0


def test_steep_terrain():
    label, confidence = classify_terrain(
        35.0,
        0.01,
        0.95,
        0.80,
    )

    assert label == CLASS_STEEP
    assert confidence > 0.0


def test_unknown_low_confidence():
    label, confidence = classify_terrain(
        5.0,
        0.01,
        0.10,
        0.80,
    )

    assert label == CLASS_UNKNOWN
    assert 0.0 <= confidence <= 1.0


def test_unknown_low_valid_ratio():
    label, confidence = classify_terrain(
        5.0,
        0.01,
        0.90,
        0.10,
    )

    assert label == CLASS_UNKNOWN
    assert 0.0 <= confidence <= 1.0


def test_unknown_nan():
    label, confidence = classify_terrain(
        math.nan,
        0.01,
        0.90,
        0.80,
    )

    assert label == CLASS_UNKNOWN
    assert confidence == 0.0


def test_boundary_flat_to_sloped():
    label, _ = classify_terrain(
        5.0,
        0.01,
        0.95,
        0.80,
    )

    assert label == CLASS_SLOPED


def test_boundary_sloped_to_steep():
    label, _ = classify_terrain(
        30.0,
        0.01,
        0.95,
        0.80,
    )

    assert label == CLASS_STEEP
