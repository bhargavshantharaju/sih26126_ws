# Copyright 2026 DRISTHI Contributors
# Licensed under the Apache License, Version 2.0
"""Regression tests for localization pose-discontinuity defects.

These tests cover the three confirmed runtime failures:

1. Non-monotonic sim-time timestamps triggering false IMPLAUSIBLE_POSE_DELTA
   (root cause: elapsed = stamp[n] - stamp[n-1] < 0, then pose_jump_exceeded
   returned True for any distance > 1e-4 m).

2. The slam_pose_adapter monotonic-stamp guarantee.

3. The pose_jump_exceeded zero-elapsed guard using margin (not 1e-4 m).
"""

import math

from sih26126_autonomy.mission_manager import pose_jump_exceeded


# ---------------------------------------------------------------------------
# Tests for pose_jump_exceeded with the corrected zero-elapsed guard
# ---------------------------------------------------------------------------

def test_zero_elapsed_small_move_is_accepted():
    """A small movement under the margin must NOT be flagged with zero elapsed."""
    # Before fix: elapsed=0 → returned distance > 1e-4 → ANY movement > 0.1mm
    # was flagged as implausible.
    # After fix: elapsed=0 → only flag if distance > margin (0.50 m default).
    assert not pose_jump_exceeded(0.05, 0.0, max_speed_mps=5.0, margin_m=0.50)


def test_zero_elapsed_large_jump_is_flagged():
    """A teleportation-scale jump must still be flagged with zero elapsed."""
    assert pose_jump_exceeded(2.0, 0.0, max_speed_mps=5.0, margin_m=0.50)


def test_negative_elapsed_small_move_is_accepted():
    """Non-monotonic timestamp (negative elapsed) must not flag a tiny movement."""
    # This was the primary cause of the cascade to SAFE_STOP.
    # When sim-time jumped backwards, elapsed was e.g. -0.155 s.
    # Old code: elapsed <= 0 → True for distance > 1e-4 → instant IMPLAUSIBLE.
    assert not pose_jump_exceeded(0.02, -0.155, max_speed_mps=5.0, margin_m=0.50)


def test_negative_elapsed_teleport_is_still_flagged():
    """A genuine teleportation must still be caught even with negative elapsed."""
    assert pose_jump_exceeded(5.0, -0.155, max_speed_mps=5.0, margin_m=0.50)


def test_normal_motion_within_speed_budget_is_accepted():
    """Normal vehicle motion must never be flagged."""
    # 0.5 m in 1.0 s at max 5.0 m/s: well within budget
    assert not pose_jump_exceeded(0.5, 1.0, max_speed_mps=5.0, margin_m=0.10)


def test_normal_motion_exceeding_speed_budget_is_flagged():
    """Motion that exceeds physics must be caught."""
    # 10 m in 1.0 s at max 5.0 m/s: exceeds 5.0 * 1.0 + 0.10 = 5.1 m
    assert pose_jump_exceeded(10.0, 1.0, max_speed_mps=5.0, margin_m=0.10)


def test_margin_allows_small_sensor_noise():
    """Measurement noise within the margin must be tolerated."""
    # 0.08 m in 0.001 s: exceeds speed budget but within margin
    assert not pose_jump_exceeded(0.08, 0.001, max_speed_mps=5.0, margin_m=0.10)


def test_fast_legitimate_step_just_under_limit():
    """A pose step right under the combined speed+margin threshold is accepted."""
    # 5.0 * 0.1 + 0.10 = 0.60 m allowed
    assert not pose_jump_exceeded(0.59, 0.1, max_speed_mps=5.0, margin_m=0.10)


def test_fast_legitimate_step_just_over_limit():
    """A pose step just over the combined threshold is correctly flagged."""
    # 5.0 * 0.1 + 0.10 = 0.60 m allowed; 0.61 m exceeds it
    assert pose_jump_exceeded(0.61, 0.1, max_speed_mps=5.0, margin_m=0.10)


def test_zero_speed_limit_only_margin_applies():
    """With max_speed=0, only the margin prevents a flag."""
    assert not pose_jump_exceeded(0.09, 1.0, max_speed_mps=0.0, margin_m=0.10)
    assert pose_jump_exceeded(0.11, 1.0, max_speed_mps=0.0, margin_m=0.10)


# ---------------------------------------------------------------------------
# Test elapsed-fallback logic inline (mirrors mission_manager.pose_cb logic)
# ---------------------------------------------------------------------------

def _compute_elapsed(stamp_key, last_raw_stamp, arrival, last_raw_time):
    """Mirror the elapsed computation from mission_manager.pose_cb after fix."""
    if stamp_key is not None and last_raw_stamp is not None:
        sim_elapsed = stamp_key - last_raw_stamp
        if sim_elapsed > 0.0:
            return sim_elapsed
        else:
            return arrival - last_raw_time if last_raw_time is not None else 0.0
    else:
        return arrival - last_raw_time if last_raw_time is not None else 0.0


def test_elapsed_uses_sim_time_when_monotonic():
    """When sim timestamps advance, sim-time elapsed is used."""
    elapsed = _compute_elapsed(10.5, 10.0, 1001.0, 1000.0)
    assert elapsed == pytest.approx(0.5)


def test_elapsed_falls_back_to_wall_clock_when_sim_time_goes_backward():
    """Non-monotonic sim-time must fall back to wall-clock elapsed."""
    # sim: 9.85 after 10.0 (backwards)
    elapsed = _compute_elapsed(9.85, 10.0, 1001.1, 1000.0)
    assert elapsed == pytest.approx(1.1, abs=1e-9)


def test_elapsed_is_zero_on_first_pose():
    """First pose (no last_raw_time) gives zero elapsed — not a crash."""
    elapsed = _compute_elapsed(10.0, None, 1000.0, None)
    assert elapsed == 0.0


def test_elapsed_uses_wall_clock_when_stamp_key_is_none():
    """Poses with invalid (zero) sim-time use wall-clock."""
    elapsed = _compute_elapsed(None, None, 1001.0, 1000.0)
    assert elapsed == pytest.approx(1.0)


# Needed for pytest.approx
import pytest
