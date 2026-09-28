"""Unit tests for the terrain-aware advisory planner baseline."""

from sih26126_planner.planner_baseline import (
    planner_state,
    score_to_speed,
)


def test_high_score_forward():
    """Verify a high traversability score produces FORWARD."""
    assert planner_state(0.90) == 'FORWARD'


def test_middle_score_slow():
    """Verify a medium traversability score produces SLOW."""
    assert planner_state(0.50) == 'SLOW'


def test_low_score_stop():
    """Verify a low traversability score produces STOP."""
    assert planner_state(0.10) == 'STOP'


def test_high_score_speed_positive():
    """Verify a high score produces a positive bounded speed."""
    speed = score_to_speed(0.90)
    assert 0.0 < speed <= 0.30


def test_low_score_zero_speed():
    """Verify a low score produces zero speed."""
    assert score_to_speed(0.10) == 0.0


def test_score_clamped_above_one():
    """Verify scores above one are clamped."""
    assert score_to_speed(1.5) == 0.30


def test_score_clamped_below_zero():
    """Verify scores below zero are clamped."""
    assert score_to_speed(-1.0) == 0.0


def test_slow_speed_less_than_forward_speed():
    """Verify slow-mode speed is below forward-mode speed."""
    slow = score_to_speed(0.50)
    forward = score_to_speed(0.90)

    assert 0.0 < slow < forward
