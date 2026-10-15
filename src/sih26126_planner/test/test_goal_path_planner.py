"""Unit tests for the goal-directed path planner."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from sih26126_planner.goal_path_planner import (
    build_straight_path,
    distance_to_goal,
    navigation_state,
    quaternion_to_yaw,
    route_lookahead_target,
    wrap_angle,
)
from sih26126_planner.planner_baseline import score_to_speed


def test_quaternion_zero_yaw():
    """Verify identity quaternion produces zero yaw."""
    assert quaternion_to_yaw(
        0.0,
        0.0,
        0.0,
        1.0,
    ) == 0.0


def test_quaternion_ninety_degree_yaw():
    """Verify a ninety-degree quaternion conversion."""
    half = math.pi / 4.0

    yaw = quaternion_to_yaw(
        0.0,
        0.0,
        math.sin(half),
        math.cos(half),
    )

    assert math.isclose(
        yaw,
        math.pi / 2.0,
        abs_tol=1e-6,
    )


def test_wrap_angle():
    """Verify angle wrapping."""
    wrapped = wrap_angle(3.0 * math.pi)
    assert math.isclose(
        wrapped,
        math.pi,
        abs_tol=1e-6,
    )


def test_distance_to_goal():
    """Verify planar goal distance."""
    assert math.isclose(
        distance_to_goal(
            0.0,
            0.0,
            3.0,
            4.0,
        ),
        5.0,
    )


def test_goal_reached_state():
    """Verify a point inside the goal tolerance."""
    assert (
        navigation_state(
            0.10,
            0.25,
        )
        == 'GOAL_REACHED'
    )


def test_navigate_state():
    """Verify a point outside the goal tolerance."""
    assert (
        navigation_state(
            2.0,
            0.25,
        )
        == 'NAVIGATE'
    )


def test_path_starts_at_current_pose():
    """Verify generated path starts at the current pose."""
    path = build_straight_path(
        1.0,
        2.0,
        4.0,
        2.0,
        'odom',
        None,
        0.25,
    )

    assert math.isclose(
        path.poses[0].pose.position.x,
        1.0,
    )
    assert math.isclose(
        path.poses[0].pose.position.y,
        2.0,
    )


def test_path_ends_at_goal():
    """Verify generated path ends exactly at the goal."""
    path = build_straight_path(
        1.0,
        2.0,
        4.0,
        2.0,
        'odom',
        None,
        0.25,
    )

    last = path.poses[-1].pose

    assert math.isclose(
        last.position.x,
        4.0,
    )
    assert math.isclose(
        last.position.y,
        2.0,
    )


def test_path_has_multiple_samples():
    """Verify a non-zero path contains intermediate poses."""
    path = build_straight_path(
        0.0,
        0.0,
        2.0,
        0.0,
        'odom',
        None,
        0.25,
    )

    assert len(path.poses) > 2


def test_route_lookahead_uses_nearby_map_waypoint_not_far_goal():
    path = Path()
    path.header.frame_id = 'odom'
    for x, y in ((0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.5), (2.0, 1.0)):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)
    target = route_lookahead_target(path, 0.05, 0.0, 0.9)
    assert target is not None
    assert target.x == 1.0
    assert target.y == 0.0


# ---------------------------------------------------------------------------
# Terrain fallback regression tests
# The fallback score of 0.5 must produce a conservative (non-zero, non-max)
# speed, confirming neither a stop nor an unsafe full-speed command.
# ---------------------------------------------------------------------------


def test_terrain_fallback_score_gives_conservative_speed():
    """Fallback score 0.5 yields a positive but well-below-max speed."""
    max_speed = 0.30
    fallback_score = 0.5  # configured terrain_fallback_score default
    speed = score_to_speed(fallback_score, max_speed, stop_threshold=0.30,
                           slow_threshold=0.70)
    # Must be > 0 (navigation is not blocked)
    assert speed > 0.0, 'Fallback should not stop navigation'
    # Must be well below max speed (conservative — terrain is unknown)
    assert speed <= max_speed * 0.5, (
        f'Fallback speed {speed:.3f} must be at most 50% of max {max_speed}')


def test_terrain_fallback_score_does_not_exceed_good_terrain_speed():
    """A fallback score (0.5) gives less speed than a good terrain score (0.9)."""
    max_speed = 0.30
    fallback_speed = score_to_speed(0.5, max_speed)
    good_speed = score_to_speed(0.9, max_speed)
    assert fallback_speed < good_speed, (
        'Fallback score must yield less speed than confidently good terrain')


def test_terrain_stop_threshold_blocks_navigation():
    """A score below the stop threshold must yield zero speed."""
    max_speed = 0.30
    speed = score_to_speed(0.25, max_speed, stop_threshold=0.30)
    assert math.isclose(speed, 0.0), (
        'Score below stop_threshold must produce zero speed')
