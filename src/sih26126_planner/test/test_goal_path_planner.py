"""Unit tests for the goal-directed path planner."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

from sih26126_planner.goal_path_planner import (
    build_straight_path,
    distance_to_goal,
    navigation_state,
    path_cost_score,
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


# ---------------------------------------------------------------------------
# Cost-map integration tests
#
# These tests verify that path_cost_score correctly evaluates the fused
# cost map along a planned path, and that the planner's speed output changes
# when map costs change — the core requirement for /map/cost_map consumption.
# ---------------------------------------------------------------------------


def _make_cost_map(width, height, resolution, origin_x=0.0, origin_y=0.0,
                   fill=0):
    """Build an OccupancyGrid suitable for path_cost_score."""
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.position.x = origin_x
    grid.info.origin.position.y = origin_y
    grid.data = [fill] * (width * height)
    return grid


def _set_cell(grid, col, row, value):
    """Set a single cell in the cost map."""
    grid.data[row * grid.info.width + col] = value


class TestPathCostScoreBasics:
    """Verify path_cost_score handles None, empty, and all-free maps."""

    def test_none_cost_map_returns_zero(self):
        """No cost map available → neutral 0.0 (graceful degradation)."""
        path = build_straight_path(0.0, 0.0, 1.0, 0.0, 'map', None, 0.25)
        assert path_cost_score(path, None) == 0.0

    def test_empty_cost_map_data_returns_zero(self):
        """Cost map with empty data → 0.0."""
        path = build_straight_path(0.0, 0.0, 1.0, 0.0, 'map', None, 0.25)
        grid = OccupancyGrid()
        grid.data = []
        assert path_cost_score(path, grid) == 0.0

    def test_empty_path_returns_zero(self):
        """No path poses → 0.0."""
        empty_path = Path()
        grid = _make_cost_map(10, 10, 0.25)
        assert path_cost_score(empty_path, grid) == 0.0

    def test_all_free_cost_map_returns_zero(self):
        """All cells free (value 0) → score 0.0."""
        path = build_straight_path(0.0, 0.0, 2.0, 0.0, 'map', None, 0.25)
        grid = _make_cost_map(10, 10, 0.25, fill=0)
        assert path_cost_score(path, grid) == 0.0


class TestPathCostScoreCellValues:
    """Verify cost values are normalised correctly from the cost map."""

    def test_medium_cost_produces_nonzero_score(self):
        """Cells with cost 50 → score 0.5 along the path."""
        # Path: 0.0 → 0.5 (1 cell at resolution 0.25 → col 0 and col 1-2)
        path = build_straight_path(0.0, 0.0, 0.5, 0.0, 'map', None, 0.25)
        grid = _make_cost_map(10, 10, 0.25, fill=50)
        score = path_cost_score(path, grid)
        assert 0.0 < score <= 1.0
        assert abs(score - 0.5) < 0.01, f'Expected ~0.5, got {score}'

    def test_near_lethal_cell_contributes_full_penalty(self):
        """Cells with cost >= 99 → contribute 1.0 to the average."""
        # Single-row grid, path goes through one cell at cost 99
        path = build_straight_path(0.0, 0.0, 0.25, 0.0, 'map', None, 0.25)
        grid = _make_cost_map(4, 4, 0.25, fill=0)
        _set_cell(grid, 0, 0, 99)
        score = path_cost_score(path, grid)
        # At least one sampled cell is 99 → score > 0
        assert score > 0.0, 'Near-lethal cell must contribute to path cost'

    def test_unknown_cell_contributes_half(self):
        """Cells with value -1 (unknown) → contribute 0.5 to the average."""
        path = build_straight_path(0.0, 0.0, 0.5, 0.0, 'map', None, 0.25)
        grid = _make_cost_map(4, 4, 0.25, fill=-1)
        score = path_cost_score(path, grid)
        assert abs(score - 0.5) < 0.01, f'Expected ~0.5 for unknown, got {score}'

    def test_cell_outside_footprint_contributes_half(self):
        """Path waypoints outside the cost map → treated as unknown (0.5)."""
        # Cost map covers 0..1m, path goes to 2m (outside)
        path = build_straight_path(0.0, 0.0, 2.0, 0.0, 'map', None, 0.25)
        grid = _make_cost_map(2, 2, 0.25, fill=0)  # covers 0..0.5m
        score = path_cost_score(path, grid)
        # Some cells are inside (cost 0), some outside (cost 0.5)
        # Score must be between 0.0 and 0.5 (mix of free and unknown)
        assert 0.0 < score <= 0.5, (
            f'Mixed inside/outside cells must give 0 < score <= 0.5, got {score}')


class TestRouteCostChangesWithMapCost:
    """The key integration test: route costs change when map costs change."""

    def test_higher_cost_map_produces_higher_path_score(self):
        """Same path, higher cost map values → higher path_cost_score."""
        path = build_straight_path(0.0, 0.0, 1.0, 0.0, 'map', None, 0.25)

        grid_low = _make_cost_map(10, 10, 0.25, fill=10)
        grid_high = _make_cost_map(10, 10, 0.25, fill=80)

        score_low = path_cost_score(path, grid_low)
        score_high = path_cost_score(path, grid_high)

        assert score_high > score_low, (
            f'High-cost map ({score_high:.3f}) must exceed '
            f'low-cost map ({score_low:.3f}) for the same path')

    def test_zero_cost_vs_high_cost_score_difference(self):
        """All-free vs all-occupied cost map must produce different scores."""
        path = build_straight_path(0.0, 0.0, 1.0, 0.0, 'map', None, 0.25)

        grid_free = _make_cost_map(10, 10, 0.25, fill=0)
        grid_blocked = _make_cost_map(10, 10, 0.25, fill=99)

        score_free = path_cost_score(path, grid_free)
        score_blocked = path_cost_score(path, grid_blocked)

        assert score_free == 0.0, 'Free cost map must give zero score'
        assert score_blocked > 0.0, 'Blocked cost map must give positive score'
        assert score_blocked > score_free, (
            'Blocked must exceed free — the core integration requirement')

    def test_partial_high_cost_path_segment_penalized(self):
        """A path crossing one high-cost cell is penalized vs all-free."""
        # 4-cell straight path along X
        path = build_straight_path(0.0, 0.0, 0.75, 0.0, 'map', None, 0.25)

        grid_free = _make_cost_map(10, 10, 0.25, fill=0)
        grid_mixed = _make_cost_map(10, 10, 0.25, fill=0)
        # Make one cell high cost
        _set_cell(grid_mixed, 2, 0, 80)

        score_free = path_cost_score(path, grid_free)
        score_mixed = path_cost_score(path, grid_mixed)

        assert score_mixed > score_free, (
            f'Mixed path ({score_mixed:.3f}) must exceed free path '
            f'({score_free:.3f}) — partial high-cost cell must be penalized')

    def test_invalid_cost_map_dimensions_handled_safely(self):
        """Zero-width or zero-resolution cost map → 0.0 (no crash)."""
        path = build_straight_path(0.0, 0.0, 1.0, 0.0, 'map', None, 0.25)

        grid_zero_w = OccupancyGrid()
        grid_zero_w.info.width = 0
        grid_zero_w.info.height = 10
        grid_zero_w.info.resolution = 0.25
        grid_zero_w.data = []
        assert path_cost_score(path, grid_zero_w) == 0.0

        grid_zero_res = OccupancyGrid()
        grid_zero_res.info.width = 10
        grid_zero_res.info.height = 10
        grid_zero_res.info.resolution = 0.0
        grid_zero_res.data = [0] * 100
        assert path_cost_score(path, grid_zero_res) == 0.0
