"""Tests for spatial local path selection."""

import math

import numpy as np

from sih26126_planner.local_path_selector import (
    angle_to_pixel,
    build_local_path,
    goal_angle_from_pose,
    obstacle_clearance,
    obstacle_clearance_score,
    sample_corridor_score,
    score_candidate,
    select_best_candidate,
    wrap_angle,
)


def test_wrap_angle():
    """Angles are wrapped to the expected range."""
    assert math.isclose(wrap_angle(0.0), 0.0)
    assert math.isclose(wrap_angle(3.5), 3.5 - 2.0 * math.pi)
    assert math.isclose(wrap_angle(math.pi), math.pi)
    assert math.isclose(wrap_angle(-math.pi), math.pi)


def test_goal_angle_straight_ahead():
    """A goal directly ahead has zero body-frame bearing."""
    angle = goal_angle_from_pose(0.0, 0.0, 0.0, 6.0, 0.0)
    assert math.isclose(angle, 0.0, abs_tol=1e-9)


def test_goal_angle_left():
    """A goal to the left has positive bearing."""
    angle = goal_angle_from_pose(0.0, 0.0, 0.0, 6.0, 6.0)
    assert angle > 0.0


def test_angle_to_pixel_center():
    """Zero heading projects to the principal point."""
    assert angle_to_pixel(0.0, 554.25, 320.0, 640) == 320


def test_angle_to_pixel_left_moves_left():
    """Positive left angle moves toward smaller image columns."""
    assert angle_to_pixel(
        math.radians(10.0),
        554.25,
        320.0,
        640,
    ) < 320


def test_sample_corridor_score_good_region():
    """A good corridor receives a high normalized score."""
    image = np.full((480, 640), 230, dtype=np.uint8)
    score, validity = sample_corridor_score(image, 320)

    assert score > 0.85
    assert validity > 0.99


def test_sample_corridor_score_bad_region():
    """A poor corridor is strongly penalized."""
    image = np.full((480, 640), 230, dtype=np.uint8)
    image[220:440, 300:341] = 30

    score, validity = sample_corridor_score(image, 320)

    assert score < 0.50
    assert validity > 0.99


def test_candidate_scoring_prefers_goal_alignment():
    """Goal alignment contributes positively to candidate score."""
    aligned = score_candidate(
        0.8,
        1.0,
        0.0,
        0.0,
        0.0,
    )
    misaligned = score_candidate(
        0.8,
        1.0,
        math.radians(25.0),
        0.0,
        0.0,
    )

    assert aligned > misaligned


def test_candidate_scoring_penalizes_obstacle():
    """Obstacle score reduces the candidate score."""
    clear = score_candidate(
        0.8,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )
    blocked = score_candidate(
        0.8,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    assert clear > blocked


def test_select_candidate_avoids_bad_center():
    """The selector moves away from a strongly degraded center corridor."""
    image = np.full((480, 640), 230, dtype=np.uint8)
    image[215:450, 300:341] = 20

    candidates = [
        math.radians(-25.0),
        math.radians(-12.5),
        0.0,
        math.radians(12.5),
        math.radians(25.0),
    ]

    selected, score, obstacle_score = select_best_candidate(
        image,
        candidates,
        554.25,
        320.0,
        0.0,
        0.0,
    )

    assert selected != 0.0
    assert score > 0.50
    assert math.isclose(obstacle_score, 1.0)


def test_obstacle_clearance_no_obstacle():
    """Ground-only points produce no detected obstacle."""
    x = np.linspace(1.0, 3.0, 20)
    points = np.column_stack([
        x,
        np.zeros_like(x),
        np.full_like(x, -1.12),
    ])

    clearance = obstacle_clearance(points, 0.0)

    assert math.isinf(clearance)


def test_obstacle_clearance_detects_center_obstacle():
    """A raised center obstacle is detected in the forward corridor."""
    ground_x = np.linspace(1.0, 3.0, 80)
    ground_y = np.linspace(-0.25, 0.25, 10)

    gx, gy = np.meshgrid(ground_x, ground_y)
    ground = np.column_stack([
        gx.ravel(),
        gy.ravel(),
        np.full(gx.size, -1.12),
    ])

    obstacle_x = np.linspace(1.8, 2.1, 20)
    obstacle_y = np.linspace(-0.15, 0.15, 10)
    ox, oy = np.meshgrid(obstacle_x, obstacle_y)
    obstacle = np.column_stack([
        ox.ravel(),
        oy.ravel(),
        np.full(ox.size, -0.70),
    ])

    points = np.vstack([ground, obstacle])

    clearance = obstacle_clearance(points, 0.0)

    assert 1.7 < clearance < 2.2


def test_obstacle_clearance_does_not_hit_right_corridor():
    """A center obstacle does not block a sufficiently right corridor."""
    ground_x = np.linspace(1.0, 3.0, 80)
    ground_y = np.linspace(-0.50, 0.50, 15)

    gx, gy = np.meshgrid(ground_x, ground_y)
    ground = np.column_stack([
        gx.ravel(),
        gy.ravel(),
        np.full(gx.size, -1.12),
    ])

    obstacle_x = np.linspace(1.8, 2.1, 20)
    obstacle_y = np.linspace(-0.10, 0.10, 10)
    ox, oy = np.meshgrid(obstacle_x, obstacle_y)
    obstacle = np.column_stack([
        ox.ravel(),
        oy.ravel(),
        np.full(ox.size, -0.70),
    ])

    points = np.vstack([ground, obstacle])

    clearance = obstacle_clearance(
        points,
        math.radians(-25.0),
    )

    assert math.isinf(clearance)


def test_obstacle_clearance_score():
    """Clearance converts monotonically into a normalized score."""
    assert obstacle_clearance_score(1.0) == 0.0
    assert obstacle_clearance_score(2.5) == 1.0
    assert 0.0 < obstacle_clearance_score(1.875) < 1.0


def test_select_candidate_avoids_depth_obstacle():
    """Depth obstacle drives selection away from the blocked center."""
    image = np.full((480, 640), 230, dtype=np.uint8)

    ground_x = np.linspace(1.0, 3.0, 80)
    ground_y = np.linspace(-0.50, 0.50, 15)

    gx, gy = np.meshgrid(ground_x, ground_y)
    ground = np.column_stack([
        gx.ravel(),
        gy.ravel(),
        np.full(gx.size, -1.12),
    ])

    obstacle_x = np.linspace(1.8, 2.1, 20)
    obstacle_y = np.linspace(-0.10, 0.10, 10)
    ox, oy = np.meshgrid(obstacle_x, obstacle_y)
    obstacle = np.column_stack([
        ox.ravel(),
        oy.ravel(),
        np.full(ox.size, -0.70),
    ])

    points = np.vstack([ground, obstacle])

    candidates = [
        math.radians(-25.0),
        math.radians(-12.5),
        0.0,
        math.radians(12.5),
        math.radians(25.0),
    ]

    selected, score, obstacle_score = select_best_candidate(
        image,
        candidates,
        554.25,
        320.0,
        0.0,
        0.0,
        points,
    )

    assert selected != 0.0
    assert score > 0.0
    assert obstacle_score > 0.0


def test_build_local_path():
    """Local path starts at the pose and extends forward."""
    path = build_local_path(
        1.0,
        2.0,
        0.0,
        0.0,
        lookahead=3.0,
        samples=4,
    )

    assert len(path.poses) == 4
    assert math.isclose(path.poses[0].pose.position.x, 1.0)
    assert math.isclose(path.poses[0].pose.position.y, 2.0)
    assert math.isclose(path.poses[-1].pose.position.x, 4.0)
    assert math.isclose(path.poses[-1].pose.position.y, 2.0)


def test_build_left_path():
    """Positive steering angle generates positive body-frame y."""
    path = build_local_path(
        0.0,
        0.0,
        0.0,
        math.radians(20.0),
        lookahead=2.0,
        samples=3,
    )

    assert path.poses[-1].pose.position.y > 0.0
