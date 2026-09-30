"""Tests for spatial local path selection."""

import math
import struct

import numpy as np
from sensor_msgs.msg import PointCloud2

from sih26126_planner.local_path_selector import (
    angle_to_pixel,
    build_local_path,
    goal_angle_from_pose,
    input_is_fresh,
    obstacle_clearance,
    obstacle_clearance_score,
    obstacle_return_evidence,
    pointcloud_to_array,
    sample_corridor_score,
    score_candidate,
    select_best_candidate,
    waypoint_command,
    wrap_angle,
)


def test_wrap_angle():
    """Angles are wrapped to the expected range."""
    assert math.isclose(wrap_angle(0.0), 0.0)
    assert math.isclose(wrap_angle(3.5), 3.5 - 2.0 * math.pi)
    assert math.isclose(wrap_angle(math.pi), math.pi)
    assert math.isclose(wrap_angle(-math.pi), math.pi)


def test_dynamic_inputs_must_be_recent():
    """Expired or unavailable depth/pose inputs must stop local motion."""
    assert input_is_fresh(9.5, 0.75, now=10.0)
    assert not input_is_fresh(9.0, 0.75, now=10.0)
    assert not input_is_fresh(None, 0.75, now=10.0)
    assert not input_is_fresh(10.1, 0.75, now=10.0)


def test_local_planner_depth_stride_preserves_organized_geometry():
    """The local scorer can downsample dense organized sensor frames safely."""
    msg = PointCloud2()
    msg.height = 4
    msg.width = 4
    msg.point_step = 12
    msg.row_step = 48
    msg.data = b''.join(
        struct.pack('<fff', float(row * 10 + col), float(col), float(row))
        for row in range(4) for col in range(4)
    )

    points = pointcloud_to_array(msg, stride=2)

    assert points.shape == (4, 3)
    np.testing.assert_array_equal(
        points[:, 0], np.array([0.0, 2.0, 20.0, 22.0], dtype=np.float32))


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


def test_visible_edge_candidate_steers_around_centered_depth_obstacle():
    """A clear path near the camera FOV edge remains reachable to the selector."""
    image = np.full((480, 640), 220, dtype=np.uint8)
    points = [
        (x, y, 0.0)
        for x in np.linspace(0.3, 3.0, 32)
        for y in np.linspace(-1.8, 1.8, 37)
    ]
    points.append((0.75, 0.0, 0.5))
    points = np.asarray(points, dtype=np.float32)
    edge_angle = math.radians(28.0)

    selected, _, obstacle_score = select_best_candidate(
        image,
        [math.radians(-28.0), math.radians(-25.0), 0.0,
         math.radians(25.0), edge_angle],
        554.25,
        320.0,
        edge_angle,
        0.0,
        points,
    )

    assert math.isclose(selected, edge_angle)
    assert obstacle_score > 0.01


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


def test_obstacle_clearance_detects_close_obstacle():
    """Detect an obstacle inside the previous 1.0 m blind zone."""
    ground_x = np.linspace(0.2, 3.0, 160)
    ground_y = np.zeros_like(ground_x)
    ground_z = np.zeros_like(ground_x)

    obstacle_x = np.full(20, 0.70)
    obstacle_y = np.zeros_like(obstacle_x)
    obstacle_z = np.full(20, 0.70)

    points = np.column_stack(
        (
            np.concatenate((ground_x, obstacle_x)),
            np.concatenate((ground_y, obstacle_y)),
            np.concatenate((ground_z, obstacle_z)),
        )
    )

    clearance = obstacle_clearance(points, 0.0)

    assert math.isfinite(clearance)
    assert clearance < 1.0
    assert math.isclose(clearance, 0.70, abs_tol=0.10)


def test_goal_behind_robot_selects_maximum_reachable_turn():
    """A bearing outside the steering envelope requests its nearest edge."""
    image = np.full((120, 160), 200, dtype=np.uint8)
    angles = [math.radians(deg) for deg in (-25, -16.7, -8.3, 0, 8.3, 16.7, 25)]
    selected, _, _ = select_best_candidate(
        image, angles, 100.0, 80.0, math.radians(-90.0), 0.0,
        np.empty((0, 3), dtype=np.float32))
    assert math.isclose(selected, math.radians(-25.0), abs_tol=1e-9)


def test_obstacle_clearance_score():
    """Clearance converts monotonically into a normalized score."""
    assert obstacle_clearance_score(1.0) == 0.0
    assert obstacle_clearance_score(3.0) == 1.0
    assert obstacle_clearance_score(2.5) < 1.0
    assert 0.0 < obstacle_clearance_score(1.875) < 1.0


def test_candidate_score_prefers_clear_path_from_runtime_case():
    """A clear candidate outranks the finite-clearance runtime winner."""
    terrain_score = 0.5267
    goal_angle = math.radians(-34.1)
    previous_angle = math.radians(-25.0)

    finite_clearance = score_candidate(
        terrain_score,
        1.0,
        math.radians(-25.0),
        goal_angle,
        previous_angle,
        obstacle_clearance_score(2.2191),
    )
    clear_corridor = score_candidate(
        terrain_score,
        1.0,
        math.radians(25.0),
        goal_angle,
        previous_angle,
        obstacle_clearance_score(math.inf),
    )

    assert clear_corridor > finite_clearance


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


def test_waypoint_outside_local_steering_range_rotates_without_translation():
    command = waypoint_command(
        math.radians(90), math.radians(25), 0.2, 1.0)
    assert command.linear.x == 0.0
    assert command.angular.z > 0.0


def test_obstacle_evidence_comes_from_depth_returns():
    ground_x = np.linspace(0.5, 2.5, 80)
    ground_y = np.linspace(-0.3, 0.3, 10)
    gx, gy = np.meshgrid(ground_x, ground_y)
    ground = np.column_stack((gx.ravel(), gy.ravel(), np.full(gx.size, -1.12)))
    ox, oy = np.meshgrid(np.linspace(1.0, 1.2, 12), np.linspace(-0.1, 0.1, 10))
    obstacle = np.column_stack((ox.ravel(), oy.ravel(), np.full(ox.size, -0.7)))
    evidence = obstacle_return_evidence(np.vstack((ground, obstacle)), 0.0)
    assert evidence['valid_points'] > 0
    assert evidence['obstacle_points'] > 0
    assert evidence['raw_clearance_m'] is not None
    assert evidence['selected_corridor_clearance_m'] is not None


def test_clear_depth_corridor_has_null_instead_of_infinite_clearance():
    x = np.linspace(0.5, 2.5, 80)
    y = np.linspace(-0.3, 0.3, 10)
    xx, yy = np.meshgrid(x, y)
    ground = np.column_stack((xx.ravel(), yy.ravel(),
                              np.full(xx.size, -1.12)))
    evidence = obstacle_return_evidence(ground, 0.0)
    assert evidence['obstacle_points'] == 0
    assert evidence['raw_clearance_m'] is None
    assert evidence['selected_corridor_clearance_m'] is None


def test_clear_obstacle_corridor_allows_goal_directed_translation():
    command = waypoint_command(0.1, 0.05, 0.2, 1.0)
    assert command.linear.x > 0.0
    assert math.isclose(command.angular.z, 0.09)


def test_blocked_local_corridor_stops_recommended_command():
    command = waypoint_command(0.0, 0.0, 0.2, 0.0)
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0
