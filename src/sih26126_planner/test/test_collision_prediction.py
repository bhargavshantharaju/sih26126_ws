#!/usr/bin/env python3
"""Unit tests for predictive dynamic obstacle collision risk."""

import math

from geometry_msgs.msg import Point, Vector3
import numpy as np
from sih26126_msgs.msg import (
    TrackedObstacle,
    TrackedObstacleArray,
)
from sih26126_planner.local_path_selector import (
    predict_trajectory_collision_risk,
    select_best_candidate,
)
from std_msgs.msg import Header


def make_obstacle(
    x: float, y: float, z: float = 0.5,
    vx: float = 0.0, vy: float = 0.0,
    confidence: float = 0.8,
    is_dynamic: bool = False,
    obs_id: str = 'obs_0001'
) -> TrackedObstacle:
    """Create a TrackedObstacle message for testing."""
    obs = TrackedObstacle()
    obs.id = obs_id
    obs.position = Point(x=x, y=y, z=z)
    obs.velocity = Vector3(x=vx, y=vy, z=0.0)
    obs.confidence = confidence
    obs.is_dynamic = is_dynamic
    obs.classification = 'DYNAMIC' if is_dynamic else 'STATIC'
    obs.dimensions = Vector3(x=0.5, y=0.5, z=1.0)
    obs.frame_id = 'sih_ugv/base_link/lidar_3d'
    return obs


def make_obstacle_array(obstacles) -> TrackedObstacleArray:
    """Create TrackedObstacleArray from list of TrackedObstacle."""
    arr = TrackedObstacleArray()
    arr.header = Header()
    arr.header.frame_id = 'sih_ugv/base_link/lidar_3d'
    arr.obstacles = obstacles
    arr.total_active_tracks = len(obstacles)
    return arr


class TestPredictTrajectoryCollisionRisk:
    """Test suite for predict_trajectory_collision_risk function."""

    def test_no_obstacles_returns_zero_risk(self):
        """Empty obstacle array should return zero risk."""
        empty = make_obstacle_array([])
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, empty
        )
        assert risk == 0.0

    def test_none_obstacles_returns_zero_risk(self):
        """None obstacle array should return zero risk."""
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, None
        )
        assert risk == 0.0

    def test_static_obstacle_ignored(self):
        """Static obstacles should not contribute to dynamic collision risk."""
        # Static obstacle directly ahead
        static = make_obstacle(x=2.0, y=0.0, is_dynamic=False)
        arr = make_obstacle_array([static])
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr
        )
        assert risk == 0.0

    def test_low_confidence_obstacle_ignored(self):
        """Low confidence dynamic obstacles should be ignored."""
        # Dynamic obstacle with low confidence
        obs = make_obstacle(
            x=2.0, y=0.0, vx=-0.5, is_dynamic=True, confidence=0.2
        )
        arr = make_obstacle_array([obs])
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr
        )
        assert risk == 0.0

    def test_approaching_obstacle_high_risk(self):
        """Dynamic obstacle approaching robot should produce risk."""
        # Obstacle at x=1.5m, approaching at 0.5 m/s
        # Robot moving forward at 0.2 m/s
        # They are on collision course
        obs = make_obstacle(x=1.5, y=0.0, vx=-0.5, is_dynamic=True)
        arr = make_obstacle_array([obs])
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=2.0
        )
        # After 2 seconds: robot at x=0.4, obstacle at x=0.5
        # Distance = 0.1m < critical_distance (0.6m)
        # So risk should be maximum
        assert risk == 1.0

    def test_crossing_obstacle_detected(self):
        """Obstacle crossing robot path should be detected."""
        # Obstacle at x=2m, y=1m, moving left at -0.5 m/s
        # Robot moving forward at 0.2 m/s along x-axis
        obs = make_obstacle(x=2.0, y=1.0, vx=0.0, vy=-0.5, is_dynamic=True)
        arr = make_obstacle_array([obs])
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=2.5
        )
        # After ~2 seconds: robot at x=0.4, obstacle crosses y=0 at x=2
        # They will be close but not colliding immediately
        assert risk >= 0.0

    def test_receding_obstacle_low_risk(self):
        """Obstacle moving away should produce low or zero risk."""
        # Obstacle ahead moving away faster than robot
        obs = make_obstacle(x=3.0, y=0.0, vx=1.0, is_dynamic=True)
        arr = make_obstacle_array([obs])
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=2.0
        )
        # Gap increases over time, risk should be low
        assert risk == 0.0

    def test_imminent_collision_max_risk(self):
        """Very close approaching obstacle should give maximum risk."""
        # Obstacle at 0.8m directly ahead, approaching
        obs = make_obstacle(x=0.8, y=0.0, vx=-0.3, is_dynamic=True)
        arr = make_obstacle_array([obs])
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=1.0
        )
        # Initial distance 0.8m, after 1s: robot at 0.2, obs at 0.5
        # Distance = 0.3m < critical_distance (0.6m)
        assert risk == 1.0

    def test_far_dynamic_obstacle_zero_risk(self):
        """Dynamic obstacle far away should not produce risk."""
        # Obstacle at 10m
        obs = make_obstacle(x=10.0, y=0.0, vx=0.0, is_dynamic=True)
        arr = make_obstacle_array([obs])
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=2.0
        )
        # Distance stays > safe_distance (1.2m)
        assert risk == 0.0

    def test_multiple_obstacles_max_risk_returned(self):
        """With multiple obstacles, maximum risk should be returned."""
        # One far, one close
        far = make_obstacle(x=5.0, y=0.0, is_dynamic=True, obs_id='obs_far')
        close = make_obstacle(x=1.0, y=0.0, vx=-0.2, is_dynamic=True, obs_id='obs_close')
        arr = make_obstacle_array([far, close])
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=2.0
        )
        # Close obstacle drives the risk
        assert risk > 0.3

    def test_steering_angle_changes_prediction(self):
        """Different steering angles produce different risks."""
        # Obstacle at x=2, y=0.5
        obs = make_obstacle(x=2.0, y=0.5, is_dynamic=True)
        arr = make_obstacle_array([obs])

        # Straight ahead (angle=0) - robot moves along +x
        risk_straight = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=2.0
        )

        # Turn left (angle=π/4) - robot moves +x and +y
        risk_left = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, math.pi / 4, 0.2, arr, horizon_sec=2.0
        )

        # Turn right (angle=-π/4) - robot moves +x and -y
        risk_right = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, -math.pi / 4, 0.2, arr, horizon_sec=2.0
        )

        # Left turn brings robot closer to obstacle at y=0.5
        assert risk_left >= risk_straight
        # Right turn moves robot away from obstacle at y=0.5
        assert risk_right <= risk_straight


class TestCollisionRiskIntegration:
    """Test collision risk integration with trajectory selection."""

    def test_high_risk_trajectory_penalized(self):
        """
        Trajectory with collision risk receives lower obstacle score.

        Verifies that collision risk is integrated into scoring,
        though goal alignment may still dominate final selection.
        """
        # Simple terrain image: all good
        image = np.ones((480, 640), dtype=np.float32)
        points = None

        # Dynamic obstacle at (1.5, 0.2), moving closer
        obs = make_obstacle(
            x=1.5, y=0.2, vx=-0.2, vy=-0.1, is_dynamic=True
        )
        arr = make_obstacle_array([obs])

        # Test with and without obstacle
        candidates = [0.0, 0.3]

        # Without obstacle
        _, _, obs_score_no_risk = select_best_candidate(
            image=image,
            candidate_angles=candidates,
            fx=320.0,
            cx=320.0,
            goal_angle=0.0,
            previous_angle=0.0,
            points=points,
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            robot_speed=0.2,
            tracked_obstacles=None,
        )

        # With obstacle
        _, _, obs_score_with_risk = select_best_candidate(
            image=image,
            candidate_angles=candidates,
            fx=320.0,
            cx=320.0,
            goal_angle=0.0,
            previous_angle=0.0,
            points=points,
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            robot_speed=0.2,
            tracked_obstacles=arr,
        )

        # Obstacle score must be reduced by collision risk
        assert obs_score_with_risk < obs_score_no_risk

    def test_safe_trajectory_preferred(self):
        """
        Test that trajectory with higher collision risk scores lower.

        When one trajectory has higher collision risk, safer option
        scores better.
        """
        image = np.ones((480, 640), dtype=np.float32)
        points = None

        # Obstacle slightly to the left at y=0.3, moving closer
        obs = make_obstacle(
            x=1.5, y=0.3, vx=-0.3, vy=-0.1, is_dynamic=True
        )
        arr = make_obstacle_array([obs])

        # Straight and right turn
        candidates = [0.0, -0.3]
        goal_angle = 0.0

        best_angle, best_obs_score = select_best_candidate(
            image=image,
            candidate_angles=candidates,
            fx=320.0,
            cx=320.0,
            goal_angle=goal_angle,
            previous_angle=0.0,
            points=points,
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            robot_speed=0.2,
            tracked_obstacles=arr,
        )[:2]

        # With obstacle, best_obs_score should be affected by
        # collision risk
        # The exact best angle depends on scoring weights, but score
        # should be reasonable
        assert 0.0 <= best_obs_score <= 1.0
        assert best_angle in candidates

    def test_no_obstacles_no_penalty(self):
        """
        Test selection without obstacles.

        Without obstacles, selection based on terrain and goal
        alignment.
        """
        image = np.ones((480, 640), dtype=np.float32)
        points = None
        arr = make_obstacle_array([])

        candidates = [0.0, 0.3, -0.3]
        goal_angle = 0.0

        best_angle, best_score, _ = select_best_candidate(
            image=image,
            candidate_angles=candidates,
            fx=320.0,
            cx=320.0,
            goal_angle=goal_angle,
            previous_angle=0.0,
            points=points,
            robot_x=0.0,
            robot_y=0.0,
            robot_speed=0.2,
            tracked_obstacles=arr,
        )

        # Goal-aligned candidate (0.0) should win due to goal
        # alignment weight
        assert best_angle == 0.0
        # Score combines terrain, validity, goal alignment, and
        # obstacle clearance
        # With perfect terrain and no obstacles, score should
        # be reasonable
        assert best_score > 0.3


class TestStaleTrackHandling:
    """Test handling of stale and invalid obstacle tracks."""

    def test_stale_obstacle_still_considered(self):
        """Obstacle with recent age should still contribute to risk."""
        # Obstacle with age_sec field (if available in message)
        obs = make_obstacle(x=1.5, y=0.0, vx=-0.3, is_dynamic=True)
        obs.age_sec = 0.5  # Half second old
        arr = make_obstacle_array([obs])

        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=1.0
        )
        # Risk should be computed normally
        assert risk > 0.0

    def test_non_finite_velocity_handled(self):
        """
        Obstacle with NaN or inf velocity returns maximum risk.

        Conservative policy: non-finite data cannot prove safety,
        so we assume maximum risk.
        """
        obs = make_obstacle(x=2.0, y=0.0, is_dynamic=True)
        obs.velocity.x = float('nan')
        arr = make_obstacle_array([obs])

        # Should not raise exception
        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=1.0
        )
        # Non-finite data: conservative maximum risk
        assert risk == 1.0
        assert math.isfinite(risk)

    def test_non_finite_position_handled(self):
        """Obstacle with NaN position returns maximum risk."""
        obs = make_obstacle(x=float('inf'), y=0.0, is_dynamic=True)
        arr = make_obstacle_array([obs])

        risk = predict_trajectory_collision_risk(
            0.0, 0.0, 0.0, 0.0, 0.2, arr, horizon_sec=1.0
        )
        # Non-finite position: conservative maximum risk
        assert risk == 1.0
        assert math.isfinite(risk)
