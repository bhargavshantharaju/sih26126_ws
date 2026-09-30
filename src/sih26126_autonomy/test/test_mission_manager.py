"""Tests for mission evidence serialization."""

import json
import math

from geometry_msgs.msg import Twist

from sih26126_autonomy.mission_manager import (
    exploration_completion_ready,
    final_alignment_ready,
    live_metrics_summary,
    pose_jump_exceeded,
    result_json,
    should_enter_return_home,
    zero_velocity_confirmed,
)


def test_result_serialization_has_version_and_preserves_unknowns():
    """Unknown measurements remain JSON null and the schema is versioned."""
    result = json.loads(result_json({
        'mission_result': 'FAILED',
        'collision_detected': None,
        'minimum_clearance_m': None,
    }))
    assert result['schema_version'] == 1
    assert result['mission_result'] == 'FAILED'
    assert result['collision_detected'] is None
    assert result['minimum_clearance_m'] is None


def test_result_serialization_converts_non_finite_evidence_to_null():
    result = json.loads(result_json({
        'obstacle_evidence': [{'clearance_m': math.inf}],
        'raw_distance': math.nan,
    }))
    assert result['obstacle_evidence'][0]['clearance_m'] is None
    assert result['raw_distance'] is None


def test_live_metrics_keep_dashboard_scalars_without_growing_histories():
    summary = live_metrics_summary({
        'mission_result': 'EXPLORING',
        'distance_travelled_m': 3.5,
        'frontiers_completed': 2,
        'trajectory': [{'x': 1.0}] * 1000,
        'obstacle_evidence': [{'clearance': 1.0}] * 1000,
        'global_path': [{'x': 2.0}] * 1000,
    })
    assert summary['mission_result'] == 'EXPLORING'
    assert summary['distance_travelled_m'] == 3.5
    assert summary['frontiers_completed'] == 2
    assert 'trajectory' not in summary
    assert 'obstacle_evidence' not in summary
    assert 'global_path' not in summary


def test_pose_discontinuity_is_detected_and_valid_step_is_accepted():
    assert pose_jump_exceeded(0.74, 0.01, max_speed_mps=2.0, margin_m=0.1)
    assert not pose_jump_exceeded(0.04, 0.05, max_speed_mps=2.0, margin_m=0.1)


def test_exploration_completion_requires_area_goals_and_no_frontiers():
    assert not exploration_completion_ready(30.0, 24.0, 3, 3, False)
    assert not exploration_completion_ready(30.0, 24.0, 2, 3, True)
    assert not exploration_completion_ready(20.0, 24.0, 3, 3, True)
    assert exploration_completion_ready(30.0, 24.0, 3, 3, True)


def test_return_home_transition_only_follows_satisfied_exploration():
    assert should_enter_return_home('EXPLORING', True)
    assert should_enter_return_home('REPLANNING', True)
    assert not should_enter_return_home('EXPLORING', False)
    assert not should_enter_return_home('MAPPING', True)


def test_final_alignment_requires_home_heading_stationarity_and_zero_command():
    assert final_alignment_ready(True, 0.2, 0.35, 0.05, 0.2, True, True)
    assert not final_alignment_ready(False, 0.2, 0.35, 0.05, 0.2, True, True)
    assert not final_alignment_ready(True, 0.5, 0.35, 0.05, 0.2, True, True)
    assert not final_alignment_ready(True, 0.2, 0.35, 0.5, 0.2, True, True)
    assert not final_alignment_ready(True, 0.2, 0.35, 0.05, 0.2, False, True)
    assert not final_alignment_ready(True, 0.2, 0.35, 0.05, 0.2, True, False)


def test_mission_complete_zero_velocity_is_verified_from_command():
    command = Twist()
    assert zero_velocity_confirmed(command)
    command.angular.z = 0.02
    assert not zero_velocity_confirmed(command)
