"""Tests for the safety supervisor."""

import math

from geometry_msgs.msg import Twist

from sih26126_safety.safety_supervisor import (
    safety_decision,
    sanitize_command,
)


def valid_context():
    """Return a normal fresh safety context."""
    return {
        'navigation_state': 'LOCAL_NAVIGATE',
        'obstacle_score': 1.0,
        'command_age': 0.1,
        'state_age': 0.1,
        'obstacle_age': 0.1,
    }


def test_safe_command_passes():
    """A fresh valid navigation command is allowed."""
    command = Twist()
    command.linear.x = 0.2
    command.angular.z = 0.1

    result, reason = safety_decision(
        command=command,
        **valid_context(),
    )

    assert reason == 'SAFE_COMMAND'
    assert math.isclose(result.linear.x, 0.2)
    assert math.isclose(result.angular.z, 0.1)


def test_command_is_clamped():
    """Command velocity is clamped to the safety limits."""
    command = Twist()
    command.linear.x = 2.0
    command.angular.z = 2.0

    result, reason = safety_decision(
        command=command,
        **valid_context(),
    )

    assert reason == 'SAFE_COMMAND'
    assert math.isclose(result.linear.x, 0.30)
    assert math.isclose(result.angular.z, 0.60)


def test_stale_command_stops():
    """A stale planner command causes a stop."""
    command = Twist()
    command.linear.x = 0.2

    context = valid_context()
    context['command_age'] = 1.0

    result, reason = safety_decision(
        command=command,
        **context,
    )

    assert reason == 'STALE_COMMAND'
    assert math.isclose(result.linear.x, 0.0)
    assert math.isclose(result.angular.z, 0.0)


def test_stale_state_stops():
    """A stale navigation state causes a stop."""
    command = Twist()
    command.linear.x = 0.2

    context = valid_context()
    context['state_age'] = 1.0

    result, reason = safety_decision(
        command=command,
        **context,
    )

    assert reason == 'STALE_NAVIGATION_STATE'
    assert math.isclose(result.linear.x, 0.0)


def test_stale_obstacle_score_stops():
    """A stale obstacle score causes a stop."""
    command = Twist()
    command.linear.x = 0.2

    context = valid_context()
    context['obstacle_age'] = 1.0

    result, reason = safety_decision(
        command=command,
        **context,
    )

    assert reason == 'STALE_OBSTACLE_SCORE'
    assert math.isclose(result.linear.x, 0.0)


def test_obstacle_stop_state_stops():
    """Planner obstacle-stop state produces zero motion."""
    command = Twist()
    command.linear.x = 0.2

    context = valid_context()
    context['navigation_state'] = 'LOCAL_STOP_OBSTACLE'

    result, reason = safety_decision(
        command=command,
        **context,
    )

    assert reason == 'OBSTACLE_STOP'
    assert math.isclose(result.linear.x, 0.0)


def test_low_obstacle_score_stops():
    """Critical obstacle score produces zero motion."""
    command = Twist()
    command.linear.x = 0.2

    context = valid_context()
    context['obstacle_score'] = 0.0

    result, reason = safety_decision(
        command=command,
        **context,
    )

    assert reason == 'OBSTACLE_STOP'
    assert math.isclose(result.linear.x, 0.0)


def test_wrong_navigation_state_stops():
    """Non-navigation states cannot actuate the vehicle."""
    command = Twist()
    command.linear.x = 0.2

    context = valid_context()
    context['navigation_state'] = 'WAITING_FOR_DATA'

    result, reason = safety_decision(
        command=command,
        **context,
    )

    assert reason == 'NAVIGATION_NOT_ACTIVE'
    assert math.isclose(result.linear.x, 0.0)


def test_nonfinite_command_stops():
    """Nan or infinity in a command produces a stop."""
    command = Twist()
    command.linear.x = float('nan')

    result, reason = safety_decision(
        command=command,
        **valid_context(),
    )

    assert reason == 'INVALID_COMMAND'
    assert math.isclose(result.linear.x, 0.0)


def test_sanitize_removes_lateral_components():
    """Safety output only permits planar forward/turn motion."""
    command = Twist()
    command.linear.x = 0.2
    command.linear.y = 0.5
    command.linear.z = 0.3
    command.angular.x = 0.2
    command.angular.y = 0.4
    command.angular.z = 0.1

    result, valid = sanitize_command(command)

    assert valid
    assert math.isclose(result.linear.x, 0.2)
    assert math.isclose(result.linear.y, 0.0)
    assert math.isclose(result.linear.z, 0.0)
    assert math.isclose(result.angular.x, 0.0)
    assert math.isclose(result.angular.y, 0.0)
    assert math.isclose(result.angular.z, 0.1)
