"""Tests for the recovery command path integrated into safety_supervisor."""

import ast
import math
from pathlib import Path
import time
from unittest.mock import MagicMock, patch

from geometry_msgs.msg import Twist

from sih26126_safety.safety_supervisor import (
    recovery_safety_decision,
    safety_decision,
    SafetySupervisor,
    sanitize_command,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def valid_planner_context():
    """Return a normal fresh planner safety context."""
    return {
        'navigation_state': 'LOCAL_NAVIGATE',
        'obstacle_score': 1.0,
        'command_age': 0.1,
        'state_age': 0.1,
        'obstacle_age': 0.1,
        'localization_age': 0.1,
        'localization_confidence': 1.0,
    }


def valid_recovery_context():
    """Return a normal fresh recovery safety context."""
    return {
        'recovery_suggestion_age': 0.1,
        'obstacle_score': 1.0,
        'obstacle_age': 0.1,
        'localization_age': 0.1,
        'localization_confidence': 1.0,
    }


def reverse_cmd(speed: float = 0.10) -> Twist:
    """Return a typical reverse recovery command."""
    cmd = Twist()
    cmd.linear.x = -speed
    return cmd


def rotate_cmd(rate: float = 0.3) -> Twist:
    """Return a typical rotate recovery command."""
    cmd = Twist()
    cmd.angular.z = rate
    return cmd


def _make_supervisor() -> SafetySupervisor:
    """
    Create a SafetySupervisor with mocked ROS infrastructure.

    All subscribers, publishers, timer, and logger are mocked so that
    process() can be driven directly without a live ROS environment.
    """
    with patch('rclpy.node.Node.__init__', return_value=None), \
         patch('rclpy.node.Node.create_subscription', return_value=MagicMock()), \
         patch('rclpy.node.Node.create_publisher', return_value=MagicMock()), \
         patch('rclpy.node.Node.create_timer', return_value=MagicMock()), \
         patch('rclpy.node.Node.get_logger', return_value=MagicMock()):
        sup = SafetySupervisor.__new__(SafetySupervisor)

        sup.command = None
        sup.navigation_state = None
        sup.obstacle_score = None
        sup.localization_confidence = None
        sup.command_time = None
        sup.state_time = None
        sup.obstacle_time = None
        sup.localization_pose_time = None
        sup.recovery_suggestion = None
        sup.recovery_state = None
        sup.recovery_suggestion_time = None
        sup.recovery_state_time = None

        sup.cmd_pub = MagicMock()
        sup.state_pub = MagicMock()
        sup.reason_pub = MagicMock()

        _params = {
            'timeout_sec': 0.75,
            'obstacle_stop_threshold': 0.01,
            'max_linear_speed': 0.30,
            'max_angular_speed': 0.60,
            'publish_hz': 10.0,
            'localization_confidence_threshold': 0.2,
        }

        def _get_parameter(name):
            m = MagicMock()
            m.value = _params[name]
            return m

        sup.get_parameter = _get_parameter
        sup.get_logger = MagicMock(return_value=MagicMock())

    return sup


def _inject_healthy_planner(sup: SafetySupervisor) -> None:
    """Inject a healthy planner context into the supervisor."""
    now = time.monotonic()
    cmd = Twist()
    cmd.linear.x = 0.2
    sup.command = cmd
    sup.command_time = now
    sup.navigation_state = 'LOCAL_NAVIGATE'
    sup.state_time = now
    sup.obstacle_score = 1.0
    sup.obstacle_time = now
    sup.localization_confidence = 1.0
    sup.localization_pose_time = now


def _inject_healthy_recovery(sup: SafetySupervisor, linear: float = -0.10) -> None:
    """Inject a healthy RECOVERY_ACTIVE context into the supervisor."""
    now = time.monotonic()
    cmd = Twist()
    cmd.linear.x = linear
    sup.recovery_suggestion = cmd
    sup.recovery_suggestion_time = now
    sup.recovery_state = 'RECOVERY_ACTIVE'
    sup.recovery_state_time = now
    sup.obstacle_score = 1.0
    sup.obstacle_time = now
    sup.localization_confidence = 1.0
    sup.localization_pose_time = now


# ---------------------------------------------------------------------------
# Normal planner path is completely unaffected
# ---------------------------------------------------------------------------

def test_normal_planner_command_passes():
    """Planner command passes when navigation is active and all data is fresh."""
    cmd = Twist()
    cmd.linear.x = 0.2
    cmd.angular.z = 0.1
    result, reason = safety_decision(command=cmd, **valid_planner_context())
    assert reason == 'SAFE_COMMAND'
    assert math.isclose(result.linear.x, 0.2)


# ---------------------------------------------------------------------------
# recovery_safety_decision() function-level tests
# ---------------------------------------------------------------------------

def test_recovery_decision_returns_recovery_command_when_valid():
    """A fresh valid recovery suggestion passes all safety checks."""
    cmd = reverse_cmd()
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd,
        **valid_recovery_context(),
    )
    assert reason == 'RECOVERY_COMMAND'
    assert math.isclose(result.linear.x, -0.10)


def test_stale_recovery_suggestion_is_rejected():
    """A stale recovery suggestion is not forwarded to /cmd_vel."""
    cmd = reverse_cmd()
    ctx = valid_recovery_context()
    ctx['recovery_suggestion_age'] = 2.0  # older than 0.75 s timeout
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd, **ctx
    )
    assert reason == 'STALE_RECOVERY_SUGGESTION'
    assert math.isclose(result.linear.x, 0.0)
    assert math.isclose(result.angular.z, 0.0)


def test_nonfinite_recovery_command_is_rejected():
    """A non-finite velocity in a recovery suggestion is rejected."""
    cmd = Twist()
    cmd.linear.x = float('nan')
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd,
        **valid_recovery_context(),
    )
    assert reason == 'INVALID_RECOVERY_COMMAND'
    assert math.isclose(result.linear.x, 0.0)


def test_recovery_respects_obstacle_stop():
    """Recovery is blocked by a critical obstacle score."""
    cmd = reverse_cmd()
    ctx = valid_recovery_context()
    ctx['obstacle_score'] = 0.0
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd, **ctx
    )
    assert reason == 'OBSTACLE_STOP'
    assert math.isclose(result.linear.x, 0.0)


def test_recovery_respects_stale_obstacle_score():
    """Recovery is blocked by a stale obstacle score."""
    cmd = reverse_cmd()
    ctx = valid_recovery_context()
    ctx['obstacle_age'] = 2.0
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd, **ctx
    )
    assert reason == 'STALE_OBSTACLE_SCORE'
    assert math.isclose(result.linear.x, 0.0)


def test_recovery_respects_low_localization_confidence():
    """Recovery is blocked when localization confidence is too low."""
    cmd = reverse_cmd()
    ctx = valid_recovery_context()
    ctx['localization_confidence'] = 0.05
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd, **ctx
    )
    assert reason == 'LOW_LOCALIZATION_CONFIDENCE'
    assert math.isclose(result.linear.x, 0.0)


def test_recovery_respects_stale_localization():
    """Recovery is blocked when localization is stale."""
    cmd = reverse_cmd()
    ctx = valid_recovery_context()
    ctx['localization_age'] = 2.0
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd, **ctx
    )
    assert reason == 'STALE_LOCALIZATION'
    assert math.isclose(result.linear.x, 0.0)


def test_recovery_command_is_velocity_clamped():
    """Recovery command velocities are clamped to safety limits."""
    cmd = Twist()
    cmd.linear.x = -5.0
    cmd.angular.z = 5.0
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd,
        **valid_recovery_context(),
    )
    assert reason == 'RECOVERY_COMMAND'
    assert result.linear.x >= -0.30
    assert result.angular.z <= 0.60


def test_rotate_recovery_suggestion_passes():
    """A rotate suggestion (zero linear, nonzero angular) passes safety checks."""
    cmd = rotate_cmd(0.3)
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd,
        **valid_recovery_context(),
    )
    assert reason == 'RECOVERY_COMMAND'
    assert math.isclose(result.linear.x, 0.0)
    assert math.isclose(result.angular.z, 0.3)


def test_zero_recovery_suggestion_passes():
    """A zero-velocity recovery suggestion (stop phase) is valid."""
    cmd = Twist()
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd,
        **valid_recovery_context(),
    )
    assert reason == 'RECOVERY_COMMAND'
    assert math.isclose(result.linear.x, 0.0)
    assert math.isclose(result.angular.z, 0.0)


def test_recovery_decision_omits_navigation_state_check():
    """
    Recovery succeeds even when planner navigation state is not NAVIGATE.

    The vehicle may be stuck precisely because the planner cannot issue
    LOCAL_NAVIGATE, so recovery must not be gated on planner state.
    """
    cmd = reverse_cmd()
    result, reason = recovery_safety_decision(
        recovery_suggestion=cmd,
        **valid_recovery_context(),
    )
    assert reason == 'RECOVERY_COMMAND'
    planner_ctx = valid_planner_context()
    planner_ctx['navigation_state'] = 'WAITING_FOR_DATA'
    _, planner_reason = safety_decision(command=cmd, **planner_ctx)
    assert planner_reason == 'NAVIGATION_NOT_ACTIVE'


# ---------------------------------------------------------------------------
# Architecture constraint: exactly one /cmd_vel publisher
# ---------------------------------------------------------------------------

def test_exactly_one_cmd_vel_publisher():
    """safety_supervisor must be the only /cmd_vel publisher across all source files."""
    source_root = Path(__file__).resolve().parents[3] / 'src'
    publishers = []
    for source in source_root.rglob('*.py'):
        tree = ast.parse(source.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not (isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'create_publisher'
                    and len(node.args) >= 2
                    and isinstance(node.args[1], ast.Constant)
                    and node.args[1].value == '/cmd_vel'):
                continue
            publishers.append(source.resolve())
    assert publishers == [
        (source_root / 'sih26126_safety/sih26126_safety/safety_supervisor.py').resolve()
    ]


def test_recovery_safety_decision_is_exported():
    """recovery_safety_decision is importable from safety_supervisor module."""
    from sih26126_safety.safety_supervisor import recovery_safety_decision as rsd
    assert callable(rsd)


def test_sanitize_still_works_after_refactor():
    """sanitize_command still zeroes lateral components after the refactor."""
    cmd = Twist()
    cmd.linear.x = 0.2
    cmd.linear.y = 0.5
    cmd.angular.z = 0.1
    result, valid = sanitize_command(cmd)
    assert valid
    assert math.isclose(result.linear.y, 0.0)
    assert math.isclose(result.linear.x, 0.2)


# ---------------------------------------------------------------------------
# SafetySupervisor node-level process() routing tests
#
# These tests exercise the full process() dispatch path through the
# _is_recovery_active() gate, verifying:
#   - recovery suggestion is ignored when state is not RECOVERY_ACTIVE
#   - recovery path takes over only when state IS RECOVERY_ACTIVE
#   - every safety check remains enforced during recovery
# ---------------------------------------------------------------------------

class TestSafetySupervisorProcessRouting:
    """Node-level process() routing tests for the SafetySupervisor."""

    def test_normal_planner_passes_through_process(self):
        """process() publishes the planner command in normal non-recovery state."""
        sup = _make_supervisor()
        _inject_healthy_planner(sup)
        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.2, abs_tol=1e-6)

    def test_recovery_suggestion_ignored_when_state_is_none(self):
        """
        Recovery suggestion is NOT used when recovery_state is None.

        The FSM has not entered RECOVERY_ACTIVE so process() falls through
        to the planner path even if a suggestion is queued on the topic.
        """
        sup = _make_supervisor()
        _inject_healthy_planner(sup)
        now = time.monotonic()
        rec_cmd = Twist()
        rec_cmd.linear.x = -0.10
        sup.recovery_suggestion = rec_cmd
        sup.recovery_suggestion_time = now
        sup.recovery_state = None

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.2, abs_tol=1e-6)

    def test_recovery_suggestion_ignored_when_state_is_normal(self):
        """
        Recovery suggestion is NOT used when recovery_state is NORMAL.

        NORMAL is a valid non-recovery FSM state; the suggestion must be
        ignored and the planner command forwarded instead.
        """
        sup = _make_supervisor()
        _inject_healthy_planner(sup)
        now = time.monotonic()
        rec_cmd = Twist()
        rec_cmd.linear.x = -0.10
        sup.recovery_suggestion = rec_cmd
        sup.recovery_suggestion_time = now
        sup.recovery_state = 'NORMAL'
        sup.recovery_state_time = now

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.2, abs_tol=1e-6)

    def test_recovery_suggestion_ignored_when_state_is_stuck_suspected(self):
        """Recovery suggestion must not be used during STUCK_SUSPECTED."""
        sup = _make_supervisor()
        _inject_healthy_planner(sup)
        now = time.monotonic()
        rec_cmd = Twist()
        rec_cmd.linear.x = -0.10
        sup.recovery_suggestion = rec_cmd
        sup.recovery_suggestion_time = now
        sup.recovery_state = 'STUCK_SUSPECTED'
        sup.recovery_state_time = now

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.2, abs_tol=1e-6)

    def test_valid_recovery_command_accepted_when_recovery_active(self):
        """
        process() publishes the recovery suggestion when RECOVERY_ACTIVE and safe.

        All safety checks must pass for the recovery command to reach /cmd_vel.
        """
        sup = _make_supervisor()
        _inject_healthy_recovery(sup, linear=-0.10)

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, -0.10, abs_tol=1e-6)

    def test_rotate_recovery_command_accepted(self):
        """process() publishes a rotate recovery suggestion in RECOVERY_ACTIVE."""
        sup = _make_supervisor()
        _inject_healthy_recovery(sup, linear=0.0)
        sup.recovery_suggestion.angular.z = 0.3

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.0, abs_tol=1e-6)
        assert math.isclose(published.angular.z, 0.3, abs_tol=1e-6)

    def test_stale_recovery_state_treated_as_inactive(self):
        """
        A stale recovery_state_time causes _is_recovery_active() to return False.

        When the recovery state message is over the timeout threshold the node
        falls back to the planner path rather than acting on stale FSM data.
        """
        sup = _make_supervisor()
        _inject_healthy_planner(sup)
        now = time.monotonic()
        rec_cmd = Twist()
        rec_cmd.linear.x = -0.10
        sup.recovery_suggestion = rec_cmd
        sup.recovery_suggestion_time = now
        sup.recovery_state = 'RECOVERY_ACTIVE'
        sup.recovery_state_time = now - 2.0   # 2 s old — stale

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.2, abs_tol=1e-6)

    def test_stale_recovery_suggestion_publishes_zero_in_recovery_active(self):
        """
        Stale suggestion yields zero output even with recovery_state=RECOVERY_ACTIVE.

        An active FSM state does not override the staleness check on the
        suggestion itself — stale data must never actuate the vehicle.
        """
        sup = _make_supervisor()
        _inject_healthy_recovery(sup)
        sup.recovery_suggestion_time = time.monotonic() - 2.0  # stale

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.0, abs_tol=1e-6)
        assert math.isclose(published.angular.z, 0.0, abs_tol=1e-6)

    def test_nonfinite_recovery_command_publishes_zero(self):
        """Non-finite recovery command in RECOVERY_ACTIVE results in zero output."""
        sup = _make_supervisor()
        _inject_healthy_recovery(sup)
        sup.recovery_suggestion.linear.x = float('nan')

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.0, abs_tol=1e-6)

    def test_obstacle_stop_overrides_recovery_command(self):
        """
        Obstacle score at or below threshold blocks recovery actuation.

        Safety checks are NOT bypassed during recovery — obstacle stop
        always takes precedence over the recovery suggestion.
        """
        sup = _make_supervisor()
        _inject_healthy_recovery(sup)
        sup.obstacle_score = 0.0

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.0, abs_tol=1e-6)

    def test_low_localization_confidence_overrides_recovery(self):
        """Low localization confidence blocks recovery actuation."""
        sup = _make_supervisor()
        _inject_healthy_recovery(sup)
        sup.localization_confidence = 0.05

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.0, abs_tol=1e-6)

    def test_stale_localization_overrides_recovery(self):
        """Stale localization pose time blocks recovery actuation."""
        sup = _make_supervisor()
        _inject_healthy_recovery(sup)
        sup.localization_pose_time = time.monotonic() - 2.0

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert math.isclose(published.linear.x, 0.0, abs_tol=1e-6)

    def test_recovery_safety_state_published_in_recovery_active(self):
        """process() publishes safety_state='RECOVERY' during RECOVERY_ACTIVE."""
        sup = _make_supervisor()
        _inject_healthy_recovery(sup, linear=-0.10)

        sup.process()
        state_call = sup.state_pub.publish.call_args[0][0]
        assert state_call.data == 'RECOVERY'

    def test_safety_state_stopped_published_when_recovery_blocked(self):
        """process() publishes safety_state='STOPPED' when recovery is blocked."""
        sup = _make_supervisor()
        _inject_healthy_recovery(sup)
        sup.obstacle_score = 0.0

        sup.process()
        state_call = sup.state_pub.publish.call_args[0][0]
        assert state_call.data == 'STOPPED'

    def test_recovery_command_clamped_to_max_linear(self):
        """Recovery command linear speed is clamped to max_linear_speed param."""
        sup = _make_supervisor()
        _inject_healthy_recovery(sup, linear=-5.0)

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert published.linear.x >= -0.30

    def test_recovery_command_clamped_to_max_angular(self):
        """Recovery command angular speed is clamped to max_angular_speed param."""
        sup = _make_supervisor()
        _inject_healthy_recovery(sup, linear=0.0)
        sup.recovery_suggestion.angular.z = 5.0

        sup.process()
        published = sup.cmd_pub.publish.call_args[0][0]
        assert published.angular.z <= 0.60
