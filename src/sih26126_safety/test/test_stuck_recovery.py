"""Deterministic unit tests for the stuck detection and recovery state machine."""

import math
import time
from unittest.mock import MagicMock, patch


# ---------------------------------------------------------------------------
# Import targets — the enum and node class
# ---------------------------------------------------------------------------
from sih26126_safety.stuck_recovery import (
    RecoveryPhase,
    RecoveryState,
    StuckRecoveryNode,
)


# ---------------------------------------------------------------------------
# Helpers — build a minimally functional StuckRecoveryNode without ROS spin
# ---------------------------------------------------------------------------

def _make_node(
    commanded_speed: float = 0.15,
    measured_speed: float = 0.0,
    safety_state: str = 'LOCAL_NAVIGATE',
    localization_confidence: float = 1.0,
    obstacle_clearance: float = 2.0,
    stuck_command_speed_threshold: float = 0.05,
    stuck_measured_speed_threshold: float = 0.02,
    stuck_timeout_sec: float = 3.0,
    stuck_suspect_timeout_sec: float = 2.0,
    max_recovery_attempts: int = 3,
    recovery_reverse_speed: float = 0.10,
    recovery_reverse_time_sec: float = 2.0,
    recovery_rotate_speed: float = 0.3,
    recovery_rotate_time_sec: float = 2.0,
    recovery_success_distance: float = 0.05,
    recovery_timeout_sec: float = 10.0,
    min_clearance_for_reverse: float = 0.3,
) -> StuckRecoveryNode:
    """Create a StuckRecoveryNode with mocked ROS infrastructure."""
    with patch('rclpy.node.Node.__init__', return_value=None), \
         patch('rclpy.node.Node.create_subscription', return_value=MagicMock()), \
         patch('rclpy.node.Node.create_publisher', return_value=MagicMock()), \
         patch('rclpy.node.Node.create_timer', return_value=MagicMock()), \
         patch('rclpy.node.Node.get_logger', return_value=MagicMock()):
        node = StuckRecoveryNode.__new__(StuckRecoveryNode)

        # Simulate what __init__ does manually (without ROS calls)
        node._fsm_state = RecoveryState.NORMAL
        node._recovery_phase = RecoveryPhase.STOP_INITIAL

        node._measured_speed = measured_speed
        node._commanded_speed = commanded_speed
        node._safety_state = safety_state
        node._localization_confidence = localization_confidence
        node._obstacle_clearance = obstacle_clearance

        node._odom_time = time.monotonic()
        node._cmd_time = time.monotonic()
        node._safety_time = time.monotonic()
        node._confidence_time = time.monotonic()
        node._clearance_time = time.monotonic()

        node._pos_x = 0.0
        node._pos_y = 0.0
        node._recovery_start_pos_x = 0.0
        node._recovery_start_pos_y = 0.0

        node._stuck_condition_start = None
        node._suspect_entry_time = None
        node._phase_start_time = None
        node._recovery_start_time = None
        node._recovery_attempts = 0
        node._prev_state = RecoveryState.NORMAL

        # Mock publishers
        node._state_pub = MagicMock()
        node._cmd_vel_suggestion_pub = MagicMock()
        node._reason_pub = MagicMock()

        # Parameter values dictionary
        _params = {
            'stuck_command_speed_threshold': stuck_command_speed_threshold,
            'stuck_measured_speed_threshold': stuck_measured_speed_threshold,
            'stuck_timeout_sec': stuck_timeout_sec,
            'stuck_suspect_timeout_sec': stuck_suspect_timeout_sec,
            'recovery_reverse_speed': recovery_reverse_speed,
            'recovery_reverse_time_sec': recovery_reverse_time_sec,
            'recovery_rotate_speed': recovery_rotate_speed,
            'recovery_rotate_time_sec': recovery_rotate_time_sec,
            'recovery_success_distance': recovery_success_distance,
            'recovery_timeout_sec': recovery_timeout_sec,
            'max_recovery_attempts': max_recovery_attempts,
            'min_clearance_for_reverse': min_clearance_for_reverse,
        }

        def _get_parameter(name):
            m = MagicMock()
            m.value = _params[name]
            return m

        node.get_parameter = _get_parameter
        node.get_logger = MagicMock(return_value=MagicMock())
        node.get_clock = MagicMock(return_value=MagicMock())

    return node


# ---------------------------------------------------------------------------
# Enum sanity tests
# ---------------------------------------------------------------------------

class TestEnumValues:
    """Verify state and phase enum members exist with expected string values."""

    def test_recovery_states_defined(self):
        """All RecoveryState enum members are defined with correct string values."""
        assert RecoveryState.NORMAL == 'NORMAL'
        assert RecoveryState.STUCK_SUSPECTED == 'STUCK_SUSPECTED'
        assert RecoveryState.RECOVERY_ACTIVE == 'RECOVERY_ACTIVE'
        assert RecoveryState.RECOVERY_SUCCESS == 'RECOVERY_SUCCESS'
        assert RecoveryState.RECOVERY_FAILED == 'RECOVERY_FAILED'
        assert RecoveryState.STOPPED == 'STOPPED'

    def test_recovery_phases_defined(self):
        """All RecoveryPhase enum members are defined with correct string values."""
        assert RecoveryPhase.STOP_INITIAL == 'STOP_INITIAL'
        assert RecoveryPhase.REVERSE == 'REVERSE'
        assert RecoveryPhase.ROTATE == 'ROTATE'
        assert RecoveryPhase.STOP_FINAL == 'STOP_FINAL'
        assert RecoveryPhase.DONE == 'DONE'


# ---------------------------------------------------------------------------
# Guard condition tests
# ---------------------------------------------------------------------------

class TestGuardConditions:
    """Guard conditions must prevent recovery when prerequisites are missing."""

    def test_low_localization_blocks_recovery(self):
        """Low localization confidence must block recovery."""
        node = _make_node(localization_confidence=0.05)
        reason = node._guard_conditions_block_recovery()
        assert reason == 'LOW_LOCALIZATION_CONFIDENCE'

    def test_none_localization_blocks_recovery(self):
        """None localization confidence must block recovery."""
        node = _make_node(localization_confidence=1.0)
        node._localization_confidence = None
        reason = node._guard_conditions_block_recovery()
        assert reason == 'LOW_LOCALIZATION_CONFIDENCE'

    def test_safety_stopped_blocks_recovery(self):
        """Safety STOPPED state must block recovery."""
        node = _make_node(safety_state='STOPPED')
        reason = node._guard_conditions_block_recovery()
        assert 'STOPPED' in reason

    def test_safety_estop_blocks_recovery(self):
        """Safety E_STOP state must block recovery."""
        node = _make_node(safety_state='E_STOP')
        reason = node._guard_conditions_block_recovery()
        assert 'E_STOP' in reason

    def test_no_planner_command_blocks_recovery(self):
        """No received planner command must block recovery."""
        node = _make_node(commanded_speed=0.15)
        node._cmd_time = None  # simulate no command received
        reason = node._guard_conditions_block_recovery()
        assert reason == 'NO_PLANNER_COMMAND'

    def test_commanded_speed_zero_blocks_recovery(self):
        """Zero commanded speed (vehicle not supposed to move) blocks recovery."""
        node = _make_node(commanded_speed=0.0)
        reason = node._guard_conditions_block_recovery()
        assert reason == 'COMMANDED_SPEED_ZERO'

    def test_commanded_speed_below_threshold_blocks_recovery(self):
        """Commanded speed below threshold blocks recovery."""
        node = _make_node(commanded_speed=0.01,
                          stuck_command_speed_threshold=0.05)
        reason = node._guard_conditions_block_recovery()
        assert reason == 'COMMANDED_SPEED_ZERO'

    def test_healthy_conditions_allow_recovery(self):
        """All healthy conditions must allow recovery (return None)."""
        node = _make_node(
            commanded_speed=0.15,
            measured_speed=0.0,
            safety_state='LOCAL_NAVIGATE',
            localization_confidence=1.0,
        )
        reason = node._guard_conditions_block_recovery()
        assert reason is None


# ---------------------------------------------------------------------------
# Stuck condition detection tests
# ---------------------------------------------------------------------------

class TestStuckConditionDetection:
    """Stuck condition must be detected only when both thresholds are met."""

    def test_stuck_detected_when_cmd_high_meas_low(self):
        """Stuck detected: commanded > threshold, measured < threshold."""
        node = _make_node(
            commanded_speed=0.15,
            measured_speed=0.005,
            stuck_command_speed_threshold=0.05,
            stuck_measured_speed_threshold=0.02,
        )
        assert node._is_stuck_condition_active() is True

    def test_not_stuck_when_vehicle_moving(self):
        """Not stuck when measured speed is above threshold."""
        node = _make_node(
            commanded_speed=0.15,
            measured_speed=0.10,
            stuck_command_speed_threshold=0.05,
            stuck_measured_speed_threshold=0.02,
        )
        assert node._is_stuck_condition_active() is False

    def test_not_stuck_when_cmd_speed_zero(self):
        """Not stuck when commanded speed is zero (not trying to move)."""
        node = _make_node(
            commanded_speed=0.0,
            measured_speed=0.0,
            stuck_command_speed_threshold=0.05,
            stuck_measured_speed_threshold=0.02,
        )
        assert node._is_stuck_condition_active() is False

    def test_not_stuck_when_cmd_speed_below_threshold(self):
        """Not stuck when commanded speed is below minimum threshold."""
        node = _make_node(
            commanded_speed=0.03,
            measured_speed=0.0,
            stuck_command_speed_threshold=0.05,
        )
        assert node._is_stuck_condition_active() is False

    def test_not_stuck_when_no_data(self):
        """Not stuck when no odometry or command received yet."""
        node = _make_node()
        node._commanded_speed = None
        node._measured_speed = None
        assert node._is_stuck_condition_active() is False


# ---------------------------------------------------------------------------
# Displacement calculation tests
# ---------------------------------------------------------------------------

class TestDisplacement:
    """Displacement from recovery start must be computed correctly."""

    def test_zero_displacement_at_start(self):
        """Displacement is zero when position has not changed."""
        node = _make_node()
        node._recovery_start_pos_x = 0.0
        node._recovery_start_pos_y = 0.0
        node._pos_x = 0.0
        node._pos_y = 0.0
        assert math.isclose(node._displacement_from_recovery_start(), 0.0)

    def test_correct_displacement_x_axis(self):
        """Displacement along x-axis equals absolute x movement."""
        node = _make_node()
        node._recovery_start_pos_x = 0.0
        node._recovery_start_pos_y = 0.0
        node._pos_x = 0.3
        node._pos_y = 0.0
        assert math.isclose(node._displacement_from_recovery_start(), 0.3)

    def test_correct_displacement_diagonal(self):
        """Displacement is computed as 2D Euclidean distance."""
        node = _make_node()
        node._recovery_start_pos_x = 1.0
        node._recovery_start_pos_y = 1.0
        node._pos_x = 1.0 + 0.3
        node._pos_y = 1.0 + 0.4
        # Expected: sqrt(0.09 + 0.16) = 0.5
        assert math.isclose(
            node._displacement_from_recovery_start(), 0.5, rel_tol=1e-5
        )


# ---------------------------------------------------------------------------
# FSM state transition tests (time-manipulated)
# ---------------------------------------------------------------------------

class TestFSMTransitions:
    """Core FSM transitions verified by direct state manipulation."""

    def test_initial_state_is_normal(self):
        """Initial FSM state is NORMAL before any inputs are received."""
        node = _make_node()
        assert node._fsm_state == RecoveryState.NORMAL

    def test_normal_stays_normal_when_not_stuck(self):
        """NORMAL state persists when no stuck condition is present."""
        node = _make_node(commanded_speed=0.15, measured_speed=0.12)
        now = time.monotonic()
        node._handle_normal(now)
        assert node._fsm_state == RecoveryState.NORMAL

    def test_normal_sets_timer_when_stuck_starts(self):
        """NORMAL state starts a timer when stuck condition first appears."""
        node = _make_node(commanded_speed=0.15, measured_speed=0.005)
        assert node._stuck_condition_start is None
        now = time.monotonic()
        node._handle_normal(now)
        assert node._stuck_condition_start is not None

    def test_normal_to_stuck_suspected_after_timeout(self):
        """NORMAL → STUCK_SUSPECTED after stuck_timeout_sec of stuck condition."""
        node = _make_node(
            commanded_speed=0.15,
            measured_speed=0.005,
            stuck_timeout_sec=3.0,
        )
        far_past = time.monotonic() - 5.0  # 5 seconds ago
        node._stuck_condition_start = far_past
        node._handle_normal(time.monotonic())
        assert node._fsm_state == RecoveryState.STUCK_SUSPECTED

    def test_stuck_suspected_to_normal_when_condition_clears(self):
        """STUCK_SUSPECTED → NORMAL when vehicle starts moving."""
        node = _make_node(commanded_speed=0.15, measured_speed=0.10)
        node._fsm_state = RecoveryState.STUCK_SUSPECTED
        node._suspect_entry_time = time.monotonic()
        now = time.monotonic()
        node._handle_stuck_suspected(now)
        assert node._fsm_state == RecoveryState.NORMAL

    def test_stuck_suspected_to_recovery_after_suspect_timeout(self):
        """STUCK_SUSPECTED → RECOVERY_ACTIVE after suspect timeout expires."""
        node = _make_node(
            commanded_speed=0.15,
            measured_speed=0.005,
            stuck_suspect_timeout_sec=2.0,
        )
        node._fsm_state = RecoveryState.STUCK_SUSPECTED
        node._suspect_entry_time = time.monotonic() - 5.0  # 5 seconds ago
        now = time.monotonic()
        node._handle_stuck_suspected(now)
        assert node._fsm_state == RecoveryState.RECOVERY_ACTIVE

    def test_recovery_success_returns_to_normal(self):
        """RECOVERY_SUCCESS state transitions back to NORMAL."""
        node = _make_node()
        node._fsm_state = RecoveryState.RECOVERY_SUCCESS
        node._handle_recovery_success()
        assert node._fsm_state == RecoveryState.NORMAL
        assert node._recovery_attempts == 0

    def test_recovery_failed_increments_attempts(self):
        """RECOVERY_FAILED increments the attempt counter."""
        node = _make_node(max_recovery_attempts=3)
        node._fsm_state = RecoveryState.RECOVERY_FAILED
        node._recovery_attempts = 1
        node._stuck_condition_start = time.monotonic() - 10.0
        node._handle_recovery_failed(time.monotonic())
        # Should try again — FSM goes back to STUCK_SUSPECTED
        assert node._fsm_state == RecoveryState.STUCK_SUSPECTED

    def test_recovery_failed_to_stopped_after_max_attempts(self):
        """RECOVERY_FAILED → STOPPED when max attempts are exhausted."""
        node = _make_node(max_recovery_attempts=3)
        node._fsm_state = RecoveryState.RECOVERY_FAILED
        node._recovery_attempts = 3  # already at max
        node._handle_recovery_failed(time.monotonic())
        assert node._fsm_state == RecoveryState.STOPPED

    def test_stopped_is_terminal(self):
        """STOPPED state does not transition to any other state."""
        node = _make_node()
        node._fsm_state = RecoveryState.STOPPED
        # Call update multiple times
        for _ in range(5):
            node._update()
        assert node._fsm_state == RecoveryState.STOPPED

    def test_guard_blocks_transition_to_suspected(self):
        """Guard conditions must prevent STUCK_SUSPECTED when localization is bad."""
        node = _make_node(
            commanded_speed=0.15,
            measured_speed=0.005,
            localization_confidence=0.05,  # bad localization
            stuck_timeout_sec=3.0,
        )
        far_past = time.monotonic() - 5.0
        node._stuck_condition_start = far_past
        node._handle_normal(time.monotonic())
        # Guard should have blocked the transition
        assert node._fsm_state == RecoveryState.NORMAL
        # Timer should have been reset
        assert node._stuck_condition_start is None


# ---------------------------------------------------------------------------
# cmd_vel_suggestion is NOT /cmd_vel (architecture test)
# ---------------------------------------------------------------------------

class TestArchitectureConstraints:
    """The recovery node must NOT publish to /cmd_vel directly."""

    def test_recovery_node_does_not_publish_cmd_vel(self):
        """Verify no direct /cmd_vel publisher in stuck_recovery.py."""
        import ast
        from pathlib import Path

        source = Path(__file__).resolve().parents[1] / \
            'sih26126_safety' / 'stuck_recovery.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))

        cmd_vel_publishers = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not (isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'create_publisher'
                    and len(node.args) >= 2
                    and isinstance(node.args[1], ast.Constant)
                    and node.args[1].value == '/cmd_vel'):
                continue
            cmd_vel_publishers.append(node)

        assert len(cmd_vel_publishers) == 0, (
            'stuck_recovery.py must NOT publish to /cmd_vel directly — '
            'safety_supervisor is the sole /cmd_vel authority'
        )

    def test_recovery_publishes_to_suggestion_topic(self):
        """Verify recovery publishes to /recovery/cmd_vel_suggestion, not /cmd_vel."""
        import ast
        from pathlib import Path

        source = Path(__file__).resolve().parents[1] / \
            'sih26126_safety' / 'stuck_recovery.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))

        suggestion_publishers = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not (isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'create_publisher'
                    and len(node.args) >= 2
                    and isinstance(node.args[1], ast.Constant)
                    and '/recovery/cmd_vel_suggestion' in node.args[1].value):
                continue
            suggestion_publishers.append(node)

        assert len(suggestion_publishers) >= 1, (
            'stuck_recovery.py must publish to /recovery/cmd_vel_suggestion'
        )
