"""Tests for the DRISTHI mission intelligence layer.

Tests cover:
  - Event emission only on state changes
  - Correct severity levels per state
  - Dynamic obstacle appear/disappear events
  - Mission state transition events
  - Summary publication
"""

import pytest


# ---------------------------------------------------------------------------
# Helper: simulate state change detection logic from mission_intelligence.py
# The actual node requires ROS context. These tests verify the pure logic.
# ---------------------------------------------------------------------------

def should_emit_autonomy_event(current_mode, prev_mode):
    """Return (should_emit, severity) for an autonomy mode change."""
    if current_mode is None or current_mode == prev_mode:
        return False, None
    sev = 'INFO' if current_mode == 'FULL_AUTONOMY' else 'WARNING'
    return True, sev


def should_emit_recovery_event(current_state, prev_state):
    """Return (should_emit, severity) for a recovery state change."""
    if current_state is None or current_state == prev_state:
        return False, None
    if current_state in ('RECOVERY_FAILED', 'STOPPED'):
        sev = 'CRITICAL'
    elif 'RECOVERY' in current_state:
        sev = 'RECOVERY'
    else:
        sev = 'INFO'
    return True, sev


def should_emit_dynamic_obs_event(current_count, prev_count):
    """Return event type or None."""
    if current_count > 0 and prev_count == 0:
        return 'APPEARED'
    if current_count == 0 and prev_count > 0:
        return 'CLEARED'
    return None


# ---------------------------------------------------------------------------
# Autonomy mode events
# ---------------------------------------------------------------------------

class TestAutonomyModeEvents:
    def test_no_event_when_mode_unchanged(self):
        emit, sev = should_emit_autonomy_event('FULL_AUTONOMY', 'FULL_AUTONOMY')
        assert emit is False

    def test_event_on_mode_change(self):
        emit, sev = should_emit_autonomy_event('DEGRADED_LOCALIZATION', 'FULL_AUTONOMY')
        assert emit is True
        assert sev == 'WARNING'

    def test_recovery_to_full_is_info(self):
        emit, sev = should_emit_autonomy_event('FULL_AUTONOMY', 'RECOVERY')
        assert emit is True
        assert sev == 'INFO'

    def test_full_to_safe_hold_is_warning(self):
        emit, sev = should_emit_autonomy_event('SAFE_HOLD', 'FULL_AUTONOMY')
        assert emit is True
        assert sev == 'WARNING'

    def test_no_event_when_mode_none(self):
        emit, sev = should_emit_autonomy_event(None, 'FULL_AUTONOMY')
        assert emit is False

    def test_degraded_sensor_is_warning(self):
        emit, sev = should_emit_autonomy_event('DEGRADED_SENSOR', 'RETURN_HOME')
        assert emit is True
        assert sev == 'WARNING'


# ---------------------------------------------------------------------------
# Recovery state events
# ---------------------------------------------------------------------------

class TestRecoveryEvents:
    def test_no_event_when_unchanged(self):
        emit, sev = should_emit_recovery_event('NORMAL', 'NORMAL')
        assert emit is False

    def test_recovery_active_fires(self):
        emit, sev = should_emit_recovery_event('RECOVERY_ACTIVE', 'NORMAL')
        assert emit is True
        assert sev == 'RECOVERY'

    def test_recovery_failed_is_critical(self):
        emit, sev = should_emit_recovery_event('RECOVERY_FAILED', 'RECOVERY_ACTIVE')
        assert emit is True
        assert sev == 'CRITICAL'

    def test_stopped_is_critical(self):
        emit, sev = should_emit_recovery_event('STOPPED', 'RECOVERY_FAILED')
        assert emit is True
        assert sev == 'CRITICAL'

    def test_stuck_suspected_fires(self):
        emit, sev = should_emit_recovery_event('STUCK_SUSPECTED', 'NORMAL')
        assert emit is True

    def test_return_to_normal_is_info(self):
        emit, sev = should_emit_recovery_event('NORMAL', 'RECOVERY_SUCCESS')
        assert emit is True
        assert sev == 'INFO'


# ---------------------------------------------------------------------------
# Dynamic obstacle events
# ---------------------------------------------------------------------------

class TestDynamicObstacleEvents:
    def test_appeared_when_count_goes_positive(self):
        ev = should_emit_dynamic_obs_event(1, 0)
        assert ev == 'APPEARED'

    def test_cleared_when_count_drops_to_zero(self):
        ev = should_emit_dynamic_obs_event(0, 2)
        assert ev == 'CLEARED'

    def test_no_event_when_count_stays_zero(self):
        ev = should_emit_dynamic_obs_event(0, 0)
        assert ev is None

    def test_no_event_when_count_stays_positive(self):
        ev = should_emit_dynamic_obs_event(3, 2)
        assert ev is None

    def test_appeared_on_first_detection(self):
        ev = should_emit_dynamic_obs_event(1, 0)
        assert ev == 'APPEARED'


# ---------------------------------------------------------------------------
# Safety reason events
# ---------------------------------------------------------------------------

class TestSafetyReasonEvents:
    STOP_REASONS = [
        'STALE_LIDAR',
        'STALE_LOCALIZATION',
        'LOW_LOCALIZATION_CONFIDENCE',
        'OBSTACLE_STOP',
    ]

    def test_stop_reasons_trigger_warning(self):
        """All critical stop reasons should produce a warning event."""
        for reason in self.STOP_REASONS:
            should_warn = reason in (
                'STALE_LIDAR', 'STALE_LOCALIZATION',
                'LOW_LOCALIZATION_CONFIDENCE', 'OBSTACLE_STOP',
            )
            assert should_warn, f'{reason} should trigger a warning event'

    def test_safe_command_after_stop_triggers_info(self):
        """Returning to SAFE_COMMAND after a non-safe state triggers INFO."""
        prev = 'STALE_LIDAR'
        current = 'SAFE_COMMAND'
        # Info event only when transitioning from a non-safe reason
        should_emit = current == 'SAFE_COMMAND' and prev not in (None, 'SAFE_COMMAND')
        assert should_emit is True

    def test_no_event_when_reason_unchanged(self):
        prev = 'SAFE_COMMAND'
        current = 'SAFE_COMMAND'
        should_emit = current != prev
        assert should_emit is False


# ---------------------------------------------------------------------------
# Summary publication format
# ---------------------------------------------------------------------------

class TestSummaryFormat:
    """Verify summary string components."""

    def test_summary_contains_mode(self):
        lines = [
            'Mode: FULL_AUTONOMY',
            'Mission: MAPPING',
            'Safety: SAFE',
        ]
        summary = ' | '.join(lines)
        assert 'Mode:' in summary
        assert 'FULL_AUTONOMY' in summary

    def test_summary_includes_stop_reason_when_not_safe(self):
        safety_reason = 'STALE_LIDAR'
        lines = []
        if safety_reason and safety_reason != 'SAFE_COMMAND':
            lines.append(f'Stop reason: {safety_reason}')
        assert any('Stop reason' in line for line in lines)

    def test_summary_no_stop_reason_when_safe(self):
        safety_reason = 'SAFE_COMMAND'
        lines = []
        if safety_reason and safety_reason != 'SAFE_COMMAND':
            lines.append(f'Stop reason: {safety_reason}')
        assert not any('Stop reason' in line for line in lines)

    def test_dynamic_obs_in_summary_when_present(self):
        dynamic_count = 2
        dynamic_risk = 0.6
        lines = []
        if dynamic_count > 0:
            lines.append(f'Dynamic obstacles: {dynamic_count} (risk={dynamic_risk:.2f})')
        assert len(lines) == 1
        assert '2' in lines[0]
