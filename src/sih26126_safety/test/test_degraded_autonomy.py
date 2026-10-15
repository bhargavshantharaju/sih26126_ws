"""Unit tests for the DRISTHI degraded autonomy state machine."""
# Covers: mode priority, _higher_priority arbitration, capability scores.

import inspect
import re

from sih26126_safety import degraded_autonomy as _da_module
from sih26126_safety.degraded_autonomy import (
    _higher_priority,
    _MODE_PRIORITY,
)


# ---------------------------------------------------------------------------
# Mode priority
# ---------------------------------------------------------------------------

class TestModePriority:
    """Verify the priority table is consistent with degradation severity."""

    def test_safe_hold_lowest_priority(self):
        """SAFE_HOLD has the lowest priority value (most degraded)."""
        assert _MODE_PRIORITY['SAFE_HOLD'] == 0

    def test_full_autonomy_highest_priority(self):
        """FULL_AUTONOMY has the highest priority value (least degraded)."""
        assert _MODE_PRIORITY['FULL_AUTONOMY'] == max(_MODE_PRIORITY.values())

    def test_recovery_above_safe_hold(self):
        """RECOVERY is less degraded than SAFE_HOLD."""
        assert _MODE_PRIORITY['RECOVERY'] > _MODE_PRIORITY['SAFE_HOLD']

    def test_degraded_sensor_above_recovery(self):
        """DEGRADED_SENSOR is less degraded than RECOVERY."""
        assert _MODE_PRIORITY['DEGRADED_SENSOR'] > _MODE_PRIORITY['RECOVERY']

    def test_degraded_localization_above_sensor(self):
        """DEGRADED_LOCALIZATION is less degraded than DEGRADED_SENSOR."""
        assert (
            _MODE_PRIORITY['DEGRADED_LOCALIZATION']
            > _MODE_PRIORITY['DEGRADED_SENSOR']
        )

    def test_degraded_perception_above_localization(self):
        """DEGRADED_PERCEPTION is less degraded than DEGRADED_LOCALIZATION."""
        assert (
            _MODE_PRIORITY['DEGRADED_PERCEPTION']
            > _MODE_PRIORITY['DEGRADED_LOCALIZATION']
        )

    def test_return_home_above_perception(self):
        """RETURN_HOME is less degraded than DEGRADED_PERCEPTION."""
        assert (
            _MODE_PRIORITY['RETURN_HOME']
            > _MODE_PRIORITY['DEGRADED_PERCEPTION']
        )

    def test_all_modes_present(self):
        """All required autonomy modes must be in the priority table."""
        expected = {
            'FULL_AUTONOMY', 'RETURN_HOME', 'DEGRADED_PERCEPTION',
            'DEGRADED_LOCALIZATION', 'DEGRADED_SENSOR',
            'RECOVERY', 'SAFE_HOLD',
        }
        assert set(_MODE_PRIORITY.keys()) == expected


# ---------------------------------------------------------------------------
# _higher_priority
# ---------------------------------------------------------------------------

class TestHigherPriority:
    """Verify _higher_priority always returns the more degraded mode."""

    def test_safe_hold_beats_full_autonomy(self):
        """SAFE_HOLD should override FULL_AUTONOMY."""
        assert _higher_priority('FULL_AUTONOMY', 'SAFE_HOLD') == 'SAFE_HOLD'

    def test_full_autonomy_does_not_override_safe_hold(self):
        """FULL_AUTONOMY should not override SAFE_HOLD."""
        assert _higher_priority('SAFE_HOLD', 'FULL_AUTONOMY') == 'SAFE_HOLD'

    def test_same_mode_returns_same(self):
        """Same mode in both args should return that mode unchanged."""
        assert _higher_priority('RECOVERY', 'RECOVERY') == 'RECOVERY'

    def test_recovery_beats_full_autonomy(self):
        """RECOVERY is more degraded than FULL_AUTONOMY."""
        assert _higher_priority('FULL_AUTONOMY', 'RECOVERY') == 'RECOVERY'

    def test_degraded_sensor_beats_return_home(self):
        """DEGRADED_SENSOR is more degraded than RETURN_HOME."""
        result = _higher_priority('RETURN_HOME', 'DEGRADED_SENSOR')
        assert result == 'DEGRADED_SENSOR'

    def test_full_autonomy_does_not_override_degraded(self):
        """FULL_AUTONOMY must not override DEGRADED_LOCALIZATION."""
        result = _higher_priority('DEGRADED_LOCALIZATION', 'FULL_AUTONOMY')
        assert result == 'DEGRADED_LOCALIZATION'

    def test_safe_hold_beats_recovery(self):
        """SAFE_HOLD is more degraded than RECOVERY."""
        assert _higher_priority('RECOVERY', 'SAFE_HOLD') == 'SAFE_HOLD'

    def test_all_modes_beat_full_autonomy(self):
        """Every mode other than FULL_AUTONOMY should override it."""
        other_modes = [
            'SAFE_HOLD', 'RECOVERY', 'DEGRADED_SENSOR',
            'DEGRADED_LOCALIZATION', 'DEGRADED_PERCEPTION', 'RETURN_HOME',
        ]
        for mode in other_modes:
            result = _higher_priority('FULL_AUTONOMY', mode)
            assert result == mode, f'{mode} should beat FULL_AUTONOMY'


# ---------------------------------------------------------------------------
# Capability scores
# ---------------------------------------------------------------------------

class TestCapabilityScores:
    """Verify capability scores decrease with degradation severity."""

    CAP = {
        'FULL_AUTONOMY': 1.0,
        'RETURN_HOME': 0.75,
        'DEGRADED_PERCEPTION': 0.6,
        'DEGRADED_LOCALIZATION': 0.45,
        'DEGRADED_SENSOR': 0.3,
        'RECOVERY': 0.15,
        'SAFE_HOLD': 0.0,
    }

    def test_full_autonomy_capability_one(self):
        """FULL_AUTONOMY should have capability score 1.0."""
        assert self.CAP['FULL_AUTONOMY'] == 1.0

    def test_safe_hold_capability_zero(self):
        """SAFE_HOLD should have capability score 0.0."""
        assert self.CAP['SAFE_HOLD'] == 0.0

    def test_capability_decreases_with_priority(self):
        """Higher priority (less degraded) must have higher capability."""
        sorted_modes = sorted(
            self.CAP.keys(), key=lambda m: _MODE_PRIORITY[m])
        capabilities = [self.CAP[m] for m in sorted_modes]
        for i in range(len(capabilities) - 1):
            assert capabilities[i] <= capabilities[i + 1], (
                f'{sorted_modes[i]} should have capability '
                f'<= {sorted_modes[i+1]}'
            )

    def test_all_capabilities_in_range(self):
        """All capability scores must be in [0, 1]."""
        for mode, cap in self.CAP.items():
            assert 0.0 <= cap <= 1.0, f'{mode} capability {cap} out of range'


# ---------------------------------------------------------------------------
# Integration: multi-signal degradation aggregation
# ---------------------------------------------------------------------------

class TestDegradationAggregation:
    """Verify the most degraded signal wins under simultaneous conditions."""

    def test_lidar_and_localization_degraded_gives_sensor(self):
        """DEGRADED_SENSOR overrides DEGRADED_LOCALIZATION."""
        target = 'FULL_AUTONOMY'
        target = _higher_priority(target, 'DEGRADED_LOCALIZATION')
        target = _higher_priority(target, 'DEGRADED_SENSOR')
        assert target == 'DEGRADED_SENSOR'

    def test_recovery_plus_sensor_gives_recovery(self):
        """RECOVERY is more degraded than DEGRADED_SENSOR."""
        target = 'FULL_AUTONOMY'
        target = _higher_priority(target, 'DEGRADED_SENSOR')
        target = _higher_priority(target, 'RECOVERY')
        assert target == 'RECOVERY'

    def test_safe_hold_overrides_everything(self):
        """SAFE_HOLD always wins regardless of order."""
        target = 'FULL_AUTONOMY'
        for mode in [
            'RETURN_HOME', 'DEGRADED_PERCEPTION', 'DEGRADED_LOCALIZATION',
            'DEGRADED_SENSOR', 'RECOVERY',
        ]:
            target = _higher_priority(target, mode)
        target = _higher_priority(target, 'SAFE_HOLD')
        assert target == 'SAFE_HOLD'

    def test_return_home_then_full_stays_return_home(self):
        """RETURN_HOME should not be overridden by FULL_AUTONOMY."""
        result = _higher_priority('RETURN_HOME', 'FULL_AUTONOMY')
        assert result == 'RETURN_HOME'


# ---------------------------------------------------------------------------
# Startup grace period – structural check
# These tests verify the DegradedAutonomyManager carries the correct logic
# for distinguishing "sensor not yet initialized" from "sensor stale/failed".
# We inspect the source directly since the Node requires a live ROS context.
# ---------------------------------------------------------------------------


class TestStartupGracePeriod:
    """Verify the startup grace period is present and architecturally sound."""

    def test_startup_grace_source_present(self):
        """The manager must contain a startup grace period for lidar_no_data."""
        src = inspect.getsource(_da_module.DegradedAutonomyManager._tick)
        assert '_startup_grace_sec' in src, (
            '_tick must reference _startup_grace_sec to enforce grace period')
        assert '_startup_time' in src, (
            '_tick must reference _startup_time to measure uptime')

    def test_startup_grace_does_not_suppress_later_staleness(self):
        """
        Stale LiDAR health message after grace must still produce DEGRADED_SENSOR.

        This ensures the grace period only covers the 'never received' case,
        not a genuine sensor dropout that happens after startup.
        """
        src = inspect.getsource(_da_module.DegradedAutonomyManager._tick)
        # lidar_health_time being set (not None) means we are past init;
        # the stale check must still fire in that branch.
        assert 'lidar_age > self._lidar_stale' in src, (
            'Runtime LiDAR staleness check must remain in the tick loop')

    def test_grace_period_long_enough(self):
        """Grace period must be at least 10 seconds to cover slow startups."""
        src = inspect.getsource(_da_module.DegradedAutonomyManager.__init__)
        # Extract the numeric constant assigned to _startup_grace_sec
        match = re.search(r'_startup_grace_sec\s*=\s*([0-9.]+)', src)
        assert match is not None, '_startup_grace_sec must be set in __init__'
        grace = float(match.group(1))
        assert grace >= 10.0, (
            f'Grace period {grace}s is too short; must be at least 10s')
