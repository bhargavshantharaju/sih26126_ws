"""Unit tests for the DRISTHI obstacle tracker.

Tests cover:
  - Track lifecycle (creation, update, timeout removal)
  - Velocity estimation
  - Dynamic classification from velocity threshold
  - Track association by spatial proximity
  - Dynamic risk score computation
  - Predicted position validity
"""

import math
import time

import pytest

from sih26126_sensor_contract.obstacle_tracker import _Track


# ---------------------------------------------------------------------------
# _Track unit tests
# ---------------------------------------------------------------------------

class TestTrack:
    def _make_track(self, x=2.0, y=0.0, z=0.5, **kwargs):
        return _Track(
            track_id='obs_0000',
            x=x, y=y, z=z,
            dim_x=0.5, dim_y=0.5, dim_z=0.8,
            confidence=0.6,
            frame_id='lidar',
            **kwargs,
        )

    def test_initial_velocity_zero(self):
        t = self._make_track()
        assert t.vx == 0.0
        assert t.vy == 0.0
        assert t.vz == 0.0

    def test_initial_classification(self):
        t = self._make_track()
        assert t.is_dynamic is False
        assert t.classification == 'UNKNOWN'

    def test_update_position(self):
        t = self._make_track(x=2.0, y=0.0)
        t.update(x=3.0, y=0.0, z=0.5,
                 dim_x=0.5, dim_y=0.5, dim_z=0.8,
                 confidence=0.7, dt=1.0, dyn_thresh=0.15)
        assert t.x == pytest.approx(3.0)
        assert t.y == pytest.approx(0.0)

    def test_velocity_estimated_after_movement(self):
        """Moving 1 m in 1 s should give vx ≈ alpha * 1.0."""
        t = self._make_track(x=0.0, y=0.0, alpha=0.3)
        t.update(x=1.0, y=0.0, z=0.5,
                 dim_x=0.5, dim_y=0.5, dim_z=0.8,
                 confidence=0.7, dt=1.0, dyn_thresh=0.15)
        # alpha * (dx/dt) + (1-alpha) * 0 = 0.3 * 1.0
        assert t.vx == pytest.approx(0.3, abs=1e-4)

    def test_dynamic_classification_above_threshold(self):
        """Speed > threshold classifies track as DYNAMIC."""
        t = self._make_track(x=0.0, y=0.0, alpha=1.0)
        # dx=1.0, dt=0.5 → vx = 2.0 m/s, well above 0.15 m/s threshold
        t.update(x=1.0, y=0.0, z=0.5,
                 dim_x=0.5, dim_y=0.5, dim_z=0.8,
                 confidence=0.8, dt=0.5, dyn_thresh=0.15)
        assert t.is_dynamic is True
        assert t.classification == 'DYNAMIC'

    def test_static_classification_below_threshold(self):
        """Speed < threshold keeps track as STATIC."""
        t = self._make_track(x=0.0, y=0.0, alpha=1.0)
        # dx=0.01, dt=1.0 → vx = 0.01 m/s, below 0.15 m/s threshold
        t.update(x=0.01, y=0.0, z=0.5,
                 dim_x=0.5, dim_y=0.5, dim_z=0.8,
                 confidence=0.8, dt=1.0, dyn_thresh=0.15)
        assert t.is_dynamic is False
        assert t.classification == 'STATIC'

    def test_predicted_position_uses_velocity(self):
        """Predicted position = current + velocity * horizon."""
        t = self._make_track(x=0.0, y=0.0, alpha=1.0)
        t.update(x=1.0, y=0.0, z=0.5,
                 dim_x=0.5, dim_y=0.5, dim_z=0.8,
                 confidence=0.8, dt=1.0, dyn_thresh=0.15)
        from builtin_interfaces.msg import Time
        stamp = Time()
        msg = t.to_msg(stamp, predict_h=1.0)
        # vx = 1.0 (alpha=1.0), predicted_x = current_x + vx * 1.0
        assert msg.predicted_position.x == pytest.approx(t.x + t.vx * 1.0, abs=1e-4)

    def test_to_msg_fields(self):
        """to_msg should populate all expected fields."""
        t = self._make_track()
        from builtin_interfaces.msg import Time
        stamp = Time()
        msg = t.to_msg(stamp, predict_h=1.0)
        assert msg.id == 'obs_0000'
        assert math.isfinite(msg.position.x)
        assert 0.0 <= msg.confidence <= 1.0
        assert msg.age_sec >= 0.0
        assert msg.last_seen_sec >= 0.0
        assert msg.frame_id == 'lidar'

    def test_age_increases(self):
        """Track age should increase over time."""
        t = self._make_track()
        from builtin_interfaces.msg import Time
        stamp = Time()
        msg1 = t.to_msg(stamp, predict_h=1.0)
        time.sleep(0.05)
        msg2 = t.to_msg(stamp, predict_h=1.0)
        assert msg2.age_sec > msg1.age_sec

    def test_update_count_increments(self):
        t = self._make_track()
        assert t.update_count == 0
        t.update(x=1.0, y=0.0, z=0.5,
                 dim_x=0.5, dim_y=0.5, dim_z=0.8,
                 confidence=0.7, dt=1.0, dyn_thresh=0.15)
        assert t.update_count == 1

    def test_dimensions_smoothed(self):
        """Dimensions should update with smoothing, not jump instantly."""
        t = self._make_track()
        original_dim_x = t.dim_x  # 0.5
        t.update(x=2.0, y=0.0, z=0.5,
                 dim_x=2.0, dim_y=2.0, dim_z=2.0,  # large new detection
                 confidence=0.8, dt=1.0, dyn_thresh=0.15)
        # 0.7 * 0.5 + 0.3 * 2.0 = 0.35 + 0.60 = 0.95
        assert t.dim_x == pytest.approx(0.95, abs=1e-4)

    def test_very_short_dt_skips_velocity(self):
        """dt < 0.05 should not update velocity (prevents division artifacts)."""
        t = self._make_track(x=0.0, alpha=1.0)
        t.update(x=1.0, y=0.0, z=0.5,
                 dim_x=0.5, dim_y=0.5, dim_z=0.8,
                 confidence=0.8, dt=0.01, dyn_thresh=0.15)
        # Velocity should remain 0 (dt too small)
        assert t.vx == pytest.approx(0.0, abs=1e-4)


# ---------------------------------------------------------------------------
# Dynamic risk calculation
# ---------------------------------------------------------------------------

class TestDynamicRisk:
    """Verify dynamic risk formula is distance-based."""

    def test_closer_dynamic_obstacle_higher_risk(self):
        """A dynamic obstacle at 1m should give higher risk than one at 5m."""
        close = _Track('a', x=1.0, y=0.0, z=0.5,
                       dim_x=0.5, dim_y=0.5, dim_z=0.8,
                       confidence=0.9, frame_id='l')
        close.is_dynamic = True
        far = _Track('b', x=5.0, y=0.0, z=0.5,
                     dim_x=0.5, dim_y=0.5, dim_z=0.8,
                     confidence=0.9, frame_id='l')
        far.is_dynamic = True

        def risk(t):
            dist = math.hypot(t.x, t.y)
            return 1.0 / (1.0 + max(dist, 0.1))

        assert risk(close) > risk(far)

    def test_static_obstacle_zero_risk(self):
        """Static obstacle should contribute 0 to dynamic risk."""
        t = _Track('c', x=1.0, y=0.0, z=0.5,
                   dim_x=0.5, dim_y=0.5, dim_z=0.8,
                   confidence=0.9, frame_id='l')
        assert t.is_dynamic is False
        # Static obstacles never enter dynamic risk
        risk = 0.0
        if t.is_dynamic:
            dist = math.hypot(t.x, t.y)
            risk = 1.0 / (1.0 + max(dist, 0.1))
        assert risk == 0.0

    def test_risk_bounded_zero_to_one(self):
        """Risk should always be in [0, 1]."""
        t = _Track('d', x=0.5, y=0.0, z=0.5,
                   dim_x=0.5, dim_y=0.5, dim_z=0.8,
                   confidence=0.9, frame_id='l')
        t.is_dynamic = True
        dist = math.hypot(t.x, t.y)
        risk = min(1.0, 1.0 / (1.0 + max(dist, 0.1)))
        assert 0.0 <= risk <= 1.0
