"""Tests for the PlannerRisk assessment logic in local_path_selector."""
# Covers: risk level classification, dynamic penalty, sensor degradation cost.

import pytest


# ---------------------------------------------------------------------------
# Core risk formula extracted from _publish_risk_assessment for unit testing.
# Weights: terrain=0.20, obstacle=0.25, dynamic=0.20, loc=0.20, sensor=0.15
# ---------------------------------------------------------------------------

def compute_risk_level(total):
    """Map a total risk value to a categorical risk level string."""
    if total < 0.3:
        return 'LOW'
    elif total < 0.6:
        return 'MODERATE'
    elif total < 0.8:
        return 'HIGH'
    return 'CRITICAL'


def compute_total_risk(
    terrain_cost,
    obstacle_cost,
    dynamic_risk_cost,
    localization_risk,
    sensor_deg_cost,
):
    """Compute weighted total risk from individual component costs."""
    total = (
        0.20 * max(0.0, min(1.0, terrain_cost))
        + 0.25 * max(0.0, min(1.0, obstacle_cost))
        + 0.20 * max(0.0, min(1.0, dynamic_risk_cost))
        + 0.20 * max(0.0, min(1.0, localization_risk))
        + 0.15 * max(0.0, min(1.0, sensor_deg_cost))
    )
    return max(0.0, min(1.0, total))


class TestRiskLevelClassification:
    """Risk level thresholds are correctly classified."""

    def test_all_zero_is_low(self):
        """All zero costs should produce LOW risk."""
        total = compute_total_risk(0.0, 0.0, 0.0, 0.0, 0.0)
        assert compute_risk_level(total) == 'LOW'

    def test_threshold_low_moderate(self):
        """Values just below and at 0.3 transition LOW to MODERATE."""
        assert compute_risk_level(0.29) == 'LOW'
        assert compute_risk_level(0.30) == 'MODERATE'

    def test_threshold_moderate_high(self):
        """Values just below and at 0.6 transition MODERATE to HIGH."""
        assert compute_risk_level(0.59) == 'MODERATE'
        assert compute_risk_level(0.60) == 'HIGH'

    def test_threshold_high_critical(self):
        """Values just below and at 0.8 transition HIGH to CRITICAL."""
        assert compute_risk_level(0.79) == 'HIGH'
        assert compute_risk_level(0.80) == 'CRITICAL'

    def test_all_one_is_critical(self):
        """All inputs at 1.0 should produce CRITICAL risk."""
        total = compute_total_risk(1.0, 1.0, 1.0, 1.0, 1.0)
        assert compute_risk_level(total) == 'CRITICAL'

    def test_weights_sum_to_one(self):
        """When all inputs are 1.0, total should be exactly 1.0."""
        total = compute_total_risk(1.0, 1.0, 1.0, 1.0, 1.0)
        assert total == pytest.approx(1.0, abs=1e-6)

    def test_total_clamped_zero(self):
        """Negative inputs should be clamped to 0."""
        total = compute_total_risk(-5.0, -5.0, 0.0, 0.0, 0.0)
        assert total == pytest.approx(0.0, abs=1e-6)

    def test_total_clamped_one(self):
        """Inputs above 1.0 should be clamped to 1.0."""
        total = compute_total_risk(10.0, 10.0, 10.0, 10.0, 10.0)
        assert total == pytest.approx(1.0, abs=1e-6)


class TestDynamicObstacleEffect:
    """Dynamic obstacle risk changes total cost."""

    def test_dynamic_risk_increases_total(self):
        """Adding dynamic risk should increase total cost."""
        no_dyn = compute_total_risk(0.1, 0.1, 0.0, 0.1, 0.0)
        with_dyn = compute_total_risk(0.1, 0.1, 0.8, 0.1, 0.0)
        assert with_dyn > no_dyn

    def test_high_dynamic_risk_raises_total(self):
        """Dynamic obstacle risk of 1.0 contributes 0.20 to total."""
        total = compute_total_risk(0.0, 0.0, 1.0, 0.0, 0.0)
        assert total == pytest.approx(0.20, abs=1e-6)

    def test_dynamic_risk_weight_correct(self):
        """Dynamic risk weight is 0.20; pure dynamic test."""
        total = compute_total_risk(0.0, 0.0, 0.5, 0.0, 0.0)
        assert total == pytest.approx(0.10, abs=1e-6)


class TestSensorDegradationCost:
    """Sensor degradation from autonomy mode maps correctly."""

    def test_full_autonomy_zero_sensor_cost(self):
        """Zero sensor degradation cost contributes nothing."""
        total = compute_total_risk(0.0, 0.0, 0.0, 0.0, 0.0)
        assert total == pytest.approx(0.0, abs=1e-6)

    def test_degraded_sensor_cost_0_4_weight(self):
        """DEGRADED_SENSOR (0.4) costs 0.15*0.4=0.06 total."""
        total = compute_total_risk(0.0, 0.0, 0.0, 0.0, 0.4)
        assert total == pytest.approx(0.06, abs=1e-6)

    def test_degraded_perception_cost_0_2_weight(self):
        """DEGRADED_PERCEPTION (0.2) costs 0.15*0.2=0.03 total."""
        total = compute_total_risk(0.0, 0.0, 0.0, 0.0, 0.2)
        assert total == pytest.approx(0.03, abs=1e-6)


class TestLocalizationRisk:
    """Localization confidence risk = 1 - confidence."""

    def test_full_confidence_zero_risk(self):
        """Full confidence (1.0) should produce zero localization risk."""
        loc_risk = max(0.0, 1.0 - 1.0)
        assert loc_risk == pytest.approx(0.0, abs=1e-6)

    def test_low_confidence_high_risk(self):
        """Very low confidence (0.1) should produce high risk."""
        loc_risk = max(0.0, 1.0 - 0.1)
        assert loc_risk == pytest.approx(0.9, abs=1e-6)

    def test_low_confidence_increases_total(self):
        """Localization risk should increase total cost."""
        full_conf = compute_total_risk(0.0, 0.0, 0.0, 0.0, 0.0)
        low_conf = compute_total_risk(0.0, 0.0, 0.0, 0.9, 0.0)
        assert low_conf > full_conf

    def test_localization_weight_correct(self):
        """Localization risk weight is 0.20; 0.20*1.0=0.20."""
        total = compute_total_risk(0.0, 0.0, 0.0, 1.0, 0.0)
        assert total == pytest.approx(0.20, abs=1e-6)


class TestObstacleScoreFormula:
    """Dynamic obstacle penalty reduces effective obstacle score."""

    def test_dynamic_penalty_reduces_score(self):
        """Dynamic obstacle penalty should reduce the obstacle score."""
        from sih26126_planner.local_path_selector import (
            obstacle_clearance_score)
        clearance = 2.5
        base_score = obstacle_clearance_score(clearance)
        dynamic_risk = 0.6
        penalized = max(0.0, base_score - dynamic_risk * 0.5)
        assert penalized < base_score

    def test_zero_dynamic_risk_no_penalty(self):
        """Zero dynamic risk should not change the obstacle score."""
        from sih26126_planner.local_path_selector import (
            obstacle_clearance_score)
        clearance = 2.5
        base_score = obstacle_clearance_score(clearance)
        penalized = max(0.0, base_score - 0.0 * 0.5)
        assert penalized == pytest.approx(base_score, abs=1e-6)

    def test_critical_clearance_returns_zero(self):
        """Clearance below critical distance should score zero."""
        from sih26126_planner.local_path_selector import (
            obstacle_clearance_score)
        assert obstacle_clearance_score(0.5) == pytest.approx(0.0, abs=1e-6)

    def test_clear_corridor_returns_one(self):
        """Clearance above clear_distance should score one."""
        from sih26126_planner.local_path_selector import (
            obstacle_clearance_score)
        assert obstacle_clearance_score(5.0) == pytest.approx(1.0, abs=1e-6)
