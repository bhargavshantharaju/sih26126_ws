#!/usr/bin/env python3
"""Tests for cost map fusion logic."""

import ast
import pathlib
import unittest

from sih26126_planner.cost_map_fusion import (
    fuse_costs,
    localization_confidence_to_cost,
    occupancy_to_cost,
    terrain_score_to_cost,
)


class TestOccupancyCostConversion(unittest.TestCase):
    """Test occupancy grid to cost conversion."""

    def test_unknown_is_neutral(self):
        """Unknown cells (-1) have neutral cost."""
        cost = occupancy_to_cost(-1)
        assert abs(cost - 0.5) < 0.01

    def test_free_is_low_cost(self):
        """Free cells (0) have zero cost."""
        cost = occupancy_to_cost(0)
        assert cost == 0.0

    def test_occupied_is_high_cost(self):
        """Occupied cells (100) have maximum cost."""
        cost = occupancy_to_cost(100)
        assert cost == 1.0

    def test_partially_occupied_proportional(self):
        """Partially occupied cells have proportional cost."""
        cost_25 = occupancy_to_cost(25)
        cost_49 = occupancy_to_cost(49)

        assert abs(cost_25 - 0.25) < 0.01
        assert abs(cost_49 - 0.49) < 0.01

    def test_threshold_at_fifty(self):
        """Threshold at 50 separates free from occupied."""
        cost_49 = occupancy_to_cost(49)
        cost_50 = occupancy_to_cost(50)

        assert cost_49 < 0.5
        assert cost_50 == 1.0


class TestTerrainCostConversion(unittest.TestCase):
    """Test terrain score to cost conversion."""

    def test_safe_terrain_low_cost(self):
        """Safe terrain (score=1.0) has zero cost."""
        cost = terrain_score_to_cost(1.0)
        assert cost == 0.0

    def test_unsafe_terrain_high_cost(self):
        """Unsafe terrain (score=0.0) has maximum cost."""
        cost = terrain_score_to_cost(0.0)
        assert cost == 1.0

    def test_moderate_terrain_moderate_cost(self):
        """Moderate terrain has proportional cost."""
        cost_70 = terrain_score_to_cost(0.7)
        assert abs(cost_70 - 0.3) < 0.01

    def test_inversion_relationship(self):
        """Cost and score are inversely related."""
        for score in [0.0, 0.25, 0.5, 0.75, 1.0]:
            cost = terrain_score_to_cost(score)
            assert abs(cost + score - 1.0) < 0.01


class TestLocalizationCostMultiplier(unittest.TestCase):
    """Test localization confidence to cost multiplier."""

    def test_perfect_confidence_no_penalty(self):
        """Perfect confidence (1.0) has multiplier of 1.0."""
        multiplier = localization_confidence_to_cost(1.0)
        assert abs(multiplier - 1.0) < 0.01

    def test_zero_confidence_double_cost(self):
        """Zero confidence doubles the cost."""
        multiplier = localization_confidence_to_cost(0.0)
        assert abs(multiplier - 2.0) < 0.01

    def test_half_confidence_midpoint(self):
        """Half confidence gives 1.5× multiplier."""
        multiplier = localization_confidence_to_cost(0.5)
        assert abs(multiplier - 1.5) < 0.01

    def test_monotonic_decrease(self):
        """Higher confidence gives lower multiplier."""
        mult_0_2 = localization_confidence_to_cost(0.2)
        mult_0_5 = localization_confidence_to_cost(0.5)
        mult_0_8 = localization_confidence_to_cost(0.8)

        assert mult_0_2 > mult_0_5 > mult_0_8


class TestCostFusion(unittest.TestCase):
    """Test multi-source cost fusion."""

    def test_all_safe_is_zero_cost(self):
        """All safe inputs produce zero cost."""
        cost = fuse_costs(
            occupancy_cost=0.0,
            terrain_cost=0.0,
            localization_multiplier=1.0,
            occupancy_weight=0.5,
            terrain_weight=0.5,
            localization_weight=0.0
        )
        assert cost == 0.0

    def test_all_unsafe_is_maximum_cost(self):
        """All unsafe inputs produce maximum cost."""
        cost = fuse_costs(
            occupancy_cost=1.0,
            terrain_cost=1.0,
            localization_multiplier=1.0,
            occupancy_weight=0.5,
            terrain_weight=0.5,
            localization_weight=0.0
        )
        assert cost == 1.0

    def test_occupancy_dominates_when_weighted_high(self):
        """High occupancy weight makes occupancy dominant."""
        cost_occupied = fuse_costs(
            occupancy_cost=1.0,
            terrain_cost=0.0,
            localization_multiplier=1.0,
            occupancy_weight=0.9,
            terrain_weight=0.1,
            localization_weight=0.0
        )
        cost_terrain = fuse_costs(
            occupancy_cost=0.0,
            terrain_cost=1.0,
            localization_multiplier=1.0,
            occupancy_weight=0.9,
            terrain_weight=0.1,
            localization_weight=0.0
        )

        assert cost_occupied > cost_terrain

    def test_poor_localization_increases_cost(self):
        """Poor localization confidence increases fused cost."""
        cost_good_loc = fuse_costs(
            occupancy_cost=0.5,
            terrain_cost=0.5,
            localization_multiplier=1.0,
            occupancy_weight=0.5,
            terrain_weight=0.3,
            localization_weight=0.2
        )
        cost_poor_loc = fuse_costs(
            occupancy_cost=0.5,
            terrain_cost=0.5,
            localization_multiplier=2.0,
            occupancy_weight=0.5,
            terrain_weight=0.3,
            localization_weight=0.2
        )

        assert cost_poor_loc > cost_good_loc

    def test_output_clamped_to_zero_one(self):
        """Fused cost is always in [0, 1]."""
        # Try to create extreme values
        cost = fuse_costs(
            occupancy_cost=1.0,
            terrain_cost=1.0,
            localization_multiplier=2.0,
            occupancy_weight=1.0,
            terrain_weight=1.0,
            localization_weight=1.0
        )

        assert 0.0 <= cost <= 1.0

    def test_equal_weights_average_costs(self):
        """Equal weights produce weighted average."""
        cost = fuse_costs(
            occupancy_cost=0.4,
            terrain_cost=0.6,
            localization_multiplier=1.0,
            occupancy_weight=0.5,
            terrain_weight=0.5,
            localization_weight=0.0
        )

        # Average of 0.4 and 0.6 = 0.5
        assert abs(cost - 0.5) < 0.01

    def test_default_weights_sum_near_one(self):
        """Default weights produce reasonable fusion."""
        cost = fuse_costs(
            occupancy_cost=0.5,
            terrain_cost=0.5,
            localization_multiplier=1.0
        )

        # With default weights (0.5, 0.3, 0.2)
        # Base = 0.5*0.5 + 0.3*0.5 = 0.4
        # Loc multiplier = 1.0 → no change
        # Result = 0.4
        assert abs(cost - 0.4) < 0.01


class TestCostFusionEdgeCases(unittest.TestCase):
    """Test edge cases in cost fusion."""

    def test_zero_weights_zero_cost(self):
        """Zero weights produce zero cost."""
        cost = fuse_costs(
            occupancy_cost=1.0,
            terrain_cost=1.0,
            localization_multiplier=2.0,
            occupancy_weight=0.0,
            terrain_weight=0.0,
            localization_weight=0.0
        )
        assert cost == 0.0

    def test_negative_costs_handled(self):
        """Negative costs are clamped to zero."""
        # This shouldn't happen in practice, but test robustness
        cost = fuse_costs(
            occupancy_cost=-0.5,
            terrain_cost=-0.5,
            localization_multiplier=1.0,
            occupancy_weight=0.5,
            terrain_weight=0.5,
            localization_weight=0.0
        )
        assert cost >= 0.0


if __name__ == '__main__':
    unittest.main()


# ---------------------------------------------------------------------------
# Regression tests: terrain_score_callback wiring (Fix-T)
# ---------------------------------------------------------------------------

_SRC_ROOT = pathlib.Path(__file__).resolve().parents[3] / 'src'
_FUSION_SRC = (
    _SRC_ROOT / 'sih26126_planner/sih26126_planner/cost_map_fusion.py'
)


def test_terrain_callback_is_not_noop():
    """
    terrain_score_callback must store the incoming score, not be a no-op.

    Regression: the original callback body was just ``pass``, so terrain
    input was silently discarded and the fusion always used the default
    score regardless of actual terrain conditions.
    """
    assert _FUSION_SRC.exists(), f'Source not found: {_FUSION_SRC}'
    source = _FUSION_SRC.read_text(encoding='utf-8')
    tree = ast.parse(source)

    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        if node.name != 'terrain_score_callback':
            continue
        # Must not be a single-statement 'pass' body
        body_stmts = node.body
        if (len(body_stmts) == 1 and isinstance(body_stmts[0], ast.Pass)):
            assert False, (
                'terrain_score_callback is still a no-op (pass). '
                'The fix must store msg.data into latest_terrain_score.'
            )
        # Body must contain an assignment to latest_terrain_score
        assigns_terrain = any(
            isinstance(stmt, ast.Assign)
            and any(
                isinstance(t, ast.Attribute)
                and t.attr == 'latest_terrain_score'
                for t in stmt.targets
                if isinstance(t, ast.Attribute)
            )
            for stmt in ast.walk(node)
        )
        assert assigns_terrain, (
            'terrain_score_callback must assign to self.latest_terrain_score. '
            'Terrain input will otherwise have no effect on the cost map.'
        )
        return  # found and validated

    assert False, 'terrain_score_callback not found in cost_map_fusion.py'


def test_terrain_score_affects_cost_map():
    """
    A non-default terrain score must change the fused cost.

    Verifies that the terrain_score_to_cost/fuse_costs pipeline actually
    propagates different terrain scores to different output costs.
    """
    from sih26126_planner.cost_map_fusion import (
        fuse_costs,
        terrain_score_to_cost,
    )

    # Default terrain score (0.5): neutral
    default_terrain_cost = terrain_score_to_cost(0.5)

    # Good terrain score (0.9): should produce lower cost
    good_terrain_cost = terrain_score_to_cost(0.9)

    # Bad terrain score (0.1): should produce higher cost
    bad_terrain_cost = terrain_score_to_cost(0.1)

    occ = 0.3
    loc_mult = 1.0

    cost_default = fuse_costs(occ, default_terrain_cost, loc_mult)
    cost_good = fuse_costs(occ, good_terrain_cost, loc_mult)
    cost_bad = fuse_costs(occ, bad_terrain_cost, loc_mult)

    assert cost_good < cost_default, (
        'Good terrain (score=0.9) must produce lower fused cost than default'
    )
    assert cost_bad > cost_default, (
        'Bad terrain (score=0.1) must produce higher fused cost than default'
    )


def test_stale_terrain_falls_back_to_default():
    """
    When terrain score is stale, the fusion must use the default score.

    This is verified at the source level: the process() method must check
    terrain age and use default_terrain_score when terrain has timed out.
    """
    assert _FUSION_SRC.exists()
    source = _FUSION_SRC.read_text(encoding='utf-8')

    # The fix adds a terrain_timeout check in process()
    assert 'terrain_timeout' in source or 'terrain_age' in source, (
        'cost_map_fusion.process() must check terrain freshness '
        'and fall back to default_terrain_score when stale.'
    )
    assert 'effective_terrain_score' in source, (
        'cost_map_fusion.process() must use an effective_terrain_score '
        'variable that reflects either the live score or the fallback.'
    )


def test_terrain_score_callback_stored_attribute_initialized():
    """latest_terrain_score must be initialized at node construction."""
    assert _FUSION_SRC.exists()
    source = _FUSION_SRC.read_text(encoding='utf-8')

    assert 'latest_terrain_score' in source, (
        'CostMapFusion must have a latest_terrain_score attribute '
        'for the terrain_score_callback to store into.'
    )
    assert 'last_terrain_update' in source, (
        'CostMapFusion must have a last_terrain_update attribute '
        'to track terrain score freshness.'
    )
