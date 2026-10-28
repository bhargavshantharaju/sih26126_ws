#!/usr/bin/env python3
"""Tests for terrain memory with time-decay and spatial tracking."""

import unittest

from sih26126_traversability.terrain_memory import (
    terrain_class_to_value,
    TERRAIN_FLAT,
    TERRAIN_ROUGH,
    TERRAIN_SLOPED,
    TERRAIN_STEEP,
    TERRAIN_UNKNOWN,
    terrain_value_to_traversability,
    TerrainCell,
)


class TestTerrainClassMapping(unittest.TestCase):
    """Test terrain class string to numeric conversion."""

    def test_all_classes_mapped(self):
        """All terrain classes have numeric values."""
        assert terrain_class_to_value('UNKNOWN') == TERRAIN_UNKNOWN
        assert terrain_class_to_value('FLAT') == TERRAIN_FLAT
        assert terrain_class_to_value('SLOPED') == TERRAIN_SLOPED
        assert terrain_class_to_value('ROUGH') == TERRAIN_ROUGH
        assert terrain_class_to_value('STEEP') == TERRAIN_STEEP

    def test_invalid_class_returns_unknown(self):
        """Invalid class string returns UNKNOWN."""
        assert terrain_class_to_value('INVALID') == TERRAIN_UNKNOWN
        assert terrain_class_to_value('') == TERRAIN_UNKNOWN


class TestTraversabilityScoring(unittest.TestCase):
    """Test terrain to traversability score conversion."""

    def test_unknown_with_zero_confidence_is_neutral(self):
        """Unknown terrain with no confidence returns neutral score."""
        score = terrain_value_to_traversability(TERRAIN_UNKNOWN, 0.0)
        assert abs(score - 0.5) < 0.01

    def test_flat_with_full_confidence_is_safe(self):
        """Flat terrain with full confidence is highly traversable."""
        score = terrain_value_to_traversability(TERRAIN_FLAT, 1.0)
        assert score > 0.95

    def test_steep_with_full_confidence_is_unsafe(self):
        """Steep terrain with full confidence has low traversability."""
        score = terrain_value_to_traversability(TERRAIN_STEEP, 1.0)
        assert score < 0.2

    def test_low_confidence_pulls_toward_neutral(self):
        """Low confidence makes any terrain class approach neutral score."""
        flat_low_conf = terrain_value_to_traversability(TERRAIN_FLAT, 0.1)
        steep_low_conf = terrain_value_to_traversability(TERRAIN_STEEP, 0.1)

        # Both should be closer to 0.5 than their base scores
        assert abs(flat_low_conf - 0.5) < 0.3
        assert abs(steep_low_conf - 0.5) < 0.3

    def test_terrain_ordering_with_full_confidence(self):
        """Terrain classes ordered by safety with full confidence."""
        flat = terrain_value_to_traversability(TERRAIN_FLAT, 1.0)
        sloped = terrain_value_to_traversability(TERRAIN_SLOPED, 1.0)
        rough = terrain_value_to_traversability(TERRAIN_ROUGH, 1.0)
        steep = terrain_value_to_traversability(TERRAIN_STEEP, 1.0)

        assert flat > sloped > rough > steep


class TestTerrainCell(unittest.TestCase):
    """Test TerrainCell update and decay behavior."""

    def test_initial_cell_is_unknown(self):
        """New cell starts as unknown with zero confidence."""
        cell = TerrainCell()
        assert cell.terrain_class == TERRAIN_UNKNOWN
        assert cell.confidence == 0.0
        assert cell.update_count == 0

    def test_update_changes_class_and_confidence(self):
        """Update modifies terrain class and confidence."""
        cell = TerrainCell()
        cell.update(TERRAIN_FLAT, 0.8, 100.0)

        assert cell.terrain_class == TERRAIN_FLAT
        assert abs(cell.confidence - 0.8) < 0.01
        assert cell.last_updated == 100.0
        assert cell.update_count == 1

    def test_update_clamps_confidence_to_zero_one(self):
        """Confidence is clamped to [0, 1]."""
        cell = TerrainCell()
        cell.update(TERRAIN_FLAT, 1.5, 100.0)
        assert cell.confidence == 1.0

        cell.update(TERRAIN_FLAT, -0.5, 101.0)
        assert cell.confidence == 0.0

    def test_multiple_updates_increment_count(self):
        """Update count increments with each update."""
        cell = TerrainCell()
        cell.update(TERRAIN_FLAT, 0.8, 100.0)
        cell.update(TERRAIN_SLOPED, 0.7, 101.0)
        cell.update(TERRAIN_ROUGH, 0.6, 102.0)

        assert cell.update_count == 3
        assert cell.terrain_class == TERRAIN_ROUGH  # Last update

    def test_fresh_observation_has_full_confidence(self):
        """Recently updated cell returns full confidence."""
        cell = TerrainCell()
        cell.update(TERRAIN_FLAT, 0.9, 100.0)

        decayed = cell.get_decayed_confidence(100.1, half_life_sec=30.0)
        assert decayed > 0.88  # Minimal decay after 0.1s

    def test_confidence_decays_over_time(self):
        """Confidence decays exponentially with time."""
        cell = TerrainCell()
        cell.update(TERRAIN_FLAT, 1.0, 100.0)

        half_life = 30.0

        # After one half-life, confidence should be ~0.5
        decayed_half = cell.get_decayed_confidence(130.0, half_life)
        assert 0.45 < decayed_half < 0.55

        # After two half-lives, confidence should be ~0.25
        decayed_double = cell.get_decayed_confidence(160.0, half_life)
        assert 0.20 < decayed_double < 0.30

    def test_very_old_observations_return_zero_confidence(self):
        """Very stale observations (>5 half-lives) return zero."""
        cell = TerrainCell()
        cell.update(TERRAIN_FLAT, 1.0, 100.0)

        half_life = 30.0
        very_old = 100.0 + 6.0 * half_life  # 6 half-lives

        decayed = cell.get_decayed_confidence(very_old, half_life)
        assert decayed == 0.0

    def test_future_timestamp_treated_as_fresh(self):
        """Timestamp in future is treated as fresh (clock skew tolerance)."""
        cell = TerrainCell()
        cell.update(TERRAIN_FLAT, 0.9, 100.0)

        # Query with timestamp before update
        decayed = cell.get_decayed_confidence(99.0, half_life_sec=30.0)
        assert decayed == 0.9

    def test_zero_confidence_stays_zero(self):
        """Cell with zero confidence returns zero regardless of age."""
        cell = TerrainCell()
        cell.update(TERRAIN_UNKNOWN, 0.0, 100.0)

        decayed = cell.get_decayed_confidence(200.0, half_life_sec=30.0)
        assert decayed == 0.0


class TestTerrainMemoryIntegration(unittest.TestCase):
    """Integration tests for terrain memory behavior."""

    def test_new_safe_observation_overrides_old_unsafe(self):
        """Recent safe observation replaces old unsafe classification."""
        cell = TerrainCell()

        # Old unsafe observation
        cell.update(TERRAIN_STEEP, 0.9, 100.0)

        # New safe observation
        cell.update(TERRAIN_FLAT, 0.8, 150.0)

        # Should now be flat
        assert cell.terrain_class == TERRAIN_FLAT
        score = terrain_value_to_traversability(
            cell.terrain_class,
            cell.get_decayed_confidence(150.0, 30.0)
        )
        assert score > 0.7  # Safe terrain

    def test_stale_unsafe_decays_to_unknown(self):
        """Very old unsafe terrain becomes neutral due to decay."""
        cell = TerrainCell()
        cell.update(TERRAIN_STEEP, 1.0, 100.0)

        # After many half-lives
        current_time = 100.0 + 200.0  # ~6-7 half-lives with 30s half-life
        decayed_conf = cell.get_decayed_confidence(current_time, 30.0)

        score = terrain_value_to_traversability(cell.terrain_class, decayed_conf)

        # Should be close to neutral due to zero confidence
        assert abs(score - 0.5) < 0.1


if __name__ == '__main__':
    unittest.main()
