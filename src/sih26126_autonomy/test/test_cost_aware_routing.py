"""Tests for cost-aware Dijkstra routing in frontier_explorer.

Tests verify:
  1. A path through a high-cost cell is longer (Dijkstra distance) than through
     a low-cost alternative.
  2. A cell with cost >= 99 (near-lethal) is never part of a route even when it
     would be the geometrically shortest path.
  3. Fallback to plain Dijkstra occurs when cost_map dimensions do not match.
  4. Unknown cost cells (-1) receive conservative multiplier 1.5 (not 0 or 1.0).
"""

import math
import sys
import os

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Ensure the package under test is importable without a full ROS 2 install.
# ---------------------------------------------------------------------------
_src = os.path.join(os.path.dirname(__file__), '..', 'sih26126_autonomy')
if _src not in sys.path:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sih26126_autonomy.frontier_explorer import (  # noqa: E402
    _cost_map_multiplier,
    shortest_paths,
    shortest_paths_with_cost,
)


# ---------------------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------------------

def _flat_grid(rows: int, cols: int, fill: int = 0):
    """Return a flat list suitable for cost_map_data (all cells = fill)."""
    return [fill] * (rows * cols)


def _make_grid(rows: int, cols: int, fill: int = 0) -> np.ndarray:
    """Return a (rows x cols) occupancy grid (all cells free = 0)."""
    return np.full((rows, cols), fill, dtype=np.int8)


# ---------------------------------------------------------------------------
# Unit tests for _cost_map_multiplier
# ---------------------------------------------------------------------------

class TestCostMapMultiplier:
    """Boundary and mid-range tests for the per-cell multiplier helper."""

    def test_zero_gives_multiplier_1_0(self):
        assert _cost_map_multiplier(0) == pytest.approx(1.0)

    def test_49_gives_multiplier_1_49(self):
        assert _cost_map_multiplier(49) == pytest.approx(1.0 + 49 / 100.0)

    def test_50_gives_multiplier_2_0(self):
        assert _cost_map_multiplier(50) == pytest.approx(2.0)

    def test_98_gives_multiplier_3_0(self):
        assert _cost_map_multiplier(98) == pytest.approx(2.0 + 48 / 48.0)

    def test_99_returns_none(self):
        assert _cost_map_multiplier(99) is None

    def test_100_returns_none(self):
        assert _cost_map_multiplier(100) is None

    def test_unknown_minus1_gives_1_5(self):
        assert _cost_map_multiplier(-1) == pytest.approx(1.5)

    def test_low_cost_range_monotone(self):
        """Multiplier should be monotonically increasing for 0..49."""
        prev = _cost_map_multiplier(0)
        for v in range(1, 50):
            cur = _cost_map_multiplier(v)
            assert cur >= prev
            prev = cur

    def test_medium_cost_range_monotone(self):
        """Multiplier should be monotonically increasing for 50..98."""
        prev = _cost_map_multiplier(50)
        for v in range(51, 99):
            cur = _cost_map_multiplier(v)
            assert cur >= prev
            prev = cur


# ---------------------------------------------------------------------------
# Test 1 – high-cost alternative path is longer
# ---------------------------------------------------------------------------

class TestHighCostPathIsLonger:
    """
    Verify that routing through a high-cost cell produces a greater Dijkstra
    distance than routing through a low-cost alternative of the same geometry.

    Layout (5-wide, 3-tall grid, all cells free in occupancy):

        S . H . .
        . . . . .
        . . L . .

    S  = start (0, 1)
    H  = high-cost cell (2, 0) — cost 80
    L  = low-cost cell  (2, 2) — cost 10
    G  = goal (4, 1)

    The geometrically shortest path must pass through either H or L (or
    diagonally around them).  We force the issue by routing separately:

      - cost_map_H: only the H cell has high cost, L is free (cost 0)
      - cost_map_L: only the L cell has high cost, H is free (cost 0)

    We then assert dist_to_goal with cost_map_H >= dist_to_goal with cost_map_L.
    """

    WIDTH = 5
    HEIGHT = 3
    START = (0, 1)   # (col, row)
    GOAL  = (4, 1)

    HIGH_COST = 80    # medium band: multiplier = 2.0 + 30/48 ≈ 2.625
    LOW_COST  = 10    # low band:    multiplier = 1.0 + 10/100 = 1.1

    def _occupancy(self):
        return _make_grid(self.HEIGHT, self.WIDTH, fill=0)

    def _cost_map(self, high_col, high_row, high_value):
        """Return a flat cost map with one elevated cell."""
        data = [0] * (self.WIDTH * self.HEIGHT)
        data[high_row * self.WIDTH + high_col] = high_value
        return data

    def test_path_via_high_cost_has_greater_distance(self):
        """
        Force the route through a specific cell by making all other mid-column
        cells occupied, so the only path at column 2 is through that cell.
        Then compare high-cost (value=80) vs low-cost (value=10) for the same
        forced cell.

        Layout (5-wide, 3-tall, row 0 and row 2 blocked at col 2):
            S . # . G    <- row 0: col 2 blocked (occupied)
            . . F . .    <- row 1: col 2 is the forced cell
            . . # . .    <- row 2: col 2 blocked (occupied)
        """
        W, H = self.WIDTH, self.HEIGHT
        grid_high = _make_grid(H, W, fill=0)
        grid_low  = _make_grid(H, W, fill=0)

        # Block col=2 in rows 0 and 2 so the only vertical pass is row 1
        grid_high[0, 2] = 100
        grid_high[2, 2] = 100
        grid_low[0, 2] = 100
        grid_low[2, 2] = 100

        # High cost at (col=2, row=1): value=80 → multiplier ≈ 2.625
        cost_high = [0] * (W * H)
        cost_high[1 * W + 2] = 80

        # Low cost at (col=2, row=1): value=10 → multiplier = 1.1
        cost_low = [0] * (W * H)
        cost_low[1 * W + 2] = 10

        start = self.START  # (0, 1)
        goal  = self.GOAL   # (4, 1)
        goal_id = goal[1] * W + goal[0]

        dist_high, _ = shortest_paths_with_cost(grid_high, cost_high, W, start)
        dist_low,  _ = shortest_paths_with_cost(grid_low,  cost_low,  W, start)

        assert goal_id in dist_high, "Goal unreachable in high-cost scenario"
        assert goal_id in dist_low,  "Goal unreachable in low-cost scenario"

        assert dist_high[goal_id] > dist_low[goal_id], (
            f"High-cost path ({dist_high[goal_id]:.3f}) should exceed "
            f"low-cost path ({dist_low[goal_id]:.3f})")

    def test_direct_comparison_high_vs_low_cost_same_corridor(self):
        """Direct comparison: same cell, high vs low cost → different distance."""
        grid = self._occupancy()
        # Force the route to go through cell (2, 1) by making all other paths
        # occupied — create a 3x1 narrow corridor: row 1 only
        narrow = np.full((self.HEIGHT, self.WIDTH), 100, dtype=np.int8)
        narrow[1, :] = 0  # only row 1 is free

        # Cost map A: (2,1) has high cost 80
        cost_high = [0] * (self.WIDTH * self.HEIGHT)
        cost_high[1 * self.WIDTH + 2] = 80

        # Cost map B: (2,1) has low cost 10
        cost_low = [0] * (self.WIDTH * self.HEIGHT)
        cost_low[1 * self.WIDTH + 2] = 10

        dist_high, _ = shortest_paths_with_cost(
            narrow, cost_high, self.WIDTH, self.START)
        dist_low, _ = shortest_paths_with_cost(
            narrow, cost_low, self.WIDTH, self.START)

        goal_id = self.GOAL[1] * self.WIDTH + self.GOAL[0]
        assert goal_id in dist_high, "Goal unreachable in high-cost narrow corridor"
        assert goal_id in dist_low, "Goal unreachable in low-cost narrow corridor"
        assert dist_high[goal_id] > dist_low[goal_id], (
            f"High cost path dist={dist_high[goal_id]:.3f} should exceed "
            f"low cost path dist={dist_low[goal_id]:.3f}")


# ---------------------------------------------------------------------------
# Test 2 – near-lethal cell (cost >= 99) is never part of a route
# ---------------------------------------------------------------------------

class TestNearLethalCellNotTraversed:
    """
    Verify that a cell with cost_map value >= 99 is NEVER included in any
    computed route, even when it is geometrically the only or shortest path.

    Layout (1x5 grid — single-row corridor through a lethal cell):

        S . X . G

    S = start (0,0), X = lethal cell (2,0), G = goal (4,0)

    All cells are free in the occupancy grid.  With X=99/100 in the cost map,
    the cost-aware Dijkstra should mark X as impassable, so G is unreachable.
    """

    WIDTH = 5
    HEIGHT = 1
    START = (0, 0)
    GOAL  = (4, 0)

    def _narrow_corridor(self):
        """Single-row all-free grid."""
        return _make_grid(self.HEIGHT, self.WIDTH, fill=0)

    def test_cost_99_blocks_corridor(self):
        grid = self._narrow_corridor()
        cost_map = [0] * (self.WIDTH * self.HEIGHT)
        cost_map[2] = 99  # col=2, row=0

        distances, _ = shortest_paths_with_cost(
            grid, cost_map, self.WIDTH, self.START)

        goal_id = self.GOAL[1] * self.WIDTH + self.GOAL[0]
        assert goal_id not in distances, (
            "Goal should be unreachable when lethal cell (cost=99) blocks "
            "the only path.")

    def test_cost_100_blocks_corridor(self):
        grid = self._narrow_corridor()
        cost_map = [0] * (self.WIDTH * self.HEIGHT)
        cost_map[2] = 100

        distances, _ = shortest_paths_with_cost(
            grid, cost_map, self.WIDTH, self.START)

        goal_id = self.GOAL[1] * self.WIDTH + self.GOAL[0]
        assert goal_id not in distances, (
            "Goal should be unreachable when lethal cell (cost=100) blocks "
            "the only path.")

    def test_lethal_cell_id_absent_from_distances(self):
        """The lethal cell itself must not appear in the distances dict."""
        grid = self._narrow_corridor()
        cost_map = [0] * (self.WIDTH * self.HEIGHT)
        cost_map[2] = 99

        distances, _ = shortest_paths_with_cost(
            grid, cost_map, self.WIDTH, self.START)

        lethal_id = 2  # col=2, row=0
        assert lethal_id not in distances, (
            "Lethal cell should not be added to the distances dict.")

    def test_alternate_route_bypasses_lethal(self):
        """When a bypass exists, the route avoids the lethal cell."""
        # 3-row grid with lethal in the middle row of column 2:
        #   row 0: S . . . G
        #   row 1: . . X . .   <- X is lethal
        #   row 2: . . . . .
        W, H = 5, 3
        grid = _make_grid(H, W, fill=0)
        cost_map = [0] * (W * H)
        cost_map[1 * W + 2] = 99  # row=1, col=2

        start = (0, 1)  # middle-left
        goal  = (4, 1)  # middle-right

        distances, _ = shortest_paths_with_cost(
            grid, cost_map, W, start)

        goal_id = 1 * W + 4
        lethal_id = 1 * W + 2

        assert goal_id in distances, (
            "Goal should be reachable via detour around lethal cell.")
        assert lethal_id not in distances, (
            "Lethal cell must not appear in distances even with bypass available.")


# ---------------------------------------------------------------------------
# Test 3 – fallback to plain Dijkstra on dimension mismatch
# ---------------------------------------------------------------------------

class TestFallbackOnDimensionMismatch:
    """
    Verify that shortest_paths_with_cost falls back to unweighted Dijkstra
    when the cost map dimensions do not match the occupancy grid.
    """

    def test_mismatch_width_falls_back(self):
        W, H = 5, 5
        grid = _make_grid(H, W, fill=0)
        start = (0, 0)

        # Cost map with wrong width (W+1 instead of W)
        wrong_width = W + 1
        cost_map_data = [50] * (wrong_width * H)  # all high cost

        warnings = []
        dist_fallback, par_fallback = shortest_paths_with_cost(
            grid, cost_map_data, wrong_width, start,
            _warn_fn=warnings.append)

        # Should equal plain Dijkstra
        dist_plain, par_plain = shortest_paths(grid, start)

        assert dist_fallback == dist_plain, (
            "Fallback distances should match plain Dijkstra on width mismatch.")
        assert par_fallback == par_plain, (
            "Fallback parents should match plain Dijkstra on width mismatch.")
        assert len(warnings) >= 1, (
            "A warning should be emitted on fallback.")

    def test_mismatch_height_falls_back(self):
        W, H = 5, 5
        grid = _make_grid(H, W, fill=0)
        start = (0, 0)

        # Cost map with wrong total size (H+1 rows)
        cost_map_data = [50] * (W * (H + 1))

        warnings = []
        dist_fallback, _ = shortest_paths_with_cost(
            grid, cost_map_data, W, start,
            _warn_fn=warnings.append)

        dist_plain, _ = shortest_paths(grid, start)

        assert dist_fallback == dist_plain, (
            "Fallback distances should match plain Dijkstra on total-size mismatch.")
        assert len(warnings) >= 1

    def test_none_cost_map_falls_back(self):
        W, H = 4, 4
        grid = _make_grid(H, W, fill=0)
        start = (0, 0)

        warnings = []
        dist_fallback, _ = shortest_paths_with_cost(
            grid, None, W, start,
            _warn_fn=warnings.append)

        dist_plain, _ = shortest_paths(grid, start)

        assert dist_fallback == dist_plain, (
            "Fallback distances should match plain Dijkstra when cost_map is None.")
        assert len(warnings) >= 1

    def test_empty_cost_map_falls_back(self):
        W, H = 4, 4
        grid = _make_grid(H, W, fill=0)
        start = (0, 0)

        warnings = []
        dist_fallback, _ = shortest_paths_with_cost(
            grid, [], W, start,
            _warn_fn=warnings.append)

        dist_plain, _ = shortest_paths(grid, start)

        assert dist_fallback == dist_plain, (
            "Fallback distances should match plain Dijkstra on empty cost map.")
        assert len(warnings) >= 1

    def test_matching_dimensions_do_not_fallback(self):
        """When dimensions match, no fallback warning should be emitted."""
        W, H = 5, 5
        grid = _make_grid(H, W, fill=0)
        start = (0, 0)
        cost_map_data = [0] * (W * H)

        warnings = []
        shortest_paths_with_cost(
            grid, cost_map_data, W, start,
            _warn_fn=warnings.append)

        assert len(warnings) == 0, (
            "No fallback warning should be emitted when dimensions match.")


# ---------------------------------------------------------------------------
# Test 4 – unknown cost cell (-1) uses conservative multiplier 1.5
# ---------------------------------------------------------------------------

class TestUnknownCostConservativeMultiplier:
    """
    Verify that a cost_map cell with value -1 (unknown) receives multiplier
    1.5 and NOT 0 or 1.0.
    """

    def test_multiplier_for_unknown_is_1_5(self):
        assert _cost_map_multiplier(-1) == pytest.approx(1.5)

    def test_unknown_cell_distance_between_free_and_medium(self):
        """
        Distance through unknown cell (mult=1.5) should be between distance
        through a free cell (mult=1.0) and a medium cost cell (mult=2.5+).
        """
        # Single-row 3-wide: start (0,0) -> mid (1,0) -> goal (2,0)
        W, H = 3, 1
        grid = _make_grid(H, W, fill=0)
        start = (0, 0)
        goal_id = 2  # col=2, row=0

        # 1. Free cost map (all 0): mid cell mult = 1.0
        cost_free = [0, 0, 0]
        dist_free, _ = shortest_paths_with_cost(grid, cost_free, W, start)

        # 2. Unknown cost map (-1 for mid cell): mid cell mult = 1.5
        cost_unknown = [0, -1, 0]
        dist_unknown, _ = shortest_paths_with_cost(grid, cost_unknown, W, start)

        # 3. Medium cost map (75 for mid cell): mid cell mult ≈ 2.0 + 25/48 ≈ 2.52
        cost_medium = [0, 75, 0]
        dist_medium, _ = shortest_paths_with_cost(grid, cost_medium, W, start)

        d_free    = dist_free[goal_id]
        d_unknown = dist_unknown[goal_id]
        d_medium  = dist_medium[goal_id]

        assert d_free < d_unknown, (
            f"Unknown cell ({d_unknown:.3f}) should cost more than free ({d_free:.3f})")
        assert d_unknown < d_medium, (
            f"Unknown cell ({d_unknown:.3f}) should cost less than medium ({d_medium:.3f})")

    def test_unknown_cell_not_blocked(self):
        """Unknown (-1) cells must remain traversable (not treated as lethal)."""
        W, H = 3, 1
        grid = _make_grid(H, W, fill=0)
        cost_unknown = [0, -1, 0]
        start = (0, 0)

        distances, _ = shortest_paths_with_cost(grid, cost_unknown, W, start)

        goal_id = 2
        assert goal_id in distances, (
            "Goal should be reachable through unknown-cost cell (not blocked).")

    def test_unknown_not_treated_as_zero_cost(self):
        """
        The distance through an unknown cell must be strictly greater than
        through a zero-cost cell, confirming multiplier != 1.0.
        """
        W, H = 3, 1
        grid = _make_grid(H, W, fill=0)
        start = (0, 0)
        goal_id = 2

        cost_zero    = [0, 0, 0]
        cost_unknown = [0, -1, 0]

        dist_zero,    _ = shortest_paths_with_cost(grid, cost_zero,    W, start)
        dist_unknown, _ = shortest_paths_with_cost(grid, cost_unknown, W, start)

        assert dist_unknown[goal_id] > dist_zero[goal_id], (
            "Unknown cell distance must be strictly greater than zero-cost "
            f"(got {dist_unknown[goal_id]:.3f} vs {dist_zero[goal_id]:.3f})")

    def test_unknown_not_treated_as_lethal(self):
        """Value -1 must NOT be treated like cost >= 99 (i.e., must not block)."""
        # Same geometry as the lethal test but with -1 instead of 99
        W, H = 5, 1
        grid = _make_grid(H, W, fill=0)
        cost_map = [0, 0, -1, 0, 0]  # mid cell unknown
        start = (0, 0)

        distances, _ = shortest_paths_with_cost(grid, cost_map, W, start)
        goal_id = 4

        assert goal_id in distances, (
            "Goal must be reachable; unknown cell (-1) is NOT lethal.")


# ---------------------------------------------------------------------------
# Test 5 – occupancy grid primacy: occupied cells stay blocked despite cost map
# ---------------------------------------------------------------------------

class TestOccupancyGridPrimacy:
    """
    Ensure the raw occupancy grid is the primary traversability authority.
    A cell that is occupied (non-zero) in the occupancy grid must NEVER be
    made traversable just because the cost map value is low/zero.
    """

    def test_occupied_cell_not_traversable_despite_low_cost(self):
        W, H = 5, 1
        grid = np.zeros((H, W), dtype=np.int8)
        grid[0, 2] = 100  # occupied in raw grid

        cost_map = [0, 0, 0, 0, 0]  # cost map says it's free
        start = (0, 0)

        distances, _ = shortest_paths_with_cost(grid, cost_map, W, start)
        goal_id = 4
        occupied_id = 2

        assert occupied_id not in distances, (
            "Occupied cell must not be traversable even if cost map is 0.")
        assert goal_id not in distances, (
            "Goal behind occupied cell should be unreachable.")


# ---------------------------------------------------------------------------
# Test 6 – zero cost map produces same topology as plain Dijkstra
# ---------------------------------------------------------------------------

class TestZeroCostMapEquivalence:
    """
    When the entire cost map is 0, all multipliers are 1.0, so the
    cost-aware Dijkstra should produce distances that are within a small
    tolerance of the plain Dijkstra distances (they are exactly equal for
    straight-line moves; diagonal moves share the same sqrt(2) base cost).
    """

    def test_all_zero_cost_map_matches_plain_dijkstra(self):
        W, H = 7, 7
        # Create a simple maze-like grid with some walls
        grid = np.zeros((H, W), dtype=np.int8)
        grid[2, 1:5] = 100   # horizontal wall
        grid[4, 2:6] = 100   # another horizontal wall

        cost_map = [0] * (W * H)
        start = (0, 0)

        dist_cost, _ = shortest_paths_with_cost(grid, cost_map, W, start)
        dist_plain, _ = shortest_paths(grid, start)

        assert set(dist_cost.keys()) == set(dist_plain.keys()), (
            "Reachable cell sets should match between cost-aware and plain Dijkstra.")

        for cell_id, d_plain in dist_plain.items():
            d_cost = dist_cost[cell_id]
            assert abs(d_cost - d_plain) < 1e-9, (
                f"Cell {cell_id}: cost-aware={d_cost:.6f} != plain={d_plain:.6f}")
