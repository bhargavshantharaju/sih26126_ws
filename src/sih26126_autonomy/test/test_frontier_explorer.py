import unittest

import numpy as np

from sih26126_autonomy.frontier_explorer import (
    exploration_mode,
    cluster_frontiers,
    filter_frontiers_by_radius,
    find_best_connected_start,
    goal_completion_reason,
    merge_nearby_clusters,
    merge_frontier_array,
    find_frontiers,
    no_progress_timeout_reached,
    nearest_free,
    reconstruct_path,
    select_frontier,
    same_frontier_region,
    shortest_paths,
)


class FrontierExplorerTests(unittest.TestCase):
    def test_frontier_search_is_bounded_from_captured_home(self):
        frontiers = np.array([[100, 100], [100, 180], [100, 210]])
        bounded = filter_frontiers_by_radius(
            frontiers, -10.0, -10.0, 0.1, 0.0, 0.0, 8.0)
        self.assertEqual(bounded.tolist(), [[100, 100]])

    def test_merged_frontiers_keep_row_column_coordinate_order(self):
        raw = np.array([[108, 171], [108, 172], [109, 171]])
        merged, clusters = merge_frontier_array(raw, 1, 0.0)
        self.assertEqual(len(clusters), 1)
        self.assertEqual(set(map(tuple, merged.tolist())), set(map(tuple, raw.tolist())))

    def test_frontiers_cluster_into_stable_regions(self):
        """Connected edge cells yield one stable frontier cluster."""
        frontiers = np.array([[2, 2], [2, 3], [3, 2], [7, 7]])
        clusters = cluster_frontiers(frontiers, minimum_size=3)
        self.assertEqual(len(clusters), 1)
        self.assertEqual(set(clusters[0]), {(2, 2), (3, 2), (2, 3)})

    def test_frontier_minimum_cluster_size_rejects_isolated_cells(self):
        """Single noisy frontier cells are not offered as mission goals."""
        self.assertEqual(
            cluster_frontiers(np.array([[1, 1], [8, 8]]), minimum_size=2), [])

    def test_frontiers_are_known_free_cells_next_to_unknown(self):
        grid = np.full((7, 7), -1, dtype=np.int8)
        grid[2:5, 2:5] = 0
        frontiers = find_frontiers(grid)
        self.assertEqual(len(frontiers), 8)
        self.assertTrue(all(grid[row, col] == 0 for row, col in frontiers))

    def test_grid_planning_uses_only_connected_known_free_cells(self):
        grid = np.full((8, 8), -1, dtype=np.int8)
        grid[1, 1:5] = 0
        grid[2:5, 4] = 0
        start = nearest_free(grid, (1, 1))
        distances, parents = shortest_paths(grid, start)
        path = reconstruct_path(parents, (4, 4), grid.shape[1])
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], (4, 4))
        self.assertTrue(all(grid[row, col] == 0 for col, row in path))
        self.assertNotIn(6 * grid.shape[1] + 6, distances)

    def test_exploration_states_follow_map_and_reachability(self):
        self.assertEqual(exploration_mode(False, 0, 0), 'MAPPING')
        self.assertEqual(exploration_mode(True, 0, 0), 'NO_FRONTIER')
        self.assertEqual(exploration_mode(True, 4, 0), 'NO_FRONTIER')
        self.assertEqual(exploration_mode(True, 4, 2), 'EXPLORING')

    def test_frontier_selection_avoids_micro_goals_and_retains_target(self):
        grid = np.full((8, 16), -1, dtype=np.int8)
        grid[2:6, 2:13] = 0
        candidates = np.array([[3, 3], [3, 10]])
        distances = {
            3 * grid.shape[1] + 3: 0.4,
            3 * grid.shape[1] + 10: 2.0,
        }
        target, distance, reachable = select_frontier(
            grid, candidates, distances, None, (2, 3), 1.0)
        self.assertEqual(target, (10, 3))
        self.assertEqual(distance, 2.0)
        self.assertEqual(reachable, 2)

        # Once observed, a goal may stop being a frontier while still being
        # free and reachable; keep navigating to it instead of jumping away.
        retained, retained_distance, _ = select_frontier(
            grid, np.array([[3, 3]]), distances, target, (9, 3), 1.0)
        self.assertEqual(retained, target)
        self.assertEqual(retained_distance, 2.0)

        released, _, _ = select_frontier(
            grid, candidates,
            {3 * grid.shape[1] + 3: 0.4, 3 * grid.shape[1] + 10: 0.5},
            target, (10, 3), 1.0)
        self.assertIsNone(released)

    def test_frontier_blacklist_excludes_failed_goal(self):
        """A timed-out frontier representative cannot be selected again."""
        grid = np.full((10, 12), -1, dtype=np.int8)
        grid[1:9, 1:11] = 0
        candidates = np.array([[2, 2], [2, 8]])
        distances = {2 * grid.shape[1] + 2: 2.0,
                     2 * grid.shape[1] + 8: 3.0}
        target, _, _ = select_frontier(
            grid, candidates, distances, None, (1, 2), 1.0,
            blacklist={(2, 2)})
        self.assertEqual(target, (8, 2))

    def test_valid_active_frontier_has_goal_hysteresis(self):
        """A higher-scoring alternative cannot steal a still-valid active goal."""
        grid = np.full((8, 16), -1, dtype=np.int8)
        grid[2:6, 2:14] = 0
        candidates = np.array([[3, 3], [3, 11]])
        distances = {3 * grid.shape[1] + 3: 2.0,
                     3 * grid.shape[1] + 11: 2.5}
        target, _, _ = select_frontier(
            grid, candidates, distances, (3, 3), (2, 3), 1.0,
            min_goal_distance=0.1, reached_distance=0.0)
        self.assertEqual(target, (3, 3))

    def test_shifted_frontier_representative_keeps_same_goal_region(self):
        self.assertTrue(same_frontier_region((71, 108), (73, 104), 8.0))
        self.assertFalse(same_frontier_region((71, 108), (82, 104), 8.0))

    def test_nearby_frontier_patches_are_merged(self):
        """Adjacent map-edge fragments become one frontier target group."""
        clusters = [[(2, 2), (2, 3)], [(4, 2), (4, 3)]]
        merged = merge_nearby_clusters(clusters, minimum_separation_cells=3.0)
        self.assertEqual(len(merged), 1)
        self.assertEqual(len(merged[0]), 4)

    def test_blacklist_cooldown_suppresses_near_duplicate_goal(self):
        """Cooldown radius prevents reselecting a shifted representative nearby."""
        grid = np.full((10, 14), -1, dtype=np.int8)
        grid[1:9, 1:13] = 0
        candidates = np.array([[2, 3], [2, 9]])
        distances = {2 * grid.shape[1] + 3: 2.0,
                     2 * grid.shape[1] + 9: 3.0}
        target, _, _ = select_frontier(
            grid, candidates, distances, None, (1, 2), 1.0,
            blacklist={(3, 2)}, blacklist_radius_cells=2.0)
        self.assertEqual(target, (9, 2))

    def test_no_progress_timeout_is_explicit(self):
        self.assertFalse(no_progress_timeout_reached(24.9, 25.0))
        self.assertTrue(no_progress_timeout_reached(25.0, 25.0))

    def test_frontier_completion_requires_arrival_and_map_evidence(self):
        self.assertIsNone(goal_completion_reason(0.6, 0.45, 10, 1, False))
        self.assertIsNone(goal_completion_reason(0.2, 0.45, 0, 1, True))
        self.assertEqual(goal_completion_reason(0.2, 0.45, 2, 1, True),
                         'REACHED_WITH_NEW_MAP_CELLS')
        self.assertEqual(goal_completion_reason(0.2, 0.45, 0, 1, False),
                         'REACHED_FRONTIER_CLOSED_BY_MAP_UPDATE')

    # ------------------------------------------------------------------
    # Bootstrap safety regression tests (Points 1-3 of safety review)
    # ------------------------------------------------------------------

    def test_disconnected_component_not_used_as_path_origin(self):
        """find_best_connected_start must ONLY return a cell from the component
        containing initial_start_seed, never from a different (disconnected)
        component even if that other component is larger.

        This test creates two isolated free regions:
        - Robot's small island (cols 1-3, rows 1-3): 9 cells
        - Larger disconnected island (cols 6-9, rows 1-4): 16 cells

        The nearest_free cell from robot position (2,2) will be in the small
        island.  When find_best_connected_start is called with that seed and
        min_component_size=10, it should return None because the robot's
        actual reachable component (9 cells) is smaller than the threshold.

        It must NOT return a cell from the larger island just because it's
        geometrically nearby — that would violate connectivity safety.
        """
        grid = np.full((12, 12), -1, dtype=np.int8)
        # Robot's small island: 9 cells
        grid[1:4, 1:4] = 0
        # Larger disconnected island: 16 cells
        grid[1:5, 6:10] = 0

        robot_col, robot_row = 2, 2
        # Find the nearest free cell (will be in the small 9-cell component)
        initial_seed = nearest_free(grid, (robot_col, robot_row))
        self.assertIsNotNone(initial_seed)
        self.assertIn(initial_seed[0], range(1, 4),
                      'nearest_free should return a cell from the small island')

        # Call find_best_connected_start with the seed from the small component
        result = find_best_connected_start(
            grid, robot_col, robot_row,
            min_component_size=10,
            initial_start_seed=initial_seed)

        # The function should return None because the robot's actual component
        # (9 cells) is smaller than min_component_size=10.
        # It must NOT return a cell from the larger island.
        self.assertIsNone(
            result,
            'find_best_connected_start must return None when the initial seed\'s '
            'component is smaller than min_component_size, even if a larger '
            'disconnected component exists nearby')

    def test_find_best_connected_start_returns_cell_from_same_component(self):
        """find_best_connected_start must return a cell from the SAME component
        as initial_start_seed, and that cell should be closer to the robot's
        projected position when possible.
        """
        # Create a large L-shaped free region (20+ cells)
        grid = np.full((15, 15), -1, dtype=np.int8)
        grid[5, 3:12] = 0    # horizontal arm: cols 3-11, row 5 (9 cells)
        grid[6:13, 11] = 0   # vertical arm: col 11, rows 6-12 (7 cells)
        # Total: 16 cells in one connected component

        robot_col, robot_row = 10, 10  # Projected position near col 11, row 11

        # nearest_free from (10,10) will find a cell in the L-shape
        initial_seed = nearest_free(grid, (robot_col, robot_row))
        self.assertIsNotNone(initial_seed)

        # Call find_best_connected_start
        result = find_best_connected_start(
            grid, robot_col, robot_row,
            min_component_size=10,
            initial_start_seed=initial_seed)

        # Should return a cell from the same component, ideally closer to (10,10)
        self.assertIsNotNone(result)

        # Verify the result is actually free
        self.assertEqual(grid[result[1], result[0]], 0)

        # Verify the result is in the same component as initial_seed
        distances, _ = shortest_paths(grid, initial_seed)
        result_idx = result[1] * grid.shape[1] + result[0]
        self.assertIn(result_idx, distances,
                      'Result must be reachable from initial_seed')

    def test_find_best_connected_start_returns_none_for_small_component(self):
        """find_best_connected_start must return None when the initial seed's
        component is smaller than min_component_size, regardless of whether
        larger components exist elsewhere in the grid.
        """
        grid = np.full((20, 20), -1, dtype=np.int8)
        # Tiny 3-cell component
        grid[5, 5:8] = 0
        # Large 50-cell component far away
        grid[15:20, 10:20] = 0  # 5×10 = 50 cells

        robot_col, robot_row = 6, 5
        initial_seed = nearest_free(grid, (robot_col, robot_row))
        self.assertEqual(initial_seed, (6, 5))  # Should be in the tiny component

        result = find_best_connected_start(
            grid, robot_col, robot_row,
            min_component_size=10,
            initial_start_seed=initial_seed)

        self.assertIsNone(result,
                          'Must return None when seed component is too small')

    def test_find_best_connected_start_returns_none_for_invalid_seed(self):
        """find_best_connected_start must return None when initial_start_seed
        is None or invalid.
        """
        grid = np.full((10, 10), -1, dtype=np.int8)
        grid[3:7, 3:7] = 0  # 4×4 free region

        # Test with None seed
        result = find_best_connected_start(
            grid, 5, 5,
            min_component_size=10,
            initial_start_seed=None)
        self.assertIsNone(result)

        # Test with out-of-bounds seed
        result = find_best_connected_start(
            grid, 5, 5,
            min_component_size=10,
            initial_start_seed=(20, 20))
        self.assertIsNone(result)

        # Test with occupied cell as seed
        grid[5, 5] = 100  # Make one cell occupied
        result = find_best_connected_start(
            grid, 5, 5,
            min_component_size=10,
            initial_start_seed=(5, 5))
        self.assertIsNone(result)

    def test_unreachable_frontier_not_promoted_to_goal(self):
        """select_frontier must not return a frontier whose representative cell
        is absent from the Dijkstra distances dict, regardless of its Euclidean
        closeness to the reachable region.
        """
        # Free corridor: row 2, cols 1-5.  Robot start at (1,2).
        # Frontier at (8,2) is NOT free and therefore not reachable.
        grid = np.full((6, 12), -1, dtype=np.int8)
        grid[2, 1:6] = 0   # navigable strip

        start = (1, 2)
        distances, _ = shortest_paths(grid, start)

        # Confirm the frontier cell is genuinely not reachable
        frontier_cell = (8, 2)
        frontier_index = frontier_cell[1] * grid.shape[1] + frontier_cell[0]
        self.assertNotIn(frontier_index, distances)

        # Build a candidates array with only the unreachable cell
        candidates = np.array([[2, 8]])  # (row, col)
        target, dist, count = select_frontier(
            grid, candidates, distances, None, start, 0.05,
            min_goal_distance=0.0)

        self.assertIsNone(target,
                          'Unreachable frontier must not be promoted to a goal')
        self.assertEqual(count, 0)

    def test_valid_dijkstra_path_contains_only_free_cells(self):
        """A path produced by reconstruct_path must visit only grid==0 cells.
        This is the baseline safety invariant that the fixes preserve: valid
        connected paths are unaffected and remain occupancy-safe.
        """
        # L-shaped free region
        grid = np.full((10, 10), -1, dtype=np.int8)
        grid[1, 1:8] = 0   # horizontal arm
        grid[1:8, 7] = 0   # vertical arm

        start = nearest_free(grid, (1, 1))
        distances, parents = shortest_paths(grid, start)

        goal_cell = (7, 7)
        path = reconstruct_path(parents, goal_cell, grid.shape[1])

        self.assertGreater(len(path), 0, 'Valid goal must produce a non-empty path')
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal_cell)

        for col, row in path:
            self.assertEqual(
                grid[row, col], 0,
                f'Path cell ({col},{row}) is not free (value={grid[row, col]})')

    def test_select_frontier_returns_only_reachable_goals(self):
        """select_frontier must never return a frontier cell that is not present
        in the distances dict, even when many unreachable frontiers exist.
        The test mixes reachable and unreachable frontier cells to confirm only
        the reachable ones can be selected.
        """
        # Free region: rows 2-7, cols 2-7 (36 free cells)
        grid = np.full((12, 12), -1, dtype=np.int8)
        grid[2:8, 2:8] = 0

        start = (2, 2)
        distances, _ = shortest_paths(grid, start)

        # Reachable frontier at edge of free region touching unknown (col 8 = -1)
        # Add a thin ring of unknown outside the free area so (7,5) is a real frontier.
        # (7,5) is free and adjacent to unknown at col 8.
        reachable_frontier = (7, 5)
        # Unreachable frontier deep in unknown space
        unreachable_frontier = (10, 10)

        # Confirm reachable is in distances, unreachable is not
        r_idx = reachable_frontier[1] * grid.shape[1] + reachable_frontier[0]
        u_idx = unreachable_frontier[1] * grid.shape[1] + unreachable_frontier[0]
        self.assertIn(r_idx, distances)
        self.assertNotIn(u_idx, distances)

        candidates = np.array([[5, 7], [10, 10]])  # (row, col)
        target, dist, count = select_frontier(
            grid, candidates, distances, None, start, 0.05,
            min_goal_distance=0.0)

        # Must select only the reachable frontier
        self.assertIsNotNone(target)
        self.assertEqual(target, reachable_frontier,
                         'Only the Dijkstra-reachable frontier must be returned')
        t_idx = target[1] * grid.shape[1] + target[0]
        self.assertIn(t_idx, distances,
                      'Selected target must be present in Dijkstra distances')


    # ------------------------------------------------------------------
    # Additional connectivity regression tests required by hardening spec
    # ------------------------------------------------------------------

    def test_unknown_space_barrier_prevents_crossing_components(self):
        """Two free regions separated by unknown cells (-1) are disconnected.

        BFS in find_best_connected_start traverses only grid==0.  Unknown cells
        (-1) are barriers identical to occupied cells for the purpose of path
        planning.  The robot's reachable component must not include cells on the
        far side of an unknown-space barrier.
        """
        grid = np.full((12, 14), -1, dtype=np.int8)
        # Left component: 3x3 free region around (col 1-3, row 1-3) — 9 cells
        grid[1:4, 1:4] = 0
        # Unknown barrier: cols 4-5 are all -1 (default)
        # Right component: 3x4 free region (col 6-9, row 1-3) — 12 cells
        grid[1:4, 6:10] = 0

        robot_col, robot_row = 2, 2
        initial_seed = nearest_free(grid, (robot_col, robot_row))
        # The seed must be in the left component (cols 1-3)
        self.assertIsNotNone(initial_seed)
        self.assertLess(initial_seed[0], 4,
                        'nearest_free must return a cell from the left (robot) component')

        # With min_component_size=10, left (9 cells) is too small → None
        result = find_best_connected_start(
            grid, robot_col, robot_row,
            min_component_size=10,
            initial_start_seed=initial_seed)
        self.assertIsNone(
            result,
            'Unknown-space barrier: robot\'s 9-cell component < min_component_size=10 '
            'must return None, not a cell from the right component')

        # With min_component_size=5, left (9 cells) is large enough
        result2 = find_best_connected_start(
            grid, robot_col, robot_row,
            min_component_size=5,
            initial_start_seed=initial_seed)
        self.assertIsNotNone(result2)
        # The returned cell must be in the LEFT component (cols 1-3)
        self.assertLess(result2[0], 4,
                        'Result must be from the robot\'s left component, not the right one')
        # Cross-verify: the result is NOT reachable from the right component
        right_seed = nearest_free(grid, (8, 2))
        right_distances, _ = shortest_paths(grid, right_seed)
        result_idx = result2[1] * grid.shape[1] + result2[0]
        self.assertNotIn(result_idx, right_distances,
                         'Result from left component must not be reachable from right component')

    def test_occupied_barrier_prevents_crossing_components(self):
        """Two free regions separated by occupied cells (100) are disconnected.

        Occupied cells (value >= 50 by convention, 100 here) act as walls.
        Dijkstra and BFS only traverse grid==0.  A planner must not produce a
        path that crosses an occupied barrier to reach the far component.
        """
        grid = np.full((10, 16), -1, dtype=np.int8)
        # Left free region: cols 1-5, rows 1-8 — 40 cells
        grid[1:9, 1:6] = 0
        # Occupied wall: col 6 is fully occupied
        grid[:, 6] = 100
        # Right free region: cols 7-13, rows 1-8 — 56 cells
        grid[1:9, 7:14] = 0

        robot_col, robot_row = 3, 4
        initial_seed = nearest_free(grid, (robot_col, robot_row))
        self.assertIsNotNone(initial_seed)
        self.assertLess(initial_seed[0], 6,
                        'nearest_free must return a cell from the left component')

        # Both components are large enough; result must come from left
        result = find_best_connected_start(
            grid, robot_col, robot_row,
            min_component_size=10,
            initial_start_seed=initial_seed)
        self.assertIsNotNone(result)
        self.assertLess(result[0], 6,
                        'Result must be from the robot\'s left component (cols < 6)')

        # Confirm occupied wall is not traversable: no path crosses col 6
        distances, parents = shortest_paths(grid, initial_seed)
        for row in range(grid.shape[0]):
            idx = row * grid.shape[1] + 6
            self.assertNotIn(idx, distances,
                             f'Occupied column 6 at row {row} must not be in Dijkstra distances')

        # Right component cells must not be reachable from left
        right_sample = (10, 4)
        right_idx = right_sample[1] * grid.shape[1] + right_sample[0]
        self.assertNotIn(right_idx, distances,
                         'Right-component cell must not be reachable through occupied wall')

    def test_robot_cell_occupied_nearest_free_finds_adjacent(self):
        """If the robot's projected map cell is occupied or unknown, nearest_free
        must find the closest actual free cell, and find_best_connected_start must
        then work correctly from that seed.

        This exercises the documented safe local-start policy: the robot's footprint
        may cause its own cell to be marked occupied; planning must still originate
        from a valid free cell in the same component.
        """
        grid = np.full((10, 10), -1, dtype=np.int8)
        # Large free region: rows 2-8, cols 2-8 — 49 cells
        grid[2:9, 2:9] = 0
        # Robot's projected cell is occupied (simulates footprint inflation)
        grid[5, 5] = 100

        robot_col, robot_row = 5, 5
        # nearest_free should return an adjacent free cell, not (5,5)
        seed = nearest_free(grid, (robot_col, robot_row))
        self.assertIsNotNone(seed, 'Should find a free cell near the occupied robot cell')
        self.assertNotEqual(seed, (5, 5), 'nearest_free must not return the occupied cell')
        # The seed must be free
        self.assertEqual(grid[seed[1], seed[0]], 0)

        # find_best_connected_start must return a cell from the same large component
        result = find_best_connected_start(
            grid, robot_col, robot_row,
            min_component_size=30,
            initial_start_seed=seed)
        self.assertIsNotNone(result, 'Should find a start in the large free component')
        self.assertEqual(grid[result[1], result[0]], 0)

        # Verify result is reachable from seed via Dijkstra
        distances, _ = shortest_paths(grid, seed)
        result_idx = result[1] * grid.shape[1] + result[0]
        self.assertIn(result_idx, distances)

    def test_robot_cell_unknown_nearest_free_finds_adjacent(self):
        """If the robot's projected cell is unknown (-1), nearest_free returns a
        free cell nearby.  The cell found must actually be free (== 0).
        """
        grid = np.full((8, 8), -1, dtype=np.int8)
        # Free region rows 1-6, cols 1-6, but leave (3,3) as unknown
        grid[1:7, 1:7] = 0
        grid[3, 3] = -1  # robot cell is unknown

        robot_col, robot_row = 3, 3
        seed = nearest_free(grid, (robot_col, robot_row))
        self.assertIsNotNone(seed)
        # Must be a free cell, not the unknown one
        self.assertEqual(grid[seed[1], seed[0]], 0)
        # Must be adjacent to (3,3)
        self.assertLessEqual(abs(seed[0] - 3) + abs(seed[1] - 3), 2)

    def test_no_valid_reachable_start_produces_safe_waiting_outcome(self):
        """When the entire map is unknown (-1) or has no free cells, nearest_free
        returns None and find_best_connected_start returns None.  The caller in
        FrontierExplorer will then emit WAITING_FOR_KNOWN_FREE_SPACE.

        This test validates that the None-returns that trigger the safe-waiting
        path are produced correctly by the helper functions.
        """
        # Completely unknown map: no free cells at all
        grid_all_unknown = np.full((8, 8), -1, dtype=np.int8)
        seed = nearest_free(grid_all_unknown, (4, 4))
        self.assertIsNone(seed, 'All-unknown map must yield no free start seed')

        result = find_best_connected_start(
            grid_all_unknown, 4, 4,
            min_component_size=1,
            initial_start_seed=None)
        self.assertIsNone(result, 'None seed must produce None from find_best_connected_start')

        # Map with only occupied cells: no free cells
        grid_all_occupied = np.full((8, 8), 100, dtype=np.int8)
        seed2 = nearest_free(grid_all_occupied, (4, 4))
        self.assertIsNone(seed2, 'All-occupied map must yield no free start seed')

        # Map with a single free cell: component size 1, below any reasonable threshold
        grid_one_cell = np.full((8, 8), -1, dtype=np.int8)
        grid_one_cell[4, 4] = 0
        seed3 = nearest_free(grid_one_cell, (4, 4))
        self.assertIsNotNone(seed3)
        result3 = find_best_connected_start(
            grid_one_cell, 4, 4,
            min_component_size=10,
            initial_start_seed=seed3)
        self.assertIsNone(result3, 'Single-cell component must return None (too small)')

    def test_connected_frontier_is_selected_when_map_is_mature(self):
        """When the robot's reachable component is large enough, a valid connected
        frontier is selected and its representative is reachable via Dijkstra.

        This test confirms that the connectivity fixes do not break normal
        frontier selection when the map is mature.
        """
        # 10x10 free region as the robot's component
        grid = np.full((20, 20), -1, dtype=np.int8)
        grid[2:12, 2:12] = 0  # 100-cell connected free region

        robot_col, robot_row = 6, 6
        initial_seed = nearest_free(grid, (robot_col, robot_row))
        self.assertIsNotNone(initial_seed)

        # find_best_connected_start should return a cell from the component
        result = find_best_connected_start(
            grid, robot_col, robot_row,
            min_component_size=50,
            initial_start_seed=initial_seed)
        self.assertIsNotNone(result, 'Large component must yield a valid start')

        # Run Dijkstra from the start cell
        distances, parents = shortest_paths(grid, result)
        self.assertGreaterEqual(len(distances), 50, 'Dijkstra must reach the full component')

        # Frontiers exist: free cells adjacent to unknown at the boundary
        frontiers = find_frontiers(grid)
        self.assertGreater(len(frontiers), 0, 'Boundary of free region must have frontiers')

        # select_frontier must return a reachable frontier
        metric_distances = {idx: dist * 0.05 for idx, dist in distances.items()}
        target, dist_m, reachable_count = select_frontier(
            grid, frontiers, metric_distances, None, result, 0.05,
            min_goal_distance=0.0)
        self.assertIsNotNone(target, 'A reachable frontier must be selected')
        target_idx = target[1] * grid.shape[1] + target[0]
        self.assertIn(target_idx, distances,
                      'Selected frontier must be present in Dijkstra distances')


if __name__ == '__main__':
    unittest.main()
