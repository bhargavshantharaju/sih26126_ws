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
        """find_best_connected_start must not be used as a drop-in start
        replacement when it returns a cell in a physically disconnected
        component.  This test verifies the *utility function* itself by
        checking that Dijkstra from the component it selects does NOT reach
        the robot's actual island — confirming disconnectedness.  The fix in
        process() gates on len(distances) < 10 and returns early rather than
        substituting a cell from an unreachable component.
        """
        # Two isolated 3×3 free squares separated by a wall of occupied cells.
        # Robot is in the top-left square (col 1-3, row 1-3).
        # A larger 4×4 free square exists at col 6-9, row 1-4.
        grid = np.full((12, 12), -1, dtype=np.int8)
        # Robot's small island: 9 cells
        grid[1:4, 1:4] = 0
        # Larger disconnected island: 16 cells
        grid[1:5, 6:10] = 0

        robot_col, robot_row = 2, 2   # centre of small island
        alt_start = find_best_connected_start(grid, robot_col, robot_row,
                                              min_component_size=10)

        # find_best_connected_start should select a cell from the LARGER island
        # (since the small one is < min_component_size=10).
        self.assertIsNotNone(alt_start)
        self.assertIn(alt_start[0], range(6, 10),
                      'alt_start should be in the large disconnected island (cols 6-9)')

        # Dijkstra from the robot's start must NOT reach alt_start — they are
        # disconnected.  This is the key safety invariant: substituting alt_start
        # as the navigation start would imply a false route.
        robot_start = nearest_free(grid, (robot_col, robot_row))
        distances, _ = shortest_paths(grid, robot_start)
        alt_index = alt_start[1] * grid.shape[1] + alt_start[0]
        self.assertNotIn(alt_index, distances,
                         'Disconnected component must not be reachable from robot island')

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


if __name__ == '__main__':
    unittest.main()
