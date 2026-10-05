import unittest

import numpy as np

from sih26126_autonomy.frontier_explorer import (
    exploration_mode,
    cluster_frontiers,
    filter_frontiers_by_radius,
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


if __name__ == '__main__':
    unittest.main()
