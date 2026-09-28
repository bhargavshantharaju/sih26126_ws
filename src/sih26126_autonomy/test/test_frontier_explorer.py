import unittest

import numpy as np

from sih26126_autonomy.frontier_explorer import (
    exploration_mode,
    find_frontiers,
    nearest_free,
    reconstruct_path,
    select_frontier,
    shortest_paths,
)


class FrontierExplorerTests(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
