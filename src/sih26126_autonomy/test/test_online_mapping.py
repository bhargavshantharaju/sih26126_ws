import unittest

import numpy as np

from sih26126_autonomy.online_mapping import (
    mark_ray,
    mark_ray_with_evidence,
    pointcloud_rays,
    world_to_cell,
)


class OnlineMappingTests(unittest.TestCase):
    def test_world_to_cell_respects_bounds_and_resolution(self):
        self.assertEqual(world_to_cell(0.0, 0.0), (100, 150))
        self.assertIsNone(world_to_cell(-11.0, 0.0))

    def test_ray_marks_free_cells_and_occupied_endpoint(self):
        grid = np.full((8, 8), -1, dtype=np.int8)
        mark_ray(grid, (1, 1), (5, 1), True)
        self.assertTrue(np.all(grid[1, 1:5] == 0))
        self.assertEqual(grid[1, 5], 100)

    def test_repeated_free_returns_clear_stale_occupied_cells(self):
        grid = np.full((3, 5), -1, dtype=np.int8)
        evidence = np.zeros((3, 5), dtype=np.float32)
        start, end = (0, 1), (3, 1)
        mark_ray_with_evidence(grid, evidence, start, end, occupied=True)
        self.assertEqual(int(grid[1, 3]), 100)
        for _ in range(4):
            mark_ray_with_evidence(grid, evidence, start, end, occupied=False)
        self.assertEqual(int(grid[1, 3]), 0)

    def test_pointcloud_rays_use_real_observation_geometry(self):
        points = np.array([[2.0, 0.0, -0.7], [2.0, 0.0, -1.12],
                           [9.0, 0.0, -0.7], [2.0, 0.0, np.nan]])
        rays = list(pointcloud_rays(points, 0.0, 0.0, 0.0))
        self.assertEqual(len(rays), 2)
        self.assertTrue(rays[0][2])
        self.assertFalse(rays[1][2])


if __name__ == '__main__':
    unittest.main()
