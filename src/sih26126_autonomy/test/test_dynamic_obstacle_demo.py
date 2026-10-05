"""Focused tests for the deterministic obstacle path."""

import unittest

from sih26126_autonomy.dynamic_obstacle_demo import obstacle_y


class DynamicObstacleDemoTests(unittest.TestCase):
    def test_fixed_delay_speed_and_clearance_end_state(self):
        self.assertEqual(obstacle_y(0.0), -2.0)
        self.assertEqual(obstacle_y(7.99), -2.0)
        self.assertEqual(obstacle_y(8.0), -2.0)
        self.assertEqual(obstacle_y(12.0), -1.0)
        self.assertEqual(obstacle_y(16.0), 0.0)
        self.assertEqual(obstacle_y(22.0), 0.0)
        self.assertEqual(obstacle_y(32.0), 1.0)
        self.assertEqual(obstacle_y(40.0), 2.0)
