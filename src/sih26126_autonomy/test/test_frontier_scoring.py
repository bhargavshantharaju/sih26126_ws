#!/usr/bin/env python3
"""Tests for enhanced frontier scoring with terrain and obstacles."""

import math
import unittest

from sih26126_autonomy.frontier_scoring import (
    compute_terrain_path_cost,
    compute_obstacle_proximity_penalty,
    score_frontier_with_terrain_and_obstacles,
    compute_path_cells_from_dijkstra,
)
import numpy as np


class TestTerrainPathCost(unittest.TestCase):
    """Test terrain cost calculation along paths."""
    
    def test_empty_path_zero_cost(self):
        """Empty path has zero terrain cost."""
        cost = compute_terrain_path_cost([], {}, resolution=0.1)
        assert cost == 0.0
    
    def test_all_safe_terrain_minimal_cost(self):
        """Path through safe terrain has minimal cost."""
        path = [(0, 0), (1, 0), (2, 0)]
        scores = {(0, 0): 1.0, (1, 0): 1.0, (2, 0): 1.0}  # All safe
        
        cost = compute_terrain_path_cost(path, scores, resolution=0.1)
        
        # Each cell adds resolution * (1.1 - 1.0) = 0.1 * 0.1 = 0.01
        expected = 3 * 0.1 * 0.1
        assert abs(cost - expected) < 0.001
    
    def test_unsafe_terrain_high_cost(self):
        """Path through unsafe terrain has high cost."""
        path = [(0, 0), (1, 0), (2, 0)]
        scores = {(0, 0): 0.1, (1, 0): 0.1, (2, 0): 0.1}  # All unsafe
        
        cost = compute_terrain_path_cost(path, scores, resolution=0.1)
        
        # Each cell adds resolution * (1.1 - 0.1) = 0.1 * 1.0 = 0.1
        expected = 3 * 0.1 * 1.0
        assert abs(cost - expected) < 0.001
    
    def test_mixed_terrain_combined_cost(self):
        """Path through mixed terrain combines costs."""
        path = [(0, 0), (1, 0), (2, 0)]
        scores = {(0, 0): 1.0, (1, 0): 0.5, (2, 0): 0.1}
        
        cost = compute_terrain_path_cost(path, scores, resolution=0.1)
        
        # Cell 0: 0.1 * (1.1 - 1.0) = 0.01
        # Cell 1: 0.1 * (1.1 - 0.5) = 0.06
        # Cell 2: 0.1 * (1.1 - 0.1) = 0.10
        expected = 0.01 + 0.06 + 0.10
        assert abs(cost - expected) < 0.001
    
    def test_unknown_cells_use_default_score(self):
        """Cells without terrain data use default neutral score."""
        path = [(0, 0), (1, 0), (2, 0)]
        scores = {}  # No data
        
        cost = compute_terrain_path_cost(path, scores, resolution=0.1, default_score=0.5)
        
        # Each cell uses default 0.5: resolution * (1.1 - 0.5) = 0.1 * 0.6 = 0.06
        expected = 3 * 0.06
        assert abs(cost - expected) < 0.001


class TestObstacleProximityPenalty(unittest.TestCase):
    """Test obstacle proximity penalty calculation."""
    
    def test_no_obstacles_zero_penalty(self):
        """No obstacles means no penalty."""
        penalty = compute_obstacle_proximity_penalty(
            frontier_cell=(10, 10),
            obstacles=[],
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
            danger_radius_m=2.0
        )
        assert penalty == 0.0
    
    def test_far_obstacle_zero_penalty(self):
        """Obstacle beyond danger radius has no penalty."""
        # Frontier at (10, 10) cells = (1.0, 1.0) world
        # Obstacle at (5.0, 5.0) world = ~5.66m away
        penalty = compute_obstacle_proximity_penalty(
            frontier_cell=(10, 10),
            obstacles=[(5.0, 5.0)],
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
            danger_radius_m=2.0,
            max_penalty=50.0
        )
        assert penalty == 0.0
    
    def test_close_obstacle_high_penalty(self):
        """Obstacle very close to frontier has high penalty."""
        # Frontier at (10, 10) cells = (1.0, 1.0) world
        # Obstacle at (1.2, 1.0) world = 0.2m away (within 2m danger radius)
        penalty = compute_obstacle_proximity_penalty(
            frontier_cell=(10, 10),
            obstacles=[(1.2, 1.0)],
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
            danger_radius_m=2.0,
            max_penalty=50.0
        )
        
        # Distance = 0.2m, radius = 2.0m
        # Penalty = 50 * (1 - 0.2/2.0) = 50 * 0.9 = 45
        assert abs(penalty - 45.0) < 1.0
    
    def test_obstacle_at_frontier_maximum_penalty(self):
        """Obstacle exactly at frontier has maximum penalty."""
        # Frontier at (10, 10) cells = (1.0, 1.0) world
        # Obstacle at (1.0, 1.0) world = 0m away
        penalty = compute_obstacle_proximity_penalty(
            frontier_cell=(10, 10),
            obstacles=[(1.0, 1.0)],
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
            danger_radius_m=2.0,
            max_penalty=50.0
        )
        
        assert abs(penalty - 50.0) < 0.1
    
    def test_multiple_obstacles_uses_closest(self):
        """Multiple obstacles - only closest matters."""
        # Frontier at (10, 10) cells = (1.0, 1.0) world
        # One close (1.5m), one far (10m)
        penalty = compute_obstacle_proximity_penalty(
            frontier_cell=(10, 10),
            obstacles=[(2.5, 1.0), (10.0, 10.0)],
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
            danger_radius_m=2.0,
            max_penalty=50.0
        )
        
        # Closest is 1.5m away
        # Penalty = 50 * (1 - 1.5/2.0) = 50 * 0.25 = 12.5
        assert abs(penalty - 12.5) < 1.0


class TestMultiCriteriaScoring(unittest.TestCase):
    """Test combined frontier scoring."""
    
    def test_high_gain_low_distance_best_score(self):
        """High information gain + low distance = best score."""
        score1 = score_frontier_with_terrain_and_obstacles(
            gain=100, distance=2.0, heading_change=0.0,
            terrain_cost=0.0, obstacle_penalty=0.0
        )
        score2 = score_frontier_with_terrain_and_obstacles(
            gain=10, distance=20.0, heading_change=0.0,
            terrain_cost=0.0, obstacle_penalty=0.0
        )
        
        assert score1 > score2
    
    def test_bad_terrain_reduces_score(self):
        """High terrain cost reduces frontier score."""
        score_good_terrain = score_frontier_with_terrain_and_obstacles(
            gain=50, distance=5.0, heading_change=0.0,
            terrain_cost=0.5, obstacle_penalty=0.0
        )
        score_bad_terrain = score_frontier_with_terrain_and_obstacles(
            gain=50, distance=5.0, heading_change=0.0,
            terrain_cost=10.0, obstacle_penalty=0.0
        )
        
        assert score_good_terrain > score_bad_terrain
    
    def test_obstacle_penalty_reduces_score(self):
        """Obstacle proximity penalty reduces frontier score."""
        score_no_obstacles = score_frontier_with_terrain_and_obstacles(
            gain=50, distance=5.0, heading_change=0.0,
            terrain_cost=0.0, obstacle_penalty=0.0
        )
        score_with_obstacles = score_frontier_with_terrain_and_obstacles(
            gain=50, distance=5.0, heading_change=0.0,
            terrain_cost=0.0, obstacle_penalty=50.0
        )
        
        assert score_no_obstacles > score_with_obstacles
    
    def test_heading_change_reduces_score(self):
        """Large heading change reduces score."""
        score_straight = score_frontier_with_terrain_and_obstacles(
            gain=50, distance=5.0, heading_change=0.0,
            terrain_cost=0.0, obstacle_penalty=0.0
        )
        score_turn = score_frontier_with_terrain_and_obstacles(
            gain=50, distance=5.0, heading_change=math.pi,
            terrain_cost=0.0, obstacle_penalty=0.0
        )
        
        assert score_straight > score_turn
    
    def test_zero_division_protection(self):
        """Scoring handles zero distance without division by zero."""
        score = score_frontier_with_terrain_and_obstacles(
            gain=10, distance=0.0, heading_change=0.0,
            terrain_cost=0.0, obstacle_penalty=0.0
        )
        
        # Should not raise exception, should return finite score
        assert math.isfinite(score)
        assert score > 0.0


class TestPathReconstruction(unittest.TestCase):
    """Test path reconstruction from Dijkstra distances."""
    
    def test_straight_line_path(self):
        """Reconstruct simple straight-line path."""
        grid = np.zeros((10, 10), dtype=np.int8)
        
        # Distances along a horizontal line
        distances = {
            0: 0.0,   # (0, 0)
            1: 1.0,   # (1, 0)
            2: 2.0,   # (2, 0)
            3: 3.0,   # (3, 0)
        }
        
        path = compute_path_cells_from_dijkstra(
            start_cell=(0, 0),
            goal_cell=(3, 0),
            distances=distances,
            grid=grid
        )
        
        expected = [(0, 0), (1, 0), (2, 0), (3, 0)]
        assert path == expected
    
    def test_empty_distances_returns_single_cell(self):
        """No distance data returns just the goal cell."""
        grid = np.zeros((10, 10), dtype=np.int8)
        distances = {}
        
        path = compute_path_cells_from_dijkstra(
            start_cell=(0, 0),
            goal_cell=(5, 5),
            distances=distances,
            grid=grid
        )
        
        # Cannot reconstruct path - returns goal only
        assert len(path) == 1
        assert path[0] == (5, 5)
    
    def test_max_path_length_prevents_infinite_loop(self):
        """Path reconstruction respects max length limit."""
        grid = np.zeros((100, 100), dtype=np.int8)
        
        # Create distances but with potential cycle
        distances = {i: float(i) for i in range(10000)}
        
        path = compute_path_cells_from_dijkstra(
            start_cell=(0, 0),
            goal_cell=(50, 50),
            distances=distances,
            grid=grid,
            max_path_length=100
        )
        
        # Should terminate within max_path_length + 1 (includes start cell)
        assert len(path) <= 101


if __name__ == '__main__':
    unittest.main()
