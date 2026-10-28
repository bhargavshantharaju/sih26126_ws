#!/usr/bin/env python3
"""
Enhanced frontier scoring with terrain cost and obstacle proximity.

Extends existing information gain scoring with:
- Terrain traversability cost from terrain memory
- Obstacle proximity penalty from tracked obstacles
"""

import math
from typing import List, Tuple, Optional


def terrain_score_to_cost_scalar(terrain_score: float) -> float:
    """
    Convert a scalar terrain traversability score to a cost value.

    Args:
        terrain_score: 0.0 (unsafe) .. 1.0 (safe)

    Returns:
        Cost 0.0 (safe path) .. 1.0 (impassable).
    """
    # Clamp and invert: high traversability = low cost
    score = max(0.0, min(1.0, terrain_score))
    return 1.0 - score


def compute_terrain_path_cost(
    path_cells: List[Tuple[int, int]],
    terrain_memory_scores: dict,
    resolution: float,
    default_score: float = 0.5
) -> float:
    """
    Compute total terrain traversability cost along a path.
    
    Args:
        path_cells: List of (col, row) grid cells forming the path
        terrain_memory_scores: Dict mapping cell -> terrain score [0,1]
        resolution: Grid resolution in meters
        default_score: Score for cells without terrain data
    
    Returns:
        Total cost in meters (higher = worse terrain)
    """
    if not path_cells:
        return 0.0
    
    total_cost = 0.0
    
    for cell in path_cells:
        # Get terrain score for this cell (0 = unsafe, 1 = safe)
        score = terrain_memory_scores.get(cell, default_score)
        
        # Convert to cost: lower score = higher cost
        # Safe terrain (score=1.0) adds minimal cost
        # Unsafe terrain (score=0.1) adds high cost
        # Formula: cost = resolution * (1.1 - score)
        # This makes unsafe terrain effectively "longer" in cost space
        cell_cost = resolution * (1.1 - score)
        total_cost += cell_cost
    
    return total_cost


def compute_obstacle_proximity_penalty(
    frontier_cell: Tuple[int, int],
    obstacles: List[Tuple[float, float]],  # (x, y) in world coordinates
    resolution: float,
    origin_x: float,
    origin_y: float,
    danger_radius_m: float = 2.0,
    max_penalty: float = 50.0
) -> float:
    """
    Compute penalty for dynamic obstacles near a frontier.
    
    Args:
        frontier_cell: (col, row) grid coordinates of frontier
        obstacles: List of obstacle positions in world frame
        resolution: Grid resolution in meters
        origin_x, origin_y: Grid origin in world frame
        danger_radius_m: Distance within which obstacles matter
        max_penalty: Maximum penalty value
    
    Returns:
        Penalty value (higher = worse, 0 = no nearby obstacles)
    """
    if not obstacles:
        return 0.0
    
    # Convert frontier cell to world coordinates
    frontier_x = frontier_cell[0] * resolution + origin_x
    frontier_y = frontier_cell[1] * resolution + origin_y
    
    min_distance = float('inf')
    
    for obs_x, obs_y in obstacles:
        distance = math.hypot(frontier_x - obs_x, frontier_y - obs_y)
        min_distance = min(min_distance, distance)
    
    if min_distance >= danger_radius_m:
        # Far enough - no penalty
        return 0.0
    
    # Linear penalty: closer = worse
    # At danger_radius: penalty = 0
    # At distance = 0: penalty = max_penalty
    penalty = max_penalty * (1.0 - min_distance / danger_radius_m)
    
    return max(0.0, penalty)


def score_frontier_with_terrain_and_obstacles(
    gain: int,
    distance: float,
    heading_change: float,
    terrain_cost: float,
    obstacle_penalty: float,
    gain_weight: float = 1.0,
    distance_weight: float = 1.0,
    heading_weight: float = 0.75,
    terrain_weight: float = 0.5,
    obstacle_weight: float = 2.0
) -> float:
    """
    Compute multi-criteria frontier score.
    
    Combines:
    - Information gain (higher = better)
    - Travel distance (lower = better)
    - Heading change (lower = better)
    - Terrain cost (lower = better)
    - Obstacle proximity (lower = better)
    
    Args:
        gain: Information gain (unknown cells revealed)
        distance: Travel distance in meters
        heading_change: Heading change in radians
        terrain_cost: Terrain traversability cost
        obstacle_penalty: Obstacle proximity penalty
        *_weight: Weighting factors for each criterion
    
    Returns:
        Combined score (higher = better frontier)
    """
    # Normalize distance and terrain to similar scale
    # Use max to avoid division by zero
    denominator = max(
        1.0,
        distance_weight * distance +
        heading_weight * heading_change +
        terrain_weight * terrain_cost +
        obstacle_weight * obstacle_penalty +
        1.0  # Baseline offset
    )
    
    # Information gain in numerator (higher = better)
    score = (gain_weight * gain) / denominator
    
    return score


def compute_path_cells_from_dijkstra(
    start_cell: Tuple[int, int],
    goal_cell: Tuple[int, int],
    distances: dict,
    grid: 'np.ndarray',
    max_path_length: int = 1000
) -> List[Tuple[int, int]]:
    """
    Reconstruct path cells from Dijkstra distances by gradient descent.
    
    Args:
        start_cell: (col, row) starting cell
        goal_cell: (col, row) goal cell
        distances: Dict mapping cell_index -> distance from start
        grid: Occupancy grid (for bounds checking)
        max_path_length: Safety limit on path length
    
    Returns:
        List of (col, row) cells from start to goal
    """
    height, width = grid.shape
    path = [goal_cell]
    current = goal_cell
    
    for _ in range(max_path_length):
        if current == start_cell:
            break
        
        col, row = current
        current_idx = row * width + col
        current_dist = distances.get(current_idx, float('inf'))
        
        # Check 4-connected neighbors for smallest distance
        neighbors = [
            (col + 1, row),  # right
            (col - 1, row),  # left
            (col, row + 1),  # down
            (col, row - 1),  # up
        ]
        
        best_neighbor = None
        best_dist = current_dist
        
        for nb_col, nb_row in neighbors:
            if not (0 <= nb_col < width and 0 <= nb_row < height):
                continue
            
            nb_idx = nb_row * width + nb_col
            nb_dist = distances.get(nb_idx, float('inf'))
            
            if nb_dist < best_dist:
                best_dist = nb_dist
                best_neighbor = (nb_col, nb_row)
        
        if best_neighbor is None:
            # Cannot find path - return what we have
            break
        
        path.append(best_neighbor)
        current = best_neighbor
    
    # Reverse to get start -> goal
    return list(reversed(path))
