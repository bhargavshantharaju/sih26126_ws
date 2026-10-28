"""Frontier selection and shortest paths over observed free occupancy cells."""

import heapq
import json
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String

from sih26126_autonomy.frontier_scoring import (
    compute_obstacle_proximity_penalty,
    score_frontier_with_terrain_and_obstacles,
    terrain_score_to_cost_scalar,
)
from sih26126_msgs.msg import TrackedObstacleArray
from visualization_msgs.msg import Marker, MarkerArray


Cell = Tuple[int, int]
NEIGHBORS = ((-1, -1), (0, -1), (1, -1), (-1, 0),
             (1, 0), (-1, 1), (0, 1), (1, 1))


def find_frontiers(grid: np.ndarray) -> np.ndarray:
    """Find known-free cells adjacent to at least one unknown cell."""
    free = grid == 0
    unknown = grid == -1
    adjacent_unknown = np.zeros_like(unknown)
    adjacent_unknown[1:, :] |= unknown[:-1, :]
    adjacent_unknown[:-1, :] |= unknown[1:, :]
    adjacent_unknown[:, 1:] |= unknown[:, :-1]
    adjacent_unknown[:, :-1] |= unknown[:, 1:]
    adjacent_unknown[1:, 1:] |= unknown[:-1, :-1]
    adjacent_unknown[1:, :-1] |= unknown[:-1, 1:]
    adjacent_unknown[:-1, 1:] |= unknown[1:, :-1]
    adjacent_unknown[:-1, :-1] |= unknown[1:, 1:]
    return np.argwhere(free & adjacent_unknown)


def cluster_frontiers(frontiers: np.ndarray, minimum_size: int = 3):
    """Group touching frontier cells into deterministic 8-connected clusters."""
    cells = {(int(col), int(row)) for row, col in np.asarray(frontiers).reshape(-1, 2)}
    clusters = []
    while cells:
        seed = min(cells, key=lambda cell: (cell[1], cell[0]))
        cells.remove(seed)
        stack = [seed]
        cluster = [seed]
        while stack:
            col, row = stack.pop()
            adjacent = [(col + dx, row + dy) for dx, dy in NEIGHBORS]
            for cell in adjacent:
                if cell in cells:
                    cells.remove(cell)
                    cluster.append(cell)
                    stack.append(cell)
        if len(cluster) >= max(1, int(minimum_size)):
            cluster.sort(key=lambda cell: (cell[1], cell[0]))
            clusters.append(cluster)
    return clusters


def merge_nearby_clusters(clusters, minimum_separation_cells: float):
    """Merge nearby frontier patches so edge noise cannot create duplicate goals."""
    groups = [list(cluster) for cluster in clusters]
    if minimum_separation_cells <= 0.0:
        return groups
    changed = True
    while changed:
        changed = False
        merged = []
        while groups:
            group = groups.pop(0)
            index = 0
            while index < len(groups):
                other = groups[index]
                center = (sum(c[0] for c in group) / len(group),
                          sum(c[1] for c in group) / len(group))
                other_center = (sum(c[0] for c in other) / len(other),
                                sum(c[1] for c in other) / len(other))
                if math.dist(center, other_center) <= minimum_separation_cells:
                    group.extend(other)
                    groups.pop(index)
                    changed = True
                else:
                    index += 1
            merged.append(sorted(set(group), key=lambda c: (c[1], c[0])))
        groups = merged
    return groups


def merge_frontier_array(frontiers: np.ndarray, minimum_size: int,
                         duplicate_radius_cells: float):
    """Return merged clusters and their cells in stable (row, column) order."""
    clusters = merge_nearby_clusters(
        cluster_frontiers(frontiers, minimum_size), duplicate_radius_cells)
    merged = np.asarray(
        [(row, col) for cluster in clusters for col, row in cluster],
        dtype=np.int64,
    ).reshape(-1, 2)
    return merged, clusters


def is_near_frontier(frontiers: np.ndarray, target: Cell, radius_cells: float) -> bool:
    """Return whether a frontier remains near the active target."""
    if not len(frontiers):
        return False
    rows, cols = np.asarray(frontiers).reshape(-1, 2).T
    return bool(np.any((cols - target[0]) ** 2 + (rows - target[1]) ** 2
                       <= radius_cells ** 2))


def filter_frontiers_by_radius(frontiers: np.ndarray, origin_x: float,
                               origin_y: float, resolution: float,
                               home_x: float, home_y: float,
                               radius_m: float) -> np.ndarray:
    """Keep only observed frontier cells inside the configured home region."""
    frontiers = np.asarray(frontiers, dtype=np.int64).reshape(-1, 2)
    if not math.isfinite(radius_m) or radius_m <= 0.0:
        return frontiers
    if not len(frontiers):
        return frontiers
    rows, cols = frontiers[:, 0], frontiers[:, 1]
    xs = origin_x + (cols + 0.5) * resolution
    ys = origin_y + (rows + 0.5) * resolution
    keep = ((xs - home_x) ** 2 + (ys - home_y) ** 2) <= radius_m ** 2
    return frontiers[keep]


def goal_completion_reason(distance_m: float, tolerance_m: float,
                           known_delta: int, min_new_cells: int,
                           frontier_still_exists: bool) -> Optional[str]:
    """Require arrival plus measurable map progress or frontier closure."""
    if distance_m > tolerance_m:
        return None
    if known_delta >= min_new_cells:
        return 'REACHED_WITH_NEW_MAP_CELLS'
    if not frontier_still_exists:
        return 'REACHED_FRONTIER_CLOSED_BY_MAP_UPDATE'
    return None


def no_progress_timeout_reached(elapsed_s: float, timeout_s: float) -> bool:
    """Return true only after configured elapsed time without route progress."""
    return timeout_s >= 0.0 and elapsed_s >= timeout_s


def frontier_goal_should_switch(active_valid: bool, reached: bool = False,
                                stalled: bool = False,
                                mission_changed: bool = False,
                                safety_replan: bool = False,
                                map_invalidated: bool = False) -> bool:
    """Keep a valid goal stable unless a documented lifecycle event releases it."""
    return (not active_valid or reached or stalled or mission_changed
            or safety_replan or map_invalidated)


def same_frontier_region(anchor: Optional[Cell], candidate: Cell,
                         hysteresis_cells: float) -> bool:
    """Treat nearby edge representatives as the same active goal region."""
    return (anchor is not None
            and math.dist(anchor, candidate) <= max(0.0, hysteresis_cells))


def nearest_free(grid: np.ndarray, start: Cell, radius: int = 15) -> Optional[Cell]:
    """Find the closest known-free cell near the robot's map cell."""
    col, row = start
    if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1] and grid[row, col] == 0:
        return start
    y0, y1 = max(0, row - radius), min(grid.shape[0], row + radius + 1)
    x0, x1 = max(0, col - radius), min(grid.shape[1], col + radius + 1)
    candidates = np.argwhere(grid[y0:y1, x0:x1] == 0)
    if not len(candidates):
        return None
    candidates[:, 0] += y0
    candidates[:, 1] += x0
    idx = np.argmin((candidates[:, 1] - col) ** 2 + (candidates[:, 0] - row) ** 2)
    return int(candidates[idx, 1]), int(candidates[idx, 0])


def find_best_connected_start(grid: np.ndarray, robot_col: int, robot_row: int,
                               min_component_size: int = 50,
                               initial_start_seed: Optional[Cell] = None) -> Optional[Cell]:
    """Find a better start cell within the robot's actual reachable component.

    SLAM's initial map often has isolated single-cell free islands separated
    by unknown space along individual laser beams.  The nearest free cell to
    the robot may belong to one of these tiny islands, causing Dijkstra to
    reach fewer than ``min_component_size`` cells and report no reachable
    frontiers.

    This function BFS-enumerates all 8-connected free-space components to
    identify which component contains the ``initial_start_seed`` (the cell
    returned by ``nearest_free``).  If that component has at least
    ``min_component_size`` cells, this function returns the cell within that
    SAME component that is nearest to the robot's projected map position,
    which may improve Dijkstra's reachability compared to the arbitrary
    nearest-free cell.

    **Critical safety constraint**: This function ONLY returns a cell from the
    component containing ``initial_start_seed``.  It will NEVER return a cell
    from a different component, even if that other component is larger or
    geometrically closer, because such a component would be physically
    unreachable from the robot's current position (separated by walls or
    unexplored unknown space).

    If ``initial_start_seed`` is None, or if its component is smaller than
    ``min_component_size``, this function returns None and the caller should
    wait for the map to mature further.
    """
    height, width = grid.shape

    # If no initial seed is provided, we cannot determine which component is
    # reachable from the robot. Fail safely.
    if initial_start_seed is None:
        return None

    # Verify the initial seed is actually a free cell
    seed_col, seed_row = initial_start_seed
    if not (0 <= seed_row < height and 0 <= seed_col < width):
        return None
    if grid[seed_row, seed_col] != 0:
        return None

    # BFS from the initial seed to enumerate its entire connected component.
    # This is the ONLY component from which we will consider returning a cell.
    robot_component: list = []
    visited: set = set()
    queue = [initial_start_seed]
    visited.add(initial_start_seed)

    while queue:
        col, row = queue.pop(0)
        robot_component.append((col, row))
        for dx, dy in NEIGHBORS:
            nx, ny = col + dx, row + dy
            if ((nx, ny) not in visited
                    and 0 <= ny < height
                    and 0 <= nx < width
                    and grid[ny, nx] == 0):
                visited.add((nx, ny))
                queue.append((nx, ny))

    # If the robot's reachable component is too small, return None.
    # Do NOT search for a different, larger component.
    if len(robot_component) < min_component_size:
        return None

    # The robot's component is large enough. Find the cell within this
    # SAME component that is nearest to the robot's projected map position.
    # This may yield a better Dijkstra start than the arbitrary nearest-free
    # cell, while maintaining the connectivity guarantee.
    cols = np.array([cell[0] for cell in robot_component])
    rows = np.array([cell[1] for cell in robot_component])
    dists_sq = (cols - robot_col) ** 2 + (rows - robot_row) ** 2
    nearest_idx = int(np.argmin(dists_sq))

    return (int(cols[nearest_idx]), int(rows[nearest_idx]))


def shortest_paths(grid: np.ndarray, start: Cell):
    """Run Dijkstra over known-free cells and return distances and parents."""
    width, height = grid.shape[1], grid.shape[0]
    start_id = start[1] * width + start[0]
    distances: Dict[int, float] = {start_id: 0.0}
    parents: Dict[int, Optional[int]] = {start_id: None}
    queue = [(0.0, start_id)]
    while queue:
        dist, idx = heapq.heappop(queue)
        if dist != distances.get(idx):
            continue
        col, row = idx % width, idx // width
        for dx, dy in NEIGHBORS:
            nx, ny = col + dx, row + dy
            if not (0 <= nx < width and 0 <= ny < height) or grid[ny, nx] != 0:
                continue
            if dx and dy and (grid[row, nx] != 0 or grid[ny, col] != 0):
                continue
            nxt = ny * width + nx
            cost = math.sqrt(2.0) if dx and dy else 1.0
            candidate = dist + cost
            if candidate < distances.get(nxt, math.inf):
                distances[nxt] = candidate
                parents[nxt] = idx
                heapq.heappush(queue, (candidate, nxt))
    return distances, parents


def _cost_map_multiplier(cost_value: int) -> Optional[float]:
    """Return the edge-weight multiplier for a cost map cell value.

    Returns None if the cell is near-lethal and should not be traversed.

    Cost tiers:
      0-49  (free, low cost)    : multiplier = 1.0 + value/100.0  (1.0 to 1.49)
      50-98 (medium cost)       : multiplier = 2.0 + (value-50)/48.0  (2.0 to 3.0)
      99-100 (near-lethal)      : None (do not traverse)
      -1   (unknown in cost map): multiplier = 1.5 (conservative)
    """
    if cost_value == -1:
        return 1.5
    if cost_value < 0 or cost_value > 100:
        # Unexpected value — treat as unknown
        return 1.5
    if cost_value >= 99:
        return None  # near-lethal: not traversable
    if cost_value <= 49:
        return 1.0 + cost_value / 100.0
    # 50-98
    return 2.0 + (cost_value - 50) / 48.0


def shortest_paths_with_cost(grid: np.ndarray,
                             cost_map_data,
                             cost_map_width: int,
                             start: Cell,
                             _warn_fn=None):
    """Run Dijkstra over known-free cells with cost-map-weighted edges.

    Parameters
    ----------
    grid:
        Raw occupancy grid (np.int8 array, shape [height, width]).
        Only cells with value 0 are traversable — this grid is the primary
        traversability authority.
    cost_map_data:
        Flat sequence of cost map cell values (0-100 or -1 for unknown),
        or None / empty to trigger fallback.
    cost_map_width:
        Width of the cost map grid (must equal grid.shape[1] for fusion).
    start:
        (col, row) start cell.
    _warn_fn:
        Optional callable(str) used to emit throttled warnings (e.g.
        node.get_logger().warning).  If None, warnings are silenced.

    Returns
    -------
    distances, parents  — same semantics as shortest_paths().
    """
    width, height = grid.shape[1], grid.shape[0]

    # Validate cost map: must exist, be non-empty, and match grid dimensions.
    use_cost_map = (cost_map_data is not None
                    and len(cost_map_data) > 0
                    and cost_map_width == width
                    and len(cost_map_data) == width * height)
    if not use_cost_map:
        if _warn_fn is not None:
            _warn_fn(
                'shortest_paths_with_cost: cost_map unavailable or dimension mismatch '
                f'(cost_map_width={cost_map_width} vs grid_width={width}, '
                f'cost_map_len={len(cost_map_data) if cost_map_data is not None else 0} '
                f'vs expected={width * height}); falling back to unweighted Dijkstra.')
        return shortest_paths(grid, start)

    start_id = start[1] * width + start[0]
    distances: Dict[int, float] = {start_id: 0.0}
    parents: Dict[int, Optional[int]] = {start_id: None}
    queue = [(0.0, start_id)]
    while queue:
        dist, idx = heapq.heappop(queue)
        if dist != distances.get(idx):
            continue
        col, row = idx % width, idx // width
        for dx, dy in NEIGHBORS:
            nx, ny = col + dx, row + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            # Primary traversability check: raw occupancy grid must be free (0).
            # The cost map cannot make an occupied cell traversable.
            if grid[ny, nx] != 0:
                continue
            # Corner-cutting check: same as plain Dijkstra
            if dx and dy and (grid[row, nx] != 0 or grid[ny, col] != 0):
                continue
            # Consult the cost map for the neighbour cell
            cost_val = int(cost_map_data[ny * width + nx])
            multiplier = _cost_map_multiplier(cost_val)
            if multiplier is None:
                # Near-lethal in cost map: skip regardless of occupancy grid
                continue
            nxt = ny * width + nx
            base_cost = math.sqrt(2.0) if dx and dy else 1.0
            cost = base_cost * multiplier
            candidate = dist + cost
            if candidate < distances.get(nxt, math.inf):
                distances[nxt] = candidate
                parents[nxt] = idx
                heapq.heappush(queue, (candidate, nxt))
    return distances, parents


def reconstruct_path(parents: Dict[int, Optional[int]], goal: Cell,
                     width: int) -> List[Cell]:
    """Reconstruct a start-to-goal cell path from Dijkstra parents."""
    idx = goal[1] * width + goal[0]
    if idx not in parents:
        return []
    path = []
    while idx is not None:
        path.append((idx % width, idx // width))
        idx = parents[idx]
    path.reverse()
    return path


def exploration_mode(map_known: bool, frontier_count: int,
                     reachable_frontier_count: int) -> str:
    """Classify actual map/frontier state for the autonomy status topic."""
    if not map_known:
        return 'MAPPING'
    if frontier_count <= 0 or reachable_frontier_count <= 0:
        return 'NO_FRONTIER'
    return 'EXPLORING'


def select_frontier(grid: np.ndarray, frontiers: np.ndarray,
                    distances: Dict[int, float], active: Optional[Cell],
                    start: Cell, resolution: float,
                    min_goal_distance: float = 1.5,
                    reached_distance: float = 0.8,
                    minimum_cluster_size: int = 1,
                    current_yaw: float = 0.0,
                    blacklist=None,
                    blacklist_radius_cells: float = 0.0,
                    duplicate_radius_cells: float = 0.0,
                    active_recovery_radius_cells: int = 3,
                    terrain_score: float = 0.5,
                    obstacle_positions: Optional[List[Tuple[float, float]]] = None,
                    map_origin_x: float = 0.0,
                    map_origin_y: float = 0.0) -> Tuple[Optional[Cell], float, int]:
    """Select an informative reachable frontier cluster with goal hysteresis.

    Only frontiers whose representative cell is present in the Dijkstra
    ``distances`` dict (i.e. reachable from ``start`` through contiguous
    known-free cells) are considered.  Frontiers that are unreachable by
    Dijkstra are silently skipped regardless of their Euclidean proximity to
    the reachable region; Euclidean closeness does not imply traversability.

    When ``terrain_score`` and/or ``obstacle_positions`` are provided the
    enhanced multi-criteria scoring from ``frontier_scoring`` is applied.
    Terrain cost and obstacle proximity penalty are incorporated so that
    high-cost or obstacle-adjacent frontiers are ranked lower.
    """
    height, width = grid.shape
    reachable = []
    clusters = merge_nearby_clusters(
        cluster_frontiers(frontiers, minimum_cluster_size), duplicate_radius_cells)

    # Pre-compute terrain cost for the scalar terrain score (broadcast to all cells).
    # Per-cell spatial terrain map integration is a future improvement; here we use
    # the current-position terrain score as a conservative uniform penalty.
    obs_list = obstacle_positions or []

    for cluster in clusters:
        center_col = sum(cell[0] for cell in cluster) / len(cluster)
        center_row = sum(cell[1] for cell in cluster) / len(cluster)
        cell = min(cluster, key=lambda point: (
            (point[0] - center_col) ** 2 + (point[1] - center_row) ** 2,
            point[1], point[0]))
        col, row = cell
        if blacklist and any(math.dist(cell, blocked) <= blacklist_radius_cells
                             for blocked in blacklist):
            continue
        index = row * width + col

        # Standard reachability check: frontier must be in Dijkstra distances
        if index in distances:
            distance = float(distances[index])
            gain = 0
            for cluster_col, cluster_row in cluster:
                neighborhood = grid[max(0, cluster_row - 2):min(height, cluster_row + 3),
                                    max(0, cluster_col - 2):min(width, cluster_col + 3)]
                gain += int(np.count_nonzero(neighborhood == -1))
            desired_heading = math.atan2(row - start[1], col - start[0])
            heading_change = abs(math.atan2(
                math.sin(desired_heading - current_yaw),
                math.cos(desired_heading - current_yaw)))
            # Enhanced scoring: incorporate terrain cost and obstacle proximity
            # when data is available; fall back to legacy formula otherwise.
            terrain_cost = terrain_score_to_cost_scalar(terrain_score)
            obstacle_penalty = compute_obstacle_proximity_penalty(
                cell, obs_list, resolution, map_origin_x, map_origin_y)
            score = score_frontier_with_terrain_and_obstacles(
                gain=gain,
                distance=distance,
                heading_change=heading_change,
                terrain_cost=terrain_cost,
                obstacle_penalty=obstacle_penalty,
            )
            reachable.append((score, gain, distance, cell))

    if active is not None:
        active_distance = math.hypot(active[0] - start[0], active[1] - start[1]) * resolution
        in_bounds = (0 <= active[0] < width and 0 <= active[1] < height)
        retained = active if in_bounds and grid[active[1], active[0]] == 0 else None
        if retained is None and in_bounds:
            retained = nearest_free(grid, active, radius=max(1, active_recovery_radius_cells))
        active_index = None if retained is None else retained[1] * width + retained[0]
        if (retained is not None and active_index in distances
                and active_distance >= reached_distance):
            return retained, float(distances[active_index]), len(reachable)

    eligible = [item for item in reachable
                if item[2] >= min_goal_distance]

    if not eligible:
        return None, 0.0, len(reachable)

    _, _, distance, target = max(eligible, key=lambda item: (item[0], item[1], -item[2]))
    return target, float(distance), len(reachable)


class FrontierExplorer(Node):
    """Select reachable frontiers and publish an occupancy-grid global path."""

    def __init__(self):
        super().__init__('frontier_explorer')
        self.map: Optional[OccupancyGrid] = None
        self.pose: Optional[PoseStamped] = None
        self.active: Optional[Cell] = None
        self.active_anchor: Optional[Cell] = None
        self.previous_state = ''
        self.previous_goal = None
        self.mission_state = ''
        self.mission_goal = None
        self.home_pose = None
        self.active_goal_started = None
        self.active_goal_best_distance = math.inf
        self.last_progress_time = None
        self.active_known_count = 0
        self.last_goal_switch_time = 0.0
        self.frontiers_completed = 0
        self.goal_event_id = 0
        self.blacklist = {}
        self.goal_change_reason = 'INITIAL'
        self.map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, self.map_qos)
        # Fused cost map: cells 0-100 representing occupancy + terrain + localization cost.
        # Subscribed with the same QoS as /map (TRANSIENT_LOCAL, RELIABLE, depth=1).
        self.cost_map: Optional[OccupancyGrid] = None
        self.create_subscription(
            OccupancyGrid, '/map/cost_map', self._cost_map_callback, self.map_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/localization/pose', self.pose_callback, 10)
        self.mission_state_sub = self.create_subscription(
            String, '/mission/state', self.mission_state_callback, 10)
        self.mission_goal_sub = self.create_subscription(
            PointStamped, '/mission/goal', self.mission_goal_callback, 10)
        self.home_pose_sub = self.create_subscription(
            PoseStamped, '/mission/home_pose', self.home_pose_callback, 10)
        # Terrain memory score from terrain_memory node — used to penalise
        # frontiers over high-cost terrain in the enhanced scoring formula.
        self.terrain_memory_score: float = 0.5  # neutral default until data arrives
        self.create_subscription(
            Float32, '/terrain/memory_score', self._terrain_score_cb, 10)
        # Tracked obstacles from obstacle_tracker — used to penalise frontiers
        # near dynamic obstacles in the enhanced scoring formula.
        self.tracked_obstacles: Optional[TrackedObstacleArray] = None
        self.create_subscription(
            TrackedObstacleArray, '/obstacles/tracked', self._obstacles_cb, 10)
        self.state_pub = self.create_publisher(String, '/exploration/state', 10)
        self.frontier_count_pub = self.create_publisher(Int32, '/exploration/frontier_count', 10)
        self.explored_pub = self.create_publisher(Float32, '/exploration/explored_percentage', 10)
        self.distance_pub = self.create_publisher(Float32, '/exploration/distance_to_frontier', 10)
        self.selected_id_pub = self.create_publisher(Int32, '/exploration/selected_frontier_id', 10)
        self.frontier_score_pub = self.create_publisher(Float32, '/exploration/frontier_score', 10)
        self.information_gain_pub = self.create_publisher(Int32, '/exploration/information_gain', 10)
        self.goal_age_pub = self.create_publisher(Float32, '/exploration/goal_age', 10)
        self.blacklist_count_pub = self.create_publisher(Int32, '/exploration/blacklisted_count', 10)
        self.goal_reason_pub = self.create_publisher(String, '/exploration/goal_change_reason', 10)
        self.goal_status_pub = self.create_publisher(String, '/exploration/goal_status', 10)
        self.completed_pub = self.create_publisher(Int32, '/exploration/frontiers_completed', 10)
        self.goal_distance_pub = self.create_publisher(Float32, '/exploration/goal_distance', 10)
        self.progress_distance_pub = self.create_publisher(
            Float32, '/exploration/progress_distance', 10)
        self.declare_parameter('minimum_frontier_cluster_size', 3)
        self.declare_parameter('progress_timeout_sec', 40.0)
        self.declare_parameter('progress_epsilon_m', 0.25)
        self.declare_parameter('blacklist_duration_sec', 45.0)
        self.declare_parameter('goal_switch_interval_sec', 3.0)
        self.declare_parameter('goal_distance_hysteresis_m', 0.75)
        self.declare_parameter('duplicate_frontier_radius_m', 0.65)
        self.declare_parameter('goal_reached_tolerance_m', 0.45)
        self.declare_parameter('goal_completion_min_new_cells', 25)
        self.declare_parameter('blacklist_radius_m', 0.75)
        self.declare_parameter('completed_goal_cooldown_sec', 90.0)
        self.declare_parameter('exploration_radius_m', 1000.0)
        self.radius_reported = False
        self.goal_pub = self.create_publisher(PointStamped, '/exploration/goal', 10)
        self.frontiers_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 10)
        self.path_pub = self.create_publisher(Path, '/planner/global_path', 10)
        self.path_state_pub = self.create_publisher(String, '/planner/global_path_status', 10)
        self.timer = self.create_timer(1.0, self.process)

    def map_callback(self, msg):
        self.map = msg

    def _cost_map_callback(self, msg):
        """Store the latest fused cost map from /map/cost_map."""
        self.cost_map = msg

    def pose_callback(self, msg):
        self.pose = msg

    def mission_state_callback(self, msg):
        self.mission_state = msg.data

    def mission_goal_callback(self, msg):
        self.mission_goal = msg

    def home_pose_callback(self, msg):
        self.home_pose = msg

    def _terrain_score_cb(self, msg):
        """Update terrain memory score for the current robot position."""
        self.terrain_memory_score = msg.data

    def _obstacles_cb(self, msg):
        """Update tracked obstacles list."""
        self.tracked_obstacles = msg

    def _obstacle_positions(self) -> List[Tuple[float, float]]:
        """Extract (x, y) world positions from the most recent TrackedObstacleArray.

        Obstacles are reported in base_link frame by obstacle_tracker.  We need
        world-frame positions for the proximity penalty.  If robot pose is
        available, transform to world frame; otherwise return an empty list so
        the enhanced scorer gracefully falls back to no-penalty.
        """
        if self.tracked_obstacles is None or self.pose is None:
            return []
        rx = self.pose.pose.position.x
        ry = self.pose.pose.position.y
        q = self.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        positions = []
        for obs in self.tracked_obstacles.obstacles:
            # Rotate from base_link to world frame
            wx = rx + cos_y * obs.position.x - sin_y * obs.position.y
            wy = ry + sin_y * obs.position.x + cos_y * obs.position.y
            positions.append((wx, wy))
        return positions

    def _bootstrap_exploration_goal(self, pose, map_info, grid, start_cell):
        """Generate a simple forward exploration goal when the map is too fragmented.
        
        When SLAM's initial map consists of disconnected free-cell islands,
        issue a goal 1.0 m ahead in the robot's current heading to bootstrap
        motion and allow the map to grow. Only used when reachable region < 10 cells.
        
        Returns a PointStamped goal in the map frame, or None if the robot's
        heading or pose is invalid.
        """
        q = self.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Bootstrap distance: 1.0 m ahead
        bootstrap_distance = 1.0
        
        goal_x = pose.x + bootstrap_distance * math.cos(yaw)
        goal_y = pose.y + bootstrap_distance * math.sin(yaw)
        
        # Verify the goal is within reasonable bounds (not wildly out of map)
        goal_col = int((goal_x - map_info.origin.position.x) / map_info.resolution)
        goal_row = int((goal_y - map_info.origin.position.y) / map_info.resolution)
        
        # Allow bootstrap goals slightly outside current map bounds — SLAM will
        # expand the map as the robot moves. Just prevent completely invalid coords.
        if (goal_col < -50 or goal_col > map_info.width + 50 or
            goal_row < -50 or goal_row > map_info.height + 50):
            return None
        
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map.header.frame_id
        goal.point.x = goal_x
        goal.point.y = goal_y
        goal.point.z = 0.0
        
        return goal

    def say_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        if state != self.previous_state:
            self.get_logger().info(f'Exploration state: {state}')
            self.previous_state = state

    def publish_empty_path(self, state):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map.header.frame_id if self.map is not None else 'odom'
        self.path_pub.publish(msg)
        status = String()
        status.data = state
        self.path_state_pub.publish(status)

    def process(self):
        if self.map is None or self.pose is None:
            self.say_state('MAPPING')
            self.publish_empty_path('WAITING_FOR_MAP_OR_POSE')
            return
        if self.home_pose is None:
            self.say_state('MAPPING')
            self.publish_empty_path('WAITING_FOR_HOME_POSE')
            return
        info = self.map.info
        grid = np.asarray(self.map.data, dtype=np.int8).reshape(info.height, info.width)
        known = grid >= 0
        known_count = int(np.count_nonzero(known))
        explored = Float32()
        explored.data = 100.0 * known_count / max(1, grid.size)
        self.explored_pub.publish(explored)
        frontiers = find_frontiers(grid)
        radius = float(self.get_parameter('exploration_radius_m').value)
        if not self.radius_reported:
            self.get_logger().info(f'Frontier search radius: {radius:.2f} m from captured home')
            self.radius_reported = True
        if self.home_pose is not None and math.isfinite(radius) and radius > 0.0:
            home = self.home_pose.pose.position
            if radius < 999.0:
                frontiers = filter_frontiers_by_radius(
                    frontiers, info.origin.position.x, info.origin.position.y,
                    info.resolution, home.x, home.y, radius)
        minimum_size = int(self.get_parameter('minimum_frontier_cluster_size').value)
        duplicate_radius_cells = float(
            self.get_parameter('duplicate_frontier_radius_m').value) / info.resolution
        frontiers, frontier_clusters = merge_frontier_array(
            frontiers, minimum_size, duplicate_radius_cells)
        count = Int32()
        count.data = int(len(frontier_clusters))
        self.frontier_count_pub.publish(count)
        markers = MarkerArray()
        marker = Marker()
        marker.header = self.map.header
        marker.ns = 'frontiers'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = info.resolution * 1.5
        marker.scale.y = info.resolution * 1.5
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 1.0, 0.9
        # Keep visualization bounded while retaining the full candidate count.
        for row, col in frontiers[::max(1, len(frontiers) // 2000)]:
            p = Point()
            p.x = info.origin.position.x + (int(col) + 0.5) * info.resolution
            p.y = info.origin.position.y + (int(row) + 0.5) * info.resolution
            marker.points.append(p)
        markers.markers.append(marker)
        self.frontiers_pub.publish(markers)

        if (not len(frontiers) and self.active is None
                and self.mission_state not in ('RETURNING_HOME', 'MISSION_COMPLETE',
                                               'SAFE_STOP', 'FAILED')):
            self.say_state(exploration_mode(known_count > 0, 0, 0))
            self.publish_empty_path('NO_FRONTIER')
            return

        now = time.monotonic()
        blacklisted_this_cycle = False
        self.blacklist = {cell: expiry for cell, expiry in self.blacklist.items()
                          if expiry > now}

        pose = self.pose.pose.position
        start_col = math.floor((pose.x - info.origin.position.x) / info.resolution)
        start_row = math.floor((pose.y - info.origin.position.y) / info.resolution)

        # If the robot's projected cell is outside the map boundary, the
        # standard nearest_free search radius of 15 cells may snap to an
        # isolated free cell at the map edge that has no path to the main
        # explored region.  Scale the search radius to cover the out-of-bounds
        # offset plus a margin so we always land in a well-connected cell.
        col_clamped = max(0, min(info.width - 1, start_col))
        row_clamped = max(0, min(info.height - 1, start_row))
        oob_cells = math.hypot(start_col - col_clamped, start_row - row_clamped)
        # 15-cell default covers normal in-bounds cases; add the out-of-bounds
        # distance plus a 20-cell safety margin for borderline cases.
        search_radius = max(15, int(math.ceil(oob_cells)) + 20)

        start = nearest_free(grid, (start_col, start_row), radius=search_radius)
        if start is None:
            self.say_state('MAPPING')
            self.publish_empty_path('WAITING_FOR_KNOWN_FREE_SPACE')
            return

        # Use cost-aware Dijkstra if the fused cost map is available and its
        # dimensions match the occupancy grid.  Fall back to plain Dijkstra
        # with a throttled warning when dimensions do not match or no cost map
        # has been received yet.
        if (self.cost_map is not None
                and self.cost_map.info.width == info.width
                and self.cost_map.info.height == info.height):
            distances, parents = shortest_paths_with_cost(
                grid,
                self.cost_map.data,
                self.cost_map.info.width,
                start,
                _warn_fn=lambda msg: self.get_logger().warning(
                    msg, throttle_duration_sec=5.0),
            )
        else:
            if self.cost_map is not None:
                self.get_logger().warning(
                    f'Cost map dimensions ({self.cost_map.info.width}x'
                    f'{self.cost_map.info.height}) do not match occupancy grid '
                    f'({info.width}x{info.height}); falling back to plain Dijkstra.',
                    throttle_duration_sec=5.0)
            distances, parents = shortest_paths(grid, start)

        # Connectivity check: SLAM's initial map creates many tiny isolated
        # free-cell islands along individual laser beam paths before they grow
        # into a connected navigable region.  If Dijkstra from the nearest
        # free cell reaches fewer than min_connected_cells cells the map is
        # not yet mature enough to plan a safe route directly from that cell.
        #
        # Fallback: call find_best_connected_start() with the initial start seed
        # to find a better cell within the SAME component.  The function will
        # ONLY return a cell from the component containing ``start`` (never from
        # a different, potentially unreachable component), and only if that
        # component has at least min_connected_cells cells.  This maintains the
        # connectivity guarantee while potentially improving Dijkstra's reach.
        min_connected_cells = 10
        if len(distances) < min_connected_cells:
            self.get_logger().info(
                f'Reachable region too small ({len(distances)} cells < '
                f'{min_connected_cells}) from nearest_free; '
                f'trying find_best_connected_start fallback.',
                throttle_duration_sec=2.0)
            fallback_start = find_best_connected_start(
                grid, start_col, start_row,
                min_component_size=min_connected_cells,
                initial_start_seed=start)
            if fallback_start is not None and fallback_start != start:
                if (self.cost_map is not None
                        and self.cost_map.info.width == info.width
                        and self.cost_map.info.height == info.height):
                    distances, parents = shortest_paths_with_cost(
                        grid, self.cost_map.data, self.cost_map.info.width,
                        fallback_start,
                        _warn_fn=lambda msg: self.get_logger().warning(
                            msg, throttle_duration_sec=5.0))
                else:
                    distances, parents = shortest_paths(grid, fallback_start)
                start = fallback_start
                self.get_logger().info(
                    f'find_best_connected_start: new start={fallback_start} '
                    f'reachable={len(distances)} cells',
                    throttle_duration_sec=2.0)
            if len(distances) < min_connected_cells:
                # Bootstrap exploration: when the SLAM map is too fragmented to
                # find reachable frontiers, issue a simple forward-exploration
                # goal to allow the robot to start moving and grow the map.
                # This breaks the deadlock: robot stopped → map doesn't grow →
                # no reachable frontiers → robot stays stopped.
                bootstrap_goal = self._bootstrap_exploration_goal(
                    pose, info, grid, start)
                if bootstrap_goal is not None:
                    self.get_logger().info(
                        f'Bootstrap exploration: issuing forward goal at '
                        f'({bootstrap_goal.point.x:.2f}, {bootstrap_goal.point.y:.2f}) '
                        f'to grow fragmented map (reachable={len(distances)} cells)',
                        throttle_duration_sec=5.0)
                    self.goal_pub.publish(bootstrap_goal)
                    # Build a trivial straight path to the bootstrap goal
                    path = Path()
                    path.header.stamp = self.get_clock().now().to_msg()
                    path.header.frame_id = self.map.header.frame_id
                    for fraction in [0.0, 0.5, 1.0]:
                        pose_msg = PoseStamped()
                        pose_msg.header = path.header
                        pose_msg.pose.position.x = (
                            pose.x * (1.0 - fraction) + bootstrap_goal.point.x * fraction)
                        pose_msg.pose.position.y = (
                            pose.y * (1.0 - fraction) + bootstrap_goal.point.y * fraction)
                        pose_msg.pose.orientation.w = 1.0
                        path.poses.append(pose_msg)
                    self.path_pub.publish(path)
                    status = String()
                    status.data = 'BOOTSTRAP_EXPLORATION'
                    self.path_state_pub.publish(status)
                    self.say_state('MAPPING')
                    return
                
                self.get_logger().info(
                    f'Reachable region too small ({len(distances)} cells < '
                    f'{min_connected_cells}); waiting for SLAM to grow free space.',
                    throttle_duration_sec=2.0)
                self.say_state('MAPPING')
                self.publish_empty_path('WAITING_FOR_CONNECTED_FREE_SPACE')
                return

        width = info.width
        if self.mission_state == 'RETURNING_HOME' and self.mission_goal is not None:
            target_col = math.floor((self.mission_goal.point.x - info.origin.position.x)
                                    / info.resolution)
            target_row = math.floor((self.mission_goal.point.y - info.origin.position.y)
                                    / info.resolution)
            target = nearest_free(grid, (target_col, target_row), radius=8)
            if target is None or target[1] * width + target[0] not in distances:
                self.publish_empty_path('HOME_ROUTE_UNAVAILABLE')
                self.say_state('RETURNING_HOME_NO_ROUTE')
                return
            route = reconstruct_path(parents, target, width)
            path = self.path_message(route, info)
            self.path_pub.publish(path)
            status = String()
            status.data = 'HOME_ROUTE_AVAILABLE'
            self.path_state_pub.publish(status)
            self.say_state('RETURNING_HOME')
            return

        if self.mission_state in ('MISSION_COMPLETE', 'SAFE_STOP', 'FAILED'):
            self.publish_empty_path(self.mission_state)
            return

        if self.mission_state not in ('EXPLORING', 'REPLANNING'):
            self.publish_empty_path('WAITING_FOR_MISSION_STATE')
            return

        metric_distances = {idx: distance * info.resolution
                            for idx, distance in distances.items()}
        if self.active is not None:
            active_x = info.origin.position.x + (self.active[0] + 0.5) * info.resolution
            active_y = info.origin.position.y + (self.active[1] + 0.5) * info.resolution
            distance_to_active = math.hypot(active_x - pose.x, active_y - pose.y)
            known_delta = max(0, known_count - self.active_known_count)
            frontier_exists = is_near_frontier(
                frontiers, self.active, max(1.0, duplicate_radius_cells))
            completion = goal_completion_reason(
                distance_to_active,
                float(self.get_parameter('goal_reached_tolerance_m').value),
                known_delta,
                int(self.get_parameter('goal_completion_min_new_cells').value),
                frontier_exists)
            if completion:
                self.frontiers_completed += 1
                self.completed_pub.publish(Int32(data=self.frontiers_completed))
                event = {
                    'event': 'GOAL_COMPLETED', 'goal_id': self.goal_event_id,
                    'goal': {'x': active_x, 'y': active_y},
                    'distance_m': distance_to_active, 'reason': completion,
                    'new_known_cells': known_delta,
                    'new_known_area_m2': known_delta * info.resolution ** 2,
                }
                self.goal_status_pub.publish(String(data=json.dumps(event, sort_keys=True)))
                self.get_logger().info(
                    f"Frontier goal completed: id={self.goal_event_id} reason={completion} "
                    f"distance={distance_to_active:.2f} m new_known_cells={known_delta} "
                    f"new_known_area={known_delta * info.resolution ** 2:.2f} m2")
                cooldown = float(self.get_parameter(
                    'completed_goal_cooldown_sec').value)
                if cooldown > 0.0:
                    self.blacklist[self.active] = now + cooldown
                    self.goal_status_pub.publish(String(data=json.dumps({
                        'event': 'GOAL_COOLDOWN', 'goal_id': self.goal_event_id,
                        'goal': {'x': active_x, 'y': active_y},
                        'duration_sec': cooldown,
                        'reason': 'COMPLETED_FRONTIER_SPATIAL_SUPPRESSION',
                    }, sort_keys=True)))
                self.active = None
                self.active_anchor = None
                self.publish_empty_path('GOAL_COMPLETED_SELECT_NEXT')
                return

        target, distance_m, reachable_count = select_frontier(
            grid, frontiers, metric_distances, self.active, start, info.resolution,
            min_goal_distance=0.20,
            reached_distance=0.0,
            minimum_cluster_size=minimum_size,
            current_yaw=math.atan2(
                2 * (self.pose.pose.orientation.w * self.pose.pose.orientation.z
                     + self.pose.pose.orientation.x * self.pose.pose.orientation.y),
                1 - 2 * (self.pose.pose.orientation.y ** 2
                         + self.pose.pose.orientation.z ** 2)),
            blacklist=set(self.blacklist),
            blacklist_radius_cells=float(
                self.get_parameter('blacklist_radius_m').value) / info.resolution,
            duplicate_radius_cells=duplicate_radius_cells,
            active_recovery_radius_cells=max(1, int(math.ceil(
                float(self.get_parameter('goal_distance_hysteresis_m').value)
                / info.resolution))),
            terrain_score=self.terrain_memory_score,
            obstacle_positions=self._obstacle_positions(),
            map_origin_x=info.origin.position.x,
            map_origin_y=info.origin.position.y)

        if self.active is not None and self.active_goal_started is not None:
            anchor = self.active_anchor or self.active
            current_distance = metric_distances.get(
                anchor[1] * width + anchor[0], math.inf)
            if not math.isfinite(current_distance):
                anchor_x = info.origin.position.x + (anchor[0] + 0.5) * info.resolution
                anchor_y = info.origin.position.y + (anchor[1] + 0.5) * info.resolution
                current_distance = math.hypot(pose.x - anchor_x, pose.y - anchor_y)
            if current_distance + float(self.get_parameter('progress_epsilon_m').value) < self.active_goal_best_distance:
                self.active_goal_best_distance = current_distance
                self.last_progress_time = now
            elif (self.last_progress_time is not None
                  and no_progress_timeout_reached(
                      now - self.last_progress_time, float(
                          self.get_parameter('progress_timeout_sec').value))):
                self.blacklist[self.active] = now + float(
                    self.get_parameter('blacklist_duration_sec').value)
                event = {
                    'event': 'GOAL_BLACKLISTED', 'goal_id': self.goal_event_id,
                    'reason': 'NO_PROGRESS_TIMEOUT',
                    'best_distance_m': self.active_goal_best_distance,
                    'current_distance_m': current_distance,
                }
                self.goal_status_pub.publish(String(data=json.dumps(event, sort_keys=True)))
                self.get_logger().warning(
                    f"Frontier goal blacklisted: id={self.goal_event_id} "
                    f"reason=NO_PROGRESS_TIMEOUT best={self.active_goal_best_distance:.2f} m "
                    f"current={current_distance:.2f} m")
                self.goal_change_reason = 'PROGRESS_TIMEOUT_BLACKLISTED'
                blacklisted_this_cycle = True
                self.active = None
                self.active_anchor = None
                target, distance_m, reachable_count = select_frontier(
                    grid, frontiers, metric_distances, None, start, info.resolution,
                    min_goal_distance=0.20,
                    reached_distance=0.0,
                    minimum_cluster_size=minimum_size,
                    current_yaw=0.0, blacklist=set(self.blacklist),
                    blacklist_radius_cells=float(
                        self.get_parameter('blacklist_radius_m').value) / info.resolution,
                    duplicate_radius_cells=duplicate_radius_cells,
                    active_recovery_radius_cells=max(1, int(math.ceil(
                        float(self.get_parameter('goal_distance_hysteresis_m').value)
                        / info.resolution))),
                    terrain_score=self.terrain_memory_score,
                    obstacle_positions=self._obstacle_positions(),
                    map_origin_x=info.origin.position.x,
                    map_origin_y=info.origin.position.y)

        if target is None:
            if self.active is not None:
                event = {
                    'event': 'GOAL_INVALIDATED', 'goal_id': self.goal_event_id,
                    'reason': 'NO_LONGER_REACHABLE_AFTER_MAP_UPDATE',
                }
                self.goal_status_pub.publish(String(data=json.dumps(event, sort_keys=True)))
                self.blacklist[self.active] = now + float(
                    self.get_parameter('blacklist_duration_sec').value)
                self.get_logger().warning(
                    f"Frontier goal invalidated: id={self.goal_event_id} "
                    "reason=NO_LONGER_REACHABLE_AFTER_MAP_UPDATE")
            self.active = None
            self.active_anchor = None
            self.say_state('NO_REACHABLE_FRONTIER')
            self.publish_empty_path('NO_REACHABLE_FRONTIER')
            return

        goal_switched = target != self.active
        hysteresis_cells = float(
            self.get_parameter('goal_distance_hysteresis_m').value) / info.resolution
        if goal_switched and same_frontier_region(
                self.active_anchor, target, hysteresis_cells):
            goal_switched = False
        if goal_switched:
            interval = float(self.get_parameter('goal_switch_interval_sec').value)
            if (self.last_goal_switch_time
                    and now - self.last_goal_switch_time < interval):
                self.get_logger().info(
                    f'Frontier switch deferred: id={self.goal_event_id} '
                    f'candidate={target} reason=GOAL_SWITCH_INTERVAL')
                self.publish_empty_path('GOAL_SWITCH_COOLDOWN')
                return
            self.active_anchor = target
            self.active_goal_started = now
            self.active_goal_best_distance = math.inf
            self.last_progress_time = now
            self.active_known_count = known_count
            self.last_goal_switch_time = now
            self.goal_event_id += 1
            if blacklisted_this_cycle:
                self.goal_change_reason = 'PROGRESS_TIMEOUT_BLACKLISTED'
            else:
                self.goal_change_reason = ('INITIAL_GOAL' if self.previous_goal is None
                                           else 'PREVIOUS_GOAL_REACHED_OR_INVALID')
        self.active = target
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map.header.frame_id
        goal.point.x = info.origin.position.x + (target[0] + 0.5) * info.resolution
        goal.point.y = info.origin.position.y + (target[1] + 0.5) * info.resolution
        self.goal_pub.publish(goal)
        if goal_switched:
            selected_event = {
                'event': 'GOAL_SELECTED', 'goal_id': self.goal_event_id,
                'goal': {'x': goal.point.x, 'y': goal.point.y},
                'reason': self.goal_change_reason,
                'distance_m': float(distance_m),
                'known_cells_at_selection': known_count,
            }
            self.goal_status_pub.publish(String(data=json.dumps(
                selected_event, sort_keys=True)))
            self.get_logger().info(
                f"Frontier selected: id={self.goal_event_id} "
                f"({goal.point.x:.2f}, {goal.point.y:.2f}) "
                f"reason={self.goal_change_reason} distance={distance_m:.2f} m "
                f"known_cells={known_count} completed={self.frontiers_completed}")
        active_marker = Marker()
        active_marker.header = goal.header
        active_marker.ns = 'active_frontier'
        active_marker.id = 1
        active_marker.type = Marker.SPHERE
        active_marker.action = Marker.ADD
        active_marker.pose.position.x = goal.point.x
        active_marker.pose.position.y = goal.point.y
        active_marker.pose.orientation.w = 1.0
        active_marker.scale.x = active_marker.scale.y = active_marker.scale.z = 0.32
        active_marker.color.r, active_marker.color.g = 0.15, 1.0
        active_marker.color.b, active_marker.color.a = 0.65, 1.0
        markers.markers.append(active_marker)
        self.frontiers_pub.publish(markers)
        distance_msg = Float32()
        distance_msg.data = float(distance_m)
        self.distance_pub.publish(distance_msg)
        self.goal_distance_pub.publish(Float32(data=float(distance_m)))
        self.progress_distance_pub.publish(Float32(data=float(
            metric_distances.get(target[1] * width + target[0], distance_m))))
        selected_id = Int32(data=int(target[1] * width + target[0]))
        self.selected_id_pub.publish(selected_id)
        selected_cluster = next((cluster for cluster in cluster_frontiers(
            frontiers, minimum_size)
            if target in cluster), [target])
        gain = 0
        for cluster_col, cluster_row in selected_cluster:
            gain += int(np.count_nonzero(
                grid[max(0, cluster_row - 2):min(info.height, cluster_row + 3),
                     max(0, cluster_col - 2):min(info.width, cluster_col + 3)] == -1))
        score_msg = Float32(data=float(gain / max(1.0, distance_m + 1.0)))
        self.frontier_score_pub.publish(score_msg)
        self.information_gain_pub.publish(Int32(data=gain))
        self.goal_age_pub.publish(Float32(data=max(0.0, now - (self.active_goal_started or now))))
        self.blacklist_count_pub.publish(Int32(data=len(self.blacklist)))
        self.goal_reason_pub.publish(String(data=self.goal_change_reason))
        raw_path_cells = reconstruct_path(parents, target, width)
        path = self.path_message(raw_path_cells, info)
        self.path_pub.publish(path)
        status = String()
        status.data = 'AVAILABLE' if path.poses else 'NO_PATH'
        self.path_state_pub.publish(status)
        self.say_state('EXPLORING' if target is not None else
                       exploration_mode(known_count > 0, len(frontiers), reachable_count))
        if target != self.previous_goal:
            self.get_logger().info(
                f'Frontier selected at ({goal.point.x:.2f}, {goal.point.y:.2f}); '
                f'id={self.goal_event_id} reason={self.goal_change_reason} '
                f'path={len(path.poses)} cells distance={distance_m:.2f} m '
                f'known_cells={known_count} completed={self.frontiers_completed}')
            self.previous_goal = target

    def path_message(self, cells, info):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        # Use the actual map frame so downstream consumers see consistent coordinates.
        # With slam:=true this is 'map'; without SLAM it is 'odom'.
        path.header.frame_id = self.map.header.frame_id if self.map is not None else 'odom'
        for col, row in cells:
            pose_msg = PoseStamped()
            pose_msg.header = path.header
            pose_msg.pose.position.x = info.origin.position.x + (col + 0.5) * info.resolution
            pose_msg.pose.position.y = info.origin.position.y + (row + 0.5) * info.resolution
            pose_msg.pose.orientation.w = 1.0
            path.poses.append(pose_msg)
        return path


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
