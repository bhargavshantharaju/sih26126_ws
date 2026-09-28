"""Frontier selection and shortest paths over observed free occupancy cells."""

import heapq
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String
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
                    reached_distance: float = 0.8) -> Tuple[Optional[Cell], float, int]:
    """Select a useful reachable frontier and retain its goal."""
    height, width = grid.shape
    reachable = []
    for row_value, col_value in frontiers:
        row, col = int(row_value), int(col_value)
        cell = (col, row)
        index = row * width + col
        if index not in distances:
            continue
        distance = float(distances[index])
        neighborhood = grid[max(0, row - 2):min(height, row + 3),
                            max(0, col - 2):min(width, col + 3)]
        gain = int(np.count_nonzero(neighborhood == -1))
        reachable.append((gain, distance, cell))

    if active is not None:
        active_distance = math.hypot(active[0] - start[0], active[1] - start[1]) * resolution
        in_bounds = (0 <= active[0] < width and 0 <= active[1] < height)
        active_index = active[1] * width + active[0]
        still_free = in_bounds and grid[active[1], active[0]] == 0
        if (still_free and active_index in distances
                and active_distance >= reached_distance):
            return active, float(distances[active[1] * width + active[0]]), len(reachable)

    eligible = [item for item in reachable
                if item[1] >= min_goal_distance]
    if not eligible:
        return None, 0.0, len(reachable)
    gain, distance, target = max(eligible, key=lambda item: (item[0], item[1]))
    return target, float(distance), len(reachable)


class FrontierExplorer(Node):
    """Select reachable frontiers and publish an occupancy-grid global path."""

    def __init__(self):
        super().__init__('frontier_explorer')
        self.map: Optional[OccupancyGrid] = None
        self.pose: Optional[PoseStamped] = None
        self.active: Optional[Cell] = None
        self.previous_state = ''
        self.previous_goal = None
        self.map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, self.map_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/localization/pose', self.pose_callback, 10)
        self.state_pub = self.create_publisher(String, '/exploration/state', 10)
        self.frontier_count_pub = self.create_publisher(Int32, '/exploration/frontier_count', 10)
        self.explored_pub = self.create_publisher(Float32, '/exploration/explored_percentage', 10)
        self.distance_pub = self.create_publisher(Float32, '/exploration/distance_to_frontier', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/exploration/goal', 10)
        self.frontiers_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 10)
        self.path_pub = self.create_publisher(Path, '/planner/global_path', 10)
        self.path_state_pub = self.create_publisher(String, '/planner/global_path_status', 10)
        self.timer = self.create_timer(1.0, self.process)

    def map_callback(self, msg):
        self.map = msg

    def pose_callback(self, msg):
        self.pose = msg

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
        msg.header.frame_id = 'odom'
        self.path_pub.publish(msg)
        status = String()
        status.data = state
        self.path_state_pub.publish(status)

    def process(self):
        if self.map is None or self.pose is None:
            self.say_state('MAPPING')
            self.publish_empty_path('WAITING_FOR_MAP_OR_POSE')
            return
        info = self.map.info
        grid = np.asarray(self.map.data, dtype=np.int8).reshape(info.height, info.width)
        known = grid >= 0
        known_count = int(np.count_nonzero(known))
        explored = Float32()
        explored.data = 100.0 * known_count / max(1, grid.size)
        self.explored_pub.publish(explored)
        frontiers = find_frontiers(grid)
        count = Int32()
        count.data = int(len(frontiers))
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

        if not len(frontiers):
            self.active = None
            self.say_state(exploration_mode(known_count > 0, 0, 0))
            self.publish_empty_path('NO_FRONTIER')
            return

        pose = self.pose.pose.position
        start_col = math.floor((pose.x - info.origin.position.x) / info.resolution)
        start_row = math.floor((pose.y - info.origin.position.y) / info.resolution)
        start = nearest_free(grid, (start_col, start_row))
        if start is None:
            self.say_state('MAPPING')
            self.publish_empty_path('WAITING_FOR_KNOWN_FREE_SPACE')
            return

        distances, parents = shortest_paths(grid, start)
        width = info.width
        metric_distances = {idx: distance * info.resolution
                            for idx, distance in distances.items()}
        target, distance_m, reachable_count = select_frontier(
            grid, frontiers, metric_distances, self.active, start, info.resolution)
        if target is None:
            self.active = None
            self.say_state(exploration_mode(known_count > 0, len(frontiers), reachable_count))
            self.publish_empty_path('NO_REACHABLE_FRONTIER')
            return

        self.active = target
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.point.x = info.origin.position.x + (target[0] + 0.5) * info.resolution
        goal.point.y = info.origin.position.y + (target[1] + 0.5) * info.resolution
        self.goal_pub.publish(goal)
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
        path = Path()
        path.header = goal.header
        for col, row in reconstruct_path(parents, target, width):
            pose_msg = PoseStamped()
            pose_msg.header = path.header
            pose_msg.pose.position.x = info.origin.position.x + (col + 0.5) * info.resolution
            pose_msg.pose.position.y = info.origin.position.y + (row + 0.5) * info.resolution
            pose_msg.pose.orientation.w = 1.0
            path.poses.append(pose_msg)
        self.path_pub.publish(path)
        status = String()
        status.data = 'AVAILABLE' if path.poses else 'NO_PATH'
        self.path_state_pub.publish(status)
        self.say_state(exploration_mode(known_count > 0, len(frontiers), reachable_count))
        if target != self.previous_goal:
            self.get_logger().info(
                f'Frontier selected at ({goal.point.x:.2f}, {goal.point.y:.2f}); '
                f'path={len(path.poses)} cells distance={distance_m:.2f} m')
            self.previous_goal = target


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
