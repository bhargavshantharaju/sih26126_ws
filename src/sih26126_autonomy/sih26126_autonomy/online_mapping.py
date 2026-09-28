"""Gazebo-odometry adapter and live depth-to-occupancy mapper."""

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from sih26126_planner.local_path_selector import pointcloud_to_array


MAP_RESOLUTION = 0.10
MAP_WIDTH = 300
MAP_HEIGHT = 300
MAP_ORIGIN_X = -10.0
MAP_ORIGIN_Y = -15.0
MAX_RANGE_M = 8.0
SENSOR_FORWARD_OFFSET_M = 0.30
FLOOR_Z_CAMERA_M = -1.12
OBSTACLE_HEIGHT_MIN_M = 0.16
OBSTACLE_HEIGHT_MAX_M = 2.5


def world_to_cell(x: float, y: float, origin_x: float = MAP_ORIGIN_X,
                  origin_y: float = MAP_ORIGIN_Y,
                  resolution: float = MAP_RESOLUTION):
    """Return integer (column, row), or None when outside the grid."""
    col = math.floor((x - origin_x) / resolution)
    row = math.floor((y - origin_y) / resolution)
    if 0 <= col < MAP_WIDTH and 0 <= row < MAP_HEIGHT:
        return col, row
    return None


def mark_ray(grid: np.ndarray, start, end, occupied: bool) -> None:
    """Mark observed free cells along a grid ray and its optional hit cell."""
    x0, y0 = int(start[0]), int(start[1])
    x1, y1 = int(end[0]), int(end[1])
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice > -dy:
            error -= dy
            x0 += sx
        if twice < dx:
            error += dx
            y0 += sy
    for col, row in cells[:-1]:
        if 0 <= col < grid.shape[1] and 0 <= row < grid.shape[0]:
            if grid[row, col] != 100:
                grid[row, col] = 0
    col, row = cells[-1]
    if 0 <= col < grid.shape[1] and 0 <= row < grid.shape[0] and occupied:
        grid[row, col] = 100


def pointcloud_rays(points: np.ndarray, pose_x: float, pose_y: float,
                    yaw: float, resolution: float = MAP_RESOLUTION):
    """Yield bounded map rays from the sampled current sensor cloud."""
    origin_x = pose_x + SENSOR_FORWARD_OFFSET_M * math.cos(yaw)
    origin_y = pose_y + SENSOR_FORWARD_OFFSET_M * math.sin(yaw)
    start = world_to_cell(origin_x, origin_y, resolution=resolution)
    if start is None:
        return
    c, s = math.cos(yaw), math.sin(yaw)
    for px, py, pz in points:
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
            continue
        distance = math.hypot(float(px), float(py))
        if distance < 0.25 or distance > MAX_RANGE_M or px <= 0.0:
            continue
        wx = origin_x + c * float(px) - s * float(py)
        wy = origin_y + s * float(px) + c * float(py)
        end = world_to_cell(wx, wy, resolution=resolution)
        if end is None:
            continue
        height = float(pz) - FLOOR_Z_CAMERA_M
        hit = OBSTACLE_HEIGHT_MIN_M <= height <= OBSTACLE_HEIGHT_MAX_M
        yield start, end, hit


class OdomPoseAdapter(Node):
    """Expose the current Gazebo odometry pose with an honest source label."""

    def __init__(self):
        super().__init__('localization_pose_adapter')
        self.pose_pub = self.create_publisher(PoseStamped, '/localization/pose', 10)
        self.source_pub = self.create_publisher(String, '/localization/source', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/model/sih_ugv/odometry', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = 'odom'
        out.pose = msg.pose.pose
        self.pose_pub.publish(out)
        source = String()
        source.data = 'Gazebo odometry (ground-truth simulation pose)'
        self.source_pub.publish(source)


class OnlineMapper(Node):
    """Accumulate depth observations in a bounded unknown-first occupancy grid."""

    def __init__(self):
        super().__init__('online_mapper')
        self.grid = np.full((MAP_HEIGHT, MAP_WIDTH), -1, dtype=np.int8)
        self.pose: Optional[PoseStamped] = None
        self.cloud: Optional[PointCloud2] = None
        self.cloud_sequence = 0
        self.integrated_sequence = -1
        self.qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', self.qos)
        self.state_pub = self.create_publisher(String, '/mapping/state', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/localization/pose', self.pose_callback, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/depth_camera/points', self.cloud_callback, 2)
        self.timer = self.create_timer(0.5, self.process)

    def pose_callback(self, msg):
        self.pose = msg

    def cloud_callback(self, msg):
        self.cloud = msg
        self.cloud_sequence += 1

    def process(self):
        if self.pose is not None and self.cloud is not None:
            if self.integrated_sequence != self.cloud_sequence:
                points = pointcloud_to_array(self.cloud)
                if points is not None:
                    # A regular image subsample bounds ray integration cost.
                    sampled = points.reshape(
                        self.cloud.height, self.cloud.width, 3)[::12, ::12].reshape(-1, 3)
                    pos = self.pose.pose.position
                    q = self.pose.pose.orientation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))
                    for start, end, occupied in pointcloud_rays(
                            sampled, pos.x, pos.y, yaw):
                        mark_ray(self.grid, start, end, occupied)
                    self.integrated_sequence = self.cloud_sequence
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = MAP_RESOLUTION
        msg.info.width = MAP_WIDTH
        msg.info.height = MAP_HEIGHT
        msg.info.origin.position.x = MAP_ORIGIN_X
        msg.info.origin.position.y = MAP_ORIGIN_Y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.reshape(-1).tolist()
        self.map_pub.publish(msg)
        state = String()
        state.data = 'BUILDING' if np.any(self.grid >= 0) else 'INITIALIZING'
        self.state_pub.publish(state)


def pose_main(args=None):
    rclpy.init(args=args)
    node = OdomPoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def mapper_main(args=None):
    rclpy.init(args=args)
    node = OnlineMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
