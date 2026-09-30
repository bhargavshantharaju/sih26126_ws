"""Spatial local path selection from terrain-aware traversability."""

import json
import math
import time
from typing import List, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32, String


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if math.isclose(wrapped, -math.pi, abs_tol=1e-12):
        return math.pi
    return wrapped


def input_is_fresh(received_at: Optional[float], max_age_s: float,
                   now: Optional[float] = None) -> bool:
    """Return whether a monotonic receive time is within its age limit."""
    if received_at is None or max_age_s < 0.0:
        return False
    current = time.monotonic() if now is None else now
    age = current - received_at
    return 0.0 <= age <= max_age_s


def quaternion_to_yaw(q) -> float:
    """Convert quaternion to planar yaw."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def goal_angle_from_pose(
    pose_x: float,
    pose_y: float,
    yaw: float,
    goal_x: float,
    goal_y: float,
) -> float:
    """Return goal bearing in the robot body frame."""
    world_angle = math.atan2(goal_y - pose_y, goal_x - pose_x)
    return wrap_angle(world_angle - yaw)


def angle_to_pixel(
    angle_rad: float,
    fx: float,
    cx: float,
    width: int,
) -> int:
    """Project a body-frame left-positive angle into an image column."""
    if width <= 0 or fx <= 0.0:
        return width // 2

    # ROS yaw/angular-z convention is left-positive.
    # A forward-facing optical image is right-positive in pixel x,
    # hence the negative sign.
    u = cx - fx * math.tan(angle_rad)
    return int(round(max(0.0, min(float(width - 1), u))))


def image_to_array(msg: Image) -> Optional[np.ndarray]:
    """Convert a mono8 image message to a 2D uint8 array."""
    if msg.encoding not in ('mono8', '8UC1'):
        return None
    if msg.width <= 0 or msg.height <= 0:
        return None
    if msg.step < msg.width:
        return None

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    required = msg.step * msg.height
    if raw.size < required:
        return None

    rows = raw[:required].reshape(msg.height, msg.step)
    return rows[:, :msg.width].copy()


def sample_corridor_score(
    image: np.ndarray,
    center_x: int,
    y_start_ratio: float = 0.45,
    y_end_ratio: float = 0.92,
    half_width_px: int = 18,
    row_samples: int = 12,
) -> Tuple[float, float]:
    """Return robust corridor score in [0, 1] and valid sample ratio."""
    if image.ndim != 2 or image.size == 0:
        return 0.0, 0.0

    height, width = image.shape
    y0 = max(0, min(height - 1, int(height * y_start_ratio)))
    y1 = max(y0 + 1, min(height, int(height * y_end_ratio)))

    if row_samples <= 0:
        row_samples = 1

    rows = np.linspace(y0, y1 - 1, row_samples, dtype=int)

    row_means: List[float] = []
    valid_count = 0
    total_count = 0

    for y in rows:
        x0 = max(0, center_x - half_width_px)
        x1 = min(width, center_x + half_width_px + 1)

        region = image[y, x0:x1]
        total_count += int(region.size)

        if region.size == 0:
            continue

        # 0 is treated as invalid/unusable for this map.
        valid = region[region > 0]

        if valid.size == 0:
            row_means.append(0.0)
            continue

        valid_count += int(valid.size)
        row_means.append(float(np.mean(valid)))

    if not row_means or total_count == 0:
        return 0.0, 0.0

    robust_mean = float(np.median(row_means))
    worst_band = float(np.min(row_means))

    # The worst band prevents a narrow poor section from being
    # completely hidden by good pixels elsewhere.
    combined = 0.65 * robust_mean + 0.35 * worst_band
    score = max(0.0, min(1.0, combined / 255.0))
    validity = valid_count / float(total_count)

    return score, validity


def pointcloud_to_array(msg: PointCloud2, stride: int = 1) -> Optional[np.ndarray]:
    """Convert an XYZ point cloud to an Nx3 float32 array."""
    if msg.width <= 0 or msg.height <= 0:
        return None

    if msg.point_step < 12:
        return None

    stride = max(1, int(stride))
    required = msg.row_step * msg.height

    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if raw.size < required:
        return None

    if msg.row_step == msg.width * msg.point_step:
        points = np.ndarray(
            shape=(msg.height, msg.width, 3),
            dtype=np.float32,
            buffer=raw,
            strides=(msg.row_step, msg.point_step, 4),
        )
        return points[::stride, ::stride].reshape(-1, 3).copy()

    # Handle row padding by extracting each row independently.
    rows = []
    for row in range(msg.height):
        row_start = row * msg.row_step
        row_bytes = raw[row_start:row_start + msg.width * msg.point_step]

        row_points = np.ndarray(
            shape=(msg.width, 3),
            dtype=np.float32,
            buffer=row_bytes,
            strides=(msg.point_step, 4),
        )
        if row % stride == 0:
            rows.append(row_points[::stride].copy())

    if not rows:
        return None

    return np.vstack(rows)


def obstacle_clearance(
    points: np.ndarray,
    candidate_angle: float,
    x_min: float = 0.20,
    x_max: float = 3.0,
    corridor_half_width: float = 0.35,
    min_obstacle_height: float = 0.18,
    max_obstacle_height: float = 1.2,
) -> float:
    """Estimate nearest obstacle distance along a candidate corridor."""
    if points is None or points.size == 0:
        return float('inf')

    points = np.asarray(points, dtype=np.float32)

    if points.ndim != 2 or points.shape[1] < 3:
        return float('inf')

    finite_mask = np.isfinite(points[:, :3]).all(axis=1)
    points = points[finite_mask]

    if points.size == 0:
        return float('inf')

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    forward_mask = (x >= x_min) & (x <= x_max) & (x > 0.0)

    x = x[forward_mask]
    y = y[forward_mask]
    z = z[forward_mask]

    if x.size == 0:
        return float('inf')

    lateral_error = np.abs(y - x * math.tan(candidate_angle))

    # Slightly widen the corridor farther from the camera.
    corridor_limit = corridor_half_width + 0.06 * np.maximum(
        x - x_min,
        0.0,
    )

    corridor_mask = lateral_error <= corridor_limit

    x = x[corridor_mask]
    z = z[corridor_mask]

    if x.size < 10:
        return float('inf')

    # Estimate the local ground surface independently along the path.
    #
    # This avoids using one fixed z reference when the robot drives
    # over a ramp or uneven terrain.
    bin_edges = np.linspace(x_min, x_max, 9)
    nearest_obstacle = float('inf')

    for i in range(len(bin_edges) - 1):
        bin_mask = (
            (x >= bin_edges[i])
            & (x < bin_edges[i + 1])
        )

        bin_z = z[bin_mask]
        bin_x = x[bin_mask]

        if bin_z.size < 5:
            continue

        # The lower portion of the vertical distribution is used as
        # the local ground estimate.
        ground_z = float(np.percentile(bin_z, 30.0))

        height_above_ground = bin_z - ground_z

        obstacle_mask = (
            (height_above_ground >= min_obstacle_height)
            & (height_above_ground <= max_obstacle_height)
        )

        if not np.any(obstacle_mask):
            continue

        obstacle_x = bin_x[obstacle_mask]

        if obstacle_x.size:
            nearest_obstacle = min(
                nearest_obstacle,
                float(np.min(obstacle_x)),
            )

    return nearest_obstacle


def obstacle_clearance_score(
    clearance: float,
    critical_distance: float = 1.25,
    clear_distance: float = 3.0,
) -> float:
    """Map obstacle clearance to a normalized safety score."""
    if not math.isfinite(clearance):
        return 1.0

    if clearance <= critical_distance:
        return 0.0

    if clearance >= clear_distance:
        return 1.0

    return (
        (clearance - critical_distance)
        / (clear_distance - critical_distance)
    )


def obstacle_return_evidence(points: Optional[np.ndarray], selected_angle: float):
    """Summarize actual depth returns in the forward field and selected corridor."""
    if points is None or not np.asarray(points).size:
        return {'valid_points': 0, 'forward_points': 0,
                'obstacle_points': 0, 'raw_clearance_m': None}
    points = np.asarray(points, dtype=np.float32)
    valid = np.isfinite(points[:, :3]).all(axis=1)
    points = points[valid]
    forward = points[(points[:, 0] >= 0.20) & (points[:, 0] <= 3.0)
                     & (points[:, 0] > 0.0)]
    if not len(forward):
        return {'valid_points': int(np.count_nonzero(valid)), 'forward_points': 0,
                'obstacle_points': 0, 'raw_clearance_m': None}
    x, y, z = forward[:, 0], forward[:, 1], forward[:, 2]
    corridor = np.abs(y - x * math.tan(selected_angle)) <= 0.35 + 0.06 * np.maximum(x - 0.20, 0.0)
    obstacle_mask = np.zeros(len(forward), dtype=bool)
    for lower, upper in zip(np.linspace(0.20, 3.0, 9)[:-1],
                            np.linspace(0.20, 3.0, 9)[1:]):
        in_bin = (x >= lower) & (x < upper)
        if np.count_nonzero(in_bin) < 5:
            continue
        ground = float(np.percentile(z[in_bin], 30.0))
        obstacle_mask |= in_bin & ((z - ground) >= 0.18) & ((z - ground) <= 1.2)
    distances = np.hypot(x[obstacle_mask], y[obstacle_mask])
    selected_corridor_clearance = obstacle_clearance(forward, selected_angle)
    return {
        'valid_points': int(len(points)),
        'forward_points': int(len(forward)),
        'obstacle_points': int(np.count_nonzero(obstacle_mask)),
        'selected_corridor_obstacle_points': int(np.count_nonzero(obstacle_mask & corridor)),
        'raw_clearance_m': (None if not len(distances) else float(np.min(distances))),
        'selected_corridor_clearance_m': (
            selected_corridor_clearance
            if math.isfinite(selected_corridor_clearance) else None),
        'selected_steering_rad': float(selected_angle),
    }


def waypoint_command(goal_angle: float, steering_angle: float, global_speed: float,
                     obstacle_score: float, max_angle: float = math.radians(25.0),
                     max_angular: float = 0.6, rotate_threshold: float = math.radians(5.0),
                     stop_score: float = 0.01) -> Twist:
    """Rotate toward a map waypoint before translating; stop for blocked corridors."""
    cmd = Twist()
    if obstacle_score <= stop_score:
        return cmd
    if abs(goal_angle) > max_angle + rotate_threshold:
        cmd.angular.z = math.copysign(min(max_angular, 0.45), goal_angle)
        return cmd
    speed_scale = 0.35 + 0.65 * obstacle_score
    cmd.linear.x = max(0.0, min(global_speed * speed_scale, 0.30))
    cmd.angular.z = max(-max_angular, min(max_angular, 1.8 * steering_angle))
    return cmd


def score_candidate(
    terrain_score: float,
    validity: float,
    candidate_angle: float,
    goal_angle: float,
    previous_angle: float,
    obstacle_score: float = 1.0,
) -> float:
    """Combine terrain, obstacle, goal, and smoothness scores."""
    terrain_score = max(0.0, min(1.0, terrain_score))
    validity = max(0.0, min(1.0, validity))
    obstacle_score = max(0.0, min(1.0, obstacle_score))

    goal_error = abs(wrap_angle(candidate_angle - goal_angle))
    goal_alignment = max(
        0.0,
        1.0 - goal_error / math.radians(35.0),
    )

    steering_change = abs(
        wrap_angle(candidate_angle - previous_angle)
    )
    smoothness = max(
        0.0,
        1.0 - steering_change / math.radians(35.0),
    )

    base_score = (
        0.40 * terrain_score
        + 0.35 * goal_alignment
        + 0.05 * smoothness
        + 0.05 * validity
    )

    # Clearance is a safety preference. Squaring its normalized score gives
    # a visible penalty to constrained corridors while leaving clear lanes
    # unchanged; the independent local stop and supervisor remain authoritative.
    return base_score * obstacle_score * obstacle_score


def select_best_candidate(
    image: np.ndarray,
    candidate_angles: Sequence[float],
    fx: float,
    cx: float,
    goal_angle: float,
    previous_angle: float,
    points: Optional[np.ndarray] = None,
    switch_margin: float = 0.04,
) -> Tuple[float, float, float]:
    """Select the candidate with highest terrain/obstacle score."""
    best_angle = candidate_angles[0]
    best_score = -1.0
    best_obstacle_score = 0.0
    scores = []
    # The selector can only command angles inside its candidate envelope.
    # Clamp the desired bearing into that reachable steering range so a goal
    # behind the robot still selects maximum-turn candidates instead of making
    # every candidate's goal-alignment score zero (which favors straight).
    steering_goal = max(
        min(goal_angle, max(candidate_angles)),
        min(candidate_angles),
    )

    for angle in candidate_angles:
        pixel = angle_to_pixel(
            angle,
            fx,
            cx,
            image.shape[1],
        )
        terrain, validity = sample_corridor_score(
            image,
            pixel,
        )

        if points is None:
            obstacle_score = 1.0
        else:
            clearance = obstacle_clearance(points, angle)
            obstacle_score = obstacle_clearance_score(clearance)

        combined = score_candidate(
            terrain,
            validity,
            angle,
            steering_goal,
            previous_angle,
            obstacle_score,
        )

        if combined > best_score:
            best_score = combined
            best_angle = angle
            best_obstacle_score = obstacle_score
        scores.append((angle, combined, obstacle_score))

    previous = next((item for item in scores if math.isclose(
        item[0], previous_angle, abs_tol=1e-9)), None)
    if previous is not None and previous[1] >= best_score - max(0.0, switch_margin):
        return previous[0], previous[1], previous[2]

    return best_angle, best_score, best_obstacle_score


def build_local_path(
    pose_x: float,
    pose_y: float,
    yaw: float,
    steering_angle: float,
    lookahead: float = 3.0,
    samples: int = 12,
) -> Path:
    """Build a straight advisory local path for one selected direction."""
    path = Path()
    path.header.frame_id = 'odom'

    count = max(2, samples)

    for i in range(count):
        distance = lookahead * i / float(count - 1)

        local_x = distance * math.cos(steering_angle)
        local_y = distance * math.sin(steering_angle)

        world_x = (
            pose_x
            + local_x * math.cos(yaw)
            - local_y * math.sin(yaw)
        )
        world_y = (
            pose_y
            + local_x * math.sin(yaw)
            + local_y * math.cos(yaw)
        )

        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = world_x
        pose.pose.position.y = world_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    return path


class LocalPathSelector(Node):
    """Select a spatially traversable local direction."""

    def __init__(self) -> None:
        super().__init__('local_path_selector')

        self.score_map: Optional[Image] = None
        self.pointcloud: Optional[PointCloud2] = None
        self.camera_info: Optional[CameraInfo] = None
        self.odom: Optional[Odometry] = None
        self.global_path: Optional[Path] = None
        self.navigation_state = 'UNKNOWN'
        self.global_speed = 0.0
        self.mission_state = ''
        self.home_pose = None
        self.previous_angle = 0.0
        self.score_map_received_at = None
        self.pointcloud_received_at = None
        self.odom_received_at = None

        # Keep edge candidates inside the depth camera's ~±30° horizontal
        # field of view so the rover can steer around obstacles that fill the
        # narrower center candidates without treating unseen space as clear.
        self.candidate_degrees = [-28.0, -25.0, -16.7, -8.3, 0.0,
                                  8.3, 16.7, 25.0, 28.0]

        self.declare_parameter('max_angle_deg', 28.0)
        self.declare_parameter('lookahead_m', 3.0)
        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('max_sensor_age_s', 0.75)
        self.declare_parameter('pointcloud_stride', 4)

        self.map_sub = self.create_subscription(
            Image,
            '/traversability/terrain_aware_score_map',
            self.score_map_callback,
            10,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.pointcloud_callback,
            10,
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/sih_ugv/odometry',
            self.odom_callback,
            10,
        )
        self.path_sub = self.create_subscription(
            Path,
            '/planner/path',
            self.path_callback,
            10,
        )
        self.state_sub = self.create_subscription(
            String,
            '/planner/navigation_state',
            self.state_callback,
            10,
        )
        self.speed_sub = self.create_subscription(
            Float32,
            '/planner/recommended_speed',
            self.speed_callback,
            10,
        )
        self.mission_sub = self.create_subscription(
            String, '/mission/state', self.mission_callback, 10)
        self.home_sub = self.create_subscription(
            PoseStamped, '/mission/home_pose', self.home_callback, 10)

        self.local_path_pub = self.create_publisher(
            Path,
            '/planner/local_path',
            10,
        )
        self.local_state_pub = self.create_publisher(
            String,
            '/planner/local_navigation_state',
            10,
        )
        self.local_angle_pub = self.create_publisher(
            Float32,
            '/planner/local_steering_angle',
            10,
        )
        self.local_score_pub = self.create_publisher(
            Float32,
            '/planner/local_path_score',
            10,
        )
        self.local_obstacle_clearance_pub = self.create_publisher(
            Float32,
            '/planner/local_obstacle_clearance',
            10,
        )
        self.local_obstacle_score_pub = self.create_publisher(
            Float32,
            '/planner/local_obstacle_score',
            10,
        )
        self.local_cmd_pub = self.create_publisher(
            Twist,
            '/planner/local_recommended_cmd_vel',
            10,
        )
        self.raw_clearance_pub = self.create_publisher(
            Float32, '/planner/raw_obstacle_clearance', 10)
        self.obstacle_evidence_pub = self.create_publisher(
            String, '/planner/obstacle_evidence', 10)
        self.declare_parameter('steering_switch_margin', 0.04)

        hz = max(0.5, float(self.get_parameter('publish_hz').value))
        self.timer = self.create_timer(1.0 / hz, self.process)

    def score_map_callback(self, msg: Image) -> None:
        """Store terrain-aware score map."""
        self.score_map = msg
        self.score_map_received_at = time.monotonic()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Store camera calibration."""
        self.camera_info = msg

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """Store the latest depth point cloud."""
        self.pointcloud = msg
        self.pointcloud_received_at = time.monotonic()

    def odom_callback(self, msg: Odometry) -> None:
        """Store vehicle odometry."""
        self.odom = msg
        self.odom_received_at = time.monotonic()

    def path_callback(self, msg: Path) -> None:
        """Store global advisory path."""
        self.global_path = msg

    def state_callback(self, msg: String) -> None:
        """Store global navigation state."""
        self.navigation_state = msg.data

    def speed_callback(self, msg: Float32) -> None:
        """Store global recommended speed."""
        self.global_speed = max(0.0, float(msg.data))

    def mission_callback(self, msg: String) -> None:
        self.mission_state = msg.data

    def home_callback(self, msg: PoseStamped) -> None:
        self.home_pose = msg.pose

    def publish_stop(self, state: str) -> None:
        """Publish zero advisory motion."""
        state_msg = String()
        state_msg.data = state
        self.local_state_pub.publish(state_msg)

        angle_msg = Float32()
        angle_msg.data = 0.0
        self.local_angle_pub.publish(angle_msg)

        score_msg = Float32()
        score_msg.data = 0.0
        self.local_score_pub.publish(score_msg)

        obstacle_clearance_msg = Float32()
        obstacle_clearance_msg.data = 0.0
        self.local_obstacle_clearance_pub.publish(
            obstacle_clearance_msg
        )

        obstacle_score_msg = Float32()
        obstacle_score_msg.data = 0.0
        self.local_obstacle_score_pub.publish(
            obstacle_score_msg
        )

        cmd = Twist()
        self.local_cmd_pub.publish(cmd)

    def process(self) -> None:
        """Evaluate candidate local paths."""
        if self.mission_state == 'MISSION_COMPLETE':
            self.publish_stop('MISSION_COMPLETE')
            return
        if self.mission_state == 'FINAL_ALIGNMENT':
            self.process_final_alignment()
            return
        if self.navigation_state != 'NAVIGATE':
            self.publish_stop(self.navigation_state)
            return

        if (
            self.score_map is None
            or self.camera_info is None
            or self.odom is None
            or self.global_path is None
            or not self.global_path.poses
        ):
            self.publish_stop('WAITING_FOR_DATA')
            return

        path_frame = self.global_path.header.frame_id
        pose_frame = self.odom.header.frame_id
        if path_frame and pose_frame and path_frame != pose_frame:
            self.publish_stop('PATH_POSE_FRAME_MISMATCH')
            return

        if self.pointcloud is None:
            self.publish_stop('WAITING_FOR_DEPTH')
            return

        max_sensor_age = float(
            self.get_parameter('max_sensor_age_s').value
        )
        now = time.monotonic()
        for received_at, state in (
            (self.pointcloud_received_at, 'STALE_DEPTH'),
            (self.score_map_received_at, 'STALE_TRAVERSABILITY'),
            (self.odom_received_at, 'STALE_LOCALIZATION'),
        ):
            if not input_is_fresh(received_at, max_sensor_age, now):
                self.publish_stop(state)
                return

        image = image_to_array(self.score_map)
        if image is None:
            self.publish_stop('INVALID_SCORE_MAP')
            return

        pose = self.odom.pose.pose.position
        yaw = quaternion_to_yaw(
            self.odom.pose.pose.orientation
        )

        goal = self.global_path.poses[-1].pose.position
        goal_angle = goal_angle_from_pose(
            pose.x,
            pose.y,
            yaw,
            goal.x,
            goal.y,
        )

        max_angle = math.radians(
            float(self.get_parameter('max_angle_deg').value)
        )
        candidates = [
            max(
                -max_angle,
                min(max_angle, math.radians(deg)),
            )
            for deg in self.candidate_degrees
        ]

        points = pointcloud_to_array(
            self.pointcloud,
            int(self.get_parameter('pointcloud_stride').value),
        )

        if points is None:
            self.publish_stop('INVALID_DEPTH')
            return

        selected_angle, selected_score, selected_obstacle_score = (
            select_best_candidate(
                image,
                candidates,
                float(self.camera_info.k[0]),
                float(self.camera_info.k[2]),
                goal_angle,
                self.previous_angle,
                points,
                float(self.get_parameter('steering_switch_margin').value),
            )
        )

        evidence = obstacle_return_evidence(points, selected_angle)
        self.obstacle_evidence_pub.publish(String(data=json.dumps(evidence, sort_keys=True)))
        if evidence['raw_clearance_m'] is not None:
            self.raw_clearance_pub.publish(Float32(data=evidence['raw_clearance_m']))

        selected_clearance = obstacle_clearance(
            points,
            selected_angle,
        )

        self.previous_angle = selected_angle

        local_path = build_local_path(
            pose.x,
            pose.y,
            yaw,
            selected_angle,
            float(self.get_parameter('lookahead_m').value),
            12,
        )

        path_score = Float32()
        path_score.data = float(selected_score)
        self.local_score_pub.publish(path_score)

        clearance_msg = Float32()
        clearance_msg.data = float(
            min(selected_clearance, 3.0)
        )
        self.local_obstacle_clearance_pub.publish(
            clearance_msg
        )

        obstacle_score_msg = Float32()
        obstacle_score_msg.data = float(
            selected_obstacle_score
        )
        self.local_obstacle_score_pub.publish(
            obstacle_score_msg
        )

        angle_msg = Float32()
        angle_msg.data = math.degrees(selected_angle)
        self.local_angle_pub.publish(angle_msg)

        state_msg = String()
        state_msg.data = 'LOCAL_NAVIGATE'
        self.local_state_pub.publish(state_msg)

        self.local_path_pub.publish(local_path)

        # Stop advisory motion when the selected corridor is critically
        # obstructed.
        if selected_obstacle_score <= 0.01:
            state_msg = String()
            state_msg.data = 'LOCAL_STOP_OBSTACLE'
            self.local_state_pub.publish(state_msg)
            self.local_cmd_pub.publish(Twist())
            self.get_logger().warn(
                f'local obstacle stop: clearance='
                f'{min(selected_clearance, 3.0):.2f} m',
                throttle_duration_sec=2.0,
            )
            return

        max_angular = float(
            self.get_parameter('max_angular_speed').value
        )
        cmd = waypoint_command(
            goal_angle, selected_angle, self.global_speed, selected_obstacle_score,
            max_angle=max_angle, max_angular=max_angular)

        self.local_cmd_pub.publish(cmd)

        self.get_logger().info(
            f'local angle={math.degrees(selected_angle):.1f} deg '
            f'score={selected_score:.3f} '
            f'goal={math.degrees(goal_angle):.1f} deg '
            f'obstacle={min(selected_clearance, 3.0):.2f} m '
            f'obstacle_score={selected_obstacle_score:.3f} '
            f'speed={cmd.linear.x:.3f}',
            throttle_duration_sec=2.0,
        )

    def process_final_alignment(self) -> None:
        """Rotate in place to the captured startup heading with fresh inputs."""
        if self.odom is None or self.home_pose is None:
            self.publish_stop('WAITING_FOR_HOME_ALIGNMENT_DATA')
            return
        now = time.monotonic()
        age_limit = float(self.get_parameter('max_sensor_age_s').value)
        for received_at, name in ((self.odom_received_at, 'STALE_LOCALIZATION'),
                                  (self.pointcloud_received_at, 'STALE_DEPTH')):
            if not input_is_fresh(received_at, age_limit, now):
                self.publish_stop(name)
                return
        current = quaternion_to_yaw(self.odom.pose.pose.orientation)
        desired = quaternion_to_yaw(self.home_pose.orientation)
        error = wrap_angle(desired - current)
        cmd = Twist()
        if abs(error) > 0.06:
            cmd.angular.z = max(-0.35, min(0.35, 1.2 * error))
        self.local_state_pub.publish(String(data='LOCAL_NAVIGATE'))
        self.local_obstacle_score_pub.publish(Float32(data=1.0))
        self.local_obstacle_clearance_pub.publish(Float32(data=3.0))
        self.local_score_pub.publish(Float32(data=1.0))
        self.local_angle_pub.publish(Float32(data=0.0))
        self.local_cmd_pub.publish(cmd)


def main(args=None) -> None:
    """Run local path selector."""
    rclpy.init(args=args)
    node = LocalPathSelector()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
