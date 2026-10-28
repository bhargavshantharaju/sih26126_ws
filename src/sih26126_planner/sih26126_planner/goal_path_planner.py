#!/usr/bin/env python3
"""Goal-directed terrain-aware advisory path planner."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PointStamped, PoseStamped, Twist

from nav_msgs.msg import OccupancyGrid, Odometry, Path

import rclpy
from rclpy.node import Node

from sih26126_planner.planner_baseline import score_to_speed

from std_msgs.msg import Float32, String


def quaternion_to_yaw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    """Convert a quaternion orientation to planar yaw."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(
        sin_yaw,
        cos_yaw,
    )


def wrap_angle(angle: float) -> float:
    """Wrap an angle into the range [-pi, pi]."""
    wrapped = (
        (angle + math.pi)
        % (2.0 * math.pi)
    ) - math.pi

    if math.isclose(
        wrapped,
        -math.pi,
        abs_tol=1e-12,
    ):
        return math.pi

    return wrapped


def distance_to_goal(
    x: float,
    y: float,
    goal_x: float,
    goal_y: float,
) -> float:
    """Return planar distance from the current pose to the goal."""
    return math.hypot(
        goal_x - x,
        goal_y - y,
    )


def navigation_state(
    distance_m: float,
    goal_tolerance_m: float,
) -> str:
    """Return GOAL_REACHED or NAVIGATE for the current distance."""
    if distance_m <= goal_tolerance_m:
        return 'GOAL_REACHED'

    return 'NAVIGATE'


def route_lookahead_target(
    path: Path,
    x: float,
    y: float,
    lookahead_m: float = 0.85,
):
    """Choose a point ahead along the supplied map route, not its distant endpoint."""
    if not path.poses:
        return None
    nearest = min(range(len(path.poses)), key=lambda i: math.hypot(
        path.poses[i].pose.position.x - x, path.poses[i].pose.position.y - y))
    selected = nearest
    distance = 0.0
    for index in range(nearest + 1, len(path.poses)):
        previous = path.poses[index - 1].pose.position
        current = path.poses[index].pose.position
        distance += math.hypot(current.x - previous.x, current.y - previous.y)
        selected = index
        if distance >= max(0.0, lookahead_m):
            break
    return path.poses[selected].pose.position


def build_straight_path(
    x: float,
    y: float,
    goal_x: float,
    goal_y: float,
    frame_id: str,
    stamp,
    resolution_m: float,
) -> Path:
    """Build a straight-line path from the pose to the goal."""
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = frame_id

    dx = goal_x - x
    dy = goal_y - y
    distance = math.hypot(dx, dy)

    if distance <= 0.0:
        sample_count = 1
        heading = 0.0
    else:
        sample_count = max(
            2,
            int(
                math.ceil(
                    distance / max(resolution_m, 0.05)
                )
            ) + 1,
        )
        heading = math.atan2(dy, dx)

    for index in range(sample_count):
        if sample_count == 1:
            fraction = 0.0
        else:
            fraction = index / float(
                sample_count - 1
            )

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id

        pose.pose.position.x = (
            x + dx * fraction
        )
        pose.pose.position.y = (
            y + dy * fraction
        )
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = math.sin(
            heading * 0.5
        )
        pose.pose.orientation.w = math.cos(
            heading * 0.5
        )

        path.poses.append(pose)

    return path


def path_cost_score(
    path: Path,
    cost_map: Optional[OccupancyGrid],
) -> float:
    """
    Return average cost-map weight along *path* (0.0 safe - 1.0 lethal).

    Each waypoint along the supplied ``path`` is projected onto the cost map
    grid and the cell value is read.  The returned score is the mean of those
    normalised cell values, clamped to ``[0, 1]``.

    A value of ``0.0`` means the path runs entirely through free/low-cost
    cells; ``1.0`` means every sampled cell is near-lethal (>= 99).

    If *cost_map* is ``None`` or empty, a neutral ``0.0`` is returned so the
    planner can operate without a cost map (graceful degradation).

    Cells outside the map footprint receive a conservative cost of 0.5
    (unknown), matching the ``cost_map_fusion`` convention.

    Occupied cells in the cost map (>= 99) contribute a full 1.0 penalty;
    unknown cells (-1) contribute 0.5.
    """
    if cost_map is None:
        return 0.0
    if not cost_map.data:
        return 0.0
    if not path.poses:
        return 0.0

    info = cost_map.info
    width = info.width
    height = info.height
    resolution = info.resolution
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return 0.0

    origin_x = float(info.origin.position.x)
    origin_y = float(info.origin.position.y)

    cost_values = []
    for pose_stamped in path.poses:
        wx = float(pose_stamped.pose.position.x)
        wy = float(pose_stamped.pose.position.y)

        col = int((wx - origin_x) / resolution)
        row = int((wy - origin_y) / resolution)

        if not (0 <= col < width and 0 <= row < height):
            # Outside the cost-map footprint — treat as unknown (0.5)
            cost_values.append(0.5)
            continue

        idx = row * width + col
        cell = int(cost_map.data[idx])

        if cell < 0:
            # Unknown cell in the cost map — conservative 0.5
            cost_values.append(0.5)
        elif cell >= 99:
            # Near-lethal — full penalty
            cost_values.append(1.0)
        else:
            # Normalise 0-100 to 0.0-1.0
            cost_values.append(cell / 100.0)

    if not cost_values:
        return 0.0

    return max(0.0, min(1.0, sum(cost_values) / len(cost_values)))


class GoalPathPlanner(Node):
    """Generate a goal-directed advisory path and Twist."""

    def __init__(self) -> None:
        """Initialize planner parameters, subscriptions, and publishers."""
        super().__init__('goal_path_planner')

        self.declare_parameter(
            'odom_topic',
            '/model/sih_ugv/odometry',
        )
        self.declare_parameter(
            'terrain_score_topic',
            '/traversability/terrain_aware_mean_score',
        )
        self.declare_parameter(
            'publish_hz',
            5.0,
        )
        self.declare_parameter(
            'goal_x',
            0.0,
        )
        self.declare_parameter(
            'goal_y',
            0.0,
        )
        self.declare_parameter(
            'goal_tolerance_m',
            0.25,
        )
        self.declare_parameter(
            'path_resolution_m',
            0.25,
        )
        self.declare_parameter(
            'max_speed_mps',
            0.30,
        )
        self.declare_parameter(
            'max_angular_speed_rps',
            0.80,
        )
        self.declare_parameter(
            'heading_gain',
            1.50,
        )
        self.declare_parameter(
            'max_input_age_s',
            1.0,
        )
        self.declare_parameter('route_lookahead_m', 0.85)
        self.declare_parameter('slowdown_distance_m', 1.5)
        self.declare_parameter('minimum_near_goal_speed_ratio', 0.25)
        # After this many seconds without a terrain score, use a conservative
        # fallback score (0.5) rather than blocking indefinitely at WAITING_TERRAIN.
        # This prevents a camera/traversability pipeline delay from permanently
        # halting navigation when the robot can otherwise move safely.
        self.declare_parameter('terrain_timeout_fallback_sec', 8.0)
        self.declare_parameter('terrain_fallback_score', 0.5)

        odom_topic = str(
            self.get_parameter('odom_topic').value
        )
        terrain_topic = str(
            self.get_parameter(
                'terrain_score_topic'
            ).value
        )

        self.publish_hz = float(
            self.get_parameter('publish_hz').value
        )
        self.goal_x = float(
            self.get_parameter('goal_x').value
        )
        self.goal_y = float(
            self.get_parameter('goal_y').value
        )
        self.goal_tolerance_m = float(
            self.get_parameter(
                'goal_tolerance_m'
            ).value
        )
        self.path_resolution_m = float(
            self.get_parameter(
                'path_resolution_m'
            ).value
        )
        self.max_speed_mps = float(
            self.get_parameter(
                'max_speed_mps'
            ).value
        )
        self.max_angular_speed_rps = float(
            self.get_parameter(
                'max_angular_speed_rps'
            ).value
        )
        self.heading_gain = float(
            self.get_parameter(
                'heading_gain'
            ).value
        )
        self.max_input_age_s = float(
            self.get_parameter(
                'max_input_age_s'
            ).value
        )

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None
        self.odom_frame = 'odom'
        self.odom_stamp_ns = 0

        self.latest_score: Optional[float] = None
        self.score_stamp_ns = 0

        # Track when we first started waiting for terrain so we can apply a
        # fallback after terrain_timeout_fallback_sec without a real score.
        self._terrain_wait_start_ns: int = 0

        self.frames = 0

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        self.score_sub = self.create_subscription(
            Float32,
            terrain_topic,
            self.score_callback,
            10,
        )

        self.exploration_goal_sub = self.create_subscription(
            PointStamped,
            '/exploration/goal',
            self.exploration_goal_callback,
            10,
        )
        self.mission_goal_sub = self.create_subscription(
            PointStamped, '/mission/goal', self.mission_goal_callback, 10)
        self.mission_state_sub = self.create_subscription(
            String, '/mission/state', self.mission_state_callback, 10)
        self.global_path_sub = self.create_subscription(
            Path, '/planner/global_path', self.global_path_callback, 10)
        self.mission_state = ''
        self.global_path: Optional[Path] = None
        self.has_goal = False

        # Subscribe to the localization pose (map frame when SLAM is active).
        # This allows the planner to follow a global path that is expressed in
        # the map frame rather than the raw Gazebo odom frame, which would
        # produce a PATH_FRAME_MISMATCH when slam:=true.
        self.localization_pose_sub = self.create_subscription(
            PoseStamped, '/localization/pose', self.localization_pose_callback, 10)
        self.localization_x: Optional[float] = None
        self.localization_y: Optional[float] = None
        self.localization_yaw: Optional[float] = None
        self.localization_frame = 'odom'
        self.localization_stamp_ns = 0

        # Cost map from cost_map_fusion — OccupancyGrid with values 0-100
        # representing fused occupancy + terrain + localization cost.
        # Used for risk-aware speed modulation along the planned path.
        self.cost_map: Optional[OccupancyGrid] = None
        self.cost_map_sub = self.create_subscription(
            OccupancyGrid, '/map/cost_map', self.cost_map_callback, 10)

        self.path_pub = self.create_publisher(
            Path,
            '/planner/path',
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            '/planner/navigation_state',
            10,
        )

        self.distance_pub = self.create_publisher(
            Float32,
            '/planner/distance_to_goal',
            10,
        )

        self.speed_pub = self.create_publisher(
            Float32,
            '/planner/recommended_speed',
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/planner/recommended_cmd_vel',
            10,
        )

        timer_period = 1.0 / max(
            self.publish_hz,
            0.1,
        )

        self.timer = self.create_timer(
            timer_period,
            self.process,
        )

        self.get_logger().info(
            'Goal path planner started. '
            f'goal=({self.goal_x:.2f}, {self.goal_y:.2f}) '
            f'odom={odom_topic} '
            f'terrain={terrain_topic} '
            'advisory-only.'
        )

    def exploration_goal_callback(self, msg: PointStamped) -> None:
        """Track the active map-derived exploration frontier."""
        if self.mission_state == 'RETURNING_HOME':
            return
        goal_x = float(msg.point.x)
        goal_y = float(msg.point.y)
        if not (math.isfinite(goal_x) and math.isfinite(goal_y)):
            return
        changed = math.hypot(goal_x - self.goal_x, goal_y - self.goal_y) > 0.15
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.has_goal = True
        if changed:
            self.get_logger().info(
                f'Exploration frontier goal=({goal_x:.2f}, {goal_y:.2f})'
            )

    def mission_goal_callback(self, msg: PointStamped) -> None:
        """Use the mission controller's home goal during return."""
        if self.mission_state == 'RETURNING_HOME':
            self.goal_x = float(msg.point.x)
            self.goal_y = float(msg.point.y)
            self.has_goal = True

    def mission_state_callback(self, msg: String) -> None:
        self.mission_state = msg.data

    def global_path_callback(self, msg: Path) -> None:
        self.global_path = msg

    def localization_pose_callback(self, msg: PoseStamped) -> None:
        """Store the localization pose (map frame when SLAM is active)."""
        pos = msg.pose.position
        ori = msg.pose.orientation
        values = (pos.x, pos.y, ori.x, ori.y, ori.z, ori.w)
        if not all(math.isfinite(float(v)) for v in values):
            return
        self.localization_x = float(pos.x)
        self.localization_y = float(pos.y)
        self.localization_yaw = quaternion_to_yaw(
            float(ori.x), float(ori.y), float(ori.z), float(ori.w))
        frame_id = msg.header.frame_id
        if frame_id:
            self.localization_frame = frame_id
        self.localization_stamp_ns = self.now_ns()

    def now_ns(self) -> int:
        """Return current ROS time in nanoseconds."""
        return int(
            self.get_clock().now().nanoseconds
        )

    def odom_callback(
        self,
        msg: Odometry,
    ) -> None:
        """Store the latest odometry pose."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        values = (
            position.x,
            position.y,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        if not all(
            math.isfinite(float(value))
            for value in values
        ):
            return

        self.current_x = float(position.x)
        self.current_y = float(position.y)

        self.current_yaw = quaternion_to_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

        frame_id = msg.header.frame_id

        if frame_id:
            self.odom_frame = frame_id

        self.odom_stamp_ns = self.now_ns()

    def score_callback(
        self,
        msg: Float32,
    ) -> None:
        """Store the latest finite terrain-aware score."""
        value = float(msg.data)

        if not math.isfinite(value):
            self.latest_score = None
            return

        self.latest_score = max(
            0.0,
            min(1.0, value),
        )
        self.score_stamp_ns = self.now_ns()
        # Reset the fallback wait timer whenever a real score arrives.
        self._terrain_wait_start_ns = 0

    def cost_map_callback(self, msg: OccupancyGrid) -> None:
        """Store the latest fused cost map from cost_map_fusion."""
        self.cost_map = msg

    def odom_is_fresh(self) -> bool:
        """Return whether a recent odometry pose is available."""
        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
        ):
            return False

        if self.odom_stamp_ns <= 0:
            return False

        age_s = (
            self.now_ns() - self.odom_stamp_ns
        ) / 1e9

        return (
            0.0 <= age_s <= self.max_input_age_s
        )

    def terrain_is_fresh(self) -> bool:
        """Return whether a recent terrain score is available."""
        if self.latest_score is None:
            return False

        if self.score_stamp_ns <= 0:
            return False

        age_s = (
            self.now_ns() - self.score_stamp_ns
        ) / 1e9

        return (
            0.0 <= age_s <= self.max_input_age_s
        )

    def process(self) -> None:
        """Publish the current path and advisory motion decision."""
        if self.mission_state in ('MISSION_COMPLETE', 'SAFE_STOP', 'FAILED'):
            self.publish_state(self.mission_state)
            self.publish_zero_outputs()
            return
        if not self.has_goal:
            self.publish_state('WAITING_FOR_EXPLORATION_GOAL')
            self.publish_zero_outputs()
            return
        if (self.mission_state == 'RETURNING_HOME'
                and (self.global_path is None or not self.global_path.poses)):
            self.publish_state('WAITING_HOME_ROUTE')
            self.publish_zero_outputs()
            return
        if (self.mission_state == 'EXPLORING'
                and (self.global_path is None or not self.global_path.poses)):
            self.publish_state('WAITING_FOR_MAP_ROUTE')
            self.publish_zero_outputs()
            return

        # Select the pose source that is consistent with the global path frame.
        # When SLAM is active the global path is in 'map' frame; the localization
        # pose adapter also publishes in 'map' frame.  Using it here ensures that
        # the robot's position and the path coordinates are in the same reference
        # frame so distance/heading calculations are correct.
        path_frame = (self.global_path.header.frame_id
                      if self.global_path is not None and self.global_path.header.frame_id
                      else self.odom_frame)
        loc_frame_matches = (
            self.localization_x is not None
            and self.localization_frame == path_frame)
        _loc_age_s = (self.now_ns() - self.localization_stamp_ns) / 1e9
        use_localization = loc_frame_matches and (
            0.0 <= _loc_age_s <= self.max_input_age_s)

        if use_localization:
            current_x = self.localization_x
            current_y = self.localization_y
            current_yaw = self.localization_yaw
            pose_frame = self.localization_frame
        else:
            if not self.odom_is_fresh():
                self.publish_state('WAITING_POSE')
                self.publish_zero_outputs()
                return
            current_x = self.current_x
            current_y = self.current_y
            current_yaw = self.current_yaw
            pose_frame = self.odom_frame

        if (self.global_path is not None and self.global_path.poses
                and self.global_path.header.frame_id
                and pose_frame
                and self.global_path.header.frame_id != pose_frame):
            self.publish_state('PATH_FRAME_MISMATCH')
            self.publish_zero_outputs()
            return

        distance = distance_to_goal(
            current_x,
            current_y,
            self.goal_x,
            self.goal_y,
        )

        path = build_straight_path(
            current_x,
            current_y,
            self.goal_x,
            self.goal_y,
            pose_frame,
            self.get_clock().now().to_msg(),
            self.path_resolution_m,
        )

        # Follow the accumulated occupancy-grid route when available. The
        # local selector receives the next map-derived waypoint as its goal.
        if self.global_path is not None and self.global_path.poses:
            target = route_lookahead_target(
                self.global_path, current_x, current_y,
                float(self.get_parameter('route_lookahead_m').value))
            if target is None:
                self.publish_state('WAITING_FOR_MAP_ROUTE')
                self.publish_zero_outputs()
                return
            path = build_straight_path(
                current_x, current_y, target.x, target.y,
                pose_frame, self.get_clock().now().to_msg(),
                self.path_resolution_m)

        self.path_pub.publish(path)

        distance_msg = Float32()
        distance_msg.data = float(distance)
        self.distance_pub.publish(distance_msg)

        state = navigation_state(
            distance,
            self.goal_tolerance_m,
        )

        if state == 'GOAL_REACHED':
            self.publish_state(state)
            self.publish_zero_outputs()
            return

        if not self.terrain_is_fresh():
            # Start timing how long we have been without terrain data.
            if self._terrain_wait_start_ns == 0:
                self._terrain_wait_start_ns = self.now_ns()
            wait_elapsed_s = (self.now_ns() - self._terrain_wait_start_ns) / 1e9
            fallback_timeout = float(
                self.get_parameter('terrain_timeout_fallback_sec').value)
            if wait_elapsed_s < fallback_timeout:
                self.publish_state('WAITING_TERRAIN')
                self.publish_zero_outputs()
                return
            # Terrain pipeline has been absent too long — use a conservative
            # fallback score so navigation is not permanently blocked.
            fallback = float(self.get_parameter('terrain_fallback_score').value)
            effective_score = max(0.0, min(1.0, fallback))
            self.get_logger().warn(
                f'Terrain score absent for {wait_elapsed_s:.1f}s; '
                f'using fallback score={effective_score:.2f}',
                throttle_duration_sec=10.0,
            )
        else:
            effective_score = self.latest_score
            self._terrain_wait_start_ns = 0  # reset timer once terrain is fresh

        target_heading = math.atan2(
            self.goal_y - current_y,
            self.goal_x - current_x,
        )

        heading_error = wrap_angle(
            target_heading - current_yaw
        )

        base_speed = score_to_speed(
            effective_score,
            self.max_speed_mps,
            0.30,
            0.70,
        )

        heading_factor = max(
            0.0,
            1.0 - min(
                abs(heading_error) / math.pi,
                1.0,
            ),
        )

        linear_speed = base_speed * (
            0.25 + 0.75 * heading_factor
        )

        # Risk-aware speed modulation: reduce speed proportionally when the
        # fused cost map indicates elevated risk along the planned path.
        # The frontier explorer's cost-aware Dijkstra already selects routes
        # that avoid high-cost cells; this modulation provides an additional
        # safety margin by slowing down when the robot is *on* a costly
        # segment (e.g. the only available route passes through medium-cost
        # terrain or a poorly-localised area).
        cost_penalty = path_cost_score(path, self.cost_map)
        if cost_penalty > 0.0:
            # Scale speed by (1 - cost_penalty): cost=0 → no reduction,
            # cost=0.5 → 50% speed, cost=1.0 → full stop.
            linear_speed *= max(0.0, 1.0 - cost_penalty)

        if self.mission_state == 'RETURNING_HOME':
            slowdown_distance = max(0.1, float(
                self.get_parameter('slowdown_distance_m').value))
            minimum_ratio = max(0.0, min(1.0, float(
                self.get_parameter('minimum_near_goal_speed_ratio').value)))
            near_goal_ratio = minimum_ratio + (1.0 - minimum_ratio) * min(
                1.0, distance / slowdown_distance)
            linear_speed *= near_goal_ratio

        angular_speed = max(
            -self.max_angular_speed_rps,
            min(
                self.max_angular_speed_rps,
                self.heading_gain * heading_error,
            ),
        )

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self.cmd_pub.publish(cmd)

        speed_msg = Float32()
        speed_msg.data = float(
            max(
                0.0,
                min(
                    self.max_speed_mps,
                    linear_speed,
                ),
            )
        )
        self.speed_pub.publish(speed_msg)

        self.publish_state(state)

        self.frames += 1

        if self.frames % 10 == 0:
            self.get_logger().info(
                f'frames={self.frames} '
                f'pose=({current_x:.2f},'
                f'{current_y:.2f}) '
                f'distance={distance:.2f} m '
                f'terrain_score={effective_score:.3f} '
                f'cost_map_penalty={cost_penalty:.3f} '
                f'state={state} '
                f'speed={linear_speed:.3f} m/s '
                f'yaw_rate={angular_speed:.3f} rad/s'
            )

    def publish_state(
        self,
        state: str,
    ) -> None:
        """Publish a navigation state string."""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def publish_zero_outputs(self) -> None:
        """Publish zero advisory motion while waiting or stopped."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

        speed = Float32()
        speed.data = 0.0
        self.speed_pub.publish(speed)


def main(args=None) -> None:
    """Run the goal-directed path planner."""
    rclpy.init(args=args)

    node = GoalPathPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
