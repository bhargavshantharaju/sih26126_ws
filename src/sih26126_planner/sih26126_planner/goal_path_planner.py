#!/usr/bin/env python3
"""Goal-directed terrain-aware advisory path planner."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Twist

from nav_msgs.msg import Odometry, Path

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
            6.0,
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
        if not self.odom_is_fresh():
            self.publish_state(
                'WAITING_POSE'
            )
            self.publish_zero_outputs()
            return

        distance = distance_to_goal(
            self.current_x,
            self.current_y,
            self.goal_x,
            self.goal_y,
        )

        path = build_straight_path(
            self.current_x,
            self.current_y,
            self.goal_x,
            self.goal_y,
            self.odom_frame,
            self.get_clock().now().to_msg(),
            self.path_resolution_m,
        )

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
            self.publish_state(
                'WAITING_TERRAIN'
            )
            self.publish_zero_outputs()
            return

        target_heading = math.atan2(
            self.goal_y - self.current_y,
            self.goal_x - self.current_x,
        )

        heading_error = wrap_angle(
            target_heading - self.current_yaw
        )

        base_speed = score_to_speed(
            self.latest_score,
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
                f'pose=({self.current_x:.2f},'
                f'{self.current_y:.2f}) '
                f'distance={distance:.2f} m '
                f'terrain_score={self.latest_score:.3f} '
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
