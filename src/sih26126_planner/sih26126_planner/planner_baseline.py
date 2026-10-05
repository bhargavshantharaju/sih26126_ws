#!/usr/bin/env python3
"""Terrain-aware advisory planner baseline for DRISTHI."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a numeric value to the specified range."""
    return max(low, min(high, value))


def score_to_speed(
    score: float,
    max_speed_mps: float = 0.30,
    stop_threshold: float = 0.30,
    slow_threshold: float = 0.70,
) -> float:
    """Convert traversability score to a speed recommendation."""
    score = clamp(float(score), 0.0, 1.0)

    if score < stop_threshold:
        return 0.0

    if score < slow_threshold:
        fraction = (
            score - stop_threshold
        ) / (
            slow_threshold - stop_threshold
        )
        return max_speed_mps * 0.5 * fraction

    fraction = (
        score - slow_threshold
    ) / (
        1.0 - slow_threshold
    )

    return max_speed_mps * (
        0.5 + 0.5 * fraction
    )


def planner_state(
    score: float,
    stop_threshold: float = 0.30,
    slow_threshold: float = 0.70,
) -> str:
    """Return the advisory motion state for a traversability score."""
    score = clamp(float(score), 0.0, 1.0)

    if score < stop_threshold:
        return 'STOP'

    if score < slow_threshold:
        return 'SLOW'

    return 'FORWARD'


class PlannerBaseline(Node):
    """
    Terrain-aware advisory planner baseline.

    It consumes the terrain-aware traversability score and publishes
    an advisory state, speed, and Twist recommendation.

    The node never publishes to /cmd_vel. It remains advisory until
    a later safety and control stage is explicitly connected.
    """

    def __init__(self) -> None:
        """Initialize the planner node and its ROS interfaces."""
        super().__init__('planner_baseline')

        self.declare_parameter(
            'input_topic',
            '/traversability/terrain_aware_mean_score',
        )
        self.declare_parameter(
            'publish_hz',
            5.0,
        )
        self.declare_parameter(
            'max_speed_mps',
            0.30,
        )
        self.declare_parameter(
            'stop_threshold',
            0.30,
        )
        self.declare_parameter(
            'slow_threshold',
            0.70,
        )
        self.declare_parameter(
            'max_input_age_s',
            1.0,
        )

        input_topic = str(
            self.get_parameter('input_topic').value
        )

        self.publish_hz = float(
            self.get_parameter('publish_hz').value
        )
        self.max_speed_mps = float(
            self.get_parameter('max_speed_mps').value
        )
        self.stop_threshold = float(
            self.get_parameter('stop_threshold').value
        )
        self.slow_threshold = float(
            self.get_parameter('slow_threshold').value
        )
        self.max_input_age_s = float(
            self.get_parameter('max_input_age_s').value
        )

        self.latest_score: Optional[float] = None
        self.score_stamp_ns = 0
        self.frames = 0

        self.sub_score = self.create_subscription(
            Float32,
            input_topic,
            self.score_callback,
            10,
        )

        self.pub_cmd = self.create_publisher(
            Twist,
            '/planner/recommended_cmd_vel',
            10,
        )

        self.pub_state = self.create_publisher(
            String,
            '/planner/state',
            10,
        )

        self.pub_speed = self.create_publisher(
            Float32,
            '/planner/recommended_speed',
            10,
        )

        period = 1.0 / max(self.publish_hz, 0.1)

        self.timer = self.create_timer(
            period,
            self.process,
        )

        self.get_logger().info(
            'Planner baseline started. '
            f'Input={input_topic} '
            f'max_speed={self.max_speed_mps:.2f} m/s '
            'output is advisory only.'
        )

    def now_ns(self) -> int:
        """Return the current ROS clock time in nanoseconds."""
        return int(
            self.get_clock().now().nanoseconds
        )

    def score_callback(self, msg: Float32) -> None:
        """Store the latest finite terrain-aware score."""
        value = float(msg.data)

        if not math.isfinite(value):
            self.latest_score = None
            return

        self.latest_score = clamp(
            value,
            0.0,
            1.0,
        )
        self.score_stamp_ns = self.now_ns()

    def score_is_fresh(self) -> bool:
        """Return whether the latest traversability score is fresh."""
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
        """Publish the current advisory planner decision."""
        if not self.score_is_fresh():
            state = 'WAITING'
            speed = 0.0
        else:
            score = float(self.latest_score)

            state = planner_state(
                score,
                self.stop_threshold,
                self.slow_threshold,
            )

            speed = score_to_speed(
                score,
                self.max_speed_mps,
                self.stop_threshold,
                self.slow_threshold,
            )

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)

        state_msg = String()
        state_msg.data = state
        self.pub_state.publish(state_msg)

        speed_msg = Float32()
        speed_msg.data = float(
            clamp(
                speed,
                0.0,
                self.max_speed_mps,
            )
        )
        self.pub_speed.publish(speed_msg)

        self.frames += 1

        if self.frames % 10 == 0:
            score_text = (
                f'{self.latest_score:.3f}'
                if self.latest_score is not None
                else 'none'
            )

            self.get_logger().info(
                f'frames={self.frames} '
                f'terrain_aware_score={score_text} '
                f'state={state} '
                f'recommended_speed={speed:.3f} m/s'
            )


def main(args=None) -> None:
    """Run the planner baseline ROS node."""
    rclpy.init(args=args)

    node = PlannerBaseline()

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
