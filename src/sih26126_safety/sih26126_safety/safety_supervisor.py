"""Safety supervisor for advisory planner commands."""

import math
import time
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String


def finite(value: float) -> bool:
    """Return whether a numeric value is finite."""
    return math.isfinite(float(value))


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a numeric value to a closed interval."""
    return max(lower, min(upper, value))


def sanitize_command(
    command: Twist,
    max_linear: float = 0.30,
    max_angular: float = 0.60,
) -> Tuple[Twist, bool]:
    """Clamp a Twist and report whether its values are valid."""
    values = (
        command.linear.x,
        command.linear.y,
        command.linear.z,
        command.angular.x,
        command.angular.y,
        command.angular.z,
    )

    if not all(finite(value) for value in values):
        return Twist(), False

    output = Twist()

    output.linear.x = clamp(
        command.linear.x,
        -max_linear,
        max_linear,
    )
    output.linear.y = 0.0
    output.linear.z = 0.0

    output.angular.x = 0.0
    output.angular.y = 0.0
    output.angular.z = clamp(
        command.angular.z,
        -max_angular,
        max_angular,
    )

    return output, True


def safety_decision(
    navigation_state: Optional[str],
    obstacle_score: Optional[float],
    command: Twist,
    command_age: float,
    state_age: float,
    obstacle_age: float,
    timeout: float = 0.75,
    obstacle_stop_threshold: float = 0.01,
) -> Tuple[Twist, str]:
    """Determine whether an advisory command may reach /cmd_vel."""
    zero = Twist()

    if navigation_state is None:
        return zero, 'WAITING_FOR_NAVIGATION_STATE'

    if state_age > timeout:
        return zero, 'STALE_NAVIGATION_STATE'

    if navigation_state == 'LOCAL_STOP_OBSTACLE':
        return zero, 'OBSTACLE_STOP'

    if navigation_state != 'LOCAL_NAVIGATE':
        return zero, 'NAVIGATION_NOT_ACTIVE'

    if command_age > timeout:
        return zero, 'STALE_COMMAND'

    if obstacle_score is None:
        return zero, 'WAITING_FOR_OBSTACLE_SCORE'

    if obstacle_age > timeout:
        return zero, 'STALE_OBSTACLE_SCORE'

    if not finite(obstacle_score):
        return zero, 'INVALID_OBSTACLE_SCORE'

    if obstacle_score <= obstacle_stop_threshold:
        return zero, 'OBSTACLE_STOP'

    safe_command, valid = sanitize_command(command)

    if not valid:
        return zero, 'INVALID_COMMAND'

    return safe_command, 'SAFE_COMMAND'


class SafetySupervisor(Node):
    """Gate advisory planner commands before vehicle actuation."""

    def __init__(self) -> None:
        super().__init__('safety_supervisor')

        self.command: Optional[Twist] = None
        self.navigation_state: Optional[str] = None
        self.obstacle_score: Optional[float] = None

        self.command_time = None
        self.state_time = None
        self.obstacle_time = None

        self.declare_parameter('timeout_sec', 0.75)
        self.declare_parameter('obstacle_stop_threshold', 0.01)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 0.60)
        self.declare_parameter('publish_hz', 10.0)

        self.command_sub = self.create_subscription(
            Twist,
            '/planner/local_recommended_cmd_vel',
            self.command_callback,
            10,
        )

        self.state_sub = self.create_subscription(
            String,
            '/planner/local_navigation_state',
            self.state_callback,
            10,
        )

        self.obstacle_sub = self.create_subscription(
            Float32,
            '/planner/local_obstacle_score',
            self.obstacle_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            '/safety/state',
            10,
        )

        self.reason_pub = self.create_publisher(
            String,
            '/safety/reason',
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(
                1.0,
                float(self.get_parameter('publish_hz').value),
            ),
            self.process,
        )

    def command_callback(self, msg: Twist) -> None:
        """Store the latest advisory command."""
        self.command = msg
        self.command_time = time.monotonic()

    def state_callback(self, msg: String) -> None:
        """Store the latest local navigation state."""
        self.navigation_state = msg.data
        self.state_time = time.monotonic()

    def obstacle_callback(self, msg: Float32) -> None:
        """Store the latest obstacle score."""
        self.obstacle_score = float(msg.data)
        self.obstacle_time = time.monotonic()

    def age_seconds(self, timestamp) -> float:
        """Return age of a received message using monotonic time."""
        if timestamp is None:
            return float('inf')
        return max(0.0, time.monotonic() - timestamp)

    def publish_safety_status(self, state: str, reason: str) -> None:
        """Publish current safety state and reason."""
        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)

        reason_msg = String()
        reason_msg.data = reason
        self.reason_pub.publish(reason_msg)

    def process(self) -> None:
        """Evaluate and publish the current safe command."""
        timeout = float(
            self.get_parameter('timeout_sec').value
        )
        obstacle_threshold = float(
            self.get_parameter('obstacle_stop_threshold').value
        )
        max_linear = float(
            self.get_parameter('max_linear_speed').value
        )
        max_angular = float(
            self.get_parameter('max_angular_speed').value
        )

        if self.command is None:
            command = Twist()
        else:
            command = self.command

        command_age = self.age_seconds(self.command_time)
        state_age = self.age_seconds(self.state_time)
        obstacle_age = self.age_seconds(self.obstacle_time)

        safe_command, reason = safety_decision(
            self.navigation_state,
            self.obstacle_score,
            command,
            command_age,
            state_age,
            obstacle_age,
            timeout=timeout,
            obstacle_stop_threshold=obstacle_threshold,
        )

        # Apply configured limits one more time at the node boundary.
        safe_command.linear.x = clamp(
            safe_command.linear.x,
            -max_linear,
            max_linear,
        )
        safe_command.angular.z = clamp(
            safe_command.angular.z,
            -max_angular,
            max_angular,
        )

        if reason == 'SAFE_COMMAND':
            safety_state = 'SAFE'
        else:
            safe_command = Twist()
            safety_state = 'STOPPED'

        self.cmd_pub.publish(safe_command)
        self.publish_safety_status(
            safety_state,
            reason,
        )

        self.get_logger().info(
            f'safety={safety_state} '
            f'reason={reason} '
            f'linear={safe_command.linear.x:.3f} '
            f'angular={safe_command.angular.z:.3f}',
            throttle_duration_sec=2.0,
        )


def main(args=None) -> None:
    """Run the safety supervisor."""
    rclpy.init(args=args)
    node = SafetySupervisor()

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
