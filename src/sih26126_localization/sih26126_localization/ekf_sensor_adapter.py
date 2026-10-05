#!/usr/bin/env python3
# Copyright 2026 DRISTHI Contributors
# Licensed under the Apache License, Version 2.0
"""Relay zero-covariance Gazebo sensor messages with explicit uncertainty."""

from copy import deepcopy

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _covariance(size: int, diagonal: dict[int, float]) -> list[float]:
    matrix = [0.0] * (size * size)
    for axis in range(size):
        matrix[axis * size + axis] = diagonal.get(axis, 1.0)
    return matrix


class EkfSensorAdapter(Node):
    """Add modeled measurement uncertainty without altering sensor values."""

    def __init__(self) -> None:
        super().__init__('ekf_sensor_adapter')
        self._odom_pub = self.create_publisher(
            Odometry, '/fusion/wheel_odometry', 10
        )
        self._imu_pub = self.create_publisher(Imu, '/fusion/imu', 10)
        self.declare_parameter('wheel_input_enabled', True)
        self.create_subscription(
            Odometry, '/model/sih_ugv/odometry', self._odom_callback, 10
        )
        self.create_subscription(Imu, '/imu', self._imu_callback, 10)

    def _odom_callback(self, msg: Odometry) -> None:
        # This explicit gate is intended for paired sensor-contribution
        # validation. Production launch defaults to enabled. Raw wheel
        # odometry remains observable, but disabled messages never reach EKF.
        if not self.get_parameter('wheel_input_enabled').value:
            return
        adapted = deepcopy(msg)
        adapted.twist.covariance = _covariance(6, {0: 0.01, 5: 0.04})
        self._odom_pub.publish(adapted)

    def _imu_callback(self, msg: Imu) -> None:
        adapted = deepcopy(msg)
        adapted.orientation_covariance = _covariance(3, {2: 0.0025})
        adapted.angular_velocity_covariance = _covariance(3, {2: 0.001})
        adapted.linear_acceleration_covariance = _covariance(3, {})
        self._imu_pub.publish(adapted)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfSensorAdapter()
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
