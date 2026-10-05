#!/usr/bin/env python3
"""DRISTHI LiDAR health and freshness monitor.

Monitors /lidar/scan_2d (produced by lidar_processor from the 3D LiDAR
PointCloud2 data) for health, freshness, and data validity.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sih26126_msgs.msg import SensorHealth


def stamp_to_seconds(stamp) -> float:
    """Convert ROS time to seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class LidarMonitor(Node):
    """Monitor 2D LiDAR scan health, freshness, and data validity."""

    def __init__(self):
        super().__init__('lidar_monitor')

        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('stale_threshold_sec', 0.5)
        self.declare_parameter('min_valid_ratio', 0.60)
        self.declare_parameter('expected_min_range', 0.20)
        self.declare_parameter('expected_max_range', 20.0)

        publish_hz = self.get_parameter('publish_hz').value
        self.stale_threshold = self.get_parameter('stale_threshold_sec').value
        self.min_valid_ratio = self.get_parameter('min_valid_ratio').value
        self.expected_min_range = self.get_parameter('expected_min_range').value
        self.expected_max_range = self.get_parameter('expected_max_range').value

        self.last_scan = None
        self.last_scan_time = None
        self.scan_count = 0

        self.health_pub = self.create_publisher(
            SensorHealth,
            '/sensor_health/lidar',
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan_2d',
            self.scan_callback,
            10,
        )

        timer_period = 1.0 / max(0.1, publish_hz)
        self.timer = self.create_timer(timer_period, self.publish_health)

        self.get_logger().info('LiDAR monitor initialized (monitoring /lidar/scan_2d)')

    def scan_callback(self, msg: LaserScan):
        """Record LiDAR scan receipt."""
        self.last_scan = msg
        self.last_scan_time = time.monotonic()
        self.scan_count += 1

    def compute_valid_ratio(self, scan: LaserScan) -> float:
        """Compute ratio of finite valid-range returns in the scan."""
        if not scan.ranges:
            return 0.0

        valid = 0
        total = len(scan.ranges)

        for r in scan.ranges:
            if (
                r >= scan.range_min
                and r <= scan.range_max
                and r >= self.expected_min_range
                and r <= self.expected_max_range
            ):
                valid += 1

        return float(valid) / float(total) if total > 0 else 0.0

    def publish_health(self):
        """Publish LiDAR health status."""
        now = time.monotonic()
        health = SensorHealth()
        health.stamp = self.get_clock().now().to_msg()
        health.sensor_name = 'lidar'
        health.sample_count = self.scan_count

        if self.last_scan is None or self.last_scan_time is None:
            health.data_valid = False
            health.stale = True
            health.healthy = False
            health.valid_ratio = 0.0
            health.age_sec = 999.0
            health.sync_offset_sec = 0.0
            health.reason = 'NO_DATA'
        else:
            age = now - self.last_scan_time
            valid_ratio = self.compute_valid_ratio(self.last_scan)

            health.age_sec = float(age)
            health.valid_ratio = float(valid_ratio)
            health.stale = age > self.stale_threshold
            health.data_valid = valid_ratio >= self.min_valid_ratio

            # Sync offset: difference between message timestamp and wall time
            scan_stamp_sec = stamp_to_seconds(self.last_scan.header.stamp)
            wall_time = self.get_clock().now().nanoseconds * 1e-9
            health.sync_offset_sec = abs(wall_time - scan_stamp_sec)

            if health.stale:
                health.healthy = False
                health.reason = 'STALE'
            elif not health.data_valid:
                health.healthy = False
                health.reason = 'LOW_VALID_RATIO'
            else:
                health.healthy = True
                health.reason = 'HEALTHY'

        self.health_pub.publish(health)


def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
