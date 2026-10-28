#!/usr/bin/env python3
"""Diagnostic tool to trace SLAM scan processing timing issues.

This script monitors:
1. /clock to track simulation time progression
2. /lidar/scan_2d headers to see scan timestamps and frame_id
3. /tf to see when base_link->lidar and odom->base_link transforms are published
4. SLAM processing through /map updates

Usage:
    ros2 run <package> diagnose_slam_timing.py
    or: python3 diagnose_slam_timing.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
import time


class SLAMTimingDiagnostic(Node):
    """Monitor SLAM pipeline timing."""

    def __init__(self):
        super().__init__('slam_timing_diagnostic')
        
        # Track state
        self.wall_start = time.monotonic()
        self.sim_start = None
        self.last_scan_stamp = None
        self.scan_count = 0
        self.odom_base_tf_count = 0
        self.base_lidar_tf_count = 0
        self.map_update_count = 0
        self.last_map_stamp = None
        
        # Subscribers
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.create_subscription(LaserScan, '/lidar/scan_2d', self._scan_cb, 10)
        
        # TF with transient_local to catch static transforms
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=100
        )
        self.create_subscription(TFMessage, '/tf', self._tf_cb, 10)
        self.create_subscription(TFMessage, '/tf_static', self._tf_static_cb, tf_qos)
        
        # Map updates
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 
                               QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        # Periodic summary
        self.create_timer(5.0, self._print_summary)
        
        self.get_logger().info('SLAM timing diagnostic started')

    def _clock_cb(self, msg):
        """Track simulation time."""
        if self.sim_start is None:
            self.sim_start = msg.clock.sec + msg.clock.nanosec * 1e-9
            self.get_logger().info(f'First /clock: {self.sim_start:.3f}s sim')

    def _scan_cb(self, msg):
        """Monitor scan publication."""
        self.scan_count += 1
        scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame = msg.header.frame_id
        
        if self.scan_count <= 5 or self.scan_count % 50 == 0:
            wall_elapsed = time.monotonic() - self.wall_start
            sim_elapsed = (scan_stamp - self.sim_start) if self.sim_start else 0
            self.get_logger().info(
                f'Scan #{self.scan_count}: stamp={scan_stamp:.3f}s frame={frame} '
                f'wall={wall_elapsed:.1f}s sim={sim_elapsed:.1f}s'
            )
        
        self.last_scan_stamp = scan_stamp

    def _tf_cb(self, msg):
        """Monitor dynamic TF (odom->base_link from EKF)."""
        for tf in msg.transforms:
            if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_link':
                self.odom_base_tf_count += 1
                if self.odom_base_tf_count == 1:
                    stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                    wall_elapsed = time.monotonic() - self.wall_start
                    self.get_logger().info(
                        f'First odom->base_link TF: stamp={stamp:.3f}s wall={wall_elapsed:.1f}s'
                    )

    def _tf_static_cb(self, msg):
        """Monitor static TF (base_link->lidar)."""
        for tf in msg.transforms:
            if 'lidar' in tf.child_frame_id:
                self.base_lidar_tf_count += 1
                if self.base_lidar_tf_count == 1:
                    self.get_logger().info(
                        f'First base_link->lidar TF: {tf.header.frame_id} -> {tf.child_frame_id}'
                    )

    def _map_cb(self, msg):
        """Monitor map updates from SLAM."""
        self.map_update_count += 1
        map_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.map_update_count == 1:
            wall_elapsed = time.monotonic() - self.wall_start
            sim_elapsed = (map_stamp - self.sim_start) if self.sim_start else 0
            self.get_logger().info(
                f'First /map published: stamp={map_stamp:.3f}s '
                f'wall={wall_elapsed:.1f}s sim={sim_elapsed:.1f}s '
                f'size={msg.info.width}x{msg.info.height}'
            )
        
        self.last_map_stamp = map_stamp

    def _print_summary(self):
        """Print periodic summary."""
        wall_elapsed = time.monotonic() - self.wall_start
        self.get_logger().info(
            f'Summary @ wall={wall_elapsed:.1f}s: '
            f'scans={self.scan_count} '
            f'odom_tf_msgs={self.odom_base_tf_count} '
            f'maps={self.map_update_count}'
        )


def main():
    rclpy.init()
    node = SLAMTimingDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
