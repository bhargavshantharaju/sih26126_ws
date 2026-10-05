#!/usr/bin/env python3
# Copyright 2026 DRISTHI Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""DRISTHI backward-compatibility scan relay.

[DEPRECATED] Relays /lidar/scan_2d to /scan for components not yet
migrated to consume /lidar/scan_2d directly.

Components should migrate to /lidar/scan_2d directly.
This relay will be removed in a future release.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    """[DEPRECATED] Relay /lidar/scan_2d to /scan for backward compatibility."""

    def __init__(self):
        super().__init__('scan_relay')

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan_2d',
            self.lidar_callback,
            10,
        )

        self.get_logger().info(
            '[DEPRECATED] Scan relay: /lidar/scan_2d -> /scan '
            '(components should migrate to /lidar/scan_2d directly)'
        )

    def lidar_callback(self, msg: LaserScan):
        """Relay scan_2d to legacy /scan topic."""
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
