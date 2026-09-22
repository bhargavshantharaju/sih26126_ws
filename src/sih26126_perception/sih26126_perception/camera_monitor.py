#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraMonitor(Node):
    def __init__(self):
        super().__init__('camera_monitor')

        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10,
        )

        self.get_logger().info('Camera monitor started. Waiting for /camera...')

    def image_callback(self, msg: Image):
        self.frame_count += 1

        if self.frame_count == 1:
            self.get_logger().info(
                f'First frame received: '
                f'{msg.width}x{msg.height}, '
                f'encoding={msg.encoding}, '
                f'step={msg.step}'
            )

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Received {self.frame_count} camera frames'
            )


def main(args=None):
    rclpy.init(args=args)

    node = CameraMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
