#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class PerceptionBaseline(Node):
    def __init__(self):
        super().__init__('perception_baseline')

        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Image,
            '/perception/traversability',
            10,
        )

        self.get_logger().info(
            'Perception baseline started. Waiting for /camera...'
        )

    def image_callback(self, msg: Image):
        if msg.encoding not in ('rgb8', 'RGB8'):
            self.get_logger().warn(
                f'Unsupported image encoding: {msg.encoding}'
            )
            return

        try:
            image = np.frombuffer(
                msg.data,
                dtype=np.uint8,
            ).reshape((msg.height, msg.width, 3))
        except ValueError as exc:
            self.get_logger().error(f'Invalid image buffer: {exc}')
            return

        # Convert RGB to grayscale.
        gray = (
            0.299 * image[:, :, 0]
            + 0.587 * image[:, :, 1]
            + 0.114 * image[:, :, 2]
        ).astype(np.uint8)

        # Focus on the lower part of the image where the ground
        # immediately in front of the rover is visible.
        roi_start = int(msg.height * 0.45)
        roi = gray[roi_start:, :]

        # Simple local gradient estimate.
        dx = np.abs(
            roi[:, 1:].astype(np.int16)
            - roi[:, :-1].astype(np.int16)
        )

        dy = np.abs(
            roi[1:, :].astype(np.int16)
            - roi[:-1, :].astype(np.int16)
        )

        gradient = np.zeros_like(roi, dtype=np.uint8)
        gradient[:, 1:] = np.maximum(gradient[:, 1:], dx)
        gradient[1:, :] = np.maximum(gradient[1:, :], dy)

        # Baseline heuristic:
        # bright + locally smooth = potentially traversable.
        traversable = (
            (roi > 80) &
            (gradient < 35)
        )

        mask = np.zeros_like(gray, dtype=np.uint8)
        mask[roi_start:, :] = np.where(traversable, 255, 0).astype(np.uint8)

        # Publish the binary traversability mask.
        output = Image()
        output.header = msg.header
        output.height = msg.height
        output.width = msg.width
        output.encoding = 'mono8'
        output.is_bigendian = 0
        output.step = msg.width
        output.data = mask.tobytes()

        self.publisher.publish(output)

        self.frame_count += 1

        if self.frame_count == 1:
            self.get_logger().info(
                f'First perception frame: {msg.width}x{msg.height}'
            )

        if self.frame_count % 300 == 0:
            traversable_ratio = float(np.mean(traversable))

            self.get_logger().info(
                f'frames={self.frame_count} '
                f'traversable_ratio={traversable_ratio:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = PerceptionBaseline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
