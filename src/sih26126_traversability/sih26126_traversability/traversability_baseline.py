#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class TraversabilityBaseline(Node):
    """
    Deterministic camera-only traversability baseline.

    Input:
        /camera : sensor_msgs/msg/Image, rgb8

    Outputs:
        /traversability/score_map : sensor_msgs/msg/Image, mono8
        /traversability/mean_score : std_msgs/msg/Float32

    The score map always matches the input image dimensions.
    Pixels above the ground-focused ROI are marked as unknown (0).
    """

    def __init__(self):
        super().__init__('traversability_baseline')

        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10,
        )

        self.score_map_publisher = self.create_publisher(
            Image,
            '/traversability/score_map',
            10,
        )

        self.mean_score_publisher = self.create_publisher(
            Float32,
            '/traversability/mean_score',
            10,
        )

        self.get_logger().info(
            'Traversability baseline started. Waiting for /camera...'
        )

    @staticmethod
    def compute_score_map(image: np.ndarray) -> tuple[np.ndarray, int]:
        """
        Compute a deterministic 0..1 traversability map.

        Bright, locally smooth terrain receives a higher score.
        Dark, high-gradient regions receive a lower score.

        Returns
        -------
        score : np.ndarray
            ROI-sized float32 array in [0, 1].
        roi_start : int
            First full-image row belonging to the ROI.

        """
        height = image.shape[0]

        # Use the lower 60% of the image as the current ground-focused ROI.
        roi_start = int(height * 0.40)
        roi = image[roi_start:, :, :]

        gray = (
            0.299 * roi[:, :, 0].astype(np.float32)
            + 0.587 * roi[:, :, 1].astype(np.float32)
            + 0.114 * roi[:, :, 2].astype(np.float32)
        )

        brightness = np.clip(
            (gray - 40.0) / 180.0,
            0.0,
            1.0,
        )

        dx = np.abs(gray[:, 1:] - gray[:, :-1])
        dy = np.abs(gray[1:, :] - gray[:-1, :])

        gradient = np.zeros_like(gray)
        gradient[:, 1:] = np.maximum(
            gradient[:, 1:],
            dx,
        )
        gradient[1:, :] = np.maximum(
            gradient[1:, :],
            dy,
        )

        smoothness = np.exp(-gradient / 30.0)

        score = (
            0.60 * brightness
            + 0.40 * smoothness
        )

        return np.clip(score, 0.0, 1.0).astype(np.float32), roi_start

    def image_callback(self, msg: Image):
        if msg.encoding not in ('rgb8', 'RGB8'):
            self.get_logger().warn(
                f'Unsupported image encoding: {msg.encoding}'
            )
            return

        expected_bytes = msg.height * msg.width * 3

        if len(msg.data) != expected_bytes:
            self.get_logger().error(
                f'Unexpected image size: got {len(msg.data)} bytes, '
                f'expected {expected_bytes}'
            )
            return

        try:
            image = np.frombuffer(
                msg.data,
                dtype=np.uint8,
            ).reshape(
                (msg.height, msg.width, 3)
            )
        except ValueError as exc:
            self.get_logger().error(
                f'Failed to reshape image: {exc}'
            )
            return

        score, roi_start = self.compute_score_map(image)

        # Full-frame output matching the camera geometry.
        score_image = np.zeros(
            (msg.height, msg.width),
            dtype=np.uint8,
        )

        score_image[roi_start:, :] = np.round(
            score * 255.0
        ).astype(np.uint8)

        output = Image()
        output.header = msg.header
        output.height = msg.height
        output.width = msg.width
        output.encoding = 'mono8'
        output.is_bigendian = 0
        output.step = msg.width
        output.data = score_image.tobytes()

        self.score_map_publisher.publish(output)

        mean_score = Float32()
        mean_score.data = float(np.mean(score))

        self.mean_score_publisher.publish(mean_score)

        self.frame_count += 1

        if self.frame_count == 1:
            self.get_logger().info(
                f'First traversability frame: '
                f'{msg.width}x{msg.height}'
            )

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'frames={self.frame_count} '
                f'mean_score={float(np.mean(score)):.3f}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = TraversabilityBaseline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
