"""SIH26126 sensor validity, freshness, and synchronization monitor."""

from collections import deque
import math
import time

import numpy as np
from builtin_interfaces.msg import Time as RosTime
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from sih26126_msgs.msg import SensorHealth
from std_msgs.msg import Float32


def stamp_to_seconds(stamp: RosTime) -> float:
    """Convert a ROS time message into seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _field_map(msg: PointCloud2) -> dict:
    """Return PointCloud2 fields keyed by name."""
    return {
        field.name: (
            int(field.offset),
            int(field.datatype),
            int(field.count),
        )
        for field in msg.fields
    }


def compute_valid_point_ratio(
    msg: PointCloud2,
    roi_y_start: float = 0.50,
    min_range_m: float = 0.10,
    max_range_m: float = 15.0,
) -> float:
    """Compute finite XYZ ratio inside the navigation ROI."""
    fields = _field_map(msg)

    for name in ('x', 'y', 'z'):
        if name not in fields:
            return 0.0

        _, datatype, count = fields[name]
        if datatype != 7 or count != 1:
            return 0.0

    height = int(msg.height)
    width = int(msg.width)

    if height <= 0 or width <= 0:
        return 0.0

    roi_y_start = float(np.clip(roi_y_start, 0.0, 1.0))
    y0 = min(height - 1, max(0, int(height * roi_y_start)))

    if y0 >= height:
        return 0.0

    endian = '>' if msg.is_bigendian else '<'

    arrays = []

    for name in ('x', 'y', 'z'):
        offset, _, _ = fields[name]

        values = np.ndarray(
            shape=(height, width),
            dtype=np.dtype(endian + 'f4'),
            buffer=msg.data,
            offset=offset,
            strides=(
                int(msg.row_step),
                int(msg.point_step),
            ),
        )

        arrays.append(values)

    x, y, z = arrays

    x = x[y0:]
    y = y[y0:]
    z = z[y0:]

    finite = (
        np.isfinite(x)
        & np.isfinite(y)
        & np.isfinite(z)
    )

    ranges = np.sqrt(
        x * x
        + y * y
        + z * z
    )

    valid = (
        finite
        & np.isfinite(ranges)
        & (ranges >= min_range_m)
        & (ranges <= max_range_m)
    )

    return float(valid.mean())


class SensorContractNode(Node):
    """Monitor the validity and freshness of core sensor streams."""

    def __init__(self) -> None:
        super().__init__('sensor_contract')

        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter(
            'depth_topic',
            '/depth_camera/points',
        )
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter(
            'odom_topic',
            '/model/sih_ugv/odometry',
        )
        self.declare_parameter('health_rate_hz', 5.0)
        self.declare_parameter(
            'camera_timeout_sec',
            0.20,
        )
        self.declare_parameter(
            'depth_timeout_sec',
            0.30,
        )
        self.declare_parameter(
            'imu_timeout_sec',
            0.15,
        )
        self.declare_parameter(
            'odom_timeout_sec',
            0.20,
        )
        self.declare_parameter(
            'depth_min_valid_ratio',
            0.60,
        )
        self.declare_parameter(
            'depth_roi_y_start',
            0.50,
        )
        self.declare_parameter(
            'depth_min_range_m',
            0.10,
        )
        self.declare_parameter(
            'depth_max_range_m',
            15.0,
        )
        self.declare_parameter(
            'max_camera_depth_sync_sec',
            0.066,
        )

        camera_topic = self.get_parameter(
            'camera_topic'
        ).value
        depth_topic = self.get_parameter(
            'depth_topic'
        ).value
        camera_info_topic = self.get_parameter(
            'camera_info_topic'
        ).value
        imu_topic = self.get_parameter(
            'imu_topic'
        ).value
        odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.camera_timeout = float(
            self.get_parameter(
                'camera_timeout_sec'
            ).value
        )
        self.depth_timeout = float(
            self.get_parameter(
                'depth_timeout_sec'
            ).value
        )
        self.imu_timeout = float(
            self.get_parameter(
                'imu_timeout_sec'
            ).value
        )
        self.odom_timeout = float(
            self.get_parameter(
                'odom_timeout_sec'
            ).value
        )
        self.depth_min_valid_ratio = float(
            self.get_parameter(
                'depth_min_valid_ratio'
            ).value
        )
        self.depth_roi_y_start = float(
            self.get_parameter(
                'depth_roi_y_start'
            ).value
        )
        self.depth_min_range_m = float(
            self.get_parameter(
                'depth_min_range_m'
            ).value
        )
        self.depth_max_range_m = float(
            self.get_parameter(
                'depth_max_range_m'
            ).value
        )
        self.max_camera_depth_sync_sec = float(
            self.get_parameter(
                'max_camera_depth_sync_sec'
            ).value
        )

        self.last_received = {
            'camera': None,
            'depth': None,
            'imu': None,
            'odom': None,
        }

        self.sample_count = {
            'camera': 0,
            'depth': 0,
            'imu': 0,
            'odom': 0,
        }

        self.data_valid = {
            'camera': False,
            'depth': False,
            'imu': False,
            'odom': False,
        }

        self.depth_valid_ratio = 0.0
        self.depth_sync_offset = math.inf
        self.camera_stamps = deque(maxlen=30)
        self.camera_info_received = False

        self.health_publishers = {
            name: self.create_publisher(
                SensorHealth,
                f'/sensor_health/{name}',
                10,
            )
            for name in (
                'camera',
                'depth',
                'imu',
                'odom',
            )
        }

        self.valid_ratio_pub = self.create_publisher(
            Float32,
            '/geometry/valid_point_ratio',
            10,
        )

        self.create_subscription(
            Image,
            camera_topic,
            self.camera_callback,
            10,
        )

        self.create_subscription(
            PointCloud2,
            depth_topic,
            self.depth_callback,
            10,
        )

        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10,
        )

        self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            20,
        )

        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        rate_hz = max(
            1.0,
            float(
                self.get_parameter(
                    'health_rate_hz'
                ).value
            ),
        )

        self.timer = self.create_timer(
            1.0 / rate_hz,
            self.publish_health,
        )

        self.get_logger().info(
            'SIH26126 sensor contract started.'
        )

    def _mark_received(
        self,
        name: str,
        stamp: RosTime,
        valid: bool,
    ) -> None:
        self.last_received[name] = time.monotonic()
        self.sample_count[name] += 1
        self.data_valid[name] = bool(valid)

    def camera_callback(self, msg: Image) -> None:
        """Track RGB camera freshness and validity."""
        valid = (
            int(msg.width) > 0
            and int(msg.height) > 0
            and len(msg.data) > 0
        )

        stamp = stamp_to_seconds(msg.header.stamp)

        if math.isfinite(stamp):
            self.camera_stamps.append(stamp)

        self._mark_received(
            'camera',
            msg.header.stamp,
            valid,
        )

    def depth_callback(self, msg: PointCloud2) -> None:
        """Validate organized XYZ depth and measure synchronization."""
        try:
            ratio = compute_valid_point_ratio(
                msg,
                self.depth_roi_y_start,
                self.depth_min_range_m,
                self.depth_max_range_m,
            )
        except (ValueError, BufferError, TypeError):
            ratio = 0.0

        self.depth_valid_ratio = ratio

        stamp = stamp_to_seconds(msg.header.stamp)

        if self.camera_stamps:
            deltas = [
                abs(stamp - previous)
                for previous in self.camera_stamps
            ]
            self.depth_sync_offset = min(deltas)
        else:
            self.depth_sync_offset = math.inf

        valid = ratio > 0.0

        self._mark_received(
            'depth',
            msg.header.stamp,
            valid,
        )

        ratio_msg = Float32()
        ratio_msg.data = float(ratio)
        self.valid_ratio_pub.publish(ratio_msg)

    def camera_info_callback(
        self,
        msg: CameraInfo,
    ) -> None:
        """Track camera calibration availability."""
        self.camera_info_received = (
            len(msg.k) == 9
            and all(
                math.isfinite(float(value))
                for value in msg.k
            )
            and float(msg.k[0]) > 0.0
            and float(msg.k[4]) > 0.0
        )

    def imu_callback(self, msg: Imu) -> None:
        """Track IMU freshness."""
        self._mark_received(
            'imu',
            msg.header.stamp,
            True,
        )

    def odom_callback(self, msg: Odometry) -> None:
        """Track wheel odometry freshness."""
        self._mark_received(
            'odom',
            msg.header.stamp,
            True,
        )

    def _health_message(
        self,
        name: str,
        timeout_sec: float,
    ) -> SensorHealth:
        """Build a sensor health message."""
        now = time.monotonic()
        received = self.last_received[name]

        if received is None:
            age = math.inf
            stale = True
        else:
            age = max(
                0.0,
                now - received,
            )
            stale = age > timeout_sec

        valid = self.data_valid[name]
        ratio = 1.0
        sync_offset = 0.0
        reason = 'nominal'

        if name == 'depth':
            ratio = self.depth_valid_ratio
            sync_offset = self.depth_sync_offset

            if not valid:
                reason = 'no_valid_xyz'
            elif ratio < self.depth_min_valid_ratio:
                reason = 'low_navigation_roi_valid_ratio'
            elif (
                not math.isfinite(sync_offset)
            ):
                reason = 'camera_depth_sync_unknown'
            elif (
                sync_offset
                > self.max_camera_depth_sync_sec
            ):
                reason = 'camera_depth_desynchronized'

        elif name == 'camera':
            if not valid:
                reason = 'invalid_image'
            elif not self.camera_info_received:
                reason = 'camera_info_missing'

        elif name == 'imu':
            if not valid:
                reason = 'no_imu_data'

        elif name == 'odom':
            if not valid:
                reason = 'no_odom_data'

        if stale:
            reason = 'stale_' + reason

        healthy = (
            valid
            and not stale
        )

        if name == 'depth':
            healthy = (
                healthy
                and ratio
                >= self.depth_min_valid_ratio
                and math.isfinite(sync_offset)
                and (
                    sync_offset
                    <= self.max_camera_depth_sync_sec
                )
            )

        msg = SensorHealth()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sensor_name = name
        msg.data_valid = bool(valid)
        msg.stale = bool(stale)
        msg.healthy = bool(healthy)
        msg.valid_ratio = float(ratio)
        msg.age_sec = (
            float(age)
            if math.isfinite(age)
            else -1.0
        )
        msg.sync_offset_sec = (
            float(sync_offset)
            if math.isfinite(sync_offset)
            else -1.0
        )
        msg.sample_count = int(
            self.sample_count[name]
        )
        msg.reason = reason

        return msg

    def publish_health(self) -> None:
        """Publish current health state for all monitored streams."""
        configs = {
            'camera': self.camera_timeout,
            'depth': self.depth_timeout,
            'imu': self.imu_timeout,
            'odom': self.odom_timeout,
        }

        for name, timeout_sec in configs.items():
            self.health_publishers[name].publish(
                self._health_message(
                    name,
                    timeout_sec,
                )
            )


def main(args=None) -> None:
    """Run the sensor contract node."""
    rclpy.init(args=args)
    node = SensorContractNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
