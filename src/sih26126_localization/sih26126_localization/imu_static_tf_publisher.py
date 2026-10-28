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
"""
Publish static TF from base_link to the IMU sensor frame.

The Gazebo SIH-UGV model places the IMU sensor at 0, 0, 0.20 relative to
base_link.  The sensor emits frames named ``sih_ugv/base_link/imu_sensor``.
robot_localization needs a static transform so it can express IMU
measurements in the base_link frame.
"""

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class ImuStaticTFPublisher(Node):
    """Publish static transform from base_link to IMU sensor frame."""

    def __init__(self) -> None:
        """Initialise and immediately broadcast the static transform."""
        super().__init__('imu_static_tf_publisher')

        self._broadcaster = StaticTransformBroadcaster(self)
        self._publish()
        self.get_logger().info(
            'IMU static TF published: base_link -> sih_ugv/base_link/imu_sensor'
        )

    def _publish(self) -> None:
        """
        Broadcast the static transform once (latched).

        Timestamp is set to zero (sec=0, nanosec=0) intentionally.
        Using get_clock().now() at startup with use_sim_time=True may return
        sim-time=0 before /clock flows, causing TF2 to discard the transform.
        A zero-stamped static TF is treated by TF2 as valid at all times.
        """
        t = TransformStamped()
        t.header.stamp.sec = 0
        t.header.stamp.nanosec = 0
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'sih_ugv/base_link/imu_sensor'

        # IMU pose from SDF: <pose>0 0 0.20 0 0 0</pose> relative to base_link
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.20

        # No rotation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self._broadcaster.sendTransform(t)


def main(args=None) -> None:
    """Entry point for the imu_static_tf_publisher executable."""
    rclpy.init(args=args)
    node = ImuStaticTFPublisher()
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
