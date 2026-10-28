#!/usr/bin/env python3
"""ROS 2 node that manages the Gazebo GUI virtual-display streaming stack.

When ``gui_stream:=true`` is passed to the full_stack launch, this node:
  - Starts Xvfb, gz sim -g, x11vnc, and websockify
  - Publishes the stream status to /gz_gui_stream/status (std_msgs/String)
  - Shuts everything down cleanly on node destruction

The dashboard HTML already has a Gazebo Simulator panel that loads the
noVNC iframe when the stream is running.  This node does not modify the
HTTP server; it only manages the subprocess group.

Ports (all localhost-only):
  VNC:       localhost:5900   (x11vnc, not browser-accessible)
  noVNC WS:  localhost:6080   (websockify WebSocket + noVNC static files)
  noVNC UI:  http://localhost:6080/vnc_lite.html  (served by websockify)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from sih26126_dashboard.gz_gui_stream import GzGuiStream, _find


class GuiStreamNode(Node):
    """Lifecycle wrapper for the Gazebo GUI virtual-display streaming stack."""

    def __init__(self) -> None:
        super().__init__('gz_gui_stream')

        self.declare_parameter('vnc_port', 5900)
        self.declare_parameter('websocket_port', 6080)
        self.declare_parameter('virtual_display', ':99')
        self.declare_parameter('resolution', '1280x800x24')

        vnc_port = int(self.get_parameter('vnc_port').value)
        ws_port = int(self.get_parameter('websocket_port').value)
        display = str(self.get_parameter('virtual_display').value)
        resolution = str(self.get_parameter('resolution').value)

        gz_binary = _find('gz') or 'gz'

        self._status_pub = self.create_publisher(
            String, '/gz_gui_stream/status', 10)

        self._stream = GzGuiStream(
            gz_binary=gz_binary,
            virtual_display=display,
            vnc_port=vnc_port,
            websocket_port=ws_port,
            resolution=resolution,
            logger=lambda msg: self.get_logger().info(msg),
        )

        self._stream_ok = False
        try:
            self._stream.start()
            self._stream_ok = True
            self.get_logger().info(
                f'Gazebo GUI stream started.  '
                f'noVNC UI: http://localhost:{ws_port}/vnc_lite.html'
            )
        except RuntimeError as exc:
            self.get_logger().error(
                f'Gazebo GUI stream failed to start: {exc}'
            )

        # Publish status every 5 s so the dashboard can show stream health
        self.create_timer(5.0, self._publish_status)
        self._publish_status()

    def _publish_status(self) -> None:
        if not self._stream_ok:
            status = 'UNAVAILABLE'
        elif self._stream.is_alive():
            status = f'RUNNING:ws_port={self._stream.websocket_port}'
        else:
            status = 'STOPPED'
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def destroy_node(self) -> None:
        self.get_logger().info('Stopping Gazebo GUI stream...')
        try:
            self._stream.stop()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Stream stop error: {exc}')
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GuiStreamNode()
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
