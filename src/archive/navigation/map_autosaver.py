"""Keep the SLAM map on disk: save it periodically and once more on shutdown.

The node keeps the latest /map in memory and writes it itself, so the final save
still works when Ctrl+C stops slam_toolbox at the same moment.
"""
from datetime import datetime
from pathlib import Path
import signal
import threading

from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from navigation.map_io import link_latest, write_map, yaw_from_quaternion


class MapAutosaver(Node):

    def __init__(self):
        super().__init__('map_autosaver')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_dir', '~/challenge_ws/maps')
        self.declare_parameter('map_name', '')
        # Accept 30 as well as 30.0 from the command line.
        self.declare_parameter('save_interval_sec', 30.0, ParameterDescriptor(dynamic_typing=True))

        self.map_dir = Path(self.get_parameter('map_dir').value).expanduser()
        self.map_name = (self.get_parameter('map_name').value
                         or datetime.now().strftime('map_%Y%m%d_%H%M%S'))
        self.latest = None
        self.saved = None

        # slam_toolbox publishes /map reliable + transient local.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                 self.on_map, qos)
        interval = float(self.get_parameter('save_interval_sec').value)
        if interval > 0:
            self.create_timer(interval, self.save)
        self.get_logger().info(
            f'Saving map to {self.map_dir / self.map_name}.yaml '
            f'(every {interval:g} s and on shutdown)')

    def on_map(self, msg):
        self.latest = msg

    def save(self):
        msg = self.latest
        if msg is None or msg is self.saved or msg.info.width == 0:
            return False
        info, origin = msg.info, msg.info.origin
        path = write_map(self.map_dir, self.map_name, msg.data, info.width, info.height,
                         info.resolution,
                         (origin.position.x, origin.position.y,
                          yaw_from_quaternion(origin.orientation)))
        link_latest(self.map_dir, self.map_name)
        self.saved = msg
        self.get_logger().info(
            f'Saved {info.width}x{info.height} map '
            f'({info.width * info.resolution:.1f} x {info.height * info.resolution:.1f} m) '
            f'to {path}')
        return True


def main():
    # Handle Ctrl+C ourselves so the final save runs before rclpy shuts down.
    stop = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = MapAutosaver()
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            if not node.save() and node.saved is None:
                node.get_logger().warn('No map was received on /map, so nothing was saved')
        except Exception as error:  # noqa: BLE001  (report, then still shut down cleanly)
            node.get_logger().error(f'Final map save failed: {error}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
