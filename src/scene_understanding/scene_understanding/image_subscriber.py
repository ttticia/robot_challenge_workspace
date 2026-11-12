"""Component 1: subscribe to camera images and convert them to OpenCV."""

from collections import deque
import time

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    """Keep the most recent camera frame as an OpenCV (numpy) image."""

    # How often the receive rate is logged, and how much history it uses.
    RATE_WINDOW_SEC = 2.0

    def __init__(self, node_name='image_subscriber'):
        # node_name lets other components (e.g. JunctionTrigger) reuse this
        # class as a base with their own node name.
        super().__init__(node_name)

        # Parameters can be overridden at launch: --ros-args -p image_topic:=/foo
        self.declare_parameter('image_topic', '/camera/image_raw')
        # Must be False for publishers that only offer best effort
        # (some real camera drivers); otherwise no images arrive.
        self.declare_parameter('reliable', True)
        topic = self.get_parameter('image_topic').value
        reliable = bool(self.get_parameter('reliable').value)

        self.bridge = CvBridge()
        self.latest_frame = None      # numpy array, BGR8
        self.latest_stamp = None      # builtin_interfaces/Time from the header
        self.latest_receive_time = None   # time.monotonic() at arrival
        self._last_info = ''
        self._arrivals = deque()      # monotonic arrival times, recent only

        # Reliable: large images arrive intact (lost fragments are re-sent).
        # Best effort: lower latency, but large images are often dropped.
        # depth=2 because only the newest frame matters.
        if reliable:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=2,
                reliability=ReliabilityPolicy.RELIABLE)
        else:
            qos = qos_profile_sensor_data
        self.subscription = self.create_subscription(
            Image, topic, self.image_callback, qos)

        self.get_logger().info(
            f'Subscribed to {topic} '
            f'({"reliable" if reliable else "best effort"})')

        self.rate_timer = self.create_timer(
            self.RATE_WINDOW_SEC, self.log_rate)

    def image_callback(self, msg: Image):
        """Convert each incoming ROS Image to a BGR numpy array."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        now = time.monotonic()
        self.latest_frame = frame
        self.latest_stamp = msg.header.stamp
        self.latest_receive_time = now
        h, w = frame.shape[:2]
        self._last_info = f'{w}x{h} | source encoding={msg.encoding}'
        self._arrivals.append(now)

    def log_rate(self):
        """
        Log the receive rate measured from the gaps between frames.

        rate = (frames - 1) / (time from first to last frame), using the
        last RATE_WINDOW_SEC (at least two frames). Unlike counting frames
        per window, this gives a stable number even at low rates.
        """
        now = time.monotonic()
        # Keep at least 2 arrivals so slow streams (< 1 frame per window)
        # still have an interval to measure.
        while (len(self._arrivals) > 2
               and now - self._arrivals[0] > self.RATE_WINDOW_SEC):
            self._arrivals.popleft()

        if not self._arrivals or now - self._arrivals[-1] > 5.0:
            self.get_logger().warning('No images received in the last 5 s')
            return
        if len(self._arrivals) < 2:
            return
        span = self._arrivals[-1] - self._arrivals[0]
        rate = (len(self._arrivals) - 1) / span if span > 0 else 0.0
        self.get_logger().info(f'{rate:.1f} Hz | {self._last_info}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
