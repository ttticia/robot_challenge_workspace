"""Component 1: subscribe to camera images and convert them to OpenCV."""

from collections import deque
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CompressedImage, Image


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
        # 'compressed' = <image_topic>/compressed (JPEG, ~75 KB a frame instead of
        # 0.9 MB). Use it on the robot: one raw image subscriber slows every
        # stream that crosses the bridge, the lidar IMU included, and Fast-LIO
        # then diverges.
        self.declare_parameter('image_transport', 'raw')
        # False: say the rate once, afterwards only when the stream stops or returns.
        self.declare_parameter('log_rate', True)
        topic = self.get_parameter('image_topic').value
        reliable = bool(self.get_parameter('reliable').value)
        transport = str(self.get_parameter('image_transport').value)
        if transport not in ('raw', 'compressed'):
            raise ValueError(f'image_transport must be raw or compressed, got "{transport}"')
        self.keep_rate_log = bool(self.get_parameter('log_rate').value)
        self._rate_said = False
        self._silent = False

        self.bridge = CvBridge()
        self._frame = None            # numpy array, BGR8
        self._jpeg = None             # newest compressed frame, decoded when asked for
        self.latest_stamp = None      # builtin_interfaces/Time from the header
        self.latest_receive_time = None   # time.monotonic() at arrival
        self._last_info = ''
        self._arrivals = deque()      # monotonic arrival times, recent only

        # Reliable: large images arrive intact (lost fragments are re-sent).
        # Best effort: lower latency, but large images are often dropped.
        # depth=2 because only the newest frame matters.
        if transport == 'compressed':
            # Best effort: a reliable reader makes the bridge re-send to us
            # whenever we fall behind.
            topic, reliable, kind = topic.rstrip('/') + '/compressed', False, CompressedImage
        else:
            kind = Image
        if reliable:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=2,
                reliability=ReliabilityPolicy.RELIABLE)
        else:
            qos = qos_profile_sensor_data
        self._stream = (kind, topic, qos)
        self.subscription = None
        # A subclass may set stream_at_start = False before calling this
        # constructor and use start_stream() / stop_stream() itself.
        if getattr(self, 'stream_at_start', True):
            self.start_stream()
            self.get_logger().info(
                f'Subscribed to {topic} '
                f'({"reliable" if reliable else "best effort"})')

        self.rate_timer = self.create_timer(
            self.RATE_WINDOW_SEC, self.log_rate)

    def start_stream(self):
        """Subscribe to the camera. Returns True when this call did it."""
        if self.subscription is not None:
            return False
        self._arrivals.clear()
        self.subscription = self.create_subscription(*self._stream[:2], self.image_callback,
                                                     self._stream[2])
        return True

    def stop_stream(self):
        """Drop the camera subscription and the frame kept from it. Do not call
        this from image_callback (it would destroy the subscription that is
        running); use a timer."""
        if self.subscription is None:
            return False
        self.destroy_subscription(self.subscription)
        self.subscription = None
        self.latest_frame = None
        self.latest_receive_time = None
        return True

    @property
    def latest_frame(self):
        """Newest frame as BGR. A compressed frame is decoded here, once, so
        the frames nobody looks at cost no CPU."""
        if self._frame is None and self._jpeg is not None:
            self._frame = cv2.imdecode(np.frombuffer(self._jpeg, np.uint8), cv2.IMREAD_COLOR)
            self._jpeg = None
            if self._frame is None:
                self.get_logger().error('could not decode the compressed image',
                                        throttle_duration_sec=5.0)
        return self._frame

    @latest_frame.setter
    def latest_frame(self, frame):
        self._frame, self._jpeg = frame, None

    def image_callback(self, msg):
        """Keep each incoming image (raw: as a BGR numpy array)."""
        if isinstance(msg, CompressedImage):
            self._frame, self._jpeg = None, bytes(msg.data)
            self.latest_stamp = msg.header.stamp
            self.latest_receive_time = time.monotonic()
            self._last_info = f'{msg.format} | {len(msg.data) // 1024} KB'
            self._arrivals.append(self.latest_receive_time)
            return
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
        if self.subscription is None:      # camera off on purpose
            return
        now = time.monotonic()
        # Keep at least 2 arrivals so slow streams (< 1 frame per window)
        # still have an interval to measure.
        while (len(self._arrivals) > 2
               and now - self._arrivals[0] > self.RATE_WINDOW_SEC):
            self._arrivals.popleft()

        if not self._arrivals or now - self._arrivals[-1] > 5.0:
            if self.keep_rate_log or not self._silent:
                self.get_logger().warning('No images received in the last 5 s')
            self._silent = True
            return
        if len(self._arrivals) < 2:
            return
        if not self.keep_rate_log and self._rate_said and not self._silent:
            return
        span = self._arrivals[-1] - self._arrivals[0]
        rate = (len(self._arrivals) - 1) / span if span > 0 else 0.0
        self.get_logger().info(f'{rate:.1f} Hz | {self._last_info}')
        self._rate_said, self._silent = True, False


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
