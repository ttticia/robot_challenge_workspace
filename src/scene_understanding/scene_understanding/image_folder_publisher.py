"""
Simulated camera: publish images from a folder on /camera/image_raw.

Alongside each image, the source file name is published on
/camera/image_source (std_msgs/String) as JSON:
    {"stamp": "<sec>.<nanosec>", "file": "img_01.jpg", "path": "/full/path"}
"stamp" equals the image's header.stamp, so a subscriber can tell exactly
which file a received frame came from.

Simulated junction detector (local testing only): for each image,
/junction_detected (std_msgs/Bool) is published as true if the file name
starts with one of junction_prefixes (case-insensitive, e.g. junction01.jpg),
otherwise false. It is sent junction_delay_sec after the image so the image
has arrived before the trigger. Set publish_junction:=false to drive
/junction_detected by hand instead (ros2 topic pub).
"""

import json
import os
import threading

from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

DEFAULT_JUNCTION_PREFIXES = [f'junction0{i}' for i in range(1, 6)]
IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff', '.webp')


def list_images(folder):
    """Return sorted full paths of image files in a folder (non-recursive)."""
    names = sorted(
        n for n in os.listdir(folder) if n.lower().endswith(IMAGE_EXTENSIONS))
    return [os.path.join(folder, n) for n in names]


def is_junction_file(name, prefixes=DEFAULT_JUNCTION_PREFIXES):
    """Return True if a file name starts with a junction prefix (any case)."""
    name = os.path.basename(name).lower()
    return any(name.startswith(p.lower()) for p in prefixes)


class ImageFolderPublisher(Node):
    """
    Publish each image in a folder at a fixed rate, optionally looping.

    Images are read from disk by a background thread and kept in memory, so
    slow disks (e.g. Windows drives under /mnt/c in WSL) do not delay the
    publishing timer. Once an image is loaded it is never read again.
    """

    def __init__(self):
        super().__init__('image_folder_publisher')

        self.declare_parameter('image_dir', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        # Topic carrying the file name of each published image.
        self.declare_parameter('source_topic', '/camera/image_source')
        self.declare_parameter('rate', 1.0)          # images per second
        self.declare_parameter('loop', True)         # restart at the end
        self.declare_parameter('frame_id', 'camera')
        # Longest image side in pixels after resizing; 0 disables resizing.
        # Full-size photos (e.g. 4032x3024 = 36 MB raw) are too large to
        # stream reliably over ROS 2.
        self.declare_parameter('max_size', 1280)
        # Simulated junction trigger (see module docstring).
        self.declare_parameter('publish_junction', True)
        self.declare_parameter('junction_topic', '/junction_detected')
        self.declare_parameter('junction_prefixes', DEFAULT_JUNCTION_PREFIXES)
        self.declare_parameter('junction_delay_sec', 0.1)
        # Junction images next to each other (e.g. junction01, junction02)
        # give one long "true", i.e. one rising edge. Set this to true to
        # send a "false" before each new junction image so each one
        # triggers separately (cooldown_sec of junction_trigger still applies).
        self.declare_parameter('separate_junctions', False)

        image_dir = os.path.expanduser(self.get_parameter('image_dir').value)
        topic = self.get_parameter('image_topic').value
        source_topic = self.get_parameter('source_topic').value
        rate = float(self.get_parameter('rate').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.max_size = int(self.get_parameter('max_size').value)

        if not image_dir or not os.path.isdir(image_dir):
            raise RuntimeError(
                f'image_dir is not a folder: "{image_dir}". '
                'Pass it with --ros-args -p image_dir:=/path/to/images')

        self.paths = list_images(image_dir)
        if not self.paths:
            raise RuntimeError(f'No images found in {image_dir}')

        self.bridge = CvBridge()
        self.index = 0
        self.frames = []          # filled by the loader thread, in order
        self.loading_done = False
        self._waiting_logged = False
        # Reliable QoS: lost network fragments are re-sent instead of the
        # whole (multi-MB) image being dropped. A reliable publisher also
        # works with best-effort subscribers.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Image, topic, qos)
        self.source_publisher = self.create_publisher(
            String, source_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.junction_publisher = None
        if self.get_parameter('publish_junction').value:
            self.junction_prefixes = list(
                self.get_parameter('junction_prefixes').value)
            self.separate_junctions = bool(
                self.get_parameter('separate_junctions').value)
            delay = min(float(self.get_parameter('junction_delay_sec').value),
                        0.5 / rate)
            self.junction_publisher = self.create_publisher(
                Bool, self.get_parameter('junction_topic').value, 10)
            self._pending_junction = None    # (file name, bool) to send
            self._previous_junction_file = None
            # One-shot timer: started (reset) after each image.
            self.junction_timer = self.create_timer(
                max(delay, 0.001), self.publish_junction)
            self.junction_timer.cancel()
            self.get_logger().info(
                f'Simulated junction trigger on '
                f'{self.get_parameter("junction_topic").value} for files '
                f'starting with {self.junction_prefixes} '
                f'(separate_junctions={self.separate_junctions})')

        self.get_logger().info(
            f'Publishing {len(self.paths)} images from {image_dir} '
            f'to {topic} at {rate} Hz (loop={self.loop}, '
            f'max_size={self.max_size})')
        if image_dir.startswith('/mnt/'):
            self.get_logger().warning(
                'Images are on a Windows drive, which is slow to read from '
                'WSL. The first pass may be slower than the requested rate. '
                'Copy the folder into the WSL filesystem (e.g. ~/ir/data) '
                'for faster startup.')

        self.loader = threading.Thread(target=self.load_images, daemon=True)
        self.loader.start()

    def load_images(self):
        """Background thread: read and downscale every image once."""
        for path in self.paths:
            frame = cv2.imread(path, cv2.IMREAD_COLOR)   # BGR, 8-bit
            if frame is None:
                self.get_logger().warning(f'Could not read {path}, skipping')
                continue
            # list.append is atomic in Python, so the timer can safely read
            # self.frames while this thread is adding to it.
            self.frames.append((path, self.downscale(frame)))
        self.loading_done = True
        self.get_logger().info(f'Loaded {len(self.frames)} images into memory')

    def timer_callback(self):
        """Publish the next already-loaded image (no disk access here)."""
        if self.loading_done and not self.frames:
            self.get_logger().error('None of the images could be read.')
            self.timer.cancel()
            return

        if self.index >= len(self.frames):
            if not self.loading_done:
                # The loader has not reached this image yet; try next tick.
                if not self._waiting_logged:
                    self.get_logger().info('Waiting for images to load...')
                    self._waiting_logged = True
                return
            if not self.loop:
                self.get_logger().info('Reached last image, stopping.')
                self.timer.cancel()
                return
            self.index = 0
        self._waiting_logged = False

        path, frame = self.frames[self.index]
        name = os.path.basename(path)
        self.index += 1

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # File name first, so it is usually already there when the image
        # arrives; subscribers match the two by the shared stamp.
        stamp = msg.header.stamp
        self.source_publisher.publish(String(data=json.dumps({
            'stamp': f'{stamp.sec}.{stamp.nanosec:09d}',
            'file': name,
            'path': path,
        })))
        self.publisher.publish(msg)

        junction = None
        if self.junction_publisher is not None:
            junction = is_junction_file(name, self.junction_prefixes)
            self._pending_junction = (name, junction)
            self.junction_timer.reset()

        total = len(self.frames) if self.loading_done else len(self.paths)
        self.get_logger().info(
            f'[{self.index}/{total}] '
            f'{name} | {frame.shape[1]}x{frame.shape[0]}'
            + (' | JUNCTION' if junction else ''))

    def publish_junction(self):
        """One-shot: publish the junction flag for the last image."""
        self.junction_timer.cancel()
        if self._pending_junction is None:
            return
        name, junction = self._pending_junction
        self._pending_junction = None
        if (junction and self.separate_junctions
                and self._previous_junction_file not in (None, name)):
            self.junction_publisher.publish(Bool(data=False))
        self._previous_junction_file = name if junction else None
        self.junction_publisher.publish(Bool(data=junction))

    def downscale(self, frame):
        """Shrink so the longest side is at most max_size, keeping aspect."""
        h, w = frame.shape[:2]
        longest = max(h, w)
        if self.max_size <= 0 or longest <= self.max_size:
            return frame
        scale = self.max_size / longest
        new_size = (round(w * scale), round(h * scale))
        # INTER_AREA gives the best quality when shrinking.
        return cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImageFolderPublisher()
    except RuntimeError as e:
        rclpy.logging.get_logger('image_folder_publisher').error(str(e))
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
