"""
QR scanner node (robot build: no debug video).

Subscribes to a colour (or left rectified) image, decodes QR codes in a background
thread, and publishes/logs every NEW code once (with a saved snapshot as evidence). The
code's text is the conscious survivor's message (mission M04, booklet update 2026-11-24:
one QR code per arena, beside the M03 survivor). It is passed on EXACTLY as decoded, on
/qr/read, and logged as `QR message: ...`; task_reporter sends it as M04.

QR locations are not reported. A code only appears in a room with a survivor, though, so
with depth the node works out the code's ROOM point (the code's spot pulled back from its
wall towards the camera, like poster_detector does for posters) in odom and puts it in
/qr/detections as `room_odom`: poster_detector uses it to find that room's survivor.

Topics (all names are parameters - VERIFY on the robot with `ros2 topic list`):
  in : image_topic        sensor_msgs/Image, or <image_topic>/compressed
                          sensor_msgs/CompressedImage if image_transport:=compressed
  in : depth_topic        sensor_msgs/Image   (aligned to image_topic; optional)
  in : camera_info_topic  sensor_msgs/CameraInfo
  out: /qr/detections     std_msgs/String     (JSON {text, room_odom, stamp, snapshot}, one
                                               message per new code, again every repeat_after_s)
  out: /found             std_msgs/String     "QR found", once per new code text
  out: /qr/read           std_msgs/String     the text, EVERY time a code is read (<= 1 Hz each)
  out: /qr/unread         geometry_msgs/PointStamped (track_frame = odom): a code that is
                          located but can't be read, persistent over unread_need of the last
                          unread_window frames (<= 1 Hz). maze_explorer turns to face it
                          (FACING_QR) until /qr/read, so it is seen squarely and read.

When it runs (user 2026-11-20): QR READING is always on - plain decodes at roam_rate_hz,
so a code passed in a corridor is still read, reported (/found, /qr/detections, CSV) and
given a room point. QR DETECTION - /qr/unread (FACING_QR) and the slow contrast-boost +
upscale full scans - only inside maze_explorer's dead-end window (/maze/perceive,
run_gate.py), at rate_hz. Both stop in run_gate's pause_states (stairs, curtains, start-up).
read_everywhere:=false = the old rule (nothing outside the window).

detect (default False since 2026-11-23, user: "remove QR detection, only read the QR
codes"): no detection at all - no /qr/unread (so no FACING_QR turns), no contrast-boost /
upscale full scans; codes are only read, with plain decodes at roam_rate_hz while the run
is under way (dead ends included). detect:=true = the dead-end detection above.
"""
import csv
import json
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)

from team_perception import qr_utils
from team_perception import run_gate
from team_perception import tf_pin
from team_perception.poster_utils import pull_back
from team_perception.qr_utils import (DepthAlignState, UnreadTracker, deproject, draw_detections,
                                      scan_qr,
                                      transform_first)


class QrScanner(Node):
    def __init__(self):
        super().__init__('qr_scanner')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('image_transport', 'raw')    # 'raw', or 'compressed' = <image_topic>/compressed
        p('depth_topic', '/camera/depth/image_raw')
        p('depth_camera_info_topic', '/camera/depth/camera_info')  # '' = depth is
        #                                already aligned to colour
        p('camera_info_topic', '/camera/color/camera_info')
        p('use_depth', True)
        p('track_frame', 'odom')       # fixed frame of room_odom (Fast-LIO: there from launch)
        p('room_pullback_m', 0.25)     # step back from the code's wall towards the camera
        p('rate_hz', 30.0)             # max decodes per second (0 = as fast as possible)
        #                                inside the dead-end window
        p('read_everywhere', True)     # outside the window still read codes (plain decodes)
        p('detect', False)             # QR detection (full scans, /qr/unread); off = read only
        p('on_demand', False)          # camera subscribed only inside the gate's window
        #                                (maze_explore: during a room / object look). Needs
        #                                explore_status_topic (run_gate.py)
        p('roam_rate_hz', 5.0)         # ... at most this many frames a second (0 = rate_hz)
        p('full_scan_every', 5)        # every Nth decode also tries contrast boost + 2x
        #                                upscale (finds small/far codes, ~5x slower); 0 = never
        p('repeat_after_s', 30.0)      # re-report the same code after this long
        # Codes located but not readable (zxing-cpp only): ask the explorer to face them.
        p('unread_topic', '/qr/unread')   # '' = don't publish
        p('unread_need', 3)            # frames with the code among ...
        p('unread_window', 5)          # ... the last this many scanned frames
        p('unread_period_s', 1.0)      # at most one /qr/unread a second
        p('unread_range_m', 1.5)       # assumed distance along the ray when there's no depth
        p('unread_full_scan_s', 3.0)   # after an unread sighting: full scans on every frame
        p('log_dir', os.path.expanduser('~/challenge_ws/logs/qr'))   # nothing outside the workspace

        def g(name):
            return self.get_parameter(name).value

        self.use_depth = g('use_depth')
        self.track_frame = g('track_frame')
        self.pullback = g('room_pullback_m')
        self.repeat_after = g('repeat_after_s')
        self.unread = UnreadTracker(g('unread_need'), g('unread_window'))
        self.unread_period = g('unread_period_s')
        self.unread_range = g('unread_range_m')
        self.unread_full = g('unread_full_scan_s')
        self.unread_last_pub = -1e9    # time of the last /qr/unread
        self.unread_last_seen = -1e9   # time an unread code was last confirmed
        self.read_last = {}            # text -> time of the last /qr/read
        self.full_every = int(g('full_scan_every'))
        rate = g('rate_hz')
        self.min_period = 1.0 / rate if rate > 0 else 0.0
        self.read_everywhere = bool(g('read_everywhere'))
        self.detect_on = bool(g('detect'))
        roam = g('roam_rate_hz')
        self.roam_period = 1.0 / roam if roam > 0 else 0.0
        self.last_taken = -1e9         # time the last frame was handed to the decoder

        self.bridge = CvBridge()
        self.latest_img = None
        self.latest_depth = None
        self.K = None                  # (fx, fy, cx, cy)
        self.align_state = DepthAlignState(bool(g('depth_camera_info_topic')))
        self.seen = {}                 # text -> last report time (s)
        self.new_frame = threading.Event()

        if g('image_transport') == 'compressed':
            image_topic = g('image_topic').rstrip('/') + '/compressed'
            self.camera = [(CompressedImage, image_topic, self.on_image)]
        else:
            image_topic = g('image_topic')
            self.camera = [(Image, image_topic, self.on_image)]
        self.camera.append((CameraInfo, g('camera_info_topic'), self.on_info))
        self.camera_subs = []
        self.on_demand = bool(g('on_demand'))
        if not self.on_demand:
            self.open_camera()
        if self.use_depth:
            self.create_subscription(Image, g('depth_topic'), self.on_depth,
                                     qos_profile_sensor_data)
            if g('depth_camera_info_topic'):
                self.create_subscription(CameraInfo, g('depth_camera_info_topic'),
                                         self.on_depth_info, qos_profile_sensor_data)

        self.pub = self.create_publisher(String, '/qr/detections', 10)
        self.read_pub = self.create_publisher(String, '/qr/read', 10)
        self.unread_pub = (self.create_publisher(PointStamped, g('unread_topic'), 10)
                           if g('unread_topic') else None)
        self.found_pub = self.create_publisher(String, '/found', 10)   # "QR found", once per code

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Robot-side /tf_static gives the depth camera a second, unpitched parent: keep the
        # correct one (team_perception/tf_pin.py).
        self.tf_pin = tf_pin.attach(self, self.tf_buffer)

        self.log_dir = g('log_dir')
        # Record of found codes (snapshot + CSV row per new code); the folder and file are
        # only created when the first code is found, so startup touches no files.
        self.csv_path = os.path.join(self.log_dir, time.strftime('qr_%Y%m%d_%H%M%S.csv'))

        # gate.active: QR detection (dead-end window); gate.running: reading (run_gate.py)
        self.gate = run_gate.attach(self, 'qr_scanner detection')
        if self.on_demand:
            self.create_timer(0.2, self.follow_gate)
            # Started by a launch file for one run: when that launch is gone (it failed or
            # was killed) this node must not stay behind on the robot's ROS domain.
            self.parent = os.getppid()
            self.create_timer(2.0, self.check_parent)

        self.running = True
        self.worker = threading.Thread(target=self.decode_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(f'QR scanner on {image_topic} using {qr_utils.BACKENDS}, '
                               f'logging to {self.csv_path}')

    # --- callbacks ---------------------------------------------------------------
    def detecting(self):
        """Dead-end window: full scans + /qr/unread (FACING_QR); never with detect off."""
        return self.detect_on and self.gate.active

    def open_camera(self):
        if not self.camera_subs:
            self.camera_subs = [self.create_subscription(kind, topic, cb, qos_profile_sensor_data)
                                for kind, topic, cb in self.camera]

    def check_parent(self):
        if os.getppid() != self.parent:
            self.get_logger().warn('the launch that started this scanner is gone: stopping')
            raise SystemExit(0)

    def follow_gate(self):
        """on_demand: the camera streams are ours only while the explorer is on a look."""
        want = self.gate.perceive is not None and self.gate.active
        if want and not self.camera_subs:
            self.open_camera()
            self.get_logger().info('camera on: reading QR codes during this look')
        elif not want and self.camera_subs:
            for sub in self.camera_subs:
                self.destroy_subscription(sub)
            self.camera_subs = []
            self.latest_img = None
            self.get_logger().info('camera off')

    def on_image(self, msg):
        if not self.detecting():
            if not (self.read_everywhere and self.gate.running):
                return                 # paused: frames not even decoded
            now = time.time()
            if now - self.last_taken < self.roam_period:
                return                 # reading only: roam_rate_hz
            self.last_taken = now
        try:
            img = self.to_bgr(msg)     # converted once here, shared with the decode thread
        except Exception as e:  # e.g. unexpected encoding or corrupt JPEG
            self.get_logger().warn(f'image conversion failed: {e}', throttle_duration_sec=5.0)
            return
        self.latest_img = (img, msg.header)
        self.new_frame.set()           # wake the decode thread

    def on_depth(self, msg):
        self.latest_depth = msg

    def on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.align_state.set_colour(self.K, (msg.width, msg.height), msg.header.frame_id)

    def on_depth_info(self, msg):
        self.align_state.set_depth((msg.k[0], msg.k[4], msg.k[2], msg.k[5]),
                                   msg.header.frame_id)

    def to_bgr(self, msg):
        """Image/CompressedImage -> BGR array (bgr8 Image: zero-copy view, no cv_bridge copy)."""
        if isinstance(msg, CompressedImage):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError(f'could not decode {msg.format!r} image')
            return img
        if msg.encoding == 'bgr8' and msg.step == msg.width * 3:
            return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # --- decoding (background thread) ---------------------------------------------
    def decode_loop(self):
        n = 0
        while self.running and rclpy.ok():
            if not self.new_frame.wait(timeout=0.5):
                continue
            self.new_frame.clear()
            frame = self.latest_img
            if frame is None:
                continue
            img, header = frame
            start = time.time()

            detect = self.detecting()
            full = self.full_every > 0 and n % self.full_every == 0
            n += 1
            # While an unreadable code is in view (robot turning to it / holding still),
            # spend the extra time on every frame: contrast boost + 2x upscale.
            full = detect and (full or time.time() - self.unread_last_seen < self.unread_full)
            try:
                codes, unread = scan_qr(img, try_upscale=full, try_clahe=full)
            except Exception as e:
                self.get_logger().error(f'decode failed: {e}', throttle_duration_sec=5.0)
                continue

            now = time.time()
            for text, corners in codes:
                if now - self.read_last.get(text, -1e9) >= 1.0:
                    self.read_last[text] = now
                    self.read_pub.publish(String(data=text))
                self.on_code(text, corners, img, header, now)
            if codes or not detect:
                self.unread.clear()    # something was read / reading only: start afresh
            if detect:
                self.on_unread(unread, img, header, now)

            if self.min_period:
                time.sleep(max(0.0, self.min_period - (time.time() - start)))

    def on_unread(self, unread, img, header, now):
        """Publish a persistent located-but-unread code's position for the explorer."""
        if self.unread_pub is None:
            return
        centres = [(*c.mean(axis=0), cv2.contourArea(c)) for c in unread]
        pick = self.unread.update(centres)
        if pick is None:
            return
        self.unread_last_seen = now
        if now - self.unread_last_pub < self.unread_period:
            return
        pt = self.point_at(pick[0], pick[1], header, img.shape)
        if pt is None:
            return
        self.unread_last_pub = now
        self.unread_pub.publish(pt)
        self.get_logger().info(f'QR code in view but unreadable at {self.track_frame} '
                               f'({pt.point.x:.2f}, {pt.point.y:.2f}): asking to face it',
                               throttle_duration_sec=5.0)

    def point_at(self, u, v, header, shape):
        """
        PointStamped in track_frame (odom) for colour pixel (u, v): from depth if there is
        any there, else unread_range_m along the camera ray (enough to turn towards it).
        """
        if self.K is None:
            return None
        z = None
        if self.use_depth and self.latest_depth is not None:
            depth = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
            align = self.depth_align(depth.shape)
            if align is not None:
                found = align.depth_at(depth, u, v, image_size=(shape[1], shape[0]))
                z = found[0] if found is not None else None
        x, y, z = deproject(u, v, z if z else self.unread_range, *self.K)
        pt = PointStamped()
        pt.header = header
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        out, errors = transform_first(self.tf_buffer, pt, [self.track_frame],
                                      timeout=Duration(seconds=0.2))   # worker thread
        if out is None:
            self.get_logger().warn(f'unreadable QR code: no tf to {self.track_frame} '
                                   f'({errors})', throttle_duration_sec=5.0)
        return out

    def on_code(self, text, corners, img, header, now):
        if now - self.seen.get(text, -1e9) < self.repeat_after:
            return
        if text not in self.seen:      # first time this code is seen: announce it once
            self.get_logger().info('QR found')
            self.found_pub.publish(String(data='QR found'))
        self.seen[text] = now
        try:
            self.report(text, corners, img, header)
        except Exception as e:
            self.get_logger().error(f'report failed for "{text}": {e}')

    def report(self, text, corners, img, header):
        u, v = corners.mean(axis=0)
        room = self.locate(u, v, header, img.shape)

        os.makedirs(self.log_dir, exist_ok=True)
        snap = os.path.join(self.log_dir, f'qr_{int(time.time() * 1000)}.jpg')
        cv2.imwrite(snap, draw_detections(img.copy(), [(text, corners)]))

        result = {'text': text, 'room_odom': room, 'snapshot': snap,
                  'stamp': Time.from_msg(header.stamp).nanoseconds * 1e-9}
        self.pub.publish(String(data=json.dumps(result)))
        new_file = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            if new_file:
                csv.writer(f).writerow(['wall_time', 'text', 'snapshot'])
            csv.writer(f).writerow([time.time(), text, snap])
        self.get_logger().info(f'QR message (M04): "{text}"'
                               + ('' if room else ' (no room point: no depth or TF)'))

    def locate(self, u, v, header, shape):
        """The code's room point [x, y, z] in track_frame (odom), or None (no depth / TF)."""
        if not self.use_depth or self.latest_depth is None or self.K is None:
            return None
        depth = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
        align = self.depth_align(depth.shape)     # colour pixel -> (unaligned) depth pixel
        if align is None:
            return None
        found = align.depth_at(depth, u, v, image_size=(shape[1], shape[0]))
        if found is None:
            return None
        x, y, z = pull_back(deproject(u, v, found[0], *self.K), self.pullback)

        pt = PointStamped()
        pt.header = header                   # camera optical frame + image time
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        # Worker thread, so waiting briefly for TF is safe here (not in a callback).
        out, errors = transform_first(self.tf_buffer, pt, [self.track_frame],
                                      timeout=Duration(seconds=0.2))
        if out is None:
            self.get_logger().warn(f'No tf to {self.track_frame} ({errors}); code has no room '
                                   'point', throttle_duration_sec=5.0)
            return None
        return [round(out.point.x, 3), round(out.point.y, 3), round(out.point.z, 3)]

    def depth_align(self, depth_shape):
        """Return the DepthAlign for this depth image, or None (looks up the sensor TF once)."""
        need = self.align_state.needs_tf()
        if need is not None:
            try:
                tr = self.tf_buffer.lookup_transform(need[0], need[1], Time()).transform
                self.align_state.t = (tr.translation.x, tr.translation.y, tr.translation.z)
            except Exception as e:
                self.get_logger().warn(f'No TF {need[1]} -> {need[0]} ({e}); mapping colour '
                                       'to depth without the sensor offset',
                                       throttle_duration_sec=10.0)
        align, why = self.align_state.get(depth_shape)
        if align is None:
            self.get_logger().warn(f'depth not used: {why}', throttle_duration_sec=5.0)
        return align

    def destroy_node(self):
        self.running = False
        self.new_frame.set()
        self.worker.join(timeout=2.0)
        super().destroy_node()


def exit_with_parent():
    """EXIT_WITH_PARENT=<pid of the launch> (explore.launch.py): have the kernel send this
    process SIGTERM when that parent dies, however it dies, so a scanner never outlives its
    launch. Linux only; without the variable nothing changes."""
    parent = os.environ.get('EXIT_WITH_PARENT', '')
    if not parent.isdigit():
        return
    import ctypes
    import signal
    try:
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)   # PR_SET_PDEATHSIG
    except OSError:
        return
    if os.getppid() != int(parent):          # it died before the line above
        raise SystemExit('qr_scanner: the launch that started it is gone, not starting')


def main():
    exit_with_parent()
    cv2.setNumThreads(1)               # OpenCV's pool would spin on every core per frame
    rclpy.init()
    node = QrScanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
