"""lidar_relay: take /livox/points from the robot ONCE and pass it on locally.

/livox/points comes from lite3_domain_bridge_4 on another VM, and DDS sends a full copy
(~0.5 MB a cloud, ~5 MB/s) to EVERY process here that subscribes. With wall_scan,
cmd_vel_bridge, stair_detector and curtain_detector each subscribing (plus Fast-LIO's
/livox/lidar), 2026-11-19 robot runs saw clouds 1.1 s late about 10 % of the time (IMU on
time, kernel UDP receive-buffer drops), so Fast-LIO's odom and wall_scan's TF fell ~1 s behind
("no TF within 0.5 s of a cloud", costmap "/wall_scan not updated"). With the relay, one copy
crosses the bridge and the local subscribers get theirs over loopback.

The cloud is forwarded as raw bytes (not decoded in Python), unchanged: same stamp, frame and
fields. Best effort, keep last 2 (all subscribers use qos_profile_sensor_data).

Cropped copy (crop_topic, 2026-11-21): the same cloud with only the points a maze_nav node can
use, so wall_scan, stair_detector and curtain_detector each parse and transform ~40 % of the
points instead of all of them (robot bags: 20k points a cloud, 17-22 % of them the Mid-360's
(0, 0, 0) no-return points, ~40 % ceiling and room above the maze). Kept: finite points more
than 1 cm and at most crop_max_range m from the lidar, crop_low..crop_high m above the lidar
along world up. Every node's own filter is inside that band with margin: wall band
0.30-0.45 m above the floor (lidar ~0.38 m up standing, lower lying down), the stair height map
0.60 m below to 0.30 m above base_link, curtain floor gap, wall_scan range_max 12 m.
World up comes from the latest TF odom -> cloud frame (never blocking; base_link -> cloud
frame if odom isn't there yet); the robot's tilt is taken into account. While the tilt is
changing (> crop_tilt_change deg within 1.5 s: Pose-mode pitch, lying down, stairs), for
pose_pitch_hold s after any non-zero /maze/pose_pitch command (it tilts the body before the
late TF shows it), or no TF is there at all, the height crop is skipped (no-return and out-of-range points still go), so
a TF that is up to ~1 s late (it often is) never cuts walls out. Same stamp, frame, fields.
crop_topic '' = off.

  in : in_topic   (/livox/points)        sensor_msgs/PointCloud2
  out: out_topic  (/livox/points_local)  sensor_msgs/PointCloud2, the same messages
       crop_topic (/livox/points_crop)   sensor_msgs/PointCloud2, cropped (above)

Every `report_period` s it logs the clouds it forwarded and their median delay (receive time -
header stamp), and warns when none arrived.

Every `subscriber_check` s it asks the ROS graph (no data, no subscription) who else
subscribes to in_topic. Each one is another full copy from the bridge (RViz showing
/livox/points, a bag recording it, `ros2 topic hz/echo`), which is what puts the lidar and
Fast-LIO odom late ("no TF within 0.5 s", costmap "/wall_scan … not updated"). It warns with
their node names while there are any, and says once when they are gone. Subscribers on other
machines on the domain are listed too; they cost the bridge, less this machine's link.

    ros2 run maze_nav lidar_relay
"""
import array
import collections
import math
import struct
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import cloud_rows, crop_mask, up_in_frame

OUT_QOS = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT)


def stamp_of(raw):
    """Header stamp (s) of a serialized message: CDR, 4-byte encapsulation, then sec, nanosec."""
    sec, nsec = struct.unpack_from('<iI', raw, 4)
    return sec + nsec * 1e-9


class LidarRelay(Node):

    def __init__(self):
        super().__init__('lidar_relay')
        self.declare_parameter('in_topic', '/livox/points')
        self.declare_parameter('out_topic', '/livox/points_local')
        self.declare_parameter('report_period', 30.0)
        self.declare_parameter('subscriber_check', 5.0)   # s; 0 = off
        self.declare_parameter('crop_topic', '/livox/points_crop')   # '' = off
        self.declare_parameter('crop_low', -1.2)          # m above the lidar (floor ~ -0.38)
        self.declare_parameter('crop_high', 1.0)
        self.declare_parameter('crop_max_range', 12.0)    # m (= wall_scan range_max)
        self.declare_parameter('crop_tilt_change', 4.0)   # deg within 1.5 s: no height crop
        self.declare_parameter('pose_pitch_topic', '/maze/pose_pitch')   # '' = not watched
        self.declare_parameter('pose_pitch_hold', 3.0)    # s after a pitch command: no crop
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.pub = self.create_publisher(PointCloud2, g('out_topic'), OUT_QOS)
        self.create_subscription(PointCloud2, g('in_topic'), self.on_cloud,
                                 qos_profile_sensor_data, raw=True)
        self.count, self.delays = 0, []
        self.period = float(g('report_period'))
        self.create_timer(self.period, self.report)
        self.in_topic, self.extra, self.warned = g('in_topic'), [], 0.0
        if float(g('subscriber_check')) > 0:
            self.create_timer(float(g('subscriber_check')), self.check_subscribers)
        self.crop_pub = None
        if g('crop_topic'):
            self.crop_pub = self.create_publisher(PointCloud2, g('crop_topic'), OUT_QOS)
            self.band = (float(g('crop_low')), float(g('crop_high')))
            self.max_range = float(g('crop_max_range'))
            self.tilt_change = math.radians(float(g('crop_tilt_change')))
            self.frames = (g('odom_frame'), g('base_frame'))
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.ups = collections.deque()           # (monotonic s, up in cloud frame)
            self.kept = self.total = self.uncropped = 0
            self.crop_ms = []
            self.pitch_hold, self.pitched = float(g('pose_pitch_hold')), -1e9
            if g('pose_pitch_topic'):
                self.create_subscription(Float32, g('pose_pitch_topic'), self.on_pitch, 10)
        self.get_logger().info(
            f'{g("in_topic")} -> {g("out_topic")} (raw, one copy from the bridge for all local '
            'subscribers)' + (f' and {g("crop_topic")} (cropped: no-return points, '
                              f'{self.band[0]:+.1f}..{self.band[1]:+.1f} m around the lidar, '
                              f'<= {self.max_range:g} m)' if self.crop_pub else ''))

    def on_cloud(self, raw):
        self.pub.publish(raw)
        self.count += 1
        if len(self.delays) < 1000:
            try:
                self.delays.append(time.time() - stamp_of(raw))
            except struct.error:
                pass
        if self.crop_pub is not None and self.crop_pub.get_subscription_count() > 0:
            t0 = time.perf_counter()
            self.publish_crop(deserialize_message(raw, PointCloud2))
            if len(self.crop_ms) < 1000:
                self.crop_ms.append((time.perf_counter() - t0) * 1e3)

    def on_pitch(self, msg):
        """The explorer's Pose-mode pitch command: the body tilts before TF says so."""
        if not math.isfinite(msg.data) or abs(msg.data) > 1e-3:
            self.pitched = time.monotonic()

    def up(self, frame):
        """World up in `frame` from the latest TF, or None while the tilt is changing or
        no TF is there (never blocks)."""
        q = None
        for parent in self.frames:
            try:
                q = self.tf.lookup_transform(parent, frame, Time()).transform.rotation
                break
            except TransformException:
                continue
        if q is None:
            return None
        up = up_in_frame(q.x, q.y, q.z, q.w)
        now = time.monotonic()
        self.ups.append((now, up))
        while self.ups and now - self.ups[0][0] > 1.5:
            self.ups.popleft()
        steady = all(float(np.dot(u, up)) >= math.cos(self.tilt_change) for _, u in self.ups)
        steady = steady and now - self.pitched > self.pitch_hold
        return up if steady else None

    def publish_crop(self, msg):
        try:
            rows, xyz = cloud_rows(msg.data, msg.fields, msg.point_step, msg.row_step,
                                   msg.width, msg.height, msg.is_bigendian)
        except (ValueError, KeyError) as e:
            self.get_logger().error(f'cloud not croppable ({e}): forwarded whole',
                                    throttle_duration_sec=30.0)
            self.crop_pub.publish(msg)
            return
        up = self.up(msg.header.frame_id)
        keep = crop_mask(xyz, up, self.band[0], self.band[1], self.max_range)
        out = PointCloud2(header=msg.header, height=1, width=int(keep.sum()),
                          fields=msg.fields, is_bigendian=msg.is_bigendian,
                          point_step=msg.point_step, is_dense=True)
        out.row_step = out.width * out.point_step
        data = array.array('B')                 # array('B'), not bytes: rclpy copies bytes slowly
        data.frombytes(rows[keep].tobytes())
        out.data = data
        self.crop_pub.publish(out)
        self.kept += out.width
        self.total += len(xyz)
        self.uncropped += up is None

    def check_subscribers(self):
        """Warn about every other subscriber to in_topic (each costs a bridge copy)."""
        me = (self.get_name(), self.get_namespace())
        extra = sorted({(i.node_namespace.rstrip('/') + '/' + i.node_name).lstrip('/') or '?'
                        for i in self.get_subscriptions_info_by_topic(self.in_topic)
                        if (i.node_name, i.node_namespace) != me})
        now = time.monotonic()
        if extra and (extra != self.extra or now - self.warned > 30.0):
            self.warned = now
            self.get_logger().warn(
                f'{len(extra)} other subscriber(s) on {self.in_topic}: {", ".join(extra)}. '
                'Each is another full lidar copy from the bridge and makes the lidar and odom '
                'late: close them (RViz: show /livox/points_local; bags: record '
                '/livox/points_local)')
        elif not extra and self.extra:
            self.get_logger().info(f'{self.in_topic}: other subscribers gone')
        self.extra = extra

    def report(self):
        if self.count == 0:
            self.get_logger().warn(f'no clouds in {self.period:.0f} s: is the lidar / bridge up?')
        else:
            d = sorted(self.delays)
            crop = ''
            if self.crop_pub is not None and self.total:
                ms = sorted(self.crop_ms)
                crop = (f'; crop kept {100 * self.kept / self.total:.0f} % of points, '
                        f'{self.uncropped} cloud(s) without the height crop (tilt changing / '
                        f'no TF), {ms[len(ms) // 2]:.1f} ms a cloud')
            self.get_logger().info(
                (f'{self.count / self.period:.1f} clouds/s forwarded; delay median '
                 f'{d[len(d) // 2]:.2f} s, worst {d[-1]:.2f} s' if d else
                 f'{self.count / self.period:.1f} clouds/s forwarded') + crop)
        self.count, self.delays = 0, []
        if self.crop_pub is not None:
            self.kept = self.total = self.uncropped = 0
            self.crop_ms = []


def main():
    rclpy.init()
    node = LidarRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
