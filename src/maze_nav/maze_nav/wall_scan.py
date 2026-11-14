"""/livox/points -> /wall_scan: a 2D scan of the maze walls only.

Why not lite3_launcher's /scan: it keeps 0.1..0.5 m above base_link, i.e. ~0.45..0.85 m above
the floor. The example maze walls are only 0.45 m tall, so that scan sees none of them. A band
fixed to base_link also moves with the body: on the 0.15 m ramps it would slide up and down.

This node transforms each cloud into odom (gravity-aligned, Fast-LIO), keeps points between
band_low and band_high above the maze floor and bins them by bearing around the robot. Terrain
(ramps, mats <= 0.15 m), stair treads (<= 0.26 m) and the floor stay out; walls (0.45 m) and
hand rails stay in. The floor height is measured once at startup (robot standing on the
start cell) unless floor_z is given.

Published scan frame: base_link. Ranges are horizontal distances, so for SLAM / costmaps the
small body pitch on ramps (a few degrees) is ignored, which costs < 1 % of range.
"""
import collections
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import cloud_xyz, estimate_floor, wall_ranges


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_to_mat(t):
    m = np.eye(4)
    m[:3, :3] = quat_to_mat(t.transform.rotation)
    tr = t.transform.translation
    m[:3, 3] = (tr.x, tr.y, tr.z)
    return m


class WallScan(Node):

    def __init__(self):
        super().__init__('wall_scan')
        p = self.declare_parameter
        p('cloud_topic', '/livox/points')
        p('scan_topic', '/wall_scan')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('band_low', 0.30)       # m above the maze floor
        p('band_high', 0.43)
        p('floor_z', float('nan'))  # odom z of the floor; NaN = measure at startup
        p('expected_floor_drop', 0.35)  # base_link height when standing (odom_relay convention)
        p('floor_tolerance', 0.06)      # measured floor further than this from expected: ignored
        p('angle_increment', math.radians(0.5))
        p('range_min', 0.25)
        p('range_max', 12.0)
        p('max_cloud_age', 0.5)   # s; older clouds are dropped
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.odom, self.base = g('odom_frame'), g('base_frame')
        self.band = (g('band_low'), g('band_high'))
        self.floor = g('floor_z')
        self.floor = None if math.isnan(self.floor) else float(self.floor)
        self.drop = g('expected_floor_drop')
        self.floor_tolerance = g('floor_tolerance')
        self.inc = g('angle_increment')
        self.n = int(round(2 * math.pi / self.inc))
        self.rmin, self.rmax = g('range_min'), g('range_max')
        self.max_age = g('max_cloud_age')
        self.floor_tries = 0

        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(LaserScan, g('scan_topic'), 10)
        # Never block waiting for TF: the odometry for a cloud (Fast-LIO) arrives just after
        # the cloud itself. Clouds wait in a short queue and a timer handles each one as soon
        # as TF covers its stamp (TF keeps flowing in between on this node's executor).
        self.pending = collections.deque(maxlen=3)
        self.create_subscription(PointCloud2, g('cloud_topic'), self.pending.append,
                                 qos_profile_sensor_data)
        self.create_timer(0.02, self.process_pending)
        self.get_logger().info(
            f'{g("cloud_topic")} -> {g("scan_topic")}: walls {self.band[0]:.2f}..'
            f'{self.band[1]:.2f} m above the floor '
            + (f'(floor_z {self.floor:.3f} given)' if self.floor is not None
               else '(floor measured at startup: robot must be STANDING)'))

    def process_pending(self):
        while self.pending:
            msg = self.pending[0]
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > self.max_age:
                self.pending.popleft()
                self.get_logger().warn(f'no TF within {self.max_age} s of a cloud; dropped',
                                       throttle_duration_sec=5.0)
                continue
            if not (self.tf.can_transform(self.odom, msg.header.frame_id, stamp)
                    and self.tf.can_transform(self.odom, self.base, stamp)):
                return                          # TF not there yet; try again next tick
            self.pending.popleft()
            try:
                to_odom = self.tf.lookup_transform(self.odom, msg.header.frame_id, stamp)
                base = self.tf.lookup_transform(self.odom, self.base, stamp)
            except TransformException as e:
                self.get_logger().warn(f'no TF for cloud: {e}', throttle_duration_sec=5.0)
                continue
            self.on_cloud(msg, to_odom, base)

    def on_cloud(self, msg, to_odom, base):
        xyz = cloud_xyz(msg.data, msg.fields, msg.point_step, msg.row_step, msg.width,
                        msg.height, msg.is_bigendian)
        if xyz.size == 0:
            return
        m = tf_to_mat(to_odom)
        pts = xyz.astype(np.float64) @ m[:3, :3].T + m[:3, 3]
        b = base.transform
        bxy = (b.translation.x, b.translation.y)
        q = b.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if self.floor is None:
            expected = b.translation.z - self.drop
            seen = estimate_floor(pts, bxy, b.translation.z, self.drop)
            self.floor_tries += 1
            if seen is not None and abs(seen - expected) <= self.floor_tolerance:
                self.floor = seen
                self.get_logger().info(
                    f'floor at odom z = {seen:.3f} (base_link {b.translation.z - seen:.2f} m '
                    'above it)')
            elif seen is not None or self.floor_tries >= 20:
                self.floor = expected
                self.get_logger().warn(
                    f'floor seen at {None if seen is None else round(seen, 3)}, expected '
                    f'{expected:.3f} ({self.drop} m below base_link): using {expected:.3f}. '
                    'Was the robot standing at launch? Else set floor_z.')
            else:
                return

        ranges = wall_ranges(pts, bxy, yaw, self.floor, self.band, -math.pi, self.inc, self.n,
                             self.rmin, self.rmax)
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.base
        scan.angle_min = -math.pi
        scan.angle_increment = self.inc
        scan.angle_max = -math.pi + self.inc * (self.n - 1)
        scan.scan_time = 0.1
        scan.range_min, scan.range_max = float(self.rmin), float(self.rmax)
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = WallScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
