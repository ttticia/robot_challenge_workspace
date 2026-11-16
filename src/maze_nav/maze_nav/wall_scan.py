"""/livox/points -> /wall_scan: a 2D scan of the maze walls only.

Why not lite3_launcher's /scan: it keeps 0.1..0.5 m above base_link, i.e. ~0.45..0.85 m above
the floor. The example maze walls are only 0.45 m tall, so that scan sees none of them. A band
fixed to base_link also moves with the body: on the 0.15 m ramps it would slide up and down.

This node transforms each cloud into odom (gravity-aligned, Fast-LIO), keeps points between
band_low and band_high above the maze floor and bins them by bearing around the robot. Terrain
(ramps, mats <= 0.15 m), stair treads (<= 0.26 m) and the floor stay out; walls (0.45 m) and
hand rails stay in. The floor height is measured once at startup (robot standing on the
start cell) unless floor_z is given. Until then it publishes base_link's height above the
floor on /maze/body_height (latched): cmd_vel_bridge uses it to stand a lying robot up, and
the floor isn't measured while the robot is lying down or getting up.

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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import StandSettle, body_height, cloud_xyz, estimate_floor, posture, wall_ranges


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
        p('angle_increment', math.radians(0.5))   # 720 beams; ~2.6 cm apart at 3 m (1 deg missed wall ends in the sim)
        # m; > 0 thins wall points to one per cell. OFF: at 0.10 the sim map had dotted walls
        # and free space only along spokes (1 beam in ~11 hit), and exploring stopped at start.
        p('voxel_size', 0.0)
        p('range_min', 0.25)
        # Robot body box (base_link frame): footprint x -0.37..0.27, y +-0.23 plus 5 cm. Lidar
        # points inside are the robot's own legs/body, not walls.
        p('body_back', 0.42)
        p('body_front', 0.32)
        p('body_half_width', 0.28)
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
        self.voxel = g('voxel_size')
        self.body = (g('body_back'), g('body_front'), g('body_half_width'))
        self.max_age = g('max_cloud_age')
        self.floor_tries = 0
        self.floor_guessed = None    # why the floor height is a guess, if it is
        self.settle = StandSettle()  # floor measured only once the robot is fully up
        self.unsettled = 0           # standing clouds seen while the height still changes

        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(LaserScan, g('scan_topic'), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.height_pub = self.create_publisher(Float32, '/maze/body_height', latched)
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
            # Not standing yet (lying at launch, cmd_vel_bridge stands it up): wait, or the
            # floor would come out 0.26 m too high. Unknown height: the old rules below.
            h = body_height(pts, bxy, b.translation.z)
            if h is not None:
                self.height_pub.publish(Float32(data=float(h)))
            if h is not None and posture(h) != 'standing':
                self.get_logger().warn(
                    f'robot not standing (base_link {h:.2f} m above the floor): floor '
                    'measured once it stands', throttle_duration_sec=5.0)
                return
            # Still getting up (0.25 m already counts as standing, fully up is ~0.35 m):
            # wait until the height stops changing. Give up waiting after ~5 s of clouds.
            if h is not None and not self.settle.update(h) and self.unsettled < 50:
                self.unsettled += 1
                self.get_logger().info(
                    f'robot getting up (base_link {h:.2f} m above the floor): floor '
                    'measured once its height is steady', throttle_duration_sec=2.0)
                return
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
                self.floor_guessed = (
                    'no floor found in the lidar cloud' if seen is None else
                    f'floor seen at z = {seen:.3f}, {abs(seen - expected):.2f} m from the '
                    f'expected {expected:.3f}')
                bar = '!' * 78
                self.get_logger().error(
                    f'\n{bar}\n!!! FLOOR DETECTION FAILED: {self.floor_guessed}.\n'
                    f'!!! Using a GUESS: floor at odom z = {expected:.3f} ({self.drop} m below '
                    'base_link).\n'
                    '!!! If that is wrong, the wall band is at the wrong height: walls go '
                    'missing or terrain/stairs show up as walls.\n'
                    '!!! Was the robot STANDING at launch? Stand it up and relaunch, or set '
                    'floor_z:=<odom z of the floor>.\n' + bar)
            else:
                return

        if self.floor_guessed:
            self.get_logger().warn(f'floor height is a GUESS ({self.floor_guessed}); '
                                   'see the FLOOR DETECTION FAILED banner above',
                                   throttle_duration_sec=30.0)
        ranges = wall_ranges(pts, bxy, yaw, self.floor, self.band, -math.pi, self.inc, self.n,
                             self.rmin, self.rmax, self.voxel, self.body)
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
