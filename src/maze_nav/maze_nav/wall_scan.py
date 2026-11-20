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

Curtains (curtains_topic, from curtain_detector, in its map frame; '' = off): points of a
curtain the robot is being walked through, or has been (curtains.scan_curtains), are
dropped: +-0.12 m of its line, and within 1 m of the robot +-0.40 m short of its ends
(fabric pushed or dragged by the robot; curtains.curtain_mask). A confirmed curtain not yet
walked through stays a wall: a false detection must not delete a real wall from the
costmaps. Dropped bins read +inf, which neither the costmaps (inf_is_valid false) nor
slam_toolbox clear with: cells marked earlier stay until the costmap is cleared
(maze_explorer clears the local one on PASSED), and /map keeps the curtain. While the
robot is within 1 m of any confirmed curtain and the lidar is covered
(curtains.BlockageMonitor), no scan is published: a covered lidar would otherwise clear
the map around the robot.
The floor height is published once known on /maze/floor_z (Float32, odom z, latched).

Published scan frame: base_link. Ranges are horizontal distances, so for SLAM / costmaps the
small body pitch on ramps (a few degrees) is ignored, which costs < 1 % of range.
"""
import collections
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import (StandSettle, body_height, cloud_xyz, estimate_floor, floor_accepted,
                           posture, wall_ranges)
from maze_nav.curtains import (BlockageMonitor, curtain_mask, curtains_from_json, invert_pose,
                               lidar_blockage, scan_curtains)


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
        p('band_high', 0.45)  # = wall top: side walls stay in band when the body rides 2-4 cm higher
        p('floor_z', float('nan'))  # odom z of the floor; NaN = measure at startup
        p('expected_floor_drop', 0.35)  # base_link height when standing (odom_relay convention)
        p('floor_tolerance', 0.06)      # measured base_link height above drop + this: ignored
        #                                 (below core.FLOOR_MIN_BODY_HEIGHT 0.27 m too)
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
        # m (3D) from the lidar origin: closer points are its own near/self returns. They reached
        # 0.29 m from the sensor and 0.37 m ahead of base_link, past body_front (run 2026-11-20
        # 01:28: stuck "in collision" from the start). Costs walls within ~0.13 m of the nose.
        p('lidar_min_range', 0.32)
        p('range_max', 12.0)
        # s; older clouds are dropped. 0.8, not 0.5: odometry through the robot bridge is often
        # 0.4-0.8 s late (2026-11-19), and dropping every such cloud stalled /wall_scan for
        # seconds (costmap "/wall_scan ... not updated"). Raised 0.8 -> 1.5 (user 2026-11-20:
        # Fast-LIO odom stalls of ~1 s). Each cloud still uses the TF AT ITS STAMP, but a scan
        # up to 1.5 s old reaches Nav2 that late: at 0.15 m/s the robot has moved up to
        # ~22 cm since (cmd_vel_bridge stops it after 0.5 s without lidar/odom). It hides lag,
        # doesn't fix it: the log counts clouds used at > 0.8 s ("late clouds").
        p('max_cloud_age', 1.5)   # s waiting for TF before dropping a cloud
        p('curtains_topic', '/curtain/curtains')   # curtain_detector; '' = no curtain override
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
        self.lidar_min = g('lidar_min_range')
        self.max_age = g('max_cloud_age')
        self.late, self.used, self.late_log_at = 0, 0, time.monotonic()
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
        self.floor_pub = self.create_publisher(Float32, '/maze/floor_z', latched)
        self.curtains = []           # curtains.Curtain in self.curtains_frame
        self.curtains_frame = self.odom
        self.curtains_tf = None      # last good (tx, ty, th) of curtains_frame in odom
        self.curtain_near = 1.0      # m: a curtain being crossed is dropped from here on
        self.blockage = BlockageMonitor()
        self.curtains_on = bool(g('curtains_topic'))
        if self.curtains_on:
            self.create_subscription(String, g('curtains_topic'), self.on_curtains, latched)
        if self.floor is not None:
            self.floor_pub.publish(Float32(data=float(self.floor)))
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

    def on_curtains(self, msg):
        frame, curtains = curtains_from_json(msg.data)
        if not frame:
            return
        old = len(scan_curtains(self.curtains))
        if len(curtains) != len(self.curtains) or len(scan_curtains(curtains)) != old:
            self.get_logger().info(
                f'{len(curtains)} curtain(s) ({frame}), {len(scan_curtains(curtains))} walked '
                'through (or being): those dropped from the wall scan')
        if frame != self.curtains_frame:
            self.curtains_tf = None
        self.curtains_frame, self.curtains = frame, curtains

    def curtains_odom(self):
        """The curtains in odom (latest TF curtains_frame -> odom; the last good one if TF
        is missing for a moment), [] if never had one."""
        if not self.curtains:
            return []
        if self.curtains_frame == self.odom:
            return self.curtains
        try:
            t = self.tf.lookup_transform(self.curtains_frame, self.odom, Time())
            q = t.transform.rotation
            self.curtains_tf = invert_pose(
                t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        except TransformException:
            if self.curtains_tf is None:
                self.get_logger().warn(f'no TF {self.curtains_frame} -> {self.odom}: curtains '
                                       'not used yet', throttle_duration_sec=10.0)
                return []
        return [c.transformed(*self.curtains_tf) for c in self.curtains]

    def process_pending(self):
        while self.pending:
            msg = self.pending[0]
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > self.max_age:
                self.pending.popleft()
                self.get_logger().warn(f'no TF within {self.max_age} s of a cloud; dropped '
                                       f'({self.tf_problem(msg.header.frame_id)})',
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
            self.used += 1
            self.late += age > 0.8
            if time.monotonic() - self.late_log_at >= 30.0:
                if self.late:
                    self.get_logger().info(f'late clouds: {self.late} of {self.used} in the last '
                                           '30 s used 0.8-' + f'{self.max_age:.1f} s old (would '
                                           'have been dropped at max_cloud_age 0.8)')
                self.late, self.used, self.late_log_at = 0, 0, time.monotonic()
            self.on_cloud(msg, to_odom, base)

    def tf_problem(self, cloud_frame):
        """Which TF is missing for a dropped cloud (latest data, never blocking)."""
        parts = []
        for parent, child in ((self.odom, self.base), (self.base, cloud_frame)):
            try:
                t = self.tf.lookup_transform(parent, child, Time())
            except TransformException as e:
                parts.append(f'{parent}->{child} missing: {str(e).splitlines()[0][:80]}')
                continue
            if t.header.stamp.sec == 0 and t.header.stamp.nanosec == 0:
                parts.append(f'{parent}->{child} static ok')
                continue
            age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
            parts.append(f'{parent}->{child} latest {age:.2f} s old')
        return '; '.join(parts)

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
            ok = floor_accepted(b.translation.z, seen, self.drop, self.floor_tolerance)
            if not ok and self.floor_tries < 20:
                # One bad cloud (robot still rising, odom z jumping) must not latch a guess:
                # wait for the height to settle again and measure again (~20 tries).
                self.settle = StandSettle()
                self.get_logger().info(
                    'floor not believable yet ('
                    + ('none found' if seen is None else
                       f'base_link {b.translation.z - seen:.2f} m above z = {seen:.3f}')
                    + f'); retrying ({self.floor_tries}/20)', throttle_duration_sec=2.0)
                return
            if ok:
                self.floor = seen
                self.floor_pub.publish(Float32(data=float(seen)))
                self.get_logger().info(
                    f'floor at odom z = {seen:.3f} (base_link {b.translation.z - seen:.2f} m '
                    'above it)')
            else:
                self.floor = expected
                self.floor_pub.publish(Float32(data=float(expected)))
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

        if self.floor_guessed:
            self.get_logger().warn(f'floor height is a GUESS ({self.floor_guessed}); '
                                   'see the FLOOR DETECTION FAILED banner above',
                                   throttle_duration_sec=30.0)
        if self.curtains_on:         # learn the usual return count from the start
            curtains = self.curtains_odom()
            near_curtain = any(c.distance(bxy) < 1.0 for c in curtains)
            n_near, n_far = lidar_blockage(pts, m[:3, 3], bxy, yaw, self.body)
            blocked, why = self.blockage.update(n_near, n_far, learn=not near_curtain)
            if blocked and near_curtain:
                self.get_logger().warn(f'lidar covered at a curtain ({why}): no wall scan',
                                       throttle_duration_sec=2.0)
                return
            # Being crossed: dropped only from 1 m. On the way there it stays in the scan:
            # in a corridor it is SLAM's only feature along the way (sim 2026-11-20: map pose
            # 0.5 m behind after a 2 m approach without it).
            drop = [c for c in scan_curtains(curtains)
                    if c.crossed or c.distance(bxy) < self.curtain_near]
            if drop:
                pts = pts[~curtain_mask(pts, drop, bxy)]
        ranges = wall_ranges(pts, bxy, yaw, self.floor, self.band, -math.pi, self.inc, self.n,
                             self.rmin, self.rmax, self.voxel, self.body,
                             sensor=m[:3, 3], sensor_min=self.lidar_min)
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
