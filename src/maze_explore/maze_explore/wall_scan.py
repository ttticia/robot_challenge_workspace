"""Lidar cloud -> levelled, floor-referenced slice for SLAM and for the explorer.

The body pitches and rolls on terrain and the lidar is tilted 15 degrees, so heights are taken
in a gravity-levelled frame and counted from the FLOOR, not from the body:

  in   cloud_topic   Fast-LIO's /cloud_registered_body (sim: /livox/points)
       /odom         pose of base_link in odom (roll, pitch, height)
  out  /scan         LaserScan of the points 0.32-0.48 m above the floor (under the 0.53 m wall
                     tops, above terrain <= 0.25 m), frame base_level  -> slam_toolbox.
                     Built from the last `scan_clouds` clouds (0.5 s): one Mid-360 cloud puts
                     only a few points into a 16 cm band
       /maze/cloud   PointCloud2 in base_level with z = height above the floor (0.05-1.3 m,
                     within 3.5 m, thinned)  -> explorer height map and front check
       TF odom -> base_level   base_link's position and heading, without roll and pitch

/maze/blind (Bool, from the explorer) is True while the robot pushes through a curtain: the
fabric lies over the lidar, so no /scan goes out then (/maze/cloud still does) (SLAM would match the fabric and move
the robot in the map) and for blind_hold s after.

Floor: measured once at the start (robot standing still): the lowest dense layer of points
1-3.5 m away gives the standing height; inside a cell the walls hide that floor, then the
robot's own leg odometry height (/leg_odom2 z) is used, then `stand_height`; from then on height = odom z + that offset. While
the floor is clearly in view again it is nudged (Fast-LIO's z drifts slowly).
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quat_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
    a = np.frombuffer(msg.data, dt, count=msg.width * msg.height)
    p = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
    return p[np.isfinite(p).all(axis=1)]


def floor_below(level, lo=1.0, hi=3.5, min_points=150):
    """Height of base_link above the floor from levelled points (x, y, z rel. base_link), or
    None: the densest 2 cm layer among the points 0.15-0.55 m below the body, 1-3.5 m away."""
    r = np.hypot(level[:, 0], level[:, 1])
    z = level[(r > lo) & (r < hi) & (level[:, 2] < -0.15) & (level[:, 2] > -0.55), 2]
    if len(z) < min_points:
        return None
    h, e = np.histogram(z, bins=np.arange(-0.55, -0.149, 0.02))
    k = int(np.argmax(h))
    if h[k] < min_points * 0.5:
        return None
    return -float(np.mean(z[(z >= e[k] - 0.02) & (z <= e[k + 1] + 0.02)]))


def make_scan(level_xy, bins=720, range_min=0.35, range_max=8.0):
    r = np.hypot(level_xy[:, 0], level_xy[:, 1])
    ok = (r >= range_min) & (r <= range_max)
    k = ((np.arctan2(level_xy[ok, 1], level_xy[ok, 0]) + math.pi) / (2 * math.pi) * bins)
    k = np.clip(k.astype(int), 0, bins - 1)
    out = np.full(bins, np.inf, np.float32)
    np.minimum.at(out, k, r[ok].astype(np.float32))
    return out


class WallScan(Node):

    def __init__(self):
        super().__init__('wall_scan')
        p = self.declare_parameter
        self.band = (p('band_low', 0.32).value, p('band_high', 0.48).value)
        self.range_min = p('range_min', 0.35).value
        self.range_max = p('range_max', 8.0).value
        self.default_stand = p('stand_height', 0.30).value
        self.base_frame = p('base_frame', 'base_link').value
        self.level_frame = p('level_frame', 'base_level').value
        self.max_odom_age = p('max_odom_age', 1.0).value
        cloud_topic = p('cloud_topic', '/cloud_registered_body').value

        self.mount = None                 # (R, t): cloud frame -> base_link
        self.odoms = deque(maxlen=50)     # (stamp s, position, R)
        self.floor_z = None               # floor height in odom
        self.stand_samples = []
        self.blind_hold = p('blind_hold', 1.0).value
        self.blind_until = 0.0
        self.leg_z = None
        self.recent = deque(maxlen=p('scan_clouds', 5).value)     # band points in odom (x, y)
        self.create_subscription(Odometry, '/leg_odom2', self.on_leg, qos_profile_sensor_data)
        self.create_subscription(Bool, '/maze/blind', self.on_blind, 5)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/maze/cloud', 5)
        self.height_pub = self.create_publisher(Float32, '/maze/body_height', 5)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_profile_sensor_data)

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        self.odoms.append((t, np.array([p.x, p.y, p.z]), quat_rot(msg.pose.pose.orientation)))

    def on_leg(self, msg):
        self.leg_z = msg.pose.pose.position.z

    def on_blind(self, msg):
        if msg.data:
            self.blind_until = self.get_clock().now().nanoseconds * 1e-9 + self.blind_hold

    def on_cloud(self, msg):
        if self.mount is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except TransformException as e:
                self.get_logger().info(f'waiting for {self.base_frame} <- {msg.header.frame_id}: {e}',
                                       throttle_duration_sec=5.0)
                return
            t = tf.transform.translation
            self.mount = (quat_rot(tf.transform.rotation), np.array([t.x, t.y, t.z]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.odoms:
            return
        ot, pos, rot = min(self.odoms, key=lambda o: abs(o[0] - stamp))
        if abs(ot - stamp) > self.max_odom_age:
            self.get_logger().warn(f'/odom is {stamp - ot:.2f} s from the cloud: slice skipped',
                                   throttle_duration_sec=2.0)
            return
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        cz, sz = math.cos(yaw), math.sin(yaw)
        unyaw = np.array([[cz, sz, 0], [-sz, cz, 0], [0, 0, 1]])
        level = (cloud_xyz(msg) @ self.mount[0].T + self.mount[1]) @ (unyaw @ rot).T

        stand = floor_below(level)
        if self.floor_z is None:
            self.stand_samples.append(stand)
            if len(self.stand_samples) < 10:
                return
            good = [s for s in self.stand_samples if s is not None and 0.24 <= s <= 0.42]
            if len(good) >= 3:
                h = float(np.median(good))
                self.get_logger().info(f'floor found: base_link stands {h:.2f} m above it')
            elif self.leg_z is not None and 0.24 <= self.leg_z <= 0.42:
                h = self.leg_z
                self.get_logger().info(f'floor hidden by walls; leg odometry: standing {h:.2f} m')
            else:
                h = self.default_stand
                self.get_logger().warn(f'floor not measured (lidar {self.stand_samples[-1]}, legs '
                                       f'{self.leg_z}); assuming a standing height of {h:.2f} m')
            self.floor_z = pos[2] - h
        elif stand is not None and abs((pos[2] - stand) - self.floor_z) < 0.08:
            self.floor_z += 0.05 * ((pos[2] - stand) - self.floor_z)
        body = pos[2] - self.floor_z
        self.height_pub.publish(Float32(data=float(body)))
        height = level[:, 2] + body

        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id = msg.header.stamp, 'odom'
        tf.child_frame_id = self.level_frame
        tf.transform.translation.x, tf.transform.translation.y = float(pos[0]), float(pos[1])
        tf.transform.translation.z = float(self.floor_z)
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_pub.sendTransform(tf)

        r = np.hypot(level[:, 0], level[:, 1])
        keep = (height > 0.05) & (height < 1.3) & (r < 3.5) & (r > 0.30)
        pts = np.column_stack([level[keep, :2], height[keep]]).astype(np.float32)
        if len(pts):
            _, first = np.unique(np.round(pts / 0.04).astype(np.int32), axis=0, return_index=True)
            pts = pts[first]
        out = PointCloud2()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.level_frame
        out.height, out.width = 1, len(pts)
        out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        out.point_step, out.row_step, out.is_dense = 12, 12 * len(pts), True
        out.data = pts.tobytes()
        self.cloud_pub.publish(out)

        if self.get_clock().now().nanoseconds * 1e-9 < self.blind_until:
            self.recent.clear()
            return
        band = (height >= self.band[0]) & (height <= self.band[1])
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = msg.header.stamp, self.level_frame
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 720
        scan.angle_max = math.pi - scan.angle_increment
        scan.range_min, scan.range_max = self.range_min, self.range_max
        scan.scan_time = 0.1
        self.recent.append(level[band, :2] @ np.array([[cz, sz], [-sz, cz]]) + pos[:2])
        merged = (np.concatenate(self.recent) - pos[:2]) @ np.array([[cz, -sz], [sz, cz]])
        ranges = make_scan(merged, 720, self.range_min, self.range_max)
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)



def main():
    rclpy.init()
    node = WallScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
