#!/usr/bin/env python3
"""stair_watch: detect the challenge stairs with the depth camera (ROS 2 Humble node).

Detection: stair_levels.py from the `stairs` package (tread levels at 12 / 24 / 36 cm, see its doc),
then the lidar must show the triangle-shaped handrails on both sides (rails_check).
Needs NO odometry: gravity (roll / pitch) comes from the IMU, the camera mount from
/tf_static (base_link -> camera_link, falls back to the measured mount). It never drives.

  in : depth_topic (sensor_msgs/Image 16UC1 mm or 32FC1 m), info_topic (CameraInfo),
       imu_topic (sensor_msgs/Imu, orientation), /tf_static; optional TF odom -> base_link
  out: /stairs/status     std_msgs/String, latched: "NONE" | "SEEN: ..." | "CONFIRMED: ..."
       /stairs/detection  std_msgs/String, JSON for every processed frame (why it did or
                          didn't count)
       /stairs/pose       geometry_msgs/PoseStamped in base_link (level), while CONFIRMED:
                          middle of the bottom riser, x pointing up the stairs
       /stairs/pose_odom  the same in odom, only if TF odom -> base_link exists

Run (robot, ROS_DOMAIN_ID=4, ROS sourced):
    python3 stair_watch.py
    python3 stair_watch.py --ros-args -p rate:=5.0 -p log_every_frame:=true
Watch:
    ros2 topic echo /stairs/status
"""
import json
import math
import os
import sys
import time


import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,  # noqa: E402
                       qos_profile_sensor_data)
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import Buffer, TransformException, TransformListener  # noqa: E402

from stairs import stair_levels as S  # noqa: E402  (team package `stairs`)

# Measured mount (robot /tf_static, 23 Nov bags): base_link -> camera_link, pitched 20 deg down
DEFAULT_CAM_T = (0.25228, -0.0005, 0.0741)
DEFAULT_CAM_Q = (0.0, 0.17364817766693033, 0.0, 0.984807753012208)
# base_link -> livox_frame (robot /tf_static), pitched 15 deg down
DEFAULT_LIDAR_T = (0.18678, 0.0, 0.12498)
DEFAULT_LIDAR_Q = (0.0, 0.13052619222005157, 0.0, 0.9914448613738104)


def quat_xyzw(q):
    return S.quat_to_matrix(q.x, q.y, q.z, q.w)


class StairWatch(Node):
    def __init__(self):
        super().__init__('stair_watch')
        p = self.declare_parameter
        p('depth_topic', '/camera/depth/image_raw')
        p('info_topic', '/camera/depth/camera_info')
        p('imu_topic', '/imu/data')
        p('base_frame', 'base_link')
        p('camera_link_frame', 'camera_link')
        p('odom_frame', 'odom')            # '' = never publish /stairs/pose_odom
        p('rate', 5.0)                     # Hz, frames processed
        p('rise', 0.12)
        p('tread', 0.20)
        p('steps', 3)
        p('body_height', 0.35)
        p('vote_window', 5)
        p('vote_need', 3)
        p('log_every_frame', False)
        p('lidar_topic', '/livox/points')  # handrail check; '' = camera only
        p('require_rails', True)           # stairs only with the triangle handrails seen
        p('lidar_window_sec', 1.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.params = S.LevelParams(rise=float(g('rise')), tread=float(g('tread')),
                                    steps=int(g('steps')), body_height=float(g('body_height')))
        self.confirm = S.Confirm(window=int(g('vote_window')), need=int(g('vote_need')))
        self.base, self.link, self.odom = g('base_frame'), g('camera_link_frame'), g('odom_frame')
        self.period = 1.0 / max(0.5, float(g('rate')))
        self.log_all = bool(g('log_every_frame'))
        self.require_rails = bool(g('require_rails')) and bool(g('lidar_topic'))
        self.lidar_window = float(g('lidar_window_sec'))
        self.clouds = []                   # (monotonic t, livox xyz)
        self.lidar_mount = (S.quat_to_matrix(*DEFAULT_LIDAR_Q), np.array(DEFAULT_LIDAR_T))
        self.lidar_frame = None

        self.k = None
        self.depth = None
        self.imu_q = None
        self.imu_t = 0.0
        self.mount = None
        self.last_status = None
        self.last_status_pub = 0.0
        self.frames = self.hits = 0

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, '/stairs/status', latched)
        self.pub_det = self.create_publisher(String, '/stairs/detection', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/stairs/pose', 10)
        self.pub_pose_odom = self.create_publisher(PoseStamped, '/stairs/pose_odom', 10)
        self.create_subscription(Image, g('depth_topic'), self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('info_topic'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, qos_profile_sensor_data)
        if g('lidar_topic'):
            self.create_subscription(PointCloud2, g('lidar_topic'), self.on_cloud,
                                     qos_profile_sensor_data)
        self.create_timer(self.period, self.tick)
        self.create_timer(5.0, self.report)
        self.set_status('NONE', force=True)
        self.get_logger().info(
            f'stair_watch: {g("depth_topic")} + {g("imu_topic")}, stairs {self.params.steps} x '
            f'{self.params.rise * 100:.0f} cm / {self.params.tread * 100:.0f} cm, {1 / self.period:.0f} Hz')

    # ------------------------------------------------------------------ inputs
    def on_info(self, msg):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg):
        self.depth = msg

    def on_imu(self, msg):
        self.imu_q = msg.orientation
        self.imu_t = time.monotonic()

    def on_cloud(self, msg):
        if self.lidar_frame is None:
            self.lidar_frame = msg.header.frame_id
            try:
                tf = self.tf.lookup_transform(self.base, msg.header.frame_id, Time())
                tr, q = tf.transform.translation, tf.transform.rotation
                self.lidar_mount = (quat_xyzw(q), np.array([tr.x, tr.y, tr.z]))
            except TransformException:
                self.lidar_frame = None     # retry with the next cloud; default mount meanwhile
        names = [f.name for f in msg.fields]
        off = {f.name: f.offset for f in msg.fields}
        if not all(n in names for n in 'xyz'):
            return
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                       'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
        a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
        xyz = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(1)]
        now = time.monotonic()
        self.clouds = [(t, c) for t, c in self.clouds if now - t <= self.lidar_window] + [(now, xyz)]

    def lidar_points(self, r_level_base):
        now = time.monotonic()
        cl = [c for t, c in self.clouds if now - t <= self.lidar_window]
        if not cl:
            return None
        return S.lidar_level(np.concatenate(cl), r_level_base, *self.lidar_mount)

    def camera_mount(self):
        if self.mount is not None:
            return self.mount
        try:
            tf = self.tf.lookup_transform(self.base, self.link, Time())
            tr, q = tf.transform.translation, tf.transform.rotation
            self.mount = (quat_xyzw(q), np.array([tr.x, tr.y, tr.z]))
            self.get_logger().info(f'camera mount from TF {self.base} -> {self.link}: '
                                   f'({tr.x:.3f}, {tr.y:.3f}, {tr.z:.3f})')
        except TransformException:
            if not hasattr(self, '_mount_wait'):
                self._mount_wait = time.monotonic()
            if time.monotonic() - self._mount_wait > 3.0:
                self.mount = (S.quat_to_matrix(*DEFAULT_CAM_Q), np.array(DEFAULT_CAM_T))
                self.get_logger().warn(f'no TF {self.base} -> {self.link}: using the measured '
                                       'mount (0.252, 0, 0.074), 20 deg down')
        return self.mount

    @staticmethod
    def image(msg):
        if msg.encoding in ('16UC1', 'mono16'):
            dt = np.uint16
        elif msg.encoding == '32FC1':
            dt = np.float32
        else:
            return None
        a = np.frombuffer(bytes(msg.data), dtype=dt)
        item = np.dtype(dt).itemsize
        a = a.reshape(msg.height, msg.step // item)[:, :msg.width]
        return a.byteswap() if msg.is_bigendian else a

    # ------------------------------------------------------------------ main loop
    def tick(self):
        msg, self.depth = self.depth, None
        if msg is None:
            return
        why = None
        if self.k is None:
            why = 'no camera_info yet'
        elif self.imu_q is None or time.monotonic() - self.imu_t > 1.0:
            why = 'no IMU orientation (needed for gravity)'
        elif self.camera_mount() is None:
            why = 'waiting for TF base_link -> camera_link (3 s, then default mount)'
        if why:
            self.get_logger().info(why, throttle_duration_sec=5.0)
            return
        depth = self.image(msg)
        if depth is None:
            self.get_logger().error(f'depth encoding {msg.encoding} not supported',
                                    throttle_duration_sec=10.0)
            return
        r_level_base = S.roll_pitch_only(quat_xyzw(self.imu_q))
        r, t = S.level_camera(r_level_base, *self.mount)
        t0 = time.monotonic()
        seen = S.detect(depth, self.k, r, t, self.params)
        rails = ''
        if seen is not None and self.require_rails:
            ok, rails = S.rails_check(self.lidar_points(r_level_base), seen)
            if not ok:
                S.detect.last_why = f'treads look like stairs but {rails}'
                seen = None
        ms = (time.monotonic() - t0) * 1000
        self.frames += 1
        self.hits += seen is not None
        now = time.monotonic()
        conf = self.confirm.add(now, seen)

        out = {'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 'ms': round(ms),
               'seen': seen is not None, 'why': S.detect.last_why, 'confirmed': conf is not None}
        if seen is not None:
            out.update(distance=round(seen.distance, 3), yaw_deg=round(math.degrees(seen.yaw), 1),
                       lateral=round(seen.lateral, 3), width=round(seen.width, 2),
                       levels=[lv[0] for lv in seen.levels], inferred_bottom=seen.inferred)
        self.pub_det.publish(String(data=json.dumps(out)))
        out['rails'] = rails
        if self.log_all:
            self.get_logger().info(f'{seen.summary()}; {rails}' if seen else f'no stairs: {S.detect.last_why}')

        if conf is not None:
            self.set_status(f'CONFIRMED: {conf.summary()}')
            self.publish_pose(conf, msg.header.stamp)
        elif seen is not None:
            self.set_status(f'SEEN: {seen.summary()}')
        else:
            self.set_status('NONE')

    def publish_pose(self, s, stamp):
        x = s.distance * math.cos(s.yaw) - s.lateral * math.sin(s.yaw)
        y = s.distance * math.sin(s.yaw) + s.lateral * math.cos(s.yaw)
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.base
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.position.z = float(s.floor_z)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(s.yaw / 2), math.cos(s.yaw / 2)
        self.pub_pose.publish(ps)
        if not self.odom:
            return
        try:
            tf = self.tf.lookup_transform(self.odom, self.base, Time())
        except TransformException:
            return
        tr, q = tf.transform.translation, tf.transform.rotation
        yaw_b = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, sn = math.cos(yaw_b), math.sin(yaw_b)
        po = PoseStamped()
        po.header.stamp = stamp
        po.header.frame_id = self.odom
        po.pose.position.x = tr.x + c * x - sn * y
        po.pose.position.y = tr.y + sn * x + c * y
        po.pose.position.z = tr.z + float(s.floor_z)
        yo = yaw_b + s.yaw
        po.pose.orientation.z, po.pose.orientation.w = math.sin(yo / 2), math.cos(yo / 2)
        self.pub_pose_odom.publish(po)

    def set_status(self, text, force=False):
        state = text.split(':')[0]
        now = time.monotonic()
        changed = self.last_status is None or state != self.last_status.split(':')[0]
        if force or changed or now - self.last_status_pub > 1.0:
            self.pub_status.publish(String(data=text))
            self.last_status_pub = now
            if changed or force:
                self.get_logger().info(f'/stairs/status -> {text}')
        self.last_status = text

    def report(self):
        if self.frames:
            self.get_logger().info(f'{self.frames} frames, stairs in {self.hits} '
                                   f'(last: {S.detect.last_why})')
        self.frames = self.hits = 0


def main():
    rclpy.init()
    node = StairWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
