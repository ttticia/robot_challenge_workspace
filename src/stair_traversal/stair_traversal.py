"""Run a single Lite3 stair crossing and reverse back to its starting landing."""

import argparse
from dataclasses import dataclass
import json
import math
import signal
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from transfer_interfaces.msg import MotionSimpleCMD

from stair_geometry import depth_points, estimate_stairs, transform_points


MOVE_MODE = 0x21010D06
RUG_GENERAL = 0x21010401
AUTO_MODE = 0x21010C03
MANUAL_MODE = 0x21010C02
MAX_ODOM_AGE = 0.4
SETTLE = 1.0
SPEED = 0.10
ENDPOINT_MARGIN = 0.08
STAIR_NOTES = {'metal stairs', 'metal staircase', 'metal steps', 'metal steps across view'}


def sees_forward_stairs(payload):
    """Use the forward path note in a scene-understanding result."""
    try:
        scene = json.loads(payload)
    except ValueError:
        return False
    paths = scene.get('paths') if isinstance(scene, dict) else None
    if not isinstance(paths, list):
        return False
    return any(
        isinstance(path, dict)
        and path.get('direction') == 'forward'
        and isinstance(path.get('notes'), str)
        and path['notes'].lower().strip(' .!?') in STAIR_NOTES
        for path in paths
    )


@dataclass(frozen=True)
class Pose:
    stamp: float
    x: float
    y: float
    z: float
    yaw: float


class StairTraversal(Node):
    def __init__(self, args):
        super().__init__('stair_traversal')
        self.args = args
        self.phase = 'waiting'
        self.pose = None
        self.start = None
        self.deadline = 0.0
        self.progress_at = 0.0
        self.best = 0.0
        self.vel = self.mode = None
        self.camera = None
        self.measurements = []
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.on_camera,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self.on_depth, qos_profile_sensor_data)
        if args.execute:
            qos = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                lifespan=Duration(seconds=0.25),
            )
            self.vel = self.create_publisher(Twist, args.cmd_topic, qos)
            self.mode = self.create_publisher(MotionSimpleCMD, '/simple_cmd', qos)
        self.create_subscription(String, args.scene_topic, self.on_scene, 1)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_service(Trigger, '~/abort', self.on_abort)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Waiting for forward metal stairs on {args.scene_topic}')

    def now_ros(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(map(math.isfinite, values)) or abs(math.hypot(*values[3:]) - 1) > 0.05:
            self.pose = None
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        previous = self.pose
        if previous and stamp <= previous.stamp:
            return
        self.pose = Pose(stamp, p.x, p.y, p.z, yaw)
        if self.active and previous and math.dist(
            (p.x, p.y, p.z), (previous.x, previous.y, previous.z)
        ) > 0.18:
            self.abort('Odometry jumped')

    @property
    def active(self):
        return self.phase not in ('waiting', 'measuring', 'done', 'aborted')

    def pose_fresh(self):
        return self.pose is not None and -0.05 <= self.now_ros() - self.pose.stamp <= MAX_ODOM_AGE

    def ready(self):
        return (
            self.vel.get_subscription_count() > 0
            and self.mode.get_subscription_count() > 0
            and self.count_publishers(self.args.cmd_topic) == 1
            and self.count_publishers('/simple_cmd') == 1
        )

    def send_mode(self, code):
        self.mode.publish(MotionSimpleCMD(cmd_code=code, size=0, type=0))

    def speed(self, speed=0.0):
        msg = Twist()
        msg.linear.x = speed
        self.vel.publish(msg)

    def on_scene(self, msg):
        if self.phase != 'waiting' or not sees_forward_stairs(msg.data):
            return
        self.phase = 'measuring'
        self.measurements.clear()
        self.depth_frames = 0
        self.depth_status = f'no frames received on {self.args.depth_topic}'
        self.deadline = time.monotonic() + 4.0
        self.get_logger().info('Forward metal stairs detected. Measuring visible landings.')

    def on_camera(self, msg):
        self.camera = msg

    def on_depth(self, msg):
        if self.phase != 'measuring':
            return
        self.depth_frames += 1
        if self.camera is None:
            self.depth_status = f'no CameraInfo received on {self.args.camera_info_topic}'
            return
        if not self.pose_fresh():
            self.depth_status = f'missing or stale odometry on {self.args.odom_topic}'
            return
        stamp = Time.from_msg(msg.header.stamp)
        age = self.now_ros() - stamp.nanoseconds * 1e-9
        if not -0.05 <= age <= MAX_ODOM_AGE:
            self.depth_status = f'depth timestamp outside the freshness window (age {age:+.2f} s)'
            return
        try:
            camera_tf = self.tf.lookup_transform(self.args.odom_frame, msg.header.frame_id, stamp)
            base_tf = self.tf.lookup_transform(self.args.odom_frame, self.args.base_frame, stamp)
            points = depth_points(msg, self.camera)
        except TransformException as exc:
            self.depth_status = f'TF unavailable at depth timestamp ({exc})'
            return
        except (ValueError, BufferError) as exc:
            self.depth_status = f'invalid depth image buffer ({exc})'
            return
        if points is None:
            self.depth_status = (f'depth encoding or calibration mismatch '
                                 f'({msg.encoding}, frame {msg.header.frame_id})')
            return
        if not len(points):
            self.depth_status = 'depth image contains no usable range samples'
            return
        base = base_tf.transform
        p, q = base.translation, base.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        local = transform_points(points, camera_tf.transform) - [p.x, p.y, p.z]
        c, s = math.cos(yaw), math.sin(yaw)
        forward = c * local[:, 0] + s * local[:, 1]
        lateral = -s * local[:, 0] + c * local[:, 1]
        measurement = estimate_stairs(np.column_stack((forward, lateral, local[:, 2])))
        if measurement is None:
            self.depth_status = 'depth received, but stair treads and both landings were not verified'
            self.measurements.clear()
            return
        pose = Pose(stamp.nanoseconds * 1e-9, p.x, p.y, p.z, yaw)
        if self.measurements:
            first, old = self.measurements[0]
            if pose.stamp <= self.measurements[-1][0].stamp:
                return
            if (math.dist((pose.x, pose.y, pose.z), (first.x, first.y, first.z)) > 0.04
                    or abs(math.remainder(pose.yaw - first.yaw, math.tau)) > math.radians(4)
                    or max(abs(a - b) for a, b in zip(measurement, old)) > 0.06):
                self.measurements.clear()
        self.measurements.append((pose, measurement))
        if len(self.measurements) < 3:
            self.depth_status = f'only {len(self.measurements)} of 3 consistent landing estimates'
            return
        self.run, self.rise = np.median([m for _, m in self.measurements], axis=0)
        self.get_logger().info(f'Measured stair landing {self.run:.2f} m ahead, {self.rise:.2f} m up')
        if not self.args.execute:
            self.phase = 'waiting'
            return
        if not self.ready():
            self.phase = 'aborted'
            self.get_logger().error('Need exclusive, connected robot commands')
            return
        if (math.dist((pose.x, pose.y, pose.z), (self.pose.x, self.pose.y, self.pose.z)) > 0.04
                or abs(math.remainder(pose.yaw - self.pose.yaw, math.tau)) > math.radians(4)):
            self.depth_status = 'depth and latest odometry disagree about starting pose'
            self.measurements.clear()
            return
        self.start = pose
        self.speed()
        self.send_mode(MANUAL_MODE)
        self.phase = 'move_mode'
        self.deadline = time.monotonic() + SETTLE

    def abort(self, reason):
        if self.phase in ('done', 'aborted'):
            return
        if self.active:
            self.speed()
        self.phase = 'aborted'
        self.get_logger().error(f'{reason}. Stair routine stopped.')

    def on_abort(self, _, response):
        self.abort('Operator abort')
        response.success = True
        response.message = 'Stair motion stopped'
        return response

    def tick(self):
        if self.phase == 'measuring' and time.monotonic() >= self.deadline:
            self.get_logger().warning(
                f'Could not verify both landings; holding. Received {self.depth_frames} '
                f'depth frames during measurement. Last check: {self.depth_status}'
            )
            self.phase = 'waiting'
        if not self.active:
            return
        if not self.pose_fresh() or not self.ready():
            self.abort('Odometry stale or robot command ownership lost')
            return

        dx, dy = self.pose.x - self.start.x, self.pose.y - self.start.y
        c, s = math.cos(self.start.yaw), math.sin(self.start.yaw)
        along, lateral = c * dx + s * dy, -s * dx + c * dy
        height = self.pose.z - self.start.z
        heading = math.remainder(self.pose.yaw - self.start.yaw, math.tau)
        run, rise = self.run, self.rise
        if (abs(lateral) > 0.06 or abs(heading) > math.radians(5)
                or not -0.12 <= height <= rise + 0.15
                or not -ENDPOINT_MARGIN <= along <= run + ENDPOINT_MARGIN):
            self.abort('Pose left the calibrated stair corridor')
            return

        now = time.monotonic()
        if self.phase in ('up', 'down'):
            self.walk(now, along, height)
        elif now >= self.deadline:
            if self.phase == 'move_mode':
                self.send_mode(MOVE_MODE)
                self.phase = 'gait'
            elif self.phase == 'gait':
                self.send_mode(RUG_GENERAL)
                self.phase = 'auto'
            elif self.phase == 'auto':
                self.send_mode(AUTO_MODE)
                self.phase = 'start_up'
            elif self.phase in ('start_up', 'top'):
                upward = self.phase == 'start_up'
                if upward:
                    at_landing = abs(along) <= ENDPOINT_MARGIN and abs(height) <= ENDPOINT_MARGIN
                else:
                    at_landing = along >= run and abs(height - rise) <= ENDPOINT_MARGIN
                if not at_landing:
                    self.abort('Pose changed while settling at the landing')
                    return
                self.phase = 'up' if upward else 'down'
                self.best = along if upward else run - along
                self.progress_at = now
                self.deadline = now + run / SPEED * 2 + 4.0
                return
            elif self.phase == 'bottom':
                if along > ENDPOINT_MARGIN or abs(height) > ENDPOINT_MARGIN:
                    self.abort('Bottom landing could not be confirmed')
                    return
                self.send_mode(MANUAL_MODE)
                self.phase = 'done'
                self.get_logger().info('Bottom landing confirmed. Returned control to manual mode.')
            self.deadline = now + SETTLE

    def walk(self, now, along, height):
        upward = self.phase == 'up'
        reached = along >= self.run if upward else along <= 0.0
        if reached:
            self.speed()
            target_height = self.rise if upward else 0.0
            if abs(height - target_height) > ENDPOINT_MARGIN:
                self.abort('Expected landing height was not reached')
                return
            self.phase = 'top' if upward else 'bottom'
            self.deadline = now + SETTLE
            return
        progress = along if upward else self.run - along
        if progress >= self.best + 0.025:
            self.best, self.progress_at = progress, now
        if now >= self.deadline or now - self.progress_at >= 2.0:
            self.abort('Stair motion made insufficient progress')
            return
        self.speed(SPEED if upward else -SPEED)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--reverse-descent-verified', action='store_true')
    parser.add_argument('--aligned-start', action='store_true',
                        help='Confirm the robot is stopped and square at the calibrated start')
    parser.add_argument('--depth-topic', default='/camera/depth/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/depth/camera_info')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--scene-topic', default='/scene_understanding/result')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.execute and not (args.aligned_start and args.reverse_descent_verified):
        parser.error('Execution requires --aligned-start and verified reverse descent')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = StairTraversal(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.abort('Shutting down')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
