"""Walk the Lite3 straight ahead and stop before it reaches an obstacle.

    ros2 run nav_straight straight_driver        (normally started by nav_straight.launch.py)

What it does
  1. Waits until /livox/points and /odom are both arriving.
  2. Refuses to start if another node already publishes /cmd_vel (one publisher rule).
  3. Counts down start_delay seconds, then records the start pose.
  4. Walks forward along the start heading (heading + cross-track hold from /odom).
     It checks a box ahead of the robot, as wide as the body plus a margin, in the raw
     lidar cloud. It slows down inside slow_distance and stops at stop_distance.
  5. Stops for good when blocked, after max_distance, or if a sensor goes quiet:
     one zero Twist, then manual mode (handheld controller drives again), then it idles.

It never switches the robot to auto mode. The operator does that (lite3_auto).

Speeds are clamped to the team limits in robot_info.md §6.2 whatever the parameters say.
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from .obstacle import (DEFAULT_LIDAR_QUAT, DEFAULT_LIDAR_XYZ, cloud_to_xyz,
                       forward_clearance, quat_to_matrix, speed_for_clearance,
                       transform, wrap_angle, yaw_from_quat)

try:
    from transfer_interfaces.msg import MotionSimpleCMD
except ImportError:  # built off-robot without ~/lite3_ws
    MotionSimpleCMD = None

# robot_info.md §6.2: RELIABLE + TRANSIENT_LOCAL, depth 1, short lifespan.
CMD_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     lifespan=Duration(seconds=0.3))
HARD_MAX_V = 0.25   # m/s, team limit
HARD_MAX_W = 0.5    # rad/s, team limit
MANUAL_MODE = 0x21010C02


def clamp(v, lim):
    return max(-lim, min(lim, v))


class StraightDriver(Node):
    def __init__(self):
        super().__init__('straight_driver')
        p = self.declare_parameter
        # Motion
        p('max_speed', 0.10)            # m/s cruising speed (first trials: 0.10)
        p('min_speed', 0.05)            # m/s just before the stop distance
        p('max_distance', 5.0)          # m, stop after this far even if clear; <=0 = no limit
        p('start_delay', 3.0)           # s countdown once sensors are ready
        p('k_heading', 1.0)             # rad/s per rad of heading error
        p('k_cross_track', 0.8)         # rad of heading correction per m off the line
        p('max_turn_rate', 0.3)         # rad/s
        # Obstacle check (base_link frame)
        p('stop_distance', 0.50)        # m from the front edge to the obstacle: stop here
        p('slow_distance', 1.20)        # m: start slowing down here
        p('lookahead', 3.0)             # m beyond the front edge that is checked
        p('front_x', 0.32)              # m, front edge incl. feet (lite3-maze footprint 0.64 m)
        p('half_width', 0.30)           # m, half body width 0.23 (with feet) + 0.07 margin
        p('ground_z', -0.35)            # m, floor height in base_link when standing
        p('min_obstacle_height', 0.10)  # m above the floor that counts as an obstacle
        p('max_obstacle_height', 0.75)  # m above the floor; higher things are passed under
        p('min_points', 5)              # points needed in the box to count as an obstacle
        # Topics / safety
        p('cloud_topic', '/livox/points')
        p('odom_topic', '/odom')
        p('sensor_timeout', 0.5)        # s without cloud or odom -> stop
        p('drive', True)                # false = dry run, commands go to ~/cmd_vel_preview
        p('manual_on_finish', True)     # send manual mode after the final stop

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in (
            'max_speed', 'min_speed', 'max_distance', 'start_delay', 'k_heading',
            'k_cross_track', 'max_turn_rate', 'stop_distance', 'slow_distance', 'lookahead',
            'front_x', 'half_width', 'ground_z', 'min_obstacle_height',
            'max_obstacle_height', 'min_points', 'sensor_timeout', 'drive',
            'manual_on_finish')}
        c = self.cfg
        c['max_speed'] = min(abs(c['max_speed']), HARD_MAX_V)
        c['min_speed'] = min(abs(c['min_speed']), c['max_speed'])
        c['max_turn_rate'] = min(abs(c['max_turn_rate']), HARD_MAX_W)

        cmd_topic = '/cmd_vel' if c['drive'] else '~/cmd_vel_preview'
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, CMD_QOS)
        self.mode_pub = None
        if c['drive'] and c['manual_on_finish']:
            if MotionSimpleCMD is None:
                self.get_logger().warn('transfer_interfaces not found: cannot send manual '
                                       'mode at the end. Run lite3_manual yourself.')
            else:
                self.mode_pub = self.create_publisher(MotionSimpleCMD, '/simple_cmd', CMD_QOS)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar_tf = None            # (R, t) cloud frame -> base_link
        self.first_cloud_time = None

        self.create_subscription(PointCloud2, g('cloud_topic'), self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)

        self.clearance = math.inf
        self.cloud_time = None
        self.pose = None                # (x, y, yaw)
        self.odom_time = None
        self.start_pose = None
        self.obstacle_along = math.inf
        self.state = 'waiting'
        self.countdown_end = None
        self.moving = False             # last command sent was non-zero
        self.last_log = self.now()

        self.create_timer(0.05, self.tick)   # 20 Hz
        self.get_logger().info(
            f'straight_driver: {"DRIVING" if c["drive"] else "DRY RUN"} -> {cmd_topic}; '
            f'speed {c["max_speed"]:.2f} m/s, stop at {c["stop_distance"]:.2f} m, '
            f'max distance {c["max_distance"]:.1f} m')

    # ---------------------------------------------------------------- inputs
    def now(self):
        return self.get_clock().now()

    def age(self, t):
        return math.inf if t is None else (self.now() - t).nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_from_quat(q.x, q.y, q.z, q.w))
        self.odom_time = self.now()

    def lookup_lidar_tf(self, frame):
        if frame == 'base_link':
            return quat_to_matrix(0, 0, 0, 1), (0.0, 0.0, 0.0)
        try:
            tf = self.tf_buffer.lookup_transform('base_link', frame, rclpy.time.Time())
            r, t = tf.transform.rotation, tf.transform.translation
            self.get_logger().info(f'using /tf_static base_link -> {frame}')
            return quat_to_matrix(r.x, r.y, r.z, r.w), (t.x, t.y, t.z)
        except Exception:  # noqa: BLE001
            if self.age(self.first_cloud_time) < 3.0:
                return None  # give /tf_static a moment
        if frame == 'livox_frame':
            self.get_logger().warn('no TF base_link -> livox_frame; using the mount from '
                                   'robot_info.md (15 deg pitch)')
            return quat_to_matrix(*DEFAULT_LIDAR_QUAT), DEFAULT_LIDAR_XYZ
        self.get_logger().error(f'no TF base_link -> {frame}; cannot check for obstacles',
                                throttle_duration_sec=5.0)
        return None

    def on_cloud(self, msg):
        if self.first_cloud_time is None:
            self.first_cloud_time = self.now()
        if self.lidar_tf is None:
            self.lidar_tf = self.lookup_lidar_tf(msg.header.frame_id)
            if self.lidar_tf is None:
                return
        try:
            pts = transform(cloud_to_xyz(msg), *self.lidar_tf)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'bad cloud: {e}', throttle_duration_sec=5.0)
            return
        c = self.cfg
        self.clearance = forward_clearance(
            pts, c['front_x'], c['half_width'],
            c['ground_z'] + c['min_obstacle_height'], c['ground_z'] + c['max_obstacle_height'],
            c['lookahead'], c['min_points'])
        self.cloud_time = self.now()

    # ----------------------------------------------------------------- logic
    def send(self, vx, wz):
        t = Twist()
        t.linear.x = clamp(vx, self.cfg['max_speed'])
        t.angular.z = clamp(wz, self.cfg['max_turn_rate'])
        self.cmd_pub.publish(t)
        self.moving = vx != 0.0 or wz != 0.0

    def stop_once(self):
        """One zero Twist (robot_info.md §6.2 rule 3), then go quiet."""
        self.send(0.0, 0.0)

    def finish(self, reason):
        self.stop_once()
        travelled = self.travelled()
        self.get_logger().info(f'STOPPED: {reason}. Travelled {travelled:.2f} m, '
                               f'clearance {self.clearance:.2f} m.')
        if self.mode_pub is not None:
            m = MotionSimpleCMD()
            m.cmd_code, m.size, m.type = MANUAL_MODE, 0, 0
            self.mode_pub.publish(m)
            self.get_logger().info('sent manual mode: handheld controller has control')
        self.state = 'done'

    def travelled(self):
        if self.start_pose is None or self.pose is None:
            return 0.0
        return math.hypot(self.pose[0] - self.start_pose[0], self.pose[1] - self.start_pose[1])

    def sensors_fresh(self):
        t = self.cfg['sensor_timeout']
        return self.age(self.cloud_time) < t and self.age(self.odom_time) < t

    def tick(self):
        c = self.cfg
        if self.state == 'done':
            return

        if self.state == 'waiting':
            if not self.sensors_fresh():
                self.get_logger().info(
                    f'waiting for sensors: cloud age {self.age(self.cloud_time):.1f} s, '
                    f'odom age {self.age(self.odom_time):.1f} s',
                    throttle_duration_sec=3.0)
                return
            if c['drive'] and self.count_publishers('/cmd_vel') > 1:
                self.get_logger().error(
                    'another node publishes /cmd_vel (twist_mux, teleop, Nav2?). Not '
                    'starting. Stop it and relaunch.')
                self.state = 'done'
                return
            self.state = 'countdown'
            self.countdown_end = self.now() + Duration(seconds=float(c['start_delay']))
            self.get_logger().info(f'sensors ready; starting in {c["start_delay"]:.0f} s '
                                   '(robot must be standing and in auto mode)')
            return

        if self.state == 'countdown':
            if not self.sensors_fresh():
                self.state = 'waiting'
                return
            if self.now() < self.countdown_end:
                return
            if self.clearance <= c['stop_distance']:
                self.finish(f'blocked at start (obstacle {self.clearance:.2f} m ahead)')
                return
            self.start_pose = self.pose
            self.obstacle_along = self.clearance   # metres along the line from the start
            self.state = 'driving'
            self.get_logger().info(
                f'GO: start x={self.pose[0]:.2f} y={self.pose[1]:.2f} '
                f'yaw={math.degrees(self.pose[2]):.1f} deg')
            return

        # driving
        if not self.sensors_fresh():
            self.finish(f'sensor timeout (cloud {self.age(self.cloud_time):.2f} s, '
                        f'odom {self.age(self.odom_time):.2f} s)')
            return
        x0, y0, yaw0 = self.start_pose
        x, y, yaw = self.pose
        along = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)
        # Remember obstacles along the line: low objects drop below the lidar's lowest
        # ray (about 22 deg down) once they are within about 1 m, so the live clearance
        # alone would jump back to "free" right before impact.
        if math.isfinite(self.clearance):
            self.obstacle_along = min(self.obstacle_along, along + self.clearance)
        clearance = min(self.clearance, self.obstacle_along - along)
        if clearance <= c['stop_distance']:
            self.finish(f'obstacle {clearance:.2f} m ahead')
            return
        dist = self.travelled()
        if c['max_distance'] > 0 and dist >= c['max_distance']:
            self.finish(f'reached max_distance {c["max_distance"]:.1f} m')
            return

        vx = speed_for_clearance(clearance, c['stop_distance'], c['slow_distance'],
                                 c['max_speed'], c['min_speed'])
        # Signed distance left of the start line; steer back towards it.
        cross = -(x - x0) * math.sin(yaw0) + (y - y0) * math.cos(yaw0)
        target_yaw = yaw0 - math.atan(c['k_cross_track'] * cross)
        wz = c['k_heading'] * wrap_angle(target_yaw - yaw)
        self.send(vx, wz)

        if self.age(self.last_log) >= 1.0:
            self.last_log = self.now()
            self.get_logger().info(
                f'driving: {dist:.2f} m done, clearance {clearance:.2f} m '
                f'(live {self.clearance:.2f}), '
                f'v {vx:.2f} m/s, off-line {cross:+.2f} m')


def main():
    # Handle Ctrl+C ourselves so the final zero Twist can still be published.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = StraightDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.state == 'driving' or node.moving:
            node.finish('shutdown (Ctrl+C)')
            # Let the reliable publishers deliver before the node goes away.
            end = node.now() + Duration(seconds=0.3)
            while node.now() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
