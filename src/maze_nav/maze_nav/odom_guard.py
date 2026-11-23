"""odom_guard: Fast-LIO odometry as odom -> base_link, with slips bridged by leg odometry.

Replaces lite3_launcher's odom_relay.py in maze_nav.launch.py (node name still odom_relay).
The slip guard is off there unless odom_guard:=true (leg_topic '' = a plain relay). Like it:

  T_odom_base = M * T_fastlio * M^-1      M = base_link -> livox_frame (static, from TF)

publishes /odom (nav_msgs/Odometry, odom -> base_link, twist from consecutive poses), the
odom -> base_link TF and a static odom -> lidar_init.

What it adds: near walls Fast-LIO's pose can slip (2026-11-19 12:25: ~0.5 m in 0.5 s while the
robot turned in a corner; cmd_vel_bridge stopped the robot, the planner put it "in lethal
space"). Every Fast-LIO step is checked against the robot's leg odometry (/leg_odom2 body
velocity; its stamps are on the robot board's clock, so the velocity received during the step
is used, not the poses):

  fl  = Fast-LIO's horizontal translation in this step (base_link frame of the previous pose)
  leg = mean leg body velocity received during the step x the step time
  |fl - leg| > max(slip_min, slip_speed x dt)  ->  SLIP: this step's horizontal translation
        comes from the legs; rotation (IMU-based) and height stay Fast-LIO's

So /odom stays continuous through a slip and the jump never reaches slam_toolbox, Nav2's
costmaps or cmd_vel_bridge. The correction is kept afterwards (a constant offset between
/odom and Fast-LIO), so nothing jumps back either.

Real divergence (2026-11-19 04:04: hundreds of metres in seconds) is not hidden: when the
slip lasts longer than max_slip_time s, or one slip moves /odom more than max_offset m off
Fast-LIO (per slip, not the total of all slips), the guard gives
up for the rest of the run and passes Fast-LIO through unchanged. /odom then jumps, and
cmd_vel_bridge's odometry watchdog stops the robot as before.

No leg odometry (none yet, or older than leg_timeout; the default, odom_guard:=false): slips
are not bridged, but a step the robot can't have made (over jump_dist 0.3 m, or > 1 m/s, or
> 2.5 rad/s) is a Fast-LIO JUMP (user 2026-11-23: "when odometry diverges too much, stop the
robot and sync again if possible"; 10:31 run: four jumps of 0.35-0.78 m at a junction, then
SENSOR FAULT until restart):

  HOLD    /odom (and the TF) stays at the last good pose, status "HOLD: ..."; cmd_vel_bridge
          stops the robot and drops Nav2 commands while it reads HOLD.
  RESYNC  once Fast-LIO has been steady (moved < steady_dist m, turned < steady_yaw rad) for
          steady_time s with the robot standing still, /odom goes on from the held pose with
          Fast-LIO's motion from then on (a constant offset, like a bridged slip): status
          "OK (... resynced ...)", cmd_vel_bridge clears the costmaps and drives again.
  FAULT   not steady within resync_timeout s, or more than resync_limit holds in
          resync_window s: /odom stays held, status "FAULT: ..." and cmd_vel_bridge latches
          (SENSOR FAULT, until restart) as before.

Start-up: Fast-LIO's first messages can arrive before the static base_link -> livox_frame
(the robot bridge's latched /tf_static takes 0.2-2 s to reach a new node). The mount is
looked up on every message until known (lite3_launcher's relay polled every 0.5 s and
warned "Dropping odometry, base_link -> livox_frame not known yet" on each launch). Nothing
is lost by skipping those messages: Fast-LIO's pose is absolute. Only a mount still missing
after mount_wait s is a fault (robot bridge down / wrong ROS_DOMAIN_ID) and is reported.

  in : /fastlio/odometry (input_topic), /leg_odom2 (leg_topic), TF base_link -> livox_frame
  out: /odom, TF odom -> base_link, static TF odom -> lidar_init,
       /odom_guard/status (std_msgs/String, latched): OK / SLIP ... / PASS-THROUGH ...

    ros2 run maze_nav odom_guard
"""
import collections
import math
import time

import numpy as np

# ------------------------------------------------------------------ maths (no ROS)


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_to_quat(r):
    """(x, y, z, w) of a rotation matrix."""
    t = np.trace(r)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
    q = np.empty(4)
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return tuple(q)


def rot_to_rotvec(r):
    angle = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0)))
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if angle < 1e-9:
        return 0.5 * v
    return angle / (2.0 * math.sin(angle)) * v


def make_tf(r, t):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


class GuardCore:
    """The guard's logic on 4x4 base_link poses (no ROS). See the module doc."""

    def __init__(self, slip_speed=0.4, slip_min=0.05, max_slip_time=3.0, max_offset=5.0,
                 leg_timeout=0.5, jump_dist=0.3, jump_min=0.08, jump_speed=1.0,
                 jump_yaw=0.3, jump_yaw_rate=2.5, steady_time=3.0, steady_dist=0.03,
                 steady_yaw=0.03, resync_timeout=30.0, resync_limit=3, resync_window=120.0):
        self.slip_speed, self.slip_min = slip_speed, slip_min
        self.max_slip_time, self.max_offset = max_slip_time, max_offset
        self.leg_timeout = leg_timeout
        # HOLD + RESYNC (no leg odometry for the step; see the module doc). jump_dist 0 = off.
        self.jump_dist, self.jump_min, self.jump_speed = jump_dist, jump_min, jump_speed
        self.jump_yaw, self.jump_yaw_rate = jump_yaw, jump_yaw_rate
        self.steady_time, self.steady_dist, self.steady_yaw = steady_time, steady_dist, steady_yaw
        self.resync_timeout = resync_timeout
        self.resync_limit, self.resync_window = resync_limit, resync_window
        self.hold = None                              # dict while /odom is held
        self.holds = collections.deque()              # stamps holds started (resync_window)
        self.fault = None                             # reason, once resyncing gave up
        self.resynced = None                          # text of the last resync
        self.legs = collections.deque(maxlen=400)     # (receive time, vx, vy) body frame
        self.fl_prev = None                           # (stamp, T) Fast-LIO base pose
        self.out = None                               # T published last
        self.slip_since = None                        # stamp the current slip started
        self.slip_dist = 0.0                          # m bridged in the current slip
        self.slip_start_offset = np.zeros(2)          # offset vector when it started
        self.passthrough = None                       # reason, once given up
        self.events = []                              # (kind, text) for the node to log

    def add_leg(self, now, vx, vy):
        self.legs.append((now, vx, vy))

    def leg_velocity(self, now, dt):
        """Mean leg body velocity (vx, vy) received over the last dt s, or None."""
        if not self.legs or now - self.legs[-1][0] > self.leg_timeout:
            return None
        window = [s for s in self.legs if s[0] >= now - max(dt, 0.15)]
        if not window:
            window = [self.legs[-1]]
        return (sum(s[1] for s in window) / len(window), sum(s[2] for s in window) / len(window))

    @property
    def offset_vec(self):
        """Published pose minus Fast-LIO's, horizontal (m)."""
        if self.out is None or self.fl_prev is None:
            return np.zeros(2)
        return self.out[:2, 3] - self.fl_prev[1][:2, 3]

    @property
    def offset(self):
        """Horizontal distance between the published pose and Fast-LIO's (m)."""
        return float(np.linalg.norm(self.offset_vec))

    @property
    def slip_offset(self):
        """How far /odom and Fast-LIO have moved apart in the current slip (m)."""
        return float(np.linalg.norm(self.offset_vec - self.slip_start_offset))

    def update(self, stamp, t_fl, now):
        """Fast-LIO base pose t_fl (4x4) at stamp -> pose to publish (4x4)."""
        if self.fl_prev is None or self.passthrough:
            self.fl_prev = (stamp, t_fl)
            self.out = t_fl.copy()
            return self.out
        stamp_prev, t_prev = self.fl_prev
        dt = stamp - stamp_prev
        d = np.linalg.inv(t_prev) @ t_fl            # this step, in the previous base frame
        if dt <= 0:
            dt = 0.1
        if self.hold is not None or self.fault:
            self.holding(stamp, t_fl)               # /odom stays where it was
            self.fl_prev = (stamp, t_fl)
            return self.out
        leg = self.leg_velocity(now, dt)
        if leg is None and self.jump_dist > 0:
            why = self.jump_reason(d, dt)
            if why:
                self.start_hold(stamp, t_fl, why)
                self.fl_prev = (stamp, t_fl)
                return self.out
        used = d
        if leg is not None:
            fl_xy = d[:2, 3]
            leg_xy = np.array(leg) * dt
            err = float(np.linalg.norm(fl_xy - leg_xy))
            if err > max(self.slip_min, self.slip_speed * dt):
                used = d.copy()
                used[:2, 3] = leg_xy
                if self.slip_since is None:
                    self.slip_since, self.slip_dist = stamp, 0.0
                    self.slip_start_offset = self.offset_vec.copy()
                    self.events.append(('warn', f'Fast-LIO slip: moved {np.linalg.norm(fl_xy):.2f} m'
                                        f' in {dt:.2f} s, legs {np.linalg.norm(leg_xy):.2f} m; '
                                        'using the legs'))
                self.slip_dist += err
            elif self.slip_since is not None:
                self.events.append(('info', f'Fast-LIO agrees with the legs again after '
                                    f'{stamp - self.slip_since:.1f} s; {self.slip_dist:.2f} m '
                                    f'bridged, /odom now {self.offset:.2f} m off Fast-LIO'))
                self.slip_since = None
        self.out = self.out @ used
        self.fl_prev = (stamp, t_fl)
        # One slip lasting too long or growing too large = diverged. Per slip, not the total:
        # many small bridged slips may add up to a large (harmless) constant offset.
        if self.slip_since is not None and (stamp - self.slip_since > self.max_slip_time or
                                            self.slip_offset > self.max_offset):
            self.passthrough = (f'Fast-LIO disagreed with the legs for '
                                f'{stamp - self.slip_since:.1f} s ({self.slip_offset:.2f} m): '
                                'diverged, not a slip')
            self.events.append(('error', f'{self.passthrough}; passing Fast-LIO through '
                                'unguarded from now on (cmd_vel_bridge will see the jump)'))
            self.out = t_fl.copy()
        return self.out

    def jump_reason(self, d, dt):
        """Why the Fast-LIO step d (dt s) is a jump the robot can't have made, or None."""
        step = float(np.linalg.norm(d[:2, 3]))
        yaw = abs(math.atan2(d[1, 0], d[0, 0]))
        if step > self.jump_dist:
            return f'Fast-LIO jumped {step:.2f} m in {dt:.2f} s'
        if step > self.jump_min and step / dt > self.jump_speed:
            return f'Fast-LIO moved {step:.2f} m in {dt:.2f} s ({step / dt:.1f} m/s)'
        if yaw > self.jump_yaw and yaw / dt > self.jump_yaw_rate:
            return f'Fast-LIO turned {math.degrees(yaw):.0f} deg in {dt:.2f} s'
        return None

    def start_hold(self, stamp, t_fl, why):
        self.holds.append(stamp)
        while self.holds and stamp - self.holds[0] > self.resync_window:
            self.holds.popleft()
        self.hold = {'since': stamp, 'why': why, 'ref': t_fl, 'ref_at': stamp}
        if len(self.holds) > self.resync_limit:
            self.give_up(f'{why}: {len(self.holds)} jumps in {self.resync_window:.0f} s, '
                         'Fast-LIO keeps diverging')
            return
        self.events.append(('warn', f'{why}: HOLD - /odom held at the last good pose until '
                            f'Fast-LIO is steady for {self.steady_time:.0f} s (the robot is '
                            'stopped meanwhile)'))

    def holding(self, stamp, t_fl):
        """While held: Fast-LIO steady (robot stopped) for steady_time s = resynced."""
        h = self.hold
        if self.fault or h is None:
            return
        m = np.linalg.inv(h['ref']) @ t_fl
        if (np.linalg.norm(m[:3, 3]) > self.steady_dist or
                abs(math.atan2(m[1, 0], m[0, 0])) > self.steady_yaw):
            h['ref'], h['ref_at'] = t_fl, stamp      # still moving: steady window anew
        elif stamp - h['ref_at'] >= self.steady_time:
            self.hold = None
            self.resynced = (f'resynced {stamp - h["since"]:.1f} s after "{h["why"]}"; /odom '
                             f'goes on from the held pose')
            self.events.append(('info', f'Fast-LIO steady for {self.steady_time:.0f} s: '
                                f'RESYNC - {self.resynced} (/odom now {self.offset:.2f} m off '
                                'Fast-LIO)'))
            return
        if stamp - h['since'] > self.resync_timeout:
            self.give_up(f'{h["why"]}, and Fast-LIO not steady within '
                         f'{self.resync_timeout:.0f} s')

    def give_up(self, why):
        self.fault = why
        self.hold = None
        self.events.append(('error', f'{why}: FAULT - /odom stays held, the robot stays '
                            'stopped until restart'))

    @property
    def status(self):
        if self.fault:
            return f'FAULT: {self.fault}'
        if self.hold is not None:
            return f'HOLD: {self.hold["why"]}; waiting for Fast-LIO to be steady'
        if self.passthrough:
            return f'PASS-THROUGH: {self.passthrough}'
        if self.slip_since is not None:
            return f'SLIP: using leg odometry ({self.slip_dist:.2f} m so far)'
        extra = f'; {self.resynced}' if self.resynced else ''
        if not self.legs:
            return f'OK (no leg odometry yet: slips unguarded, jumps held{extra})'
        return f'OK (offset {self.offset:.2f} m{extra})'


# ------------------------------------------------------------------ node


def main():
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import (Buffer, StaticTransformBroadcaster, TransformBroadcaster,
                         TransformException, TransformListener)

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def fill(tr, m):
        tr.translation.x, tr.translation.y, tr.translation.z = (float(v) for v in m[:3, 3])
        tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w = rot_to_quat(m[:3, :3])

    class OdomGuard(Node):
        def __init__(self):
            super().__init__('odom_guard')
            p = self.declare_parameter
            p('input_topic', '/fastlio/odometry')
            p('output_topic', '/odom')
            p('leg_topic', '/leg_odom2')              # '' = no guard (plain odom_relay)
            p('odom_frame', 'odom')
            p('base_frame', 'base_link')
            p('lidar_frame', 'livox_frame')
            p('lidar_init_frame', 'lidar_init')
            p('publish_tf', True)
            p('slip_speed', 0.4)        # m/s Fast-LIO vs legs, and ...
            p('slip_min', 0.05)         # ... at least this many m in one step = slip
            # Diverged = one slip lasting max_slip_time s (Fast-LIO keeps disagreeing), or one
            # slip over max_offset m (a safety cap). A single big jump followed by agreement is
            # bridged: the legs say the robot did not move (bag replay: 1.9 m in 0.3 s, then
            # Fast-LIO steady again).
            p('max_slip_time', 3.0)
            p('max_offset', 5.0)
            p('leg_timeout', 0.5)       # s; older leg odometry = unguarded
            p('mount_wait', 5.0)        # s without the mount TF at start-up before it is an ERROR
            # HOLD / RESYNC (no leg odometry): a Fast-LIO step over jump_dist m, or over
            # jump_min m at more than jump_speed m/s, or over jump_yaw rad at more than
            # jump_yaw_rate rad/s holds /odom; steady (within steady_dist m / steady_yaw rad)
            # for steady_time s resyncs; not steady in resync_timeout s, or more than
            # resync_limit holds in resync_window s = FAULT. jump_dist 0 = off (old relay).
            p('jump_dist', 0.3)
            p('jump_min', 0.08)
            p('jump_speed', 1.0)
            p('jump_yaw', 0.3)
            p('jump_yaw_rate', 2.5)
            p('steady_time', 3.0)
            p('steady_dist', 0.03)
            p('steady_yaw', 0.03)
            p('resync_timeout', 30.0)
            p('resync_limit', 3)
            p('resync_window', 120.0)
            g = lambda n: self.get_parameter(n).value  # noqa: E731
            self.f = {n: g(n) for n in ('odom_frame', 'base_frame', 'lidar_frame',
                                        'lidar_init_frame', 'publish_tf')}
            self.core = GuardCore(
                g('slip_speed'), g('slip_min'), g('max_slip_time'), g('max_offset'),
                g('leg_timeout'), **{n: g(n) for n in (
                    'jump_dist', 'jump_min', 'jump_speed', 'jump_yaw', 'jump_yaw_rate',
                    'steady_time', 'steady_dist', 'steady_yaw', 'resync_timeout',
                    'resync_limit', 'resync_window')})
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.tf_pub = TransformBroadcaster(self)
            self.static_pub = StaticTransformBroadcaster(self)
            self.mount = self.mount_inv = None
            self.mount_wait = g('mount_wait')
            self.started = time.monotonic()
            self.skipped = 0                           # Fast-LIO messages before the mount
            self.prev = None                           # (stamp s, T published)
            self.status = None
            self.odom_pub = self.create_publisher(Odometry, g('output_topic'), 20)
            self.status_pub = self.create_publisher(String, '/odom_guard/status', latched)
            self.create_subscription(Odometry, g('input_topic'), self.on_fastlio, 20)
            self.leg_topic = g('leg_topic')
            if self.leg_topic:
                self.create_subscription(Odometry, self.leg_topic, self.on_leg,
                                         qos_profile_sensor_data)
            self.mount_timer = self.create_timer(0.5, self.lookup_mount)
            self.create_timer(5.0, self.check_legs)
            self.get_logger().info(
                f'{g("input_topic")} -> {g("output_topic")} + TF {self.f["odom_frame"]} -> '
                f'{self.f["base_frame"]}; slips bridged with '
                + (self.leg_topic if self.leg_topic else 'nothing (leg_topic empty)'))

        def lookup_mount(self):
            try:
                tf = self.tf_buffer.lookup_transform(self.f['base_frame'], self.f['lidar_frame'],
                                                     Time())
            except TransformException as e:
                waited = time.monotonic() - self.started
                if waited > self.mount_wait:
                    self.get_logger().error(
                        f'no TF {self.f["base_frame"]} -> {self.f["lidar_frame"]} after '
                        f'{waited:.0f} s: NO /odom. Is the robot bridge (lite3_domain_bridge) '
                        f'up and ROS_DOMAIN_ID right? ({e})', throttle_duration_sec=5.0)
                return
            q, t = tf.transform.rotation, tf.transform.translation
            self.mount = make_tf(quat_to_rot(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])
            self.mount_inv = np.linalg.inv(self.mount)
            self.mount_timer.cancel()
            init = TransformStamped()
            init.header.stamp = self.get_clock().now().to_msg()
            init.header.frame_id = self.f['odom_frame']
            init.child_frame_id = self.f['lidar_init_frame']
            fill(init.transform, self.mount)
            self.static_pub.sendTransform(init)
            self.get_logger().info(
                f'got {self.f["base_frame"]} -> {self.f["lidar_frame"]} after '
                f'{time.monotonic() - self.started:.1f} s'
                + (f' ({self.skipped} early Fast-LIO messages skipped)' if self.skipped else ''))

        def on_leg(self, msg):
            v = msg.twist.twist.linear
            self.core.add_leg(time.monotonic(), v.x, v.y)

        def check_legs(self):
            if self.leg_topic and (not self.core.legs or
                                   time.monotonic() - self.core.legs[-1][0] > 2.0):
                self.get_logger().warn(f'no {self.leg_topic}: Fast-LIO slips are NOT bridged',
                                       throttle_duration_sec=30.0)

        def on_fastlio(self, msg):
            if self.mount is None:
                self.lookup_mount()     # not only on the 0.5 s timer: the TF may be in by now
                if self.mount is None:
                    self.skipped += 1   # start-up; lookup_mount reports it if it lasts
                    return
            pp, q = msg.pose.pose.position, msg.pose.pose.orientation
            t_fl = self.mount @ make_tf(quat_to_rot(q.x, q.y, q.z, q.w),
                                        [pp.x, pp.y, pp.z]) @ self.mount_inv
            stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
            t = self.core.update(stamp, t_fl, time.monotonic())
            for kind, text in self.core.events:
                getattr(self.get_logger(), kind)(text)
            self.core.events.clear()
            status = self.core.status
            if status.split(' ', 1)[0] != (self.status or '').split(' ', 1)[0]:
                self.status_pub.publish(String(data=status))
            self.status = status

            out = Odometry()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.f['odom_frame']
            out.child_frame_id = self.f['base_frame']
            o = out.pose.pose
            o.position.x, o.position.y, o.position.z = (float(v) for v in t[:3, 3])
            o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w = \
                rot_to_quat(t[:3, :3])
            j = np.kron(np.eye(2), self.mount[:3, :3])
            cov = np.array(msg.pose.covariance).reshape(6, 6)
            out.pose.covariance = [float(v) for v in (j @ cov @ j.T).flatten()]
            if self.prev is not None and stamp - self.prev[0] > 0:
                d = np.linalg.inv(self.prev[1]) @ t
                dt = stamp - self.prev[0]
                tw = out.twist.twist
                tw.linear.x, tw.linear.y, tw.linear.z = (float(v) for v in d[:3, 3] / dt)
                tw.angular.x, tw.angular.y, tw.angular.z = \
                    (float(v) for v in rot_to_rotvec(d[:3, :3]) / dt)
            self.prev = (stamp, t)
            self.odom_pub.publish(out)
            if self.f['publish_tf']:
                tf = TransformStamped()
                tf.header = out.header
                tf.child_frame_id = self.f['base_frame']
                fill(tf.transform, t)
                self.tf_pub.sendTransform(tf)

    rclpy.init()
    node = OdomGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
