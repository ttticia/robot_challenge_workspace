"""Forward Nav2's velocity commands to the Lite3 motion bridge on /cmd_vel.

Nav2 can't talk to the robot directly, so this glue stays:
- QoS: Nav2 publishes VOLATILE; lite3_domain_bridge_4 may subscribe TRANSIENT_LOCAL, which
  never matches a VOLATILE publisher. This node republishes RELIABLE + TRANSIENT_LOCAL with
  a short lifespan (robot_info.md §6.2).
- Limits: clamps to 0.25 m/s and 0.5 rad/s whatever Nav2 asks for. Never drives backwards.
- Idle: when Nav2 goes quiet, one zero Twist, then nothing (teleop works again).
- Sensor watchdog: if /livox/points or /odom stops for sensor_timeout while driving, it
  sends one zero Twist and blocks all further commands until restart. Nav2 Humble keeps
  driving on a stale costmap, so this is the only place that catches it.
- drive:=false: publishes to ~/cmd_vel_preview instead of /cmd_vel (dry run).
- Gait: at startup, once the robot bridge subscribes /simple_cmd, sends Move mode
  (0x21010D06, needed before gait requests) and then the `gait` (default `rug` = RUG gait,
  General gear, 0x21010401; robot_info.md §6.3), each once. `none` or drive:=false: nothing
  is sent. The operator sends lite3_auto AFTER this ("gait sent" in the log): the documented
  order is gait request, then auto mode. There is no gait feedback; watch the robot.
"""
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2

HARD_MAX_V = 0.25   # m/s, team limit
HARD_MAX_W = 0.5    # rad/s, team limit

MOVE_MODE = 0x21010D06
GAITS = {'rug': 0x21010401, 'none': None}   # robot_info.md §6.3; H-Step left out on purpose


def clamp(value, limit):
    return max(-limit, min(limit, value))


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')
        p = self.declare_parameter
        p('input_topic', 'nav_cmd_vel')
        p('drive', True)
        p('max_linear', HARD_MAX_V)
        p('max_angular', HARD_MAX_W)
        p('idle_timeout', 0.5)          # s without Nav2 commands -> one stop
        p('sensor_timeout', 0.5)        # s without cloud or odom while driving -> stop, latch
        p('cloud_topic', '/livox/points')
        p('odom_topic', '/odom')
        p('gait', 'rug')                # sent once at startup; 'none' = leave the gait alone
        p('gait_wait', 20.0)            # s to wait for a /simple_cmd subscriber before giving up
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.max_v = min(abs(get('max_linear')), HARD_MAX_V)
        self.max_w = min(abs(get('max_angular')), HARD_MAX_W)
        self.idle_timeout = get('idle_timeout')
        self.sensor_timeout = get('sensor_timeout')

        out_topic = '/cmd_vel' if get('drive') else '~/cmd_vel_preview'
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         lifespan=Duration(seconds=0.3))
        self.pub = self.create_publisher(Twist, out_topic, qos)
        self.create_subscription(Twist, get('input_topic'), self.on_cmd, 10)
        self.create_subscription(PointCloud2, get('cloud_topic'),
                                 lambda _: self.mark('cloud'), qos_profile_sensor_data)
        self.create_subscription(Odometry, get('odom_topic'), lambda _: self.mark('odom'), 10)
        self.create_timer(0.1, self.watchdog)
        self.gait_steps = []
        self.setup_gait(get('gait'), get('drive'), get('gait_wait'))

        self.seen = {'cloud': None, 'odom': None}
        self.last_cmd = 0.0
        self.active = False      # last forwarded command was non-zero / not yet stopped
        self.blocked = None      # reason, once the sensor watchdog has tripped
        self.get_logger().info(
            f"{get('input_topic')} -> {out_topic} "
            f"({'DRIVING' if get('drive') else 'DRY RUN'}), "
            f'limits {self.max_v} m/s, {self.max_w} rad/s')

    def setup_gait(self, gait, drive, wait):
        if gait not in GAITS:
            self.get_logger().error(f'gait {gait!r} unknown (one of {sorted(GAITS)}); not sent')
            return
        if GAITS[gait] is None:
            return
        if not drive:
            self.get_logger().info(f'DRY RUN: gait {gait} not sent')
            return
        try:
            from transfer_interfaces.msg import MotionSimpleCMD
        except ImportError:
            self.get_logger().warn('transfer_interfaces not found (source ~/lite3_ws); '
                                   f'gait {gait} not sent')
            return
        self.simple_cmd = MotionSimpleCMD
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         lifespan=Duration(seconds=1.0))
        self.simple_pub = self.create_publisher(MotionSimpleCMD, '/simple_cmd', qos)
        self.gait_name = gait
        self.gait_steps = [('Move mode', MOVE_MODE), (f'gait {gait}', GAITS[gait])]
        self.gait_deadline = time.monotonic() + wait
        self.gait_next = 0.0
        self.gait_timer = self.create_timer(0.2, self.gait_tick)
        self.get_logger().info(f'gait {gait}: waiting for the robot bridge on /simple_cmd')

    def gait_tick(self):
        now = time.monotonic()
        if not self.gait_steps:
            return
        if self.simple_pub.get_subscription_count() == 0:
            # lite3_domain_bridge_4 subscribes only after it sees a publisher (robot_info §6.3)
            if now > self.gait_deadline:
                self.get_logger().warn(f'no subscriber on /simple_cmd; gait {self.gait_name} '
                                       'NOT sent (set it on the handheld controller)')
                self.gait_steps = []
                self.gait_timer.cancel()
            return
        if now < self.gait_next:
            return
        name, code = self.gait_steps.pop(0)
        self.simple_pub.publish(self.simple_cmd(cmd_code=code, size=0, type=0))
        self.get_logger().info(f'sent {name} (0x{code:08X}) on /simple_cmd')
        self.gait_next = now + 1.0                  # one command at a time
        if not self.gait_steps:
            self.get_logger().info(f'gait sent ({self.gait_name}); the operator can now send '
                                   'lite3_auto')
            self.gait_timer.cancel()

    def mark(self, name):
        self.seen[name] = time.monotonic()

    def sensor_age(self, name):
        t = self.seen[name]
        return float('inf') if t is None else time.monotonic() - t

    def on_cmd(self, msg):
        if self.blocked:
            return
        stale = [n for n in self.seen if self.sensor_age(n) > self.sensor_timeout]
        if stale and (msg.linear.x != 0.0 or msg.angular.z != 0.0):
            self.block(f'no {"/".join(stale)} for > {self.sensor_timeout} s')
            return
        out = Twist()
        out.linear.x = max(0.0, clamp(msg.linear.x, self.max_v))
        out.angular.z = clamp(msg.angular.z, self.max_w)
        self.last_cmd, self.active = time.monotonic(), True
        self.pub.publish(out)

    def watchdog(self):
        if not self.active:
            return
        stale = [n for n in self.seen if self.sensor_age(n) > self.sensor_timeout]
        if stale and not self.blocked:
            self.block(f'no {"/".join(stale)} for > {self.sensor_timeout} s')
        elif time.monotonic() - self.last_cmd > self.idle_timeout:
            self.stop('Nav2 stopped sending commands')

    def block(self, reason):
        self.blocked = reason
        self.active = True
        self.stop(f'SENSOR TIMEOUT ({reason}); ignoring Nav2 until restart')

    def stop(self, reason):
        if self.active:
            self.pub.publish(Twist())
            self.active = False
            self.get_logger().info(f'{reason}: sent stop')


def main():
    stop = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = CmdVelBridge()
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.active = True
        node.stop('shutting down')
        end = time.monotonic() + 0.3  # let the reliable stop go out
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
