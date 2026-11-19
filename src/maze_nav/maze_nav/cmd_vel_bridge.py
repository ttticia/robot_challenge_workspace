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
- Auto stand (auto_stand:=true): before the gait, if wall_scan's /maze/body_height says the
  robot is LYING DOWN, sends stand (0x21010202) once and waits until it stands (stand_timeout).
  0x21010202 is a stand/sit TOGGLE: sent to a standing robot it sits down, so it is never sent
  when the robot already stands or its posture is unknown (no height within stand_wait, e.g.
  floor_z given). Runs at launch, before the explorer waits for /maze/start.
- AI mode at the stairs (ai_on_stairs:=true): when stair_detector reports AT_STAIRS lined up
  (not "NOT LINED UP"), this node stops forwarding velocity, sends one zero Twist if it was
  driving, then on /simple_cmd: manual mode (0x21010C02; AI mode can't be entered from auto
  mode), AI state (0x21010528), AI basic gait (0x2101052A), `ai_settle` s apart. While in AI
  mode nothing goes to /cmd_vel: no Nav2 commands, no idle stop, not even the zero Twist on
  shutdown. The AI-mode walk is up to the operator (handheld controller).
  /maze/start (the operator's, which also resumes the explorer) leaves AI mode: AI exit
  (0x2101052B), manual mode, then Move mode + `gait` again (not valid in AI state, so lost),
  then velocity is forwarded again; the robot follows it once the operator sends lite3_auto.
  The same stairs don't switch again until /stairs/status has left AT_STAIRS.
  drive:=false: nothing is sent (logged).
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
from std_msgs.msg import Empty, Float32, String

from maze_nav.core import posture

HARD_MAX_V = 0.25   # m/s, team limit
HARD_MAX_W = 0.5    # rad/s, team limit

MOVE_MODE = 0x21010D06
STAND_TOGGLE = 0x21010202   # stand <-> sit; only ever sent to a robot measured lying down
GAITS = {'rug': 0x21010401, 'none': None}   # robot_info.md §6.3; H-Step left out on purpose
MANUAL_MODE = 0x21010C02
AI_STATE = 0x21010528
AI_BASIC_GAIT = 0x2101052A
AI_EXIT = 0x2101052B


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
        p('auto_stand', True)           # lying at launch -> send stand once (see module doc)
        p('stand_wait', 30.0)           # s from start to get a posture before giving up (not sent)
        p('stand_timeout', 15.0)        # s after stand was sent to be standing, else no gait
        p('ai_on_stairs', True)         # AT_STAIRS lined up -> AI mode, no more velocity
        p('stairs_status_topic', '/stairs/status')
        p('ai_settle', 1.0)             # s still before the first mode command, and between them
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
        # raw=True: the watchdog only needs "a cloud arrived", so don't decode each cloud
        # into a Python message (a small saving; the sim showed no measurable change).
        self.create_subscription(PointCloud2, get('cloud_topic'),
                                 lambda _: self.mark('cloud'), qos_profile_sensor_data,
                                 raw=True)
        self.create_subscription(Odometry, get('odom_topic'), lambda _: self.mark('odom'), 10)
        self.create_timer(0.1, self.watchdog)
        self.gait_steps = []
        self.stand = 'done'      # 'check' -> ('rising') -> 'done'
        self.setup_gait(get('gait'), get('drive'), get('gait_wait'), get('auto_stand'),
                        get('stand_wait'), get('stand_timeout'))

        self.seen = {'cloud': None, 'odom': None}
        self.last_cmd = 0.0
        self.active = False      # last forwarded command was non-zero / not yet stopped
        self.blocked = None      # reason, once the sensor watchdog has tripped
        self.drive = get('drive')
        self.ai_mode = False     # True from AI requested until AI exit sent: no velocity at all
        self.ai_armed = True     # False after an AI switch until /stairs/status leaves AT_STAIRS
        self.ai_exit_wanted = False
        self.ai_steps, self.ai_kind = [], None
        self.ai_settle = float(get('ai_settle'))
        self.gait_req = get('gait')
        self.gait_code = GAITS.get(self.gait_req)
        if get('ai_on_stairs') and get('stairs_status_topic'):
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, get('stairs_status_topic'), self.on_stairs,
                                     latched)
            self.create_subscription(Empty, '/maze/start', self.on_start, 10)
            self.create_timer(0.2, self.ai_tick)
        self.get_logger().info(
            f"{get('input_topic')} -> {out_topic} "
            f"({'DRIVING' if get('drive') else 'DRY RUN'}), "
            f'limits {self.max_v} m/s, {self.max_w} rad/s')

    def setup_gait(self, gait, drive, wait, auto_stand, stand_wait, stand_timeout):
        if gait not in GAITS:
            self.get_logger().error(f'gait {gait!r} unknown (one of {sorted(GAITS)}); not sent')
            gait = 'none'
        what = ' + '.join((['stand'] if auto_stand else [])
                          + ([f'gait {gait}'] if GAITS[gait] is not None else []))
        if not what:
            return
        if not drive:
            self.get_logger().info(f'DRY RUN: {what} not sent')
            return
        if not self.simple_publisher():
            self.get_logger().warn('transfer_interfaces not found (source ~/lite3_ws); '
                                   f'{what} not sent')
            return
        if auto_stand:
            self.stand = 'check'
            self.height = None
            self.stand_check_deadline = time.monotonic() + stand_wait
            self.stand_timeout = stand_timeout
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.height_sub = self.create_subscription(
                Float32, '/maze/body_height', lambda m: setattr(self, 'height', m.data),
                latched)
        self.gait_name = gait
        if GAITS[gait] is not None:
            self.gait_steps = [('Move mode', MOVE_MODE), (f'gait {gait}', GAITS[gait])]
        self.what = what
        self.gait_deadline = time.monotonic() + wait
        self.gait_next = 0.0
        self.gait_timer = self.create_timer(0.2, self.gait_tick)
        self.get_logger().info(f'{what}: waiting for the robot bridge on /simple_cmd')

    def simple_publisher(self):
        """Create the /simple_cmd publisher once; False if transfer_interfaces is missing."""
        if getattr(self, 'simple_pub', None) is not None:
            return True
        try:
            from transfer_interfaces.msg import MotionSimpleCMD
        except ImportError:
            return False
        self.simple_cmd = MotionSimpleCMD
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         lifespan=Duration(seconds=1.0))
        self.simple_pub = self.create_publisher(MotionSimpleCMD, '/simple_cmd', qos)
        return True

    # ------------------------------------------------------------------ AI mode at the stairs
    def on_stairs(self, msg):
        state, _, why = msg.data.partition(':')
        if state.strip() != 'AT_STAIRS':
            self.ai_armed = True                # stairs left behind: the next ones may switch
            return
        if self.ai_mode or not self.ai_armed or 'NOT LINED UP' in why:
            return
        self.enter_ai(f'lined up at the stairs ({why.strip()})')

    def on_start(self, _):
        """/maze/start while in AI mode: leave it (after entering has finished)."""
        if self.ai_mode and not self.ai_exit_wanted:
            self.ai_exit_wanted = True
            self.get_logger().warn('/maze/start: leaving AI mode'
                                   + (' once it is entered' if self.ai_steps else ''))
            if not self.ai_steps:
                self.exit_ai()

    def enter_ai(self, reason):
        """Stop forwarding velocity, then manual -> AI state -> AI basic gait."""
        self.stop('switching to AI mode')       # one zero Twist, only if it was driving
        self.ai_mode, self.ai_armed, self.ai_exit_wanted = True, False, False
        if not self.drive:
            self.get_logger().warn(f'{reason}: DRY RUN, AI mode NOT sent; velocity '
                                   'forwarding stopped until /maze/start')
            return
        if not self.simple_publisher():
            self.get_logger().error(f'{reason}: transfer_interfaces not found, AI mode NOT '
                                    'sent (lite3_ai by hand); velocity forwarding stopped')
            return
        if getattr(self, 'gait_steps', None):
            self.finish_gait()                  # a gait still pending at launch: drop it
        self.get_logger().warn(f'{reason}: velocity forwarding STOPPED; switching to AI mode '
                               '(manual -> AI state -> AI basic gait)')
        self.run_ai_steps([('manual mode', MANUAL_MODE), ('AI state', AI_STATE),
                           ('AI basic gait', AI_BASIC_GAIT)], 'enter')

    def exit_ai(self):
        """AI exit -> manual -> Move mode -> gait, then forward velocity again (the robot
        follows it only after the operator's lite3_auto)."""
        if not self.drive:
            self.ai_mode = False
            self.get_logger().warn('DRY RUN: AI exit NOT sent; forwarding velocity again')
            return
        if not self.simple_publisher():
            self.get_logger().error('transfer_interfaces not found: AI exit NOT sent '
                                    '(lite3_ai_off by hand, then restart maze_nav); '
                                    'velocity forwarding stays off')
            return
        steps = [('AI exit', AI_EXIT), ('manual mode', MANUAL_MODE)]
        if self.gait_code is not None:
            steps += [('Move mode', MOVE_MODE), (f'gait {self.gait_req}', self.gait_code)]
        self.run_ai_steps(steps, 'exit')

    def run_ai_steps(self, steps, kind):
        self.ai_steps, self.ai_kind = steps, kind
        self.ai_next = time.monotonic() + self.ai_settle
        self.ai_deadline = time.monotonic() + 10.0

    def ai_tick(self):
        if not self.ai_steps:
            return
        now = time.monotonic()
        if self.simple_pub.get_subscription_count() == 0:
            if now > self.ai_deadline:
                what = 'AI mode' if self.ai_kind == 'enter' else 'AI exit'
                self.get_logger().error(f'no subscriber on /simple_cmd: {what} NOT sent '
                                        '(lite3_ai / lite3_ai_off by hand); velocity '
                                        'forwarding stays off')
                self.ai_steps = []
            return
        if now < self.ai_next:
            return
        name, code = self.ai_steps.pop(0)
        self.send_simple(name, code)
        self.ai_next = now + self.ai_settle
        if self.ai_steps:
            return
        if self.ai_kind == 'enter':
            self.get_logger().warn('AI mode sent (no feedback: watch the robot / controller). '
                                   'No velocity is sent until /maze/start')
            if self.ai_exit_wanted:
                self.exit_ai()
        else:
            self.ai_exit_wanted = False
            self.ai_mode = False
            self.get_logger().warn('AI mode left (no feedback: watch the robot). Forwarding '
                                   'velocity again; the operator sends lite3_auto to drive')

    def send_simple(self, name, code):
        self.simple_pub.publish(self.simple_cmd(cmd_code=code, size=0, type=0))
        self.get_logger().info(f'sent {name} (0x{code:08X}) on /simple_cmd')

    def stand_tick(self, now):
        """Auto stand; True once it's done (whatever the outcome) and the gait may follow."""
        h = self.height
        state = posture(h)
        if self.stand == 'check':
            if state == 'standing':
                self.get_logger().info(f'robot already standing (base_link {h:.2f} m above '
                                       'the floor): stand not sent')
                self.stand = 'done'
            elif state == 'lying':
                self.get_logger().warn(f'robot LYING DOWN (base_link {h:.2f} m above the '
                                       'floor): standing it up')
                self.send_simple('stand', STAND_TOGGLE)
                self.stand = 'rising'
                self.stand_deadline = now + self.stand_timeout
            elif now > self.stand_check_deadline:
                why = ('no /maze/body_height from wall_scan' if h is None else
                       f'base_link {h:.2f} m above the floor, neither lying nor standing')
                self.get_logger().warn(f'posture unknown ({why}): stand NOT sent (stand it '
                                       'with the handheld controller)')
                self.stand = 'done'
            return self.stand == 'done'
        # rising
        if state == 'standing':
            self.get_logger().info(f'robot standing (base_link {h:.2f} m above the floor)')
            self.stand = 'done'
            self.gait_next = now + 1.0                  # let it settle before Move mode
            return True
        if now > self.stand_deadline:
            self.get_logger().error(
                f'stand sent but the robot is not standing after {self.stand_timeout:.0f} s '
                f'(base_link {"?" if h is None else f"{h:.2f}"} m above the floor): gait NOT '
                'sent. Check the robot; stand it with the handheld controller')
            self.stand = 'done'
            self.gait_steps = []
            return True
        return False

    def gait_tick(self):
        now = time.monotonic()
        if self.simple_pub.get_subscription_count() == 0:
            # lite3_domain_bridge_4 subscribes only after it sees a publisher (robot_info §6.3)
            if now > self.gait_deadline:
                self.get_logger().warn(f'no subscriber on /simple_cmd; {self.what} NOT sent '
                                       '(use the handheld controller)')
                self.finish_gait()
            return
        if self.stand != 'done' and not self.stand_tick(now):
            return
        if not self.gait_steps:
            self.finish_gait()
            return
        if now < self.gait_next:
            return
        name, code = self.gait_steps.pop(0)
        self.send_simple(name, code)
        self.gait_next = now + 1.0                  # one command at a time
        if not self.gait_steps:
            self.get_logger().info(f'gait sent ({self.gait_name}); the operator can now send '
                                   'lite3_auto')
            self.finish_gait()

    def finish_gait(self):
        self.gait_steps = []
        self.stand = 'done'
        if getattr(self, 'gait_timer', None) is not None:
            self.gait_timer.cancel()
        if getattr(self, 'height_sub', None) is not None:
            self.destroy_subscription(self.height_sub)
            self.height_sub = None

    def mark(self, name):
        self.seen[name] = time.monotonic()

    def sensor_age(self, name):
        t = self.seen[name]
        return float('inf') if t is None else time.monotonic() - t

    def on_cmd(self, msg):
        if self.ai_mode:
            if msg.linear.x != 0.0 or msg.angular.z != 0.0:
                self.get_logger().warn('AI mode: velocity command dropped',
                                       throttle_duration_sec=5.0)
            return
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
        if not self.active or self.ai_mode:
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
        if self.active and not self.ai_mode:     # in AI mode not even a zero Twist
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
