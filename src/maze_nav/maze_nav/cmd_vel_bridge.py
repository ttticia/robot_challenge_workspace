"""Forward Nav2's velocity commands to the Lite3 motion bridge on /cmd_vel.

Nav2 can't talk to the robot directly, so this glue stays:
- QoS: Nav2 publishes VOLATILE; lite3_domain_bridge_4 may subscribe TRANSIENT_LOCAL, which
  never matches a VOLATILE publisher. This node republishes RELIABLE + TRANSIENT_LOCAL with
  a short lifespan (robot_info.md §6.2).
- Limits: clamps to 0.25 m/s and 0.5 rad/s whatever Nav2 asks for. Never drives backwards.
- Minimum turn on the spot (`min_spin_rate`, 0.15 rad/s; 0 = off): a turn with no forward
  speed below it is raised to it. The Lite3 doesn't turn at all below ~0.1 rad/s, and RPP's
  rotate-to-heading only ever asks for measured yaw rate + max_angular_accel * 0.1 s: with
  the robot still that stays at 0.05 rad/s and nothing moves (robot 2026-11-20 07:18, bag
  s_maze1: 0.04-0.06 rad/s for 8 s, yaw frozen).
- Idle: when Nav2 goes quiet, one zero Twist, then nothing (teleop works again).
- Sensor watchdog: if /livox/points or /odom stops for sensor_timeout while driving, it
  sends one zero Twist and PAUSES: Nav2 commands are dropped until both have come steadily
  again for sensor_recover s, then they are forwarded again by themselves. Only an outage
  longer than sensor_timeout + sensor_latch (one sensor silent the whole time) latches until
  restart. Nav2 Humble keeps driving on a stale costmap, so this is the only place that
  catches it. Why not latch at once: through the robot bridge a single cloud / odom is
  sometimes ~1 s late (2026-11-19 12:47: one 0.94 s spike latched the bridge mid-spin and
  ended the run, although odom was back 0.5 s later). Late data is not a wrong pose, so the
  maps stay valid.
- Odometry watchdog: if /odom jumps more than odom_max_jump (0.3 m) between two messages, or
  implies more than odom_max_speed (1 m/s), Fast-LIO jumped or diverged: stop and PAUSE as
  above, and the AI walk ends. It drives again once /odom has been consistent (no new jump)
  for odom_recover s and cloud + odom are fresh, and then clears both costmaps (walls marked
  from the pose before the jump would sit in the wrong place). The user's call 2026-11-19:
  recover instead of latching. A real divergence (2026-11-19 04:04: hundreds of metres in
  seconds) keeps jumping, so more than odom_jump_limit jumps within odom_jump_window s still
  latch until restart (odom_jump_limit 0 = latch at the first jump, the old behaviour).
- drive:=false: publishes to ~/cmd_vel_preview instead of /cmd_vel (dry run).
- Gait: at startup, once the robot bridge subscribes /simple_cmd, sends Move mode
  (0x21010D06, needed before gait requests) and then the `gait` (default `rug` = RUG gait General gear,
  0x21010401, again since 2026-11-21; `hstep` = H-Step, the high-stepping RUG gait,
  0x21010407, the default 2026-11-20/21; robot_info.md §6.3), each once. The same gait is sent again
  after Pose-mode looks, lying down and AI mode. `none` or drive:=false: nothing
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
  mode Nav2 commands are dropped.
  AI walk (ai_walk_distance > 0, default 1.2 m; the user's call 2026-11-19, overriding the
  earlier "no velocity in AI mode" rule): `ai_walk_delay` s after the AI basic gait, this node
  itself publishes straight-ahead /cmd_vel at `ai_walk_speed` (0.1 m/s, 10 Hz) until /odom
  shows `ai_walk_distance` travelled in the plane, then ONE zero Twist and nothing more. It
  also stops (zero Twist) when there is less than 0.1 m of progress in `ai_walk_timeout` s
  (robot_info.md §6.3: AI state may ignore /cmd_vel; untested), when /odom or the cloud go
  stale, on /maze/start, and on shutdown. Outside the walk, nothing goes to /cmd_vel in AI
  mode, not even an idle stop.
  /maze/start (the operator's, which also resumes the explorer) leaves AI mode: AI exit
  (0x2101052B), manual mode, then Move mode + `gait` again (not valid in AI state, so lost),
  then velocity is forwarded again; the robot follows it once the operator sends lite3_auto.
  The same stairs don't switch again until /stairs/status has left AT_STAIRS.
  Descending stairs (AT_STAIRS "descending stairs: ...", stair_detector): the walk is
  `ai_walk_distance_down` (2.0 m: edge, 2-3 treads, the body off the last step) instead.
- Terrain speed cap (`terrain_topic`, /maze/terrain from stair_detector, presentation mazes
  2026-11-21): forward speed <= `terrain_rough_speed` while the strip ahead is rough, <=
  `terrain_obstacle_speed` for obstacle / drop (slats, blocks, a plate edge). Older than
  `terrain_timeout` s or unknown: no cap. Gait switching on terrain is NOT done (to_do.md).
  drive:=false: nothing is sent (logged).
- Pose-mode looks (`pose_yaw_topic`, /maze/pose_yaw, std_msgs/Float32: body yaw in rad,
  + = left, relative to where the feet stand; NaN = done). The explorer uses these instead
  of Nav2 spins for the start scan and the dead-end side walls (user's call 2026-11-19): the
  feet stay put and only the body (lidar, camera) yaws. The first fresh target: one zero
  Twist, Nav2 velocity dropped from now on, then on /simple_cmd [manual mode (`pose_manual`)]
  -> Pose mode (0x21010D05), `pose_settle` s apart; then the yaw axis command (0x21010135,
  value in `size`, + = rotate RIGHT on the robot, so the sign is flipped here; dead zone
  +-9553 of 32767) at 10 Hz, value = -yaw / `pose_max_yaw` * 32767 (linear, clamped to
  `pose_axis_limit` of full axis: full axis buzzed the motors, 2026-11-21 10:56; the
  angle at full axis is only roughly measured, 16 deg: each target's line in the log says
  how far /odom saw the body get). A target older
  than `pose_timeout` s -> axis 0 (the robot also returns to standing by itself after 1 s
  without an axis command). NaN, or no fresh target for `pose_linger` s -> axis 0 for
  0.5 s, then Move mode -> gait [-> navigation (auto) mode 0x21010C03 if `pose_manual`],
  then Nav2 velocity is forwarded again. Not in AI mode, not when latched, not with
  drive:=false or without transfer_interfaces / a /simple_cmd subscriber: /maze/pose_status
  (latched String: IDLE, ENTERING, POSING, LEAVING, UNAVAILABLE: why) says so and the
  explorer falls back to Nav2 spins. Untested on the robot: whether the group bridge
  passes `size` through for axis commands, and whether Pose mode works from auto mode
  (pose_manual:=false tries that).
  Pitch (user 2026-11-20, curtain inspection): `pose_pitch_topic` (/maze/pose_pitch,
  std_msgs/Float32, rad, + = nose DOWN) during a pose look also sends the pitch axis
  (0x21010130, + = lowers the head as on the robot, dead zone +-6553) at 10 Hz, value =
  pitch / `pose_max_pitch` * 32767 (NOT measured either). The yaw target alone starts and
  ends the look: send both. Pitch older than `pose_timeout` s, NaN, or leaving -> pitch 0.
- Lie down / stand up for the curtain inspection (`lie_topic`, /maze/lie, std_msgs/Bool,
  user 2026-11-20): True (repeated at >= 1 Hz) = lie down and stay down; False, or no True for
  `lie_linger` s = stand up. Height = /odom z - the floor under the feet (/maze/local_floor_z
  from stair_detector, while younger than `local_floor_timeout` s; else /maze/floor_z from
  wall_scan, the START floor, 5-10 cm off on a lower curtain cell): standing >= 0.25 m,
  lying <= 0.18 m (core.posture). Down: velocity forwarding STOPPED, one zero Twist,
  [manual mode (`lie_manual`)], then the stand/sit TOGGLE (0x21010202) only while the robot
  measures STANDING; LYING once it measures lying (`lie_down_timeout` s, else it stands up
  again). Up: the toggle only while it measures LYING, wait until standing for 1 s
  (`lie_up_timeout`), then Move mode -> gait [-> auto mode if `lie_manual`], `pose_settle` s
  apart, then velocity is forwarded again. /maze/lie_status (latched String): IDLE,
  LYING_DOWN, LYING, STANDING_UP, UNAVAILABLE: why, ERROR: why (not standing after the
  stand-up: forwarding stays off, stand it with the handheld controller). Untested on the
  robot: whether the toggle works from auto/manual mode mid-run.
"""
import collections
import json
import math
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Empty, Float32, String

from maze_nav.core import posture

HARD_MAX_V = 0.25   # m/s, team limit
HARD_MAX_W = 0.5    # rad/s, team limit

MOVE_MODE = 0x21010D06
STAND_TOGGLE = 0x21010202   # stand <-> sit; only ever sent to a robot measured lying down
# robot_info.md §6.3. hstep = H-Step (high-stepping RUG gait, gait state 13): the
# navigation gait 2026-11-20/21; it made the robot unsteady once (2026-11-12). rug = RUG gait
# General gear: the default again since 2026-11-21 (user).
GAITS = {'hstep': 0x21010407, 'rug': 0x21010401, 'none': None}
MANUAL_MODE = 0x21010C02
AI_STATE = 0x21010528
AI_BASIC_GAIT = 0x2101052A
AI_EXIT = 0x2101052B
POSE_MODE = 0x21010D05
AUTO_MODE = 0x21010C03
YAW_AXIS = 0x21010135      # pose mode: + = rotate right (robot convention)
PITCH_AXIS = 0x21010130    # pose mode: + = lowers the head (nose down)
AXIS_MAX = 32767
YAW_DEAD_ZONE = 9553


def terrain_cap(level, age, max_v, rough=0.15, obstacle=0.10, timeout=2.0):
    """Speed limit (m/s) for the terrain ahead: /maze/terrain level, its age (s)."""
    if level is None or age is None or age > timeout:
        return max_v
    if level == 'rough':
        return min(max_v, rough)
    if level in ('obstacle', 'drop'):
        return min(max_v, obstacle)
    return max_v


def clamp(value, limit):
    return max(-limit, min(limit, value))


def min_spin(v, w, min_w, v_eps=0.02, w_eps=0.01):
    """Turn on the spot (|v| < v_eps) slower than min_w -> min_w, same sign. Noise below
    w_eps and any turn while walking are left alone; min_w <= 0 = off."""
    if min_w <= 0.0 or abs(v) >= v_eps or abs(w) < w_eps or abs(w) >= min_w:
        return w
    return math.copysign(min_w, w)


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')
        p = self.declare_parameter
        p('input_topic', 'nav_cmd_vel')
        p('drive', True)
        p('max_linear', HARD_MAX_V)
        p('max_angular', HARD_MAX_W)
        p('min_spin_rate', 0.15)        # rad/s: slowest turn on the spot sent (0 = off)
        p('idle_timeout', 0.5)          # s without Nav2 commands -> one stop
        p('sensor_timeout', 0.5)        # s without cloud or odom while driving -> stop, pause
        p('sensor_recover', 1.0)        # s of steady cloud + odom after a pause -> drive again
        p('sensor_latch', 3.0)          # s more of one sensor silent -> latch; 0 = never latch
        p('odom_max_jump', 0.3)         # m between two /odom messages -> stop, pause
        p('odom_max_speed', 1.0)        # m/s implied by /odom -> stop, pause (robot: <= 0.25)
        p('odom_recover', 2.0)          # s of /odom without a new jump -> drive again
        p('odom_jump_limit', 3)         # jumps within odom_jump_window that recover; more latch
        p('odom_jump_window', 60.0)     # s
        p('cloud_topic', '/livox/points')
        p('odom_topic', '/odom')
        p('gait', 'rug')              # sent once at startup; 'none' = leave the gait alone
        p('gait_wait', 20.0)            # s to wait for a /simple_cmd subscriber before giving up
        p('auto_stand', True)           # lying at launch -> send stand once (see module doc)
        p('stand_wait', 30.0)           # s from start to get a posture before giving up (not sent)
        p('stand_timeout', 15.0)        # s after stand was sent to be standing, else no gait
        p('ai_on_stairs', True)         # AT_STAIRS lined up -> AI mode, no more velocity
        p('stairs_status_topic', '/stairs/status')
        p('ai_settle', 1.0)             # s still before the first mode command, and between them
        p('ai_walk_distance', 1.2)      # m forward in AI mode after the switch; 0 = don't walk
        p('ai_walk_speed', 0.1)         # m/s for that walk (clamped to max_linear)
        p('ai_walk_delay', 2.0)         # s after the AI basic gait before walking
        p('ai_walk_timeout', 8.0)       # s with < 0.1 m progress -> stop the walk
        # m for DESCENDING stairs (/stairs/status AT_STAIRS "descending stairs: ..."): from
        # 0.45 m before the top edge, down 2-3 treads (~0.3 m each) and the body (0.64 m) off
        # the last step. Presentation maze 2026-11-21; NOT measured on the robot.
        p('ai_walk_distance_down', 2.0)
        p('pose_yaw_topic', '/maze/pose_yaw')   # explorer's pose-mode looks; '' = off
        # rad of body yaw at full axis: 16 deg on /odom in the open start cell (11-20 run,
        # full axis, 2.5 s); 8 deg in a tight cell on 11-21 10:56 (motor buzzing: body
        # against a wall?). Each target's reached yaw is logged: refine from those lines.
        p('pose_max_yaw', math.radians(16))
        p('pose_manual', True)          # manual mode before Pose mode, auto mode after
        p('pose_settle', 0.5)           # s between the mode commands
        p('pose_timeout', 0.3)          # s: target older than this -> axis 0
        p('pose_linger', 1.5)           # s at axis 0 (no target) before leaving Pose mode
        # Fraction of the full yaw axis never exceeded: full axis holds the body at the end of
        # its range, and the motors buzzed doing that (run 2026-11-21 10:56). = the explorer's.
        p('pose_axis_limit', 0.8)
        p('pose_pitch_topic', '/maze/pose_pitch')   # curtain inspection: + = nose down
        p('pose_max_pitch', math.radians(20))   # rad of body pitch at full axis (NOT measured)
        p('lie_topic', '/maze/lie')     # curtain inspection: lie down (True) / stand up
        p('lie_manual', True)           # manual mode before the toggle, auto mode after standing
        p('lie_linger', 2.0)            # s without a True -> stand up
        p('lie_down_timeout', 8.0)      # s for the robot to measure lying
        p('lie_up_timeout', 12.0)       # s for the robot to measure standing again
        # Terrain speed cap (presentation mazes 2026-11-21: the only way out crosses slats,
        # a block field and a raised box). stair_detector's /maze/terrain (JSON 'level' of the
        # strip 0.25-1.2 m ahead), fresh within terrain_timeout s: rough -> terrain_rough_speed,
        # obstacle or drop -> terrain_obstacle_speed; even / unknown / stale -> no cap. '' = off.
        p('terrain_topic', '/maze/terrain')
        p('terrain_rough_speed', 0.15)  # m/s
        p('terrain_obstacle_speed', 0.10)
        p('terrain_timeout', 2.0)
        # Floor under the feet (stair_detector) for the lying / standing height, instead of
        # wall_scan's start floor: a curtain cell or room on the bare lab floor is 5-10 cm
        # below a raised start plate, so the robot standing there measured ~0.19 m, "not
        # standing", and the lie-down toggle was refused. Used while younger than
        # local_floor_timeout s (it isn't updated while lying). '' = start floor only.
        p('local_floor_topic', '/maze/local_floor_z')
        p('local_floor_timeout', 30.0)
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.max_v = min(abs(get('max_linear')), HARD_MAX_V)
        self.max_w = min(abs(get('max_angular')), HARD_MAX_W)
        self.min_spin = min(abs(get('min_spin_rate')), self.max_w)
        self.idle_timeout = get('idle_timeout')
        self.sensor_timeout = get('sensor_timeout')
        self.sensor_recover = float(get('sensor_recover'))
        self.sensor_latch = float(get('sensor_latch'))
        self.odom_max_jump = float(get('odom_max_jump'))
        self.odom_max_speed = float(get('odom_max_speed'))
        self.odom_hist = collections.deque()   # (stamp s, x, y) of recent /odom
        self.odom_recover = float(get('odom_recover'))
        self.odom_jump_limit = int(get('odom_jump_limit'))
        self.odom_jump_window = float(get('odom_jump_window'))
        self.jumps = collections.deque()       # monotonic s of recent /odom jumps
        self.clear_clients = [self.create_client(ClearEntireCostmap, name) for name in (
            '/local_costmap/clear_entirely_local_costmap',
            '/global_costmap/clear_entirely_global_costmap')]

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
        self.odom_xy = None
        self.create_subscription(Odometry, get('odom_topic'), self.on_odom, 10)
        self.create_timer(0.1, self.watchdog)
        self.gait_steps = []
        self.stand = 'done'      # 'check' -> ('rising') -> 'done'
        self.setup_gait(get('gait'), get('drive'), get('gait_wait'), get('auto_stand'),
                        get('stand_wait'), get('stand_timeout'))

        self.seen = {'cloud': None, 'odom': None}
        self.last_cmd = 0.0
        self.active = False      # last forwarded command was non-zero / not yet stopped
        self.blocked = None      # reason, once latched (odometry diverged / long outage)
        self.paused = None       # reason, while late sensors hold Nav2 off (clears itself)
        self.paused_at = 0.0     # monotonic s the pause began
        self.fresh_since = None  # monotonic s since when every sensor is fresh again
        self.drive = get('drive')
        self.ai_mode = False     # True from AI requested until AI exit sent: no velocity at all
        self.ai_armed = True     # False after an AI switch until /stairs/status leaves AT_STAIRS
        self.ai_exit_wanted = False
        self.ai_steps, self.ai_kind = [], None
        self.ai_settle = float(get('ai_settle'))
        self.gait_req = get('gait')
        self.gait_code = GAITS.get(self.gait_req)
        self.walk_dist = max(0.0, float(get('ai_walk_distance')))
        self.walk_dist_down = max(0.0, float(get('ai_walk_distance_down')))
        self.walk_dist_now = self.walk_dist     # the stairs being switched for: up or down
        self.walk_speed = min(abs(float(get('ai_walk_speed'))), self.max_v)
        self.walk_delay = float(get('ai_walk_delay'))
        self.walk_timeout = float(get('ai_walk_timeout'))
        self.walk = None         # dict while the AI walk is pending or driving
        self.create_timer(0.1, self.walk_tick)
        self.pose_max_yaw = max(1e-3, abs(float(get('pose_max_yaw'))))
        self.pose_manual = bool(get('pose_manual'))
        self.pose_settle = float(get('pose_settle'))
        self.pose_timeout = float(get('pose_timeout'))
        self.pose_linger = float(get('pose_linger'))
        self.pose_axis_max = int(AXIS_MAX * min(1.0, max(0.0, float(get('pose_axis_limit')))))
        self.pose_track = None            # [target rad, furthest body yaw towards it rad]
        self.pose_max_pitch = max(1e-3, abs(float(get('pose_max_pitch'))))
        self.pitch_target, self.pitch_at, self.pitch_used = 0.0, -1e9, False
        self.lie_manual = bool(get('lie_manual'))
        self.lie_linger = float(get('lie_linger'))
        self.lie_down_timeout = float(get('lie_down_timeout'))
        self.lie_up_timeout = float(get('lie_up_timeout'))
        self.lie_phase = 'idle'           # idle -> down -> lying -> up -> leave -> idle
        self.lie_want, self.lie_at = False, -1e9
        self.lie_steps, self.lie_next, self.lie_deadline, self.lie_stood = [], 0.0, 0.0, None
        self.lie_status = None
        self.odom_z, self.floor_z = None, None
        self.local_floor, self.local_floor_at = None, -1e9
        self.local_floor_timeout = float(get('local_floor_timeout'))
        if get('local_floor_topic'):
            self.create_subscription(Float32, get('local_floor_topic'), self.on_local_floor, 10)
        self.terrain_level, self.terrain_at = None, -1e9
        self.terrain_cfg = {n: float(get(n)) for n in ('terrain_rough_speed',
                                                        'terrain_obstacle_speed',
                                                        'terrain_timeout')}
        if get('terrain_topic'):
            self.create_subscription(String, get('terrain_topic'), self.on_terrain, 10)
        self.pose_phase = 'idle'          # idle -> enter -> posing -> leave -> idle
        self.pose_target, self.pose_at, self.pose_end = 0.0, -1e9, False
        self.pose_steps, self.pose_next = [], 0.0
        self.pose_status = None
        self.odom_yaw = None
        if get('pose_yaw_topic'):
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pose_status_pub = self.create_publisher(String, '/maze/pose_status', latched)
            self.create_subscription(Float32, get('pose_yaw_topic'), self.on_pose_yaw, 10)
            if get('pose_pitch_topic'):
                self.create_subscription(Float32, get('pose_pitch_topic'), self.on_pose_pitch,
                                         10)
            self.create_timer(0.1, self.pose_tick)
            why = self.pose_unavailable(check_sub=False)
            self.set_pose_status(f'UNAVAILABLE: {why}' if why else 'IDLE')
        if get('lie_topic'):
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.lie_status_pub = self.create_publisher(String, '/maze/lie_status', latched)
            self.create_subscription(Bool, get('lie_topic'), self.on_lie, 10)
            self.create_subscription(Float32, '/maze/floor_z',
                                     lambda m: setattr(self, 'floor_z', float(m.data)), latched)
            self.create_timer(0.1, self.lie_tick)
            why = self.pose_unavailable(check_sub=False)
            self.set_lie_status(f'UNAVAILABLE: {why}' if why else 'IDLE')
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
        self.walk_dist_now = self.walk_dist_down if 'descending' in why else self.walk_dist
        self.enter_ai(f'lined up at the stairs ({why.strip()})')

    def on_start(self, _):
        """/maze/start while in AI mode: leave it (after entering has finished)."""
        if self.walk is not None:
            self.end_walk('/maze/start')
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
            if self.ai_exit_wanted:
                self.get_logger().warn('AI mode sent; leaving it again (/maze/start)')
                self.exit_ai()
            elif self.walk_dist_now > 0.0 and self.walk_speed > 0.0:
                self.walk = {'at': now + self.walk_delay, 'start': None,
                             'dist': self.walk_dist_now}
                self.get_logger().warn(
                    f'AI mode sent (no feedback: watch the robot / controller). Walking '
                    f'{self.walk_dist_now:.2f} m forward at {self.walk_speed:.2f} m/s in '
                    f'{self.walk_delay:.1f} s')
            else:
                self.get_logger().warn('AI mode sent (no feedback: watch the robot / '
                                       'controller). No velocity is sent until /maze/start')
        else:
            self.ai_exit_wanted = False
            self.ai_mode = False
            self.get_logger().warn('AI mode left (no feedback: watch the robot). Forwarding '
                                   'velocity again; the operator sends lite3_auto to drive')

    # ------------------------------------------------------------------ AI walk
    def on_odom(self, msg):
        self.mark('odom')
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)
        self.odom_z = p.z
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Horizontal only: the body rises fast on stairs and ramp edges. Speed over >= 0.5 s
        # of stamps, not between two messages: stamp jitter made 2.4 m/s out of 0.15 (sim).
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        hist = self.odom_hist
        last = hist[-1] if hist else None
        hist.append((t, p.x, p.y))
        while len(hist) > 2 and t - hist[1][0] >= 0.5:
            hist.popleft()                      # keep the newest sample >= 0.5 s old first
        if last is None or self.blocked:
            return
        why = None
        d = math.hypot(p.x - last[1], p.y - last[2])
        if d > self.odom_max_jump:
            why = f'/odom jumped {d:.2f} m in one message'
        else:
            span = t - hist[0][0]
            if span >= 0.5:
                v = math.hypot(p.x - hist[0][1], p.y - hist[0][2]) / span
                if v > self.odom_max_speed:
                    why = f'/odom moving {v:.1f} m/s over {span:.1f} s'
        if why:
            # Fast-LIO jumped: stop and wait until /odom is consistent again (check_pause).
            # Start the speed window afresh, or the pre-jump sample trips it again for 0.5 s.
            why += ' (odometry jumped)'
            hist.clear()
            hist.append((t, p.x, p.y))
            now = time.monotonic()
            self.jumps.append(now)
            while self.jumps and now - self.jumps[0] > self.odom_jump_window:
                self.jumps.popleft()
            if self.walk is not None:
                self.end_walk(why, warn=True)
            if len(self.jumps) > self.odom_jump_limit:
                self.block(f'{why}; {len(self.jumps)} jumps in {self.odom_jump_window:.0f} s '
                           '(odometry diverged?)')
            else:
                self.pause(why)

    def walk_tick(self):
        """Drive the AI walk: straight ahead until walk_dist on /odom, then one stop."""
        w = self.walk
        if w is None:
            return
        now = time.monotonic()
        if now < w['at']:
            return
        if self.blocked:
            self.end_walk(f'sensor fault ({self.blocked})', warn=True)
            return
        stale = [n for n in self.seen if self.sensor_age(n) > self.sensor_timeout]
        if stale or self.odom_xy is None:
            self.end_walk(f'no {"/".join(stale) or "odom"} for > {self.sensor_timeout} s')
            return
        if w['start'] is None:
            w['start'] = self.odom_xy
            w['best'], w['best_t'] = 0.0, now
            self.get_logger().warn(f'AI walk: driving {w["dist"]:.2f} m forward')
        dx, dy = self.odom_xy[0] - w['start'][0], self.odom_xy[1] - w['start'][1]
        d = (dx * dx + dy * dy) ** 0.5
        if d >= w['dist']:
            self.end_walk(f'{d:.2f} m done')
            return
        if d >= w['best'] + 0.1:
            w['best'], w['best_t'] = d, now
        elif now - w['best_t'] > self.walk_timeout:
            self.end_walk(f'no progress for {self.walk_timeout:.0f} s at {d:.2f} m (does the '
                          'robot follow /cmd_vel in AI mode?)', warn=True)
            return
        out = Twist()
        out.linear.x = self.walk_speed
        self.pub.publish(out)

    def end_walk(self, why, warn=False):
        """One zero Twist if the walk was driving, then no more velocity in AI mode."""
        drove = self.walk is not None and self.walk['start'] is not None
        self.walk = None
        if drove:
            self.pub.publish(Twist())
        log = self.get_logger().warn if warn else self.get_logger().info
        log(f'AI walk ended: {why}' + ('; sent stop' if drove else ' (had not started)')
            + '. No velocity until /maze/start')

    def send_simple(self, name, code):
        self.simple_pub.publish(self.simple_cmd(cmd_code=code, size=0, type=0))
        self.get_logger().info(f'sent {name} (0x{code:08X}) on /simple_cmd')

    # ------------------------------------------------------------------ pose-mode looks
    def set_pose_status(self, status):
        if status != self.pose_status:
            self.pose_status = status
            self.pose_status_pub.publish(String(data=status))

    def pose_unavailable(self, check_sub=True):
        """Why a pose look can't start now, or None."""
        if not self.drive:
            return 'DRY RUN (drive:=false)'
        if not self.simple_publisher():
            return 'transfer_interfaces not found'
        if check_sub and self.simple_pub.get_subscription_count() == 0:
            return 'no subscriber on /simple_cmd'
        if self.ai_mode:
            return 'AI mode'
        if self.blocked:
            return f'sensor fault ({self.blocked})'
        if getattr(self, 'gait_steps', None):
            return 'gait not sent yet'
        if getattr(self, 'lie_phase', 'idle') != 'idle':
            return f'lying down / standing up ({self.lie_phase})'
        return None

    def on_pose_yaw(self, msg):
        if math.isnan(msg.data):
            self.pose_end = True
            return
        self.pose_target, self.pose_at, self.pose_end = float(msg.data), time.monotonic(), False

    def on_pose_pitch(self, msg):
        v = float(msg.data)
        self.pitch_target, self.pitch_at = (0.0 if math.isnan(v) else v), time.monotonic()

    def pitch_axis(self, pitch):
        """Body pitch (rad, + = nose down) -> pitch axis value (+ = lowers the head)."""
        v = int(round(pitch / self.pose_max_pitch * AXIS_MAX))
        return max(-AXIS_MAX, min(AXIS_MAX, v))

    def send_pitch(self, value):
        self.simple_pub.publish(self.simple_cmd(cmd_code=PITCH_AXIS, size=value, type=0))

    def pose_axis(self, yaw):
        """Body yaw (rad, + = left) -> yaw axis value (+ = right on the robot)."""
        v = int(round(-yaw / self.pose_max_yaw * AXIS_MAX))
        return max(-self.pose_axis_max, min(self.pose_axis_max, v))

    def track_target(self, target, d):
        """Per yaw target: the furthest /odom saw the body go towards it (d: signed body yaw
        from the stance, rad, + = left) -> the real yaw at that axis value (pose_max_yaw)."""
        t = self.pose_track
        if t is not None and abs(target - t[0]) < math.radians(1):
            t[1] = max(t[1], math.copysign(1.0, t[0]) * d)
            return
        self.log_track()
        self.pose_track = [target, math.copysign(1.0, target) * d]

    def log_track(self):
        t, self.pose_track = self.pose_track, None
        if t is None or abs(t[0]) < math.radians(1):
            return
        a = self.pose_axis(t[0])
        self.get_logger().info(
            f'pose target {math.degrees(t[0]):+.0f} deg -> yaw axis {a} '
            f'({100.0 * abs(a) / AXIS_MAX:.0f} % of full): body reached '
            f'{math.copysign(math.degrees(t[1]), t[0]):+.0f} deg on /odom')

    def send_axis(self, value):
        self.simple_pub.publish(self.simple_cmd(cmd_code=YAW_AXIS, size=value, type=0))

    def pose_tick(self):
        now = time.monotonic()
        fresh = not self.pose_end and now - self.pose_at <= self.pose_timeout
        if self.pose_phase == 'idle':
            if not fresh:
                # Re-check a passing UNAVAILABLE (gait not sent yet at startup, AI mode,
                # sensor fault): the explorer never sends a target while it reads
                # UNAVAILABLE, so without this the status stayed stuck and every look
                # fell back to Nav2 spins (run 2026-11-20 01:17).
                if self.pose_status.startswith('UNAVAILABLE'):
                    why = self.pose_unavailable()
                    if not why:
                        self.get_logger().info('pose looks available now')
                    self.set_pose_status(f'UNAVAILABLE: {why}' if why else 'IDLE')
                return
            why = self.pose_unavailable()
            if why:
                if not self.pose_status.startswith('UNAVAILABLE'):
                    self.get_logger().warn(f'pose look requested but {why}: not sent')
                self.set_pose_status(f'UNAVAILABLE: {why}')
                return
            self.stop('pose mode')                  # one zero Twist, only if driving
            self.pose_phase = 'enter'
            self.pose_steps = ([('manual mode', MANUAL_MODE)] if self.pose_manual else []) \
                + [('Pose mode', POSE_MODE)]
            self.pose_next = now
            self.pose_yaw0, self.pose_seen, self.pose_asked = self.odom_yaw, 0.0, 0.0
            self.pose_track = None
            self.pitch_used, self.pitch_asked = False, 0.0
            self.get_logger().warn('pose look: velocity forwarding STOPPED; '
                                   + ' -> '.join(n for n, _ in self.pose_steps))
            self.set_pose_status('ENTERING')
        if self.pose_phase == 'enter':
            if now < self.pose_next:
                return
            if self.pose_steps:
                self.send_simple(*self.pose_steps.pop(0))
                self.pose_next = now + self.pose_settle
                return
            self.pose_phase, self.pose_fresh_at = 'posing', now
            self.set_pose_status('POSING')
        if self.pose_phase == 'posing':
            if fresh:
                self.pose_fresh_at = now
                self.pose_asked = max(self.pose_asked, abs(self.pose_target))
            self.send_axis(self.pose_axis(self.pose_target) if fresh else 0)
            pitch_fresh = now - self.pitch_at <= self.pose_timeout
            if pitch_fresh and abs(self.pitch_target) > 0:
                self.pitch_used = True
                self.pitch_asked = max(self.pitch_asked, self.pitch_target)
            if self.pitch_used:            # only looks that pitch send the pitch axis
                self.send_pitch(self.pitch_axis(self.pitch_target) if pitch_fresh and fresh
                                else 0)
            if self.odom_yaw is not None and self.pose_yaw0 is not None:
                d = math.atan2(math.sin(self.odom_yaw - self.pose_yaw0),
                               math.cos(self.odom_yaw - self.pose_yaw0))
                self.pose_seen = max(self.pose_seen, abs(d))
                if fresh:
                    self.track_target(self.pose_target, d)
            if self.pose_end or now - self.pose_fresh_at > self.pose_linger:
                self.log_track()
                self.pose_phase = 'leave'
                self.pose_steps = [('Move mode', MOVE_MODE)]
                if self.gait_code is not None:
                    self.pose_steps.append((f'gait {self.gait_req}', self.gait_code))
                if self.pose_manual:
                    self.pose_steps.append(('navigation (auto) mode', AUTO_MODE))
                self.pose_next = now + 0.5             # axis 0 first: body back to centre
                self.set_pose_status('LEAVING')
                self.get_logger().info(
                    f'pose look done: asked up to {math.degrees(self.pose_asked):.0f} deg '
                    f'(pose_max_yaw {math.degrees(self.pose_max_yaw):.0f} deg at full axis, at most '
                    f'{100.0 * self.pose_axis_max / AXIS_MAX:.0f} % sent), '
                    f'/odom yaw moved up to {math.degrees(self.pose_seen):.0f} deg'
                    + (f'; pitch asked up to {math.degrees(self.pitch_asked):.0f} deg nose down '
                       f'(pose_max_pitch {math.degrees(self.pose_max_pitch):.0f} deg)'
                       if self.pitch_used else ''))
            return
        if self.pose_phase == 'leave':
            if now < self.pose_next:
                if self.pose_steps and self.pose_steps[0][1] == MOVE_MODE:
                    self.send_axis(0)       # only in Pose mode: in Move mode it's a turn stick
                    if self.pitch_used:
                        self.send_pitch(0)  # in Move mode the same code is forward/back
                return
            if self.pose_steps:
                self.send_simple(*self.pose_steps.pop(0))
                self.pose_next = now + self.pose_settle
                return
            self.pose_phase = 'idle'
            self.set_pose_status('IDLE')
            self.get_logger().warn('pose look: back in Move mode'
                                   + (' + auto mode' if self.pose_manual else '')
                                   + '; forwarding velocity again')

    # ------------------------------------------------------------------ lie down / stand up
    def set_lie_status(self, status):
        if status != self.lie_status:
            self.lie_status = status
            self.lie_status_pub.publish(String(data=status))
            self.get_logger().info(f'lie: {status}')

    def on_lie(self, msg):
        self.lie_want = bool(msg.data)
        if self.lie_want:
            self.lie_at = time.monotonic()

    def on_local_floor(self, msg):
        self.local_floor, self.local_floor_at = float(msg.data), time.monotonic()

    def on_terrain(self, msg):
        try:
            self.terrain_level = json.loads(msg.data).get('level')
        except (ValueError, AttributeError):
            return
        self.terrain_at = time.monotonic()

    def body_h(self):
        """base_link height above the floor under the feet (stair_detector, while fresh;
        else wall_scan's start floor_z), or None."""
        floor = self.floor_z
        if self.local_floor is not None and \
                time.monotonic() - self.local_floor_at <= self.local_floor_timeout:
            floor = self.local_floor
        if self.odom_z is None or floor is None:
            return None
        return self.odom_z - floor

    def lie_tick(self):
        now = time.monotonic()
        want = self.lie_want and now - self.lie_at <= self.lie_linger
        h = self.body_h()
        state = posture(h)
        hs = '?' if h is None else f'{h:.2f}'
        if self.lie_phase == 'idle':
            st = self.lie_status or ''
            if not want:
                if st.startswith('UNAVAILABLE'):
                    why = self.pose_unavailable()
                    self.set_lie_status(f'UNAVAILABLE: {why}' if why else 'IDLE')
                return
            why = self.pose_unavailable() or (None if state == 'standing' else
                                              f'robot not measured standing (height {hs} m)')
            if self.pose_phase != 'idle':
                why = f'pose look running ({self.pose_phase})'
            if why:
                self.set_lie_status(f'UNAVAILABLE: {why}')
                return
            self.stop('lie down')
            self.lie_phase = 'down'
            self.lie_steps = ([('manual mode', MANUAL_MODE)] if self.lie_manual else []) + \
                [('stand/sit toggle (lie down)', STAND_TOGGLE)]
            self.lie_next, self.lie_deadline = now, now + self.lie_down_timeout + 1.0
            self.get_logger().warn(f'lie down (base_link {hs} m up): velocity forwarding '
                                   'STOPPED')
            self.set_lie_status('LYING_DOWN')
        if self.lie_phase == 'down':
            if self.lie_steps:
                if now >= self.lie_next:
                    name, code = self.lie_steps.pop(0)
                    if code == STAND_TOGGLE and state != 'standing':
                        self.get_logger().error(f'not standing before the toggle (height {hs} '
                                                'm): toggle NOT sent')
                        return self.begin_lie_leave(now, 'not standing before the toggle')
                    self.send_simple(name, code)
                    self.lie_next = now + self.pose_settle
                return
            if state == 'lying':
                self.lie_phase = 'lying'
                self.set_lie_status('LYING')
                return
            if now > self.lie_deadline:
                self.get_logger().warn(f'robot not lying {self.lie_down_timeout:.0f} s after '
                                       f'the toggle (height {hs} m)')
                if state == 'standing':
                    return self.begin_lie_leave(now, 'did not lie down')
                self.lie_phase = 'lying'            # somewhere between: stand it up properly
                return
            return
        if self.lie_phase == 'lying':
            if want:
                return
            self.lie_phase = 'up'
            self.lie_steps = [('stand/sit toggle (stand up)', STAND_TOGGLE)] \
                if state == 'lying' else []
            if state != 'lying':
                self.get_logger().warn(f'stand up: robot not measured lying (height {hs} m): '
                                       'toggle NOT sent, waiting for it to stand')
            self.lie_next, self.lie_deadline, self.lie_stood = now, now + self.lie_up_timeout, \
                None
            self.set_lie_status('STANDING_UP')
        if self.lie_phase == 'up':
            if self.lie_steps and now >= self.lie_next:
                self.send_simple(*self.lie_steps.pop(0))
                self.lie_next = now + self.pose_settle
                return
            if state == 'standing':
                self.lie_stood = self.lie_stood or now
                if now - self.lie_stood >= 1.0:
                    return self.begin_lie_leave(now, 'standing')
                return
            self.lie_stood = None
            if now > self.lie_deadline:
                self.lie_phase = 'error'
                self.get_logger().error(f'robot NOT standing {self.lie_up_timeout:.0f} s after '
                                        f'the stand-up toggle (height {hs} m): velocity stays '
                                        'blocked. Stand it with the handheld controller, then '
                                        'restart maze_nav')
                self.set_lie_status(f'ERROR: not standing after stand-up (height {hs} m)')
            return
        if self.lie_phase == 'leave':
            if now < self.lie_next:
                return
            if self.lie_steps:
                self.send_simple(*self.lie_steps.pop(0))
                self.lie_next = now + self.pose_settle
                return
            self.lie_phase = 'idle'
            self.get_logger().warn('lie: standing, Move mode + gait'
                                   + (' + auto mode' if self.lie_manual else '')
                                   + ' sent; forwarding velocity again')
            self.set_lie_status('IDLE')

    def begin_lie_leave(self, now, why):
        """Back to driving: Move mode -> gait [-> auto mode]."""
        self.lie_phase = 'leave'
        self.lie_steps = [('Move mode', MOVE_MODE)]
        if self.gait_code is not None:
            self.lie_steps.append((f'gait {self.gait_req}', self.gait_code))
        if self.lie_manual:
            self.lie_steps.append(('navigation (auto) mode', AUTO_MODE))
        self.lie_next = now + 0.5
        self.set_lie_status(f'STANDING_UP: {why}; mode commands')

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
        if self.pose_phase != 'idle':
            if msg.linear.x != 0.0 or msg.angular.z != 0.0:
                self.get_logger().warn(f'pose mode ({self.pose_phase}): velocity command '
                                       'dropped', throttle_duration_sec=5.0)
            return
        if self.lie_phase != 'idle':
            if msg.linear.x != 0.0 or msg.angular.z != 0.0:
                self.get_logger().warn(f'lying down / standing up ({self.lie_phase}): velocity '
                                       'command dropped', throttle_duration_sec=5.0)
            return
        if self.blocked or self.paused:
            return
        stale = [n for n in self.seen if self.sensor_age(n) > self.sensor_timeout]
        if stale and (msg.linear.x != 0.0 or msg.angular.z != 0.0):
            self.pause(f'no {"/".join(stale)} for > {self.sensor_timeout} s')
            return
        out = Twist()
        now = time.monotonic()
        c = self.terrain_cfg
        cap = terrain_cap(self.terrain_level, now - self.terrain_at, self.max_v,
                          c['terrain_rough_speed'], c['terrain_obstacle_speed'],
                          c['terrain_timeout'])
        if cap < self.max_v and msg.linear.x > cap:
            self.get_logger().info(f'terrain ahead {self.terrain_level}: speed capped at '
                                   f'{cap:.2f} m/s', throttle_duration_sec=5.0)
        out.linear.x = max(0.0, clamp(msg.linear.x, cap))
        out.angular.z = min_spin(out.linear.x, clamp(msg.angular.z, self.max_w),
                                 self.min_spin)
        self.last_cmd, self.active = now, True
        self.pub.publish(out)

    def watchdog(self):
        if self.paused and not self.blocked:
            self.check_pause()
            return
        if not self.active or self.ai_mode:
            return
        stale = [n for n in self.seen if self.sensor_age(n) > self.sensor_timeout]
        if stale and not self.blocked:
            self.pause(f'no {"/".join(stale)} for > {self.sensor_timeout} s')
        elif time.monotonic() - self.last_cmd > self.idle_timeout:
            self.stop('Nav2 stopped sending commands')

    def pause(self, reason):
        """Late cloud / odom, or an /odom jump: stop and drop Nav2 commands until steady."""
        self.paused = reason
        self.paused_at = time.monotonic()
        self.fresh_since = None
        self.active = True
        jumped = 'odometry jumped' in reason
        self.stop(f'{"ODOM JUMP" if jumped else "SENSOR STALE"} ({reason}); holding Nav2 off '
                  'until cloud and odom are steady for '
                  f'{self.sensor_recover + (self.odom_recover if jumped else 0.0):.1f} s')
        if jumped:
            self.get_logger().warn(f'odometry jump {len(self.jumps)}/{self.odom_jump_limit} in '
                                   f'{self.odom_jump_window:.0f} s: pausing; one more latches'
                                   if len(self.jumps) == self.odom_jump_limit else
                                   f'odometry jump {len(self.jumps)}/{self.odom_jump_limit} in '
                                   f'{self.odom_jump_window:.0f} s: pausing')

    def check_pause(self):
        """While paused: resume once every sensor has been fresh for sensor_recover s; latch
        when one has been silent for sensor_timeout + sensor_latch s."""
        now = time.monotonic()
        ages = {n: self.sensor_age(n) for n in self.seen}
        stale = [n for n, a in ages.items() if a > self.sensor_timeout]
        if stale:
            self.fresh_since = None
            worst = max(ages[n] for n in stale)
            if self.sensor_latch > 0 and worst > self.sensor_timeout + self.sensor_latch:
                self.paused = None
                self.block(f'no {"/".join(stale)} for {min(worst, 999.0):.1f} s')
            return
        if self.jumps and now - self.jumps[-1] < self.odom_recover:
            self.fresh_since = None             # /odom jumped recently: not steady yet
            return
        if self.fresh_since is None:
            self.fresh_since = now
        if now - self.fresh_since >= self.sensor_recover:
            jumped = 'odometry jumped' in self.paused
            self.get_logger().info(f'sensors steady again after {now - self.paused_at:.1f} s '
                                   f'({self.paused}): forwarding Nav2 again'
                                   + ('; costmaps cleared' if jumped else ''))
            if jumped:
                self.clear_costmaps()
            self.paused = None
            self.fresh_since = None

    def clear_costmaps(self):
        """After an odometry jump: drop walls marked from the old pose (fire and forget)."""
        for client in self.clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
            else:
                self.get_logger().warn(f'{client.srv_name} not available: costmap not cleared')

    def block(self, reason):
        self.blocked = reason
        self.paused = None
        self.active = True
        self.stop(f'SENSOR FAULT ({reason}); ignoring Nav2 until restart')

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
        if node.walk is not None:
            node.end_walk('shutting down')
        if node.pose_phase in ('posing', 'leave'):
            node.send_axis(0)
            node.get_logger().warn('shutting down in Pose mode: axis 0 sent; the robot stays '
                                   'in Pose mode (send lite3_manual)')
        node.active = True
        node.stop('shutting down')
        end = time.monotonic() + 0.3  # let the reliable stop go out
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
