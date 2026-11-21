"""curtain_detector: find curtains (lidar + depth camera), then walk the robot through them.

A curtain is a wall in front with a gap at the bottom smaller than `max_gap` (0.15 m) and
wider than `min_width` (0.6 m; spec 0.7, measured runs short) (detection:
maze_nav/curtains.py). Run in maze_nav.launch.py
with curtains:=true (default).

  0. Every /livox/points cloud goes into odom (TF at its stamp, never blocking) and into a
     short history (`accumulate` clouds). Its lidar blockage is measured too (below).
     With `sensors` depth or both (default; needs depth_topic + depth_camera_info_topic),
     every depth frame becomes points (`depth_stride`, up to `depth_max` m) in odom too
     (camera pose from `camera_link_frame` + the optical rotation, odom -> base_link at
     its stamp), last `depth_accumulate` frames. Range from base_link: lidar ~1.2-2.6 m,
     depth ~0.7-2.0 m, both ~0.6-2.6 m; closer than ~0.55 m nothing sees under a curtain.
  1. IDLE: at `rate` Hz, while maze_explorer is EXPLORING, TURNING, LOOKING or FACING_QR
     (`trigger_states`), and also DEAD_END / JUNCTION (`detect_states`: detection only, no
     handover - a curtain looks like a dead end), curtains.detect_curtains on the history.
     `vote_need` hits within `vote_window` s confirm a curtain. Wall band = band_low /
     band_high (the launch passes wall_scan's). Every `diag_period` s at most, a near miss is
     logged: "no curtain: wall with a 20 cm gap under it ... taken as a hurdle", "seen under
     only 2 of 9 bins", "curtain candidate ...: 1/3 votes" (curtains.diagnose).
     Confirmed curtains are kept and published (latched JSON) in the map frame (`map_frame`;
     SLAM corrects Fast-LIO drift and jumps, which are worst right at a curtain with the
     lidar covered; map_frame:='' = odom) on /curtain/curtains. maze_explorer clears them on
     its copy of /map (frontiers behind them become reachable; goals there are sent to a
     spot in front of the curtain first). wall_scan drops a curtain's points only while the
     robot is being walked through it (`crossing`) and once it has been (`crossed`): a false
     detection (a real wall with a low gap) stays a wall in the costmaps. SLAM's /map keeps
     every curtain as a wall (wall_scan leaves +inf there, which slam_toolbox ignores);
     Nav2's global costmap reads maze_explorer's /maze/map instead (curtains free), so it
     can plan through one walked through. maze_explorer sends any goal only reachable
     through a curtain to the spot in front of it first, so this node drives every crossing.
  2. A confirmed curtain within `trigger_distance` m, the robot on its near side, within
     `trigger_max_offset` of its centre line and facing it within `trigger_max_bearing`
     (crossing back one already walked through: `back_max_offset`, `back_max_bearing`)
     -> HANDOVER. maze_explorer cancels Nav2, stops and goes to CURTAIN; only then does
     this node drive (nav_cmd_vel, through cmd_vel_bridge). No CURTAIN within
     `handover_timeout` s -> IDLE.
  2b. INSPECTING (`inspect`, user 2026-11-20; curtain_inspect.py): no confirmed curtain, but
     a single unconfirmed detection, or a near miss with a gap < `inspect_max_gap` (0.22 m)
     over > `inspect_min_width` (0.6 m, curtains.near_miss_segment), `inspect_min_distance` (0.6)
     to `inspect_trigger_distance` (2.5) m ahead and within `inspect_max_bearing` (35 deg)
     of the heading, its line facing the robot (normal within 45 deg of the heading: the
     extru walls' side windows read as 12-46 cm gaps on the side walls 0.4-0.6 m away),
     while maze_explorer is in trigger_states -> the same HANDOVER. Candidates are only
     taken during a junction look or dead-end stop (`look_states`, user 2026-11-21); ones
     seen elsewhere are logged and dropped (confirmed curtains are still crossed anywhere).
     Junction looks, dead-end and start scans come first: a candidate seen during DEAD_END /
     JUNCTION / SCANNING is kept and inspected once the explorer is back in trigger_states
     (within 20 s, still in range). Then (`inspect_mode` lie): drive to the middle of the
     cell in front (`inspect_distance` 0.5 m before the line, at its centre; forward only),
     square up, hold still, LIE DOWN (/maze/lie -> cmd_vel_bridge's stand/sit toggle, the
     height checked) - status LYING, which maze_explorer doesn't pre-empt - and measure the
     gap from low down for `inspect_lie_hold` s (curtains.gap_at_line: hem height per slice
     with floor seen behind it). Stand up (Move mode, gait, auto mode again). A median gap
     of `gap_min`..`gap_max` (12-20 cm) over >= half the width in >= 3 readings ->
     confirmed (tracker.confirm), FACING (3.) and through; else IDLE and the spot is left
     alone for `inspect_forget` s. Can't lie down (sim, drive:=false): stand up, then the
     Pose-mode look (`inspect_mode` pose: /maze/pose_yaw + /maze/pose_pitch, 12 deg nose
     down, yaw both ways; no Pose mode either: whole-robot turns) and the tracker's votes
     decide. Not standing after the stand-up -> STUCK (maze_explorer holds until
     /maze/start). INSPECTING / LYING are not in DRIVING: wall_scan keeps the candidate as a
     wall throughout.
  3. FACING: turn on the spot to face the curtain's middle (skipped within `align_tol`).
     APPROACHING / THROUGH: drive at `approach_ratio` x `max_speed` (70 % of Nav2's speed),
     steering at the curtain's middle, until base_link is `through_distance` (0.20 m) past
     the curtain line (THROUGH once the front reaches the curtain).
     WIGGLING: turn `wiggle_angle` (20 deg) left, twice that right, then back left again,
     each side cut to what the walls on /map allow (curtain cells count as free).
     CHECKING: hold still `settle_time` s, then look at fresh sensor data. The lidar is
     blocked if > 30 % of its returns are within 0.5 m (body left out) or its far returns
     drop below 40 % of normal. The depth camera (depth_topic, '' = off) is blocked if
     less than 25 % of its pixels read further than 0.3 m.
     Neither blocked -> PASSED (maze_explorer explores on). Blocked -> alternately CREEPING
     (`creep_step` 5 cm straight ahead) and WIGGLING again, CHECKING after each step, up to
     `max_unblock_steps` steps.
  4. STUCK: still blocked after all steps, a wall in the way, or `manoeuvre_timeout` s
     gone: zero velocity. maze_explorer holds (AT_CURTAIN) until /maze/start, so the
     operator can lift the curtain off the sensors:
         ros2 topic pub --once /maze/start std_msgs/msg/Empty
  Yield: maze_explorer leaves CURTAIN for JUNCTION, DEAD_END or EXPLORING while FACING or
  APPROACHING (a junction or dead end comes first): stop, IDLE, curtain not marked passed.
  ABORTED: no TF for 2 s, or maze_explorer left CURTAIN otherwise. Zero velocity.
A curtain walked through is marked passed on that side (`passed` in /curtain/curtains) and
crossed (PASSED, or STUCK / ABORTED with base_link past the line), and
isn't triggered again from that side. Once walked through, it is only crossed back (from the
other side) when maze_explorer's goal (/maze/goal) is beyond it or within
`regoal_radius` of it, so the robot doesn't walk back just because it faces it.

The manoeuvre drives on odom TF (Fast-LIO), on an odom copy of the curtain taken when it
starts (map -> odom at that moment). With the lidar covered, Fast-LIO runs on its IMU; the
moves are short (20 cm, 20-40 deg turns) so its drift stays small.

Topics
  in : /livox/points (cloud_topic) + TF odom -> its frame, TF odom -> base_link,
       /maze/status (latched), /maze/floor_z (std_msgs/Float32, latched, from wall_scan),
       /maze/local_floor_z (Float32, stair_detector: the floor under the feet; used instead
       of floor_z while fresh: the curtain cell can be lower than the start plate),
       /map (latched) + TF map -> odom for the wall checks, depth_topic (optional)
  out: /curtain/status   std_msgs/String latched "STATE: why", STATE one of IDLE, HANDOVER,
                         FACING, APPROACHING, THROUGH, WIGGLING, CREEPING, CHECKING, PASSED,
                         STUCK, ABORTED, INSPECTING, LYING, CONFIRMED ("CONFIRMED: curtain <id>
                         ...": an inspection confirmed it while maze_explorer holds new curtains;
                         not walked through)
  in : /maze/curtain_hold std_msgs/String latched JSON {"hold_new": bool, "ids": [int]}
                         (maze_explorer, curtain_branch.CurtainHold; none = hold nothing).
                         User 2026-11-20: a curtain confirmed at a junction is crossed only
                         after the junction's other ways are explored. hold_new: an inspection
                         that confirms reports CONFIRMED and goes IDLE instead of walking
                         through. ids: no crossing HANDOVER for these.
       /curtain/curtains std_msgs/String latched JSON (curtains.curtains_to_json, map_frame)
       nav_cmd_vel       geometry_msgs/Twist, only while driving

    ros2 run maze_nav curtain_detector --ros-args -p drive:=false    # detect + log only
"""
import collections
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import Grid, cloud_xyz, free_distance_ahead, max_turn
from maze_nav.curtains import (BlockageMonitor, Curtain, CurtainParams, CurtainTracker,
                               clear_curtains, curtains_to_json, depth_blockage, depth_points,
                               detect_curtains, diagnose, gap_at_line, invert_pose,
                               lidar_blockage, near_miss_segment, wall_between, wrap)
from maze_nav.curtain_branch import CurtainHold
from maze_nav.curtain_inspect import InspectParams, InspectPlan
from maze_nav.stairs import WallCheck, camera_from_link

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
DRIVING = ('FACING', 'APPROACHING', 'THROUGH', 'WIGGLING', 'CREEPING', 'CHECKING')


def tf_matrix(t):
    q, a = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return r, np.array([a.x, a.y, a.z])


def tf_yaw(t):
    q = t.transform.rotation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CurtainDetector(Node):

    def __init__(self):
        super().__init__('curtain_detector')
        p = self.declare_parameter
        p('cloud_topic', '/livox/points')
        p('depth_topic', '')                   # '' = lidar only
        p('depth_camera_info_topic', '')       # needed to detect with depth (sensors)
        # Sensors for detection: lidar (~1.2-2.6 m), depth (~0.6-2.0 m, +-40 deg) or both
        # (~0.6-2.6 m). The depth camera is used for the covered-sensor check either way.
        p('sensors', 'both')
        p('camera_link_frame', 'camera_link')  # camera pose = this + the Orbbec's optical
        #                                        rotation; '' = the depth image's frame in TF
        p('depth_stride', 4)                   # px between depth points (160 x 120 at 640x480)
        p('depth_max', 2.0)                    # m camera z; further on the lidar is better
        p('depth_accumulate', 3)               # depth frames per detection (odom)
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('map_frame', 'map')
        p('status_topic', '/maze/status')      # '' = no explorer handshake (drives at once)
        p('trigger_states', ['EXPLORING', 'TURNING', 'LOOKING', 'FACING_QR'])
        # Detect (publish, clear from the map) but never hand over in these too: a curtain
        # looks like a dead end, and there the robot holds still facing it (user 2026-11-20).
        p('detect_states', ['DEAD_END', 'JUNCTION'])
        # Possible curtains are only taken (for an inspection) during these explorer looks
        # (user 2026-11-21): the junction look and the dead-end stop. Confirmed curtains are
        # still crossed wherever they are. [] = any time (the old behaviour).
        p('look_states', ['JUNCTION', 'DEAD_END'])
        p('band_low', 0.30)                    # m above the floor: wall band (= wall_scan)
        p('band_high', 0.45)
        p('diag_period', 3.0)                  # s between near-miss log lines (0 = off)
        # Inspection (user 2026-11-20, curtain_inspect.py): a wall ahead with a suitable gap
        # that isn't a confirmed curtain -> stop, square up at inspect_distance, look in Pose
        # mode (pitch down, yaw both ways) while detecting; confirmed -> walk through.
        p('inspect', True)
        p('inspect_max_gap', 0.22)             # m: near misses with a smaller gap qualify
        p('inspect_min_width', 0.60)           # m of seen-under wall, more than this (user 2026-11-21)
        p('inspect_trigger_distance', 2.5)     # m to the candidate's line
        p('inspect_min_distance', 0.6)         # m: closer candidates are side-wall windows etc.
        p('inspect_max_bearing', math.radians(35))  # candidate centre off the heading
        p('inspect_mode', 'lie')               # 'lie' (user 2026-11-20) or 'pose'
        p('inspect_distance', 0.5)             # m before the line: middle of the cell in front
        p('gap_min', 0.12)                     # m: lying-down gap that confirms a curtain ...
        p('gap_max', 0.20)                     # ... up to this
        p('inspect_lie_hold', 3.0)             # s lying still while measuring
        p('lie_topic', '/maze/lie')
        p('lie_status_topic', '/maze/lie_status')
        p('inspect_pitch', math.radians(12))   # rad nose down during the look
        p('inspect_look_yaw', math.radians(15))
        p('inspect_hold', 1.5)                 # s per look
        p('inspect_forget', 120.0)             # s a rejected spot is left alone
        # s a possible curtain seen outside a look is kept and added to the next junction /
        # dead-end look's candidates (user 2026-11-21: the 10:48 run dropped the lab curtain
        # seen while EXPLORING); the usual distance / bearing limits apply then
        p('inspect_remember', 60.0)
        p('inspect_forget_radius', 0.6)        # m around a rejected spot
        p('pose_yaw_topic', '/maze/pose_yaw')
        p('pose_pitch_topic', '/maze/pose_pitch')
        p('pose_status_topic', '/maze/pose_status')
        p('cmd_topic', 'nav_cmd_vel')
        p('drive', True)                       # false = detect and publish only
        p('rate', 2.0)                         # Hz detection
        p('control_rate', 20.0)
        p('accumulate', 5)                     # clouds per detection (Mid-360: sparse)
        p('max_age', 1.5)                      # s: older clouds / depth frames are dropped
        #                                        (= wall_scan max_cloud_age: odom often late)
        p('max_gap', 0.15)                     # m (to-do spec)
        p('min_width', 0.60)                   # m (spec 0.7; measured runs short)
        p('max_range', 2.6)                    # m from the lidar
        p('vote_window', 3.0)                  # s
        p('vote_need', 3)
        p('trigger_distance', 2.0)             # m base_link -> curtain line
        p('trigger_max_offset', 0.35)          # m off the curtain's centre line
        p('trigger_max_bearing', math.radians(50))
        # Crossing BACK one already walked through: looser, so Nav2 (whose global costmap has
        # it free) never pushes into the fabric itself; FACING turns to its middle first.
        p('back_max_offset', 0.45)             # m off its centre line
        p('back_max_bearing', math.radians(80))
        p('regoal_radius', 1.0)                # m: goal this close to a passed curtain = cross back
        p('handover_timeout', 3.0)
        p('max_speed', 0.15)                   # m/s: Nav2's desired_linear_vel (launch max_speed)
        p('approach_ratio', 0.7)               # approach + walk-through at this x max_speed
        p('through_distance', 0.20)            # m base_link past the curtain line
        p('wiggle_angle', math.radians(20))
        p('creep_step', 0.05)                  # m per unblock creep
        p('creep_speed', 0.05)
        p('settle_time', 0.8)                  # s still before judging the sensors
        p('max_unblock_steps', 12)             # creeps + wiggles after the first check
        p('manoeuvre_timeout', 120.0)
        p('align_tol', math.radians(5))
        p('max_turn_rate', 0.35)
        p('min_turn_rate', 0.15)
        p('turn_tol', math.radians(3))
        p('wall_check', True)
        p('map_topic', '/map')
        p('footprint_back', 0.37)
        p('footprint_front', 0.27)
        p('footprint_half_width', 0.23)
        p('turn_margin', 0.03)
        p('min_margin', 0.01)
        # Lidar body box for blockage (= wall_scan body_*: footprint + 5 cm)
        p('body_back', 0.42)
        p('body_front', 0.32)
        p('body_half_width', 0.28)
        p('standing_height', 0.35)             # floor fallback: base_link z - this at start
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in (
            'drive', 'trigger_distance', 'trigger_max_offset', 'trigger_max_bearing',
            'back_max_offset', 'back_max_bearing',
            'handover_timeout', 'regoal_radius', 'max_speed', 'approach_ratio', 'through_distance',
            'wiggle_angle', 'creep_step', 'creep_speed', 'settle_time', 'max_unblock_steps',
            'manoeuvre_timeout', 'align_tol', 'max_turn_rate', 'min_turn_rate', 'turn_tol',
            'wall_check', 'turn_margin', 'min_margin', 'standing_height')}
        self.odom, self.base, self.map_frame = g('odom_frame'), g('base_frame'), g('map_frame')
        self.cframe = self.map_frame or self.odom   # frame the curtains are kept in
        self.params = CurtainParams(max_gap=float(g('max_gap')), min_width=float(g('min_width')),
                                    max_range=float(g('max_range')),
                                    band=(float(g('band_low')), float(g('band_high'))))
        self.detect_states = tuple(g('detect_states'))
        self.look_states = tuple(g('look_states'))
        self.diag_period = float(g('diag_period'))
        self.diag_last = -1e9
        self.inspect_on = bool(g('inspect'))
        self.inspect_cfg = {n: float(g(n)) for n in (
            'inspect_max_gap', 'inspect_min_width', 'inspect_trigger_distance', 'inspect_forget',
            'inspect_remember',
            'inspect_forget_radius', 'inspect_min_distance', 'inspect_max_bearing', 'gap_min',
            'gap_max')}
        self.inspect_params = InspectParams(
            distance=float(g('inspect_distance')), pitch=float(g('inspect_pitch')),
            look_yaw=float(g('inspect_look_yaw')), hold=float(g('inspect_hold')),
            speed=min(0.08, float(g('max_speed'))), look_mode=str(g('inspect_mode')),
            lie_hold=float(g('inspect_lie_hold')))
        self.inspect_target = None      # dict (centre, n, p1, p2, what) in odom: pending
        self.gaps = []                  # (gap, coverage) measured lying down
        self.lie_status = None
        self.lie_sent_at = -1e9
        self.lie_pub = self.create_publisher(Bool, g('lie_topic'), 10) if g('lie_topic') \
            else None
        if g('lie_status_topic'):
            self.create_subscription(String, g('lie_status_topic'),
                                     lambda m: setattr(self, 'lie_status', m.data), LATCHED)
        self.inspection = None          # InspectPlan while INSPECTING
        self.inspect_done_at = None     # plan finished: waiting for Pose mode to end
        self.rejected = []              # (xy in odom, monotonic time) not-a-curtain spots
        self.deferred = None            # (candidates, time) seen during a look: inspect after
        self.remembered = []            # (p1, p2, gap, what, time) seen outside looks
        self.pose_live = False
        self.pose_status = None
        self.pose_yaw_pub = self.create_publisher(Float32, g('pose_yaw_topic'), 10) \
            if g('pose_yaw_topic') else None
        self.pose_pitch_pub = self.create_publisher(Float32, g('pose_pitch_topic'), 10) \
            if g('pose_pitch_topic') and g('pose_yaw_topic') else None
        if g('pose_status_topic'):
            self.create_subscription(String, g('pose_status_topic'), self.on_pose_status,
                                     LATCHED)
        self.tracker = CurtainTracker(window=float(g('vote_window')), need=int(g('vote_need')))
        self.max_age = float(g('max_age'))
        self.box = (g('footprint_back'), g('footprint_front'), g('footprint_half_width'))
        self.body = (g('body_back'), g('body_front'), g('body_half_width'))
        self.status_topic = g('status_topic')
        self.trigger_states = tuple(g('trigger_states'))

        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.pending = collections.deque(maxlen=3)
        self.history = collections.deque(maxlen=int(g('accumulate')))   # (pts odom, lidar xyz)
        self.create_subscription(PointCloud2, g('cloud_topic'), self.pending.append,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, '/maze/floor_z', self.on_floor, LATCHED)
        # Floor under the feet (stair_detector, 2 Hz while standing; not updated lying down).
        # The start floor is a raised plate in the presentation mazes (2026-11-21) and the
        # curtain cell bare lab floor 5-10 cm lower: the wall band and the gap measured lying
        # down are taken from this floor while it is fresh.
        self.local_floor, self.local_floor_at = None, -1e9
        self.create_subscription(Float32, '/maze/local_floor_z', self.on_local_floor, 10)
        if self.status_topic:
            self.create_subscription(String, self.status_topic, self.on_explorer, LATCHED)
            self.create_subscription(PoseStamped, '/maze/goal', self.on_goal, LATCHED)
        self.hold = CurtainHold()        # maze_explorer's /maze/curtain_hold
        self.create_subscription(String, '/maze/curtain_hold', self.on_hold, LATCHED)
        if self.cfg['wall_check']:
            self.create_subscription(OccupancyGrid, g('map_topic'), self.on_map, LATCHED)
        self.sensors = str(g('sensors')).lower()
        if self.sensors not in ('lidar', 'depth', 'both'):
            self.get_logger().warn(f'sensors:={self.sensors} unknown, using both')
            self.sensors = 'both'
        self.depth_cfg = {n: g(n) for n in ('camera_link_frame', 'depth_stride', 'depth_max')}
        self.k = None                   # (fx, fy, cx, cy) of the depth image
        self.cam = None                 # (R, t) depth optical frame in base_link
        self.depth_pending = collections.deque(maxlen=2)               # (stamp msg, frame, d)
        self.depth_history = collections.deque(maxlen=int(g('depth_accumulate')))
        use_depth = self.sensors != 'lidar'
        if use_depth and not (g('depth_topic') and g('depth_camera_info_topic')):
            self.get_logger().warn(f'sensors:={self.sensors} but no depth_topic / '
                                   'depth_camera_info_topic: detecting with the lidar only')
            use_depth = False
        self.use_lidar = self.sensors != 'depth' or not use_depth
        self.use_depth = use_depth
        if g('depth_topic'):
            self.create_subscription(Image, g('depth_topic'), self.on_depth,
                                     qos_profile_sensor_data)
        if self.use_depth:
            self.create_subscription(CameraInfo, g('depth_camera_info_topic'), self.on_info,
                                     qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/curtain/status', LATCHED)
        self.curtains_pub = self.create_publisher(String, '/curtain/curtains', LATCHED)
        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic'), 10)
        self.create_timer(0.02, self.process_pending)
        if self.use_depth:
            self.create_timer(0.05, self.process_depth)
        self.create_timer(1.0 / float(g('rate')), self.detect)
        self.create_timer(1.0 / float(g('control_rate')), self.control)

        self.goal = None                # maze_explorer's newest goal (PoseStamped)
        self.floor_z = None             # from wall_scan
        self.floor_guess = None         # until then: base_link z at the first cloud - standing
        self.grid = None
        self.explorer = None
        self.state = None
        self.since = 0.0
        self.lock = None                # Curtain being walked through: an odom copy
        self.lock_id = None             # its id in the tracker (self.cframe)
        self.side = 1                   # side it is crossed towards
        self.plan = None                # current step: dict
        self.steps = 0                  # unblock steps done
        self.last_step = None           # 'creep' or 'wiggle'
        self.started_at = 0.0
        self.last_pose_at = 0.0
        self.monitor = BlockageMonitor()
        self.lidar_block = None         # (monotonic time, blocked, why)
        self.depth_block = None
        self.cooldown_until = 0.0
        self.set_state('IDLE', 'looking for curtains' + (
            f' while {self.status_topic} is {"/".join(self.trigger_states)}'
            if self.status_topic else ' (no explorer handshake: drives on its own)')
            + ('' if self.cfg['drive'] else '; drive:=false, detect only'))
        self.publish_curtains()

    # ------------------------------------------------------------------ inputs
    def set_state(self, state, why=''):
        if state == self.state:
            return
        self.state = state
        self.since = time.monotonic()
        if state in ('STUCK', 'ABORTED'):
            self.get_logger().warn(f'[{state}] {why}')
        else:
            self.get_logger().info(f'[{state}] {why}')
        self.status_pub.publish(String(data=f'{state}: {why}' if why else state))
        crossing = self.lock_id if state in DRIVING else None
        changed = False
        for c in self.tracker.confirmed:
            if c.crossing != (c.id == crossing):
                c.crossing, changed = c.id == crossing, True
        if changed:                     # wall_scan drops it while crossing
            self.publish_curtains()

    def on_floor(self, msg):
        self.floor_z = float(msg.data)

    def on_local_floor(self, msg):
        self.local_floor, self.local_floor_at = float(msg.data), time.monotonic()

    def floor_here(self, max_age):
        """Floor height for the curtain: the floor under the feet if younger than max_age s,
        else the start floor (wall_scan), else the guess from the first cloud."""
        if self.local_floor is not None and time.monotonic() - self.local_floor_at <= max_age:
            return self.local_floor
        return self.floor_z if self.floor_z is not None else self.floor_guess

    def on_map(self, msg):
        self.grid = Grid.from_msg(msg)

    def on_explorer(self, msg):
        self.explorer = msg.data.split(':', 1)[0].strip()
        if self.state == 'HANDOVER' and self.explorer == 'CURTAIN':
            self.start_inspection() if self.inspect_target is not None else \
                self.start_manoeuvre()

    def on_pose_status(self, msg):
        self.pose_status = msg.data

    def on_goal(self, msg):
        self.goal = msg

    def goal_odom(self):
        """maze_explorer's goal (x, y) in odom, or None."""
        if self.goal is None:
            return None
        try:
            tf = self.tf.lookup_transform(self.odom, self.goal.header.frame_id, Time())
        except TransformException:
            return None
        th, t = tf_yaw(tf), tf.transform.translation
        x, y = self.goal.pose.position.x, self.goal.pose.position.y
        return (t.x + math.cos(th) * x - math.sin(th) * y,
                t.y + math.sin(th) * x + math.cos(th) * y)

    def on_depth(self, msg):
        enc = msg.encoding
        if enc in ('16UC1', 'mono16'):
            d = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, -1)[:, :msg.width]
            d = d.astype(np.float32) * 0.001
        elif enc == '32FC1':
            d = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, -1)[:, :msg.width]
        else:
            self.get_logger().warn(f'depth encoding {enc} not supported',
                                   throttle_duration_sec=30.0)
            return
        blocked, frac = depth_blockage(d)
        self.depth_block = (time.monotonic(), blocked, f'{frac:.0%} of depth pixels clear')
        if self.use_depth and self.k is not None:
            self.depth_pending.append((msg.header.stamp, msg.header.frame_id, d))

    def on_info(self, msg):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def camera_pose(self, frame):
        """(R, t) of the depth optical frame in base_link (static, looked up once). From
        camera_link + the fixed optical rotation, as stair_detector does: the robot's
        /tf_static also has an unpitched base_link -> optical frame that TF would pick."""
        if self.cam is None:
            link = self.depth_cfg['camera_link_frame']
            try:
                if link:
                    self.cam = camera_from_link(
                        *tf_matrix(self.tf.lookup_transform(self.base, link, Time())))
                else:
                    self.cam = tf_matrix(self.tf.lookup_transform(self.base, frame, Time()))
            except TransformException:
                return None
            self.get_logger().info(f'depth camera in {self.base}: '
                                   f't={np.round(self.cam[1], 3).tolist()} '
                                   f'(from {link or frame}); curtain sensors: {self.sensors}')
        return self.cam

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.odom, self.base, Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y), tf_yaw(t), \
            t.transform.translation.z

    def frame_tf(self):
        """(tx, ty, th) of odom in self.cframe (latest TF), identity in odom, or None."""
        if self.cframe == self.odom:
            return 0.0, 0.0, 0.0
        try:
            tf = self.tf.lookup_transform(self.cframe, self.odom, Time())
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y, tf_yaw(tf)

    def curtains_odom(self):
        """Odom copies (ids, passed kept) of the confirmed curtains, [] without TF."""
        t = self.frame_tf()
        if t is None:
            return []
        inv = invert_pose(*t)
        return [c.transformed(*inv) for c in self.tracker.confirmed]

    def mark(self, side, crossed):
        """The locked curtain: passed on `side`, crossed if it was walked through."""
        targets = [c for c in self.tracker.confirmed if c.id == self.lock_id]
        if self.lock is not None:
            targets.append(self.lock)
        for c in targets:
            if side not in c.passed:
                c.passed.append(side)
            c.crossed = c.crossed or crossed
        self.publish_curtains()

    def process_pending(self):
        while self.pending:
            msg = self.pending[0]
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > self.max_age:
                self.pending.popleft()
                self.warn_stale('lidar cloud', age, msg.header.frame_id)
                continue
            if not (self.tf.can_transform(self.odom, msg.header.frame_id, stamp)
                    and self.tf.can_transform(self.odom, self.base, stamp)):
                return
            self.pending.popleft()
            try:
                to_odom = self.tf.lookup_transform(self.odom, msg.header.frame_id, stamp)
                base = self.tf.lookup_transform(self.odom, self.base, stamp)
            except TransformException:
                continue
            xyz = cloud_xyz(msg.data, msg.fields, msg.point_step, msg.row_step, msg.width,
                            msg.height, msg.is_bigendian)
            r, t = tf_matrix(to_odom)
            pts = xyz @ r.T + t
            bxy = (base.transform.translation.x, base.transform.translation.y)
            if self.floor_guess is None:
                self.floor_guess = base.transform.translation.z - self.cfg['standing_height']
            self.history.append((pts, t))
            near, far = lidar_blockage(pts, t, bxy, tf_yaw(base), self.body)
            close = any(c.distance(bxy) < 1.2 for c in self.curtains_odom())
            blocked, why = self.monitor.update(near, far, learn=self.state == 'IDLE' and not close)
            self.lidar_block = (time.monotonic(), blocked, why)

    def warn_stale(self, what, age, frame):
        """Dropped data would otherwise stop curtain detection without a word (robot
        2026-11-19: lidar and odom ~1.1 s late through the domain bridge)."""
        tf_ok = self.tf.can_transform(self.odom, self.base, Time())
        self.get_logger().warn(
            f'dropping a {what} ({frame}) {age:.2f} s old (> max_age {self.max_age:.1f} s: it '
            f'arrived late, or TF {self.odom} -> {self.base} at its stamp came too late'
            f'{"" if tf_ok else f"; no {self.odom} -> {self.base} at all yet"}): '
            'no curtain detection from it', throttle_duration_sec=10.0)

    def process_depth(self):
        """Newest depth frames -> points in odom (TF at their stamp, never blocking)."""
        while self.depth_pending:
            stamp_msg, frame, d = self.depth_pending[0]
            stamp = Time.from_msg(stamp_msg)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > self.max_age:
                self.depth_pending.popleft()
                self.warn_stale('depth frame', age, frame)
                continue
            if not self.tf.can_transform(self.odom, self.base, stamp):
                return
            self.depth_pending.popleft()
            cam = self.camera_pose(frame)
            if cam is None:
                continue
            try:
                base = self.tf.lookup_transform(self.odom, self.base, stamp)
            except TransformException:
                continue
            rb, tb = tf_matrix(base)
            r, t = rb @ cam[0], rb @ cam[1] + tb
            pts = depth_points(d, self.k, r, t, stride=int(self.depth_cfg['depth_stride']),
                               max_depth=float(self.depth_cfg['depth_max']))
            self.depth_history.append((pts, t))

    # ------------------------------------------------------------------ detection
    def looking(self):
        if self.state != 'IDLE' or time.monotonic() < self.cooldown_until:
            return False
        return not self.status_topic or self.explorer in self.trigger_states

    def detection_points(self):
        """(points in odom, origin xyz) from the sensors in use, or (None, None). The origin
        (bins by bearing, ranges) is the lidar when it is in use, else the camera."""
        parts, origin = [], None
        if self.use_lidar and self.history:
            parts += [h[0] for h in self.history]
            origin = self.history[-1][1]
        if self.use_depth and self.depth_history:
            parts += [h[0] for h in self.depth_history]
            if origin is None:
                origin = self.depth_history[-1][1]
        if not parts:
            return None, None
        return np.concatenate(parts), origin

    def detect(self):
        if not (self.history or self.depth_history):
            return
        pose = self.pose()
        if pose is None:
            return
        xy, yaw, _ = pose
        # Lying: the floor under the feet from just before lying down (it isn't updated while
        # lying; an inspection lies ~10-20 s). Standing: only a fresh one.
        floor = self.floor_here(60.0 if self.state == 'LYING' else 3.0)
        if floor is None:
            return
        if self.state == 'LYING':
            return self.measure_gap(floor)
        if self.state == 'INSPECTING' or self.state == 'IDLE' and (
                not self.status_topic or self.explorer in
                self.trigger_states + self.detect_states + ('SCANNING', 'CREEPING')):
            t = self.frame_tf()
            if t is None:
                self.get_logger().warn(f'no TF {self.cframe} -> {self.odom} (SLAM): no curtain '
                                       'detection', throttle_duration_sec=10.0)
            else:
                pts, origin = self.detection_points()
                cells = []
                found_odom = [] if pts is None else detect_curtains(
                    pts, origin[:2], yaw, floor, self.params, cells_out=cells)
                found = [c.transformed(*t) for c in found_odom]     # odom -> self.cframe
                c0, s0 = math.cos(t[2]), math.sin(t[2])
                fxy = (t[0] + c0 * xy[0] - s0 * xy[1], t[1] + s0 * xy[0] + c0 * xy[1])
                new = self.tracker.add(time.monotonic(), found, fxy)
                self.log_near_miss(found, cells, new, fxy)
                for c in new:
                    self.get_logger().info(
                        f'curtain {c.id} found: {c.width:.2f} m wide, gap {c.gap * 100:.0f} cm, '
                        f'centre {self.cframe} ({c.centre[0]:.2f}, {c.centre[1]:.2f}), '
                        f'{c.distance(fxy):.2f} m away; cleared from the explorer\'s map (from '
                        'the wall scan once walked through)')
                    self.publish_curtains()
                if self.inspect_on and self.cfg['drive'] and not new and self.state == 'IDLE':
                    self.inspect_tick(found_odom, cells, xy)
        if self.looking() and self.cfg['drive']:
            self.maybe_trigger(xy, yaw)

    def log_near_miss(self, found, cells, new, xy):
        """Every diag_period s at most: why the wall ahead isn't (yet) a curtain."""
        now = time.monotonic()
        if self.diag_period <= 0 or now - self.diag_last < self.diag_period or new:
            return
        msg = None
        for c in found:
            hits, confirmed = self.tracker.votes(c)
            if not confirmed:
                msg = (f'curtain candidate {c.width:.2f} m wide, gap {c.gap * 100:.0f} cm, '
                       f'{c.distance(xy):.2f} m away: {hits}/{self.tracker.need} votes in '
                       f'{self.tracker.window:.0f} s')
                break
        if msg is None and not found:
            msg = diagnose(cells, self.params)
        if msg:
            self.diag_last = now
            self.get_logger().info(f'[{self.explorer or "-"}] no curtain: {msg}'
                                   if not found else f'[{self.explorer or "-"}] {msg}')

    def publish_curtains(self):
        self.curtains_pub.publish(String(data=curtains_to_json(self.tracker.confirmed,
                                                               self.cframe)))

    def on_hold(self, msg):
        hold = CurtainHold.from_json(msg.data)
        if (hold.hold_new, sorted(hold.ids)) != (self.hold.hold_new, sorted(self.hold.ids)):
            self.get_logger().info(f'maze_explorer hold: new curtains '
                                   f'{"held" if hold.hold_new else "walked through"}, '
                                   f'held ids {sorted(hold.ids)}')
        self.hold = hold

    def after_confirm(self, cur, why):
        """A curtain an inspection confirmed: walk through, unless maze_explorer holds new
        curtains (at a junction: its other ways first) -> CONFIRMED, not walked through."""
        if not self.hold.hold_new:
            self.get_logger().info(f'{why}: walking through')
            return self.start_manoeuvre()
        cid = self.lock_id
        self.lock, self.lock_id = None, None
        self.cooldown_until = time.monotonic() + 3.0
        self.set_state('CONFIRMED', f'curtain {cid} confirmed ({why}): maze_explorer explores '
                       'the other junction ways first')

    def maybe_trigger(self, xy, yaw):
        c = self.cfg
        for cur in self.curtains_odom():
            side = cur.side_of(xy)
            if side in cur.passed or cur.id in self.hold.ids:
                continue
            if cur.passed and self.status_topic:     # crossed once: only back for a goal
                goal = self.goal_odom()
                if goal is None or not (cur.signed(goal) * side > 0 or
                                        cur.distance(goal) < c['regoal_radius']):
                    continue
            dist = abs(cur.signed(xy))
            lat = abs(cur.lateral(xy))
            bearing = abs(wrap(math.atan2(*(cur.centre - np.asarray(xy))[::-1]) - yaw))
            back = bool(cur.passed)                  # crossing back: looser limits
            if dist > c['trigger_distance'] or dist < 0.1 or \
                    lat > c['back_max_offset' if back else 'trigger_max_offset'] or \
                    bearing > c['back_max_bearing' if back else 'trigger_max_bearing']:
                continue
            self.lock, self.lock_id, self.side = cur, cur.id, side
            self.set_state('HANDOVER', f'curtain {cur.id} {dist:.2f} m ahead, '
                           f'{lat * 100:.0f} cm off its centre line: asking maze_explorer to '
                           'stop Nav2')
            return

    # ------------------------------------------------------------------ inspection
    def inspect_tick(self, found_odom, cells, xy):
        """Candidates only from the explorer's looks (`look_states`, user 2026-11-21). Dead-end
        stops come first (user 2026-11-20): candidates seen in DEAD_END are remembered
        (`deferred`) and inspected once maze_explorer is back in one of trigger_states (within
        20 s, still in range). In a JUNCTION look the inspection comes first: maze_explorer
        pauses the look for it. Outside the looks only those deferred candidates count;
        candidates seen while exploring are logged and `remembered` (`inspect_remember` s) for
        the next look, where the usual distance / bearing limits decide (user 2026-11-21)."""
        now = time.monotonic()
        cands = [(c.p1, c.p2, c.gap, 'unconfirmed curtain') for c in found_odom]
        seg = near_miss_segment(cells, self.params, self.inspect_cfg['inspect_max_gap'],
                                self.inspect_cfg['inspect_min_width'])
        if seg is not None:
            cands.append((*seg, 'wall with a gap under it'))
        # At a junction the lie-down check comes first (user 2026-11-20): maze_explorer
        # pauses the junction look for it. Dead-end and start scans still come first.
        in_look = not self.look_states or self.explorer in self.look_states
        keep = self.inspect_cfg['inspect_remember']
        self.remembered = [r for r in self.remembered if now - r[4] < keep]
        if self.status_topic and not in_look and cands and \
                self.explorer in self.trigger_states:
            self.remember(cands, xy, now)
            cands = []
        if in_look and self.remembered:
            cands = cands + [(*r[:3], r[3] + f', seen {now - r[4]:.0f} s ago while exploring')
                             for r in self.remembered]
        if self.status_topic and self.explorer not in self.trigger_states + ('JUNCTION',):
            if cands and in_look:
                self.deferred = (cands, now)
            return
        if self.status_topic and not in_look:
            cands = []
        if self.deferred is not None:
            if now - self.deferred[1] < 20.0:
                cands += [(*c[:3], c[3] + ', seen during the look') for c in self.deferred[0]]
            self.deferred = None
        if now >= self.cooldown_until:
            self.maybe_inspect(cands, xy)

    def remember(self, cands, xy, now):
        """Possible curtains seen outside a look: keep them for the next look (one per spot,
        newest wins) and log what they were."""
        pose = self.pose()
        yaw = pose[1] if pose is not None else None
        rxy = np.asarray(xy, float)
        for p1, p2, gap, what in cands:
            p1, p2 = np.asarray(p1, float), np.asarray(p2, float)
            centre = (p1 + p2) / 2
            self.remembered = [r for r in self.remembered
                               if np.linalg.norm((np.asarray(r[0]) + np.asarray(r[1])) / 2
                                                 - centre) > 0.5]
            self.remembered.append((p1, p2, gap, what, now))
            d = centre - rxy
            bearing = '' if yaw is None else \
                f', bearing {math.degrees(wrap(math.atan2(d[1], d[0]) - yaw)):+.0f} deg'
            self.get_logger().info(
                f'possible curtain ({what}: {np.linalg.norm(p2 - p1):.2f} m wide, gap '
                f'{gap * 100:.0f} cm, {np.linalg.norm(d):.2f} m away{bearing}) while '
                f'{self.explorer}: kept {self.inspect_cfg["inspect_remember"]:.0f} s for the '
                'next junction / dead-end look', throttle_duration_sec=5.0)

    def maybe_inspect(self, cands, xy):
        """A wall ahead with a suitable gap but no confirmed curtain -> HANDOVER to inspect.
        Candidates: single unconfirmed detections and near misses (near_miss_segment), as
        (p1, p2, gap, what) in odom."""
        now = time.monotonic()
        if self.state != 'IDLE':
            return
        self.rejected = [r for r in self.rejected
                         if now - r[1] < self.inspect_cfg['inspect_forget']]
        known = self.curtains_odom()
        rxy = np.asarray(xy, float)
        for p1, p2, gap, what in cands:
            p1, p2 = np.asarray(p1, float), np.asarray(p2, float)
            centre = (p1 + p2) / 2
            d = p2 - p1
            n = np.array([d[1], -d[0]]) / max(np.linalg.norm(d), 1e-9)
            if np.dot(centre - rxy, n) < 0:
                n = -n                                   # away from the robot
            dist = float(np.dot(centre - rxy, n))
            if dist > self.inspect_cfg['inspect_trigger_distance'] or \
                    dist < self.inspect_cfg['inspect_min_distance']:
                continue
            pose = self.pose()
            yaw = pose[1] if pose is not None else None
            if yaw is not None and (
                    abs(wrap(math.atan2(*(centre - rxy)[::-1]) - yaw)) >
                    self.inspect_cfg['inspect_max_bearing'] or
                    abs(wrap(math.atan2(n[1], n[0]) - yaw)) > math.radians(45)):
                continue        # off to the side, or a wall along the way (extru side windows)
            if any(np.linalg.norm(centre - r[0]) < self.inspect_cfg['inspect_forget_radius']
                   for r in self.rejected):
                continue
            if any(np.linalg.norm(centre - k.centre) < 0.5 for k in known):
                continue                                 # already a curtain
            if self.behind_wall(rxy, centre):
                self.get_logger().info(
                    f'possible curtain ({what}, {dist:.2f} m ahead) is behind a wall on the '
                    'map: not reachable from here, not inspected', throttle_duration_sec=5.0)
                continue
            width = float(np.linalg.norm(d))
            self.inspect_target = {'centre': centre, 'n': n, 'p1': p1, 'p2': p2,
                                   'what': f'{what}: {width:.2f} m wide, gap '
                                   f'{gap * 100:.0f} cm, {dist:.2f} m ahead'}
            self.lock, self.lock_id = None, None
            self.set_state('HANDOVER', f'possible curtain ({self.inspect_target["what"]}): '
                           'asking maze_explorer to stop Nav2 so it can be inspected')
            return

    def behind_wall(self, robot_odom, target_odom):
        """A /map wall between the robot and a candidate (curtains.wall_between), both in
        odom; False without a map or TF (then inspect as before)."""
        if self.grid is None:
            return False
        try:
            tf = self.tf.lookup_transform(self.map_frame, self.odom, Time())
        except TransformException:
            return False
        ox, oy, oth = tf.transform.translation.x, tf.transform.translation.y, tf_yaw(tf)
        c, s = math.cos(oth), math.sin(oth)
        to_map = lambda p: (ox + c * p[0] - s * p[1], oy + s * p[0] + c * p[1])  # noqa: E731
        grid = clear_curtains(self.grid, self.tracker.confirmed if self.cframe == self.map_frame
                              else [k.transformed(ox, oy, oth) for k in self.tracker.confirmed])
        return wall_between(grid, to_map(robot_odom), to_map(target_odom))

    def start_inspection(self):
        tg = self.inspect_target
        self.inspection = InspectPlan(tg['centre'], tg['n'], time.monotonic(),
                                      self.inspect_params)
        self.inspect_done_at = None
        self.gaps = []
        look = (f'lie down and measure the gap ({self.inspect_cfg["gap_min"] * 100:.0f}-'
                f'{self.inspect_cfg["gap_max"] * 100:.0f} cm = curtain)'
                if self.inspect_params.look_mode == 'lie' else
                f'Pose-mode look (pitch {math.degrees(self.inspect_params.pitch):.0f} deg down)')
        self.set_state('INSPECTING', f'{tg["what"]}: to the middle of the cell in front '
                       f'({self.inspect_params.distance:.1f} m from it), square up, {look}')

    def measure_gap(self, floor):
        """LYING: the gap under the candidate from the newest points (gap_at_line)."""
        if self.inspection is None or self.inspection.phase != 'LYING' or \
                time.monotonic() - self.inspection.since < 0.7:   # older clouds still standing
            return
        pts, _ = self.detection_points()
        if pts is None:
            return
        tg = self.inspect_target
        gap, cov, _ = gap_at_line(pts, tg['p1'], tg['p2'], tg['n'], floor)
        if gap is not None:
            self.gaps.append((gap, cov))
            self.get_logger().info(f'lying: gap {gap * 100:.0f} cm under '
                                   f'{cov:.0%} of the curtain', throttle_duration_sec=1.0)

    def send_lie(self, want, now):
        """/maze/lie at 2 Hz while lying is wanted (True) or standing up (False)."""
        if self.lie_pub is None or want is None or now - self.lie_sent_at < 0.5:
            return
        self.lie_pub.publish(Bool(data=bool(want)))
        self.lie_sent_at = now

    def send_pose(self, pose):
        """Pose-mode target (yaw, pitch) to cmd_vel_bridge; None = release (NaN once)."""
        if self.pose_yaw_pub is None:
            return
        if pose is None:
            if self.pose_live:
                self.pose_yaw_pub.publish(Float32(data=float('nan')))
                if self.pose_pitch_pub is not None:
                    self.pose_pitch_pub.publish(Float32(data=float('nan')))
            self.pose_live = False
            return
        self.pose_live = True
        self.pose_yaw_pub.publish(Float32(data=float(pose[0])))
        if self.pose_pitch_pub is not None:
            self.pose_pitch_pub.publish(Float32(data=float(pose[1])))

    def end_inspection(self, xy):
        """Plan done. Lay down: a gap of gap_min..gap_max measured -> confirmed, walk through.
        Otherwise (Pose look / turns): a curtain confirmed by the votes there -> walk through.
        Else the spot is left alone for inspect_forget s."""
        tg = self.inspect_target
        centre = tg['centre']
        why = self.inspection.result
        lay = self.inspection.lay
        self.inspection, self.inspect_target = None, None
        if why.startswith('STAND-UP NOT CONFIRMED'):
            return self.set_state('STUCK', f'{why}: check the robot, stand it up, then: '
                                  'ros2 topic pub --once /maze/start std_msgs/msg/Empty')
        if lay:
            return self.decide_lying(tg, xy, why)
        hit = [c for c in self.curtains_odom()
               if np.linalg.norm(c.centre - centre) < 0.6 and c.side_of(xy) not in c.passed]
        if hit:
            cur = min(hit, key=lambda c: np.linalg.norm(c.centre - centre))
            self.lock, self.lock_id, self.side = cur, cur.id, cur.side_of(xy)
            return self.after_confirm(cur, f'inspection ({why}): curtain {cur.id} confirmed, '
                                      f'{cur.width:.2f} m wide, gap {cur.gap * 100:.0f} cm')
        self.reject(centre, f'inspection ({why}): not a curtain')

    def reject(self, centre, why):
        self.rejected.append((np.asarray(centre, float), time.monotonic()))
        self.cooldown_until = time.monotonic() + 3.0
        self.set_state('IDLE', f'{why}; left alone for '
                       f'{self.inspect_cfg["inspect_forget"]:.0f} s')

    def decide_lying(self, tg, xy, why):
        """Gap measured lying down: gap_min..gap_max over >= half the width -> curtain."""
        lo, hi = self.inspect_cfg['gap_min'], self.inspect_cfg['gap_max']
        good = [(g, c) for g, c in self.gaps if c >= 0.5]
        if len(good) < 3:
            return self.reject(tg['centre'], f'lying down ({why}): gap seen in only '
                               f'{len(good)} of {len(self.gaps)} readings over >= half the '
                               'width: not a curtain')
        gap = float(np.median([g for g, _ in good]))
        if not lo <= gap <= hi:
            return self.reject(tg['centre'], f'lying down: gap {gap * 100:.0f} cm, not '
                               f'{lo * 100:.0f}-{hi * 100:.0f} cm: not a curtain')
        t = self.frame_tf()
        if t is None:
            return self.reject(tg['centre'], 'no SLAM TF to store the curtain')
        c = Curtain(np.asarray(tg['p1'], float), np.asarray(tg['p2'], float),
                    np.asarray(tg['n'], float), gap)
        stored = self.tracker.confirm(c.transformed(*t), time.monotonic())
        self.publish_curtains()
        odom = [k for k in self.curtains_odom() if k.id == stored.id]
        cur = odom[0] if odom else c
        self.lock, self.lock_id, self.side = cur, stored.id, cur.side_of(xy)
        return self.after_confirm(cur, f'lying down: gap {gap * 100:.0f} cm over {len(good)} '
                                  f'readings: curtain {stored.id} confirmed '
                                  f'({cur.width:.2f} m wide)')

    def inspect_control(self, now):
        """INSPECTING, at control_rate: run the plan; after it, wait for Pose mode to end."""
        if self.status_topic and self.explorer != 'CURTAIN':
            self.send_pose(None)
            if self.lie_pub is not None:
                self.lie_pub.publish(Bool(data=False))           # stand up (bridge blocks
            self.inspection, self.inspect_target = None, None    # driving until it is up)
            self.cooldown_until = now + 5.0
            return self.set_state('IDLE', f'inspection stopped: maze_explorer is '
                                  f'{self.explorer}')
        pose = self.pose()
        if pose is None:
            self.cmd_pub.publish(Twist())
            return
        xy, yaw, _ = pose
        if self.inspect_done_at is not None:             # let cmd_vel_bridge leave Pose mode
            st = str(self.pose_status or '')
            if st.split(':')[0] in ('ENTERING', 'POSING', 'LEAVING') and \
                    now - self.inspect_done_at < 6.0:
                self.send_pose(None)
                return
            return self.end_inspection(xy)
        step = self.inspection.tick(now, xy, yaw, self.pose_status, self.lie_status)
        if step.done:
            self.stop_robot()
            self.send_pose(None)
            self.inspect_done_at = now
            if self.state == 'LYING':
                self.set_state('INSPECTING', f'standing again ({step.why})')
            return
        # Lying down / lying / standing up: its own state, which maze_explorer doesn't
        # pre-empt (a junction look needs a standing robot in auto mode).
        lying = step.phase in ('LIE_DOWN', 'LYING', 'STAND_UP')
        if lying and self.state != 'LYING':
            self.set_state('LYING', f'{step.phase.lower()}: measuring the gap from low down')
        elif not lying and self.state == 'LYING':
            self.set_state('INSPECTING', 'standing again')
        self.send_lie(step.lie, now)
        self.send_pose(step.pose)
        v, w = step.v, step.w
        if v or w:
            walls = self.walls()
            if walls is not None:
                v, w, _ = walls.safe(xy, yaw, v, w)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------ driving
    def walls(self):
        """WallCheck on /map with the curtains cleared, in odom, or None."""
        if not self.cfg['wall_check'] or self.grid is None:
            return None
        try:
            tf = self.tf.lookup_transform(self.map_frame, self.odom, Time())
        except TransformException:
            return None
        ox, oy, oth = tf.transform.translation.x, tf.transform.translation.y, tf_yaw(tf)
        curtains = self.tracker.confirmed if self.cframe == self.map_frame else \
            [c.transformed(ox, oy, oth) for c in self.tracker.confirmed]
        grid = clear_curtains(self.grid, curtains)
        return WallCheck(grid, self.box, margin=self.cfg['turn_margin'],
                         min_margin=self.cfg['min_margin'], offset=(ox, oy, oth))

    def start_manoeuvre(self):
        self.started_at = self.last_pose_at = time.monotonic()
        self.steps, self.last_step = 0, None
        pose = self.pose()
        if pose is None:
            return self.abort('no TF odom -> base_link')
        xy, yaw, _ = pose
        fresh = [c for c in self.curtains_odom() if c.id == self.lock_id]
        if fresh:                       # latest map -> odom; fixed for the manoeuvre
            self.lock = fresh[0]
        target = self.lock.centre
        heading = math.atan2(target[1] - xy[1], target[0] - xy[0])
        if abs(wrap(heading - yaw)) > self.cfg['align_tol']:
            self.plan = {'kind': 'turn', 'target': heading}
            self.set_state('FACING', f'turning {math.degrees(wrap(heading - yaw)):+.0f} deg to '
                           f'face curtain {self.lock.id}\'s middle')
        else:
            self.begin_approach()

    def begin_approach(self):
        v = self.cfg['approach_ratio'] * self.cfg['max_speed']
        self.plan = {'kind': 'approach', 'v': v}
        self.set_state('APPROACHING', f'{v:.3f} m/s ({self.cfg["approach_ratio"]:.0%} of '
                       f'{self.cfg["max_speed"]:.2f}) at the middle of curtain {self.lock.id}, '
                       f'through until {self.cfg["through_distance"] * 100:.0f} cm past it')

    def wiggle_plan(self, xy, yaw):
        """Turns +a, -2a, +a, each side cut to what the walls on /map allow."""
        a = abs(self.cfg['wiggle_angle'])
        walls = self.walls()
        left = right = a
        if walls is not None:
            mxy, myaw = walls.to_map(xy, yaw)
            for m in (self.cfg['turn_margin'], self.cfg['min_margin']):
                left = max_turn(walls.grid, mxy, myaw, self.box, +1, a, m)
                right = max_turn(walls.grid, mxy, myaw, self.box, -1, a, m)
                if left > math.radians(5) or right > math.radians(5):
                    break
        note = '' if left >= a - 1e-6 and right >= a - 1e-6 else \
            f' (walls: {math.degrees(left):.0f} left, {math.degrees(right):.0f} right)'
        targets = [yaw + left, yaw - right, yaw]
        return targets, note

    def begin_wiggle(self, xy, yaw, why):
        targets, note = self.wiggle_plan(xy, yaw)
        self.plan = {'kind': 'wiggle', 'targets': targets}
        self.last_step = 'wiggle'
        a = math.degrees(self.cfg['wiggle_angle'])
        self.set_state('WIGGLING', f'{why}: {a:.0f} deg left, {2 * a:.0f} right, {a:.0f} left'
                       + note)

    def begin_creep(self, xy, yaw):
        d = self.cfg['creep_step']
        walls = self.walls()
        if walls is not None:
            mxy, myaw = walls.to_map(xy, yaw)
            d = free_distance_ahead(walls.grid, mxy, myaw, self.box, d, self.cfg['min_margin'])
        self.last_step = 'creep'
        if d < 0.01:
            return self.begin_wiggle(xy, yaw, 'still blocked, no room to creep ahead')
        self.plan = {'kind': 'creep', 'from': xy, 'd': d,
                     'until': time.monotonic() + d / self.cfg['creep_speed'] + 3.0}
        self.set_state('CREEPING', f'sensors still blocked: {d * 100:.0f} cm ahead')

    def begin_check(self):
        self.stop_robot()
        self.plan = {'kind': 'check', 'from': time.monotonic()}
        self.set_state('CHECKING', f'holding {self.cfg["settle_time"]:.1f} s, then checking the '
                       'lidar and depth camera')

    def blocked(self, since):
        """(blocked, why) from sensor data newer than `since`, or (None, why) if none yet."""
        out, why = False, []
        lid = self.lidar_block
        if lid is None or lid[0] < since:
            return None, 'waiting for a lidar cloud'
        if lid[1]:
            out = True
        why.append(f'lidar {"BLOCKED" if lid[1] else "clear"} ({lid[2]})')
        dep = self.depth_block
        if dep is not None and dep[0] >= since:
            out = out or dep[1]
            why.append(f'depth {"BLOCKED" if dep[1] else "clear"} ({dep[2]})')
        elif dep is not None and time.monotonic() - dep[0] < 3.0:
            return None, 'waiting for a depth frame'
        return out, ', '.join(why)

    def turn_cmd(self, err):
        c = self.cfg
        w = max(c['min_turn_rate'], min(c['max_turn_rate'], 1.5 * abs(err)))
        return math.copysign(w, err)

    def control(self):
        now = time.monotonic()
        if self.state == 'HANDOVER':
            if not self.status_topic:
                return self.start_inspection() if self.inspect_target is not None else \
                    self.start_manoeuvre()
            if now - self.since > self.cfg['handover_timeout']:
                self.cooldown_until = now + 10.0
                self.lock, self.inspect_target = None, None
                self.set_state('IDLE', f'maze_explorer did not stop Nav2 within '
                               f'{self.cfg["handover_timeout"]:.0f} s (it is {self.explorer})')
            return
        if self.state in ('PASSED', 'STUCK', 'ABORTED', 'CONFIRMED'):
            if not self.status_topic or self.explorer not in ('CURTAIN', 'AT_CURTAIN'):
                self.lock = None
                self.set_state('IDLE', f'maze_explorer is {self.explorer}; looking again')
            return
        if self.state in ('INSPECTING', 'LYING'):
            return self.inspect_control(now)
        if self.state not in DRIVING:
            return
        if self.status_topic and self.explorer != 'CURTAIN':
            if self.state in ('FACING', 'APPROACHING') and \
                    self.explorer in ('JUNCTION', 'DEAD_END', 'EXPLORING'):
                return self.yield_to_explorer()
            return self.abort(f'maze_explorer is {self.explorer}, not CURTAIN')
        if now - self.started_at > self.cfg['manoeuvre_timeout']:
            return self.stuck(f'not through after {self.cfg["manoeuvre_timeout"]:.0f} s')
        pose = self.pose()
        if pose is None:
            self.cmd_pub.publish(Twist())
            if now - self.last_pose_at > 2.0:
                self.abort('no TF odom -> base_link for 2 s')
            return
        self.last_pose_at = now
        xy, yaw, _ = pose
        v, w = self.step(now, xy, yaw)
        if self.state not in DRIVING or (v == 0.0 and w == 0.0):
            return
        walls = self.walls()
        if walls is not None:
            v2, w2, why = walls.safe(xy, yaw, v, w)
            if (v2, w2) != (v, w) and v2 == 0.0 and w2 == 0.0:
                return self.stuck(f'{self.state.lower()}: {why}')
            v, w = v2, w2
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def step(self, now, xy, yaw):
        """(v, w) for this tick; moves the plan on."""
        c, plan, cur = self.cfg, self.plan, self.lock
        kind = plan['kind']
        if kind == 'turn':
            err = wrap(plan['target'] - yaw)
            if abs(err) < c['turn_tol']:
                self.stop_robot()
                self.begin_approach()
                return 0.0, 0.0
            return 0.0, self.turn_cmd(err)
        if kind == 'approach':
            past = cur.signed(xy) * self.side
            if past >= c['through_distance']:
                self.stop_robot()
                self.begin_wiggle(xy, yaw, f'{past * 100:.0f} cm past curtain {cur.id}')
                return 0.0, 0.0
            if self.state == 'APPROACHING' and past >= -self.box[1]:
                self.set_state('THROUGH', f'front at curtain {cur.id}: walking straight through')
            # Aim at a point beyond the middle on the curtain's normal: crosses at the middle.
            aim = cur.centre + self.side * cur.n * 0.6
            err = wrap(math.atan2(aim[1] - xy[1], aim[0] - xy[0]) - yaw)
            w = max(-c['max_turn_rate'], min(c['max_turn_rate'], 1.2 * err))
            return plan['v'] * max(0.0, math.cos(err)), w
        if kind == 'wiggle':
            err = wrap(plan['targets'][0] - yaw)
            if abs(err) < c['turn_tol']:
                plan['targets'].pop(0)
                if not plan['targets']:
                    self.begin_check()
                return 0.0, 0.0
            return 0.0, self.turn_cmd(err)
        if kind == 'creep':
            moved = math.hypot(xy[0] - plan['from'][0], xy[1] - plan['from'][1])
            if moved >= plan['d'] or now > plan['until']:
                self.begin_check()
                return 0.0, 0.0
            return c['creep_speed'], 0.0
        if kind == 'check':
            if now - plan['from'] < c['settle_time']:
                return 0.0, 0.0
            blocked, why = self.blocked(plan['from'] + c['settle_time'] * 0.5)
            if blocked is None:
                if now - plan['from'] > c['settle_time'] + 3.0:
                    blocked, why = False, f'no fresh sensor data ({why}); going on'
                else:
                    return 0.0, 0.0
            if not blocked:
                self.mark(self.side, crossed=True)
                self.set_state('PASSED', f'through curtain {cur.id}; sensors clear: {why}')
                return 0.0, 0.0
            if self.steps >= c['max_unblock_steps']:
                return self.stuck(f'sensors still blocked after {self.steps} creeps/wiggles: '
                                  f'{why}') or (0.0, 0.0)
            self.steps += 1
            self.get_logger().warn(f'sensors blocked ({why}); step {self.steps}/'
                                   f'{c["max_unblock_steps"]}')
            if self.last_step == 'wiggle':
                self.begin_creep(xy, yaw)
            else:
                self.begin_wiggle(xy, yaw, 'sensors still blocked')
            return 0.0, 0.0
        return 0.0, 0.0

    def stuck(self, why):
        self.stop_robot()
        pose = self.pose()
        if self.lock is not None and pose is not None and self.side not in self.lock.passed \
                and self.lock.signed(pose[0]) * self.side > 0:
            self.mark(self.side, crossed=True)     # it is through, just blind
        self.set_state('STUCK', f'{why} - lift the curtain off the sensors / move the robot, '
                       'then: ros2 topic pub --once /maze/start std_msgs/msg/Empty')

    def yield_to_explorer(self):
        """maze_explorer took the robot back for a junction or dead end before the front
        reached the fabric (junctions and scans first, user 2026-11-20): stop, IDLE, and
        don't mark the curtain passed, so it is crossed afterwards."""
        self.stop_robot()
        self.lock, self.lock_id = None, None
        self.cooldown_until = time.monotonic() + 5.0
        self.set_state('IDLE', f'maze_explorer took over ({self.explorer}): curtain later')

    def abort(self, why):
        self.stop_robot()
        if self.lock is not None and self.side not in self.lock.passed:
            pose = self.pose()                     # don't loop on it; crossed if past it
            self.mark(self.side, crossed=pose is not None and
                      self.lock.signed(pose[0]) * self.side > 0)
        self.cooldown_until = time.monotonic() + 5.0
        self.set_state('ABORTED', why)

    def stop_robot(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = CurtainDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.state in DRIVING and rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
