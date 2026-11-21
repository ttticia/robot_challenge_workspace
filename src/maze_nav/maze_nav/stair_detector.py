"""stair_detector: find stairs (depth camera + lidar), then face them square and stop in front.

Detection is in maze_nav/stairs.py (risers in the depth image) and maze_nav/stairs_lidar.py
(an odom height map from /livox/points; see their docs). This node:

  0. LIDAR (lidar_topic, '' = off): every scan goes into the height map (TF odom at the scan's
     stamp), always, so the map remembers what was seen. While looking, at `lidar_rate` Hz:
     floor height from the map, lidar-only stairs into their own vote tracker. Each camera
     estimate is checked against the map: confirmed, VETOED (map well covered and flat
     there: a camera false hit), or unknown. Fusion (stairs_lidar.fuse): the camera's first
     riser unless the lidar has one rising from the measured floor 0.1-0.5 m nearer (the
     camera's bottom step out of view); lidar-only stairs anchored on the floor count too
     (`lidar_alone_trigger`), e.g. off to the side or with the camera blocked. The lidar
     sees the floor only from ~1.4 m ahead of base_link: close up the camera decides.

  1. LOOKING (status IDLE): runs the detector at `rate` Hz on the newest depth frame while
     maze_explorer is EXPLORING, TURNING, LOOKING or FACING_QR (`trigger_states`). Each hit
     becomes a stair estimate in odom. Stairs count as found once `vote_need` of the last `vote_window`
     frames agree (StairTracker).
  2. Found within `trigger_distance` m, with the robot IN FRONT of them (within
     `trigger_max_offset` of their centre line; seen from a neighbouring cell it can be
     beside them) -> prints "stairs detected" (once per staircase) and status HANDOVER.
     maze_explorer cancels its Nav2 goal, stops and goes to STAIRS. Nothing moves until it
     has, so Nav2's
     velocity_smoother and this node never both drive nav_cmd_vel. No STAIRS within
     `handover_timeout` -> back to IDLE.
  2b. CHECKING (`check`, user 2026-11-20; stair_check.py): stair-like evidence that doesn't
     trigger 2. - a single camera frame with a riser chain, a lidar height-map estimate,
     stairs found but further than trigger_distance or seen from beside, anything seen
     while maze_explorer is in a junction / dead-end / start look (`detect_states`: it
     detects there now, but never hands over: stair checks come after junction checking) -
     is a CANDIDATE within `check_distance` m - but only while the explorer is in a junction
     look or dead-end stop (`look_states`, user 2026-11-21; candidates elsewhere are logged
     and dropped, and must be > `check_min_width` m wide). Once the explorer is back in trigger_states
     (candidate <= `check_max_age` s old): HANDOVER, then turn on the spot to face its first
     riser and hold still `check_hold` s, detecting: camera frames with a chain of discrete
     risers near it (and lidar estimates, half weight) are hits. Confirmed by the trackers,
     or `check_need` hits -> 3. (line up); else IDLE (explorer explores on) and the spot is
     left alone for `check_forget` s.
  3. ALIGNING: turn on the spot to face the stop point (on the stairs' centre line,
     `stop_distance` before the first riser). If the stairs' sides weren't both in view, it
     faces the stairs straight on instead and doesn't centre.
     APPROACHING: drive there at <= `max_speed`, steering, never backwards.
     SQUARING: turn on the spot until perpendicular to the step edges (`final_tol`),
     held `settle_time` s.
     Wall checks (`wall_check`, stairs.WallCheck on the SLAM /map via TF map -> odom, the
     explorer's footprint box): the stop pose is moved a few cm off any wall; each turn on
     the spot is checked over its whole sweep first (short way, long way, else creep up to
     0.3 m ahead for room, else hold); every command is checked `check_horizon` s ahead and
     cut to what stays clear (`turn_margin`, falling back to `min_margin`: the cell before
     the example maze's stairs leaves the turning circle 1 cm). No /map yet: no checks
     (warned). Squaring up blocked within 8 deg: good enough, noted.
     Commands go to nav_cmd_vel, so cmd_vel_bridge still clamps them, runs its sensor
     watchdog and honours drive:=false. The estimate is driven in odom (no SLAM jumps). It
     is refined by new detections, then locked inside `freeze_distance`: close up the
     camera can't see the bottom riser, so a later frame could take the second riser for
     the first.
  4. AT_STAIRS: zero velocity. maze_explorer holds (status AT_STAIRS) until /maze/start,
     so the operator can switch gait first. Also when the line-up is blocked by walls or
     takes longer than `manoeuvre_timeout`: the robot is at the stairs either way, so it
     holds there, not lined up, and the reason says why (the operator lines it up by hand):
         ros2 topic pub --once /maze/start std_msgs/msg/Empty
     Stairs that were handled (or aborted) are not triggered again (`suppress_radius`).
  ABORTED: no TF for 2 s, or the explorer left STAIRS (stopped, max runtime). Zero
     velocity, and the explorer goes back to exploring.
  DESCENDING STAIRS (`descend`, presentation maze 2026-11-21: the exit is two steps down
     from a raised cell): stairs_lidar.detect_down on the height map, against the floor
     under the robot's feet. From the top the lidar can't see the treads (each nosing
     shadows the one below); it sees its own floor end at a line and lower ground past an
     unseen strip. Such an estimate has down=True, heading pointing DOWN the stairs and
     `along` at the top edge, so 1.-4. run unchanged: the robot stops stop_distance before
     the edge, square to it. AT_STAIRS then reads "descending stairs: ..." and
     cmd_vel_bridge walks ai_walk_distance_down (not ai_walk_distance) in AI mode. The
     camera detector only finds rising risers; descending stairs are lidar-only (anchored:
     the floor under the feet was measured). Seen from ~2 m to ~0.6 m when walking towards
     them (simulated scans, test_stairs_lidar.py); nothing is found standing still at
     the start, before the robot has walked over (and so seen) its own floor.
  TERRAIN (`terrain_topic`, `local_floor_topic`, `terrain_rate` Hz): /maze/terrain (JSON:
     level even / rough / obstacle / drop / unknown of the strip 0.25-1.2 m ahead, and
     on_rough for the ground under the robot) and /maze/local_floor_z (the floor under the
     feet, odom z; nothing while lying down).

Topics
  in : depth_topic (sensor_msgs/Image, 16UC1 mm or 32FC1 m), depth_camera_info_topic,
       lidar_topic (sensor_msgs/PointCloud2, /livox/points) + TF odom -> its frame,
       /maze/status (std_msgs/String, latched), TF odom -> base_link -> camera,
       /map (nav_msgs/OccupancyGrid, latched) + TF map -> odom for the wall checks
  out: /stairs/status    std_msgs/String latched: "STATE: why", STATE one of IDLE,
                         HANDOVER, CHECKING, ALIGNING, APPROACHING, SQUARING, AT_STAIRS,
                         ABORTED
       /stairs/detection std_msgs/String JSON per frame with stairs (distance, angle, rise,
                         risers, edge point and heading in odom, confirmed, handled,
                         source camera/lidar/both, fusion: what the lidar said)
       /stairs/pose      geometry_msgs/PoseStamped latched (odom): middle of the first
                         riser, pointing up the stairs (RViz)
       nav_cmd_vel       geometry_msgs/Twist, only during ALIGNING/APPROACHING/SQUARING

    ros2 run maze_nav stair_detector                                       # with maze_explorer
    ros2 run maze_nav stair_detector --ros-args -p "status_topic:=''"      # alone (no Nav2!)
    ros2 run maze_nav stair_detector --ros-args -p drive:=false            # detect + log only

status_topic:='' drops the explorer handshake: it drives as soon as it finds stairs, so only
run it like that with nothing else driving (no Nav2, hands off the controller's sticks).
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav import stairs_lidar
from maze_nav.stair_check import CheckParams, StairCheck
from maze_nav.core import Grid, cloud_xyz
from maze_nav.stairs import (Approach, StairParams, StairTracker, WallCheck, camera_from_link,
                             detect_stairs, in_front, level_transform, mount_disagreement,
                             quat_to_matrix, refine_lock, to_odom, wrap)

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
DRIVING = ('ALIGNING', 'APPROACHING', 'SQUARING')
CHECK_STATES = ('CHECKING',)        # possible stairs: turning to face them / holding still
PHASE_STATE = {'ALIGN': 'ALIGNING', 'MAKE_ROOM': 'ALIGNING', 'APPROACH': 'APPROACHING',
               'FINAL_ALIGN': 'SQUARING', 'DONE': 'AT_STAIRS', 'BLOCKED': 'AT_STAIRS'}
ENCODINGS = {'16UC1': np.uint16, 'mono16': np.uint16, '32FC1': np.float32}


def matrix_of(tf):
    q, a = tf.transform.rotation, tf.transform.translation
    return quat_to_matrix(q.x, q.y, q.z, q.w), np.array([a.x, a.y, a.z])


class StairDetector(Node):

    def __init__(self):
        super().__init__('stair_detector')
        p = self.declare_parameter
        p('depth_topic', '/camera/depth/image_raw')
        p('depth_camera_info_topic', '/camera/depth/camera_info')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('camera_link_frame', 'camera_link')  # camera pose from here + fixed optical rotation;
        #                                        '' = the depth image's own frame from TF
        p('status_topic', '/maze/status')      # '' = no explorer handshake (drives at once)
        p('trigger_states', ['EXPLORING', 'TURNING', 'LOOKING', 'FACING_QR'])
        p('cmd_topic', 'nav_cmd_vel')          # through cmd_vel_bridge
        p('drive', True)                       # false = detect and log only
        p('rate', 5.0)                         # Hz detection
        p('control_rate', 20.0)                # Hz velocity commands
        p('trigger_distance', 1.5)             # m to the first riser to take over
        p('trigger_max_offset', 0.35)          # m off the stairs' centre line to take over
        p('stop_distance', 0.45)               # m base_link -> first riser at the stop (front 0.18 m off)
        p('freeze_distance', 0.75)             # m; nearer, the estimate is no longer updated
        p('centre', True)                      # also line up on the stairs' centre line
        p('max_speed', 0.08)                   # m/s
        p('max_turn_rate', 0.35)               # rad/s
        p('min_turn_rate', 0.15)               # rad/s; slower turns may not move the legs
        p('align_tol_deg', 5.0)                # heading error before driving
        p('final_tol_deg', 2.5)                # perpendicular to the steps within this
        p('settle_time', 0.6)                  # s held within final_tol before AT_STAIRS
        p('vote_window', 6)
        p('vote_need', 3)
        p('vote_pos_tol', 0.08)                # m; one tread off (0.15) must not agree
        p('vote_nearer_need', 2)               # this many frames 0.1-0.5 m nearer than the
        #                                        largest group win (bottom riser missed = one
        #                                        tread late = front on the step); 0 = off
        p('infer_bottom', True)                # riser below the view, on evidence (stairs.py)
        p('vote_yaw_tol_deg', 8.0)
        p('handover_timeout', 3.0)             # s for the explorer to stop Nav2
        p('manoeuvre_timeout', 45.0)           # s for the whole line-up
        p('suppress_radius', 1.0)              # m; stairs handled once are not taken again
        p('standing_height', 0.35)             # base_link above the floor
        p('min_risers', 2)
        p('max_range', 2.5)                    # m ahead
        p('wall_check', True)                  # check the line-up against walls on /map
        p('map_topic', '/map')
        p('map_frame', 'map')
        p('footprint_back', 0.37)              # m; = maze_explorer / nav2_params.yaml
        p('footprint_front', 0.27)
        p('footprint_half_width', 0.23)
        p('turn_margin', 0.03)                 # m kept clear of walls (= maze_explorer)
        p('min_margin', 0.01)                  # fallback where turn_margin doesn't fit
        p('check_horizon', 0.5)                # s each command is checked ahead
        p('lidar_topic', '/livox/points')      # '' = camera only
        p('lidar_rate', 2.0)                   # Hz lidar stair detection on the height map
        # s between height-map scans while not looking (PAUSED, junction looks without the
        # stair check, inspections...): the map stays current at ~1/3 the cost. 0 = every scan.
        p('idle_lidar_period', 0.3)
        p('lidar_vote_window', 4)
        p('lidar_vote_need', 2)
        p('lidar_vote_pos_tol', 0.12)          # m; lidar risers are good to ~5-10 cm
        p('lidar_veto', True)                  # drop camera stairs the lidar map shows flat
        p('lidar_freeze_distance', 1.4)        # m; lidar-only estimates don't move the lock nearer
        p('lidar_stop_margin', 0.10)           # m further back when only the lidar saw the stairs
        p('lidar_alone_trigger', True)         # lidar-only stairs (anchored on the floor) may
        #                                        start a line-up (camera blind or looking away)
        # Descending stairs (presentation maze 2026-11-21: the exit is two steps DOWN from a
        # raised cell): stairs_lidar.detect_down against the floor under the robot's feet.
        # Lined up like rising stairs (stop_distance before the top edge); AT_STAIRS then
        # starts with "descending" so cmd_vel_bridge walks ai_walk_distance_down in AI mode.
        p('descend', True)
        # Terrain ahead + the floor under the feet from the height map (stairs_lidar.terrain /
        # floor_under), at terrain_rate Hz: cmd_vel_bridge (speed cap, height when lying down),
        # curtain_detector (gap when lying down), maze_explorer (no Pose looks on rough
        # ground). '' = not published.
        p('terrain_topic', '/maze/terrain')
        p('local_floor_topic', '/maze/local_floor_z')
        p('terrain_rate', 2.0)
        # Possible-stairs check (user 2026-11-20, stair_check.py): candidates also from single
        # frames, lidar, far / beside stairs and the explorer's looks; checked after them.
        p('check', True)
        p('detect_states', ['JUNCTION', 'DEAD_END', 'SCANNING'])   # detect, never hand over
        # Possible stairs are only taken (for a check) during these explorer looks (user
        # 2026-11-21): the junction look and the dead-end stop; checked right after the look.
        # Confirmed stairs still take over anywhere. [] = any time (the old behaviour).
        p('look_states', ['JUNCTION', 'DEAD_END'])
        p('check_distance', 2.5)               # m: candidates nearer than this get checked
        p('check_max_age', 20.0)               # s a candidate stays worth checking
        p('check_hold', 3.0)                   # s still, facing it
        p('check_need', 3)                     # camera hits (lidar half) during the hold
        p('check_hit_radius', 0.5)             # m: a hit's first riser this near the candidate
        p('check_forget', 120.0)               # s a rejected spot is left alone
        p('check_forget_radius', 0.8)          # m
        # m of step edge seen across (each step is >= 0.6 m long, user 2026-11-21) before a
        # possible-stairs check; the check's own hits and confirmed stairs (>= 2 risers,
        # tracked) don't need it
        p('check_min_width', 0.6)
        # Possible stairs need >= check_min_risers steps averaging > check_min_rise m (user
        # 2026-11-21: a single step-like obstacle was a false positive)
        p('check_min_risers', 2)
        p('check_min_rise', 0.08)
        # Pose-mode pitch sweep during the check's hold (user 2026-11-21): level, nose up
        # check_pitch_deg, nose down check_pitch_deg (cmd_vel_bridge /maze/pose_*), so the
        # camera sees the upper steps and the bottom riser. Unavailable -> the plain hold.
        p('check_pose', True)
        p('check_pitch_deg', 10.0)
        p('pose_yaw_topic', '/maze/pose_yaw')
        p('pose_pitch_topic', '/maze/pose_pitch')
        p('pose_status_topic', '/maze/pose_status')
        # Curtains are checked before stairs (user 2026-11-21): no possible-stairs HANDOVER
        # while curtain_detector is busy (/curtain/status not IDLE), nor within curtain_grace
        # s after the explorer leaves a look (its held curtain inspection goes first).
        p('curtain_status_topic', '/curtain/status')   # '' = don't wait for curtains
        p('curtain_grace', 2.0)
        # Possible stairs seen in a JUNCTION look (user 2026-11-21) are held, not checked after
        # the look: published on /stairs/candidate ({id, frame, edge, heading}); maze_explorer
        # explores the junction's other ways first, then comes back and releases them on
        # /maze/stairs_release ({id}). Stairs seen again near a held one join it.
        p('junction_defer', True)
        p('stairs_release_topic', '/maze/stairs_release')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.params = StairParams(standing_height=float(g('standing_height')),
                                  min_risers=int(g('min_risers')),
                                  max_range=float(g('max_range')),
                                  infer_bottom=bool(g('infer_bottom')))
        self.tracker = StairTracker(window=int(g('vote_window')), need=int(g('vote_need')),
                                    pos_tol=float(g('vote_pos_tol')),
                                    yaw_tol=math.radians(g('vote_yaw_tol_deg')),
                                    nearer_need=int(g('vote_nearer_need')))
        self.cfg = {n: g(n) for n in ('trigger_distance', 'trigger_max_offset',
                                      'stop_distance', 'freeze_distance',
                                      'centre', 'max_speed', 'max_turn_rate', 'min_turn_rate',
                                      'align_tol_deg', 'final_tol_deg', 'settle_time',
                                      'handover_timeout', 'manoeuvre_timeout',
                                      'suppress_radius', 'drive', 'lidar_freeze_distance',
                                      'lidar_stop_margin')}
        self.odom_frame, self.base_frame = g('odom_frame'), g('base_frame')
        self.wall_cfg = {n: g(n) for n in ('wall_check', 'map_frame', 'footprint_back',
                                           'footprint_front', 'footprint_half_width',
                                           'turn_margin', 'min_margin', 'check_horizon')}
        self.grid = None                # newest /map as core.Grid
        self.last_note = ''             # Approach.note last logged
        self.status_topic = g('status_topic')
        self.trigger_states = tuple(g('trigger_states'))
        self.lidar_cfg = {n: g(n) for n in ('lidar_topic', 'lidar_veto', 'lidar_alone_trigger',
                                            'descend')}
        self.hmap = stairs_lidar.HeightMap(stairs_lidar.LidarParams(
            standing_height=float(g('standing_height')))) if g('lidar_topic') else None
        self.lidar_tracker = StairTracker(window=int(g('lidar_vote_window')),
                                          need=int(g('lidar_vote_need')),
                                          pos_tol=float(g('lidar_vote_pos_tol')),
                                          yaw_tol=math.radians(g('vote_yaw_tol_deg')))
        self.clouds = []                # PointCloud2 waiting for TF
        self.floor_z = None             # odom z of the floor from the lidar map
        self.lidar_scans = 0
        self.idle_lidar_period = float(g('idle_lidar_period'))
        self.last_hmap_add = -1e9       # monotonic s of the last scan added to the height map

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(Image, g('depth_topic'), self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('depth_camera_info_topic'), self.on_info,
                                 qos_profile_sensor_data)
        if self.status_topic:
            self.create_subscription(String, self.status_topic, self.on_explorer, LATCHED)
        if self.wall_cfg['wall_check']:
            self.create_subscription(OccupancyGrid, g('map_topic'), self.on_map, LATCHED)
        if self.hmap is not None:
            self.create_subscription(PointCloud2, g('lidar_topic'), self.clouds.append,
                                     qos_profile_sensor_data)
            self.create_timer(0.1, self.add_clouds)
            self.create_timer(1.0 / float(g('lidar_rate')), self.process_lidar)
        self.status_pub = self.create_publisher(String, '/stairs/status', LATCHED)
        self.det_pub = self.create_publisher(String, '/stairs/detection', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/stairs/pose', LATCHED)
        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic'), 10)
        self.create_timer(1.0 / float(g('rate')), self.process)
        self.create_timer(1.0 / float(g('control_rate')), self.control)

        self.k = None                   # (fx, fy, cx, cy) of the depth image
        self.depth = None               # newest Image, processed at most once
        self.depth_new = False
        self.cam = None                 # (R, t) camera optical frame in base_link (static)
        self.explorer = None            # maze_explorer state, None = not heard yet
        self.state = None
        self.lock = None                # StairEstimate being driven to
        self.approach = None
        self.since = 0.0                # monotonic time of the last state change
        self.last_pose_at = None
        self.started_at = 0.0           # monotonic start of the manoeuvre
        self.handled = []               # StairEstimates done (or given up): never again
        self.announced = []             # StairEstimates "stairs detected" was printed for
        self.cooldown_until = 0.0       # after a refused handover
        self.stop_xy = None             # where AT_STAIRS was reached (standalone re-arm)
        self.check_on = bool(g('check'))
        self.detect_states = tuple(g('detect_states'))
        self.look_states = tuple(g('look_states'))
        self.check_cfg = {n: float(g(n)) for n in (
            'check_distance', 'check_max_age', 'check_hit_radius', 'check_forget',
            'check_forget_radius', 'check_min_width', 'check_min_risers', 'check_min_rise')}
        pitch = math.radians(float(g('check_pitch_deg')))
        self.check_params = CheckParams(hold=float(g('check_hold')), need=int(g('check_need')),
                                        turn_rate=float(g('max_turn_rate')),
                                        min_turn_rate=float(g('min_turn_rate')),
                                        pose=bool(g('check_pose')) and bool(g('pose_yaw_topic')),
                                        pitches=((0.0, 1.0), (-pitch, 1.5), (pitch, 1.5),
                                                 (0.0, 0.5)))
        self.pose_status = None         # cmd_vel_bridge /maze/pose_status
        self.pose_yaw_pub = self.create_publisher(Float32, g('pose_yaw_topic'), 10) \
            if g('pose_yaw_topic') else None
        self.pose_pitch_pub = self.create_publisher(Float32, g('pose_pitch_topic'), 10) \
            if g('pose_pitch_topic') else None
        if g('pose_status_topic'):
            self.create_subscription(String, g('pose_status_topic'), self.on_pose_status,
                                     LATCHED)
        self.curtain_status = None      # curtain_detector state, None = not heard
        self.curtain_grace = float(g('curtain_grace'))
        self.look_left_at = -1e9        # monotonic s the explorer last left a look state
        self.junction_defer = bool(g('junction_defer'))
        self.held = {}                  # id -> (StairEstimate, what): held until released
        self.next_held_id = 1
        self.cand_pub = self.create_publisher(String, '/stairs/candidate', 10)
        if g('stairs_release_topic'):
            self.create_subscription(String, g('stairs_release_topic'), self.on_release, 10)
        if g('curtain_status_topic'):
            self.create_subscription(String, g('curtain_status_topic'), self.on_curtain_status,
                                     LATCHED)
        self.candidate = None           # (StairEstimate odom, monotonic time, what)
        self.checking = False           # the pending HANDOVER is for a check
        self.check = None               # StairCheck while CHECKING
        self.rejected = []              # (xy odom, monotonic time): checked, not stairs
        self.create_timer(0.2, self.check_tick)
        self.terrain_pub = self.local_floor_pub = None
        if self.hmap is not None and g('terrain_topic'):
            self.terrain_pub = self.create_publisher(String, g('terrain_topic'), 10)
        if self.hmap is not None and g('local_floor_topic'):
            self.local_floor_pub = self.create_publisher(Float32, g('local_floor_topic'), 10)
        self.start_floor = None         # wall_scan's /maze/floor_z: terrain before floor_under
        if self.terrain_pub is not None or self.local_floor_pub is not None:
            self.create_timer(1.0 / float(g('terrain_rate')), self.terrain_tick)
            self.create_subscription(Float32, '/maze/floor_z',
                                     lambda m: setattr(self, 'start_floor', float(m.data)),
                                     LATCHED)
        self.set_state('IDLE', 'looking for stairs' + (
            f' while {self.status_topic} is {"/".join(self.trigger_states)}'
            if self.status_topic else ' (no explorer handshake: drives on its own)')
            + ('' if self.cfg['drive'] else '; drive:=false, detect only'))

    # ------------------------------------------------------------------ inputs
    def set_state(self, state, why=''):
        if state == self.state:
            return
        self.state = state
        self.since = time.monotonic()
        # One call site per severity: rclpy raises if a call site's severity changes.
        if state == 'ABORTED':
            self.get_logger().warn(f'[{state}] {why}')
        else:
            self.get_logger().info(f'[{state}] {why}')
        self.status_pub.publish(String(data=f'{state}: {why}' if why else state))

    def on_map(self, msg):
        self.grid = Grid.from_msg(msg)

    def walls(self):
        """WallCheck for this tick (map + latest TF map -> odom), or None: no checks."""
        c = self.wall_cfg
        if not c['wall_check'] or self.grid is None:
            return None
        try:
            tf = self.tf.lookup_transform(c['map_frame'], self.odom_frame, Time())
        except TransformException:
            return None
        r, a = matrix_of(tf)
        return WallCheck(self.grid, (c['footprint_back'], c['footprint_front'],
                                     c['footprint_half_width']),
                         margin=c['turn_margin'], min_margin=c['min_margin'],
                         offset=(a[0], a[1], math.atan2(r[1, 0], r[0, 0])),
                         horizon=c['check_horizon'])

    def on_info(self, msg):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg):
        self.depth, self.depth_new = msg, True

    def on_pose_status(self, msg):
        self.pose_status = msg.data

    def on_curtain_status(self, msg):
        self.curtain_status = msg.data.split(':', 1)[0].strip()

    def send_look(self, look):
        """StairCheck's look request -> /maze/pose_yaw + /maze/pose_pitch (float: keep the
        Pose look going at that pitch; 'end': NaN)."""
        if look is None or self.pose_yaw_pub is None:
            return
        yaw = float('nan') if look == 'end' else 0.0
        pitch = float('nan') if look == 'end' else float(look)
        self.pose_yaw_pub.publish(Float32(data=yaw))
        if self.pose_pitch_pub is not None:
            self.pose_pitch_pub.publish(Float32(data=pitch))

    def on_explorer(self, msg):
        prev = self.explorer
        self.explorer = msg.data.split(':', 1)[0].strip()
        if prev in self.look_states and self.explorer not in self.look_states:
            self.look_left_at = time.monotonic()
        if self.state == 'HANDOVER' and self.explorer == 'STAIRS':
            self.start_check() if self.checking else self.start_manoeuvre()

    def looking(self):
        if self.state in ('HANDOVER', 'ALIGNING', 'APPROACHING') + CHECK_STATES:
            return True                 # keep refining until freeze_distance
        if self.state != 'IDLE':
            return False
        return not self.status_topic or self.explorer in self.trigger_states + (
            self.detect_states if self.check_on else ())

    # ------------------------------------------------------------------ possible stairs
    def in_look(self):
        """maze_explorer is in a junction / dead-end / start look: detect, don't hand over."""
        return bool(self.status_topic) and self.explorer in self.detect_states

    def is_rejected(self, est, xy):
        now = time.monotonic()
        self.rejected = [r for r in self.rejected
                         if now - r[1] < self.check_cfg['check_forget']]
        pt = np.asarray(est.edge_point(np.asarray(xy, float)), float)
        return any(np.linalg.norm(pt - r[0]) < self.check_cfg['check_forget_radius']
                   for r in self.rejected)

    def note_candidate(self, est, xy, what):
        """Remember stair-like evidence to check (nearest within check_distance)."""
        if not self.check_on or est is None or self.is_handled(est) or \
                self.is_rejected(est, xy):
            return
        if self.state in CHECK_STATES and self.check is not None:
            near = np.linalg.norm(np.asarray(est.edge_point(np.asarray(xy, float))) -
                                  self.check.target) < self.check_cfg['check_hit_radius']
            if near:
                self.check.add_hit('lidar' if est.source == 'lidar' else 'camera')
            return
        d = abs(est.distance(np.asarray(xy, float)))
        if d > self.check_cfg['check_distance']:
            return
        if self.status_topic and self.look_states and self.explorer not in self.look_states:
            self.get_logger().info(f'possible stairs {d:.2f} m away ({what}) while '
                                   f'{self.explorer}: checked only from a junction or dead-end '
                                   'look', throttle_duration_sec=10.0)
            return
        if est.width < self.check_cfg['check_min_width']:
            self.get_logger().info(f'step edge {d:.2f} m away ({what}) only {est.width:.2f} m '
                                   f'wide < {self.check_cfg["check_min_width"]:.2f}: no '
                                   'stair check', throttle_duration_sec=5.0)
            return
        if est.risers < self.check_cfg['check_min_risers'] or \
                est.rise <= self.check_cfg['check_min_rise']:
            self.get_logger().info(f'step-like edge {d:.2f} m away ({what}): {est.risers} '
                                   f'step(s) of {est.rise * 100:.0f} cm; needs >= '
                                   f'{self.check_cfg["check_min_risers"]:.0f} steps over '
                                   f'{self.check_cfg["check_min_rise"] * 100:.0f} cm: no stair '
                                   'check', throttle_duration_sec=5.0)
            return
        if self.hold_candidate(est, xy, d, what):
            return
        now = time.monotonic()
        old = self.candidate
        if old is None or now - old[1] > 1.0 or d < abs(old[0].distance(np.asarray(xy))):
            if old is None or now - old[1] > self.check_cfg['check_max_age']:
                self.get_logger().info(f'possible stairs {d:.2f} m away ({what}); checked '
                                       + ('after the explorer\'s look' if self.in_look()
                                          else 'now'), throttle_duration_sec=3.0)
            self.candidate = (est, now, what)

    def hold_candidate(self, est, xy, d, what):
        """Possible stairs at a junction: hold them for maze_explorer (True = held)."""
        if not (self.junction_defer and self.status_topic):
            return False
        pt = np.asarray(est.edge_point(np.asarray(xy, float)), float)
        sid = self.held_id(est, xy)
        if sid is not None:
            self.held[sid] = (est, what)              # seen again: same hold
            return True
        if self.explorer != 'JUNCTION':
            return False
        sid, self.next_held_id = self.next_held_id, self.next_held_id + 1
        self.held[sid] = (est, what)
        self.cand_pub.publish(String(data=json.dumps({
            'id': sid, 'frame': self.odom_frame, 'edge': [float(pt[0]), float(pt[1])],
            'heading': float(est.heading), 'distance': round(float(d), 2), 'what': what})))
        self.get_logger().info(f'possible stairs {d:.2f} m away ({what}) at a junction: held '
                               f'(id {sid}) until its other ways are explored')
        return True

    def held_id(self, est, xy):
        """Id of the held stairs est is (first riser within check_forget_radius), or None."""
        pt = np.asarray(est.edge_point(np.asarray(xy, float)), float)
        for sid, (old, _) in self.held.items():
            if np.linalg.norm(np.asarray(old.edge_point(np.asarray(xy, float))) - pt) < \
                    self.check_cfg['check_forget_radius']:
                return sid
        return None

    def on_release(self, msg):
        """maze_explorer is back in front of held stairs: check them now."""
        try:
            sid = int(json.loads(msg.data)['id'])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn(f'bad {msg.data[:80]!r} on stairs_release_topic')
            return
        if sid not in self.held:
            return
        est, what = self.held.pop(sid)
        self.candidate = (est, time.monotonic(), f'{what}, held at a junction')
        self.cooldown_until = 0.0
        self.get_logger().info(f'held stairs {sid} released: checking them')

    def check_tick(self):
        """5 Hz: IDLE, the explorer exploring again, a fresh candidate -> HANDOVER to check."""
        now = time.monotonic()
        if not self.check_on or not self.cfg['drive'] or self.state != 'IDLE' or \
                self.candidate is None or now < self.cooldown_until:
            return
        if self.status_topic and self.explorer not in self.trigger_states:
            return                      # junction / dead-end / start looks come first
        if self.curtain_status not in (None, 'IDLE') or now - self.look_left_at < self.curtain_grace:
            return                      # curtains are checked first (user 2026-11-21)
        est, t, what = self.candidate
        if now - t > self.check_cfg['check_max_age']:
            self.candidate = None
            return
        pose = self.pose()
        if pose is None:
            return
        xy = pose[0]
        if self.is_handled(est) or self.is_rejected(est, xy):
            self.candidate = None
            return
        d = abs(est.distance(np.asarray(xy)))
        if d > self.check_cfg['check_distance'] + 0.5:
            return                      # drove away from it meanwhile
        self.checking = True
        self.set_state('HANDOVER', f'possible stairs {d:.2f} m away ({what}): asking '
                       'maze_explorer to stop Nav2 so the robot can face them and check')
        if not self.status_topic or self.explorer == 'STAIRS':
            self.start_check()

    def start_check(self):
        est = self.candidate[0]
        pose = self.pose()
        if pose is None:
            return self.end_check(False, 'no TF')
        self.checking = False
        self.check = StairCheck(est, pose[0], time.monotonic(), self.check_params)
        self.tracker.clear()            # fresh votes, facing them
        self.lidar_tracker.clear()
        self.set_state('CHECKING', 'turning to face the possible stairs, then '
                       + ('Pose mode: level, nose up, nose down,' if self.check_params.pose
                          else f'{self.check_params.hold:.0f} s still,')
                       + ' looking for discrete steps')

    def end_check(self, stairs, why, found=None):
        """Check done: stairs -> line up (start_manoeuvre); else reject the spot."""
        chk, self.check = self.check, None
        cand = self.candidate
        self.candidate = None
        if stairs and found is not None:
            self.lock = found
            pose = self.pose()
            if pose is not None:
                self.announce(found, found.distance(np.asarray(pose[0])))
            self.get_logger().info(f'stair check: stairs ({why}); lining up')
            return self.start_manoeuvre()
        if cand is not None:
            pose = self.pose()
            xy = pose[0] if pose is not None else (0.0, 0.0)
            pt = chk.target if chk is not None else cand[0].edge_point(np.asarray(xy))
            self.rejected.append((np.asarray(pt, float), time.monotonic()))
        self.cooldown_until = time.monotonic() + 3.0
        self.set_state('IDLE', f'stair check: not stairs ({why}); left alone for '
                       f'{self.check_cfg["check_forget"]:.0f} s')

    def check_control(self, now):
        if self.status_topic and self.explorer != 'STAIRS':
            self.stop_robot()
            if self.check is not None and self.check.phase in ('POSE_ENTER', 'HOLD',
                                                                 'POSE_LEAVE'):
                self.send_look('end')
            self.check = None
            self.cooldown_until = now + 5.0
            return self.set_state('IDLE', f'stair check stopped: maze_explorer is '
                                  f'{self.explorer}')
        pose = self.pose()
        if pose is None:
            self.cmd_pub.publish(Twist())
            return
        xy, yaw, _ = pose
        w, phase, look = self.check.tick(now, xy, yaw, self.pose_status)
        self.send_look(look)
        if phase == 'DONE':
            self.stop_robot()
            found, why = self.fused(now, xy)
            chk = self.check
            hits = (f'{chk.cam_hits} camera / {chk.lidar_hits} lidar hits in '
                    f'{chk.hold_time():.0f} s'
                    + (' (Pose mode, pitched up and down)' if chk.posing else
                       f' ({chk.pose_note})' if chk.pose_note else ''))
            if found is not None and not self.is_handled(found):
                return self.end_check(True, f'confirmed: {found.risers} risers of '
                                      f'{found.rise * 100:.0f} cm, {why}; {hits}', found)
            if chk.passed():
                est = chk.est
                return self.end_check(True, f'{hits}', est)
            return self.end_check(False, hits)
        if w:
            walls = self.walls()
            if walls is not None:
                _, w, _ = walls.safe(xy, yaw, 0.0, w)
        msg = Twist()
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def pose(self, stamp=None, with_z=False):
        """((x, y), yaw, R) of base_link in odom (+ z with_z); stamp None = latest. Never
        blocks."""
        tf = None
        for t in ((stamp, Time()) if stamp is not None else (Time(),)):
            try:
                tf = self.tf.lookup_transform(self.odom_frame, self.base_frame, t)
                break
            except TransformException:
                continue
        if tf is None:
            return None
        r, a = matrix_of(tf)
        out = (a[0], a[1]), math.atan2(r[1, 0], r[0, 0]), r
        return out + (a[2],) if with_z else out

    def camera(self, frame):
        """Depth optical frame's pose in base_link, looked up once.

        From camera_link (+ the Orbbec's fixed optical rotation) when it exists, not from the
        image's frame: on 2026-11-18 the robot's /tf_static gave camera_depth_optical_frame a
        second, unpitched parent (base_link, 0.25/0/0.05) and TF resolved through it, skewing
        every height by 20 deg. Without camera_link (simulator, test rigs): the image frame."""
        if self.cam is not None:
            return self.cam
        link = self.get_parameter('camera_link_frame').value
        direct = None
        try:
            direct = matrix_of(self.tf.lookup_transform(self.base_frame, frame, Time()))
        except TransformException:
            pass
        if link:
            try:
                self.cam = camera_from_link(*matrix_of(
                    self.tf.lookup_transform(self.base_frame, link, Time())))
            except TransformException:
                self.cam = None
        if self.cam is None:
            if direct is None:
                return None
            self.cam = direct
            self.get_logger().info(f'camera pose from TF {self.base_frame} -> {frame}'
                                   + (f' ({link} not in TF)' if link else ''))
            return self.cam
        msg = f'camera pose from {self.base_frame} -> {link} + optical rotation'
        if direct is not None:
            ang, dist = mount_disagreement(self.cam, direct)
            if ang > 3.0 or dist > 0.03:
                self.get_logger().warn(
                    f'{msg}; TF {self.base_frame} -> {frame} disagrees by {ang:.0f} deg / '
                    f'{dist * 100:.0f} cm (a second parent for {frame} on /tf_static?). '
                    f'Using {link}')
                return self.cam
        self.get_logger().info(msg)
        return self.cam

    @staticmethod
    def image_array(msg):
        dtype = ENCODINGS.get(msg.encoding)
        if dtype is None:
            return None
        item = np.dtype(dtype).itemsize
        a = np.frombuffer(msg.data, dtype=dtype)
        if a.size * item < msg.step * msg.height:
            return None
        a = a.reshape(msg.height, msg.step // item)[:, :msg.width]
        if bool(msg.is_bigendian) != (sys.byteorder == 'big'):
            a = a.byteswap()
        return a

    # ------------------------------------------------------------------ detection
    def process(self):
        if not self.looking() or not self.depth_new or self.k is None:
            return
        msg, self.depth_new = self.depth, False
        depth = self.image_array(msg)
        if depth is None:
            self.get_logger().error(f'depth encoding {msg.encoding!r} / size not usable '
                                    '(want 16UC1 mm or 32FC1 m)', throttle_duration_sec=10.0)
            return
        cam = self.camera(msg.header.frame_id)
        pose = self.pose(Time.from_msg(msg.header.stamp), with_z=True)
        if cam is None or pose is None:
            self.get_logger().info(f'waiting for TF {self.odom_frame} -> {self.base_frame} -> '
                                   f'{msg.header.frame_id}', throttle_duration_sec=10.0)
            return
        xy, yaw, r_ob, z = pose
        r, t, _ = level_transform(r_ob, *cam)
        # The lidar map's floor (odom z) in the level frame at base_link: close up the camera
        # no longer sees the floor, the map remembers it (bottom-riser inference needs it).
        floor = None if self.floor_z is None else self.floor_z - z
        det = detect_stairs(depth, self.k, r, t, self.params, floor_z=floor)
        now = time.monotonic()
        est = to_odom(det, xy, yaw) if det is not None else None
        handled = est is not None and self.is_handled(est)
        self.tracker.add(now, None if handled else est)
        if est is not None and not handled:
            self.note_candidate(est, xy, f'camera: {det.risers} risers')
        found, why = self.fused(now, xy)
        if det is not None:
            self.publish_detection(msg, det, est, found, why, handled)
        self.decide(now, xy, yaw, found, why, det)

    # ------------------------------------------------------------------ lidar
    def add_clouds(self):
        """Queued /livox/points into the odom height map (TF at each scan's stamp, never
        blocking; scans whose TF isn't there within 0.5 s are dropped). While not looking,
        one scan per idle_lidar_period s; the others are dropped."""
        keep = []
        idle, now = not self.looking(), time.monotonic()
        for msg in self.clouds[-10:]:
            if idle and now - self.last_hmap_add < self.idle_lidar_period:
                continue
            stamp = Time.from_msg(msg.header.stamp)
            try:
                tf_l = self.tf.lookup_transform(self.odom_frame, msg.header.frame_id, stamp)
                tf_b = self.tf.lookup_transform(self.odom_frame, self.base_frame, stamp)
            except TransformException:
                age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
                if age < 0.8:                   # = wall_scan max_cloud_age (odom often late)
                    keep.append(msg)
                continue
            r_l, t_l = matrix_of(tf_l)
            r_b, t_b = matrix_of(tf_b)
            pts = cloud_xyz(msg.data, msg.fields, msg.point_step, msg.row_step, msg.width,
                            msg.height, msg.is_bigendian)
            self.hmap.add(pts @ r_l.T + t_l, t_b, math.atan2(r_b[1, 0], r_b[0, 0]))
            self.lidar_scans += 1
            self.last_hmap_add = now
        self.clouds[:] = keep

    def process_lidar(self):
        """Lidar-only stairs from the height map into lidar_tracker, then decide()."""
        if not self.looking() or self.lidar_scans == 0:
            return
        try:
            r, a = matrix_of(self.tf.lookup_transform(self.odom_frame, self.base_frame, Time()))
        except TransformException:
            return
        floor = self.hmap.floor(a)
        if floor is not None:
            self.floor_z = floor
        now = time.monotonic()
        ests = [e for e in stairs_lidar.detect(self.hmap, a, self.floor_z)
                if not self.is_handled(e)]
        if self.lidar_cfg['descend']:
            # The floor under the feet, not floor_z (the lowest layer around: on a raised
            # plate that is the lab floor past the walls, and the robot's own floor would not
            # be "the floor" the drop starts from).
            ests += [e for e in stairs_lidar.detect_down(self.hmap, a, self.hmap.floor_under(a))
                     if not self.is_handled(e)]
        xy = (a[0], a[1])
        near = min(ests, key=lambda e: e.distance(np.asarray(xy))) if ests else None
        self.lidar_tracker.add(now, near)
        if near is not None:
            self.note_candidate(near, xy, 'lidar height map')
        found, why = self.fused(now, xy)
        self.decide(now, xy, math.atan2(r[1, 0], r[0, 0]), found, why, None)

    def terrain_tick(self):
        """terrain_rate Hz: /maze/local_floor_z (floor under the feet; nothing while lying or
        not seen yet: listeners keep the last) and /maze/terrain (JSON: level of the strip
        ahead, max_step / low / spread m, on_rough under the robot, floor used)."""
        if self.lidar_scans == 0:
            return
        try:
            r, a = matrix_of(self.tf.lookup_transform(self.odom_frame, self.base_frame, Time()))
        except TransformException:
            return
        under = self.hmap.floor_under(a)
        if under is not None and self.local_floor_pub is not None:
            self.local_floor_pub.publish(Float32(data=float(under)))
        if self.terrain_pub is None:
            return
        # Until the robot has walked over seen ground (~1 m) there is no floor under it:
        # wall_scan's start floor, then the lowest layer around (floor_z) as a last resort.
        ref, src = under, 'under'
        if ref is None:
            ref, src = (self.start_floor, 'start') if self.start_floor is not None else \
                (self.floor_z, 'around')
        t = stairs_lidar.terrain(self.hmap, a, math.atan2(r[1, 0], r[0, 0]), ref)
        t = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in t.items()}
        t['floor'] = src if ref is not None else None
        self.terrain_pub.publish(String(data=json.dumps(t)))

    def fused(self, now, xy):
        """(estimate, why): camera tracker + lidar check + lidar tracker (stairs_lidar.fuse)."""
        cam = self.tracker.estimate(now)
        if self.hmap is None:
            return cam, 'camera only (no lidar)'
        lid = self.lidar_tracker.estimate(now)
        if lid is not None and not lid.anchored and not self.lidar_cfg['lidar_alone_trigger']:
            lid = None
        chk = None
        if cam is not None and self.lidar_scans:
            chk = stairs_lidar.check(self.hmap, cam, self.floor_z, xy)
            if chk.kind == 'veto' and not self.lidar_cfg['lidar_veto']:
                chk = None
        if cam is None and lid is not None and \
                not (self.lidar_cfg['lidar_alone_trigger'] and lid.anchored):
            return None, 'lidar only, not anchored on the floor'
        return stairs_lidar.fuse(cam, lid, chk, xy)

    def decide(self, now, xy, yaw, found, why, det):
        """Trigger / refine with the fused estimate (from a camera frame or a lidar pass)."""
        if found is None:
            if why and 'vetoed' in why:
                self.get_logger().info(why, throttle_duration_sec=3.0)
            return
        self.publish_pose(found, xy)
        dist = found.distance(np.asarray(xy))
        if self.state in CHECK_STATES:
            return                      # check_control decides at the end of the hold
        if self.state == 'IDLE' and self.check_on and self.in_look():
            self.note_candidate(found, xy, f'confirmed, {found.risers} risers, seen during '
                                f'{self.explorer}')
            return                      # stair checks come after junction checking
        if self.state == 'IDLE' and self.held_id(found, xy) is not None:
            self.get_logger().info(f'stairs {dist:.2f} m ahead are held (junction id '
                                   f'{self.held_id(found, xy)}): checked after its other ways',
                                   throttle_duration_sec=5.0)
            return
        if self.state == 'IDLE':
            if dist > self.cfg['trigger_distance'] or now < self.cooldown_until:
                self.note_candidate(found, xy, f'{found.risers} risers, {dist:.1f} m away')
                self.get_logger().info(f'stairs {dist:.2f} m ahead ({found.risers} risers, '
                                       f'{found.rise * 100:.0f} cm; {why})',
                                       throttle_duration_sec=3.0)
                return
            front, off = in_front(found, xy, det, self.cfg['trigger_max_offset'])
            if not front:
                self.note_candidate(found, xy, 'seen from beside')
                self.get_logger().info(
                    f'stairs {dist:.2f} m ahead but the robot is beside them ('
                    + (f'{off:.2f} m off their centre line' if off is not None else
                       'its line misses the risers in view')
                    + '): not taking over until it is in front', throttle_duration_sec=3.0)
                return
            if not self.cfg['drive']:
                self.announce(found, dist)
                self.get_logger().info(f'stairs {dist:.2f} m ahead; drive:=false, not moving',
                                       throttle_duration_sec=3.0)
                return
            self.announce(found, dist)
            self.lock = found
            if self.status_topic:
                self.set_state('HANDOVER', ('descending ' if found.down else '')
                               + f'stairs {dist:.2f} m ahead at '
                               f'{math.degrees(wrap(found.heading - yaw)):+.0f} deg, '
                               f'{found.risers} risers of {found.rise * 100:.0f} cm ({why}); '
                               'waiting for maze_explorer to stop Nav2')
                if self.explorer == 'STAIRS':
                    self.start_manoeuvre()
            else:
                self.start_manoeuvre()
        elif self.lock is not None:
            # Lidar-only estimates stop refining earlier: inside ~1.4 m the lidar sees neither
            # the floor nor the bottom riser, and each new estimate drifts a little later
            # (end-to-end test: stopped 0.36 m from the riser instead of 0.45).
            freeze = self.cfg['freeze_distance'] if found.source != 'lidar' else \
                max(self.cfg['freeze_distance'], self.cfg['lidar_freeze_distance'])
            self.lock = refine_lock(self.lock, found, xy, freeze,
                                    self.tracker.pos_tol, self.tracker.yaw_tol)

    def announce(self, est, dist):
        """Print "stairs detected" once per staircase (on the terminal / ros2 launch output,
        and in the log), when the robot takes over to line up with it (or, with drive:=false,
        when it is confirmed)."""
        if any(a.close_to(est, self.cfg['suppress_radius'], math.radians(30))
               for a in self.announced):
            return
        self.announced.append(est)
        print('stairs detected', flush=True)
        self.get_logger().info(f'stairs detected: {dist:.2f} m ahead, {est.risers} risers '
                               + ('DOWN ' if est.down else '')
                               + f'({est.source}); lining up with them'
                               if self.cfg['drive'] else
                               f'stairs detected: {dist:.2f} m ahead ({est.source})')

    def is_handled(self, est):
        return any(h.close_to(est, self.cfg['suppress_radius'], math.radians(30))
                   for h in self.handled)

    def publish_detection(self, msg, det, est, found, why, handled):
        e = est.edge_point(est.d() * est.along)
        confirmed = found is not None
        self.det_pub.publish(String(data=json.dumps({
            'stamp': f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}',
            'frame': self.odom_frame, 'distance': round(det.distance, 3),
            'angle_deg': round(math.degrees(det.yaw), 1), 'lateral': round(det.lateral, 3),
            'centre_ok': det.centre_ok, 'front_seen': det.front_seen, 'risers': det.risers,
            'rise': round(det.rise, 3), 'tread': round(det.tread, 3),
            'first_rise': round(det.first_rise, 3), 'floor_measured': det.floor_measured,
            'inferred_bottom': det.inferred_bottom,
            'width': round(det.width, 3), 'edge': [round(e[0], 3), round(e[1], 3)],
            'heading_deg': round(math.degrees(est.heading), 1),
            'confirmed': confirmed, 'handled': handled,
            'source': found.source if found is not None else None, 'fusion': why,
            'lidar_floor_z': None if self.floor_z is None else round(self.floor_z, 3)})))

    def publish_pose(self, est, xy):
        m = PoseStamped()
        m.header.frame_id = self.odom_frame
        m.header.stamp = self.get_clock().now().to_msg()
        e = est.edge_point(np.asarray(xy))
        m.pose.position.x, m.pose.position.y = float(e[0]), float(e[1])
        m.pose.orientation.z = math.sin(est.heading / 2)
        m.pose.orientation.w = math.cos(est.heading / 2)
        self.pose_pub.publish(m)

    # ------------------------------------------------------------------ driving
    def start_manoeuvre(self):
        c = self.cfg
        # Lidar-only stairs: its first riser can be ~10 cm late (at 1.4 m it barely sees the
        # floor before the bottom step): stop that much earlier.
        stop = c['stop_distance'] + (c['lidar_stop_margin'] if self.lock.source == 'lidar'
                                     else 0.0)
        self.approach = Approach(
            stop_distance=stop, align_tol=math.radians(c['align_tol_deg']),
            final_tol=math.radians(c['final_tol_deg']), max_v=c['max_speed'],
            max_w=c['max_turn_rate'], min_w=c['min_turn_rate'],
            settle_time=c['settle_time'], centre=c['centre'], walls=self.walls())
        self.last_note = ''
        self.last_pose_at = self.started_at = time.monotonic()
        pose = self.pose()
        why = 'turning to face the stairs'
        if pose is not None:
            d = self.lock.distance(np.asarray(pose[0]))
            if d < stop:
                why = (f'already {d:.2f} m from the first riser (< stop_distance '
                       f'{c["stop_distance"]:.2f}); squaring up only')
        if self.lock.centre is None:
            why += '; stair sides not both seen: facing them straight, not centring'
        if self.approach.walls is None and self.wall_cfg['wall_check']:
            why += '; NO wall checks yet (no /map or TF map -> odom)'
        self.set_state('ALIGNING', why)

    def control(self):
        now = time.monotonic()
        if self.state in CHECK_STATES:
            return self.check_control(now)
        if self.state == 'HANDOVER':
            if now - self.since > self.cfg['handover_timeout']:
                self.cooldown_until = now + 10.0
                self.lock, self.checking = None, False
                self.set_state('IDLE', f'maze_explorer did not stop Nav2 within '
                               f'{self.cfg["handover_timeout"]:.0f} s (it is {self.explorer}); '
                               'not driving')
            return
        if self.state in ('AT_STAIRS', 'ABORTED'):
            self.maybe_rearm()
            return
        if self.state not in DRIVING:
            return
        if self.status_topic and self.explorer != 'STAIRS':
            return self.abort(f'maze_explorer is {self.explorer}, not STAIRS')
        if now - self.started_at > self.cfg['manoeuvre_timeout']:
            return self.hold(f'not lined up after {self.cfg["manoeuvre_timeout"]:.0f} s')
        pose = self.pose()
        if pose is None:
            self.cmd_pub.publish(Twist())
            if now - self.last_pose_at > 2.0:
                self.abort(f'no TF {self.odom_frame} -> {self.base_frame} for 2 s')
            return
        self.last_pose_at = now
        xy, yaw, _ = pose
        walls = self.walls()
        if walls is not None:               # fresh map -> odom offset (SLAM corrections)
            self.approach.walls = walls
        v, w = self.approach.update(now, xy, yaw, self.lock)
        if self.approach.note != self.last_note:
            self.last_note = self.approach.note
            self.get_logger().info(f'line-up: {self.approach.note}')
        if self.approach.phase == 'BLOCKED':
            return self.hold(f'line-up blocked: {self.approach.note}')
        state = PHASE_STATE[self.approach.phase]
        if state == 'AT_STAIRS':
            self.stop_robot()
            self.handled.append(self.lock)
            self.stop_xy = np.asarray(xy)
            dist = self.lock.distance(np.asarray(xy))
            err = math.degrees(wrap(self.lock.heading - yaw))
            lat = self.lock.lateral(np.asarray(xy))
            self.set_state('AT_STAIRS', ('descending stairs: ' if self.lock.down else '')
                           + f'{dist:.2f} m from the '
                           + ('top edge' if self.lock.down else 'first riser')
                           + f', {err:+.1f} deg off perpendicular'
                           + (f', {lat * 100:+.0f} cm off centre'
                              if self.lock.centre is not None else '')
                           + (f' ({self.approach.note})' if self.approach.note else ''))
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)
        if state != self.state:
            dist = self.lock.distance(np.asarray(xy))
            self.set_state(state, f'{dist:.2f} m from the first riser, heading '
                           f'{math.degrees(wrap(self.lock.heading - yaw)):+.0f} deg off')

    def hold(self, why):
        """At the stairs but not lined up: stop and hold (AT_STAIRS) for the operator."""
        self.stop_robot()
        pose = self.pose()
        where = ''
        if pose is not None and self.lock is not None:
            xy, yaw, _ = pose
            where = (f'; {self.lock.distance(np.asarray(xy)):.2f} m from the first riser, '
                     f'{math.degrees(wrap(self.lock.heading - yaw)):+.0f} deg off '
                     'perpendicular')
            self.stop_xy = np.asarray(xy)
        if self.lock is not None:
            self.handled.append(self.lock)
        down = 'descending stairs, ' if self.lock is not None and self.lock.down else ''
        self.set_state('AT_STAIRS', f'NOT LINED UP: {down}{why}{where} - line it up by hand')

    def abort(self, why):
        self.stop_robot()
        if self.lock is not None:
            self.handled.append(self.lock)       # don't loop on stairs it can't line up with
        self.lock = None
        self.set_state('ABORTED', why)

    def maybe_rearm(self):
        """Back to IDLE once the explorer has moved on (or, alone, the robot has)."""
        if self.status_topic:
            if self.explorer not in ('STAIRS', 'AT_STAIRS'):
                self.lock = None
                self.tracker.clear()
                self.set_state('IDLE', f'maze_explorer is {self.explorer}; looking again')
            return
        if self.state == 'ABORTED':
            self.set_state('IDLE', 'looking again')
            return
        pose = self.pose()
        if pose is not None and self.stop_xy is not None and \
                np.hypot(*(np.asarray(pose[0]) - self.stop_xy)) > 0.5:
            self.lock = None
            self.tracker.clear()
            self.set_state('IDLE', 'moved away from the stairs; looking again')

    def stop_robot(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = StairDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # After Ctrl+C the context may already be gone; cmd_vel_bridge then sends its own
        # stop once nav_cmd_vel goes quiet (idle_timeout).
        if node.state in DRIVING and rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
