"""Explore the maze on the SLAM map with Nav2 until the robot is out.

Start: once /map, TF and Nav2 are up, the robot holds still (status PAUSED) until the
operator publishes on /maze/start (std_msgs/Empty):
    ros2 topic pub --once /maze/start std_msgs/msg/Empty
so there is time to start a rosbag recording. Messages sent before PAUSED are ignored.
wait_for_start:=false skips the wait (simulator). Then it creeps `creep_distance` straight
ahead (status CREEPING), with no steering and no turn on the spot: a sim run that steered
the creep 28 deg towards the frontier swung the rear corner into a side wall 0.15 m away,
which the lidar can't see that close (vertical FOV down to -7 deg: no wall band within
~0.5 m beside the robot, so it is missing from the map and from every costmap). The lidar sees no walls behind the robot, so the SLAM map ends at about the
start pose and the robot can sit a few cm off it (seen live: base_link 2-3 cm behind the map
edge, "Robot is out of bounds of the costmap"). The creep also takes the robot off the start
cell's back wall: placed with its rear ~8 cm from it, turning on the spot hit it (spin
"Collision Ahead", RPP "detected collision ahead"). Humble's Smac planner crashes when asked
to plan from outside the global costmap; that is now ruled out by the rolling global
costmap in nav2_params.yaml, not by the creep. Then a scan (status SCANNING, Nav2's /spin
behavior, `start_scan`) turns `scan_angle` left, twice that right and `scan_angle` left again,
back to the heading it had before the scan, so SLAM sees the start area. Both are checked
against walls on the SLAM map with the footprint box, so the robot can start with its body
0.15 m from a wall: the creep is shortened to keep `creep_margin` to a wall ahead (or
skipped), and each scan side is cut to what its sweep clears with `turn_margin` (e.g. ~20 deg
left / ~32 deg right with a wall 0.15 m off the right side), still ending on the start heading;
both sides < 5 deg -> no scan. A failed
turn is logged and the run moves on; it never blocks exploring. Creep and scan are skipped
with go:=false.

Loop (period `replan_period`):
  1. read /map (slam_toolbox) and the robot pose (TF map -> base_link)
  2. exit check: was in corridors, has moved >= min_displacement, and a disk of seen-free
     floor >= exit_open_radius lies within exit_near_radius of it (and away from the
     start) for `dwell` s -> DONE
  3. frontier = free cell next to unknown; pick the nearest along free space, keep the
     current one unless another is clearly closer (no flip-flopping)
  4. send it to Nav2 /navigate_to_pose; a reached goal is blacklisted. A failed one is only
     set aside (other frontiers first), and nothing if it failed while Nav2 was down
     (lifecycle manager is_active = false)
  5. recovery: `max_failures` failures in a row, or only set-aside frontiers left, while open
     frontiers remain -> clear both costmaps, forget the set-aside goals and try again, up to
     `max_recoveries` times. Seen live: a phantom wall (the robot's own leg) put the robot
     "in lethal space" for the planner, the goal failed, was blacklisted, and the run ended
     NO_EXIT with a 2 m wide frontier still open.
Turning near a wall (`pre_turn`): before each new goal, if the way there leaves the robot
more than `pre_turn_min_angle` off its heading (Nav2's path follower would turn on the spot
first, always the short way) and that turn's sweep would bring the footprint within
`turn_margin` of a wall on the SLAM map, the explorer stops (status TURNING) and turns the
robot itself the long way round, if that sweep is clear, then goes on (core.plan_turn).
Blocked both ways (the usual case in a 0.9 m corridor: the rear corners turn on a 0.437 m
circle): it creeps straight ahead, no steering, up to `pre_turn_max_creep` m, to the spot
where the turn is clear with the most room to the nearest wall, then turns there
(core.plan_creep_turn: `turn_margin`, else 1 cm, which is all a 0.9 m corridor corner
leaves). No such spot: left to Nav2. Clear short turns are left to Nav2 too. Once per goal.
Sim 2026-11-16: an 80 deg short-way turn at a cell edge swung the rear corner into the wall. Walls within ~0.5 m
beside the robot that the lidar never saw from further away aren't on the map, so this only
covers walls it has seen.
Stairs (stair_detector.py): while EXPLORING, TURNING or LOOKING, stair_detector's /stairs/status
HANDOVER means stairs are close ahead. The explorer cancels the Nav2 goal (or turn), sends a
zero Twist and goes to STAIRS; only then does stair_detector drive (face the stairs, square up,
stop in front), so it and Nav2 never both command nav_cmd_vel. Its AT_STAIRS -> AT_STAIRS here:
hold still until /maze/start (switch gait for the stairs first), then explore on. Its ABORTED
or IDLE, or `stairs_timeout` s in STAIRS -> back to EXPLORING. stairs_topic:='' = ignore it.
Look-around (`look_topic`, from poster_detector): a QR room with no survivor poster confirmed
yet (QR codes only appear in rooms with survivors). While EXPLORING within `look_radius` of the
requested point, the explorer cancels the goal and sweeps left, right and back (LOOKING), each
way as far as `look_angle` or until the body would touch a wall (like the start scan), so
the camera sees the room; then EXPLORING again. Requests never reached expire after
`look_timeout` s (poster_detector then reports that room's survivor from the QR alone).
Unreadable QR code (`qr_unread_topic`, from qr_scanner: located in the image but not read,
e.g. too far round, blurred or at a steep angle): while EXPLORING, the explorer cancels the
goal and turns to face it (FACING_QR; cut short where the body would touch a wall, like
LOOKING), then holds still while qr_scanner runs its slow, thorough decode on every frame.
It goes back to EXPLORING when the code is read (`qr_read_topic`), when it hasn't been seen
for `face_lost` s, or after `face_timeout` s (that spot is then ignored for `face_cooldown`
s, so the robot can't get stuck on it). Re-aims up to `face_max_turns` times if it drifts.
Stairs (HANDOVER) interrupt it like LOOKING.
Nav2 down for `nav_down_timeout` s (a server crashed and the lifecycle manager deactivated
the rest; Humble doesn't bring it back) -> STOPPED, not NO_EXIT.
Nothing left to explore and no exit -> DONE (no exit found): the running goal is cancelled
at once, a zero Twist goes to nav_cmd_vel and "nav stopped" is printed. Either way Nav2 is
cancelled, the map is saved and /maze/status says why. Ctrl+C / SIGTERM (also from
ros2 launch) ends the run as STOPPED the same way: rclpy's own signal handler is off so
the context is still valid for the cancel and the final status.

Status on /maze/status (std_msgs/String, latched): STARTING, PAUSED, CREEPING, SCANNING, EXPLORING, TURNING,
LOOKING, FACING_QR, STAIRS, AT_STAIRS, EXITED, NO_EXIT, STOPPED. Goals on /maze/goal (PoseStamped) for RViz. `go`
parameter false = plan and publish goals but never send them to Nav2 (dry run on the robot).
"""
import math
import os
import signal
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import (ExitDetector, Grid, choose_frontier, find_frontiers,
                           free_distance_ahead, heading_towards, max_turn, plan_creep_turn,
                           plan_turn)

START_CMD = 'ros2 topic pub --once /maze/start std_msgs/msg/Empty'

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Explorer(Node):

    def __init__(self):
        super().__init__('maze_explorer')
        p = self.declare_parameter
        p('go', True)
        p('start_delay', 5.0)          # s after the first map before moving
        p('wait_for_start', True)      # hold still until /maze/start (start rosbag recording)
        p('replan_period', 1.0)
        p('robot_radius', 0.25)        # half width incl. feet + a little
        p('goal_clearance', 0.33)      # goals at least this far from walls
        p('min_frontier_size', 5)      # cells
        p('robot_width', 0.46)         # m, footprint width (nav2_params.yaml)
        p('min_frontier_width_ratio', 0.8)   # frontiers narrower than this x robot_width: ignored
        p('goal_timeout', 90.0)        # s per goal before giving up on it
        p('max_failures', 3)           # failed goals in a row before NO_EXIT
        p('done_confirm', 5)           # replans in a row with no frontier before NO_EXIT
        p('max_recoveries', 3)         # costmap clear + retry rounds while frontiers are open
        p('exit_open_radius', 1.1)     # m of seen-free floor around a point = outside
        p('exit_near_radius', 0.8)     # that point within this distance of the robot
        p('exit_min_displacement', 1.5)
        p('exit_dwell', 2.0)
        p('max_runtime', 900.0)        # s; hard stop
        p('creep_distance', 0.10)      # m straight ahead after start, onto the map and
        #                                off the back wall so the scan turns clear it; 0 = off
        p('creep_speed', 0.05)         # m/s
        p('start_scan', True)          # after the creep: scan_angle left, 2x right, left again
        p('scan_angle', math.pi / 4)   # rad; less on a side where the body would hit a wall
        # Footprint box (nav2_params.yaml: x -0.37..0.27, y +-0.23) for the start creep and scan
        # wall checks on the SLAM map, so the robot can start with its body 0.15 m from a wall.
        p('footprint_back', 0.37)
        p('footprint_front', 0.27)
        p('footprint_half_width', 0.23)
        p('turn_margin', 0.03)         # m kept clear of walls while turning on the spot
        p('pre_turn', True)            # turn the robot ourselves when Nav2's turn would hit a wall
        p('pre_turn_min_angle', 0.6)   # rad; = RPP rotate_to_heading_min_angle (nav2_params.yaml)
        p('pre_turn_max_creep', 0.6)   # m straight ahead to find room when blocked both ways; 0 = off
        p('creep_margin', 0.05)        # m kept clear of walls ahead at the end of the creep
        p('goal_max_offset', 7.5)      # m in x and y from the robot; global costmap is 16 x 16 rolling
        p('goal_grace', 3)             # replans the current goal's frontier may be missing
        #                                before switching goals (a blink isn't a switch)
        p('nav_down_timeout', 5.0)     # s of Nav2 reported inactive before giving up (STOPPED)
        p('stairs_topic', '/stairs/status')   # stair_detector handshake; '' = ignore stairs
        p('stairs_timeout', 60.0)      # s in STAIRS (stair_detector driving) before exploring on
        # Look-around requests from poster_detector: a QR room (QR codes only appear in rooms
        # with survivors) where no poster has been confirmed yet. '' = ignore them.
        p('look_topic', '/perception/look_request')
        p('look_radius', 1.5)          # m: look around once the robot is this close
        p('look_angle', math.pi)       # rad each way (less where the body would hit a wall)
        p('look_timeout', 120.0)       # s: drop a request the robot never got close to
        # QR codes seen but unreadable (qr_scanner /qr/unread): turn to face them until read
        # (/qr/read). '' = ignore; maze_nav.launch.py blanks both when perception:=false.
        p('qr_unread_topic', '/qr/unread')
        p('qr_read_topic', '/qr/read')
        p('face_timeout', 10.0)        # s facing a code without a read before giving up on it
        p('face_lost', 4.0)            # s with no unread sighting: code gone (or was noise)
        p('face_min_angle', math.radians(8))   # rad: closer than this = already facing it
        p('face_max_turns', 3)         # re-aims while it drifts off-centre
        p('face_cooldown', 60.0)       # s: ignore a code given up on (within 0.75 m)
        p('map_dir', os.path.expanduser('~/challenge_ws/maps'))
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in ('go', 'start_delay', 'wait_for_start', 'robot_radius', 'goal_clearance',
                                      'min_frontier_size', 'robot_width', 'min_frontier_width_ratio', 'goal_timeout', 'max_failures', 'done_confirm', 'max_recoveries',
                                      'max_runtime', 'map_dir', 'creep_distance', 'creep_speed',
                                      'start_scan', 'scan_angle', 'footprint_back',
                                      'footprint_front', 'footprint_half_width', 'turn_margin',
                                      'pre_turn', 'pre_turn_min_angle', 'pre_turn_max_creep',
                                      'creep_margin', 'goal_max_offset', 'goal_grace',
                                      'nav_down_timeout', 'stairs_timeout', 'look_radius',
                                      'look_angle', 'look_timeout', 'face_timeout', 'face_lost',
                                      'face_min_angle', 'face_max_turns', 'face_cooldown')}
        self.exit = ExitDetector(open_radius=g('exit_open_radius'),
                                 near_radius=g('exit_near_radius'),
                                 min_displacement=g('exit_min_displacement'),
                                 dwell=g('exit_dwell'))

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        # When a Nav2 server dies, the lifecycle manager deactivates the rest and every goal
        # then fails at once. Those failures say nothing about the maze, so they must not
        # blacklist frontiers (which ended a run as NO_EXIT 6 s after planner_server crashed).
        self.nav_check = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        self.clear_costmaps = [
            self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')]
        self.status_pub = self.create_publisher(String, '/maze/status', LATCHED)
        self.goal_pub = self.create_publisher(PoseStamped, '/maze/goal', LATCHED)
        # Same input as Nav2's velocity_smoother output, so the stop goes through
        # cmd_vel_bridge (clamps, drive:=false preview) like any other command.
        self.cmd_pub = self.create_publisher(Twist, 'nav_cmd_vel', 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        # Volatile on purpose: a start sent before this run is never replayed.
        self.create_subscription(Empty, '/maze/start', self.on_start, 10)
        if g('stairs_topic'):
            self.create_subscription(String, g('stairs_topic'), self.on_stairs, LATCHED)
        if g('look_topic'):
            self.create_subscription(PointStamped, g('look_topic'), self.on_look_request, 10)
        if g('qr_unread_topic'):
            self.create_subscription(PointStamped, g('qr_unread_topic'), self.on_qr_unread, 10)
            if g('qr_read_topic'):
                self.create_subscription(String, g('qr_read_topic'), self.on_qr_read, 10)
        self.create_timer(g('replan_period'), self.step)
        self.create_timer(0.1, self.creep_tick)   # faster than cmd_vel_bridge's idle_timeout

        self.map = None
        self.first_map = None
        self.started = None
        self.start_requested = False
        self.state = None
        self.current = None          # Frontier being driven to
        self.current_missing = 0     # replans in a row its frontier wasn't found
        self.goal_handle = None
        self.goal_sent_at = None
        self.goal_result = None      # None while running, else GoalStatus code
        self.blacklist = []          # reached goals: never again
        self.failed = []             # failed goals: set aside until the next recovery
        self.recoveries = 0
        self.failures = 0
        self.empty_replans = 0       # replans in a row that found no frontier
        self.box = (self.cfg['footprint_back'], self.cfg['footprint_front'],
                    self.cfg['footprint_half_width'])
        self.scan_turns = None       # planned at the start of SCANNING, from the map
        self.creep_target = None     # m the creep actually drives (shortened near a wall)
        self.spin_busy = False       # scan turn sent, result not back yet
        self.spin_handle = None
        self.pre_turned = None       # goal (x, y) the last pre-turn was for: once per goal
        self.turn_creep = None       # TURNING: creeping ahead to room for the turn {goal, ...}
        self.creep_from = None       # (x, y) in map where the creep started
        self.creep_until = None      # monotonic deadline for the creep
        self.goal_result_at = None   # monotonic time goal_result arrived
        self.nav_query = None        # (future, sent_at) of the pending is_active call
        self.nav_reply_at = -1.0     # sent_at of the newest is_active answer
        self.nav_active = True
        self.nav_down_since = None
        self.stairs_since = None     # monotonic time STAIRS began
        self.resume_requested = False   # /maze/start while AT_STAIRS
        self.looks = []              # pending look-around requests: (PointStamped, received)
        self.look_turns = None       # LOOKING: turns still to make
        self.unread = None           # newest /qr/unread: (PointStamped in odom, monotonic time)
        self.qr_read_at = -1e9       # monotonic time of the newest /qr/read
        self.face = None             # FACING_QR: {'since', 'turns'}
        self.face_given_up = []      # [((x, y) in map, monotonic time)]: codes not read in time
        self.set_state('STARTING', 'waiting for /map, TF map->base_link and Nav2')

    # ------------------------------------------------------------------ helpers
    def set_state(self, state, why=''):
        if state != self.state:
            self.state = state
            self.get_logger().info(f'[{state}] {why}')
            self.status_pub.publish(String(data=f'{state}: {why}' if why else state))

    def on_map(self, msg):
        self.map = msg
        if self.first_map is None:
            self.first_map = time.monotonic()

    def on_start(self, _msg):
        if self.state == 'PAUSED':
            self.start_requested = True
            self.get_logger().info('start received')
        elif self.state == 'AT_STAIRS':
            self.resume_requested = True
            self.get_logger().info('start received: exploring on from the stairs')
        elif self.state == 'STARTING':
            self.get_logger().warn('start ignored: not ready yet (wait for PAUSED)')

    def on_stairs(self, msg):
        """stair_detector's /stairs/status (see the module doc)."""
        stairs = msg.data.split(':', 1)[0].strip()
        if stairs == 'HANDOVER' and \
                self.state in ('EXPLORING', 'TURNING', 'LOOKING', 'FACING_QR') \
                and self.cfg['go']:
            if self.spin_handle is not None:          # a turn, scan or look-around
                self.spin_handle.cancel_goal_async()
                self.spin_handle = None
            self.look_turns = None                    # a look-around ends here
            self.face = None                          # so does facing a QR code
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.goal_handle, self.current, self.goal_result = None, None, None
            self.stop_robot()
            self.stairs_since = time.monotonic()
            self.set_state('STAIRS', 'stairs ahead: Nav2 stopped, stair_detector lines the '
                           'robot up')
        elif self.state == 'STAIRS':
            if stairs == 'AT_STAIRS':
                self.resume_requested = False
                self.set_state('AT_STAIRS', f'{msg.data.split(":", 1)[-1].strip()}; holding '
                               f'still - switch gait, then: {START_CMD}')
            elif stairs in ('IDLE', 'ABORTED'):
                self.begin_after_stairs(f'stair_detector {msg.data}')

    def begin_after_stairs(self, why):
        self.pre_turned = None
        self.set_state('EXPLORING', why)

    def robot_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())   # latest, no waiting
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y), yaw

    def min_frontier_width(self):
        return self.cfg['min_frontier_width_ratio'] * self.cfg['robot_width']

    def poll_nav(self):
        """Ask the lifecycle manager whether Nav2 is active; non-blocking, one call at a time."""
        now = time.monotonic()
        if self.nav_query is not None:
            future, sent = self.nav_query
            if future.done():
                active = future.result() is not None and future.result().success
            elif now - sent > 3.0:
                # No answer: the manager is stuck resetting (it blocks on the dead server).
                self.nav_check.remove_pending_request(future)
                active = False
            else:
                return
            self.nav_query = None
            self.nav_reply_at = sent
            if active != self.nav_active:
                # Separate call sites: rclpy raises "Logger severity cannot be changed between
                # calls" if one call site logs at two severities.
                if active:
                    self.get_logger().info('Nav2 is active again')
                else:
                    self.get_logger().error('Nav2 is NOT active (a server died? see the Nav2 '
                                            'log above)')
            self.nav_active = active
            self.nav_down_since = None if active else (self.nav_down_since or now)
        if self.nav_check.service_is_ready():
            self.nav_query = (self.nav_check.call_async(Trigger.Request()), now)

    def nav_known_after(self, t):
        """True once an is_active answer to a call sent after t is in (or none can come)."""
        return self.nav_reply_at >= t or not self.nav_check.service_is_ready()

    # ------------------------------------------------------------------ main loop
    def step(self):
        if self.state in ('EXITED', 'NO_EXIT', 'STOPPED'):
            return
        pose = self.robot_pose()
        if self.map is None or pose is None:
            return
        if self.state == 'STARTING':
            if not self.nav.server_is_ready():
                self.get_logger().info('waiting for Nav2 navigate_to_pose',
                                       throttle_duration_sec=5.0)
                return
            if time.monotonic() - self.first_map < self.cfg['start_delay']:
                return
            if self.cfg['wait_for_start']:
                self.set_state('PAUSED', 'ready; holding still - start the rosbag recording, '
                               f'then: {START_CMD}')
        if self.state in ('STARTING', 'PAUSED'):        # STARTING here = ready, no wait
            if self.state == 'PAUSED' and not self.start_requested:
                self.get_logger().info(f'waiting for start: {START_CMD}',
                                       throttle_duration_sec=15.0)
                return
            self.started = time.monotonic()
            if not self.cfg['go']:
                self.begin()
            else:
                self.creep_or_scan(pose)
        if self.state == 'CREEPING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            return                                  # creep_tick drives, then moves on
        if self.state in ('STAIRS', 'AT_STAIRS'):  # stair_detector drives / holding still
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.state == 'STAIRS' and \
                    time.monotonic() - self.stairs_since > self.cfg['stairs_timeout']:
                self.get_logger().warn(f'stair_detector still busy after '
                                       f'{self.cfg["stairs_timeout"]:.0f} s')
                self.begin_after_stairs('stairs timeout; exploring on')
            elif self.state == 'AT_STAIRS' and self.resume_requested:
                self.resume_requested = False
                self.begin_after_stairs('resumed at the stairs')
            return
        if self.state == 'TURNING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.spin_busy or self.turn_creep is not None:
                return                              # spinning, or creep_tick creeping
            self.set_state('EXPLORING', 'turned; going on')   # replan from the new heading
            return
        if self.state == 'LOOKING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.spin_busy:
                return
            if self.look_turns:
                self.send_spin(self.look_turns.pop(0), 'look around')
                return
            self.set_state('EXPLORING', 'looked around; going on')   # replan from here
            return
        if self.state == 'FACING_QR':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            return self.face_tick(pose)
        if self.state == 'SCANNING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.spin_busy:
                return
            if self.scan_turns is None:
                self.plan_scan(pose)
            if not self.scan_turns:
                self.begin()
            elif not self.spin.server_is_ready():
                if time.monotonic() - self.started < 10.0:   # action discovery takes a moment
                    return
                self.get_logger().warn('Nav2 /spin not available; skipping start scan')
                self.scan_turns = []
                self.begin()
            else:
                self.send_spin(self.scan_turns.pop(0))
                return
        if self.cfg['go']:
            self.poll_nav()
            if self.nav_down_since is not None and \
                    time.monotonic() - self.nav_down_since >= self.cfg['nav_down_timeout']:
                self.finish('STOPPED', 'Nav2 is down (lifecycle manager inactive); '
                            'check the log for a crashed Nav2 server')
                self.stop_robot()
                return
        xy, yaw = pose
        grid = Grid.from_msg(self.map)
        if self.exit.update(grid, xy, time.monotonic()):
            return self.finish('EXITED', f'open space around ({xy[0]:.2f}, {xy[1]:.2f}), '
                               f'{self.exit.open_nearby:.2f} m of open floor next to it')
        if time.monotonic() - self.started > self.cfg['max_runtime']:
            return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')

        if self.maybe_face(grid, xy, yaw):
            return
        if self.maybe_look(grid, xy, yaw):
            return

        # Goal bookkeeping
        if self.goal_result is not None:
            ok = self.goal_result == GoalStatus.STATUS_SUCCEEDED
            if not ok and not self.nav_known_after(self.goal_result_at):
                return                              # was Nav2 still up? ask first
            if not ok and not self.nav_active:
                self.get_logger().warn('goal failed while Nav2 is down; not blacklisting it')
                self.goal_result, self.goal_handle, self.current = None, None, None
                return
            if self.current is not None and ok:
                # Reached goals are blacklisted: a frontier still there after the robot
                # stood on it can't be cleared from there (e.g. unknown seen through a slit
                # at a wall corner, which the example maze has next to the start cell).
                self.blacklist.append(self.current.goal)
            if not ok:
                self.failures += 1
                if self.current is not None:
                    self.failed.append(self.current.goal)
                    self.get_logger().warn(
                        f'goal ({self.current.goal[0]:.2f}, {self.current.goal[1]:.2f}) failed '
                        f'(status {self.goal_result}); set aside, other frontiers first')
            else:
                self.failures = 0
            self.goal_result, self.goal_handle, self.current = None, None, None
        elif self.goal_handle is not None and \
                time.monotonic() - self.goal_sent_at > self.cfg['goal_timeout']:
            self.get_logger().warn('goal timed out; cancelling')
            self.goal_handle.cancel_goal_async()
            return

        if self.failures >= self.cfg['max_failures']:
            return self.recover_or_finish(grid, xy, f'{self.failures} goals failed in a row')

        if not self.nav_active:
            return                                  # no goals into a dead Nav2
        frontiers = self.find(grid, xy, self.blacklist + self.failed)
        # Smac (Humble) doesn't check the goal is inside the rolling global costmap either.
        reach = self.in_reach(frontiers, xy)
        off = self.cfg['goal_max_offset']
        if frontiers and not reach:
            self.get_logger().warn(f'{len(frontiers)} frontiers, all > {off} m away '
                                   '(outside the global costmap); waiting',
                                   throttle_duration_sec=10.0)
            return
        frontiers = reach
        if not frontiers:
            # One empty result can be a map still filling in (start-up, SLAM update);
            # only a few in a row mean everything is mapped.
            self.empty_replans += 1
            if self.empty_replans < self.cfg['done_confirm']:
                self.get_logger().info(f'no frontier ({self.empty_replans}/'
                                       f'{self.cfg["done_confirm"]}); checking again')
                return
            if self.failed:
                return self.recover_or_finish(grid, xy, 'only failed frontiers left')
            # Everything mapped: stop now, even mid-goal (finish cancels it).
            self.finish('NO_EXIT', 'all frontiers mapped, no open space found')
            self.stop_robot()
            print('nav stopped', flush=True)
            return
        self.empty_replans = 0
        nxt = choose_frontier(frontiers, self.current, yaw, xy)
        if self.current is not None and self.goal_handle is not None:
            cg = self.current.goal
            if math.hypot(nxt.goal[0] - cg[0], nxt.goal[1] - cg[1]) < 0.4:
                self.current_missing = 0
                return                              # same goal, keep driving
            # The current goal's frontier can blink out for one map update and come back
            # (live run 2026-11-15 09:39: two goals swapped every 1-3 s, each swap a Nav2
            # preempt). Keep driving it for goal_grace replans before switching.
            if not any(math.hypot(f.goal[0] - cg[0], f.goal[1] - cg[1]) < 0.4
                       for f in frontiers):
                self.current_missing += 1
                if self.current_missing < self.cfg['goal_grace']:
                    return
        self.current_missing = 0
        if self.pre_turn(grid, xy, yaw, nxt):
            return
        self.send(nxt, len(frontiers))

    def pre_turn(self, grid, xy, yaw, f):
        """Turn the robot ourselves before goal f if Nav2's own turn would hit a wall."""
        if not (self.cfg['go'] and self.cfg['pre_turn']) or not self.spin.server_is_ready():
            return False
        if self.pre_turned is not None and \
                math.hypot(f.goal[0] - self.pre_turned[0], f.goal[1] - self.pre_turned[1]) < 0.4:
            return False                            # turned for this goal already
        heading = heading_towards(grid, xy, f.goal, self.cfg['robot_radius'])
        angle, why = plan_turn(grid, xy, yaw, heading, self.box, self.cfg['turn_margin'],
                               self.cfg['pre_turn_min_angle'])
        short = math.degrees(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
        where = f'goal ({f.goal[0]:.2f}, {f.goal[1]:.2f})'
        if angle is None and why == 'blocked both ways' and self.cfg['pre_turn_max_creep'] > 0:
            # Find room straight ahead (core.plan_creep_turn), creep there, turn there.
            d, margin, why2 = plan_creep_turn(
                grid, xy, yaw, f.goal, self.box,
                margins=tuple(sorted({self.cfg['turn_margin'], 0.01}, reverse=True)),
                min_angle=self.cfg['pre_turn_min_angle'],
                max_creep=self.cfg['pre_turn_max_creep'],
                creep_margin=self.cfg['creep_margin'], robot_radius=self.cfg['robot_radius'])
            if d is None:
                self.get_logger().warn(f'{where} needs a {short:+.0f} deg turn, blocked by walls '
                                       f'both ways and {why2}; leaving it to Nav2')
                return False
            self.halt_for_turn(f)
            self.turn_creep = {'goal': f.goal, 'from': xy, 'target': d, 'margin': margin,
                               'until': time.monotonic() + d / self.cfg['creep_speed'] + 3.0}
            self.set_state('TURNING', f'{short:+.0f} deg to {where} blocked by walls both ways: '
                           f'creeping {d * 100:.0f} cm ahead to turn there ({why2})')
            return True
        if angle is None:
            return False
        self.halt_for_turn(f)
        self.set_state('TURNING', f'{short:+.0f} deg to {where} would hit a wall: turning '
                       f'{math.degrees(angle):+.0f} deg instead')
        self.send_spin(angle, 'turn before goal')
        return True

    def halt_for_turn(self, f):
        """Stop driving (cancel the Nav2 goal) before turning for goal f, once per goal."""
        self.pre_turned = f.goal
        if self.goal_handle is not None:            # stop where we are
            self.goal_handle.cancel_goal_async()
        self.goal_handle, self.current, self.goal_result = None, None, None
        self.stop_robot()

    def on_look_request(self, msg):
        """poster_detector: a QR room with no poster yet; look around there once."""
        self.looks.append((msg, time.monotonic()))
        self.get_logger().info(f'look-around requested at {msg.header.frame_id} '
                               f'({msg.point.x:.2f}, {msg.point.y:.2f})')

    def maybe_look(self, grid, xy, yaw):
        """
        EXPLORING: near a requested point, stop and sweep left and right (as far as
        look_angle or until the body would touch a wall, like the start scan) so the camera
        sees the whole room. Returns True if a look-around started.
        """
        now = time.monotonic()
        self.looks = [(m, t) for m, t in self.looks if now - t < self.cfg['look_timeout']]
        if not self.looks or not self.cfg['go'] or self.state != 'EXPLORING' or \
                self.spin_busy or self.turn_creep is not None or \
                not self.spin.server_is_ready():
            return False
        for i, (msg, _) in enumerate(self.looks):
            try:   # requests are in odom; the map <- odom correction moves with SLAM
                tf = self.tf.lookup_transform('map', msg.header.frame_id, Time())
            except TransformException:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            px = t.x + math.cos(th) * msg.point.x - math.sin(th) * msg.point.y
            py = t.y + math.sin(th) * msg.point.x + math.cos(th) * msg.point.y
            if math.hypot(px - xy[0], py - xy[1]) > self.cfg['look_radius']:
                continue
            del self.looks[i]
            a, m = abs(self.cfg['look_angle']), self.cfg['turn_margin']
            left = max_turn(grid, xy, yaw, self.box, +1, a, m)
            right = max_turn(grid, xy, yaw, self.box, -1, a, m)
            turns = [t for t in (left, -(left + right), right) if abs(t) > math.radians(5)]
            if not turns:
                self.get_logger().warn('look-around skipped: walls too close to turn')
                return False
            self.halt_for_look()
            self.look_turns = turns
            self.set_state('LOOKING', f'QR room at ({px:.2f}, {py:.2f}) with no survivor '
                           f'poster yet: {math.degrees(left):.0f} deg left, '
                           f'{math.degrees(left + right):.0f} right, {math.degrees(right):.0f} '
                           'left')
            return True
        return False

    def on_qr_unread(self, msg):
        """qr_scanner: a QR code in view that it can't read (position in odom)."""
        self.unread = (msg, time.monotonic())

    def on_qr_read(self, _msg):
        self.qr_read_at = time.monotonic()

    def odom_to_map(self, msg):
        """(x, y) in map of a PointStamped in another fixed frame (latest TF), or None."""
        try:
            tf = self.tf.lookup_transform('map', msg.header.frame_id, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.x + math.cos(th) * msg.point.x - math.sin(th) * msg.point.y,
                t.y + math.sin(th) * msg.point.x + math.cos(th) * msg.point.y)

    def face_turn(self, grid, xy, yaw, target):
        """Turn (rad, + = left) towards target, cut to what the walls allow (max_turn)."""
        want = wrap(math.atan2(target[1] - xy[1], target[0] - xy[0]) - yaw)
        if abs(want) < self.cfg['face_min_angle']:
            return 0.0, want
        sign = 1 if want > 0 else -1
        ok = max_turn(grid, xy, yaw, self.box, sign, abs(want), self.cfg['turn_margin'])
        return (sign * ok if ok > math.radians(5) else 0.0), want

    def maybe_face(self, grid, xy, yaw):
        """
        EXPLORING: a QR code is in view but unreadable: stop and turn to face it (FACING_QR)
        so the camera sees it squarely, until qr_scanner reads it. Returns True if started.
        """
        now = time.monotonic()
        self.face_given_up = [(p, t) for p, t in self.face_given_up
                              if now - t < self.cfg['face_cooldown']]
        if self.unread is None or now - self.unread[1] > 1.5 or not self.cfg['go'] or \
                self.state != 'EXPLORING' or self.spin_busy or self.turn_creep is not None \
                or now - self.qr_read_at < 1.0 or not self.spin.server_is_ready():
            return False
        target = self.odom_to_map(self.unread[0])
        if target is None or any(math.hypot(target[0] - p[0], target[1] - p[1]) < 0.75
                                 for p, _ in self.face_given_up):
            return False
        turn, want = self.face_turn(grid, xy, yaw, target)
        self.halt_for_look()
        self.face = {'since': now, 'turns': 0, 'target': target}
        what = (f'turning {math.degrees(turn):+.0f} deg' if turn else
                'already facing it' if abs(want) < self.cfg['face_min_angle'] else
                f'walls block the {math.degrees(want):+.0f} deg turn, holding')
        self.set_state('FACING_QR', f'QR code at ({target[0]:.2f}, {target[1]:.2f}) in view '
                       f'but unreadable: {what} until it is read')
        if turn:
            self.face['turns'] = 1
            self.send_spin(turn, 'face QR code')
        return True

    def face_tick(self, pose):
        """FACING_QR: wait for the read; re-aim if it drifts; give up after face_timeout."""
        now, f = time.monotonic(), self.face
        if f is None:
            return self.set_state('EXPLORING', 'facing QR code interrupted; going on')
        if self.qr_read_at > f['since']:
            self.face = None
            return self.set_state('EXPLORING', 'QR code read; going on')
        if self.spin_busy:
            return
        if now - f['since'] > self.cfg['face_timeout']:
            self.face_given_up.append((f['target'], now))
            self.face = None
            self.get_logger().warn(f'QR code at ({f["target"][0]:.2f}, {f["target"][1]:.2f}) '
                                   f'still unreadable after {self.cfg["face_timeout"]:.0f} s; '
                                   f'ignoring it for {self.cfg["face_cooldown"]:.0f} s')
            return self.set_state('EXPLORING', 'QR code not read in time; going on')
        if self.unread is None or now - self.unread[1] > self.cfg['face_lost']:
            self.face = None
            return self.set_state('EXPLORING', 'unreadable QR code no longer in view; going on')
        target = self.odom_to_map(self.unread[0])
        if target is None or f['turns'] >= self.cfg['face_max_turns']:
            return
        xy, yaw = pose
        turn, want = self.face_turn(Grid.from_msg(self.map), xy, yaw, target)
        if turn and abs(want) > math.radians(15):
            f['turns'] += 1
            f['target'] = target
            self.send_spin(turn, 'face QR code again')

    def halt_for_look(self):
        """Stop driving (cancel the Nav2 goal) before a look-around."""
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle, self.current, self.goal_result = None, None, None
        self.stop_robot()

    def end_turn_creep(self, why):
        """Crept ahead for a blocked turn: turn now, if it is clear from here."""
        tc, self.turn_creep = self.turn_creep, None
        self.stop_robot()
        self.get_logger().info(f'turn creep: {why}')
        pose = self.robot_pose()
        if pose is None or self.map is None:
            return                                  # step() moves on to EXPLORING
        xy, yaw = pose
        grid = Grid.from_msg(self.map)
        heading = heading_towards(grid, xy, tc['goal'], self.cfg['robot_radius'])
        want = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        if abs(want) < math.radians(5):
            return
        m, s = tc['margin'], (1.0 if want >= 0 else -1.0)
        other = 2 * math.pi - abs(want)
        if max_turn(grid, xy, yaw, self.box, s, abs(want), m) >= abs(want) - 1e-6:
            angle = want
        elif max_turn(grid, xy, yaw, self.box, -s, other, m) >= other - 1e-6:
            angle = -s * other
        else:
            self.get_logger().warn(f'turn creep: {math.degrees(want):+.0f} deg still blocked '
                                   'here; leaving it to Nav2')
            return
        self.send_spin(angle, 'turn after creep')

    def begin(self):
        self.set_state('EXPLORING', 'go' if self.cfg['go'] else 'DRY RUN (go:=false)')

    def find(self, grid, xy, exclude):
        return find_frontiers(grid, xy, self.cfg['robot_radius'], self.cfg['goal_clearance'],
                              self.cfg['min_frontier_size'], exclude,
                              min_width=self.min_frontier_width())

    def in_reach(self, frontiers, xy):
        off = self.cfg['goal_max_offset']
        return [f for f in frontiers
                if abs(f.goal[0] - xy[0]) < off and abs(f.goal[1] - xy[1]) < off]

    def recover_or_finish(self, grid, xy, why):
        """Goals keep failing: retry while open frontiers remain, else NO_EXIT."""
        open_ = self.in_reach(self.find(grid, xy, self.blacklist), xy)
        if not open_:
            self.finish('NO_EXIT', f'{why}; no open frontier left')
            self.stop_robot()
            return
        if self.recoveries >= self.cfg['max_recoveries']:
            self.finish('NO_EXIT', f'{why}; {len(open_)} frontiers still open but unreachable '
                        f'after {self.recoveries} recoveries')
            self.stop_robot()
            return
        self.recoveries += 1
        self.get_logger().warn(
            f'{why}, but {len(open_)} frontiers are still open: recovery '
            f'{self.recoveries}/{self.cfg["max_recoveries"]} - clearing costmaps, retrying them')
        for c in self.clear_costmaps:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
        self.failed, self.failures, self.empty_replans = [], 0, 0

    def scan_or_begin(self):
        if self.cfg['start_scan']:
            self.set_state('SCANNING', 'turning left and right to see the start area')
        else:
            self.begin()

    def plan_scan(self, pose):
        """Left, right, back: each side as far as scan_angle or until the body would touch a
        wall on the map, so it ends on the heading it started with."""
        xy, yaw = pose
        grid, a = Grid.from_msg(self.map), abs(self.cfg['scan_angle'])
        m = self.cfg['turn_margin']
        left = max_turn(grid, xy, yaw, self.box, +1, a, m)
        right = max_turn(grid, xy, yaw, self.box, -1, a, m)
        dl, dr = math.degrees(left), math.degrees(right)
        if left < math.radians(5) and right < math.radians(5):
            self.get_logger().warn(f'start scan skipped: walls too close to turn '
                                   f'({dl:.0f} deg left, {dr:.0f} deg right possible)')
            self.scan_turns = []
            return
        if left < a or right < a:
            self.get_logger().warn(f'start scan limited by walls: {dl:.0f} deg left, '
                                   f'{dr:.0f} deg right')
        self.get_logger().info(f'start scan: {dl:.0f} deg left, {dl + dr:.0f} right, '
                               f'{dr:.0f} left')
        self.scan_turns = [t for t in (left, -(left + right), right) if abs(t) > 1e-3]

    def creep_or_scan(self, pose):
        if self.cfg['creep_distance'] > 0:
            self.begin_creep(pose)
        else:
            self.scan_or_begin()

    def begin_creep(self, pose):
        xy, yaw = pose
        d = self.cfg['creep_distance']
        grid = Grid.from_msg(self.map)
        self.creep_from = xy
        # Shorter if a wall ahead is close (e.g. the front 0.15 m from a wall): end with
        # creep_margin to spare.
        free = free_distance_ahead(grid, xy, yaw, self.box, d,
                                   self.cfg['creep_margin'])
        if free < d:
            self.get_logger().warn(f'creep shortened to {free * 100:.0f} cm: wall ahead')
        if free < 0.02:
            self.creep_target = 0.0
            self.set_state('CREEPING', 'skipped: wall right ahead')
            return self.end_creep('creep skipped: wall right ahead')
        self.creep_target = free
        self.creep_until = time.monotonic() + free / self.cfg['creep_speed'] + 3.0
        self.set_state('CREEPING', f'{free * 100:.0f} cm straight ahead, no steering')

    def creep_tick(self):
        tc = self.turn_creep
        if tc is not None and self.state != 'TURNING':
            self.turn_creep = None                  # taken over (STAIRS) or finished
            self.stop_robot()
            return
        if self.state != 'CREEPING' and tc is None:
            return
        start, target, until = ((tc['from'], tc['target'], tc['until']) if tc is not None
                                else (self.creep_from, self.creep_target, self.creep_until))
        pose = self.robot_pose()
        done, moved = None, 0.0
        if pose is not None:
            moved = math.hypot(pose[0][0] - start[0], pose[0][1] - start[1])
            if moved >= target:
                done = f'crept {moved * 100:.0f} cm'
        if done is None and time.monotonic() > until:
            done = f'creep timed out after {moved * 100:.0f} cm'
        if done is None:
            msg = Twist()
            msg.linear.x = float(self.cfg['creep_speed'])   # angular stays 0: no steering
            self.cmd_pub.publish(msg)
            return
        if tc is not None:
            return self.end_turn_creep(done)
        self.end_creep(done)

    def end_creep(self, why):
        self.stop_robot()
        self.get_logger().info(why)
        # The lidar never saw behind the start pose, so that patch reads as a big frontier
        # (seen in the sim even after a 360 deg turn). It is the start cell's back, not a way
        # on: an earlier run drove back into the dead-end start cell and got stuck turning.
        self.blacklist.append(self.creep_from)
        self.scan_or_begin()

    def send_spin(self, angle, why='start scan'):
        self.spin_busy = True
        self.spin_why = why
        # behavior_server min_rotational_vel is 0.2 rad/s; allow that plus margin
        allowance = abs(angle) / 0.2 + 5.0
        goal = Spin.Goal(target_yaw=float(angle),
                         time_allowance=Duration(sec=int(math.ceil(allowance))))
        self.get_logger().info(f'{why}: turn {math.degrees(angle):+.0f} deg')
        self.spin.send_goal_async(goal).add_done_callback(self.on_spin_accepted)

    def on_spin_accepted(self, future):
        handle = future.result()
        if self.state not in ('SCANNING', 'TURNING', 'LOOKING', 'FACING_QR'):   # stopped meanwhile
            if handle.accepted:
                handle.cancel_goal_async()
            self.spin_busy = False
            return
        if not handle.accepted:
            self.get_logger().warn(f'{self.spin_why}: turn rejected; continuing')
            self.spin_busy = False
            return
        self.spin_handle = handle
        handle.get_result_async().add_done_callback(self.on_spin_result)

    def on_spin_result(self, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'{self.spin_why}: turn ended with status {status}; continuing')
        self.spin_handle = None
        self.spin_busy = False

    def send(self, f, n):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = f.goal
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(f.yaw / 2), math.cos(f.yaw / 2)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'frontier goal ({f.goal[0]:.2f}, {f.goal[1]:.2f}) '
                               f'{f.path_length:.1f} m away, {f.width:.2f} m wide, {f.size} cells; '
                               f'{n} frontiers')
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.current = f
        if not self.cfg['go']:
            return
        self.goal_sent_at = time.monotonic()
        self.goal_result = None
        future = self.nav.send_goal_async(NavigateToPose.Goal(pose=msg))
        future.add_done_callback(lambda fut, f=f: self.on_accepted(fut, f))

    def on_accepted(self, future, f):
        handle = future.result()
        if f is not self.current:                   # superseded while being accepted
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.goal_result = GoalStatus.STATUS_ABORTED
            self.goal_result_at = time.monotonic()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda fut, f=f: self.on_result(fut, f))

    def on_result(self, future, f):
        if f is self.current:
            self.goal_result = future.result().status
            self.goal_result_at = time.monotonic()

    def finish(self, state, why):
        if self.state == 'CREEPING' or self.turn_creep is not None:
            self.stop_robot()
        self.turn_creep = None
        if self.spin_handle is not None:
            self.spin_handle.cancel_goal_async()
            self.spin_handle = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.current = None
        self.save_map()
        self.set_state(state, why)

    def stop_robot(self):
        if self.cfg['go']:
            for _ in range(3):                      # a few in case one is dropped
                self.cmd_pub.publish(Twist())

    def save_map(self):
        if self.map is None:
            return
        try:
            from maze_nav.map_io import save_map
            path = save_map(self.map, self.cfg['map_dir'])
            self.get_logger().info(f'map saved: {path}')
        except Exception as e:                      # never let saving stop the shutdown
            self.get_logger().error(f'map not saved: {e}')


def main():
    # rclpy's default SIGINT handler shuts the context down before we get control back
    # (rclpy.ok() is already False), which would skip the goal cancel and the map save.
    # Same pattern as cmd_vel_bridge: catch the signal ourselves and stop the loop.
    stop = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = Explorer()
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node.state not in ('EXITED', 'NO_EXIT', 'STOPPED') and rclpy.ok():
            node.finish('STOPPED', 'shutdown')
            end = time.monotonic() + 0.3     # let the cancel requests and status go out
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
