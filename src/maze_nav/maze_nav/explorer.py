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
Nav2 down for `nav_down_timeout` s (a server crashed and the lifecycle manager deactivated
the rest; Humble doesn't bring it back) -> STOPPED, not NO_EXIT.
Nothing left to explore and no exit -> DONE (no exit found): the running goal is cancelled
at once, a zero Twist goes to nav_cmd_vel and "nav stopped" is printed. Either way Nav2 is
cancelled, the map is saved and /maze/status says why. Ctrl+C / SIGTERM (also from
ros2 launch) ends the run as STOPPED the same way: rclpy's own signal handler is off so
the context is still valid for the cancel and the final status.

Junction photos (`junction_photos`): when the two nearest frontiers are about equally far
(core.find_junction: both within `junction_range`, path lengths within `junction_dist_tol`,
ways >= `junction_min_angle` apart), the robot stops (status JUNCTION), turns to face the
first way (the smaller turn), waits `junction_settle` s, takes a colour image, does the same
for the second way, saves both plus a JSON (pose, headings, frontier goals, image files) in
`junction_dir`/run_<time>/, then explores on as before. Each junction once (`junction_revisit`).
Turns use Nav2's spin, cut short where the footprint would hit a mapped wall. No image within
`image_timeout` s: logged, recorded as null, the run goes on. Skipped with go:=false.

Status on /maze/status (std_msgs/String, latched): STARTING, PAUSED, CREEPING, SCANNING, JUNCTION, EXPLORING, EXITED, NO_EXIT,
STOPPED. Goals on /maze/goal (PoseStamped) for RViz. `go` parameter false = plan and publish
goals but never send them to Nav2 (dry run on the robot).
"""
import json
import math
import os
import signal
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import (ExitDetector, Grid, choose_frontier, find_frontiers,
                           find_junction, footprint_hits, free_distance_ahead, max_turn)

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
        p('creep_margin', 0.05)        # m kept clear of walls ahead at the end of the creep
        p('goal_max_offset', 7.5)      # m in x and y from the robot; global costmap is 16 x 16 rolling
        p('goal_grace', 3)             # replans the current goal's frontier may be missing
        #                                before switching goals (a blink isn't a switch)
        p('nav_down_timeout', 5.0)     # s of Nav2 reported inactive before giving up (STOPPED)
        p('map_dir', os.path.expanduser('~/challenge_ws/maps'))
        p('junction_photos', True)     # at a junction: face each way and take a picture
        p('junction_range', 1.5)       # m along free space: both ways' frontiers within this
        p('junction_dist_tol', 0.5)    # m: their path lengths differ by at most this
        p('junction_min_angle', 60.0)  # deg between the two ways, seen from the robot
        p('junction_revisit', 1.0)     # m: a junction this close to a photographed one is skipped
        p('junction_settle', 0.7)      # s still after a turn before the picture
        p('image_topic', '/camera/color/image_raw/compressed')   # CompressedImage (JPEG)
        p('image_raw_topic', '/camera/color/image_raw')          # Image, used if no JPEG comes
        p('image_timeout', 3.0)        # s to wait for a picture
        p('junction_dir', os.path.expanduser('~/challenge_ws/logs/junctions'))
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in ('go', 'start_delay', 'wait_for_start', 'robot_radius', 'goal_clearance',
                                      'min_frontier_size', 'robot_width', 'min_frontier_width_ratio', 'goal_timeout', 'max_failures', 'done_confirm', 'max_recoveries',
                                      'max_runtime', 'map_dir', 'creep_distance', 'creep_speed',
                                      'start_scan', 'scan_angle', 'footprint_back',
                                      'footprint_front', 'footprint_half_width', 'turn_margin',
                                      'creep_margin', 'goal_max_offset', 'goal_grace',
                                      'nav_down_timeout',
                                      'junction_photos', 'junction_range', 'junction_dist_tol',
                                      'junction_min_angle', 'junction_revisit', 'junction_settle',
                                      'image_topic', 'image_raw_topic', 'image_timeout',
                                      'junction_dir')}
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
        self.create_timer(g('replan_period'), self.step)
        self.create_timer(0.1, self.creep_tick)   # faster than cmd_vel_bridge's idle_timeout

        self.junctions = []          # (x, y) of photographed junctions
        self.jn = None               # the junction being photographed
        self.img_subs = []
        self.img_msg = None
        self.run_dir = os.path.join(self.cfg['junction_dir'], time.strftime('run_%Y%m%d_%H%M%S'))

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
        self.creep_from = None       # (x, y) in map where the creep started
        self.creep_until = None      # monotonic deadline for the creep
        self.goal_result_at = None   # monotonic time goal_result arrived
        self.nav_query = None        # (future, sent_at) of the pending is_active call
        self.nav_reply_at = -1.0     # sent_at of the newest is_active answer
        self.nav_active = True
        self.nav_down_since = None
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
        elif self.state == 'STARTING':
            self.get_logger().warn('start ignored: not ready yet (wait for PAUSED)')

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
                log = self.get_logger().info if active else self.get_logger().error
                log('Nav2 is active again' if active else
                    'Nav2 is NOT active (a server died? see the Nav2 log above)')
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
        if self.state == 'JUNCTION':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            return                                  # junction_tick drives it
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
        if self.cfg['junction_photos'] and self.cfg['go'] and \
                not any(math.hypot(xy[0] - j[0], xy[1] - j[1]) < self.cfg['junction_revisit']
                        for j in self.junctions):
            ways = find_junction(grid, xy, yaw, frontiers, self.cfg['robot_radius'],
                                 self.cfg['junction_range'], self.cfg['junction_dist_tol'],
                                 math.radians(self.cfg['junction_min_angle']))
            if ways:
                return self.begin_junction(ways, xy, yaw)
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
        self.send(nxt, len(frontiers))

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

    # ------------------------------------------------------------------ junction photos
    def begin_junction(self, ways, xy, yaw):
        if self.goal_handle is not None:            # stop where we are
            self.goal_handle.cancel_goal_async()
        self.goal_handle, self.current, self.goal_result = None, None, None
        self.stop_robot()
        n = len(self.junctions) + 1
        self.junctions.append(xy)
        self.jn = {'n': n, 'xy': xy, 'yaw': yaw, 'ways': ways, 'k': 0, 'phase': 'turn',
                   'sent': False, 't': time.monotonic(), 'records': []}
        desc = ', '.join(f'{math.degrees(h):+.0f} deg to ({f.goal[0]:.2f}, {f.goal[1]:.2f}) '
                         f'{f.path_length:.1f} m' for f, h in ways)
        self.set_state('JUNCTION', f'junction {n} at ({xy[0]:.2f}, {xy[1]:.2f}): {desc}')

    def junction_tick(self):
        j, now = self.jn, time.monotonic()
        f, heading = j['ways'][j['k']]
        if j['phase'] == 'turn':
            if self.spin_busy:
                return
            if j['sent']:                           # turn finished (or failed: logged)
                j['phase'], j['t'] = 'settle', now
                return
            pose = self.robot_pose()
            if pose is None:
                return
            xy, yaw = pose
            want = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
            sign, grid, m = (1.0 if want >= 0 else -1.0), Grid.from_msg(self.map), self.cfg['turn_margin']
            if footprint_hits(grid, xy, yaw, self.box, m):
                # A wall where the robot stands is a map artefact (stray cells, seen in the
                # sim); the check would block every turn. Rely on Nav2 spin's own check.
                self.get_logger().warn('junction: map shows a wall at the robot; turning '
                                       'without the map check (Nav2 spin still checks)')
                can = abs(want)
            else:
                can = max_turn(grid, xy, yaw, self.box, sign, abs(want), m)
                if can < abs(want) - math.radians(3):
                    other = 2 * math.pi - abs(want)         # the long way round
                    if max_turn(grid, xy, yaw, self.box, -sign, other, m) >= other - 1e-6:
                        self.get_logger().info(f'junction: short turn blocked by a wall; '
                                               f'turning {math.degrees(other):.0f} deg the other way')
                        sign, can = -sign, other
                    else:
                        self.get_logger().warn(f'junction: turn cut to {math.degrees(can):.0f} of '
                                               f'{math.degrees(abs(want)):.0f} deg by a wall')
            j['sent'] = True
            if can >= math.radians(3):
                self.send_spin(sign * can, 'junction')
        elif j['phase'] == 'settle':
            if now - j['t'] >= self.cfg['junction_settle']:
                self.img_msg = None
                self.img_subs = [
                    self.create_subscription(CompressedImage, self.cfg['image_topic'],
                                             self.on_image, qos_profile_sensor_data),
                    self.create_subscription(Image, self.cfg['image_raw_topic'],
                                             self.on_image, qos_profile_sensor_data)]
                j['phase'], j['t'] = 'capture', now
        elif j['phase'] == 'capture':
            if self.img_msg is None and now - j['t'] < self.cfg['image_timeout']:
                return
            self.drop_image_subs()
            pose = self.robot_pose()
            name = f'j{j["n"]:02d}_{j["k"] + 1}_{math.degrees(heading):+04.0f}deg'
            path = None
            if self.img_msg is None:
                self.get_logger().warn(f'junction {j["n"]}: no image within '
                                       f'{self.cfg["image_timeout"]} s; going on without it')
            else:
                try:
                    path = save_image(self.img_msg, self.run_dir, name)
                    self.get_logger().info(f'junction {j["n"]}: picture {j["k"] + 1} -> {path}')
                except Exception as e:              # never let a picture stop the run
                    self.get_logger().error(f'junction {j["n"]}: image not saved: {e}')
            j['records'].append({
                'way': j['k'] + 1, 'heading_deg': round(math.degrees(heading), 1),
                'facing_deg': None if pose is None else round(math.degrees(pose[1]), 1),
                'frontier_goal': [round(f.goal[0], 3), round(f.goal[1], 3)],
                'path_length_m': round(f.path_length, 2),
                'image': None if path is None else os.path.basename(path),
                'time': time.strftime('%Y-%m-%dT%H:%M:%S')})
            j['k'] += 1
            if j['k'] < len(j['ways']):
                j['phase'], j['sent'] = 'turn', False
                return
            self.end_junction()

    def on_image(self, msg):
        if self.img_msg is None:
            self.img_msg = msg

    def drop_image_subs(self):
        for sub in self.img_subs:
            self.destroy_subscription(sub)
        self.img_subs = []

    def end_junction(self):
        j, self.jn = self.jn, None
        meta = {'junction': j['n'], 'frame': 'map',
                'robot': {'x': round(j['xy'][0], 3), 'y': round(j['xy'][1], 3),
                          'yaw_deg': round(math.degrees(j['yaw']), 1)},
                'ways': j['records']}
        try:
            os.makedirs(self.run_dir, exist_ok=True)
            path = os.path.join(self.run_dir, f'j{j["n"]:02d}.json')
            with open(path, 'w') as fh:
                json.dump(meta, fh, indent=2)
            self.get_logger().info(f'junction {j["n"]}: saved {path}')
        except Exception as e:
            self.get_logger().error(f'junction {j["n"]}: metadata not saved: {e}')
        self.set_state('EXPLORING', f'junction {j["n"]} photographed')

    def creep_tick(self):
        if self.state == 'JUNCTION':
            return self.junction_tick()
        if self.state != 'CREEPING':
            return
        pose = self.robot_pose()
        done, moved = None, 0.0
        if pose is not None:
            moved = math.hypot(pose[0][0] - self.creep_from[0], pose[0][1] - self.creep_from[1])
            if moved >= self.creep_target:
                done = f'crept {moved * 100:.0f} cm'
        if done is None and time.monotonic() > self.creep_until:
            done = f'creep timed out after {moved * 100:.0f} cm'
        if done is None:
            msg = Twist()
            msg.linear.x = float(self.cfg['creep_speed'])   # angular stays 0: no steering
            self.cmd_pub.publish(msg)
            return
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
        if self.state not in ('SCANNING', 'JUNCTION'):   # stopped while being accepted
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
        if self.state in ('CREEPING', 'JUNCTION'):
            self.stop_robot()
        self.drop_image_subs()
        if self.jn is not None and self.jn['records']:
            self.end_junction()                     # keep the pictures already taken
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


def save_image(msg, directory, name):
    """Write a CompressedImage as-is (.jpg/.png) or a raw Image as PNG; return the path."""
    os.makedirs(directory, exist_ok=True)
    if isinstance(msg, CompressedImage):
        ext = '.png' if 'png' in msg.format.lower() else '.jpg'
        path = os.path.join(directory, name + ext)
        with open(path, 'wb') as fh:
            fh.write(bytes(msg.data))
        return path
    import numpy as np
    from PIL import Image as PILImage
    enc = msg.encoding.lower()
    ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(enc)
    if ch is None:
        raise ValueError(f'unsupported image encoding {msg.encoding}')
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    a = a[:, :msg.width * ch].reshape(msg.height, msg.width, ch) if ch > 1 else a[:, :msg.width]
    if enc.startswith('bgr'):
        a = a[..., [2, 1, 0] + ([3] if ch == 4 else [])]
    path = os.path.join(directory, name + '.png')
    PILImage.fromarray(a).save(path)
    return path


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
