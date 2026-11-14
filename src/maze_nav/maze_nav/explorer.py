"""Explore the maze on the SLAM map with Nav2 until the robot is out.

Start: once /map, TF and Nav2 are up, the robot holds still (status PAUSED) until the
operator publishes on /maze/start (std_msgs/Empty):
    ros2 topic pub --once /maze/start std_msgs/msg/Empty
so there is time to start a rosbag recording. Messages sent before PAUSED are ignored.
wait_for_start:=false skips the wait (simulator). Then it creeps `creep_distance` towards the
open frontier (status CREEPING): it picks the frontier the explorer would go to (not the
unmapped patch behind the start pose) and walks forward, steering (at most `creep_max_turn`
rad/s) the way free space leads there; no turn on the spot, which could hit the back wall
(see below). The lidar sees no walls behind the robot, so the SLAM map ends at about the
start pose and the robot can sit a few cm off it (seen live: base_link 2-3 cm behind the map
edge, "Robot is out of bounds of the costmap"). The creep also takes the robot off the start
cell's back wall: placed with its rear ~8 cm from it, turning on the spot hit it (spin
"Collision Ahead", RPP "detected collision ahead"). Humble's Smac planner crashes when asked
to plan from outside the global costmap; that is now ruled out by the rolling global
costmap in nav2_params.yaml, not by the creep. Then a scan (status SCANNING, Nav2's /spin
behavior, `start_scan`) turns `scan_angle` left, twice that right and `scan_angle` left again,
back to the heading it had before the scan, so SLAM sees the start area. At 45 deg the rear
corners swing ~5 cm further back; the creep leaves more than that to the back wall. A failed
turn is logged and the run moves on; it never blocks exploring. Creep and scan are skipped
with go:=false.

Loop (period `replan_period`):
  1. read /map (slam_toolbox) and the robot pose (TF map -> base_link)
  2. exit check: was in corridors, has moved >= min_displacement, and a disk of seen-free
     floor >= exit_open_radius lies within exit_near_radius of it (and away from the
     start) for `dwell` s -> DONE
  3. frontier = free cell next to unknown; pick the nearest along free space, keep the
     current one unless another is clearly closer (no flip-flopping)
  4. send it to Nav2 /navigate_to_pose; a finished goal (reached or failed) is blacklisted,
     except one that failed while Nav2 was down (lifecycle manager is_active = false)
Nav2 down for `nav_down_timeout` s (a server crashed and the lifecycle manager deactivated
the rest; Humble doesn't bring it back) -> STOPPED, not NO_EXIT.
Nothing left to explore and no exit -> DONE (no exit found): the running goal is cancelled
at once, a zero Twist goes to nav_cmd_vel and "nav stopped" is printed. Either way Nav2 is
cancelled, the map is saved and /maze/status says why. Ctrl+C / SIGTERM (also from
ros2 launch) ends the run as STOPPED the same way: rclpy's own signal handler is off so
the context is still valid for the cancel and the final status.

Status on /maze/status (std_msgs/String, latched): STARTING, PAUSED, CREEPING, SCANNING, EXPLORING, EXITED, NO_EXIT,
STOPPED. Goals on /maze/goal (PoseStamped) for RViz. `go` parameter false = plan and publish
goals but never send them to Nav2 (dry run on the robot).
"""
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
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import ExitDetector, Grid, choose_frontier, find_frontiers, heading_towards

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
        p('exit_open_radius', 1.1)     # m of seen-free floor around a point = outside
        p('exit_near_radius', 0.8)     # that point within this distance of the robot
        p('exit_min_displacement', 1.5)
        p('exit_dwell', 2.0)
        p('max_runtime', 900.0)        # s; hard stop
        p('creep_distance', 0.10)      # m towards the open frontier after start, onto the map and
        #                                off the back wall so the scan turns clear it; 0 = off
        p('creep_speed', 0.05)         # m/s
        p('creep_max_turn', 0.3)       # rad/s steering while creeping (no turn on the spot)
        p('start_scan', True)          # after the creep: scan_angle left, 2x right, left again
        p('scan_angle', math.pi / 4)   # rad
        p('goal_max_offset', 7.5)      # m in x and y from the robot; global costmap is 16 x 16 rolling
        p('nav_down_timeout', 5.0)     # s of Nav2 reported inactive before giving up (STOPPED)
        p('map_dir', os.path.expanduser('~/challenge_ws/maps'))
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in ('go', 'start_delay', 'wait_for_start', 'robot_radius', 'goal_clearance',
                                      'min_frontier_size', 'robot_width', 'min_frontier_width_ratio', 'goal_timeout', 'max_failures', 'done_confirm',
                                      'max_runtime', 'map_dir', 'creep_distance', 'creep_speed',
                                      'creep_max_turn', 'start_scan', 'scan_angle', 'goal_max_offset', 'nav_down_timeout')}
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

        self.map = None
        self.first_map = None
        self.started = None
        self.start_requested = False
        self.state = None
        self.current = None          # Frontier being driven to
        self.goal_handle = None
        self.goal_sent_at = None
        self.goal_result = None      # None while running, else GoalStatus code
        self.blacklist = []
        self.failures = 0
        self.empty_replans = 0       # replans in a row that found no frontier
        a = abs(self.cfg['scan_angle'])
        self.scan_turns = [a, -2 * a, a]   # left, right, back to the heading before the scan
        self.spin_busy = False       # scan turn sent, result not back yet
        self.spin_handle = None
        self.creep_from = None       # (x, y) in map where the creep started
        self.creep_heading = None    # map-frame heading the creep steers to
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
            elif self.cfg['creep_distance'] > 0:
                self.begin_creep(pose)
            else:
                self.scan_or_begin()
        if self.state == 'CREEPING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            return                                  # creep_tick drives, then moves on
        if self.state == 'SCANNING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.spin_busy:
                return
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
            if self.current is not None:
                # Done goals are blacklisted too: a frontier still there after the robot
                # stood on it can't be cleared from there (e.g. unknown seen through a slit
                # at a wall corner, which the example maze has next to the start cell).
                self.blacklist.append(self.current.goal)
            if not ok:
                self.failures += 1
                self.get_logger().warn(
                    f'goal ({self.current.goal[0]:.2f}, {self.current.goal[1]:.2f}) failed '
                    f'(status {self.goal_result}); blacklisted')
            else:
                self.failures = 0
            self.goal_result, self.goal_handle, self.current = None, None, None
        elif self.goal_handle is not None and \
                time.monotonic() - self.goal_sent_at > self.cfg['goal_timeout']:
            self.get_logger().warn('goal timed out; cancelling')
            self.goal_handle.cancel_goal_async()
            return

        if self.failures >= self.cfg['max_failures']:
            return self.finish('NO_EXIT', f'{self.failures} goals failed in a row')

        if not self.nav_active:
            return                                  # no goals into a dead Nav2
        frontiers = find_frontiers(grid, xy, self.cfg['robot_radius'], self.cfg['goal_clearance'],
                                   self.cfg['min_frontier_size'], self.blacklist,
                                   min_width=self.min_frontier_width())
        # Smac (Humble) doesn't check the goal is inside the rolling global costmap either.
        off = self.cfg['goal_max_offset']
        reach = [f for f in frontiers
                 if abs(f.goal[0] - xy[0]) < off and abs(f.goal[1] - xy[1]) < off]
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
            # Everything mapped: stop now, even mid-goal (finish cancels it).
            self.finish('NO_EXIT', 'all frontiers mapped, no open space found')
            self.stop_robot()
            print('nav stopped', flush=True)
            return
        self.empty_replans = 0
        nxt = choose_frontier(frontiers, self.current, yaw, xy)
        if self.current is not None and self.goal_handle is not None and \
                math.hypot(nxt.goal[0] - self.current.goal[0],
                           nxt.goal[1] - self.current.goal[1]) < 0.4:
            return                                  # same goal, keep driving
        self.send(nxt, len(frontiers))

    def begin(self):
        self.set_state('EXPLORING', 'go' if self.cfg['go'] else 'DRY RUN (go:=false)')

    def scan_or_begin(self):
        if self.cfg['start_scan']:
            a = math.degrees(abs(self.cfg['scan_angle']))
            self.set_state('SCANNING', f'turning {a:.0f} deg left, {2 * a:.0f} right, '
                           f'{a:.0f} left to see the start area')
        else:
            self.begin()

    def begin_creep(self, pose):
        xy, yaw = pose
        d = self.cfg['creep_distance']
        # Head for the frontier the explorer would pick, minus the unmapped patch behind the
        # start pose (blacklisted: it is the start cell's back, not a way on).
        grid = Grid.from_msg(self.map)
        fs = find_frontiers(grid, xy, self.cfg['robot_radius'], self.cfg['goal_clearance'],
                            self.cfg['min_frontier_size'], [xy],
                            min_width=self.min_frontier_width())
        f = choose_frontier(fs, None, yaw, xy)
        if f is None:
            self.creep_heading, where = yaw, 'straight ahead (no frontier yet)'
        else:
            self.creep_heading = heading_towards(grid, xy, f.goal, self.cfg['robot_radius'])
            where = (f'towards frontier ({f.goal[0]:.2f}, {f.goal[1]:.2f}), '
                     f'{math.degrees(wrap(self.creep_heading - yaw)):+.0f} deg off the nose')
        self.creep_from = xy
        self.creep_until = time.monotonic() + d / self.cfg['creep_speed'] + 3.0
        self.set_state('CREEPING', f'{d * 100:.0f} cm {where}')

    def creep_tick(self):
        if self.state != 'CREEPING':
            return
        pose = self.robot_pose()
        done, moved = None, 0.0
        if pose is not None:
            moved = math.hypot(pose[0][0] - self.creep_from[0], pose[0][1] - self.creep_from[1])
            if moved >= self.cfg['creep_distance']:
                done = f'crept {moved * 100:.0f} cm'
        if done is None and time.monotonic() > self.creep_until:
            done = f'creep timed out after {moved * 100:.0f} cm'
        if done is None:
            msg = Twist()
            msg.linear.x = float(self.cfg['creep_speed'])
            if pose is not None:                    # steer towards the frontier, gently
                lim = float(self.cfg['creep_max_turn'])
                msg.angular.z = max(-lim, min(lim, 1.5 * wrap(self.creep_heading - pose[1])))
            self.cmd_pub.publish(msg)
            return
        self.stop_robot()
        self.get_logger().info(done)
        # The lidar never saw behind the start pose, so that patch reads as a big frontier
        # (seen in the sim even after a 360 deg turn). It is the start cell's back, not a way
        # on: an earlier run drove back into the dead-end start cell and got stuck turning.
        self.blacklist.append(self.creep_from)
        self.scan_or_begin()

    def send_spin(self, angle):
        self.spin_busy = True
        # behavior_server min_rotational_vel is 0.2 rad/s; allow that plus margin
        allowance = abs(angle) / 0.2 + 5.0
        goal = Spin.Goal(target_yaw=float(angle),
                         time_allowance=Duration(sec=int(math.ceil(allowance))))
        self.get_logger().info(f'start scan: turn {math.degrees(angle):+.0f} deg')
        self.spin.send_goal_async(goal).add_done_callback(self.on_spin_accepted)

    def on_spin_accepted(self, future):
        handle = future.result()
        if self.state != 'SCANNING':                # stopped while being accepted
            if handle.accepted:
                handle.cancel_goal_async()
            self.spin_busy = False
            return
        if not handle.accepted:
            self.get_logger().warn('start scan: turn rejected; continuing')
            self.spin_busy = False
            return
        self.spin_handle = handle
        handle.get_result_async().add_done_callback(self.on_spin_result)

    def on_spin_result(self, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'start scan: turn ended with status {status}; continuing')
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
        if self.state == 'CREEPING':
            self.stop_robot()
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
