"""Explore the maze on the SLAM map with Nav2 until the robot is out.

Start: once /map, TF and Nav2 are up, the robot holds still (status PAUSED) until the
operator publishes on /maze/start (std_msgs/Empty):
    ros2 topic pub --once /maze/start std_msgs/msg/Empty
so there is time to start a rosbag recording. Messages sent before PAUSED are ignored.
wait_for_start:=false skips the wait (simulator). Then a scan (`start_scan`) turns the robot
`scan_angle` left, back through to `scan_angle` right, then back to the start heading using
Nav2's /spin behavior, so SLAM sees the start area before the first frontier is picked. A
failed spin is logged and the scan moves on; it never blocks exploring. Skipped when go:=false.

Loop (period `replan_period`):
  1. read /map (slam_toolbox) and the robot pose (TF map -> base_link)
  2. exit check: was in corridors, has moved >= min_displacement, and a disk of seen-free
     floor >= exit_open_radius lies within exit_near_radius of it (and away from the
     start) for `dwell` s -> DONE
  3. frontier = free cell next to unknown; pick the nearest along free space, keep the
     current one unless another is clearly closer (no flip-flopping)
  4. send it to Nav2 /navigate_to_pose; a finished goal (reached or failed) is blacklisted
Nothing left to explore and no exit -> DONE (no exit found): the running goal is cancelled
at once, a zero Twist goes to nav_cmd_vel and "nav stopped" is printed. Either way Nav2 is
cancelled, the map is saved and /maze/status says why.

Status on /maze/status (std_msgs/String, latched): STARTING, PAUSED, SCANNING, EXPLORING, EXITED, NO_EXIT,
STOPPED. Goals on /maze/goal (PoseStamped) for RViz. `go` parameter false = plan and publish
goals but never send them to Nav2 (dry run on the robot).
"""
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.core import ExitDetector, Grid, choose_frontier, find_frontiers

START_CMD = 'ros2 topic pub --once /maze/start std_msgs/msg/Empty'

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


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
        p('goal_timeout', 90.0)        # s per goal before giving up on it
        p('max_failures', 3)           # failed goals in a row before NO_EXIT
        p('done_confirm', 5)           # replans in a row with no frontier before NO_EXIT
        p('exit_open_radius', 1.1)     # m of seen-free floor around a point = outside
        p('exit_near_radius', 0.8)     # that point within this distance of the robot
        p('exit_min_displacement', 1.5)
        p('exit_dwell', 2.0)
        p('max_runtime', 900.0)        # s; hard stop
        p('start_scan', True)          # turn left/right on the spot before exploring
        p('scan_angle', math.pi / 2)   # rad each way
        p('map_dir', os.path.expanduser('~/challenge_ws/maps'))
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in ('go', 'start_delay', 'wait_for_start', 'robot_radius', 'goal_clearance',
                                      'min_frontier_size', 'goal_timeout', 'max_failures', 'done_confirm',
                                      'max_runtime', 'map_dir', 'start_scan', 'scan_angle')}
        self.exit = ExitDetector(open_radius=g('exit_open_radius'),
                                 near_radius=g('exit_near_radius'),
                                 min_displacement=g('exit_min_displacement'),
                                 dwell=g('exit_dwell'))

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        self.status_pub = self.create_publisher(String, '/maze/status', LATCHED)
        self.goal_pub = self.create_publisher(PoseStamped, '/maze/goal', LATCHED)
        # Same input as Nav2's velocity_smoother output, so the stop goes through
        # cmd_vel_bridge (clamps, drive:=false preview) like any other command.
        self.cmd_pub = self.create_publisher(Twist, 'nav_cmd_vel', 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        # Volatile on purpose: a start sent before this run is never replayed.
        self.create_subscription(Empty, '/maze/start', self.on_start, 10)
        self.create_timer(g('replan_period'), self.step)

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
        self.scan_turns = [a, -2 * a, a]   # left, right, back to the start heading
        self.spin_busy = False
        self.spin_handle = None
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
            if self.cfg['go'] and self.cfg['start_scan']:
                self.set_state('SCANNING', f'turning {math.degrees(abs(self.cfg["scan_angle"])):.0f}'
                               ' deg left and right to see the start area')
            else:
                self.set_state('EXPLORING', 'go' if self.cfg['go'] else 'DRY RUN (go:=false)')
        if self.state == 'SCANNING':
            if time.monotonic() - self.started > self.cfg['max_runtime']:
                return self.finish('STOPPED', f'max_runtime {self.cfg["max_runtime"]:.0f} s')
            if self.spin_busy:
                return
            if not self.scan_turns:
                self.set_state('EXPLORING', 'start scan done')
            elif not self.spin.server_is_ready():
                if time.monotonic() - self.started < 10.0:   # action discovery takes a moment
                    return
                self.get_logger().warn('Nav2 /spin not available; skipping start scan')
                self.scan_turns = []
                self.set_state('EXPLORING', 'start scan skipped')
            else:
                self.send_spin(self.scan_turns.pop(0))
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

        frontiers = find_frontiers(grid, xy, self.cfg['robot_radius'], self.cfg['goal_clearance'],
                                   self.cfg['min_frontier_size'], self.blacklist)
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

    def send(self, f, n):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = f.goal
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(f.yaw / 2), math.cos(f.yaw / 2)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'frontier goal ({f.goal[0]:.2f}, {f.goal[1]:.2f}) '
                               f'{f.path_length:.1f} m away, {f.size} cells; {n} frontiers')
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

    def send_spin(self, angle):
        self.spin_busy = True
        # behavior_server min_rotational_vel is 0.2 rad/s; allow that plus margin
        allowance = abs(angle) / 0.2 + 5.0
        goal = Spin.Goal(target_yaw=float(angle),
                         time_allowance=Duration(sec=int(math.ceil(allowance))))
        self.get_logger().info(f'start scan: spin {math.degrees(angle):+.0f} deg')
        self.spin.send_goal_async(goal).add_done_callback(self.on_spin_accepted)

    def on_spin_accepted(self, future):
        handle = future.result()
        if self.state != 'SCANNING':               # stopped while being accepted
            if handle.accepted:
                handle.cancel_goal_async()
            self.spin_busy = False
            return
        if not handle.accepted:
            self.get_logger().warn('start scan: spin rejected')
            self.spin_busy = False
            return
        self.spin_handle = handle
        handle.get_result_async().add_done_callback(self.on_spin_result)

    def on_spin_result(self, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'start scan: spin ended with status {status}; continuing')
        self.spin_handle = None
        self.spin_busy = False

    def on_accepted(self, future, f):
        handle = future.result()
        if f is not self.current:                   # superseded while being accepted
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.goal_result = GoalStatus.STATUS_ABORTED
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda fut, f=f: self.on_result(fut, f))

    def on_result(self, future, f):
        if f is self.current:
            self.goal_result = future.result().status

    def finish(self, state, why):
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
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.state not in ('EXITED', 'NO_EXIT', 'STOPPED') and rclpy.ok():
            node.finish('STOPPED', 'shutdown')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
