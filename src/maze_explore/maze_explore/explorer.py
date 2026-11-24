"""Maze explorer: decides where to go (plan.py), drives there cell by cell, and switches the
robot's modes. See README.md for the whole picture.

  in   /map                SLAM map (slam_toolbox)
       /maze/cloud         levelled cloud with heights above the floor (wall_scan)
       TF map -> base_level   where the robot is
       /odom               only to catch a diverged odometry
       /nav/start, /explore/stop   std_msgs/Empty
  out  /cmd_vel            velocity (never backwards), /simple_cmd  mode and gait requests
       /explore/status     String, one line: state, cell, job, action
       /explore/event      String: "curtain", "room|object centre|left|right", "stairs", "exit" ...
                           with the cell and direction (for perception / logging)
       /explore/cells      MarkerArray: visited cells and what is on each side
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from transfer_interfaces.msg import MotionSimpleCMD
from visualization_msgs.msg import Marker, MarkerArray

from maze_explore import robot
from maze_explore.grid import Grid, fit_grid, nb, opp, wrap
from maze_explore.plan import next_job
from maze_explore.sense import HeightMap, Occupancy, Sense

COLOURS = {'wall': (0.3, 0.3, 0.3), 'free': (0.1, 0.8, 0.1), 'room': (0.1, 0.4, 1.0),
           'curtain': (1.0, 0.5, 0.8), 'stairs': (1.0, 0.6, 0.0), 'blocked': (0.8, 0.1, 0.1),
           'unknown': (1.0, 1.0, 0.2), 'exit': (1.0, 0.0, 0.0), 'object': (0.6, 0.0, 0.8)}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Explorer(Node):

    def __init__(self):
        super().__init__('maze_explorer')
        p = self.declare_parameter
        self.cell_size = p('cell_size', 1.02).value
        self.v_max = p('v_max', 0.20).value            # m/s (team limit 0.25)
        self.w_max = p('w_max', 0.5).value             # rad/s
        self.min_spin = p('min_spin', 0.2).value       # the Lite3 does not turn below ~0.1
        self.creep = p('creep', 0.10).value            # m forward at the start
        self.gait = p('gait', 'hstep').value
        self.drive_on = p('drive', True).value         # false: /explore/cmd_vel_preview only
        self.send_modes = p('send_modes', True).value  # false: no /simple_cmd at all
        self.enter_rooms = p('enter_rooms', False).value
        self.look_angle = math.radians(p('look_angle', 25.0).value)
        self.look_hold = p('look_hold', 1.5).value
        self.greet_hold = p('greet_hold', 2.0).value
        self.cmd_gap = p('cmd_gap', 1.5).value         # s between mode requests
        self.ai_speed = p('ai_speed', 0.15).value
        self.curtain_speed = p('curtain_speed', 0.20).value
        self.pose_timeout = p('pose_timeout', 1.0).value
        self.push_stop = p('push_stop', True).value    # obstacle stop also while pushing (a
        #                                                real curtain then stops the push too)
        auto_start = p('auto_start', False).value

        self.state = 'WAIT'           # WAIT sensors, READY, RUN, DONE, STOPPED, FAULT
        self.why = ''
        self.grid = None
        self.aligned = False          # the grid has been lined up with the walls once
        self.heights = HeightMap()
        self.occ = None
        self.pose = None              # x, y, yaw in map
        self.pose_time = None
        self.cloud = None             # latest /maze/cloud as (n, 3), base_level
        self.cloud_time = None
        self.last_odom = None
        self.blind_until = 0.0        # curtain over the lidar: its returns are not walls
        self.queue = deque()
        self.act = None
        self.act_t0 = 0.0
        self.act_mem = {}
        self.moving = False
        self.job = ''
        self.kinds = {}
        self.start = (0, 0)
        self.visited, self.closed = set(), set()
        self.walked, self.forced_wall = set(), set()
        self.links, self.looked, self.greeted = {}, set(), set()
        self.exit_cell = self.exit_d = None
        self.step = None              # (from cell, d, dest) of the drive under way
        self.auto_start = auto_start

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        vel_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             lifespan=Duration(seconds=0.5))
        self.vel_pub = self.create_publisher(
            Twist, '/cmd_vel' if self.drive_on else '/explore/cmd_vel_preview', vel_qos)
        self.cmd_pub = self.create_publisher(MotionSimpleCMD, '/simple_cmd', latched)
        self.status_pub = self.create_publisher(String, '/explore/status', 10)
        self.event_pub = self.create_publisher(String, '/explore/event', 10)
        self.blind_pub = self.create_publisher(Bool, '/maze/blind', 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/explore/cells', 2)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PointCloud2, '/maze/cloud', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Empty, '/nav/start', lambda m: self.begin(), 1)
        self.create_subscription(Empty, '/explore/stop', lambda m: self.halt('STOPPED', 'stop asked'), 1)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_cells)

    # ---- inputs -------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, msg):
        i = msg.info
        self.occ = Occupancy(np.asarray(msg.data, np.int8).reshape(i.height, i.width),
                             i.resolution, i.origin.position.x, i.origin.position.y)

    def on_cloud(self, msg):
        pts = np.frombuffer(msg.data, np.float32).reshape(-1, 3)
        self.cloud, self.cloud_time = pts, self.now()
        if self.pose is None or not len(pts) or self.now() < self.blind_until:
            return
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        self.heights.add(x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1],
                         pts[:, 2])

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        if self.last_odom is not None:
            t0, x0, y0 = self.last_odom
            jump = math.hypot(p.x - x0, p.y - y0)
            if jump > 0.4 or (t - t0 > 0.02 and jump / (t - t0) > 1.5):
                self.halt('FAULT', f'/odom jumped {jump:.2f} m in {t - t0:.2f} s: odometry has '
                                   'diverged. Stop, check the lidar / IMU rates, relaunch')
        self.last_odom = (t, p.x, p.y)

    def read_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_level', Time())   # never blocks
        except TransformException:
            return False
        q, t = tf.transform.rotation, tf.transform.translation
        self.pose = (t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        self.pose_time = stamp
        return True

    # ---- outputs ------------------------------------------------------------------------
    def send(self, vx=0.0, vy=0.0, wz=0.0):
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            if not self.moving:
                return                      # one zero, then silence (teleop works again)
            self.moving = False
        else:
            self.moving = True
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(max(0.0, vx)), float(vy), float(wz)
        self.vel_pub.publish(m)

    def simple_cmd(self, code):
        self.get_logger().info(f'/simple_cmd 0x{code:08X}' + ('' if self.sending() else ' (not sent)'))
        if self.sending():
            self.cmd_pub.publish(MotionSimpleCMD(cmd_code=code, size=0, type=0))

    def sending(self):
        return self.drive_on and self.send_modes

    def event(self, text):
        self.get_logger().info(f'event: {text}')
        self.event_pub.publish(String(data=text))

    def halt(self, state, why):
        if self.state in ('FAULT',) or (self.state == state and self.why == why):
            return
        self.moving = True
        self.send()
        self.queue.clear()
        self.act = None
        self.state, self.why = state, why
        (self.get_logger().error if state == 'FAULT' else self.get_logger().info)(f'{state}: {why}')

    # ---- start --------------------------------------------------------------------------
    def begin(self):
        if self.state not in ('READY', 'STOPPED'):
            self.get_logger().warn(f'/nav/start ignored in state {self.state} ({self.why})')
            return
        if self.grid is None:
            x, y, yaw = self.pose
            self.grid = Grid(x, y, yaw, self.cell_size)
            self.visited = {self.start}
            for code in robot.start_sequence(self.gait):
                self.queue.append(('cmd', code))
                self.queue.append(('hold', self.cmd_gap))
            fx, fy = x + self.creep * math.cos(yaw), y + self.creep * math.sin(yaw)
            self.queue.append(('drive', fx, fy, yaw, 'creep'))
            self.queue.append(('hold', 1.5))
            self.queue.append(('call', self.align_grid))
            self.queue.append(('resolve', self.start))
        self.state, self.why = 'RUN', ''
        self.get_logger().info('exploring')

    def align_grid(self):
        """Line the grid up with the walls in the map. Returns True when this call was the
        first fit that worked.

        Until one fit has worked the limits are wide (30 deg, 0.40 m): that is what lets the
        robot start off-centre and at an angle, the start cell's centre and the maze
        directions come from the walls, not from where the robot was put down. The map right
        after the start creep is one or two scans and often too thin for it (run1: 37 wall
        points, the grid then stayed 6 deg and 0.27 m off), so the wide fit is tried again at
        every planning step until it works. After that only small corrections (3 deg, 0.08 m)."""
        if self.occ is None:
            return False
        wide = not self.aligned
        max_yaw, max_shift = (math.radians(30), 0.40) if wide else (math.radians(3), 0.08)
        px, py = self.occ.occupied_points()
        near = np.hypot(px - self.pose[0], py - self.pose[1]) < 4.0
        g, q = fit_grid(self.grid, px[near], py[near], max_yaw, max_shift)
        if g is None or q < 0.5:
            how = 'too few wall points or further off than the limits' if g is None \
                else f'quality {q:.2f}'
            self.get_logger().warn(
                f"grid {'NOT LINED UP YET' if wide else 'not corrected'}: no fit within "
                f'{math.degrees(max_yaw):.0f} deg / {max_shift:.2f} m ({how}, '
                f'{int(near.sum())} wall points); kept', throttle_duration_sec=5.0)
            return False
        turned, moved = math.degrees(wrap(g.yaw - self.grid.yaw)), \
            math.hypot(g.x0 - self.grid.x0, g.y0 - self.grid.y0)
        self.grid = g
        if wide:
            self.aligned = True
            # sides taken as walls and rooms looked at with the unfitted grid were judged in
            # the wrong places
            self.forced_wall.clear()
            self.looked.clear()
            self.get_logger().info(f'grid lined up with the walls: turned {turned:+.1f} deg, '
                                   f'moved {moved:.2f} m (quality {q:.2f})')
            return True
        self.get_logger().info(f'grid corrected: {turned:+.1f} deg, {moved:.2f} m '
                               f'(quality {q:.2f})', throttle_duration_sec=10.0)
        return False

    # ---- what is around -----------------------------------------------------------------
    def kind(self, sense, cell, d):
        b = nb(cell, d)
        if (cell, d) in self.walked:
            return 'free'
        if (cell, d) in self.forced_wall or (cell == self.start and d == 2) or b in self.closed:
            return 'wall'
        k = sense.kind(cell, d, may_exit=cell != self.start)
        if k == 'exit' and self.exit_cell is None:
            self.exit_cell, self.exit_d = cell, d
            self.event(f'exit seen from cell {cell} side {d}')
        return k

    def update_kinds(self):
        sense = Sense(self.grid, self.occ, self.heights)
        self.kinds = {(c, d): self.kind(sense, c, d)
                      for c in self.visited if c not in self.closed for d in range(4)}

    def cur_cell(self):
        return self.grid.cell_of(self.pose[0], self.pose[1])

    def arrive(self, cell):
        self.visited.add(cell)
        if cell != self.start and self.start not in self.closed:
            self.closed.add(self.start)           # never back into the start cell
        self.step = None
        self.queue.appendleft(('resolve', cell))

    # ---- planning -----------------------------------------------------------------------
    def plan(self):
        cur = self.cur_cell()
        if cur not in self.visited or cur in self.closed:
            cur = min(self.visited - self.closed, key=lambda c: math.dist(self.grid.centre(c), self.pose[:2]))
        if self.align_grid():                 # first fit: the cells have moved, look again
            cur = self.cur_cell()
            if cur not in self.visited or cur in self.closed:
                cur = min(self.visited - self.closed,
                          key=lambda c: math.dist(self.grid.centre(c), self.pose[:2]))
            self.queue.append(('resolve', cur))
            return
        self.update_kinds()
        facing = self.grid.nearest_dir(self.pose[2])
        j = next_job(self.kinds, self.visited, self.closed, self.links, cur, facing,
                     self.looked, self.greeted, self.exit_cell, self.enter_rooms)
        job = f"{j['job']} cell {j['cell']} side {j['d']} via {len(j['path'])} cells"
        if job != self.job:
            here = ' '.join(self.kinds.get((cur, k), '-') for k in range(4))
            self.get_logger().info(f'job: {job}   [in {cur}: ahead/left/back/right = {here}]')
            self.job = job
        q = self.queue
        if j['path']:
            d, how, dest = j['path'][0]
            self.go(cur, d, how)
            return
        c, d, h = j['cell'], j['d'], None if self.grid is None else self.grid.heading(j['d'])
        if j['job'] == 'greet':
            q.extend([('turn', h, 2.0), ('event', f'curtain cell {c} side {d}'),
                      ('hold', self.greet_hold), ('call', lambda: self.greeted.add((c, d)))])
        elif j['job'] == 'visit':
            self.go(c, d, 'walk')
        elif j['job'] == 'look':
            for name, off in (('centre', 0.0), ('left', self.look_angle), ('right', -self.look_angle)):
                what = 'object' if self.kinds.get((c, d)) == 'object' else 'room'
                q.extend([('turn', wrap(h + off), 3.0), ('event', f'{what} {name} cell {c} side {d}'),
                          ('hold', self.look_hold)])
            q.append(('call', lambda: self.looked.add((c, d))))
        elif j['job'] in ('curtain', 'stairs'):
            self.go(c, d, j['job'])
        elif j['job'] == 'exit':
            q.extend([('turn', self.grid.heading(self.exit_d), 3.0),
                      ('event', f'exit: stopped in the last cell {c}'),
                      ('call', lambda: self.halt('DONE', 'in the last cell'))])
        else:
            self.halt('DONE', 'nothing left to explore and no exit seen')

    def go(self, a, d, how):
        """Queue one step from the centre of cell a through side d."""
        q, h, b = self.queue, self.grid.heading(d), nb(a, d)
        ax, ay = self.grid.centre(a)
        if how == 'walk':
            bx, by = self.grid.centre(b)
            q.extend([('turn', h, 4.0), ('call', lambda: setattr(self, 'step', (a, d, b))),
                      ('drive', bx, by, h, 'normal'),
                      ('call', lambda: (self.walked.update({(a, d), (b, opp(d))}), self.arrive(b)))])
        elif how == 'curtain':
            bx, by = self.grid.centre(b)

            def crossed():
                self.links[(a, d)], self.links[(b, opp(d))] = ('curtain', b), ('curtain', a)
                self.arrive(b)
            q.extend([('drive', ax, ay, h, 'square'), ('turn', h, 2.0),
                      ('event', f'curtain: going through, cell {a} side {d}'),
                      ('call', lambda: setattr(self, 'step', (a, d, b))),
                      ('drive', bx, by, h, 'push'), ('call', crossed)])
        elif how == 'stairs':
            c = nb(b, d)
            cx, cy = self.grid.centre(c)

            def crossed():
                self.links[(a, d)], self.links[(c, opp(d))] = ('stairs', c), ('stairs', a)
                self.closed.add(b)
                self.arrive(c)
            q.extend([('drive', ax, ay, h, 'square'), ('turn', h, 2.0),
                      ('event', f'stairs: AI mode, cell {a} side {d}')])
            for code in robot.ai_enter_sequence():
                q.extend([('cmd', code), ('hold', self.cmd_gap)])
            q.extend([('hold', 1.0), ('call', lambda: setattr(self, 'step', (a, d, c))),
                      ('drive', cx, cy, h, 'ai'), ('hold', 1.0)])
            for code in robot.ai_leave_sequence(self.gait):
                q.extend([('cmd', code), ('hold', self.cmd_gap)])
            q.extend([('event', f'stairs: crossed, {self.gait} again'), ('call', crossed)])

    # ---- actions ------------------------------------------------------------------------
    def tick(self):
        have = self.read_pose()
        if self.state == 'WAIT':
            miss = [n for n, ok in (('/map', self.occ is not None), ('map -> base_level', have),
                                    ('/maze/cloud', self.cloud is not None)) if not ok]
            self.why = 'waiting for ' + ', '.join(miss)
            if not miss:
                self.state, self.why = 'READY', 'send /nav/start'
                self.get_logger().info('ready: ros2 topic pub --once /nav/start std_msgs/msg/Empty')
                if self.auto_start:
                    self.begin()
        elif self.state == 'RUN':
            age = max(self.now() - self.pose_time, self.now() - (self.cloud_time or 0.0))
            if age > self.pose_timeout:
                self.send()
                self.why = f'pose / lidar {age:.1f} s old: standing still until it is fresh'
                self.act_t0 += 0.1                     # the action's clock stands still too
            else:
                self.why = ''
                for _ in range(20):                    # instant actions do not cost a tick
                    if self.act is None:
                        if not self.queue:
                            self.plan()
                        if not self.queue or self.state != 'RUN':
                            break
                        self.act, self.act_t0, self.act_mem = self.queue.popleft(), self.now(), {}
                    done = getattr(self, 'do_' + self.act[0])(*self.act[1:])
                    if not done:
                        break
                    self.act = None
        cell = self.cur_cell() if self.grid and self.pose else None
        self.status_pub.publish(String(
            data=f"{self.state} cell={cell} job=[{self.job}] act={self.act[0] if self.act else '-'} {self.why}"))

    def elapsed(self):
        return self.now() - self.act_t0

    def do_hold(self, seconds):
        self.send()
        return self.elapsed() >= seconds

    def do_cmd(self, code):
        self.send()
        self.simple_cmd(code)
        return True

    def do_event(self, text):
        self.event(text)
        return True

    def do_call(self, fn):
        fn()
        return True

    def do_turn(self, target, tol_deg):
        err = wrap(target - self.pose[2])
        if abs(err) < math.radians(tol_deg):
            self.act_mem['ok'] = self.act_mem.get('ok', 0) + 1
            self.send()
            return self.act_mem['ok'] >= 3
        self.act_mem['ok'] = 0
        if self.elapsed() > 25.0:
            self.get_logger().warn(f'turn timed out {math.degrees(err):.0f} deg short')
            self.send()
            return True
        wz = clamp(1.5 * err, -self.w_max, self.w_max)
        self.send(wz=math.copysign(max(abs(wz), self.min_spin), wz))
        return False

    def front_blocked(self):
        """Something from knee height up, 6-20 cm ahead of the body, in the body's width."""
        p = self.cloud
        m = (p[:, 0] > 0.33) & (p[:, 0] < 0.47) & (np.abs(p[:, 1]) < 0.20) & (p[:, 2] > 0.30) & (p[:, 2] < 1.0)
        return int(m.sum()) >= 6

    def do_drive(self, tx, ty, heading, mode):
        x, y, yaw = self.pose
        dx, dy = tx - x, ty - y
        c, s = math.cos(heading), math.sin(heading)
        along, lat, herr = dx * c + dy * s, -dx * s + dy * c, wrap(heading - yaw)
        mem = self.act_mem
        if mode == 'push':                   # curtain over the lidar: wall_scan sends no scans
            self.blind_pub.publish(Bool(data=True))
            self.blind_until = self.now() + 1.5
        if mode == 'square':                 # onto the cell's centre line, facing `heading`
            if abs(lat) < 0.04 and abs(herr) < math.radians(3) or self.elapsed() > 15.0:
                self.send()
                return True
            self.send(vy=clamp(0.8 * lat, -0.10, 0.10) if abs(herr) < math.radians(15) else 0.0,
                      wz=clamp(1.5 * herr, -self.w_max, self.w_max))
            return False
        if along <= 0.03:
            self.send()
            return True
        mem.setdefault('best', along)
        if along < mem['best'] - 0.05:
            mem['best'], mem['t'] = along, self.now()
        if self.now() - mem.setdefault('t', self.now()) > (12.0 if mode == 'ai' else 8.0):
            self.send()
            self.get_logger().warn(f'{mode} drive: no progress, {along:.2f} m short')
            return self.stuck(mode, along)
        if abs(herr) > math.radians(25):
            self.send(wz=math.copysign(max(self.min_spin, min(self.w_max, 1.5 * abs(herr))), herr))
            return False
        checked = mode in ('normal', 'creep') or (mode == 'push' and self.push_stop)
        if checked and along > 0.10 and self.front_blocked():
            self.send()
            mem['blocked'] = mem.get('blocked', 0) + 1
            if mem['blocked'] >= 8:
                self.get_logger().warn(f'way blocked {along:.2f} m before the cell centre')
                return self.stuck(mode, along)
            return False
        mem['blocked'] = 0
        top = {'ai': self.ai_speed, 'push': self.curtain_speed, 'creep': 0.10}.get(mode, self.v_max)
        self.send(vx=clamp(1.0 * along, 0.08, top), vy=clamp(0.8 * lat, -0.10, 0.10),
                  wz=clamp(1.5 * herr, -0.4, 0.4))
        return False

    def stuck(self, mode, along):
        """A drive that cannot go on. Returns True (the action is over)."""
        if mode == 'ai':
            self.halt('STOPPED', 'no progress on the stairs in AI mode: take over by hand '
                                 '(lite3_ai_off), then /nav/start')
            return True
        if mode == 'creep' or self.step is None:
            return True
        a, d, b = self.step
        self.queue.clear()
        cur = self.cur_cell()
        self.step = None
        if cur == b and along < 0.35:           # far enough in: count the cell as reached
            self.walked.update({(a, d), (b, opp(d))})
            self.arrive(b)
            return True
        self.forced_wall.update({(a, d), (b, opp(d))})
        self.links.pop((a, d), None)
        ax, ay = self.grid.centre(a)
        back = math.atan2(ay - self.pose[1], ax - self.pose[0])
        if math.hypot(ax - self.pose[0], ay - self.pose[1]) > 0.10:
            self.queue.extend([('turn', back, 5.0), ('drive', ax, ay, back, 'creep')])
        return True

    def do_resolve(self, cell):
        """Standing in a new cell: wait until all four sides are known; face the ones that
        stay unknown (the lidar sees low walls best ahead); still unknown = wall."""
        self.send()
        if self.elapsed() < 1.2:               # one map update
            return False
        self.update_kinds()
        unknown = [d for d in range(4) if self.kinds.get((cell, d)) == 'unknown']
        if not unknown or cell in self.closed:
            return True
        if self.elapsed() < 2.5:
            return False
        for d in unknown:
            def settle(d=d):
                self.update_kinds()
                if self.kinds.get((cell, d)) == 'unknown':
                    self.get_logger().warn(f'cell {cell} side {d} still unknown: taken as a wall')
                    self.forced_wall.add((cell, d))
            self.queue.extendleft(reversed([('turn', self.grid.heading(d), 4.0), ('hold', 2.0),
                                            ('call', settle)]))
        return True

    # ---- display ------------------------------------------------------------------------
    def publish_cells(self):
        if self.grid is None:
            return
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        stamp = self.get_clock().now().to_msg()

        def marker(i, kind, rgb, scale):
            m = Marker(ns='explore', id=i, type=kind, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = *rgb, 1.0
            m.pose.orientation.w = 1.0
            return m
        cells = marker(1, Marker.CUBE_LIST, (0.2, 0.7, 0.9), 0.15)
        cells.points = [Point(x=self.grid.centre(c)[0], y=self.grid.centre(c)[1]) for c in self.visited]
        out.markers.append(cells)
        for i, (name, rgb) in enumerate(COLOURS.items()):
            m = marker(10 + i, Marker.LINE_LIST, rgb, 0.05)
            for (c, d), k in self.kinds.items():
                if k == name:
                    x, y = self.grid.box(c, d, [-0.4, 0.4], [0.0])
                    m.points += [Point(x=float(x[0, 0]), y=float(y[0, 0])), Point(x=float(x[1, 0]), y=float(y[1, 0]))]
            if m.points:
                out.markers.append(m)
        self.marker_pub.publish(out)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.moving = True
        try:
            node.send()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
