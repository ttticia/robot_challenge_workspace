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
import json
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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Empty, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from transfer_interfaces.msg import MotionSimpleCMD
from visualization_msgs.msg import Marker, MarkerArray

from maze_explore import robot
from maze_explore.curtain_node import centred, edge_along
from maze_explore.grid import Grid, fit_grid, nb, opp, wrap
from maze_explore.plan import next_job
from maze_explore.sense import HeightMap, Occupancy, Sense
from stairs import stair_levels

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
        self.gait = p('gait', 'hstep').value           # hstep = H-Step (default), rug = RUG general gear
        self.drive_on = p('drive', True).value         # false: /explore/cmd_vel_preview only
        self.send_modes = p('send_modes', True).value  # false: no /simple_cmd at all
        self.enter_rooms = p('enter_rooms', False).value
        # Plain navigation switches (launch: stairs / curtains / terrain, or plain:=true for
        # all three). Off: stairs are just a blocked cell (no depth check, no AI mode), a
        # curtain is a wall (not faced, not pushed), uneven cells are walked into head first.
        self.stairs_on = p('stairs', True).value
        self.curtains_on = p('curtains', True).value
        self.terrain_on = p('terrain', True).value
        # Curtain seen by the camera (curtain_node, YOLO; launch curtain_cv:=true, OFF by
        # default). A sighting only names a direction. The first closed edge that way becomes
        # a candidate after cv_hits frames; the robot then faces it from a cell centre and it
        # counts when the box lies across the middle of the image on cv_verify_hits frames.
        # A verified edge is a curtain (passable) even where the map and the height map call
        # it a wall.
        self.cv_on = p('curtain_cv', True).value and self.curtains_on   # on by default (user, 2026-11-25)
        self.cv_conf = p('cv_conf', 0.5).value
        self.cv_hits = p('cv_hits', 3).value                 # frames, within cv_window s
        self.cv_window = p('cv_window', 3.0).value
        self.cv_verify_time = p('cv_verify_time', 3.0).value   # s facing the edge
        self.cv_verify_hits = p('cv_verify_hits', 3).value
        self.cv_seen = {}             # (cell, d) -> times of the frames that pointed at it
        self.cv_pending, self.cv_curtains, self.cv_rejected = set(), set(), set()
        # Curtain edges are latched (user, 2026-11-25): once the lidar has read an edge as a
        # curtain it stays one, both sides of it, whatever the map shows later. Fabric that
        # sways, is parted or pushed aside changes the map; it must not turn the edge into an
        # open way (walked into head first) or a plain wall (never crossed). Only a failed
        # crossing (stuck) or the first grid fit (cells move) undoes it.
        self.curtain_latched = set()
        self.cv_frames = deque(maxlen=60)     # (time, box across the image centre?) per frame
        self.cv_checking = set()              # edges whose verification is queued or running
        # Before a room / object look is shown to scene understanding (the VLM prompt goes
        # out after its three views), the way ahead is checked for stairs (depth detector)
        # and for a curtain (camera, with curtain_cv). If one is verified the look is
        # called off: no views, no prompt (user, 2026-11-25).
        self.look_check_on = p('look_check', True).value
        self.look_checked = set()             # (cell, d) whose check has been made
        # Uneven terrain is entered SIDEWAYS (user, 2026-11-24): turn on the spot at the cell
        # centre until the body is square to the way in, then step sideways into the cell.
        self.side_speed = p('side_speed', 0.10).value      # m/s sideways
        self.side_offset = p('side_offset', 0.075).value   # m the base_link line is moved
        #    towards the nose, so the body (0.27 m ahead, 0.42 m behind) is centred in the way in
        self.terrain_min = p('terrain_min', 0.06).value    # m above the floor = uneven
        self.terrain_squares = p('terrain_squares', 15).value   # 5 cm squares of the cell
        # The lidar's lowest ray meets the floor ~1.3 m ahead, so from the cell before it the
        # next cell's floor is mostly too close to see; it is known only if it was seen from
        # further back. With fewer than terrain_seen_min of its 121 squares seen, the standing
        # depth-camera check runs before walking in, and reads the floor there too (user,
        # 2026-11-25). The camera's verdict, when it has one, beats the lidar's.
        self.terrain_depth = p('terrain_depth', True).value
        self.terrain_seen_min = p('terrain_seen_min', 40).value
        self.rough_checked = {}       # (cell, d) -> True | False | None (camera saw too little)
        self.look_angle = math.radians(p('look_angle', 25.0).value)
        self.look_hold = p('look_hold', 1.5).value
        self.greet_hold = p('greet_hold', 2.0).value
        self.cmd_gap = p('cmd_gap', 1.5).value         # s between mode requests
        self.ai_speed = p('ai_speed', 0.15).value
        self.curtain_speed = p('curtain_speed', 0.20).value
        self.pose_timeout = p('pose_timeout', 1.0).value
        self.push_stop = p('push_stop', True).value    # obstacle stop also while pushing (a
        #                                                real curtain then stops the push too)
        # Body outline from base_link, m. The rear is the 0.37 m body plus 0.05 m for the bent
        # hind legs (user, 2026-11-24). The rear corner sweeps turn_radius when turning on the
        # spot: 0.48 m, against 0.485 m from a cell's centre to its walls, so turns are made
        # ON the cell centre and held there (do_turn).
        self.fp_front = p('footprint_front', 0.27).value
        self.fp_rear = p('footprint_rear', 0.42).value
        self.fp_half_width = p('footprint_half_width', 0.23).value
        self.turn_radius = math.hypot(self.fp_rear, self.fp_half_width)
        self.turn_ff = p('turn_forward', 0.03).value     # m/s forward at full turn rate: the
        #                                                  Lite3 drifts ~9 cm back per 90 deg
        self.recentre = p('recentre', 0.20).value         # m off the cell centre from which the
        #                    robot walks back onto it before the next job (10:03 run, 11-24: a
        #                    look was made 0.5 m ahead of the centre, between two cells)
        self.recentre_tries = 0
        self.pivot_snap = p('pivot_snap', 0.25).value     # m: turn about the cell centre when
        #                                                  the robot stands this near it
        # Stair check with the depth camera (stair_levels.py, from ~/stairs_detect). The depth
        # stream is only subscribed while the robot stands still for the check: one raw depth
        # subscriber slows every stream from the robot, lidar and its IMU included.
        self.stair_check_on = p('stair_check', True).value and self.stairs_on
        self.stair_check_time = p('stair_check_time', 3.0).value     # s of looking
        self.stair_rails = p('stair_rails', True).value   # handrails must show in the lidar too
        self.stair_raise_min = p('stair_raise_min', 0.28).value   # m: lidar height in the next
        #                                    cell from which it is looked at before walking in
        self.stair_params = stair_levels.LevelParams(
            rise=p('stair_rise', 0.12).value, tread=p('stair_tread', 0.20).value,
            steps=p('stair_steps', 3).value,
            # Wider than the detector's defaults, for close range (2026-11-24, on the real
            # bags: 0.26 m shot 12 -> 18 of 21 frames, walk-in 61 -> 80 of 191, the look-alike
            # obstacle still 0 of 27). Close up the floor is out of view, so the top tread's
            # height is less exact, and less of each tread is seen.
            top_min=p('stair_top_min', 0.31).value, top_max=p('stair_top_max', 0.42).value,
            edge_max_foreign=p('stair_edge_foreign', 4).value)
        self.stair_quick_no = p('stair_quick_no', 8).value   # frames without one hit = not stairs
        self.depth_topic = p('depth_topic', '/camera/depth/image_raw').value
        self.depth_info_topic = p('depth_info_topic', '/camera/depth/camera_info').value
        self.stairs_checked = {}      # (cell, d) -> True (stairs) | False (not) | 'lidar' (no depth)
        self.odom_rot = None          # base_link orientation in odom (roll / pitch for the check)
        self.body_height = None
        self.clouds = deque(maxlen=12)        # the last ~1.2 s of /maze/cloud, for the handrails
        self.odom_grace_until = 0.0
        self.odom_settle = p('odom_settle', 3.0).value   # s of pause after one odometry jump
        self.odom_jumps = []
        self.odom_hold_until = 0.0
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
        self.create_subscription(Float32, '/maze/body_height',
                                 lambda m: setattr(self, 'body_height', m.data), 5)
        if self.cv_on:
            self.create_subscription(String, '/curtain/seen', self.on_curtain_seen, 10)
        self.create_subscription(Empty, '/nav/start', lambda m: self.begin(), 1)
        self.create_subscription(Empty, '/explore/stop', lambda m: self.halt('STOPPED', 'stop asked'), 1)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_cells)

    # ---- inputs -------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_curtain_seen(self, msg):
        """One camera frame from curtain_node (it only works outside room checks)."""
        try:
            dets = json.loads(msg.data).get('detections') or []
        except (ValueError, AttributeError):
            return
        now = self.now()
        self.cv_frames.append((now, any(centred(d, self.cv_conf) for d in dets)))
        if self.state != 'RUN' or self.grid is None or self.pose is None or self.occ is None \
                or (self.act and self.act[0] in ('turn', 'cv_verify')):
            return                    # while turning the heading of the frame is not known
        sense = self.sense()
        for d in dets:
            if d.get('conf', 0.0) < self.cv_conf:
                continue
            edge = edge_along(self.grid, sense, self.pose, float(d.get('bearing', 0.0)))
            if edge is None or edge in self.cv_curtains or edge in self.cv_rejected \
                    or edge in self.cv_pending or edge in self.cv_checking \
                    or self.kinds.get(edge) == 'curtain' \
                    or edge in self.links:
                continue
            times = [t for t in self.cv_seen.get(edge, []) if now - t <= self.cv_window] + [now]
            self.cv_seen[edge] = times
            if len(times) >= self.cv_hits:
                self.cv_pending.add(edge)
                self.get_logger().info(f'camera: curtain seen towards cell {edge[0]} side '
                                       f'{edge[1]} on {len(times)} frames: to be verified')

    def on_map(self, msg):
        i = msg.info
        self.occ = Occupancy(np.asarray(msg.data, np.int8).reshape(i.height, i.width),
                             i.resolution, i.origin.position.x, i.origin.position.y)

    def on_cloud(self, msg):
        pts = np.frombuffer(msg.data, np.float32).reshape(-1, 3)
        self.cloud, self.cloud_time = pts, self.now()
        self.clouds.append(pts)
        if self.pose is None or not len(pts) or self.now() < self.blind_until:
            return
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        self.heights.add(x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1],
                         pts[:, 2])

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom_rot = stair_levels.quat_to_matrix(q.x, q.y, q.z, q.w)
        if self.last_odom is not None:
            t0, x0, y0 = self.last_odom
            jump = math.hypot(p.x - x0, p.y - y0)
            dt = max(t - t0, 0.0)
            # More than the robot can walk in that time (0.6 m/s) plus 0.15 m. After a gap in
            # the lidar stream Fast-LIO catches up in one step (run3, 2026-11-24: 0.48 m in
            # 0.40 s after a 1 s stall, pose fine afterwards), so one jump only pauses the
            # robot. A runaway keeps jumping or goes far: that latches.
            if jump > 0.15 + 0.6 * dt:
                now = self.now()
                self.odom_jumps = [j for j in self.odom_jumps if now - j < 10.0] + [now]
                if now < self.odom_grace_until:     # depth stream on: the lidar feed is slow
                    self.odom_jumps = []
                elif jump > 2.0 or len(self.odom_jumps) >= 3:
                    self.halt('FAULT', f'/odom jumped {jump:.2f} m in {dt:.2f} s '
                                       f'({len(self.odom_jumps)} jumps in 10 s): odometry has '
                                       'diverged. Stop, run explore_check, relaunch')
                elif self.state == 'RUN':
                    self.get_logger().warn(f'/odom jumped {jump:.2f} m in {dt:.2f} s: paused '
                                           f'{self.odom_settle:.0f} s to see that it holds')
                self.odom_hold_until = now + self.odom_settle
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
        else:
            # Resuming after a stop. Whoever stopped it may have taken over with the handheld
            # controller, or left AI mode with lite3_ai_off: both leave the robot in manual
            # mode, where it ignores /cmd_vel. So switch it back before anything else
            # (Move -> gait -> navigation mode, 1.5 s apart, standing still).
            self.get_logger().info(f'resuming: sending Move, {self.gait} and navigation mode again')
            for code in robot.start_sequence(self.gait):
                self.queue.append(('cmd', code))
                self.queue.append(('hold', self.cmd_gap))
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
            self.curtain_latched.clear()      # same: they name cells of the unfitted grid
            self.cv_curtains.clear()
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
        if (cell, d) in self.cv_curtains and self.curtains_on:
            return 'curtain'          # verified with the camera: passable, whatever the map says
        if (cell, d) in self.curtain_latched and self.curtains_on:
            return 'curtain'          # read as a curtain before: stays one
        k = sense.kind(cell, d, may_exit=cell != self.start)
        if k == 'curtain' and not self.curtains_on:
            k = 'wall'
        if k == 'curtain':
            self.curtain_latched.update({(cell, d), (b, opp(d))})
            self.get_logger().info(f'curtain latched: cell {cell} side {d} (and cell {b} side {opp(d)})')
        if k == 'stairs' and not self.stairs_on:
            k = 'blocked'
        # The depth camera has the last word on stairs. Until it has looked, something that
        # blocks the next cell is kept as possible stairs (looked at before anything is done).
        checked = self.stairs_checked.get((cell, d))
        if checked is True:
            return 'stairs'
        if checked is False and k == 'stairs':
            k = 'blocked'
        if checked is None and k == 'blocked' and self.stair_check_on:
            k = 'stairs'
        if k == 'room' and any((b, e) in self.cv_curtains or (b, e) in self.curtain_latched
                               for e in range(4)):
            k = 'free'                # not a dead end: one of its "walls" is a curtain
        if k == 'exit' and self.exit_cell is None:
            self.exit_cell, self.exit_d = cell, d
            self.event(f'exit seen from cell {cell} side {d}')
        return k

    def sense(self):
        return Sense(self.grid, self.occ, self.heights,
                     dict(terrain_min=self.terrain_min, terrain_squares=self.terrain_squares))

    def update_kinds(self):
        sense = self.sense()
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
        # Every job starts from a cell centre. Turns end on heading alone and nothing drives
        # backwards, so a robot that has ended up off the centre is walked back onto it first.
        cx, cy = self.grid.centre(cur)
        off = math.hypot(cx - self.pose[0], cy - self.pose[1])
        if off > self.recentre and self.recentre_tries < 2:
            self.recentre_tries += 1
            back = math.atan2(cy - self.pose[1], cx - self.pose[0])
            self.get_logger().info(f'{off:.2f} m off the centre of cell {cur}: walking back onto it')
            self.queue.extend([('turn', back, 5.0), ('drive', cx, cy, back, 'creep')])
            return
        self.recentre_tries = 0
        self.update_kinds()
        # A curtain the camera has seen is verified from a cell centre, looking straight at
        # its edge: from the cell before it (the camera is then 1.3 m away, inside the range
        # the model was trained for), or from its own cell when the robot is already there.
        self.cv_checking.clear()              # plan() runs with an empty queue: none is under way
        for a, d in sorted(self.cv_pending):
            if a == cur or (nb(cur, d) == a and self.kinds.get((cur, d)) in ('free', 'room')):
                self.cv_pending.discard((a, d))
                self.cv_checking.add((a, d))
                h = self.grid.heading(d)
                self.queue.extend([('turn', h, 2.0), ('hold', 0.5), ('cv_verify', a, d)])
                return
        facing = self.grid.nearest_dir(self.pose[2])
        j = next_job(self.kinds, self.visited, self.closed, self.links, cur, facing,
                     self.looked, self.greeted, self.exit_cell, self.enter_rooms)
        job = f"{j['job']} cell {j['cell']} side {j['d']} via {len(j['path'])} cells"
        if job != self.job:
            here = ' '.join(self.kinds.get((cur, k), '-') for k in range(4))
            self.get_logger().info(f'job: {job}   [in {cur}: ahead/left/back/right = {here}; '
                                   f'{off:.2f} m off its centre]')
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
        elif j['job'] == 'look' and self.look_check(c, d, h):
            pass                              # the check is queued; the look is planned again after it
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

    def look_check(self, c, d, h):
        """Queue the stairs and curtain checks that come before a room / object look.
        Returns False when there is nothing (left) to check."""
        if not self.look_check_on or (c, d) in self.look_checked:
            return False
        stairs = self.stair_check_on and (c, d) not in self.stairs_checked
        edge = None
        if self.cv_on:                        # the first closed edge straight ahead
            cx, cy = self.grid.centre(c)
            edge = edge_along(self.grid, self.sense(), (cx, cy, h), 0.0)
            if edge in self.cv_curtains or edge in self.curtain_latched or edge in self.cv_rejected \
                    or edge in self.links:
                edge = None
        self.look_checked.add((c, d))
        if not stairs and edge is None:
            return False
        q = self.queue
        q.extend([('turn', h, 2.0), ('hold', 0.5)])
        if stairs:
            q.append(('stair_check', c, d))
        if edge is not None:
            self.cv_pending.discard(edge)
            self.cv_checking.add(edge)
            q.append(('cv_verify', *edge))

        def verdict():
            found = [w for w, yes in (('stairs', self.stairs_checked.get((c, d)) is True),
                                      ('curtain', edge is not None and edge in self.cv_curtains)) if yes]
            if found:
                self.looked.add((c, d))       # no views: scene understanding gets nothing to send
                self.event(f'look called off, cell {c} side {d}: {" and ".join(found)} verified')
        q.append(('call', verdict))
        return True

    def go(self, a, d, how):
        """Queue one step from the centre of cell a through side d."""
        q, h, b = self.queue, self.grid.heading(d), nb(a, d)
        ax, ay = self.grid.centre(a)
        stairs = self.stair_check_on and (a, d) not in self.stairs_checked and (
            how == 'stairs' or (how == 'walk' and b not in self.visited and self.raised(a, d)))
        # the lidar has hardly seen the floor in there: let the depth camera read it
        terrain = (self.terrain_on and self.terrain_depth and how == 'walk'
                   and b not in self.visited and (a, d) not in self.rough_checked
                   and self.sense().floor_seen(a, d) < self.terrain_seen_min
                   and not self.sense().rough(a, d))      # already uneven: nothing to add
        if stairs or terrain:
            # look first; the next plan then knows whether this is stairs / uneven
            q.extend([('drive', ax, ay, h, 'square'), ('turn', h, 2.0), ('hold', 0.5),
                      ('stair_check', a, d, stairs)])
            return
        if how == 'walk' and self.terrain_on and self.is_rough(a, d):
            # Uneven cell ahead: turn on the spot until the body is square to the way in
            # (whichever quarter turn is nearer), then step sideways to the next centre.
            body = min((wrap(h + math.pi / 2), wrap(h - math.pi / 2)),
                       key=lambda t: abs(wrap(t - self.pose[2])))
            bx, by = self.grid.centre(b)
            q.extend([('turn', body, 3.0),
                      ('event', f'terrain: sideways into cell {b} from cell {a} side {d}'),
                      ('call', lambda: setattr(self, 'step', (a, d, b))),
                      ('strafe', bx, by, h, body),
                      ('call', lambda: (self.walked.update({(a, d), (b, opp(d))}), self.arrive(b)))])
        elif how == 'walk':
            bx, by = self.grid.centre(b)
            q.extend([('turn', h, 4.0), ('call', lambda: setattr(self, 'step', (a, d, b))),
                      ('drive', bx, by, h, 'normal'),
                      ('call', lambda: (self.walked.update({(a, d), (b, opp(d))}), self.arrive(b)))])
        elif how == 'curtain':
            bx, by = self.grid.centre(b)

            def crossed():
                self.links[(a, d)], self.links[(b, opp(d))] = ('curtain', b), ('curtain', a)
                self.arrive(b)
            # Sideways (user, 2026-11-25): square up on the centre line facing the curtain,
            # make the nearer quarter turn on the spot, step sideways through it all the way
            # to the centre of the cell behind, and there turn the quarter back on the spot
            # (about that centre), so the robot faces the way it came through.
            body = min((wrap(h + math.pi / 2), wrap(h - math.pi / 2)),
                       key=lambda t: abs(wrap(t - self.pose[2])))
            q.extend([('drive', ax, ay, h, 'square'), ('turn', body, 2.0),
                      ('event', f'curtain: going through sideways, cell {a} side {d}'),
                      ('call', lambda: setattr(self, 'step', (a, d, b))),
                      ('strafe', bx, by, h, body, 'push'), ('turn', h, 3.0),
                      ('call', crossed)])
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

    def do_cv_verify(self, cell, d):
        """Standing still, facing side d of `cell` squarely: does the camera keep seeing a
        curtain straight ahead? Yes = that edge is a curtain, also where it reads as a wall."""
        self.send()
        if self.elapsed() < self.cv_verify_time:
            return False
        frames = [hit for t, hit in self.cv_frames if t >= self.act_t0]
        b = nb(cell, d)
        if sum(frames) >= self.cv_verify_hits:
            self.cv_curtains.update({(cell, d), (b, opp(d))})
            self.forced_wall.difference_update({(cell, d), (b, opp(d))})
            self.event(f'curtain: seen by the camera, cell {cell} side {d} '
                       f'({sum(frames)} of {len(frames)} frames): passable')
        else:
            self.cv_rejected.add((cell, d))
            self.event(f'curtain check cell {cell} side {d}: not confirmed by the camera '
                       f'({sum(frames)} of {len(frames)} frames)')
        return True

    def raised(self, cell, d):
        """The lidar shows something stair-high (not just terrain) in the cell behind side d."""
        _, front, top = self.sense().interior(cell, d)
        return max([v for v in (front, top) if not math.isnan(v)], default=0.0) >= self.stair_raise_min

    def do_stair_check(self, cell, d, stairs=True):
        """Standing still, facing side d: watch the depth camera for stair_check_time s.
        It looks for stairs (unless stairs=False) and reads the floor of the next cell for
        uneven terrain (terrain_from_depth)."""
        self.send()
        mem, now = self.act_mem, self.now()
        self.odom_grace_until = now + 4.0
        if 'subs' not in mem:
            mem.update(frame=None, k=None, frames=0, hits=0, why='no depth frame arrived',
                       confirm=stair_levels.Confirm(), rails='', mount=None, rough=[])
            mem['subs'] = [
                self.create_subscription(Image, self.depth_topic,
                                         lambda m: mem.update(frame=m), qos_profile_sensor_data),
                self.create_subscription(CameraInfo, self.depth_info_topic,
                                         lambda m: mem.update(k=(m.k[0], m.k[4], m.k[2], m.k[5])),
                                         qos_profile_sensor_data)]
            self.get_logger().info(f'stair check: cell {cell} side {d}, depth camera on')
            return False
        if mem['mount'] is None:
            try:
                tf = self.tf_buffer.lookup_transform('base_link', 'camera_link', Time())
                t, q = tf.transform.translation, tf.transform.rotation
                mem['mount'] = (stair_levels.quat_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z]))
            except TransformException:       # measured mount: 0.252 m ahead, 20 deg down
                mem['mount'] = (stair_levels.quat_to_matrix(0.0, 0.17364817766693033, 0.0,
                                                            0.984807753012208),
                                np.array([0.25228, -0.0005, 0.0741]))
        found = None
        msg, mem['frame'] = mem['frame'], None
        if msg is not None and mem['k'] is not None and self.odom_rot is not None:
            depth = self.depth_image(msg)
            if depth is not None:
                level = stair_levels.roll_pitch_only(self.odom_rot)
                r, t = stair_levels.level_camera(level, *mem['mount'])
                if self.terrain_on and self.terrain_depth:
                    mem['rough'].append(self.terrain_from_depth(depth, mem['k'], r, t, cell, d))
                # The detector looks for the floor within 12 cm of the standing height it is
                # given. Try the measured one, then the two usual ones (0.35 nominal, 0.30).
                h = self.body_height
                seen = None
                for height in (([h] if h and 0.25 <= h <= 0.40 else []) + [0.35, 0.30]) if stairs else []:
                    self.stair_params.body_height = height
                    seen = stair_levels.detect(depth, mem['k'], r, t, self.stair_params)
                    if seen is not None:
                        break
                mem['why'] = ('stairs seen' if seen else stair_levels.detect.last_why) if stairs \
                    else 'stairs not looked for'
                if seen is not None and self.stair_rails:
                    pts = np.concatenate(list(self.clouds)).astype(float) if self.clouds else None
                    if pts is not None:      # /maze/cloud z = height above the floor
                        pts[:, 2] += seen.floor_z
                    ok, mem['rails'] = stair_levels.rails_check(pts, seen)
                    if not ok:
                        mem['why'], seen = f'treads look like stairs but {mem["rails"]}', None
                mem['frames'] += 1
                mem['hits'] += seen is not None
                found = mem['confirm'].add(now, seen)
        waited = self.elapsed()
        quick_no = mem['hits'] == 0 and mem['frames'] >= self.stair_quick_no   # close depth early
        quick_no = quick_no and sum(r is not None for r, _ in mem['rough']) >= 3 \
            if self.terrain_on and self.terrain_depth else quick_no
        if found is None and not quick_no and \
                waited < self.stair_check_time + (2.0 if mem['frames'] == 0 else 0.0):
            return False
        for sub in mem['subs']:
            self.destroy_subscription(sub)
        if found is not None:
            verdict = True
            text = f'STAIRS: {found.summary()}; {mem["rails"]}'
        elif mem['frames'] == 0:
            verdict = 'lidar'                # no camera: the lidar's reading stands
            text = f'no depth frames in {waited:.1f} s; keeping what the lidar says'
        else:
            verdict = False
            text = f'not stairs ({mem["hits"]} of {mem["frames"]} frames; last: {mem["why"]})'
        if stairs:
            self.stairs_checked[(cell, d)] = verdict
            self.event(f'stair check cell {cell} side {d}: {text}')
        if self.terrain_on and self.terrain_depth:
            votes = [r for r, _ in mem['rough'] if r is not None]
            seen = max((n for _, n in mem['rough']), default=0)
            rough = None if len(votes) < 3 else sum(votes) * 2 > len(votes)
            self.rough_checked[(cell, d)] = rough
            self.event(f'terrain check cell {nb(cell, d)} (from cell {cell} side {d}): ' + (
                f'too little floor seen ({len(votes)} usable frames, {seen} squares)' if rough is None
                else f'{"UNEVEN" if rough else "flat"} ({sum(votes)} of {len(votes)} frames uneven, '
                     f'up to {seen} squares seen)'))
        self.queue.appendleft(('hold', 1.5))        # the robot's streams recover
        return True

    def terrain_from_depth(self, depth, k, r, t, cell, d):
        """One depth frame -> (rough, squares seen) for the cell behind side d, or
        (None, n). Points in the level frame (base_link without roll / pitch), the floor
        found as the lowest dense layer near the standing height, then onto the map."""
        g = stair_levels.grid_points(depth, k, r, t, self.stair_params)
        pts = g.reshape(-1, 3)
        pts = pts[np.isfinite(pts[:, 2])]
        if len(pts) < 100 or self.pose is None:
            return None, 0
        h = self.body_height if self.body_height and 0.25 <= self.body_height <= 0.40 else 0.30
        self.stair_params.body_height = h
        floor, measured = stair_levels.find_floor(pts, self.stair_params)
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my = x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]
        return self.sense().rough_from_points(cell, d, mx, my, pts[:, 2] - floor)

    def is_rough(self, a, d):
        """Uneven terrain behind side d of cell a: the depth camera's verdict if it gave
        one, else the lidar's height map."""
        checked = self.rough_checked.get((a, d))
        return checked if checked is not None else self.sense().rough(a, d)

    @staticmethod
    def depth_image(msg):
        if msg.encoding in ('16UC1', 'mono16'):
            kind = np.uint16
        elif msg.encoding == '32FC1':
            kind = np.float32
        else:
            return None
        a = np.frombuffer(bytes(msg.data), kind)
        a = a.reshape(msg.height, msg.step // np.dtype(kind).itemsize)[:, :msg.width]
        return a.byteswap() if msg.is_bigendian else a

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
            if self.now() < self.odom_hold_until:
                self.send()
                self.why = 'odometry jumped: paused until it has held steady'
                self.act_t0 += 0.1
                if 't' in self.act_mem:
                    self.act_mem['t'] += 0.1
            elif age > self.pose_timeout and not (self.act and self.act[0] == 'stair_check'):
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
        """Turn on the spot to heading `target`, WITHOUT leaving the spot. Left alone, the
        Lite3 walks backwards while it turns (run1: 9 cm per 90 deg, 0.3 m in a half turn),
        which puts its rear into the wall behind. So the turn has a pivot point (the cell
        centre if the robot stands near it, else where the turn began) and the robot is
        steered back onto it all the way round: forward only (never a backward command),
        sideways both ways, plus a little forward that cancels the usual drift."""
        x, y, yaw = self.pose
        mem = self.act_mem
        if 'pivot' not in mem:
            mem['pivot'] = (x, y)
            if self.grid is not None:
                cx, cy = self.grid.centre(self.cur_cell())
                if math.hypot(cx - x, cy - y) <= self.pivot_snap:
                    mem['pivot'] = (cx, cy)
        dx, dy = mem['pivot'][0] - x, mem['pivot'][1] - y
        ahead = dx * math.cos(yaw) + dy * math.sin(yaw)       # pivot ahead of the robot (+)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        err = wrap(target - yaw)
        if abs(err) < math.radians(tol_deg):
            mem['ok'] = mem.get('ok', 0) + 1
            self.send()
            return mem['ok'] >= 3
        mem['ok'] = 0
        if self.elapsed() > 25.0:
            self.get_logger().warn(f'turn timed out {math.degrees(err):.0f} deg short')
            self.send()
            return True
        wz = clamp(1.5 * err, -self.w_max, self.w_max)
        wz = math.copysign(max(abs(wz), self.min_spin), wz)
        vx = clamp(1.5 * ahead, 0.0, 0.12)
        if ahead > -0.03:
            vx += self.turn_ff * abs(wz) / self.w_max
        if self.front_blocked():
            vx = 0.0
        self.send(vx=vx, vy=clamp(1.5 * left, -0.10, 0.10), wz=wz)
        return False

    def front_blocked(self):
        """Something from knee height up, 6-20 cm ahead of the body, in the body's width."""
        p = self.cloud
        m = (p[:, 0] > self.fp_front + 0.06) & (p[:, 0] < self.fp_front + 0.20) & \
            (np.abs(p[:, 1]) < self.fp_half_width - 0.03) & (p[:, 2] > 0.30) & (p[:, 2] < 1.0)
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

    def side_blocked(self, sign):
        """Something from knee height up, 6-20 cm beside the body on the side it steps
        towards (sign +1 = left), along the body's length."""
        p = self.cloud
        y = sign * p[:, 1]
        m = (y > self.fp_half_width + 0.06) & (y < self.fp_half_width + 0.20) & \
            (p[:, 0] > -self.fp_rear + 0.03) & (p[:, 0] < self.fp_front - 0.03) & \
            (p[:, 2] > 0.30) & (p[:, 2] < 1.0)
        return int(m.sum()) >= 6

    def do_strafe(self, tx, ty, travel, body, mode='side'):
        """Step sideways to (tx, ty): the robot moves along `travel` while its body keeps
        the heading `body` (travel +- 90 deg). Used to enter uneven cells. The base_link line
        is side_offset towards the nose, which centres the body's length in the way in."""
        x, y, yaw = self.pose
        c, s = math.cos(travel), math.sin(travel)
        tx, ty = tx + self.side_offset * math.cos(body), ty + self.side_offset * math.sin(body)
        dx, dy = tx - x, ty - y
        along, lat, herr = dx * c + dy * s, -dx * s + dy * c, wrap(body - yaw)
        mem = self.act_mem
        if mode == 'push':                   # curtain over the lidar: wall_scan sends no scans
            self.blind_pub.publish(Bool(data=True))
            self.blind_until = self.now() + 1.5
        if along <= 0.03:
            self.send()
            return True
        mem.setdefault('best', along)
        if along < mem['best'] - 0.05:
            mem['best'], mem['t'] = along, self.now()
        if self.now() - mem.setdefault('t', self.now()) > 8.0:
            self.send()
            self.get_logger().warn(f'sideways step: no progress, {along:.2f} m short')
            return self.stuck(mode, along)
        if abs(herr) > math.radians(25):         # knocked round: turn back first, on the spot
            self.send(wz=math.copysign(max(self.min_spin, min(self.w_max, 1.5 * abs(herr))), herr))
            return False
        side = 1.0 if math.sin(wrap(travel - body)) > 0 else -1.0     # +1: stepping to the left
        if (mode != 'push' or self.push_stop) and along > 0.10 and self.side_blocked(side):
            self.send()
            mem['blocked'] = mem.get('blocked', 0) + 1
            if mem['blocked'] >= 8:
                self.get_logger().warn(f'way blocked {along:.2f} m before the cell centre (sideways)')
                return self.stuck(mode, along)
            return False
        mem['blocked'] = 0
        # wanted velocity in the map frame, then into the body frame as it stands now
        v, w = clamp(1.0 * along, 0.06, self.side_speed), clamp(0.8 * lat, -0.10, 0.10)
        wx, wy = v * c - w * s, v * s + w * c
        self.send(vx=wx * math.cos(yaw) + wy * math.sin(yaw),          # send() never lets
                  vy=-wx * math.sin(yaw) + wy * math.cos(yaw),         # vx go backwards
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
        self.walked.difference_update({(a, d), (b, opp(d))})    # blocked now: not "walked"
        self.cv_curtains.difference_update({(a, d), (b, opp(d))})
        self.curtain_latched.difference_update({(a, d), (b, opp(d))})
        self.cv_rejected.add((a, d))
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
