"""
Poster detector node: reports pictures of people, ignores real people (robot build).

Takes person boxes from human_detector and decides, for each one, whether it is a poster
using two tests from poster_utils:
  1. edge-ring test: on a poster the surface continues just outside the box; behind a
     real person the depth jumps backwards.
  2. static-map test: a poster hangs on a wall/board that is in the static map; a real
     person stands in free space.
A poster is reported once it wins `vote_need` of the last `vote_window` frames.

Posters are tracked in track_frame (Fast-LIO's odom: there from launch, never jumps), so
positions are only ever compared in one fixed frame: never the camera frame, never map (which
jumps on SLAM corrections). The survivor's location is reported as the centre of a maze cell
(cell_size / origin_x / origin_y / grid_yaw_deg, same as maze_nav's scene_trigger): the cell
of the poster point pulled back room_pullback_m towards the camera (walls sit on cell edges),
taken into map with the latest map <- odom. Before SLAM is up the cell comes from odom (the
robot holds still until /maze/start, so they start out together); when SLAM comes up, or a
correction moves a survivor into another cell, it is reported again (no new "Human found").
TF lookups never wait: a wait in this callback starves the node's /tf subscription.

Topics (all names are parameters - VERIFY on the robot with `ros2 topic list`):
  in : /humans/detections   std_msgs/String  JSON from human_detector (boxes + stamp)
  in : depth_topic          sensor_msgs/Image  depth ALIGNED to the colour image
  in : camera_info_topic    sensor_msgs/CameraInfo  (intrinsics of that depth image)
  in : map_topic            nav_msgs/OccupancyGrid  static map (transient local)
  out: /posters/detections        std_msgs/String  JSON, one per newly confirmed poster
  out: /found                     std_msgs/String  "Human found", once per new poster position
"""
import bisect
import csv
from collections import deque
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)

from team_perception import tf_pin
from team_perception.poster_utils import (MAP_MODES, Announced, MapParams, MazeCells,
                                          PosterTracker, QrRooms, RingParams, pull_back,
                                          StaticMap, combine, edge_ring_test, static_map_test,
                                          to_metres, torso_pixel_and_depth)
from team_perception.qr_utils import DepthAlignState, transform_first


def _quat_matrix(x, y, z, w):
    """3x3 rotation matrix of a unit quaternion."""
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _stamp(msg_header):
    return Time.from_msg(msg_header.stamp).nanoseconds * 1e-9


class StampBuffer:
    """Keep the last few seconds of messages (sorted by stamp) and find the one closest to a
    stamp. Messages may arrive out of order (a late one is inserted where it belongs)."""

    def __init__(self, keep_s=2.0):
        self.keep = keep_s
        self.stamps, self.msgs = [], []

    def add(self, stamp, msg):
        i = bisect.bisect_right(self.stamps, stamp)
        self.stamps.insert(i, stamp)
        self.msgs.insert(i, msg)
        cut = bisect.bisect_left(self.stamps, self.stamps[-1] - self.keep)
        if cut:
            del self.stamps[:cut], self.msgs[:cut]

    def match(self, stamp, max_dt):
        """
        ('ok', msg) if a message lies within max_dt of stamp, else ('wait', dt): one may
        still arrive (the depth source is sometimes ~0.3 s late, even out of order), so the
        caller holds the detection for a while. dt = gap to the closest one (None = empty).
        """
        msg, dt = self.closest(stamp, max_dt)
        return ('ok', msg) if msg is not None else ('wait', dt)

    def closest(self, stamp, max_dt):
        if not self.stamps:
            return None, None
        s = self.stamps
        i = bisect.bisect_left(s, stamp)
        best = min((j for j in (i - 1, i) if 0 <= j < len(s)), key=lambda j: abs(s[j] - stamp))
        dt = abs(s[best] - stamp)
        return (self.msgs[best], dt) if dt <= max_dt else (None, dt)


class PosterDetector(Node):
    def __init__(self):
        super().__init__('poster_detector')
        p = self.declare_parameter
        p('detections_topic', '/humans/detections')
        p('depth_topic', '/camera/depth/image_raw')
        p('depth_camera_info_topic', '/camera/depth/camera_info')  # '' = depth is
        #                                already aligned to colour
        p('camera_info_topic', '/camera/color/camera_info')
        p('map_topic', '/map')
        p('map_frame', 'map')
        p('track_frame', 'odom')       # fixed frame posters are tracked in (Fast-LIO: never jumps)
        p('use_map', True)
        p('map_mode', 'veto')          # 'require' | 'veto' | 'off' (see poster_utils.combine)
        p('max_dt_s', 0.1)             # max stamp difference between detection and depth
        p('depth_wait_s', 0.5)         # hold a detection this long for its depth to arrive
        # edge-ring test
        p('ring_tol_m', 0.05)
        p('ring_tol_quad', 0.01)
        p('ring_step_m', 0.25)
        p('ring_frac', 0.10)
        p('max_range_m', 4.0)
        p('min_sides', 2)
        # static-map test
        p('map_on_radius_m', 0.30)
        p('map_free_radius_m', 0.60)
        p('map_occupied', 65)
        # voting / reporting
        p('merge_radius_m', 0.5)
        p('vote_window', 8)
        p('vote_need', 5)
        p('repeat_after_s', 60.0)
        p('log_dir', os.path.expanduser('~/poster_logs'))
        # Maze cells (same names and defaults as maze_nav's scene_trigger): the survivor is
        # reported as the centre of the cell the poster is seen in.
        p('cell_size', 1.02)           # m (maze_info.md / booklet: 1.02 m cell pitch)
        p('grid_topic', '/maze/grid')  # maze_explorer's wall-fitted grid; '' = fixed grid
        p('origin_x', 0.0)             # map (x, y) of the start cell's centre
        p('origin_y', 0.0)
        p('grid_yaw_deg', 0.0)         # map heading of the grid's +i axis
        p('room_pullback_m', 0.25)     # step back from the wall towards the camera
        p('cell_check_period_s', 1.0)  # re-check reported cells after SLAM corrections
        # QR codes only appear in rooms with survivors (not every survivor room has one)
        p('qr_topic', '/qr/detections')
        p('room_radius_m', 1.0)        # poster and code room points this close = same room
        p('vote_need_qr', 3)           # poster votes needed in a room with a QR code
        p('look_delay_s', 3.0)         # no poster this long after a code: ask to look around
        p('look_topic', '/perception/look_request')   # PointStamped (odom), for maze_explorer
        p('leave_radius_m', 1.5)       # robot this far from a code's room: it has left it
        p('qr_backup_timeout_s', 90.0)  # ... or this long after the code: report from QR alone
        p('base_frame', 'base_link')

        def g(name):
            return self.get_parameter(name).value
        self.g = g

        self.map_frame = g('map_frame')
        self.use_map = g('use_map')
        self.map_mode = g('map_mode') if self.use_map else 'off'
        if self.map_mode not in MAP_MODES:
            raise ValueError(f'map_mode must be one of {MAP_MODES}')
        self.max_dt = g('max_dt_s')
        self.depth_wait = g('depth_wait_s')
        # Detections whose depth frame hasn't arrived yet (depth_relay is sometimes ~0.3 s
        # late): retried on every new depth frame, dropped after depth_wait_s. Never waited
        # for in a callback (that would starve /tf, see qr_utils.transform_first).
        self.pending = deque(maxlen=10)        # (det, received monotonic time)
        self.ring_p = RingParams(tol_m=g('ring_tol_m'), tol_quad=g('ring_tol_quad'),
                                 step_m=g('ring_step_m'), ring_frac=g('ring_frac'),
                                 max_range_m=g('max_range_m'), min_sides=g('min_sides'))
        self.map_p = MapParams(on_radius_m=g('map_on_radius_m'),
                               free_radius_m=g('map_free_radius_m'),
                               occupied=g('map_occupied'))
        # All tracking and "Human found" de-duplication in ONE fixed frame (odom): never the
        # camera frame (moves with the robot), never map (jumps on SLAM corrections).
        self.track_frame = g('track_frame')
        self.tracker = PosterTracker(g('merge_radius_m'), g('vote_window'), g('vote_need'),
                                     repeat_after_s=g('repeat_after_s'))
        self.announced = Announced(g('merge_radius_m'))

        self.cells = MazeCells(g('cell_size'), (g('origin_x'), g('origin_y')),
                               math.radians(g('grid_yaw_deg')))
        self.pullback = g('room_pullback_m')
        self.qr = QrRooms(g('room_radius_m'))
        self.vote_need_qr = g('vote_need_qr')
        self.look_delay = g('look_delay_s')
        self.leave_radius = g('leave_radius_m')
        self.qr_timeout = g('qr_backup_timeout_s')
        self.base_frame = g('base_frame')
        if g('cell_check_period_s') > 0:   # 0 = never re-place reported survivors
            self.create_timer(g('cell_check_period_s'), self.check_cells)   # + QR rooms
        else:
            self.create_timer(1.0, self.check_qr_rooms)

        self.bridge = CvBridge()
        self.depth_buf = StampBuffer()
        self.K = None
        self.align_state = DepthAlignState(bool(g('depth_camera_info_topic')))
        self.static_map = None

        self.create_subscription(String, g('detections_topic'), self.on_detections, 10)
        self.create_subscription(Image, g('depth_topic'),
                                 self.on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, g('camera_info_topic'), self.on_info,
                                 qos_profile_sensor_data)
        if g('depth_camera_info_topic'):
            self.create_subscription(CameraInfo, g('depth_camera_info_topic'),
                                     self.on_depth_info, qos_profile_sensor_data)
        if self.use_map:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, g('map_topic'), self.on_map, latched)

        self.pub = self.create_publisher(String, '/posters/detections', 10)
        self.found_pub = self.create_publisher(String, '/found', 10)   # "Human found", once each
        self.look_pub = self.create_publisher(PointStamped, g('look_topic'), 10)
        self.create_subscription(String, g('qr_topic'), self.on_qr, 10)
        if g('grid_topic'):
            latched_grid = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                      reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(String, g('grid_topic'), self.on_grid, latched_grid)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Robot-side /tf_static gives the depth camera a second, unpitched parent: keep the
        # correct one (team_perception/tf_pin.py).
        self.tf_pin = tf_pin.attach(self, self.tf_buffer)

        self.log_dir = g('log_dir')
        # record of confirmed posters; created on the first one, not at startup
        self.csv_path = os.path.join(self.log_dir, time.strftime('posters_%Y%m%d_%H%M%S.csv'))
        self.get_logger().info(
            f'Poster detector: {g("detections_topic")} + {g("depth_topic")}, map_mode='
            f'{self.map_mode}{" (" + g("map_topic") + ")" if self.use_map else ""}')

    # --- inputs ------------------------------------------------------------------
    def on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)
        self.align_state.set_colour(self.K[:4], (msg.width, msg.height),
                                    msg.header.frame_id)

    def on_depth_info(self, msg):
        self.align_state.set_depth((msg.k[0], msg.k[4], msg.k[2], msg.k[5]),
                                   msg.header.frame_id)

    def on_map(self, msg):
        q = msg.info.origin.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.static_map = StaticMap(msg.data, msg.info.width, msg.info.height,
                                    msg.info.resolution,
                                    (msg.info.origin.position.x, msg.info.origin.position.y),
                                    float(yaw))
        self.get_logger().info(f'Static map received: {msg.info.width}x{msg.info.height} '
                               f'@ {msg.info.resolution:.3f} m')

    # --- main step ---------------------------------------------------------------
    def on_depth(self, msg):
        self.depth_buf.add(_stamp(msg.header), msg)
        self.retry_pending()           # a held detection's depth may be here now

    def retry_pending(self):
        """Process held detections whose depth is here; drop (and warn) expired ones."""
        if self.pending:
            waiting, self.pending = list(self.pending), deque(maxlen=self.pending.maxlen)
            for det, since in waiting:
                self.process(det, since)

    def on_detections(self, msg):
        self.check_cells()             # re-place reported survivors first (SLAM may be up now)
        try:
            det = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn('bad JSON on detections topic', throttle_duration_sec=5.0)
            return
        self.retry_pending()           # also expires held ones if depth stopped coming
        self.process(det, time.monotonic())

    def process(self, det, since):
        """Classify one detection message against its depth frame, or hold it for that."""
        stamp = det.get('stamp', 0.0)
        state, found = self.depth_buf.match(stamp, self.max_dt)
        if state == 'wait' and time.monotonic() - since < self.depth_wait:
            self.pending.append((det, since))
            return
        if state != 'ok':
            waited = (time.monotonic() - since) * 1000
            self.get_logger().warn(
                'no depth image within max_dt_s of the detection'
                + (f' (closest {found * 1000:.0f} ms' if found is not None else ' (no depth yet')
                + (f', waited {waited:.0f} ms)' if waited >= 1 else ')'),
                throttle_duration_sec=5.0)
            return
        depth_msg = found
        depth = to_metres(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'))
        align = self.depth_align(depth.shape)
        if align is None:
            return

        for person in det.get('people', []):
            r = self.classify(person, det, depth, depth_msg.header, align)
            if r['report']:
                self.report(r, stamp)

    def classify(self, person, det, depth, depth_header, align):
        # Colour box -> depth-image box: first without parallax to get the torso depth,
        # then again corrected for the offset between the two sensors at that depth.
        size = det.get('image_size')
        box = align.box(person['box'], image_size=size)
        tp = torso_pixel_and_depth(depth, box, self.ring_p)
        if tp is not None:
            box = align.box(person['box'], z=tp[2], image_size=size)
        ring, ring_m = edge_ring_test(depth, box, self.ring_p)
        out = {'box': person['box'], 'conf': person.get('conf'), 'ring': ring,
               'ring_metrics': ring_m, 'map': 'unknown', 'map_metrics': {},
               'position': None, 'report': False}

        loc = self.locate(depth, box, depth_header, align)
        if loc is not None:
            out['position'] = [round(v, 3) for v in loc['poster']]
            if self.map_mode != 'off' and loc['poster_map'] is not None:
                out['map'], out['map_metrics'] = static_map_test(
                    self.static_map, loc['poster_map'][0], loc['poster_map'][1], self.map_p)
            elif self.map_mode != 'off':
                out['map_metrics'] = {'reason': f'no tf to {self.map_frame}'}

        out['label'] = combine(ring, out['map'], self.map_mode)
        if out['position'] is not None:
            near_qr = loc['room'] is not None and self.qr.near(loc['room'])
            track, out['report'] = self.tracker.update(
                out['position'], out['label'], time.time(),
                need=self.vote_need_qr if near_qr else None)
            if loc['room'] is not None:
                track.add_room(loc['room'])
            out['track'] = track.id
            out['track_obj'] = track
            out['votes'] = self.tracker.votes(track)
            out['confirmed_poster'] = self.tracker.is_poster(track)
        return out

    def locate(self, depth, box, header, align):
        """
        {'poster', 'room', 'poster_map'} for one person box, or None.

        'poster': torso point in track_frame (odom). 'room': the point pulled back from the
        wall towards the camera (poster_utils.pull_back), also in track_frame: it decides the
        survivor's cell. 'poster_map': the torso point in map at the image time, for the
        static-map test only (None before SLAM is up). All lookups are non-blocking: a wait
        in this callback starves the node's /tf subscription (see qr_utils.transform_first).
        """
        tp = torso_pixel_and_depth(depth, box, self.ring_p)
        if tp is None:
            return None
        u, v, z = tp
        fx, fy, cx, cy = align.depth_k         # box is in depth-image pixels
        pt = PointStamped()
        pt.header = header
        pt.point.x, pt.point.y, pt.point.z = (u - cx) * z / fx, (v - cy) * z / fy, z
        poster, errors = transform_first(self.tf_buffer, pt, [self.track_frame])
        if poster is None:
            self.get_logger().warn(f'No tf to {self.track_frame} ({errors}); person not '
                                   'tracked', throttle_duration_sec=5.0)
            return None
        rp = PointStamped()
        rp.header = header
        rp.point.x, rp.point.y, rp.point.z = pull_back(
            (pt.point.x, pt.point.y, pt.point.z), self.pullback)
        room, _ = transform_first(self.tf_buffer, rp, [self.track_frame])
        pm, _ = transform_first(self.tf_buffer, pt, [self.map_frame])

        def xyz(p):
            return None if p is None else (p.point.x, p.point.y, p.point.z)
        return {'poster': xyz(poster), 'room': xyz(room), 'poster_map': xyz(pm)}

    def on_grid(self, msg):
        """maze_explorer refitted the grid to the walls: follow it (same cell size only).
        check_cells re-reports survivors whose cell this changes."""
        try:
            d = json.loads(msg.data)
            if abs(float(d['size']) - self.cells.size) > 1e-6:
                return
            self.cells.origin = (float(d['origin'][0]), float(d['origin'][1]))
            self.cells.yaw = float(d['yaw'])
        except (ValueError, KeyError, TypeError, IndexError):
            self.get_logger().warn(f'bad /maze/grid message: {msg.data[:80]}',
                                   throttle_duration_sec=30.0)

    def place(self, track):
        """(frame, cell, centre) of a track's survivor: the cell on the camera's side of
        the wall the poster hangs on (MazeCells.room_cell; place_point)."""
        poster = (track.x, track.y, track.z)
        return self.place_point(track.room if track.room is not None else poster,
                                poster=poster)

    def place_point(self, room, poster=None):
        """
        (frame, cell, centre) of a room point in odom: taken into map with the LATEST
        map <- odom (so SLAM corrections move it), or left in odom before SLAM is up (the
        robot holds still until /maze/start, so odom and map start out together).
        """
        frame, xy = self.track_frame, room[:2]
        pxy = None if poster is None else poster[:2]
        try:
            tr = self.tf_buffer.lookup_transform(self.map_frame, self.track_frame, Time())
            q, t = tr.transform.rotation, tr.transform.translation
            rot, off = _quat_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])
            frame, xy = self.map_frame, (rot @ np.asarray(room, float) + off)[:2]
            if poster is not None:
                pxy = (rot @ np.asarray(poster, float) + off)[:2]
        except Exception:
            pass
        cell = self.cells.cell(xy) if pxy is None else self.cells.room_cell(pxy, xy)
        return frame, cell, [round(float(v), 3) for v in self.cells.centre(cell)]

    def on_qr(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get('text') is None or not d.get('room_odom'):
            return                     # no room point (no depth / TF): nothing to place
        r = self.qr.add(d['text'], d['room_odom'], time.time())
        if r.n == 1:
            self.get_logger().info(f'QR room "{r.text}" at {self.track_frame} '
                                   f'{[round(v, 2) for v in r.room]}: a survivor is here')

    def check_qr_rooms(self):
        """
        Ask the explorer to look around a QR room with no poster confirmed yet, and report a
        survivor from the code alone once the robot has left that room (or after a timeout).
        """
        now = time.time()
        for r in self.qr.to_look(now, self.look_delay):
            r.look_requested = True
            pt = PointStamped()
            pt.header.frame_id = self.track_frame
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = (float(v) for v in r.room)
            self.look_pub.publish(pt)
            self.get_logger().info(f'QR room "{r.text}": no poster yet, asking to look around')
        robot = None
        try:
            t = self.tf_buffer.lookup_transform(self.track_frame, self.base_frame,
                                                Time()).transform.translation
            robot = (t.x, t.y, t.z)
        except Exception:
            pass
        for r in self.qr.to_report(robot, now, self.leave_radius, self.qr_timeout):
            r.backup = True
            self.report_qr(r, 'no poster seen in the QR room')

    def report_qr(self, r, why):
        """Report the survivor a QR room is known to hold, at its cell centre."""
        first = r.cell is None
        frame, cell, centre = self.place_point(r.room)
        r.cell, r.cell_frame = cell, frame
        if first:
            self.get_logger().info('Human found')
            self.found_pub.publish(String(data='Human found'))
        self.log_survivor({'stamp': None, 'id': 'qr', 'source': 'qr', 'qr_text': r.text,
                           'cell': list(cell), 'frame': frame, 'cell_centre': centre,
                           'poster_' + self.track_frame: [round(v, 3) for v in r.room],
                           'votes': None, 'conf': None, 'box': None,
                           'ring': f'QR "{r.text}": {why}', 'map': None})

    def check_cells(self):
        """
        Re-report a survivor when its placement changes: SLAM came up (odom -> map), or a
        SLAM correction (loop closure moves map->odom) put it in a different cell. No new
        "Human found". Tracks stay in odom, so nothing has to be moved or merged. This
        trusts Fast-LIO's odom to drift much less than SLAM corrects (short maze runs); each
        re-placement is logged so it can be checked afterwards.
        """
        self.check_qr_rooms()
        for r in self.qr.rooms.values():
            if r.backup and r.cell is not None:
                frame, cell, _ = self.place_point(r.room)
                if (frame, cell) != (r.cell_frame, r.cell):
                    self.report_qr(r, f'SLAM re-placed it (was {r.cell_frame} {r.cell})')
        for track in self.tracker.tracks:
            if track.cell is None:
                continue
            frame, cell, _ = self.place(track)
            if (frame, cell) == (track.cell_frame, track.cell):
                continue
            why = (f'SLAM is up (was {track.cell_frame})' if frame != track.cell_frame
                   else f'SLAM correction: was cell {track.cell}')
            self.report({'track': track.id, 'track_obj': track,
                         'position': [round(track.x, 3), round(track.y, 3), round(track.z, 3)],
                         'votes': self.tracker.votes(track), 'conf': None, 'box': None,
                         'ring_metrics': {'reason': why}, 'map_metrics': {}}, None)

    def depth_align(self, depth_shape):
        """Return the DepthAlign for this depth image, or None (looks up the sensor TF once)."""
        need = self.align_state.needs_tf()
        if need is not None:
            try:
                tr = self.tf_buffer.lookup_transform(need[0], need[1], Time()).transform
                self.align_state.t = (tr.translation.x, tr.translation.y, tr.translation.z)
            except Exception as e:
                self.get_logger().warn(f'No TF {need[1]} -> {need[0]} ({e}); mapping colour '
                                       'to depth without the sensor offset',
                                       throttle_duration_sec=10.0)
        align, why = self.align_state.get(depth_shape)
        if align is None:
            self.get_logger().warn(f'depth not used: {why}', throttle_duration_sec=5.0)
        return align

    # --- outputs -----------------------------------------------------------------
    def report(self, r, stamp):
        # Announce each poster once, checked by odom position (not track id: a track out of
        # view for forget_s is dropped and comes back with a new id).
        track = r['track_obj']
        why = r['ring_metrics'].get('reason')
        if self.announced.add(r['position']):
            room = track.room if track.room is not None else r['position']
            claim = self.qr.poster_confirmed(room)
            if claim is not None:      # the survivor already reported from this QR room
                why = f'is QR "{claim.text}" survivor; {why}'
                self.get_logger().info(f'Poster #{r["track"]} is the survivor already '
                                       f'reported for QR room "{claim.text}"')
            else:
                self.get_logger().info('Human found')
                self.found_pub.publish(String(data='Human found'))
        # The survivor's location: centre of the cell its room point is in.
        frame, cell, centre = self.place(track)
        track.cell, track.cell_frame = cell, frame
        self.log_survivor({'stamp': stamp, 'id': r['track'], 'source': 'poster',
                           'cell': list(cell), 'frame': frame, 'cell_centre': centre,
                           'poster_' + self.track_frame: r['position'],
                           'votes': r['votes'], 'conf': r['conf'], 'box': r['box'],
                           'ring': why, 'map': r['map_metrics'].get('reason')})

    def log_survivor(self, msg):
        """Publish one survivor report on /posters/detections and append it to the CSV."""
        self.pub.publish(String(data=json.dumps(msg)))
        os.makedirs(self.log_dir, exist_ok=True)
        new_file = not os.path.exists(self.csv_path)
        tf = self.track_frame
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(['wall_time', 'poster_id', 'frame', 'cell_i', 'cell_j', 'centre_x',
                            'centre_y', f'{tf}_x', f'{tf}_y', f'{tf}_z', 'votes', 'conf',
                            'ring', 'map'])
            w.writerow([time.time(), msg['id'], msg['frame'], *msg['cell'], *msg['cell_centre'],
                        *msg['poster_' + tf], msg['votes'], msg['conf'], msg['ring'],
                        msg['map']])
        c = msg['cell_centre']
        self.get_logger().info(
            f'SURVIVOR #{msg["id"]} in cell {tuple(msg["cell"])}, centre ({c[0]:.2f}, '
            f'{c[1]:.2f}) {msg["frame"]} ({msg["ring"]}, votes {msg["votes"]})')


def main():
    rclpy.init()
    node = PosterDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
