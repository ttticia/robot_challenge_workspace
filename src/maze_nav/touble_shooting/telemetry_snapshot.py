"""Telemetry snapshots for reviewing a run while it happens (or right after).

    ros2 run maze_nav telemetry_snapshot            # or maze_nav.launch.py troubleshoot:=true

Writes to <out_dir>/<run time>/ (and points <out_dir>/current at it):

  snap_NNNNN_<reason>.json   state: explorer / stairs / curtain status, pose, goal, cmd_vel,
                             wall_scan clearances, motion over the last stuck_time s, every
                             live param change so far, the last events
  snap_NNNNN_<reason>.png    top-down map crop around the robot (map frame, x right, y up,
                             1 m grid): SLAM map, wall_scan points (red), footprint (blue),
                             heading, goal (green), status banner
  snap_NNNNN_<reason>.jpg    camera frame (camera_mode event/always only)
  latest.json / latest.png / latest.jpg   copies of the newest ones
  events.log                 one line per event: `<time> #NNNNN <KIND> <detail>`

Snapshots are taken every `period` s and on events: a status change on /maze/status,
/stairs/status or /curtain/status, a dead end (/scene/trigger), a junction
(/scene/junction), a live param change (/troubleshoot/param_change), STUCK (active state
but moved < stuck_dist in stuck_time s), or a manual mark:

    ros2 topic pub --once /troubleshoot/mark std_msgs/msg/String "data: 'curtain missed?'"

Camera (camera_mode): every camera copy crosses the robot bridge (see maze_nav.launch.py
lidar_relay), so the default 'event' subscribes only for an event snapshot and drops the
subscription after the first frame or camera_wait s. 'always' keeps one subscription (the
periodic snapshots get a frame too), 'none' never subscribes.

Read-only: it publishes nothing the robot acts on.
"""
import collections
import json
import math
import os
import re
import shutil
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    import cv2
except ImportError:          # no images then; the JSON still works
    cv2 = None

from touble_shooting.live_params import CHANGE_TOPIC, attach

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
STATUS_TOPICS = {'explorer': '/maze/status', 'stairs': '/stairs/status',
                 'curtain': '/curtain/status'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def state_word(status):
    """'EXPLORING: goal 3 ...' -> 'EXPLORING'."""
    return (status or '').split(':', 1)[0].strip()


class TelemetrySnapshot(Node):

    def __init__(self):
        super().__init__('telemetry_snapshot')
        p = self.declare_parameter
        p('out_dir', os.path.expanduser('~/live_debug'))
        p('period', 5.0)               # s between periodic snapshots; 0 = events only
        p('min_gap', 1.0)              # s: events closer than this share one snapshot
        p('keep', 400)                 # snapshots kept on disk (oldest deleted)
        p('camera_topic', '/camera/color/image_raw/compressed')
        p('camera_mode', 'event')      # event / always / none ('off' reads as a YAML bool)
        p('camera_wait', 3.0)          # s to wait for a frame in event mode
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('window', 6.0)               # m: side of the map crop
        p('px_per_m', 80.0)
        p('scan_topic', '/wall_scan')
        p('cmd_topic', '/cmd_vel')
        p('goal_topic', '/maze/goal')
        # footprint drawn on the map (= explorer footprint_*)
        p('footprint', [0.37, 0.27, 0.23])   # back, front, half width (m)
        # STUCK: in one of these states but moved < stuck_dist m over stuck_time s
        # (explorer set_state names; LOOKING / FACING_QR / AT_* hold still on purpose)
        p('active_states', ['EXPLORING', 'TURNING', 'CREEPING', 'JUNCTION', 'DEAD_END'])
        p('stuck_time', 20.0)
        p('stuck_dist', 0.15)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in ('period', 'min_gap', 'keep', 'camera_wait', 'window',
                                      'px_per_m', 'stuck_time', 'stuck_dist')}
        self.map_frame, self.base_frame = g('map_frame'), g('base_frame')
        self.footprint = tuple(float(x) for x in g('footprint'))
        self.active_states = set(g('active_states'))
        self.camera_topic, self.camera_mode = g('camera_topic'), g('camera_mode')

        stamp = time.strftime('%Y%m%d_%H%M%S')
        root = g('out_dir')
        self.dir = os.path.join(root, stamp)
        os.makedirs(self.dir, exist_ok=True)
        cur = os.path.join(root, 'current')
        try:
            if os.path.islink(cur) or os.path.exists(cur):
                os.remove(cur)
            os.symlink(self.dir, cur)
        except OSError as e:
            self.get_logger().warning(f'no {cur} link: {e}')
        self.events_path = os.path.join(self.dir, 'events.log')

        self.status = {k: None for k in STATUS_TOPICS}       # text
        self.status_at = {k: None for k in STATUS_TOPICS}    # monotonic s
        self.map = None
        self.scan, self.scan_at = None, None
        self.cmd, self.cmd_at = None, None
        self.goal = None
        self.frame = None              # newest CompressedImage (always mode)
        self.poses = collections.deque()   # (monotonic, x, y, yaw)
        self.overrides = {}            # 'node/name' -> new value
        self.events = collections.deque(maxlen=20)
        self.files = collections.deque()   # snapshot stems, oldest first
        self.seq = 0
        self.pending = []              # event reasons waiting for min_gap
        self.last_snap = -1e9
        self.last_periodic = time.monotonic()
        self.stuck = False
        self.cam_sub, self.cam_waiting, self.cam_since = None, None, None

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        for key, topic in STATUS_TOPICS.items():
            self.create_subscription(String, topic,
                                     lambda m, k=key: self.on_status(k, m.data), LATCHED)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, g('cmd_topic'), self.on_cmd, 10)
        self.create_subscription(PoseStamped, g('goal_topic'),
                                 lambda m: setattr(self, 'goal', m), LATCHED)
        self.create_subscription(String, '/scene/trigger',
                                 lambda m: self.event('DEAD_END', m.data[:200]), 10)
        self.create_subscription(String, '/scene/junction',
                                 lambda m: self.event('JUNCTION', m.data[:200]), 10)
        self.create_subscription(String, CHANGE_TOPIC, self.on_param_change, 10)
        self.create_subscription(String, '/troubleshoot/mark',
                                 lambda m: self.event('MARK', m.data), 10)
        if self.camera_mode == 'always' and self.camera_topic:
            self.cam_sub = self.create_subscription(
                CompressedImage, self.camera_topic,
                lambda m: setattr(self, 'frame', m), qos_profile_sensor_data)
        self.create_timer(0.2, self.tick)
        attach(self, cfgs=[self.cfg])
        self.event('START', f'writing to {self.dir}; camera {self.camera_mode}')
        self.get_logger().info(f'snapshots -> {self.dir} (also {cur})')

    # ------------------------------------------------------------------ inputs
    def on_status(self, key, text):
        old = self.status[key]
        self.status[key], self.status_at[key] = text, time.monotonic()
        if state_word(old) != state_word(text):
            self.event('STATE', f'{key} {state_word(old) or "-"} -> {text[:200]}')

    def on_map(self, msg):
        self.map = msg

    def on_scan(self, msg):
        self.scan, self.scan_at = msg, time.monotonic()

    def on_cmd(self, msg):
        self.cmd, self.cmd_at = msg, time.monotonic()

    def on_param_change(self, msg):
        try:
            d = json.loads(msg.data)
            self.overrides[f'{d["node"]}/{d["name"]}'] = d['new']
            self.event('PARAM', f'{d["node"]} {d["name"]}: {d["old"]!r} -> {d["new"]!r}')
        except (ValueError, KeyError):
            self.event('PARAM', msg.data[:200])

    # ------------------------------------------------------------------ events
    def event(self, kind, detail=''):
        self.pending.append((kind, detail))

    def write_event_lines(self, items, seq):
        with open(self.events_path, 'a') as f:
            for kind, detail in items:
                line = f'{time.strftime("%H:%M:%S")} #{seq:05d} {kind} {detail}'
                self.events.append(line)
                f.write(line + '\n')

    def tick(self):
        now = time.monotonic()
        pose = self.pose()
        if pose:
            self.poses.append((now, *pose))
            while self.poses and now - self.poses[0][0] > self.cfg['stuck_time'] + 1:
                self.poses.popleft()
            self.check_stuck(now)
        if self.cam_waiting is not None and now - self.cam_since > self.cfg['camera_wait']:
            self.drop_camera()
        if self.pending and now - self.last_snap >= self.cfg['min_gap']:
            items, self.pending = self.pending, []
            self.snapshot(items)
        elif self.cfg['period'] > 0 and now - self.last_periodic >= self.cfg['period'] \
                and now - self.last_snap >= self.cfg['min_gap']:
            self.snapshot([('PERIODIC', '')], periodic=True)

    def moved(self, now):
        """(m moved over stuck_time, covers the full window)."""
        old = [p for p in self.poses if now - p[0] >= self.cfg['stuck_time']]
        if not old or not self.poses:
            return None, False
        a, b = old[-1], self.poses[-1]
        return math.hypot(b[1] - a[1], b[2] - a[2]), True

    def check_stuck(self, now):
        dist, full = self.moved(now)
        active = state_word(self.status['explorer']) in self.active_states
        if not full or dist is None:
            return
        if active and dist < self.cfg['stuck_dist'] and not self.stuck:
            self.stuck = True
            self.event('STUCK', f'{dist:.2f} m in {self.cfg["stuck_time"]:.0f} s while '
                                f'{state_word(self.status["explorer"])}')
        elif self.stuck and (dist > 2 * self.cfg['stuck_dist'] or not active):
            self.stuck = False
            self.event('UNSTUCK', f'{dist:.2f} m')

    # ------------------------------------------------------------------ state
    def pose(self):
        for frame in (self.map_frame, 'odom'):
            try:
                t = self.tf.lookup_transform(frame, self.base_frame, Time())
            except Exception:   # noqa: BLE001 - TF not up yet
                continue
            tr = t.transform.translation
            return tr.x, tr.y, yaw_of(t.transform.rotation)
        return None

    def scan_summary(self):
        s = self.scan
        if s is None:
            return None
        r = np.asarray(s.ranges, float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        a = (a + math.pi) % (2 * math.pi) - math.pi
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)

        def sector(lo, hi):
            m = ok & (a >= math.radians(lo)) & (a < math.radians(hi))
            return round(float(r[m].min()), 3) if m.any() else None
        return {'age_s': round(time.monotonic() - self.scan_at, 2),
                'front': sector(-20, 20), 'left': sector(60, 120),
                'right': sector(-120, -60),
                'back': (lambda x, y: min(v for v in (x, y) if v is not None)
                         if (x or y) else None)(sector(160, 181), sector(-181, -160)),
                'nearest': round(float(r[ok].min()), 3) if ok.any() else None}

    def state(self, seq, items):
        now = time.monotonic()
        pose = self.pose()
        dist, full = self.moved(now)
        d = {'seq': seq, 'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'wall_time': time.time(),
             'reasons': [f'{k} {v}'.strip() for k, v in items],
             'status': {k: {'text': v, 'age_s': None if self.status_at[k] is None
                            else round(now - self.status_at[k], 1)}
                        for k, v in self.status.items()},
             'pose_map': None if pose is None else
             {'x': round(pose[0], 3), 'y': round(pose[1], 3),
              'yaw_deg': round(math.degrees(pose[2]), 1)},
             'moved_m': None if dist is None else round(dist, 3),
             'moved_window_s': self.cfg['stuck_time'] if full else None,
             'stuck': self.stuck,
             'scan': self.scan_summary(),
             'cmd_vel': None if self.cmd is None else
             {'vx': round(self.cmd.linear.x, 3), 'vy': round(self.cmd.linear.y, 3),
              'wz': round(self.cmd.angular.z, 3), 'age_s': round(now - self.cmd_at, 2)},
             'goal': None, 'param_overrides': self.overrides,
             'recent_events': list(self.events)}
        if self.goal is not None:
            gp = self.goal.pose.position
            d['goal'] = {'x': round(gp.x, 3), 'y': round(gp.y, 3),
                         'yaw_deg': round(math.degrees(yaw_of(self.goal.pose.orientation)), 1)}
            if pose:
                d['goal']['dist_m'] = round(math.hypot(gp.x - pose[0], gp.y - pose[1]), 3)
        return d, pose

    # ------------------------------------------------------------------ output
    def snapshot(self, items, periodic=False):
        now = time.monotonic()
        self.last_snap = now
        if periodic:
            self.last_periodic = now
        self.seq += 1
        seq = self.seq
        tag = re.sub(r'[^A-Za-z0-9]+', '-', '+'.join(sorted({k for k, _ in items}))).lower()
        stem = os.path.join(self.dir, f'snap_{seq:05d}_{tag[:40]}')
        if not periodic:
            self.write_event_lines(items, seq)
        d, pose = self.state(seq, items)
        self.write_json(stem + '.json', d)
        self.copy_latest(stem + '.json', 'latest.json')
        if cv2 is not None:
            img = self.render(pose, d)
            if img is not None:
                cv2.imwrite(stem + '.png', img)
                self.copy_latest(stem + '.png', 'latest.png')
        if self.camera_mode == 'always' and self.frame is not None:
            self.save_frame(self.frame, stem)
        elif self.camera_mode == 'event' and not periodic and self.camera_topic:
            self.want_camera(stem)
        self.files.append(stem)
        while len(self.files) > int(self.cfg['keep']):
            old = self.files.popleft()
            for ext in ('.json', '.png', '.jpg'):
                try:
                    os.remove(old + ext)
                except OSError:
                    pass

    def write_json(self, path, d):
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(d, f, indent=1, default=str)
        os.replace(tmp, path)

    def copy_latest(self, src, name):
        dst = os.path.join(self.dir, name)
        shutil.copyfile(src, dst + '.tmp')
        os.replace(dst + '.tmp', dst)

    # camera ------------------------------------------------------------
    def want_camera(self, stem):
        self.cam_waiting, self.cam_since = stem, time.monotonic()
        if self.cam_sub is None:
            self.cam_sub = self.create_subscription(CompressedImage, self.camera_topic,
                                                    self.on_frame, qos_profile_sensor_data)

    def on_frame(self, msg):
        if self.cam_waiting is not None:
            self.save_frame(msg, self.cam_waiting)
        self.drop_camera()

    def drop_camera(self):
        self.cam_waiting = None
        if self.cam_sub is not None and self.camera_mode == 'event':
            self.destroy_subscription(self.cam_sub)
            self.cam_sub = None

    def save_frame(self, msg, stem):
        data = bytes(msg.data)
        if 'jpeg' in msg.format or 'jpg' in msg.format:
            with open(stem + '.jpg', 'wb') as f:
                f.write(data)
        elif cv2 is not None:
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return
            cv2.imwrite(stem + '.jpg', img)
        else:
            return
        self.copy_latest(stem + '.jpg', 'latest.jpg')

    # map picture ---------------------------------------------------------
    def render(self, pose, d):
        size = int(self.cfg['window'] * self.cfg['px_per_m'])
        ppm = self.cfg['px_per_m']
        img = np.full((size, size, 3), 128, np.uint8)
        cx, cy = (pose[0], pose[1]) if pose else (0.0, 0.0)

        def px(x, y):    # map frame -> image (x right, y up)
            return (int(round((x - cx) * ppm + size / 2)),
                    int(round(size / 2 - (y - cy) * ppm)))

        m = self.map
        if m is not None and m.info.width and m.info.height:
            res, w, h = m.info.resolution, m.info.width, m.info.height
            ox, oy = m.info.origin.position.x, m.info.origin.position.y
            grid = np.asarray(m.data, np.int16).reshape(h, w)
            # image pixel centres -> map cells (map origin yaw assumed 0, as slam_toolbox)
            u = np.arange(size)
            xs = cx + (u - size / 2 + 0.5) / ppm
            ys = cy - (u - size / 2 + 0.5) / ppm
            ci = np.floor((xs - ox) / res).astype(int)
            ri = np.floor((ys - oy) / res).astype(int)
            okc, okr = (ci >= 0) & (ci < w), (ri >= 0) & (ri < h)
            sub = np.full((size, size), -1, np.int16)
            sub[np.ix_(okr, okc)] = grid[np.ix_(ri[okr], ci[okc])]
            gray = np.where(sub < 0, 128, np.where(sub >= 65, 0,
                                                   np.where(sub <= 25, 255, 190)))
            img[:] = gray.astype(np.uint8)[:, :, None]
        # 1 m grid
        for k in range(int(math.floor(cx - self.cfg['window'])),
                       int(math.ceil(cx + self.cfg['window'])) + 1):
            x0, _ = px(k, 0)
            cv2.line(img, (x0, 0), (x0, size), (210, 200, 170), 1)
        for k in range(int(math.floor(cy - self.cfg['window'])),
                       int(math.ceil(cy + self.cfg['window'])) + 1):
            _, y0 = px(0, k)
            cv2.line(img, (0, y0), (size, y0), (210, 200, 170), 1)
        if pose:
            x, y, th = pose
            c, s = math.cos(th), math.sin(th)
            # scan (base_link frame) at the current pose
            if self.scan is not None:
                sc = self.scan
                r = np.asarray(sc.ranges, float)
                a = sc.angle_min + np.arange(len(r)) * sc.angle_increment
                ok = np.isfinite(r) & (r >= sc.range_min) & (r <= sc.range_max)
                bx, by = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
                for X, Y in zip(x + c * bx - s * by, y + s * bx + c * by):
                    cv2.circle(img, px(X, Y), 1, (0, 0, 230), -1)
            back, front, hw = self.footprint
            box = [(-back, -hw), (front, -hw), (front, hw), (-back, hw)]
            pts = np.array([px(x + c * bx - s * by, y + s * bx + c * by) for bx, by in box],
                           np.int32)
            cv2.polylines(img, [pts], True, (200, 80, 0), 2)
            cv2.arrowedLine(img, px(x, y), px(x + 0.45 * c, y + 0.45 * s), (200, 80, 0), 2,
                            tipLength=0.3)
        if self.goal is not None:
            gp = self.goal.pose.position
            gy = yaw_of(self.goal.pose.orientation)
            cv2.circle(img, px(gp.x, gp.y), 6, (0, 170, 0), 2)
            cv2.arrowedLine(img, px(gp.x, gp.y),
                            px(gp.x + 0.3 * math.cos(gy), gp.y + 0.3 * math.sin(gy)),
                            (0, 170, 0), 2, tipLength=0.3)
        # banner
        lines = [f'#{d["seq"]} {d["time"][11:]}  ' + ', '.join(d['reasons'])[:60]]
        for k in ('explorer', 'stairs', 'curtain'):
            t = d['status'][k]['text']
            if t:
                lines.append(f'{k}: {t}'[:70])
        if d['cmd_vel']:
            cv = d['cmd_vel']
            lines.append(f'cmd vx {cv["vx"]:+.2f} wz {cv["wz"]:+.2f} ({cv["age_s"]:.1f}s ago)')
        if d['scan']:
            sc = d['scan']
            lines.append('clear F {} L {} R {} B {}'.format(
                *[('-' if sc[k] is None else f'{sc[k]:.2f}') for k in
                  ('front', 'left', 'right', 'back')]))
        cv2.rectangle(img, (0, 0), (size, 16 * len(lines) + 6), (40, 40, 40), -1)
        for i, t in enumerate(lines):
            cv2.putText(img, t, (5, 16 * i + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, '1 m grid, map frame (x right, y up)', (5, size - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (60, 60, 60), 1, cv2.LINE_AA)
        return img


def main():
    rclpy.init()
    node = TelemetrySnapshot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
