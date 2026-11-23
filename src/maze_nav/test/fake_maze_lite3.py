"""Simulated Lite3 in the example maze (example maze.step) for testing maze_nav without the robot.

Publishes /livox/points (livox_frame, 15 deg pitch, Mid-360 vertical FOV -7..52 deg), /odom,
odom->base_link and base_link->livox_frame, and walks from /cmd_vel (RELIABLE +
TRANSIENT_LOCAL, like lite3_domain_bridge_4). The body follows the terrain height (ramps,
mats, stairs), so the floor-referenced wall band in wall_scan gets exercised.

The maze sits in a 10 x 8 m lab room. The robot starts in the closed start cell, facing its
only opening. The exit is the stair block on the far side; the run counts as out once the
robot is past the stairs (logged as "OUT OF THE MAZE").

Other mazes (parameter maze:=<json>, same format as example_maze.json) may add optional keys:
"room": [x0, x1, y0, y1], "start": [x, y, yaw], "exit": {"axis": "x" or "y", "below" or
"above": value}, and "curtain" solids (min/max boxes, plus "travel": [nx, ny], the way the
robot walks through). A curtain is seen by the lidar but the body walks through it, and
while the lidar is 0..`drape_depth` m past it the curtain lies over the lidar (returns
within 0.2 m in front, none beyond). A maze with a curtain turns on lidar occlusion
(walls and curtains hide what is behind them; parameter occlusion), which the plain sim
leaves out. test/curtain_maze.json: a straight corridor with a curtain in it.
A wall solid with "style": "extru" is the aluminium-frame wall (resource/extru wall.stl,
test/extru_wall.py): its lidar points come from the frame and board, so the 10 x 37 cm side
windows at each end are see-through (only with occlusion on: the plain sim sees through every
wall), and "board_side" (-1/+1, default -1) puts the board on the min or max face. The body
still collides with the whole bounding box. A maze with an extru wall turns occlusion on too.
test/extru_maze.json: the curtain corridor built from extru walls.
A "steps" solid is a staircase with explicit size: its min/max box is the footprint, "down"
("+x", "-x", "+y" or "-y") the way it descends, "treads" the number of treads; tread i (from
the high end) is at max_z - i * rise, rise = (max_z - min_z) / treads, and the ground past
the last tread is at min_z. test/maze_curtains.json, test/maze_stairs.json: the two
presentation mazes (resource/Presentation, 2026-11-21; make_presentation_mazes.py).
A "pin" solid is a low bar the robot steps over with the H-Step gait (the ~18 cm rolling pin
across a cell, user 2026-11-22): the lidar and depth camera see it (and it hides what is
behind it), but the body neither rides up on it nor collides with it (H-Step clears it).

Always run it on a spare domain so it cannot meet the real robot:

    export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
    python3 test/fake_maze_lite3.py &
    ros2 launch maze_nav maze_nav.launch.py bringup:=false
"""
import array
import json
import math
import os

# Before numpy loads: OpenBLAS threads busy-wait after every matmul (see maze_nav/__init__.py).
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402
from scipy import ndimage  # noqa: E402
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from extru_wall import extru_boxes  # noqa: E402  (test/, next to this script)
from stair_scene import K as CAM_K, R_BASE_CAM, T_BASE_CAM  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
STAND = 0.35                       # base_link above the feet
LIDAR_T = np.array([0.18678, 0.0, 0.12498])
PITCH = math.radians(15.0)
R_LIDAR = np.array([[math.cos(PITCH), 0, math.sin(PITCH)],   # livox -> base (pitch down)
                    [0, 1, 0],
                    [-math.sin(PITCH), 0, math.cos(PITCH)]])
START = (0.0, 0.0, math.pi / 2)    # maze frame: start cell centre, facing +y (its opening)
ROOM = (-7.0, 3.0, -3.5, 4.5)      # lab walls x0, x1, y0, y1 (maze frame), 1 m tall
EXIT_X = -3.55                     # past the stair block
FOOTPRINT = (-0.37, 0.27, -0.23, 0.23)   # base_link x min/max, y min/max (nav2_params.yaml)
# /livox/points layout as published on the robot (checked live 2026-11-14)
LIVOX_FIELDS = [('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
                ('z', 8, PointField.FLOAT32), ('t', 12, PointField.UINT32),
                ('intensity', 16, PointField.FLOAT32), ('tag', 20, PointField.UINT8),
                ('line', 21, PointField.UINT8)]
LIVOX_DTYPE = np.dtype({'names': [f[0] for f in LIVOX_FIELDS],
                        'formats': ['<f4', '<f4', '<f4', '<u4', '<f4', 'u1', 'u1'],
                        'offsets': [f[1] for f in LIVOX_FIELDS], 'itemsize': 22})


def surface(lo, hi, step, faces='all'):
    """Points on the faces of an axis-aligned box."""
    def ax(a, b):
        return np.linspace(a, b, max(2, int(math.ceil((b - a) / step)) + 1))
    xs, ys, zs = ax(lo[0], hi[0]), ax(lo[1], hi[1]), ax(lo[2], hi[2])
    out = []
    g = lambda a, b, c: np.stack(np.meshgrid(a, b, c, indexing='ij'), -1).reshape(-1, 3)  # noqa
    if faces in ('all', 'top'):
        out.append(g(xs, ys, [hi[2]]))
    if faces == 'all':
        out += [g([lo[0], hi[0]], ys, zs), g(xs, [lo[1], hi[1]], zs)]
    return np.concatenate(out)


def steps_height(s, x, y):
    """Height of a "steps" solid at (x, y) inside its footprint (see the module doc)."""
    lo, hi = s['min'], s['max']
    n = int(s.get('treads', 2))
    axis = 0 if s['down'][1] == 'x' else 1
    length = hi[axis] - lo[axis]
    v = (x, y)[axis]
    t = (v - lo[axis]) / length if s['down'][0] == '+' else (hi[axis] - v) / length
    i = min(n - 1, max(0, int(t * n)))
    return hi[2] - i * (hi[2] - lo[2]) / n


def steps_risers(s):
    """[(z_low, z_high, edge coordinate)] of a "steps" solid's riser faces (down its axis),
    the last one down to min_z at the far end."""
    lo, hi = s['min'], s['max']
    n = int(s.get('treads', 2))
    axis = 0 if s['down'][1] == 'x' else 1
    rise = (hi[2] - lo[2]) / n
    length = hi[axis] - lo[axis]
    out = []
    for i in range(1, n + 1):
        f = i / n
        edge = lo[axis] + f * length if s['down'][0] == '+' else hi[axis] - f * length
        out.append((hi[2] - i * rise, hi[2] - (i - 1) * rise, edge))
    return out


BOX_EDGES = [(a, b) for a in range(8) for b in range(a + 1, 8) if bin(a ^ b).count('1') == 1]


def render_depth(boxes, origin, r_oc, rays, k, near=0.05, max_range=3.5):
    """Optical-axis depth (m, inf = no hit) per pixel: the floor plane z = 0 and boxes.

    rays: (H, W, 3) camera-frame rays with z = 1, so the hit parameter t is the depth.
    Each box is only tested against the pixel rectangle it can cover: its corners (and where
    its edges cross the near plane) projected into the image. Boxes further than max_range
    are skipped (the detectors use < 3 m)."""
    fx, fy, cx, cy = k
    h, w = rays.shape[:2]
    dirs = rays @ r_oc.T                                   # world direction per pixel
    with np.errstate(divide='ignore', invalid='ignore'):
        t = -origin[2] / dirs[..., 2]
        # one contiguous float32 plane per axis: the per-box slab test below is elementwise
        inv = [np.ascontiguousarray(1.0 / dirs[..., a], dtype=np.float32) for a in range(3)]
    depth = np.where(t > 0, t, np.inf).astype(np.float32)
    bits = np.array([[(c >> 2) & 1, (c >> 1) & 1, c & 1] for c in range(8)])
    for lo, hi in boxes:
        if np.linalg.norm(np.clip(origin, lo, hi) - origin) > max_range:
            continue
        cam = (np.where(bits, hi, lo) - origin) @ r_oc      # corners in the camera frame
        front = cam[:, 2] > near
        if not front.any():
            continue
        pts = [cam[front]]
        for a, b in BOX_EDGES:
            if front[a] != front[b]:
                f = (near - cam[a, 2]) / (cam[b, 2] - cam[a, 2])
                pts.append((cam[a] + f * (cam[b] - cam[a]))[None])
        p = np.concatenate(pts)
        u = fx * p[:, 0] / p[:, 2] + cx
        v = fy * p[:, 1] / p[:, 2] + cy
        u0, u1 = max(0, int(np.floor(u.min()))), min(w - 1, int(np.ceil(u.max())))
        v0, v1 = max(0, int(np.floor(v.min()))), min(h - 1, int(np.ceil(v.max())))
        if u0 > u1 or v0 > v1:
            continue
        tmin = tmax = None
        with np.errstate(invalid='ignore'):
            for a in range(3):
                i = inv[a][v0:v1 + 1, u0:u1 + 1]
                t1, t2 = (lo[a] - origin[a]) * i, (hi[a] - origin[a]) * i
                near_t, far_t = np.minimum(t1, t2), np.maximum(t1, t2)
                tmin = near_t if tmin is None else np.maximum(tmin, near_t)
                tmax = far_t if tmax is None else np.minimum(tmax, far_t)
        sub = depth[v0:v1 + 1, u0:u1 + 1]
        hit = (tmax >= tmin) & (tmin > 0) & (tmin < sub)
        sub[hit] = tmin[hit]
    return depth


class Maze:
    """Example maze geometry: points for the lidar and a height map for the feet."""

    def __init__(self, path):
        d = json.load(open(path))
        self.solids = d['solids']
        self.room = tuple(d.get('room', ROOM))
        self.start = tuple(d.get('start', START))
        self.exit = d.get('exit', {'axis': 'x', 'below': EXIT_X})
        self.curtains = [s for s in self.solids if s['kind'] == 'curtain']

    def is_out(self, x, y):
        v = x if self.exit['axis'] == 'x' else y
        return v < self.exit['below'] if 'below' in self.exit else v > self.exit['above']

    def wall_boxes(self):
        """(N, 4) x0, y0, x1, y1 of everything the body can hit (walls, stair rails)."""
        return np.array([[s['min'][0], s['min'][1], s['max'][0], s['max'][1]]
                         for s in self.solids if s['kind'] in ('wall', 'rail')])

    def height(self, x, y):
        h = 0.0                                  # lab floor
        for s in self.solids:
            lo, hi = s['min'], s['max']
            if not (lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1]):
                continue
            k = s['kind']
            if k in ('tile', 'mat', 'block'):
                h = max(h, hi[2])
            elif k == 'wedge':                   # linear from low edge to high edge
                hx, hy = s['high_xy']
                ux = 0 if abs(hx - (lo[0] + hi[0]) / 2) < 1e-3 else (1 if hx > lo[0] + 1e-3 else -1)
                uy = 0 if abs(hy - (lo[1] + hi[1]) / 2) < 1e-3 else (1 if hy > lo[1] + 1e-3 else -1)
                if ux:
                    t = (x - lo[0]) / (hi[0] - lo[0]) if ux > 0 else (hi[0] - x) / (hi[0] - lo[0])
                else:
                    t = (y - lo[1]) / (hi[1] - lo[1]) if uy > 0 else (hi[1] - y) / (hi[1] - lo[1])
                h = max(h, lo[2] + t * (hi[2] - lo[2]))
            elif k == 'steps':
                h = max(h, steps_height(s, x, y))
            elif k == 'stairs':                  # 3 steps up, 3 down, 0.15 m treads, 0.12 m rise
                c = (lo[0] + hi[0]) / 2
                n = 3 - min(2, int((abs(x - c) + 0.075) / 0.15)) if abs(x - c) < 0.375 else 0
                h = max(h, 0.12 * n)
        return h

    def depth_boxes(self):
        """[(lo, hi)] boxes the depth camera's rays hit (the lab floor is a plane at z 0).
        Stepped solids become boxes: one per tread for steps / stairs, four slices per wedge."""
        out = []
        x0, x1, y0, y1 = self.room
        out += [((x0 - 0.05, y0, 0), (x0, y1, 1)), ((x1, y0, 0), (x1 + 0.05, y1, 1)),
                ((x0, y0 - 0.05, 0), (x1, y0, 1)), ((x0, y1, 0), (x1, y1 + 0.05, 1))]
        for s in self.solids:
            k, lo, hi = s['kind'], s['min'], s['max']
            if k == 'wall' and s.get('style') == 'extru':
                out += [(tuple(b[:3]), tuple(b[3:])) for b in extru_boxes(lo, hi,
                                                                       s.get('board_side', -1))]
            elif k in ('wall', 'rail', 'curtain', 'tile', 'mat', 'block', 'pin'):
                out.append((tuple(lo), tuple(hi)))
            elif k == 'steps':
                axis = 0 if s['down'][1] == 'x' else 1
                n = int(s.get('treads', 2))
                length = hi[axis] - lo[axis]
                for i in range(n):
                    a, b = lo[axis] + i * length / n, lo[axis] + (i + 1) * length / n
                    top = steps_height(s, *(((a + b) / 2, lo[1]) if axis == 0 else (lo[0], (a + b) / 2)))
                    blo, bhi = list(lo), [hi[0], hi[1], top]
                    blo[axis], bhi[axis] = a, b
                    out.append((tuple(blo), tuple(bhi)))
            elif k == 'stairs':                  # see height(): 3 up, 3 down about the middle
                c = (lo[0] + hi[0]) / 2
                for half, top in ((0.375, 0.12), (0.225, 0.24), (0.075, 0.36)):
                    out.append(((c - half, lo[1], 0.0), (c + half, hi[1], top)))
            elif k == 'wedge':
                hx, hy = s['high_xy']
                along_x = abs(hx - (lo[0] + hi[0]) / 2) > 1e-3
                for i in range(4):
                    blo, bhi = list(lo), list(hi)
                    if along_x:
                        blo[0], bhi[0] = lo[0] + i * (hi[0] - lo[0]) / 4, lo[0] + (i + 1) * (hi[0] - lo[0]) / 4
                    else:
                        blo[1], bhi[1] = lo[1] + i * (hi[1] - lo[1]) / 4, lo[1] + (i + 1) * (hi[1] - lo[1]) / 4
                    bhi[2] = self.height((blo[0] + bhi[0]) / 2, (blo[1] + bhi[1]) / 2)
                    out.append((tuple(blo), tuple(bhi)))
        return out

    def points(self, dense=False):
        """(world points (N, 3), occluder mask): walls, rails, curtains, lab walls and
        furniture hide what is behind them when occlusion is on; floor and terrain don't.
        dense: 4 cm floor and 1 cm walls (for occlusion, and floor seen under curtains)."""
        fs, ws = (0.04, 0.01) if dense else (0.08, 0.02)
        pts, occ = [], []

        def add(a, o):
            pts.append(a)
            occ.append(np.full(len(a), o))
        x0, x1, y0, y1 = self.room
        g = np.stack(np.meshgrid(np.arange(x0, x1, fs), np.arange(y0, y1, fs), [0.0],
                                 indexing='ij'), -1).reshape(-1, 3)
        # Lab floor only where no terrain covers it: floor never occludes here, so the lab
        # floor under the 0.1 m tiles showed through, and wall_scan measured the floor at
        # odom -0.45 (lab floor) instead of -0.35 (the tiles the robot stands on) and guessed.
        covered = np.zeros(len(g), bool)
        for s in self.solids:
            if s['kind'] in ('tile', 'mat', 'block', 'wedge', 'stairs', 'steps'):
                lo, hi = s['min'], s['max']
                covered |= (g[:, 0] >= lo[0]) & (g[:, 0] <= hi[0]) & \
                    (g[:, 1] >= lo[1]) & (g[:, 1] <= hi[1])
        add(g[~covered], False)                                           # lab floor
        for lo, hi in (((x0, y0, 0), (x1, y0, 1)), ((x0, y1, 0), (x1, y1, 1)),
                       ((x0, y0, 0), (x0, y1, 1)), ((x1, y0, 0), (x1, y1, 1))):
            add(surface(lo, hi, 0.05, 'all'), True)                       # lab walls
        if self.room == ROOM:
            add(surface((-5.5, 2.0, 0), (-4.8, 2.6, 0.75), 0.04), True)   # a table-ish box
        for s in self.solids:
            k, lo, hi = s['kind'], s['min'], s['max']
            if k == 'wall' and s.get('style') == 'extru':
                for b in extru_boxes(lo, hi, s.get('board_side', -1)):
                    add(surface(b[:3], b[3:], ws, 'all'), True)
            elif k in ('wall', 'rail'):
                add(surface(lo, hi, ws, 'all'), True)
            elif k == 'curtain':
                add(surface(lo, hi, 0.01, 'all'), True)
            elif k in ('tile', 'mat', 'block'):
                add(surface(lo, hi, 0.04, 'top'), False)
            elif k == 'pin':                     # stepped over: seen (faces), never ridden on
                add(surface(lo, hi, ws, 'all'), True)
            elif k == 'wedge':
                xs, ys = np.arange(lo[0], hi[0], 0.03), np.arange(lo[1], hi[1], 0.03)
                xx, yy = np.meshgrid(xs, ys)
                zz = np.vectorize(self.height)(xx, yy)
                add(np.stack([xx.ravel(), yy.ravel(), zz.ravel()], 1), False)
            elif k == 'steps':                   # treads + riser faces
                xs, ys = np.arange(lo[0], hi[0], 0.03), np.arange(lo[1], hi[1], 0.03)
                xx, yy = np.meshgrid(xs, ys)
                zz = np.vectorize(lambda a, b: steps_height(s, a, b))(xx, yy)
                tread = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], 1)
                rise = []
                for r0, r1, edge in steps_risers(s):
                    for z in np.arange(r0, r1, 0.02):
                        if s['down'][1] == 'x':
                            rise += [(edge, y, z) for y in ys]
                        else:
                            rise += [(x, edge, z) for x in xs]
                add(np.concatenate([tread, np.array(rise).reshape(-1, 3)]), False)
            elif k == 'stairs':
                xs, zs = np.arange(lo[0], hi[0], 0.02), np.arange(0, hi[2], 0.03)
                ys = np.arange(lo[1], hi[1], 0.03)
                tread = [(x, y, self.height(x, y)) for x in xs for y in ys]
                rise = [(x, y, z) for x in xs for z in zs for y in ys[::3]
                        if abs(z - self.height(x, y)) < 0.015]
                add(np.array(tread + rise), False)
        return np.concatenate(pts), np.concatenate(occ)


class FakeMazeLite3(Node):
    def __init__(self):
        super().__init__('fake_maze_lite3')
        self.declare_parameter('maze', os.path.join(HERE, 'example_maze.json'))
        self.declare_parameter('yaw_drift', 0.02)   # rad/s uncommanded turn while walking
        self.declare_parameter('track_csv', '')     # write t, x, y, yaw, clearance per step
        self.declare_parameter('occlusion', 'auto')  # auto = on with a curtain or extru wall
        self.declare_parameter('drape_depth', 0.45)  # m past a curtain the lidar is covered
        # Depth camera (the robot's Orbbec mount and intrinsics, stair_scene.py): 16-bit mm on
        # /camera/depth/image_raw, ray-cast against depth_boxes(). 0 Hz = off.
        self.declare_parameter('depth_rate', 5.0)
        # px (640 x 480 scaled; height = 3/4). Keep 640: stairs.py's grid strides are in px.
        self.declare_parameter('depth_width', 640)
        self.maze = Maze(self.get_parameter('maze').value)
        occ = str(self.get_parameter('occlusion').value).lower()
        extru = any(s.get('style') == 'extru' for s in self.maze.solids)
        self.occlusion = (bool(self.maze.curtains) or extru) if occ == 'auto' \
            else occ in ('true', '1')
        self.world, self.occluder = self.maze.points(dense=self.occlusion)
        self.drape_depth = float(self.get_parameter('drape_depth').value)
        self.draped = False
        self.drift = self.get_parameter('yaw_drift').value
        self.x, self.y, self.th = self.maze.start
        self.z0 = self.maze.height(self.x, self.y) + STAND
        self.cmd = Twist()
        self.out = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '/livox/points', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.truth_pub = self.create_publisher(String, '/sim/truth', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id, st.child_frame_id = 'base_link', 'livox_frame'
        st.transform.translation.x, st.transform.translation.z = LIDAR_T[0], LIDAR_T[2]
        st.transform.rotation.y = math.sin(PITCH / 2)
        st.transform.rotation.w = math.cos(PITCH / 2)
        statics = [st]
        # Camera like the robot: base_link -> camera_link (20 deg down) -> optical frame.
        for parent, child, xyz, q in (
                ('base_link', 'camera_link', T_BASE_CAM, (0.0, 0.1736, 0.0, 0.9848)),
                ('camera_link', 'camera_depth_optical_frame', (0.0, 0.0, 0.0),
                 (-0.5, 0.5, -0.5, 0.5))):
            c = TransformStamped()
            c.header.stamp = st.header.stamp
            c.header.frame_id, c.child_frame_id = parent, child
            (c.transform.translation.x, c.transform.translation.y,
             c.transform.translation.z) = (float(v) for v in xyz)
            (c.transform.rotation.x, c.transform.rotation.y, c.transform.rotation.z,
             c.transform.rotation.w) = q
            statics.append(c)
        self.static_tf.sendTransform(statics)
        self.dt = 0.05
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.publish_cloud)
        rate = float(self.get_parameter('depth_rate').value)
        if rate > 0:
            self.setup_depth(int(self.get_parameter('depth_width').value))
            self.create_timer(1.0 / rate, self.publish_depth)
        self.create_timer(2.0, self.report)
        self.get_logger().info(f'{len(self.world)} world points; start {self.maze.start}'
                               + ('; lidar occlusion on' if self.occlusion else '')
                               + (f'; {len(self.maze.curtains)} curtain(s)'
                                  if self.maze.curtains else ''))
        # Wall clearance of the footprint (true pose), to tune how close Nav2 drives to walls.
        self.boxes = self.maze.wall_boxes()
        x0, x1, y0, y1 = FOOTPRINT
        n = 12
        edge = np.linspace(0, 1, n, endpoint=False)
        self.foot = np.concatenate([
            np.c_[x0 + (x1 - x0) * edge, np.full(n, y0)], np.c_[np.full(n, x1), y0 + (y1 - y0) * edge],
            np.c_[x1 - (x1 - x0) * edge, np.full(n, y1)], np.c_[np.full(n, x0), y1 - (y1 - y0) * edge]])
        self.clear = self.min_clear = float('inf')
        self.min_clear_at = None
        self.left_start = False
        path = self.get_parameter('track_csv').value
        self.track = open(path, 'w', buffering=1) if path else None   # line-buffered: survives SIGTERM
        if self.track:
            self.track.write('t,x,y,yaw,clearance\n')

    def clearance(self):
        """Smallest distance (m) from the footprint outline to a wall or rail; 0 = touching."""
        c, s = math.cos(self.th), math.sin(self.th)
        px = self.x + c * self.foot[:, 0] - s * self.foot[:, 1]
        py = self.y + s * self.foot[:, 0] + c * self.foot[:, 1]
        b = self.boxes
        dx = np.maximum(np.maximum(b[None, :, 0] - px[:, None], 0), px[:, None] - b[None, :, 2])
        dy = np.maximum(np.maximum(b[None, :, 1] - py[:, None], 0), py[:, None] - b[None, :, 3])
        return float(np.hypot(dx, dy).min())

    def odom_pose(self):
        """Pose relative to the start pose (what Fast-LIO + odom_relay would report)."""
        st = self.maze.start
        dx, dy = self.x - st[0], self.y - st[1]
        c, s = math.cos(-st[2]), math.sin(-st[2])
        z = self.maze.height(self.x, self.y) + STAND - self.z0
        return c * dx - s * dy, s * dx + c * dy, z, self.th - st[2]

    def step(self):
        v, vy, w = self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z
        if v != 0.0:
            w += self.drift
        self.th += w * self.dt
        self.x += (v * math.cos(self.th) - vy * math.sin(self.th)) * self.dt
        self.y += (v * math.sin(self.th) + vy * math.cos(self.th)) * self.dt
        ox, oy, oz, oth = self.odom_pose()
        stamp = self.get_clock().now().to_msg()
        q = (0.0, 0.0, math.sin(oth / 2), math.cos(oth / 2))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = ox, oy, oz
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        o.twist.twist = self.cmd
        self.odom_pub.publish(o)
        t = TransformStamped()
        t.header = o.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = ox, oy, oz
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf.sendTransform(t)
        self.clear = self.clearance()
        if self.track and (v or vy or w):
            self.track.write(f'{self.get_clock().now().nanoseconds * 1e-9:.2f},{self.x:.3f},'
                             f'{self.y:.3f},{self.th:.3f},{self.clear:.3f}\n')
        # The start cell is a dead end the robot only turns in: score the drive after it.
        self.left_start = self.left_start or math.hypot(self.x - self.maze.start[0],
                                                        self.y - self.maze.start[1]) > 0.6
        if self.left_start and self.clear < self.min_clear:
            self.min_clear, self.min_clear_at = self.clear, (self.x, self.y)
            if self.clear < 0.05:
                self.get_logger().warn(f'{self.clear * 100:.0f} cm from a wall at maze '
                                       f'({self.x:.2f}, {self.y:.2f})'
                                       + (' - TOUCHING' if self.clear <= 0 else ''))
        if not self.out and self.maze.is_out(self.x, self.y):
            self.out = True
            self.get_logger().info(f'OUT OF THE MAZE at maze ({self.x:.2f}, {self.y:.2f}); '
                                   + self.clear_summary())

    def clear_summary(self):
        if self.min_clear_at is None:
            return 'min wall clearance: not left the start cell yet'
        return (f'min wall clearance after the start cell {self.min_clear * 100:.1f} cm at maze '
                f'({self.min_clear_at[0]:.2f}, {self.min_clear_at[1]:.2f})')

    def report(self):
        msg = (f'maze x={self.x:+.2f} y={self.y:+.2f} th={math.degrees(self.th):+.0f}deg '
               f'h={self.maze.height(self.x, self.y):.2f} out={self.out} '
               f'clear={self.clear * 100:.0f}cm min={self.min_clear * 100:.0f}cm')
        self.truth_pub.publish(String(data=msg))
        self.get_logger().info(msg)

    def publish_cloud(self):
        stamp = self.get_clock().now().to_msg()        # acquisition time, like the Livox driver
        c, s = math.cos(self.th), math.sin(self.th)
        bz = self.maze.height(self.x, self.y) + STAND
        d = self.world - np.array([self.x, self.y, bz])
        base = np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]], 1)
        lid = (base - LIDAR_T) @ R_LIDAR
        rng = np.linalg.norm(lid, axis=1)
        elev = np.degrees(np.arcsin(lid[:, 2] / np.maximum(rng, 1e-9)))
        keep = (rng > 0.1) & (rng < 40) & (elev > -7) & (elev < 52)
        if self.occlusion:
            keep &= self.visible(lid, rng, elev)
        lid = lid[keep]
        draped = self.drape_check(c, s)
        if draped:   # fabric over the front of the lidar: near returns, nothing beyond
            lid = lid[lid[:, 0] < 0.0]
            rnd = np.random.default_rng()
            d = rnd.normal(size=(3000, 3))
            d[:, 0] = np.abs(d[:, 0])
            d /= np.linalg.norm(d, axis=1, keepdims=True)
            lid = np.concatenate([lid, d * rnd.uniform(0.11, 0.2, (3000, 1))])
        if draped != self.draped:
            self.draped = draped
            self.get_logger().info('curtain lies over the lidar' if draped else
                                   'lidar clear of the curtain')
        lid = lid.astype(np.float32)
        # Same mixed-type layout as the robot's /livox/points (point_step 22), so readers that
        # assume one field type fail here and not first on the robot.
        out = np.zeros(len(lid), LIVOX_DTYPE)
        out['x'], out['y'], out['z'] = lid[:, 0], lid[:, 1], lid[:, 2]
        out['intensity'] = 100.0
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'livox_frame'
        msg.height, msg.width = 1, len(out)
        msg.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in LIVOX_FIELDS]
        msg.point_step, msg.row_step = LIVOX_DTYPE.itemsize, LIVOX_DTYPE.itemsize * len(out)
        msg.is_dense = True
        # array.array, not bytes: rclpy (Humble) checks a bytes value element by element
        # (~65 ms for this cloud, most of the sim's CPU); an array('B') is taken as is (~0.3 ms).
        msg.data = array.array('B', out.tobytes())
        self.cloud_pub.publish(msg)


    def setup_depth(self, width):
        s = 640.0 / width
        self.depth_w, self.depth_h = width, int(round(480 / s))
        fx, fy, cx, cy = CAM_K[0] / s, CAM_K[1] / s, (CAM_K[2] + 0.5) / s - 0.5, \
            (CAM_K[3] + 0.5) / s - 0.5
        self.depth_k = (fx, fy, cx, cy)
        u, v = np.meshgrid(np.arange(self.depth_w), np.arange(self.depth_h))
        self.depth_rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1)
        self.depth_boxes = [(np.asarray(lo, float), np.asarray(hi, float))
                            for lo, hi in self.maze.depth_boxes()]
        info = CameraInfo()
        info.header.frame_id = 'camera_depth_optical_frame'
        info.width, info.height = self.depth_w, self.depth_h
        info.distortion_model, info.d = 'plumb_bob', [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.depth_info = info
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self.depth_rng = np.random.default_rng(1)
        self.get_logger().info(f'depth camera {self.depth_w}x{self.depth_h}, '
                               f'{len(self.depth_boxes)} boxes')

    def publish_depth(self):
        stamp = self.get_clock().now().to_msg()
        c, s = math.cos(self.th), math.sin(self.th)
        r_ob = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        origin = np.array([self.x, self.y, self.maze.height(self.x, self.y) + STAND]) \
            + r_ob @ T_BASE_CAM
        z = render_depth(self.depth_boxes, origin, r_ob @ R_BASE_CAM, self.depth_rays,
                         self.depth_k)
        ok = np.isfinite(z) & (z >= 0.1) & (z <= 10.0)
        z = np.where(ok, z + self.depth_rng.normal(0.0, 1.0, z.shape) * (0.001 + 0.005 * z * z),
                     0.0)
        img = Image()
        img.header.stamp, img.header.frame_id = stamp, 'camera_depth_optical_frame'
        img.height, img.width = self.depth_h, self.depth_w
        img.encoding, img.step = '16UC1', self.depth_w * 2
        img.data = array.array('B', np.round(z * 1000).astype('<u2').tobytes())
        self.depth_info.header.stamp = stamp
        self.depth_pub.publish(img)
        self.info_pub.publish(self.depth_info)

    def visible(self, lid, rng, elev, res=0.5):
        """Points not hidden behind an occluder (wall, rail, curtain) in 0.5 deg bins. The
        occluders' nearest range per bin is spread to the neighbouring bins (min filter),
        since their 1 cm point spacing leaves gaps of ~1 deg within 0.6 m."""
        az = np.degrees(np.arctan2(lid[:, 1], lid[:, 0]))
        ia = ((az + 180.0) / res).astype(int) % int(360 / res)
        ie = np.clip(((elev + 10.0) / res).astype(int), 0, int(65 / res) - 1)
        depth = np.full((int(360 / res), int(65 / res)), np.inf)
        o = self.occluder & (rng > 0.1)
        np.minimum.at(depth, (ia[o], ie[o]), rng[o])
        depth = ndimage.grey_erosion(depth, size=(3, 3), mode='wrap')
        return rng <= depth[ia, ie] + 0.04

    def drape_check(self, c, s):
        """True while the lidar is 0..drape_depth m past a curtain, inside its span."""
        lx = self.x + c * LIDAR_T[0]
        ly = self.y + s * LIDAR_T[0]
        for k in self.maze.curtains:
            lo, hi = k['min'], k['max']
            nx, ny = k.get('travel', (0.0, 1.0))
            cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
            past = (lx - cx) * nx + (ly - cy) * ny
            along = abs(-(lx - cx) * ny + (ly - cy) * nx)
            span = abs(-(hi[0] - lo[0]) * ny + (hi[1] - lo[1]) * nx) / 2
            if 0.0 <= past <= self.drape_depth and along <= span + 0.1:
                return True
        return False


def main():
    rclpy.init()
    try:
        rclpy.spin(FakeMazeLite3())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
