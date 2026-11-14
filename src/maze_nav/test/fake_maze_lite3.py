"""Simulated Lite3 in the example maze (example maze.step) for testing maze_nav without the robot.

Publishes /livox/points (livox_frame, 15 deg pitch, Mid-360 vertical FOV -7..52 deg), /odom,
odom->base_link and base_link->livox_frame, and walks from /cmd_vel (RELIABLE +
TRANSIENT_LOCAL, like lite3_domain_bridge_4). The body follows the terrain height (ramps,
mats, stairs), so the floor-referenced wall band in wall_scan gets exercised.

The maze sits in a 10 x 8 m lab room. The robot starts in the closed start cell, facing its
only opening. The exit is the stair block on the far side; the run counts as out once the
robot is past the stairs (logged as "OUT OF THE MAZE").

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
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

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


class Maze:
    """Example maze geometry: points for the lidar and a height map for the feet."""

    def __init__(self, path):
        self.solids = json.load(open(path))['solids']

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
            elif k == 'stairs':                  # 3 steps up, 3 down, 0.15 m treads, 0.12 m rise
                c = (lo[0] + hi[0]) / 2
                n = 3 - min(2, int((abs(x - c) + 0.075) / 0.15)) if abs(x - c) < 0.375 else 0
                h = max(h, 0.12 * n)
        return h

    def points(self):
        pts = []
        x0, x1, y0, y1 = ROOM
        g = np.stack(np.meshgrid(np.arange(x0, x1, 0.08), np.arange(y0, y1, 0.08), [0.0],
                                 indexing='ij'), -1).reshape(-1, 3)
        pts.append(g)                                                     # lab floor
        for lo, hi in (((x0, y0, 0), (x1, y0, 1)), ((x0, y1, 0), (x1, y1, 1)),
                       ((x0, y0, 0), (x0, y1, 1)), ((x1, y0, 0), (x1, y1, 1))):
            pts.append(surface(lo, hi, 0.05, 'all'))                      # lab walls
        pts.append(surface((-5.5, 2.0, 0), (-4.8, 2.6, 0.75), 0.04))      # a table-ish box
        for s in self.solids:
            k, lo, hi = s['kind'], s['min'], s['max']
            if k in ('wall', 'rail'):
                pts.append(surface(lo, hi, 0.02, 'all'))
            elif k in ('tile', 'mat', 'block'):
                pts.append(surface(lo, hi, 0.04, 'top'))
            elif k == 'wedge':
                xs, ys = np.arange(lo[0], hi[0], 0.03), np.arange(lo[1], hi[1], 0.03)
                xx, yy = np.meshgrid(xs, ys)
                zz = np.vectorize(self.height)(xx, yy)
                pts.append(np.stack([xx.ravel(), yy.ravel(), zz.ravel()], 1))
            elif k == 'stairs':
                xs, zs = np.arange(lo[0], hi[0], 0.02), np.arange(0, hi[2], 0.03)
                ys = np.arange(lo[1], hi[1], 0.03)
                tread = [(x, y, self.height(x, y)) for x in xs for y in ys]
                rise = [(x, y, z) for x in xs for z in zs for y in ys[::3]
                        if abs(z - self.height(x, y)) < 0.015]
                pts.append(np.array(tread + rise))
        return np.concatenate(pts)


class FakeMazeLite3(Node):
    def __init__(self):
        super().__init__('fake_maze_lite3')
        self.declare_parameter('maze', os.path.join(HERE, 'example_maze.json'))
        self.declare_parameter('yaw_drift', 0.02)   # rad/s uncommanded turn while walking
        self.maze = Maze(self.get_parameter('maze').value)
        self.world = self.maze.points()
        self.drift = self.get_parameter('yaw_drift').value
        self.x, self.y, self.th = START
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
        self.static_tf.sendTransform(st)
        self.dt = 0.05
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.publish_cloud)
        self.create_timer(2.0, self.report)
        self.get_logger().info(f'{len(self.world)} world points; start {START}')

    def odom_pose(self):
        """Pose relative to the start pose (what Fast-LIO + odom_relay would report)."""
        dx, dy = self.x - START[0], self.y - START[1]
        c, s = math.cos(-START[2]), math.sin(-START[2])
        z = self.maze.height(self.x, self.y) + STAND - self.z0
        return c * dx - s * dy, s * dx + c * dy, z, self.th - START[2]

    def step(self):
        v, w = self.cmd.linear.x, self.cmd.angular.z
        if v != 0.0:
            w += self.drift
        self.th += w * self.dt
        self.x += v * math.cos(self.th) * self.dt
        self.y += v * math.sin(self.th) * self.dt
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
        if not self.out and self.x < EXIT_X:
            self.out = True
            self.get_logger().info(f'OUT OF THE MAZE at maze ({self.x:.2f}, {self.y:.2f})')

    def report(self):
        msg = (f'maze x={self.x:+.2f} y={self.y:+.2f} th={math.degrees(self.th):+.0f}deg '
               f'h={self.maze.height(self.x, self.y):.2f} out={self.out}')
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
        lid = lid[(rng > 0.1) & (rng < 40) & (elev > -7) & (elev < 52)].astype(np.float32)
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


def main():
    rclpy.init()
    try:
        rclpy.spin(FakeMazeLite3())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
