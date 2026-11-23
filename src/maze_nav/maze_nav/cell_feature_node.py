"""cell_features: toy-set rooms, raised (stair) cells and tall edges from the lidar.

What it looks for and why: maze_nav/cell_features.py. In short, points ABOVE the 0.53 m
walls (0.58-1.30 m) inside a cell are a toy set (a room); points in the wall band inside a
cell with railings along two opposite edges are the pyramid stairs; points 0.58-0.85 m along
an edge are curtain fabric.

  in : cloud_topic (/livox/points_crop: up to ~1.4 m above the floor, enough for the toy
       sets), /maze/floor_z (wall_scan), /maze/grid (maze_explorer's wall-fitted grid),
       /maze/status (accumulates only while the run is under way: not the operators
       standing around the start), TF map <- cloud frame / base_link (never blocking)
  out: /maze/cell_features (latched JSON, cell_features.features_to_dict, 2 Hz):
       maze_explorer queues a room look into each 'object' cell; stair_detector takes
       'stairs' / 'raised' cells as stair candidates (and as hits while checking them)

CPU: at most `max_rate` clouds a second are used (the classes only need a few views of
each cell), and a cloud only adds its points to a sparse 5 cm raster.

    ros2 run maze_nav cell_features
"""
import collections
import json
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from maze_nav.cell_features import CellFeatures, FeatureParams, features_to_dict
from maze_nav.core import cloud_xyz
from maze_nav.scene_cells import ACTIVE_STATES, CellGrid, run_active

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
RUN_STATES = ACTIVE_STATES + ('STAIRS', 'AT_STAIRS', 'CURTAIN', 'AT_CURTAIN', 'TO_EXIT')


def tf_matrix(t):
    q, tr = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return r, np.array([tr.x, tr.y, tr.z])


class CellFeatureNode(Node):
    def __init__(self):
        super().__init__('cell_features')
        p = self.declare_parameter
        p('cloud_topic', '/livox/points_crop')
        p('out_topic', '/maze/cell_features')
        p('map_frame', 'map')
        p('base_frame', 'base_link')
        p('cell_size', 1.02)
        p('status_topic', '/maze/status')     # '' = always accumulate
        p('max_rate', 2.0)                    # clouds a second used
        p('publish_period', 0.5)              # s
        p('max_cloud_age', 1.5)               # s waiting for TF before dropping a cloud
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame, self.base = g('map_frame'), g('base_frame')
        self.cells = CellGrid(float(g('cell_size')))
        self.features = CellFeatures(FeatureParams())
        self.floor = None
        self.status_topic = g('status_topic')
        self.active = not self.status_topic
        self.min_period = 1.0 / max(0.1, float(g('max_rate')))
        self.max_age = float(g('max_cloud_age'))
        self.last_used = 0.0
        self.used = 0
        self.kinds = {}                       # cell -> kind last logged
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(String, g('out_topic'), LATCHED)
        self.pending = collections.deque(maxlen=2)
        self.create_subscription(PointCloud2, g('cloud_topic'), self.on_cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, '/maze/floor_z', self.on_floor, LATCHED)
        self.create_subscription(String, '/maze/grid', self.on_grid, LATCHED)
        if self.status_topic:
            self.create_subscription(String, self.status_topic, self.on_status, LATCHED)
        self.create_timer(0.05, self.process_pending)
        self.create_timer(float(g('publish_period')), self.publish)
        self.get_logger().info(
            f'{g("cloud_topic")} -> {g("out_topic")}: toy sets (0.58-1.30 m inside a cell), '
            'stairs (wall band inside a cell, rails on two sides), tall edges (fabric); '
            + (f'only while {self.status_topic} says the run is under way'
               if self.status_topic else 'always'))

    def on_floor(self, msg):
        self.floor = float(msg.data)

    def on_grid(self, msg):
        try:
            d = json.loads(msg.data)
            if abs(float(d['size']) - self.cells.size) > 1e-6:
                return
            self.cells.origin = (float(d['origin'][0]), float(d['origin'][1]))
            self.cells.yaw = float(d['yaw'])
        except (ValueError, KeyError, TypeError, IndexError):
            self.get_logger().warn(f'bad /maze/grid message: {msg.data[:80]}')

    def on_status(self, msg):
        self.active = run_active(msg.data, RUN_STATES)

    def on_cloud(self, msg):
        now = time.monotonic()
        if not self.active or self.floor is None or now - self.last_used < self.min_period:
            return
        self.last_used = now
        self.pending.append(msg)

    def process_pending(self):
        while self.pending:
            msg = self.pending[0]
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            if age > self.max_age:
                self.pending.popleft()
                continue
            if not (self.tf.can_transform(self.map_frame, msg.header.frame_id, stamp) and
                    self.tf.can_transform(self.map_frame, self.base, stamp)):
                return                          # TF not there yet: next tick
            self.pending.popleft()
            try:
                to_map = self.tf.lookup_transform(self.map_frame, msg.header.frame_id, stamp)
                base = self.tf.lookup_transform(self.map_frame, self.base, stamp)
            except TransformException:
                continue
            xyz = cloud_xyz(msg.data, msg.fields, msg.point_step, msg.row_step, msg.width,
                            msg.height, msg.is_bigendian)
            if xyz.size == 0:
                continue
            r, t = tf_matrix(to_map)
            pts = xyz.astype(np.float64) @ r.T + t
            b = base.transform.translation
            self.features.add(pts, self.floor, (b.x, b.y), sensor_xy=t[:2])
            self.used += 1

    def publish(self):
        if self.floor is None:
            return
        self.features.forget_old()
        res = self.features.classify(self.cells)
        msg = features_to_dict(res, self.cells)
        self.pub.publish(String(data=json.dumps(msg)))
        kinds = {tuple(c['cell']): c['kind'] for c in msg['cells']}
        for cell, kind in kinds.items():
            if self.kinds.get(cell) != kind:
                e = res['cells'][cell]
                self.get_logger().info(
                    f'cell {list(cell)}: {kind}'
                    + (f' (axis {e["axis"]})' if e['axis'] else '')
                    + f' - {e["high"]:.3f} m2 above the walls, {e["raised"]:.3f} m2 in the '
                    'wall band inside it')
        for cell in set(self.kinds) - set(kinds):
            self.get_logger().info(f'cell {list(cell)}: no longer {self.kinds[cell]}')
        self.kinds = kinds


def main():
    rclpy.init()
    node = CellFeatureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
