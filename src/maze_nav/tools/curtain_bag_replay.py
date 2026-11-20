#!/usr/bin/env python3
"""Replay a bag's depth (and lidar) frames through curtains.detect_curtains, offline.

    source /opt/ros/humble/setup.bash && source install/setup.bash
    nice -n 19 python3 src/maze_nav/tools/curtain_bag_replay.py ROS_bags/curtain_test1 \
        [--max-gap 0.15 0.25] [--every 5] [--lidar] [--csv out.csv]

Needs no /tf: every frame is taken on its own in base_link (camera pose from /tf_static's
base_link -> camera_link + the optical rotation, as curtain_detector does; lidar from
base_link -> the cloud's frame), so it works on teleop bags without Fast-LIO
(curtain_test1). The floor is measured per frame (lowest dense layer of points ahead).
Each frame prints, per max_gap: curtains found, else curtains.diagnose()'s near-miss line,
plus the range to the wall straight ahead (depth centre column).
"""
import argparse
import csv
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from maze_nav.core import cloud_xyz
from maze_nav.curtains import CurtainParams, depth_points, detect_curtains, diagnose
from maze_nav.stairs import camera_from_link


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class StaticTf:
    """Chains /tf_static transforms: pose(parent, child) -> (R, t)."""

    def __init__(self):
        self.edges = {}                        # child -> (parent, R, t)

    def add(self, msg):
        for tr in msg.transforms:
            t = tr.transform.translation
            self.edges[tr.child_frame_id] = (tr.header.frame_id, quat_matrix(tr.transform.rotation),
                                             np.array([t.x, t.y, t.z]))

    def pose(self, parent, child):
        r, t, f = np.eye(3), np.zeros(3), child
        while f != parent:
            if f not in self.edges:
                return None
            p, re, te = self.edges[f]
            r, t = re @ r, re @ t + te
            f = p
        return r, t


def floor_of(pts):
    """z of the floor in base_link: the lowest dense layer of points ahead (0.3-2.5 m)."""
    ahead = pts[(pts[:, 0] > 0.3) & (pts[:, 0] < 2.5) & (pts[:, 2] < -0.15)]
    if len(ahead) < 30:
        return None
    z = ahead[:, 2]
    lo = np.percentile(z, 5)
    layer = z[z < lo + 0.04]
    return float(np.median(layer))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--max-gap', type=float, nargs='+', default=[0.15, 0.25])
    ap.add_argument('--every', type=int, default=5, help='use every Nth depth frame')
    ap.add_argument('--lidar', action='store_true', help='add the newest lidar cloud')
    ap.add_argument('--depth-topic', default='/camera/depth/image_raw')
    ap.add_argument('--info-topic', default='/camera/depth/camera_info')
    ap.add_argument('--cloud-topic', default='/livox/points')
    ap.add_argument('--camera-link', default='camera_link')
    ap.add_argument('--csv')
    a = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {'/tf_static', a.depth_topic, a.info_topic} | ({a.cloud_topic} if a.lidar else set())
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}

    tf, k, cloud, n, t0 = StaticTf(), None, None, 0, None
    rows = []
    params = {g: CurtainParams(max_gap=g) for g in a.max_gap}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, cls[topic])
        t0 = stamp if t0 is None else t0
        if topic == '/tf_static':
            tf.add(msg)
            continue
        if topic == a.info_topic:
            k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            continue
        if topic == a.cloud_topic:
            cloud = msg
            continue
        n += 1
        if k is None or n % a.every:
            continue
        link = tf.pose('base_link', a.camera_link)
        if link is None:
            print(f'no /tf_static base_link -> {a.camera_link}', file=sys.stderr)
            return 1
        r, t = camera_from_link(*link)
        d = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, -1)[:, :msg.width]
        d = d.astype(np.float32) * 0.001
        pts = depth_points(d, k, r, t)
        origin = t
        if a.lidar and cloud is not None:
            lp = tf.pose('base_link', cloud.header.frame_id)
            if lp is not None:
                xyz = cloud_xyz(cloud.data, cloud.fields, cloud.point_step, cloud.row_step,
                                cloud.width, cloud.height, cloud.is_bigendian)
                xyz = xyz[np.isfinite(xyz).all(1)] @ lp[0].T + lp[1]
                pts = np.concatenate([pts, xyz])
                origin = lp[1]
        floor = floor_of(pts)
        col = d[:, d.shape[1] // 2]
        mid = col[(col > 0.15)]
        ahead = float(np.percentile(mid, 50)) if len(mid) else math.nan
        ts = (stamp - t0) * 1e-9
        out = []
        for g, p in params.items():
            if floor is None:
                out.append((g, 'no floor seen'))
                continue
            cells = []
            found = detect_curtains(pts, origin[:2], 0.0, floor, p, cells_out=cells)
            if found:
                c = max(found, key=lambda c: c.width)
                out.append((g, f'CURTAIN {c.width:.2f} m wide, gap {c.gap * 100:.0f} cm, '
                               f'{np.linalg.norm(c.centre):.2f} m'))
            else:
                out.append((g, diagnose(cells, p) or '-'))
        fl = 'nan' if floor is None else f'{floor:+.2f}'
        print(f'{ts:6.1f} s  ahead {ahead:4.2f} m  floor {fl}  ' +
              '  |  '.join(f'max_gap {g:.2f}: {m}' for g, m in out))
        rows.append([round(ts, 2), round(ahead, 3), floor] + [m for _, m in out])
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'ahead_m', 'floor_z'] + [f'max_gap_{g}' for g in a.max_gap])
            w.writerows(rows)
    return 0


if __name__ == '__main__':
    sys.exit(main())
