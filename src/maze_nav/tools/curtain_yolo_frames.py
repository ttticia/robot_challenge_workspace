#!/usr/bin/env python3
"""Bag -> YOLO dataset frames for a curtain detector, with draft labels from depth.

    source /opt/ros/humble/setup.bash && source install/setup.bash
    nice -n 19 python3 src/maze_nav/tools/curtain_yolo_frames.py ROS_bags/curtain_test1 \
        ~/challenge_ws/datasets/curtain [--every 3] [--max-gap 0.25] [--lidar]

For every Nth colour frame (/camera/color/image_raw/compressed) it writes
  <out>/images/<bag>_<t>.jpg            the frame
  <out>/labels/<bag>_<t>.txt            DRAFT YOLO label (class 0 = curtain), only when
                                        curtains.detect_curtains finds one in the depth
                                        frame nearest in time (+ the lidar with --lidar)
  <out>/manifest.csv                    t, range straight ahead (depth), draft yes/no and
                                        curtains.diagnose()'s reason when there is none
The draft box is the detected curtain segment (p1..p2, from the gap up to --top m above the
floor) projected into the colour image, so it covers the hanging fabric. Drafts are
automatic and WILL be wrong at times: check / fix every one in a labelling tool (e.g. CVAT,
Label Studio, labelImg in YOLO format) and label the frames without a draft by hand before
training. Frames are named by bag + time, so several bags go into one dataset.

No /tf needed (teleop bags without Fast-LIO): each frame is taken on its own in base_link,
camera pose from /tf_static base_link -> camera_link + the optical rotation (the colour and
depth sensors are taken as co-located; ~1-3 cm on a 640 px image is < 1 % of a box).
"""
import argparse
import csv
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from maze_nav.core import cloud_xyz
from maze_nav.curtains import CurtainParams, depth_points, detect_curtains, diagnose
from maze_nav.stairs import camera_from_link

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from curtain_bag_replay import StaticTf, floor_of  # noqa: E402


def project_box(c, floor, top, r, t, k, size, samples=20):
    """YOLO (cx, cy, w, h) normalised of curtain c (base_link) in the image, or None."""
    fx, fy, cx, cy = k
    w_img, h_img = size
    s = np.linspace(0.0, 1.0, samples)[:, None]
    xy = c.p1[None, :] * (1 - s) + c.p2[None, :] * s
    pts = []
    for z in (floor + c.gap, floor + top):
        pts.append(np.column_stack([xy, np.full(len(xy), z)]))
    p = np.concatenate(pts)
    opt = (p - t) @ r                   # base_link -> optical: R^T (p - t)
    opt = opt[opt[:, 2] > 0.1]
    if len(opt) < 4:
        return None
    u = fx * opt[:, 0] / opt[:, 2] + cx
    v = fy * opt[:, 1] / opt[:, 2] + cy
    x0, x1 = np.clip([u.min(), u.max()], 0, w_img - 1)
    y0, y1 = np.clip([v.min(), v.max()], 0, h_img - 1)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None
    return ((x0 + x1) / 2 / w_img, (y0 + y1) / 2 / h_img, (x1 - x0) / w_img,
            (y1 - y0) / h_img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--every', type=int, default=3, help='every Nth colour frame')
    ap.add_argument('--max-gap', type=float, default=0.25,
                    help='0.25 for the lab curtain (~20 cm up), 0.15 = competition spec')
    ap.add_argument('--top', type=float, default=1.6, help='m above the floor: box top')
    ap.add_argument('--max-dt', type=float, default=0.1, help='s colour <-> depth')
    ap.add_argument('--lidar', action='store_true')
    ap.add_argument('--image-topic', default='/camera/color/image_raw/compressed')
    ap.add_argument('--color-info-topic', default='/camera/color/camera_info')
    ap.add_argument('--depth-topic', default='/camera/depth/image_raw')
    ap.add_argument('--depth-info-topic', default='/camera/depth/camera_info')
    ap.add_argument('--cloud-topic', default='/livox/points')
    ap.add_argument('--camera-link', default='camera_link')
    a = ap.parse_args()

    name = os.path.basename(os.path.normpath(a.bag))
    os.makedirs(os.path.join(a.out, 'images'), exist_ok=True)
    os.makedirs(os.path.join(a.out, 'labels'), exist_ok=True)
    manifest = os.path.join(a.out, 'manifest.csv')
    new_manifest = not os.path.exists(manifest)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {'/tf_static', a.image_topic, a.color_info_topic, a.depth_topic,
            a.depth_info_topic} | ({a.cloud_topic} if a.lidar else set())
    missing = {a.image_topic, a.depth_topic} - set(types)
    if missing:
        print(f'bag has no {sorted(missing)}', file=sys.stderr)
        return 1
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}

    tf, kc, kd, size, depth, cloud = StaticTf(), None, None, None, None, None
    p = CurtainParams(max_gap=a.max_gap)
    n = n_draft = n_saved = 0
    t0 = None
    rows = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, cls[topic])
        t0 = stamp if t0 is None else t0
        if topic == '/tf_static':
            tf.add(msg)
        elif topic == a.color_info_topic:
            kc, size = (msg.k[0], msg.k[4], msg.k[2], msg.k[5]), (msg.width, msg.height)
        elif topic == a.depth_info_topic:
            kd = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        elif topic == a.depth_topic:
            depth = (stamp, msg)
        elif topic == a.cloud_topic:
            cloud = msg
        elif topic == a.image_topic:
            n += 1
            if n % a.every or kc is None:
                continue
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            if size is None or (img.shape[1], img.shape[0]) != tuple(size):
                size = (img.shape[1], img.shape[0])
            ts = (stamp - t0) * 1e-9
            stem = f'{name}_{ts:07.2f}'.replace('.', '_')
            cv2.imwrite(os.path.join(a.out, 'images', stem + '.jpg'), img)
            n_saved += 1
            row = {'image': stem + '.jpg', 't_s': round(ts, 2), 'ahead_m': '',
                   'draft': 'no', 'reason': ''}
            link = tf.pose('base_link', a.camera_link)
            if depth is None or kd is None or link is None or \
                    abs(depth[0] - stamp) * 1e-9 > a.max_dt:
                row['reason'] = 'no depth frame close in time / no camera TF'
                rows.append(row)
                continue
            r, t = camera_from_link(*link)
            dm = depth[1]
            d = np.frombuffer(bytes(dm.data), np.uint16).reshape(dm.height, -1)[:, :dm.width]
            d = d.astype(np.float32) * 0.001
            col = d[:, d.shape[1] // 2]
            col = col[col > 0.15]
            if len(col):
                row['ahead_m'] = round(float(np.median(col)), 3)
            pts, origin = depth_points(d, kd, r, t), t
            if a.lidar and cloud is not None:
                lp = tf.pose('base_link', cloud.header.frame_id)
                if lp is not None:
                    xyz = cloud_xyz(cloud.data, cloud.fields, cloud.point_step, cloud.row_step,
                                    cloud.width, cloud.height, cloud.is_bigendian)
                    pts = np.concatenate([pts, xyz @ lp[0].T + lp[1]])
                    origin = lp[1]
            floor = floor_of(pts)
            if floor is None:
                row['reason'] = 'no floor seen'
                rows.append(row)
                continue
            cells = []
            found = detect_curtains(pts, origin[:2], 0.0, floor, p, cells_out=cells)
            boxes = [b for b in (project_box(c, floor, a.top, r, t, kc, size) for c in found)
                     if b is not None]
            if boxes:
                with open(os.path.join(a.out, 'labels', stem + '.txt'), 'w') as f:
                    for b in boxes:
                        f.write('0 ' + ' '.join(f'{v:.5f}' for v in b) + '\n')
                row['draft'] = 'yes'
                n_draft += 1
            else:
                row['reason'] = diagnose(cells, p) or 'nothing curtain-like'
            rows.append(row)
    with open(manifest, 'a', newline='') as f:
        w = csv.DictWriter(f, ['image', 't_s', 'ahead_m', 'draft', 'reason'])
        if new_manifest:
            w.writeheader()
        w.writerows(rows)
    print(f'{n_saved} frames -> {a.out}/images, {n_draft} with a draft label; '
          f'manifest {manifest}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
