"""Render a maze_nav rosbag to a side-by-side video: camera | top-down SLAM map.

    python3 ~/challenge_ws/runs/ROS_bags/bag_video.py <bag_dir> <out.webm> [--fps 15]

Write .webm (VP8): it plays in VS Code, browsers and VLC. This OpenCV build has no H.264
encoder, so a .mp4 comes out as MPEG-4 Part 2, which only players like VLC can open.

Needs /camera/color/image_raw/compressed plus /map and /tf: rec_full or rec_nav bags
(rec_video has no map, so the right panel stays empty).

Right panel: /map, robot footprint (TF map->base_link), driven track, latest /plan,
explorer state from /rosout, commanded speed from /cmd_vel, bag time.
"""
import argparse
import bisect
import math
import os
import re

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

FOOT = np.array([[0.27, 0.23], [0.27, -0.23], [-0.37, -0.23], [-0.37, 0.23]])
H = 720                       # output height; each panel is H x H
STATE = re.compile(r'\[(STARTING|PAUSED|CENTERING|CREEPING|SCANNING|SPINNING|EXPLORING|'
                   r'EXITED|NO_EXIT|STOPPED)\]')


def reader(bag, topics):
    # bags recorded with zstd compression (rec_manual, rec_explore) need the other reader
    meta = os.path.join(bag, 'metadata.yaml')
    packed = os.path.isfile(meta) and re.search(r'compression_format:\s*\S*zstd', open(meta).read())
    r = rosbag2_py.SequentialCompressionReader() if packed else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    while r.has_next():
        topic, data, t = r.read_next()
        yield topic, deserialize_message(data, get_message(types[topic])), t * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """2D pose a * b, poses as (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


class Series:
    """Time-sorted values; .at(t) = latest value at or before t."""

    def __init__(self):
        self.t, self.v = [], []

    def add(self, t, v):
        self.t.append(t)
        self.v.append(v)

    def at(self, t, default=None):
        i = bisect.bisect_right(self.t, t) - 1
        return self.v[i] if i >= 0 else default


def load(bag):
    maps, plans, states, cmds = Series(), Series(), Series(), Series()
    map_odom, odom_base = Series(), Series()
    logs = Series()
    for topic, m, t in reader(bag, ['/map', '/plan', '/rosout', '/cmd_vel', '/tf']):
        if topic == '/map':
            g = np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width)
            maps.add(t, (g, m.info.resolution, m.info.origin.position.x,
                         m.info.origin.position.y))
        elif topic == '/plan':
            plans.add(t, np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses]))
        elif topic == '/cmd_vel':
            cmds.add(t, (m.linear.x, m.linear.y, m.angular.z))
        elif topic == '/rosout':
            if m.name == 'maze_explorer':
                s = STATE.search(m.msg)
                if s:
                    states.add(t, s.group(1))
                logs.add(t, m.msg)
        elif topic == '/tf':
            for tr in m.transforms:
                p = (tr.transform.translation.x, tr.transform.translation.y,
                     yaw_of(tr.transform.rotation))
                key = (tr.header.frame_id, tr.child_frame_id)
                if key == ('map', 'odom'):
                    map_odom.add(t, p)
                elif key == ('odom', 'base_link'):
                    odom_base.add(t, p)
    return maps, plans, states, cmds, map_odom, odom_base, logs


def map_panel(t, maps, plans, states, cmds, map_odom, odom_base, logs, track, view):
    img = np.full((H, H, 3), 60, np.uint8)
    mp = maps.at(t)
    ob = odom_base.at(t)
    pose = compose(map_odom.at(t, (0.0, 0.0, 0.0)), ob) if ob else None
    x0, y0, x1, y1 = view
    scale = H / max(x1 - x0, y1 - y0)

    def px(xy):
        xy = np.atleast_2d(xy)
        return np.c_[(xy[:, 0] - x0) * scale, H - (xy[:, 1] - y0) * scale].astype(np.int32)

    if mp is not None:
        g, res, ox, oy = mp
        rgb = np.full(g.shape + (3,), 128, np.uint8)            # unknown grey
        rgb[(g >= 0) & (g < 50)] = 235                           # free
        rgb[g >= 50] = 20                                        # wall
        rgb = rgb[::-1]                                          # row 0 = top
        # place the map image into the view
        w, h = g.shape[1] * res * scale, g.shape[0] * res * scale
        tl = px((ox, oy + g.shape[0] * res))[0]
        sm = cv2.resize(rgb, (max(1, int(round(w))), max(1, int(round(h)))),
                        interpolation=cv2.INTER_NEAREST)
        xa, ya = max(tl[0], 0), max(tl[1], 0)
        xb, yb = min(tl[0] + sm.shape[1], H), min(tl[1] + sm.shape[0], H)
        if xb > xa and yb > ya:
            img[ya:yb, xa:xb] = sm[ya - tl[1]:yb - tl[1], xa - tl[0]:xb - tl[0]]
    # 1 m grid
    for gx in range(math.floor(x0), math.ceil(x1) + 1):
        cv2.line(img, tuple(px((gx, y0))[0]), tuple(px((gx, y1))[0]), (90, 90, 140), 1)
    for gy in range(math.floor(y0), math.ceil(y1) + 1):
        cv2.line(img, tuple(px((x0, gy))[0]), tuple(px((x1, gy))[0]), (90, 90, 140), 1)
    plan = plans.at(t)
    if plan is not None and len(plan) > 1:
        cv2.polylines(img, [px(plan)], False, (0, 200, 0), 2, cv2.LINE_AA)
    if pose:
        track.append(pose[:2])
    if len(track) > 1:
        cv2.polylines(img, [px(np.array(track))], False, (255, 140, 0), 2, cv2.LINE_AA)
    if pose:
        c, s = math.cos(pose[2]), math.sin(pose[2])
        fp = np.c_[pose[0] + c * FOOT[:, 0] - s * FOOT[:, 1],
                   pose[1] + s * FOOT[:, 0] + c * FOOT[:, 1]]
        cv2.polylines(img, [px(fp)], True, (0, 220, 255), 2, cv2.LINE_AA)
        tip = px((pose[0] + 0.35 * c, pose[1] + 0.35 * s))[0]
        cv2.arrowedLine(img, tuple(px(pose[:2])[0]), tuple(tip), (0, 220, 255), 2,
                        cv2.LINE_AA, tipLength=0.3)
    # text
    st = states.at(t, '-')
    cmd = cmds.at(t, (0.0, 0.0, 0.0))
    lines = [f'state: {st}',
             f'cmd: vx {cmd[0]:+.2f}  vy {cmd[1]:+.2f}  w {cmd[2]:+.2f}']
    log = logs.at(t)
    if log:
        lines.append(log[:62])
    for i, (label, col) in enumerate((('footprint', (0, 220, 255)), ('driven', (255, 140, 0)),
                                       ('plan', (0, 200, 0)))):
        y = H - 14 - 22 * i
        cv2.line(img, (H - 150, y - 5), (H - 120, y - 5), col, 3)
        cv2.putText(img, label, (H - 112, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                    cv2.LINE_AA)
    cv2.putText(img, 'grid 1 m', (10, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                cv2.LINE_AA)
    for i, ln in enumerate(lines):
        y = 26 + 24 * i
        cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                    cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--fps', type=float, default=15.0)
    a = ap.parse_args()

    maps, plans, states, cmds, map_odom, odom_base, logs = load(a.bag)
    # view: every map's extent plus the track, padded, square
    # view: the robot's track, padded 2 m, at least 5 m across (the map's long lidar rays
    # would otherwise shrink the robot to a few pixels)
    xs, ys = [], []
    for t, ob in zip(odom_base.t, odom_base.v):
        p = compose(map_odom.at(t, (0.0, 0.0, 0.0)), ob)
        xs.append(p[0])
        ys.append(p[1])
    if not xs:
        xs, ys = [-3, 3], [-3, 3]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    half = max(max(xs) - min(xs), max(ys) - min(ys), 1.0) / 2 + 2.0
    view = (cx - half, cy - half, cx + half, cy + half)

    codec = 'VP80' if a.out.lower().endswith('.webm') else 'mp4v'
    if codec == 'mp4v':
        print('note: .mp4 = MPEG-4 Part 2 (no H.264 encoder here); VS Code and browsers '
              "can't play it, VLC can. Use .webm for those.")
    out = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*codec), a.fps, (2 * H, H))
    if not out.isOpened():
        raise SystemExit(f'cannot write {a.out} with codec {codec}')
    track, next_t, n, t_start = [], None, 0, None
    for _topic, m, t in reader(a.bag, ['/camera/color/image_raw/compressed']):
        if t_start is None:
            t_start = t
        if next_t is not None and t < next_t:
            continue
        next_t = (next_t or t) + 1.0 / a.fps
        cam = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        if cam is None:
            continue
        cam = cv2.resize(cam, (int(cam.shape[1] * H / cam.shape[0]), H))
        if cam.shape[1] > H:                                  # centre-crop to square
            o = (cam.shape[1] - H) // 2
            cam = cam[:, o:o + H]
        else:
            cam = cv2.copyMakeBorder(cam, 0, 0, 0, H - cam.shape[1], cv2.BORDER_CONSTANT)
        cv2.putText(cam, f't = {t - t_start:5.1f} s', (10, H - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2, cv2.LINE_AA)
        panel = map_panel(t, maps, plans, states, cmds, map_odom, odom_base, logs, track, view)
        out.write(np.hstack([cam, panel]))
        n += 1
    out.release()
    print(f'{n} frames at {a.fps:g} fps -> {a.out}; states seen: '
          f'{sorted(set(states.v))}; {len(maps.v)} maps, {len(plans.v)} plans')


if __name__ == '__main__':
    main()
