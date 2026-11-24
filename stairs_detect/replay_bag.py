#!/usr/bin/env python3
"""Replay a recorded stairs bag through stair_levels (offline, no ROS needed).

    pip install rosbags zstandard
    python3 replay_bag.py ../build/stairs_bags_real/stairs_walkin1 [--every 6]

Prints one line per processed frame (time, detection or why not, confirmed). Handles the
zstd file-compressed bags from ros2 bag record --compression-mode file.
"""
import argparse
import glob
import math
import os
import sqlite3
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stair_levels as S  # noqa: E402

CAM_T = (0.25228, -0.0005, 0.0741)
CAM_Q = (0.0, 0.17364817766693033, 0.0, 0.984807753012208)


def open_db(bagdir):
    db = glob.glob(os.path.join(bagdir, '*.db3'))
    if db:
        return db[0]
    z = glob.glob(os.path.join(bagdir, '*.db3.zstd'))[0]
    import zstandard
    out = os.path.join(tempfile.gettempdir(), os.path.basename(z)[:-5])
    if not os.path.exists(out):
        with open(z, 'rb') as fi, open(out, 'wb') as fo:
            zstandard.ZstdDecompressor().copy_stream(fi, fo)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--every', type=int, default=6, help='process every Nth depth frame (~5 Hz at 30 fps)')
    a = ap.parse_args()
    from rosbags.typesys import Stores, get_typestore
    ts_ = get_typestore(Stores.ROS2_HUMBLE)
    c = sqlite3.connect(open_db(a.bag))
    topics = {i: (n, t) for i, n, t in c.execute('select id,name,type from topics')}
    imu, depth, k = [], [], None
    for tid, stamp, data in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
        n, t = topics[tid]
        if n == '/imu/data':
            imu.append((stamp, ts_.deserialize_cdr(data, t).orientation))
        elif n == '/camera/depth/image_raw':
            depth.append((stamp, data, t))
        elif n == '/camera/depth/camera_info' and k is None:
            m = ts_.deserialize_cdr(data, t)
            k = (m.k[0], m.k[4], m.k[2], m.k[5])
    its = np.array([s for s, _ in imu])
    conf = S.Confirm()
    r_link = S.quat_to_matrix(*CAM_Q)
    t0 = depth[0][0]
    for i, (stamp, data, t) in enumerate(depth):
        if i % a.every:
            continue
        im = ts_.deserialize_cdr(data, t)
        d = np.frombuffer(bytes(im.data), np.uint16).reshape(im.height, im.width)
        q = imu[int(np.argmin(np.abs(its - stamp)))][1]
        r, tt = S.level_camera(S.roll_pitch_only(S.quat_to_matrix(q.x, q.y, q.z, q.w)), r_link, CAM_T)
        seen = S.detect(d, k, r, tt)
        st = conf.add((stamp - t0) / 1e9, seen)
        print(f'{(stamp - t0) / 1e9:6.2f}s  {seen.summary() if seen else "-- " + S.detect.last_why:70s}'
              f'  {"CONFIRMED %.2f m" % st.distance if st else ""}')


if __name__ == '__main__':
    main()
