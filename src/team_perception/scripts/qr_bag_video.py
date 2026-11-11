#!/usr/bin/env python3
"""Run the team QR scanner on a ROS 2 bag and write an annotated .mp4.

Usage:
  python3 qr_bag_video.py <bag_folder> [output.mp4] [topic]

Defaults: output is <bag_folder>/<bag_name>_qr.mp4,
topic is /camera/color/image_raw/compressed.
Also writes <output>.csv with one row per decoded frame.
Needs the team_perception package sourced (source ~/ros2_ws/install/setup.bash).
"""
import csv
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image

from team_perception.qr_utils import BACKENDS, decode_qr, draw_detections

HOLD_S = 0.5   # keep a box on screen this long after its last decode (same as the live node)


def to_bgr(msg):
    if isinstance(msg, CompressedImage):
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    if msg.encoding in ('mono8', '8UC1'):
        return cv2.cvtColor(img[:, :msg.width], cv2.COLOR_GRAY2BGR)
    img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img


def open_reader(bag, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return reader


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    bag = sys.argv[1].rstrip('/')
    name = os.path.basename(bag)
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(bag, name + '_qr.mp4')
    topic = sys.argv[3] if len(sys.argv) > 3 else '/camera/color/image_raw/compressed'

    reader = open_reader(bag, topic)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        sys.exit(f'Topic {topic} not in bag. Available: {list(types)}')
    msg_type = CompressedImage if types[topic].endswith('CompressedImage') else Image

    stamps = []
    while reader.has_next():
        stamps.append(reader.read_next()[2])
    if len(stamps) < 2:
        sys.exit(f'Only {len(stamps)} frame(s) on {topic}; nothing to write.')
    t0 = stamps[0]
    fps = (len(stamps) - 1) / ((stamps[-1] - t0) / 1e9)

    reader = open_reader(bag, topic)
    writer = None
    frames = hits = 0
    last = {}      # text -> (corners, time last decoded)
    seen = {}      # text -> [first time, last time, frames decoded]
    rows = []
    while reader.has_next():
        _, data, stamp = reader.read_next()
        img = to_bgr(deserialize_message(data, msg_type))
        if img is None:
            continue
        t = (stamp - t0) / 1e9
        codes = decode_qr(img)
        if codes:
            hits += 1
        for text, corners in codes:
            last[text] = (corners, t)
            s = seen.setdefault(text, [t, t, 0])
            s[1], s[2] = t, s[2] + 1
            rows.append([f'{t:.2f}', frames, text])
        shown = [(txt, c) for txt, (c, tl) in last.items() if t - tl <= HOLD_S]
        header = [f'{name}  t={t:5.1f}s  frame {frames}',
                  f'decoder: {BACKENDS}',
                  f'codes found so far: {len(seen)}']
        draw_detections(img, shown, None, header)
        if writer is None:
            h, w = img.shape[:2]
            writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'avc1'), fps, (w, h))
        writer.write(img)
        frames += 1
    writer.release()

    with open(os.path.splitext(out)[0] + '.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time_s', 'frame', 'text'])
        w.writerows(rows)

    print(f'Wrote {out}: {frames} frames @ {fps:.1f} fps, '
          f'QR decoded in {hits} frames ({100 * hits / max(frames, 1):.0f}%)')
    for text, (first, lst, n) in seen.items():
        print(f'  {n:4d} frames  {first:6.1f}s - {lst:6.1f}s  "{text}"')


if __name__ == '__main__':
    main()
