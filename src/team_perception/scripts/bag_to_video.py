#!/usr/bin/env python3
"""Convert a camera topic in a ROS 2 bag to an .mp4 video.

Usage:
  python3 bag_to_video.py <bag_folder> [output.mp4] [topic]

Defaults: output is <bag_folder>.mp4, topic is /camera/color/image_raw/compressed.
Handles sensor_msgs/CompressedImage and sensor_msgs/Image (rgb8/bgr8/mono8).
"""

import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image


def to_bgr(msg):
    if isinstance(msg, CompressedImage):
        return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(img[:, :msg.width], cv2.COLOR_GRAY2BGR)
    img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    bag = sys.argv[1].rstrip('/')
    out = sys.argv[2] if len(sys.argv) > 2 else bag + '.mp4'
    topic = sys.argv[3] if len(sys.argv) > 3 else '/camera/color/image_raw/compressed'

    def open_reader():
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        return reader

    reader = open_reader()
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        sys.exit(f'Topic {topic} not in bag. Available: {list(types)}')
    msg_type = CompressedImage if types[topic].endswith('CompressedImage') else Image

    # First pass: timestamps only, to get the real frame rate.
    stamps = []
    while reader.has_next():
        stamps.append(reader.read_next()[2])
    if len(stamps) < 2:
        sys.exit(f'Only {len(stamps)} frame(s) on {topic}; nothing to write.')
    fps = (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)

    # Second pass: decode and stream frames into the video.
    reader = open_reader()
    writer, count = None, 0
    while reader.has_next():
        frame = to_bgr(deserialize_message(reader.read_next()[1], msg_type))
        if frame is None:
            continue
        if writer is None:
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'avc1'), fps, (w, h))
        writer.write(frame)
        count += 1
    writer.release()
    print(f'Wrote {out}: {count} frames, {w}x{h} @ {fps:.1f} fps')


if __name__ == '__main__':
    main()
