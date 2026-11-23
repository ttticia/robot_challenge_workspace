"""Colour-only bag -> staging images/ + manifest.csv rows (curtain dataset naming).

    python3 extract_colour.py <bag_dir> <out_dir> <hz> [topic]
Frames <bag>_<SSSS>_<cc>.jpg (seconds, hundredths into the bag), at most <hz> per second."""
import csv
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

bag, out, hz = sys.argv[1].rstrip('/'), sys.argv[2], float(sys.argv[3])
topic = sys.argv[4] if len(sys.argv) > 4 else '/camera/color/image_raw/compressed'
name = os.path.basename(bag)
os.makedirs(os.path.join(out, 'images'), exist_ok=True)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
t0, last, rows = None, -1e9, []
while r.has_next():
    _, data, ts = r.read_next()
    t0 = ts if t0 is None else t0
    t = (ts - t0) * 1e-9
    if t - last < 1.0 / hz:
        continue
    im = cv2.imdecode(np.frombuffer(deserialize_message(data, CompressedImage).data, np.uint8),
                      cv2.IMREAD_COLOR)
    if im is None:
        continue
    last = t
    f = f'{name}_{int(t):04d}_{int(round((t % 1) * 100)) % 100:02d}.jpg'
    cv2.imwrite(os.path.join(out, 'images', f), im, [cv2.IMWRITE_JPEG_QUALITY, 92])
    rows.append({'image': f, 't_s': f'{t:.2f}', 'ahead_m': '', 'draft': 'no',
                 'reason': 'colour-only bag (no depth): no draft'})
with open(os.path.join(out, 'manifest.csv'), 'a', newline='') as fh:
    w = csv.DictWriter(fh, ['image', 't_s', 'ahead_m', 'draft', 'reason'])
    if fh.tell() == 0:
        w.writeheader()
    w.writerows(rows)
print(name, len(rows), 'frames')
