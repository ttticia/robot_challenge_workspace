#!/usr/bin/env python3
"""Pull camera frames out of rosbags for dataset work.

    python3 datasets/extract_frames.py <bag_dir> [<bag_dir> ...] [options]

    --out DIR      where to write (default datasets/frames); each bag gets DIR/<bag>/
    --hz N         at most N frames per second (default 3; 0 = every frame)
    --topic T      image topic (default: the bag's colour topic, compressed if it has one)
    --start S      skip the first S seconds of the bag
    --end S        stop S seconds into the bag
    --dedup T      drop near-repeats: keep a frame only if it differs from every frame kept
                   in the last 3 s by >= T (same measure as curtain/dedup_frames.py, which
                   used 0.45). Off by default.
    --list         only print the image topics in each bag

Output per bag: DIR/<bag>/images/<bag>_<SSSS>_<cc>.jpg (seconds, hundredths into the bag,
counted from the bag's first message) and DIR/<bag>/manifest.csv (image, t_s, stamp).
Compressed JPEG frames are written byte for byte (no re-encoding); raw frames and other
formats are encoded as JPEG quality 95. Reads plain and zstd-compressed bags.
A bag that was extracted before is refused unless --force is given.

Run it niced on the robot computer: nice -n 19 python3 datasets/extract_frames.py ...
"""
import argparse
import csv
import os
import re
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMAGE_TYPES = ('sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/Image')
WINDOW = 3.0        # s: frames kept this recently are compared for --dedup


def open_bag(bag):
    """Reader for a bag folder, plain or zstd-compressed, with {topic: type}."""
    meta = os.path.join(bag, 'metadata.yaml')
    if not os.path.isfile(meta):
        raise SystemExit(f'{bag}: no metadata.yaml (not a rosbag2 folder)')
    text = open(meta).read()
    packed = re.search(r'compression_format:\s*\S*zstd', text)
    storage = re.search(r'storage_identifier:\s*(\S+)', text)
    r = rosbag2_py.SequentialCompressionReader() if packed else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage.group(1) if storage else 'sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return r, {t.name: t.type for t in r.get_all_topics_and_types()}, text


def bag_start_ns(meta_text):
    m = re.search(r'starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)', meta_text)
    return int(m.group(1)) if m else None


def image_topics(types):
    return sorted(t for t, ty in types.items() if ty in IMAGE_TYPES)


def pick_topic(types):
    """Colour camera topic: compressed before raw, never depth."""
    cands = [t for t in image_topics(types) if 'depth' not in t.lower()]
    cands.sort(key=lambda t: (types[t] != IMAGE_TYPES[0], 'color' not in t and 'colour' not in t, t))
    return cands[0] if cands else None


def raw_to_bgr(m):
    """sensor_msgs/Image -> BGR uint8 (or None for an encoding this tool does not know)."""
    enc = m.encoding.lower()
    buf = np.frombuffer(bytes(m.data), np.uint8)
    if enc in ('bgr8', 'rgb8'):
        im = buf.reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        return im[:, :, ::-1].copy() if enc == 'rgb8' else im
    if enc in ('bgra8', 'rgba8'):
        im = buf.reshape(m.height, m.step)[:, :m.width * 4].reshape(m.height, m.width, 4)
        return cv2.cvtColor(im, cv2.COLOR_BGRA2BGR if enc == 'bgra8' else cv2.COLOR_RGBA2BGR)
    if enc in ('mono8', '8uc1'):
        return cv2.cvtColor(buf.reshape(m.height, m.step)[:, :m.width], cv2.COLOR_GRAY2BGR)
    if enc in ('mono16', '16uc1'):          # depth in mm or 16-bit grey: scaled for viewing only
        d = np.frombuffer(bytes(m.data), '>u2' if m.is_bigendian else '<u2')
        d = d.reshape(m.height, m.step // 2)[:, :m.width]
        top = max(int(d.max()), 1)
        return cv2.cvtColor((d.astype(np.float32) * (255.0 / top)).astype(np.uint8),
                            cv2.COLOR_GRAY2BGR)
    return None


def thumb(im):
    g = cv2.cvtColor(cv2.resize(im, (24, 18), interpolation=cv2.INTER_AREA),
                     cv2.COLOR_BGR2GRAY).astype(np.float32)
    return (g - g.mean()) / (g.std() + 1e-6)


def sdiff(a, b, r=2):
    """Mean abs difference of two thumbnails, the smallest over shifts of up to r pixels."""
    h, w = a.shape
    return min(np.abs(a[max(0, dy):h + min(0, dy), max(0, dx):w + min(0, dx)]
                      - b[max(0, -dy):h + min(0, -dy), max(0, -dx):w + min(0, -dx)]).mean()
               for dy in range(-r, r + 1) for dx in range(-r, r + 1))


def extract(bag, a):
    bag = os.path.abspath(bag.rstrip('/'))
    name = os.path.basename(bag)
    r, types, meta = open_bag(bag)
    if a.list:
        counts = dict(re.findall(r'name:\s*(\S+)\s*\n\s*type:.*\n(?:.*\n)*?\s*message_count:\s*(\d+)',
                                 meta))
        for t in image_topics(types):
            print(f'{name}: {t}  [{types[t]}]  {counts.get(t, "?")} msgs')
        if not image_topics(types):
            print(f'{name}: no image topics')
        return 0
    topic = a.topic or pick_topic(types)
    if topic not in types:
        print(f'{name}: SKIPPED, no image topic' + (f' {a.topic}' if a.topic else '')
              + f' (has: {", ".join(image_topics(types)) or "none"})', file=sys.stderr)
        return None
    if types[topic] not in IMAGE_TYPES:
        raise SystemExit(f'{name}: {topic} is {types[topic]}, not an image topic')
    out = os.path.join(a.out, name)
    img_dir = os.path.join(out, 'images')
    if os.path.isdir(img_dir) and os.listdir(img_dir) and not a.force:
        raise SystemExit(f'{img_dir} already has frames; use --force to add to / overwrite them')
    os.makedirs(img_dir, exist_ok=True)

    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    msg_type = get_message(types[topic])
    compressed = types[topic] == IMAGE_TYPES[0]
    t0 = bag_start_ns(meta)
    last, kept, rows, seen, bad, repeats = -1e9, [], [], 0, 0, 0
    used = set()
    while r.has_next():
        _, data, ts = r.read_next()
        t0 = ts if t0 is None else t0
        t = (ts - t0) * 1e-9
        if t < a.start:
            continue
        if a.end is not None and t > a.end:
            break
        seen += 1
        if a.hz > 0 and t - last < 1.0 / a.hz - 1e-6:
            continue
        m = deserialize_message(data, msg_type)
        jpeg = None
        if compressed:
            raw = bytes(m.data)
            im = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if raw[:2] == b'\xff\xd8':
                jpeg = raw
        else:
            im = raw_to_bgr(m)
        if im is None:
            bad += 1
            continue
        if a.dedup > 0:
            th = thumb(im)
            kept = [k for k in kept if t - k[0] <= WINDOW]
            if any(sdiff(th, k[1]) < a.dedup for k in kept):
                repeats += 1
                last = t
                continue
            kept.append((t, th))
        last = t
        cs = int(t * 100)
        f = f'{name}_{cs // 100:04d}_{cs % 100:02d}.jpg'
        while f in used:                    # two frames in the same hundredth (--hz 0)
            cs += 1
            f = f'{name}_{cs // 100:04d}_{cs % 100:02d}.jpg'
        used.add(f)
        path = os.path.join(img_dir, f)
        if jpeg is not None:
            with open(path, 'wb') as fh:
                fh.write(jpeg)
        else:
            cv2.imwrite(path, im, [cv2.IMWRITE_JPEG_QUALITY, 95])
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        rows.append({'image': f, 't_s': f'{int(t * 100) / 100:.2f}', 'stamp': f'{stamp:.3f}'})

    with open(os.path.join(out, 'manifest.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, ['image', 't_s', 'stamp'])
        w.writeheader()
        w.writerows(rows)
    notes = [f'{seen} on {topic}']
    if repeats:
        notes.append(f'{repeats} near-repeats dropped')
    if bad:
        notes.append(f'{bad} could not be decoded')
    print(f'{name}: {len(rows)} frames -> {img_dir}  ({", ".join(notes)})')
    return len(rows)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+', help='rosbag2 folders (the ones holding metadata.yaml)')
    ap.add_argument('--out', default=os.path.join(here, 'frames'))
    ap.add_argument('--hz', type=float, default=3.0)
    ap.add_argument('--topic')
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--end', type=float)
    ap.add_argument('--dedup', type=float, default=0.0)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    results = [extract(b, a) for b in a.bags]
    if not a.list and any(n is None for n in results):
        sys.exit(1)


if __name__ == '__main__':
    main()
