"""Contact sheets for labelling review.

    python3 -I sheet.py <prefix> <out_dir> [--cols 6 --rows 5 --tile 266] [--grid]
                       [--boxes boxes.json | --labels] [--only idx.txt]

Frames images/<prefix>*.jpg in name order, numbered 0..N-1 (the index used in boxes.json).
--grid draws a 10x10 grid (yellow = centre lines) for placing boxes by eye; --boxes draws
{index: [[x0, y0, x1, y1], ...]} (normalised), --labels draws the YOLO labels on disk.
"""
import argparse
import glob
import json
import os

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('prefix')
ap.add_argument('out')
ap.add_argument('--cols', type=int, default=6)
ap.add_argument('--rows', type=int, default=5)
ap.add_argument('--tile', type=int, default=266)
ap.add_argument('--grid', action='store_true')
ap.add_argument('--boxes')
ap.add_argument('--labels', action='store_true')
ap.add_argument('--only')
a = ap.parse_args()

names = sorted(os.path.basename(f) for f in glob.glob(os.path.join('images', a.prefix + '*.jpg')))
idx = list(range(len(names)))
if a.only:
    keep = {int(x) for x in open(a.only).read().split()}
    idx = [i for i in idx if i in keep]
boxes = {int(k): v for k, v in json.load(open(a.boxes)).items()} if a.boxes else {}
if a.labels:
    for i, n in enumerate(names):
        p = os.path.join('labels', n[:-4] + '.txt')
        if os.path.exists(p):
            boxes[i] = [[x - w / 2, y - h / 2, x + w / 2, y + h / 2] for x, y, w, h in
                        (map(float, line.split()[1:]) for line in open(p) if line.strip())]
os.makedirs(a.out, exist_ok=True)
for f in glob.glob(os.path.join(a.out, 'sheet_*.jpg')):
    os.remove(f)
tw = a.tile
th = tw * 3 // 4
per = a.cols * a.rows
for s in range(0, len(idx), per):
    sheet = np.full((a.rows * th, a.cols * tw, 3), 40, np.uint8)
    for k, i in enumerate(idx[s:s + per]):
        im = cv2.resize(cv2.imread(os.path.join('images', names[i])), (tw, th))
        if a.grid:
            for g in range(1, 10):
                c = (0, 255, 255) if g == 5 else (255, 255, 0)
                cv2.line(im, (g * tw // 10, 0), (g * tw // 10, th), c, 1)
                cv2.line(im, (0, g * th // 10), (tw, g * th // 10), c, 1)
                cv2.putText(im, str(g), (g * tw // 10 + 2, 12), 0, 0.4, c, 1)
                cv2.putText(im, str(g), (2, g * th // 10 - 2), 0, 0.4, c, 1)
        for x0, y0, x1, y1 in boxes.get(i, []) if boxes.get(i) != 'X' else []:
            cv2.rectangle(im, (int(x0 * tw), int(y0 * th)), (int(x1 * tw) - 1, int(y1 * th) - 1),
                          (0, 255, 0), 2)
        cv2.rectangle(im, (0, th - 22), (70, th), (0, 0, 0), -1)
        cv2.putText(im, str(i), (4, th - 5), 0, 0.6, (255, 255, 255), 2)
        r, c = divmod(k, a.cols)
        sheet[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = im
    cv2.imwrite(os.path.join(a.out, f'sheet_{s // per:02d}.jpg'), sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])
print(len(idx), 'frames,', (len(idx) + per - 1) // per, 'sheets in', a.out)
