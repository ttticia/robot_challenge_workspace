#!/usr/bin/env python3
"""Hard-negative mining for the curtain YOLO: run a model over frames that are NOT in the
dataset (and over the dataset's own negatives), collect every frame it puts a box on, and
write review sheets + a CSV to fill in.

    python3 src/maze_nav/tools/mine_curtain_negatives.py <dataset> <out> --pool DIR [--pool DIR]
        [--weights runs/curtain/weights/best.pt] [--conf 0.25] [--imgsz 416]
        [--val-bags a,b] [--test-bags c] [--per-sheet 12]

<dataset>: images/ + labels/ (the current, reviewed labels). Each --pool is a folder of
.jpg frames (curtain_yolo_frames.py / extract_frames.py names <bag>_<SSSS>_<cc>.jpg), e.g. the
old dataset's duplicates/ and excluded/, or frames extracted from a new bag; a labels/ folder
inside it is read as the OLD label of each frame (frames whose old label has a curtain are
skipped).

A pool frame is often a near-duplicate of a dataset frame (dedup_frames.py kept one of a run
of similar frames), so each candidate gets the nearest dataset frame of the same bag in time
and that frame's CURRENT label:
  suggestion 'hard negative?'      model box, nearest dataset frame has no curtain
  suggestion 'unlabelled curtain?' model box, nearest dataset frame has a curtain (the box is
                                   probably right: label it, don't add it as a negative)
  suggestion 'check'               no dataset frame of that bag within --max-dt s (a new bag)
Dataset negatives the model boxes are listed too (source 'dataset'): to_do.md step 1, a curtain
the labels miss, or a real hard example already in the set.

Frames of val / test bags are marked: add them to val / test only, never to training (leak).

Output in <out>: candidates.csv (fill the 'decision' column: neg / curtain / skip),
sheets/sheet_NN.jpg (red = model box with confidence, green = the nearest dataset frame's
current label; the number in the corner is the CSV 'idx'), images/ (copies of the candidate
frames, to open in the labelling tool).
"""
import argparse
import bisect
import csv
import os
import re
import shutil

import cv2
import numpy as np

NAME = re.compile(r'(.+)_(\d+)_(\d{2})\.jpg$')


def parse(name):
    """(bag, seconds) of <bag>_<SSSS>_<cc>.jpg, or (stem, None)."""
    m = NAME.match(os.path.basename(name))
    if not m:
        return os.path.splitext(os.path.basename(name))[0], None
    return m.group(1), int(m.group(2)) + int(m.group(3)) / 100


def read_label(path):
    if not os.path.exists(path):
        return None
    return [tuple(map(float, ln.split()[1:5])) for ln in open(path) if ln.strip()]


def draw(img, pred, gt, idx, title, tile):
    h, w = img.shape[:2]
    a = cv2.resize(img, (tile, int(h * tile / w)))
    H, W = a.shape[:2]
    for x, y, bw, bh in gt or []:
        cv2.rectangle(a, (int((x - bw / 2) * W), int((y - bh / 2) * H)),
                      (int((x + bw / 2) * W), int((y + bh / 2) * H)), (0, 200, 0), 2)
    for (x1, y1, x2, y2), c in pred:
        p1, p2 = (int(x1 * W), int(y1 * H)), (int(x2 * W), int(y2 * H))
        cv2.rectangle(a, p1, p2, (0, 0, 255), 2)
        cv2.putText(a, f'{c:.2f}', (p1[0] + 3, max(p1[1], 0) + 18), 0, 0.6, (0, 0, 255), 2)
    cv2.rectangle(a, (0, 0), (52, 26), (0, 0, 0), -1)
    cv2.putText(a, str(idx), (4, 20), 0, 0.7, (255, 255, 255), 2)
    cv2.rectangle(a, (0, H - 20), (W, H), (0, 0, 0), -1)
    cv2.putText(a, title[:58], (4, H - 6), 0, 0.42, (255, 255, 255), 1)
    return a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset')
    ap.add_argument('out')
    ap.add_argument('--pool', action='append', default=[])
    ap.add_argument('--weights', default='runs/curtain/weights/best.pt')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--imgsz', type=int, default=416)
    ap.add_argument('--val-bags', default='')
    ap.add_argument('--test-bags', default='')
    ap.add_argument('--max-dt', type=float, default=10.0)
    ap.add_argument('--per-sheet', type=int, default=12)
    ap.add_argument('--tile', type=int, default=400)
    ap.add_argument('--no-dataset', action='store_true', help="skip the dataset's negatives")
    a = ap.parse_args()
    os.environ.setdefault('YOLO_OFFLINE', '1')
    from ultralytics import YOLO

    root = os.path.abspath(os.path.expanduser(a.dataset))
    weights = a.weights if os.path.isabs(a.weights) else os.path.join(root, a.weights)
    val = {b for b in a.val_bags.split(',') if b}
    test = {b for b in a.test_bags.split(',') if b}

    def role(bag):
        return 'TEST bag' if bag in test else 'VAL bag' if bag in val else 'train'

    ds = {}                         # current dataset: per bag, sorted (t, name, label)
    for f in sorted(os.listdir(os.path.join(root, 'images'))):
        if not f.endswith('.jpg') or '_mirror_' in f:
            continue
        bag, t = parse(f)
        lab = read_label(os.path.join(root, 'labels', f[:-4] + '.txt')) or []
        ds.setdefault(bag, []).append((t if t is not None else -1.0, f, lab))
    for v in ds.values():
        v.sort()
    in_ds = {f for v in ds.values() for _, f, _ in v}

    def nearest(bag, t):
        rows = ds.get(bag)
        if not rows or t is None:
            return None
        ts = [r[0] for r in rows]
        i = bisect.bisect_left(ts, t)
        best = min((rows[j] for j in (i - 1, i) if 0 <= j < len(rows)),
                   key=lambda r: abs(r[0] - t))
        return best if abs(best[0] - t) <= a.max_dt else None

    jobs = []                       # (source, path, old label or None)
    for pool in a.pool:
        pool = os.path.abspath(os.path.expanduser(pool))
        for f in sorted(os.listdir(pool)):
            if f.endswith('.jpg') and f not in in_ds:
                jobs.append((os.path.basename(pool.rstrip('/')), os.path.join(pool, f),
                             read_label(os.path.join(pool, 'labels', f[:-4] + '.txt'))))
    if not a.no_dataset:
        for rows in ds.values():
            for _, f, lab in rows:
                if not lab:
                    jobs.append(('dataset', os.path.join(root, 'images', f), []))
    n_ds = sum(j[0] == 'dataset' for j in jobs)
    print(f'{len(jobs)} frames to check ({len(jobs) - n_ds} from pools, {n_ds} dataset '
          'negatives)', flush=True)

    model = YOLO(weights, task='detect')
    cands = []
    for k in range(0, len(jobs), 32):
        chunk = jobs[k:k + 32]
        res = model.predict([j[1] for j in chunk], imgsz=a.imgsz, conf=a.conf, verbose=False,
                            device='cpu')
        for (src, path, old), r in zip(chunk, res):
            if not len(r.boxes) or old:     # no box, or the old label already had a curtain
                continue
            pred = sorted(((tuple(b.tolist()), float(c)) for b, c in
                           zip(r.boxes.xyxyn, r.boxes.conf)), key=lambda x: -x[1])
            f = os.path.basename(path)
            bag, t = parse(f)
            if src == 'dataset':
                near, sug = (t, f, []), 'dataset negative: curtain missed or hard example?'
            else:
                near = nearest(bag, t)
                sug = 'check' if near is None else \
                    'unlabelled curtain?' if near[2] else 'hard negative?'
            cands.append(dict(src=src, path=path, file=f, bag=bag, role=role(bag),
                              conf=pred[0][1], n=len(pred), pred=pred, near=near, sug=sug))
        print(f'  {min(k + 32, len(jobs))}/{len(jobs)} checked, {len(cands)} candidates',
              flush=True)

    order = {'hard negative?': 0, 'check': 1, 'unlabelled curtain?': 2}
    cands.sort(key=lambda c: (order.get(c['sug'], 3), -c['conf']))
    for d in ('sheets', 'images'):
        os.makedirs(os.path.join(a.out, d), exist_ok=True)
    tiles = []
    with open(os.path.join(a.out, 'candidates.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['idx', 'sheet', 'file', 'source', 'bag', 'bag_role', 'max_conf', 'boxes',
                    'nearest_dataset_frame', 'nearest_has_curtain', 'dt_s', 'suggestion',
                    'decision (neg / curtain / skip)', 'note'])
        for i, c in enumerate(cands):
            near = c['near']
            t = parse(c['file'])[1]
            dt = '' if near is None or t is None or near[0] < 0 else f'{abs(near[0] - t):.1f}'
            w.writerow([i, f'sheet_{i // a.per_sheet:02d}.jpg', c['file'], c['src'], c['bag'],
                        c['role'], f'{c["conf"]:.2f}', c['n'],
                        '' if near is None else near[1],
                        '' if near is None else ('yes' if near[2] else 'no'), dt, c['sug'],
                        '', ''])
            shutil.copy2(c['path'], os.path.join(a.out, 'images', c['file']))
            gt = near[2] if near is not None and c['src'] != 'dataset' else []
            tiles.append(draw(cv2.imread(c['path']), c['pred'], gt, i,
                              f'{c["sug"]} | {c["role"]} | {c["file"]}', a.tile))
    cols = 4
    for s in range(0, len(tiles), a.per_sheet):
        page = tiles[s:s + a.per_sheet]
        hmax = max(t.shape[0] for t in page)
        page = [cv2.copyMakeBorder(t, 0, hmax - t.shape[0], 0, 0, cv2.BORDER_CONSTANT)
                for t in page]
        while len(page) % cols:
            page.append(np.zeros_like(page[0]))
        rows = [np.hstack(page[i:i + cols]) for i in range(0, len(page), cols)]
        cv2.imwrite(os.path.join(a.out, 'sheets', f'sheet_{s // a.per_sheet:02d}.jpg'),
                    np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 85])
    counts = {}
    for c in cands:
        counts[(c['sug'], c['role'])] = counts.get((c['sug'], c['role']), 0) + 1
    print(f'{len(cands)} candidates -> {a.out}')
    for (sug, r), n in sorted(counts.items()):
        print(f'  {n:4d}  {sug:50s} {r}')


if __name__ == '__main__':
    main()
