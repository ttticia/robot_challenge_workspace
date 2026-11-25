#!/usr/bin/env python3
"""Score a curtain YOLO model on the curtains the robot has to see (0.3-1.5 m), per bag.

    python3 src/maze_nav/tools/eval_curtain_yolo.py ~/challenge_ws/datasets/curtain \
        [--weights runs/curtain/weights/best.pt] [--list split_val.txt] [--conf 0.25,0.5]
        [--near 0.3] [--far 1.5] [--imgsz 416] [--on 0.5] [--cv]

The val mAP / recall of a training run counts every labelled curtain, also the small ones far
in the background that the robot doesn't have to detect, and asks for boxes that match the
label. This counts frames, and asks whether the robot would see a curtain:

  in range   the curtain box spans the image centre column and manifest.csv's ahead_m (median
             depth down that column, from curtain_yolo_frames.py) is within --near..--far.
             The acceptance number (to_do.md ## curtains: recall >= 0.9).
  out        same, but ahead_m outside the range.
  unknown    no depth for the frame, or the box is off the centre column (ahead_m is then the
             range to something else). Split by box height (>= / < 0.25 of the frame) as a
             rough near / far.
  negatives  frames without a curtain: any box is a false positive (acceptance: none at 0.5).

A curtain frame is DETECTED when a model box lies on a labelled curtain: at least --on (0.5) of
the box's area inside a label box. A box on one curtain half, or a box looser or tighter than
the label, still counts: the robot only needs to know a curtain is there (curtain_yolo places
it with depth). Also reported per group: 'iou' = frames with a box matching a label at IoU >=
0.5 (the old, stricter score), 'any' = frames with a box anywhere, and 'off-curtain' = curtain
frames with a box that lies on no curtain (a false box next to a real one). --cv scores every
runs/cv_<bag>/weights/best.pt on its own cv_<bag>_val.txt (after train_curtain_yolo.py --cv)
and adds the total.
"""
import argparse
import collections
import csv
import os
import re

KEYS = ('in range', 'out', 'unknown big', 'unknown small')


def bag_of(name):
    stem = os.path.splitext(os.path.basename(name))[0]
    m = re.match(r'(.+)_\d+_\d{2}$', stem)
    return m.group(1) if m else stem.rsplit('_', 1)[0]


def read_labels(root, im):
    p = os.path.join(root, 'labels', os.path.splitext(os.path.basename(im))[0] + '.txt')
    if not os.path.exists(p):
        return []
    return [tuple(map(float, ln.split()[1:5])) for ln in open(p) if ln.strip()]


def iou(a, b):
    """IoU of two (cx, cy, w, h) boxes."""
    iw = min(a[0] + a[2] / 2, b[0] + b[2] / 2) - max(a[0] - a[2] / 2, b[0] - b[2] / 2)
    ih = min(a[1] + a[3] / 2, b[1] + b[3] / 2) - max(a[1] - a[3] / 2, b[1] - b[3] / 2)
    inter = max(0.0, iw) * max(0.0, ih)
    return inter / (a[2] * a[3] + b[2] * b[3] - inter + 1e-9)


def inside(box, label):
    """Share of box's area (cx, cy, w, h) that lies inside label."""
    iw = min(box[0] + box[2] / 2, label[0] + label[2] / 2) - \
        max(box[0] - box[2] / 2, label[0] - label[2] / 2)
    ih = min(box[1] + box[3] / 2, label[1] + label[3] / 2) - \
        max(box[1] - box[3] / 2, label[1] - label[3] / 2)
    return max(0.0, iw) * max(0.0, ih) / (box[2] * box[3] + 1e-9)


def on_curtain(box, gt, on=0.5):
    """True if at least `on` of the box lies inside one of the label boxes."""
    return any(inside(box, g) >= on for g in gt)


def frame_group(gt, ahead, near, far):
    """Which group a curtain frame is scored in (see the module docstring)."""
    centred = [g for g in gt if g[0] - g[2] / 2 <= 0.5 <= g[0] + g[2] / 2]
    if centred and ahead is not None:
        return 'in range' if near <= ahead <= far else 'out'
    return 'unknown big' if max(g[3] for g in gt) >= 0.25 else 'unknown small'


def score(root, weights, images, confs, near, far, imgsz, ahead, on=0.5):
    """{conf: {bag: {group: [frames, detected, iou, any], 'neg': [frames, with a box, 0, 0],
    'off': [curtain frames, with an off-curtain box, 0, 0]}}}"""
    from ultralytics import YOLO
    res = YOLO(weights, task='detect').predict(images, imgsz=imgsz, conf=min(confs),
                                               verbose=False, device='cpu')
    out = {c: collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0, 0, 0]))
           for c in confs}
    for im, r in zip(images, res):
        gt = read_labels(root, im)
        boxes = [(tuple(b.tolist()), float(c)) for b, c in zip(r.boxes.xywhn, r.boxes.conf)]
        for conf in confs:
            pred = [b for b, c in boxes if c >= conf]
            k = out[conf][bag_of(im)]
            if not gt:
                k['neg'][0] += 1
                k['neg'][1] += bool(pred)
                continue
            g = k[frame_group(gt, ahead.get(os.path.basename(im)), near, far)]
            g[0] += 1
            g[1] += any(on_curtain(b, gt, on) for b in pred)
            g[2] += any(iou(a, b) >= 0.5 for a in gt for b in pred)
            g[3] += bool(pred)
            k['off'][0] += 1
            k['off'][1] += any(not on_curtain(b, gt, on) for b in pred)
    return out


def report(out, total=None):
    for conf, bags in sorted(out.items()):
        for bag, k in sorted(bags.items()):
            parts = [f'{key} {k[key][1]}/{k[key][0]} (iou {k[key][2]}, any {k[key][3]})'
                     for key in KEYS if k[key][0]]
            if k['off'][0]:
                parts.append(f'off-curtain box {k["off"][1]}/{k["off"][0]}')
            if k['neg'][0]:
                parts.append(f'negatives with a box {k["neg"][1]}/{k["neg"][0]}')
            print(f'conf {conf:.2f}  {bag:28s} ' + ' | '.join(parts))
            if total is not None:
                for key in KEYS + ('neg', 'off'):
                    t = total[conf][key]
                    for i, v in enumerate(k[key]):
                        t[i] += v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset')
    ap.add_argument('--weights', default='runs/curtain/weights/best.pt')
    ap.add_argument('--list', default='split_val.txt', help='image list (in the dataset)')
    ap.add_argument('--conf', default='0.25,0.5')
    ap.add_argument('--near', type=float, default=0.3)
    ap.add_argument('--far', type=float, default=1.5)
    ap.add_argument('--imgsz', type=int, default=416)
    ap.add_argument('--on', type=float, default=0.5,
                    help='share of a model box inside a label box for "on the curtain"')
    ap.add_argument('--cv', action='store_true', help='every cv_<bag> model on its own val')
    a = ap.parse_args()
    os.environ.setdefault('YOLO_OFFLINE', '1')
    root = os.path.abspath(os.path.expanduser(a.dataset))
    confs = [float(c) for c in a.conf.split(',')]
    ahead = {}
    man = os.path.join(root, 'manifest.csv')
    if os.path.exists(man):
        for r in csv.DictReader(open(man)):
            if r.get('ahead_m'):
                ahead[r['image']] = float(r['ahead_m'])
    path = lambda p: p if os.path.isabs(p) else os.path.join(root, p)  # noqa: E731
    read = lambda p: [ln.strip() for ln in open(path(p)) if ln.strip()]  # noqa: E731

    if not a.cv:
        out = score(root, path(a.weights), read(a.list), confs, a.near, a.far, a.imgsz, ahead,
                    a.on)
        total = {c: collections.defaultdict(lambda: [0, 0, 0, 0]) for c in confs}
        report(out, total)
        summary(total, 'ALL')
        return
    total = {c: collections.defaultdict(lambda: [0, 0, 0, 0]) for c in confs}
    folds = sorted(f[3:-8] for f in os.listdir(root)
                   if f.startswith('cv_') and f.endswith('_val.txt'))
    for bag in folds:
        w = os.path.join(root, 'runs', f'cv_{bag}', 'weights', 'best.pt')
        if not os.path.exists(w):
            print(f'{bag}: no model yet ({w})')
            continue
        report(score(root, w, read(f'cv_{bag}_val.txt'), confs, a.near, a.far, a.imgsz, ahead,
                     a.on), total)
    summary(total, 'ALL FOLDS')


def summary(total, label):
    """Totals over every bag: detected (and iou) per group, false boxes."""
    for conf, t in sorted(total.items()):
        parts = [f'{key} {t[key][1]}/{t[key][0]}' +
                 (f' = {t[key][1] / t[key][0]:.2f} (iou {t[key][2]})' if t[key][0] else '')
                 for key in KEYS]
        print(f'conf {conf:.2f}  {label}: detected ' + ' | '.join(parts) +
              f' | off-curtain box {t["off"][1]}/{t["off"][0]}'
              f' | negatives with a box {t["neg"][1]}/{t["neg"][0]}')


if __name__ == '__main__':
    main()
