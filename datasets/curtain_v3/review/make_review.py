"""Review sheets of missed and false curtain detections on val + test.

    env -u PYTHONPATH ~/venvs/yolo-export/bin/python review/make_review.py   (from curtain_v3/)
        [--model runs/curtain/export/curtain_int8_320x256_openvino_model] [--conf 0.5] [--imgsz 320]

Scored by detection, like tools/eval_curtain_yolo.py: a model box is ON a curtain when at least
half of its area lies inside a label box (a box on one curtain half still counts).
missed: a frame with a curtain label and no model box on a curtain.
false:  a model box on no curtain: on a frame labelled "no curtain" (empty label file), or next
        to a real curtain (off-curtain box on a curtain frame).
Writes review/missed_NN.jpg, review/false_NN.jpg (12 frames a sheet, numbered like review.csv)
and review/review.csv (fill in 'verdict': model error / label error / ok). The auto_reason
is a guess from the box geometry only; check every frame by eye.
"""
import argparse
import csv
import os

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--model', default='runs/curtain/export/curtain_int8_320x256_openvino_model')
ap.add_argument('--conf', type=float, default=0.5)
ap.add_argument('--imgsz', type=int, default=320)
ap.add_argument('--out', default='review')
a = ap.parse_args()
os.environ.setdefault('YOLO_OFFLINE', '1')
from ultralytics import YOLO  # noqa: E402


def iou(p, q):
    iw = min(p[2], q[2]) - max(p[0], q[0])
    ih = min(p[3], q[3]) - max(p[1], q[1])
    i = max(0, iw) * max(0, ih)
    return i / ((p[2] - p[0]) * (p[3] - p[1]) + (q[2] - q[0]) * (q[3] - q[1]) - i + 1e-9)


def inside(p, q):
    """Share of box p's area inside box q (x0, y0, x1, y1)."""
    iw = min(p[2], q[2]) - max(p[0], q[0])
    ih = min(p[3], q[3]) - max(p[1], q[1])
    return max(0, iw) * max(0, ih) / ((p[2] - p[0]) * (p[3] - p[1]) + 1e-9)


def on_curtain(p, gt):
    return any(inside(p, g) >= 0.5 for g in gt)


def reason(gt, best):
    if best > 0:
        return f'box touches the curtain but lies mostly off it (IoU {best:.2f})'
    if max(g[3] - g[1] for g in gt) < 0.25:
        return 'small / far curtain (label box < 25% of frame height)'
    if all(g[0] < 0.03 or g[2] > 0.97 for g in gt) and max(g[2] - g[0] for g in gt) < 0.3:
        return 'curtain at the frame edge'
    return 'no box on a large curtain label (model error, or check the label)'


ahead = {r['image']: float(r['ahead_m']) for r in csv.DictReader(open('manifest.csv'))
         if r.get('ahead_m')}
model = YOLO(a.model, task='detect')
items = {'missed': [], 'false': []}
for lst, split in (('split_val.txt', 'val'), ('split_test.txt', 'test')):
    for im in [ln.strip() for ln in open(lst) if ln.strip()]:
        f = os.path.basename(im)
        gt = [(x - w / 2, y - h / 2, x + w / 2, y + h / 2) for x, y, w, h in
              ([float(v) for v in ln.split()[1:]]
               for ln in open('labels/' + f[:-4] + '.txt') if ln.strip())]
        r = model.predict(im, imgsz=a.imgsz, conf=a.conf, verbose=False)[0]
        pred = [(tuple(b.tolist()), float(c)) for b, c in zip(r.boxes.xyxyn, r.boxes.conf)]
        best = max((iou(g, p) for g in gt for p, _ in pred), default=0.0)
        d = dict(im=im, f=f, split=split, a=ahead.get(f), gt=gt, pred=pred, best=best)
        if gt and not any(on_curtain(p, gt) for p, _ in pred):
            items['missed'].append(dict(d, why=reason(gt, best)))
        if not gt and pred:
            items['false'].append(dict(d, why='box on a frame labelled "no curtain"'))
        elif gt and any(not on_curtain(p, gt) for p, _ in pred):
            items['false'].append(dict(d, why='off-curtain box on a curtain frame'))
rank = {'no': 0, 'box': 1, 'curtain': 2, 'small': 3}
items['missed'].sort(key=lambda d: (rank.get(d['why'].split()[0], 4), d['f']))


def tile(d, idx, tw=480):
    img = cv2.imread(d['im'])
    img = cv2.resize(img, (tw, int(img.shape[0] * tw / img.shape[1])))
    H, W = img.shape[:2]
    for g in d['gt']:
        cv2.rectangle(img, (int(g[0] * W), int(g[1] * H)), (int(g[2] * W), int(g[3] * H)),
                      (0, 200, 0), 3)
    for p, c in d['pred']:
        cv2.rectangle(img, (int(p[0] * W), int(p[1] * H)), (int(p[2] * W), int(p[3] * H)),
                      (0, 0, 255), 2)
        cv2.putText(img, f'{c:.2f}', (int(p[0] * W) + 4, max(int(p[1] * H), 0) + 20), 0, 0.6,
                    (0, 0, 255), 2)
    cv2.rectangle(img, (0, 0), (46, 28), (0, 0, 0), -1)
    cv2.putText(img, str(idx), (5, 21), 0, 0.7, (255, 255, 255), 2)
    cap = f"{d['split']} | {d['f'][:-4]}" + (f" | {d['a']:.1f} m" if d['a'] else '')
    bar = np.zeros((44, W, 3), np.uint8)
    cv2.putText(bar, cap[:64], (5, 17), 0, 0.45, (255, 255, 255), 1)
    cv2.putText(bar, d['why'][:66], (5, 37), 0, 0.45, (170, 220, 255), 1)
    return np.vstack([img, bar])


os.makedirs(a.out, exist_ok=True)
for old in os.listdir(a.out):
    if old.endswith('.jpg') and old.split('_')[0] in items:
        os.remove(os.path.join(a.out, old))
legend = np.full((40, 4 * 480, 3), 255, np.uint8)
cv2.putText(legend, f'green = label (curtain), red = model box with confidence.  Model: '
            f'{os.path.basename(a.model)[:40]}, conf {a.conf}', (8, 26), 0, 0.6, (40, 40, 40), 1)
with open(os.path.join(a.out, 'review.csv'), 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(['sheet', 'idx', 'type', 'split', 'file', 'ahead_m', 'label_boxes', 'model_boxes',
                'max_conf', 'best_iou', 'auto_reason',
                'verdict (model error / label error / ok)', 'note'])
    for kind, lst in items.items():
        for s in range(0, len(lst), 12):
            page = [tile(d, i) for i, d in enumerate(lst[s:s + 12], start=s)]
            hmax = max(t.shape[0] for t in page)
            page = [cv2.copyMakeBorder(t, 0, hmax - t.shape[0], 0, 0, cv2.BORDER_CONSTANT)
                    for t in page]
            while len(page) % 4:
                page.append(np.full_like(page[0], 30))
            name = f'{kind}_{s // 12 + 1:02d}.jpg'
            cv2.imwrite(os.path.join(a.out, name),
                        np.vstack([legend] + [np.hstack(page[i:i + 4])
                                              for i in range(0, len(page), 4)]),
                        [cv2.IMWRITE_JPEG_QUALITY, 88])
            for i, d in enumerate(lst[s:s + 12], start=s):
                w.writerow([name, i, kind, d['split'], d['f'],
                            '' if d['a'] is None else f"{d['a']:.2f}", len(d['gt']),
                            len(d['pred']), f"{max((c for _, c in d['pred']), default=0):.2f}",
                            f"{d['best']:.2f}", d['why'], '', ''])
for k, v in items.items():
    print(f'{k}: {len(v)}')
    for d in v:
        print(f"   {d['f'][:-4]:40s} {d['why']}")
