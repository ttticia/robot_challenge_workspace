#!/usr/bin/env python3
"""Export a curtain YOLO model to OpenVINO (FP16 / INT8, camera-shaped inputs), time each
variant on this CPU and score it with eval_curtain_yolo.py on val and test.

    env -u PYTHONPATH ~/venvs/yolo-export/bin/python src/maze_nav/tools/bench_curtain_export.py \
        ~/challenge_ws/datasets/curtain [--weights runs/curtain/weights/best.pt] \
        [--sizes 320x416,256x320] [--frames 120]

Needs openvino + nncf (INT8). On the dev laptop they live in ~/venvs/yolo-export (made with
--system-site-packages, plus its own scipy / matplotlib: the system ones are built for NumPy
1.x and break nncf / the INT8 calibration). Unset PYTHONPATH so ROS's packages stay out.

Sizes are HxW. Use the camera's 4:3 shape (640x480 -> 320x416 or 256x320): a square export
(416x416) pads every frame with grey the model never saw in training and gave more false
boxes. INT8 is calibrated on the training list (<dataset>/split.yaml from
train_curtain_yolo.py). Exports land in <weights dir>/../export/curtain_<fp16|int8>_<W>x<H>
_openvino_model (Ultralytics recognises OpenVINO models by that suffix). Timings are one frame
at a time, like curtain_yolo.py; measure again on the robot, its CPU is slower.
"""
import argparse
import csv
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_curtain_yolo as E  # noqa: E402


def size_mb(path):
    if os.path.isdir(path):
        return sum(os.path.getsize(os.path.join(d, f))
                   for d, _, fs in os.walk(path) for f in fs) / 1e6
    return os.path.getsize(path) / 1e6


def export(best, out, quant, hw, data):
    path = os.path.join(out, f'curtain_{quant}_{hw[1]}x{hw[0]}_openvino_model')
    if not os.path.exists(path):
        from ultralytics import YOLO
        kw = dict(format='openvino', imgsz=list(hw), half=quant == 'fp16', int8=quant == 'int8')
        if quant == 'int8':
            kw['data'] = data
        shutil.move(YOLO(best).export(**kw), path)
    return path


def bench(path, imgsz, frames):
    from ultralytics import YOLO
    m = YOLO(path, task='detect')
    m.predict(frames[:5], imgsz=imgsz, verbose=False, device='cpu')      # warm-up
    t = time.time()
    for im in frames:
        m.predict(im, imgsz=imgsz, verbose=False, device='cpu')
    return (time.time() - t) / len(frames) * 1000


def summary(root, path, images, imgsz, ahead, label):
    res = E.score(root, path, images, [0.25, 0.5], 0.3, 1.5, imgsz, ahead)
    for conf, bags in sorted(res.items()):
        tot = {k: [0, 0, 0] for k in E.KEYS + ('neg',)}
        for k in bags.values():
            for key in tot:
                for i, v in enumerate(k[key]):
                    tot[key][i] += v
        found = sum(tot[k][1] for k in E.KEYS)
        n = sum(tot[k][0] for k in E.KEYS)
        parts = [f'{k} {tot[k][1]}/{tot[k][0]}' for k in E.KEYS if tot[k][0]]
        print(f'{label} conf {conf:.2f}: curtain frames {found}/{n} | ' + ' | '.join(parts) +
              f' | negatives with a box {tot["neg"][1]}/{tot["neg"][0]}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset')
    ap.add_argument('--weights', default='runs/curtain/weights/best.pt')
    ap.add_argument('--sizes', default='320x416,256x320', help='HxW list')
    ap.add_argument('--frames', type=int, default=120, help='frames timed per variant')
    a = ap.parse_args()
    os.environ.setdefault('YOLO_OFFLINE', '1')
    root = os.path.abspath(os.path.expanduser(a.dataset))
    best = a.weights if os.path.isabs(a.weights) else os.path.join(root, a.weights)
    out = os.path.join(os.path.dirname(os.path.dirname(best)), 'export')
    os.makedirs(out, exist_ok=True)
    read = lambda p: [ln.strip() for ln in open(os.path.join(root, p)) if ln.strip()]  # noqa
    val = read('split_val.txt')
    test = read('split_test.txt') if os.path.exists(os.path.join(root, 'split_test.txt')) else []
    ahead = {}
    man = os.path.join(root, 'manifest.csv')
    if os.path.exists(man):
        ahead = {r['image']: float(r['ahead_m']) for r in csv.DictReader(open(man))
                 if r.get('ahead_m')}
    variants = [('pytorch', 416, best)]
    for s in a.sizes.split(','):
        hw = tuple(int(x) for x in s.lower().split('x'))
        for quant in ('fp16', 'int8'):
            variants.append((quant, hw, export(best, out, quant, hw,
                                               os.path.join(root, 'split.yaml'))))
    for name, imgsz, path in variants:
        ms = bench(path, imgsz, (val + test)[:a.frames])
        shape = imgsz if isinstance(imgsz, int) else f'{imgsz[1]}x{imgsz[0]}'
        print(f'\n=== {name} {shape}: {ms:.1f} ms/frame, {size_mb(path):.1f} MB  ({path})',
              flush=True)
        summary(root, path, val, imgsz, ahead, 'VAL ')
        if test:
            summary(root, path, test, imgsz, ahead, 'TEST')


if __name__ == '__main__':
    main()
