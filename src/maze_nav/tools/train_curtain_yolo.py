#!/usr/bin/env python3
"""Train the curtain YOLO model (maze_nav/curtain_yolo.py) from a reviewed dataset.

    python3 src/maze_nav/tools/train_curtain_yolo.py ~/challenge_ws/datasets/curtain \
        [--epochs 80] [--imgsz 416] [--base yolo11n.pt] [--val 0.2] [--install]

The dataset is what tools/curtain_yolo_frames.py writes (images/*.jpg, labels/*.txt, YOLO
format, class 0 = curtain), AFTER every label has been checked in a labelling tool (the drafts
are automatic and wrong at times). An image without a label file, or with an empty one, is a
negative (no curtain): keep plenty (walls, wall cavities, doorways, slats, the lab curtain from
behind). The split is by bag (the file name up to the last '_'), so frames of one bag never sit
in both train and val. --install copies best.pt to ~/.cache/team_perception/curtain_yolo.pt
(curtain_yolo's default model) and exports an OpenVINO copy next to it for the robot CPU.
Check the val numbers and the to_do.md ## curtains acceptance test before relying on it.
"""
import argparse
import os
import random
import shutil

CACHE = os.path.expanduser('~/.cache/team_perception')


def bag_of(name):
    stem = os.path.splitext(os.path.basename(name))[0]
    return stem.rsplit('_', 1)[0] if '_' in stem else stem


def split_by_bag(images, val_frac, seed=0):
    """(train, val) image lists; whole bags go to val until val_frac of the images are in."""
    bags = {}
    for im in images:
        bags.setdefault(bag_of(im), []).append(im)
    names = sorted(bags)
    random.Random(seed).shuffle(names)
    val, train, n = [], [], len(images)
    for b in names:
        (val if len(val) < val_frac * n and len(names) > 1 else train).extend(bags[b])
    if not train:                         # one bag only: split its frames instead
        train, val = val[:int(len(val) * (1 - val_frac))], val[int(len(val) * (1 - val_frac)):]
    return sorted(train), sorted(val)


def count_labels(root, images):
    pos = 0
    for im in images:
        lab = os.path.join(root, 'labels', os.path.splitext(os.path.basename(im))[0] + '.txt')
        if os.path.exists(lab) and os.path.getsize(lab) > 0:
            pos += 1
    return pos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset')
    ap.add_argument('--epochs', type=int, default=80)
    ap.add_argument('--imgsz', type=int, default=416)
    ap.add_argument('--base', default='yolo11n.pt', help='start weights (bare name -> cache)')
    ap.add_argument('--val', type=float, default=0.2)
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--install', action='store_true')
    a = ap.parse_args()
    root = os.path.abspath(os.path.expanduser(a.dataset))
    img_dir = os.path.join(root, 'images')
    images = sorted(os.path.join(img_dir, f) for f in os.listdir(img_dir)
                    if f.lower().endswith(('.jpg', '.jpeg', '.png')))
    if not images:
        raise SystemExit(f'no images in {img_dir}')
    train, val = split_by_bag(images, a.val)
    for name, lst in (('train', train), ('val', val)):
        with open(os.path.join(root, f'{name}.txt'), 'w') as f:
            f.write('\n'.join(lst) + '\n')
        print(f'{name}: {len(lst)} images, {count_labels(root, lst)} with a curtain, '
              f'{len({bag_of(i) for i in lst})} bag(s)')
    data = os.path.join(root, 'data.yaml')
    with open(data, 'w') as f:
        f.write(f'path: {root}\ntrain: train.txt\nval: val.txt\nnames:\n  0: curtain\n')
    os.environ.setdefault('YOLO_OFFLINE', '1')
    from ultralytics import YOLO
    base = a.base if os.sep in a.base else os.path.join(CACHE, a.base)
    model = YOLO(base if os.path.exists(base) else a.base)
    res = model.train(data=data, epochs=a.epochs, imgsz=a.imgsz, device=a.device,
                      project=os.path.join(root, 'runs'), name='curtain', exist_ok=True,
                      fliplr=0.5, mosaic=1.0, patience=20)
    best = os.path.join(str(res.save_dir), 'weights', 'best.pt')
    print(f'best weights: {best}')
    if a.install:
        os.makedirs(CACHE, exist_ok=True)
        dst = os.path.join(CACHE, 'curtain_yolo.pt')
        shutil.copy(best, dst)
        print(f'installed {dst}')
        try:
            out = YOLO(dst).export(format='openvino', imgsz=a.imgsz)
            print(f'OpenVINO export: {out} (curtain_yolo_model:=curtain_yolo_openvino_model)')
        except Exception as e:          # openvino not installed on this machine
            print(f'OpenVINO export skipped ({e})')


if __name__ == '__main__':
    main()
