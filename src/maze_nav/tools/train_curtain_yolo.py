#!/usr/bin/env python3
"""Train the curtain YOLO model (maze_nav/curtain_yolo.py) from a reviewed dataset.

    python3 src/maze_nav/tools/train_curtain_yolo.py ~/challenge_ws/datasets/curtain \
        [--epochs 80] [--imgsz 416] [--base yolo11n.pt] [--install]
        [--val 0.2 | --val-bags bagA,bagB] [--test-bags bagC] [--cv] [--dry-run]

Split (whole bags only): --val-bags names the val bags; otherwise bags with a curtain go to val
until it holds --val of the curtain frames (one always trains), and negatives-only bags join
val only while there is more than one. Val without a curtain is refused. --test-bags are kept
out of both (acceptance test set, <tag>_test.txt). --cv trains one model per curtain bag with
that bag as val (leave-one-bag-out; runs/cv_summary.csv) - the honest score while there are
few bags. --dry-run prints the split and writes the lists without training.

The dataset is what tools/curtain_yolo_frames.py writes (images/*.jpg, labels/*.txt, YOLO
format, class 0 = curtain), AFTER every label has been checked in a labelling tool (the drafts
are automatic and wrong at times). An image without a label file, or with an empty one, is a
negative (no curtain): keep plenty (walls, wall cavities, doorways, slats, the lab curtain from
behind). The split is by bag (the file name before _<SSSS>_<cc>), so frames of one bag never sit
in both train and val. --install copies best.pt to ~/.cache/team_perception/curtain_yolo.pt
(curtain_yolo's default model) and exports an OpenVINO copy next to it for the robot CPU.
Check the val numbers and the to_do.md ## curtains acceptance test before relying on it.
"""
import argparse
import os
import random
import re
import shutil

CACHE = os.path.expanduser('~/.cache/team_perception')


def bag_of(name):
    """Bag of a frame: curtain_yolo_frames.py names them <bag>_<SSSS>_<cc>.jpg (seconds,
    hundredths), and bag names have '_' in them too. Other names: up to the last '_'."""
    stem = os.path.splitext(os.path.basename(name))[0]
    m = re.match(r'(.+)_\d+_\d{2}$', stem)
    if m:
        return m.group(1)
    return stem.rsplit('_', 1)[0] if '_' in stem else stem


def group_by_bag(images):
    bags = {}
    for im in images:
        bags.setdefault(bag_of(im), []).append(im)
    return bags


def split_by_bag(images, val_frac, seed=0, is_pos=None, val_bags=None):
    """(train, val) image lists, whole bags on one side.

    val_bags: these bags are val, the rest train. Otherwise automatic: bags with a curtain
    (is_pos true for some frame; is_pos None = every frame counts) go to val until it holds
    val_frac of the curtain frames, keeping at least one curtain bag for training; of the bags
    with no curtain (negatives-only recordings) one goes to val when there are 2+ (false
    positives get measured), more while val_frac of the negative frames isn't reached, and at
    least one stays in train. So val always has curtains to score - a
    negatives-only val set makes mAP / recall meaningless (it got all of val on 2026-11-23).
    One curtain bag only: its frames are split in time (last val_frac to val) - a leaky
    fallback, neighbouring frames are near-identical."""
    is_pos = is_pos or (lambda im: True)
    bags = group_by_bag(images)
    if val_bags is not None:
        unknown = set(val_bags) - set(bags)
        if unknown:
            raise ValueError(f'unknown val bag(s) {sorted(unknown)}; bags: {sorted(bags)}')
        if any(is_phone(b) for b in val_bags):
            raise ValueError('phone_* frames are training-only (another camera)')
        val = [im for b in val_bags for im in bags[b]]
        train = [im for b in bags if b not in val_bags for im in bags[b]]
        return sorted(train), sorted(val)
    pos = {b: sum(1 for im in ims if is_pos(im)) for b, ims in bags.items()}
    phone = [im for b in bags if is_phone(b) for im in bags[b]]     # always train
    curtain = sorted(b for b in bags if pos[b] and not is_phone(b))
    negonly = sorted(b for b in bags if not pos[b] and not is_phone(b))
    rnd = random.Random(seed)
    rnd.shuffle(curtain)
    rnd.shuffle(negonly)
    if len(curtain) < 2:                  # nothing to hold out whole: split in time
        train, val = [], []
        for b in curtain:
            ims = sorted(bags[b])
            cut = int(len(ims) * (1 - val_frac))
            train += ims[:cut]
            val += ims[cut:]
        train += [im for b in negonly for im in bags[b]] + phone
        return sorted(train), sorted(val)
    total_pos = sum(pos[b] for b in curtain + negonly)
    total_neg = sum(len(bags[b]) for b in curtain + negonly) - total_pos
    val_b = []
    for b in curtain[:-1]:                # the last one always trains
        if sum(pos[v] for v in val_b) >= val_frac * total_pos:
            break
        val_b.append(b)
    for k, b in enumerate(negonly[:-1]):  # with 2+, one always validates (false positives)
        neg_in_val = sum(len(bags[v]) - pos[v] for v in val_b)
        if k and neg_in_val + len(bags[b]) > val_frac * total_neg:
            break
        val_b.append(b)
    return split_by_bag(images, val_frac, val_bags=val_b)


def is_phone(bag):
    """Frames from phone videos (tools: maze_examples/curtains, `phone_<video>`): hand-held,
    portrait, a different camera and height from the robot's. Training only - never val or
    test, a score on them says nothing about the robot camera."""
    return bag.startswith('phone_')


def leave_one_bag_out(images, is_pos=None, test_bags=()):
    """[(bag, train, val)]: one fold per robot bag with a curtain, that bag alone as val."""
    is_pos = is_pos or (lambda im: True)
    bags = group_by_bag(images)
    folds = []
    for b in sorted(bags):
        if b in test_bags or is_phone(b) or not any(is_pos(im) for im in bags[b]):
            continue
        train, val = split_by_bag(images, 0, val_bags=[b])
        folds.append((b, [im for im in train if bag_of(im) not in test_bags], val))
    return folds


def has_curtain(root, im):
    lab = os.path.join(root, 'labels', os.path.splitext(os.path.basename(im))[0] + '.txt')
    return os.path.exists(lab) and os.path.getsize(lab) > 0


def count_labels(root, images):
    return sum(1 for im in images if has_curtain(root, im))


def bag_list(s):
    return [b for b in (s or '').split(',') if b]


def write_split(root, tag, train, val, test):
    """<tag>_train.txt / _val.txt (/ _test.txt) and <tag>.yaml; prints what is where."""
    for name, lst in (('train', train), ('val', val), ('test', test)):
        if name == 'test' and not lst:
            continue
        with open(os.path.join(root, f'{tag}_{name}.txt'), 'w') as f:
            f.write('\n'.join(lst) + '\n')
        bags = sorted({bag_of(i) for i in lst})
        print(f'{tag} {name}: {len(lst)} images, {count_labels(root, lst)} with a curtain, '
              f'bags {", ".join(bags)}')
    if not count_labels(root, val):
        raise SystemExit(f'{tag}: no curtain in val - mAP / recall would be meaningless; '
                         'pick val bags with --val-bags')
    data = os.path.join(root, f'{tag}.yaml')
    with open(data, 'w') as f:
        f.write(f'path: {root}\ntrain: {tag}_train.txt\nval: {tag}_val.txt\n' +
                (f'test: {tag}_test.txt\n' if test else '') + 'names:\n  0: curtain\n')
    return data


def train_one(a, root, data, name):
    os.environ.setdefault('YOLO_OFFLINE', '1')
    from ultralytics import YOLO
    base = a.base if os.sep in a.base else os.path.join(CACHE, a.base)
    model = YOLO(base if os.path.exists(base) else a.base)
    return model.train(data=data, epochs=a.epochs, imgsz=a.imgsz, device=a.device,
                       project=os.path.join(root, 'runs'), name=name, exist_ok=True,
                       fliplr=0.5, mosaic=1.0, patience=20)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset')
    ap.add_argument('--epochs', type=int, default=80)
    ap.add_argument('--imgsz', type=int, default=416)
    ap.add_argument('--base', default='yolo11n.pt', help='start weights (bare name -> cache)')
    ap.add_argument('--val', type=float, default=0.2, help='automatic split: share of curtain '
                    '(and negative) frames for val, whole bags')
    ap.add_argument('--val-bags', help='comma-separated bags for val (instead of --val)')
    ap.add_argument('--test-bags', help='comma-separated bags kept out of train AND val '
                    '(acceptance test set; written to <tag>_test.txt)')
    ap.add_argument('--cv', action='store_true', help='leave-one-bag-out: one model per '
                    'curtain bag, that bag as val; summary in runs/cv_summary.csv')
    ap.add_argument('--dry-run', action='store_true', help='write and print the split only')
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--install', action='store_true')
    a = ap.parse_args()
    root = os.path.abspath(os.path.expanduser(a.dataset))
    img_dir = os.path.join(root, 'images')
    images = sorted(os.path.join(img_dir, f) for f in os.listdir(img_dir)
                    if f.lower().endswith(('.jpg', '.jpeg', '.png')))
    if not images:
        raise SystemExit(f'no images in {img_dir}')
    known = set(group_by_bag(images))
    test_bags = bag_list(a.test_bags)
    for b in test_bags + bag_list(a.val_bags):
        if b not in known:
            raise SystemExit(f'unknown bag {b}; bags: {", ".join(sorted(known))}')
    test = [im for im in images if bag_of(im) in test_bags]
    pool = [im for im in images if bag_of(im) not in test_bags]
    pos = lambda im: has_curtain(root, im)  # noqa: E731

    if a.cv:
        folds = leave_one_bag_out(pool, pos)
        if len(folds) < 2:
            raise SystemExit('--cv needs at least 2 bags with a curtain')
        rows = []
        for bag, train, val in folds:
            data = write_split(root, f'cv_{bag}', train, val, test)
            if a.dry_run:
                continue
            res = train_one(a, root, data, f'cv_{bag}')
            r = getattr(res, 'results_dict', {}) or {}
            rows.append((bag, len(val), count_labels(root, val),
                         *(round(float(r.get(k, float('nan'))), 3) for k in
                           ('metrics/precision(B)', 'metrics/recall(B)', 'metrics/mAP50(B)'))))
        if rows:
            out = os.path.join(root, 'runs', 'cv_summary.csv')
            with open(out, 'w') as f:
                f.write('val_bag,images,with_curtain,precision,recall,mAP50\n')
                f.writelines(','.join(map(str, row)) + '\n' for row in rows)
            for row in rows:
                print('{}: {} images ({} curtain) P {} R {} mAP50 {}'.format(*row))
            print(f'summary: {out} (models: runs/cv_<bag>/weights/best.pt, not installed)')
        return

    train, val = split_by_bag(pool, a.val, is_pos=pos, val_bags=bag_list(a.val_bags) or None)
    data = write_split(root, 'split', train, val, test)
    if a.dry_run:
        return
    res = train_one(a, root, data, 'curtain')
    best = os.path.join(str(res.save_dir), 'weights', 'best.pt')
    print(f'best weights: {best}')
    if a.install:
        os.makedirs(CACHE, exist_ok=True)
        dst = os.path.join(CACHE, 'curtain_yolo.pt')
        shutil.copy(best, dst)
        print(f'installed {dst}')
        from ultralytics import YOLO
        try:
            out = YOLO(dst).export(format='openvino', imgsz=a.imgsz)
            print(f'OpenVINO export: {out} (curtain_yolo_model:=curtain_yolo_openvino_model)')
        except Exception as e:          # openvino not installed on this machine
            print(f'OpenVINO export skipped ({e})')


if __name__ == '__main__':
    main()
