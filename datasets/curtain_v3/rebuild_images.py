#!/usr/bin/env python3
"""Restore curtain_v3's images/ after a clone (they are not in git: duplicates).

    python3 datasets/curtain_v3/rebuild_images.py          (from the repo root)

- copies the 717 original frames from `curtain CV/images/images/` into images/
- writes the left-right mirrored copy <bag>_mirror_<SSSS>_<cc>.jpg of every TRAINING frame
  (labels/ already holds the mirrored labels; val / test recordings are never mirrored)
- rewrites split_train.txt / split_val.txt / split_test.txt and split.yaml to this folder's
  path (they were written with the dev laptop's absolute paths)
Then train / evaluate as in agent.md (section 4).
"""
import os
import re
import shutil

import cv2

here = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(here, '..', '..', 'curtain CV', 'images', 'images')
img = os.path.join(here, 'images')
os.makedirs(img, exist_ok=True)

lists = {}
for name in ('train', 'val', 'test'):
    p = os.path.join(here, f'split_{name}.txt')
    lists[name] = [os.path.basename(ln.strip()) for ln in open(p) if ln.strip()]

copied = mirrored = 0
for f in lists['train'] + lists['val'] + lists['test']:
    m = re.match(r'(.+)_mirror_(\d+_\d{2})\.jpg$', f)
    if m:
        continue
    s = os.path.join(src, f)
    if not os.path.exists(s):
        raise SystemExit(f'missing original frame {s}')
    if not os.path.exists(os.path.join(img, f)):
        shutil.copy2(s, os.path.join(img, f))
        copied += 1
for f in lists['train']:
    m = re.match(r'(.+)_mirror_(\d+_\d{2})\.jpg$', f)
    if not m:
        continue
    out = os.path.join(img, f)
    if not os.path.exists(out):
        orig = cv2.imread(os.path.join(img, f'{m.group(1)}_{m.group(2)}.jpg'))
        cv2.imwrite(out, cv2.flip(orig, 1), [cv2.IMWRITE_JPEG_QUALITY, 95])
        mirrored += 1

for name, files in lists.items():
    with open(os.path.join(here, f'split_{name}.txt'), 'w') as fh:
        fh.write('\n'.join(os.path.join(img, f) for f in files) + '\n')
with open(os.path.join(here, 'split.yaml'), 'w') as fh:
    fh.write(f'path: {here}\ntrain: split_train.txt\nval: split_val.txt\n'
             'test: split_test.txt\nnames:\n  0: curtain\n')
print(f'{copied} frames copied, {mirrored} mirrored copies written; split lists now point at {img}')
