"""dedup_frames.py <T> [apply] (env PREFIX=<bag prefix> limits it to those frames): within each bag, keep a frame if it differs from the last kept frame
(64x48 normalised grey, mean abs diff) by >= T, or its label differs (pos/neg, box IoU < 0.5)."""
import os, re, sys, shutil, collections, cv2, numpy as np
T = float(sys.argv[1]); WINDOW = 3.0; MIN_GAP = float(os.environ.get('MIN_GAP', '1.0')); apply = len(sys.argv) > 2 and sys.argv[2] == 'apply'
bags = collections.defaultdict(list)
for f in sorted(os.listdir('images')):
    if not f.startswith(os.environ.get('PREFIX', '')):
        continue
    bags[re.match(r'(.+)_\d+_\d{2}\.jpg$', f).group(1)].append(f)
def thumb(f):
    g = cv2.cvtColor(cv2.resize(cv2.imread('images/' + f), (24, 18), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY).astype(np.float32)
    return (g - g.mean()) / (g.std() + 1e-6)
def boxes(f):
    p = 'labels/' + f[:-4] + '.txt'
    return [list(map(float, l.split()[1:])) for l in open(p)] if os.path.exists(p) else []
def iou(a, b):
    ax0, ay0, ax1, ay1 = a[0]-a[2]/2, a[1]-a[3]/2, a[0]+a[2]/2, a[1]+a[3]/2
    bx0, by0, bx1, by1 = b[0]-b[2]/2, b[1]-b[3]/2, b[0]+b[2]/2, b[1]+b[3]/2
    i = max(0, min(ax1, bx1) - max(ax0, bx0)) * max(0, min(ay1, by1) - max(ay0, by0))
    return i / (a[2]*a[3] + b[2]*b[3] - i + 1e-9)
def label_changed(a, b):
    return bool(a) != bool(b)
def sdiff(a, b, r=2):
    h, w = a.shape
    return min(np.abs(a[max(0,dy):h+min(0,dy), max(0,dx):w+min(0,dx)] - b[max(0,-dy):h+min(0,-dy), max(0,-dx):w+min(0,-dx)]).mean()
               for dy in range(-r, r+1) for dx in range(-r, r+1))
tot = collections.Counter()
drop = []
for bag, fs in sorted(bags.items()):
    kept = []; k = p = n = 0     # (time s, thumb, boxes) of kept frames
    for f in fs:
        t, b = thumb(f), boxes(f)
        ts = float(f[len(bag) + 1:-4].replace('_', '.'))
        recent = [x for x in kept if ts - x[0] <= WINDOW]
        if all((sdiff(t, x[1]) >= T and ts - x[0] >= MIN_GAP) or label_changed(b, x[2]) for x in recent):
            kept.append((ts, t, b)); k += 1; p += bool(b); n += not b
        else:
            drop.append(f)
    print(f'{bag:28s} {len(fs):4d} -> {k:4d} kept ({p} curtain, {n} negative)')
    tot.update(kept=k, pos=p, neg=n, all=len(fs))
open(os.environ.get('KEEPLIST', '/dev/null'), 'w').write('\n'.join(sorted(set(os.listdir('images')) - set(drop))))
print(f"T={T}: {tot['all']} -> {tot['kept']} ({tot['pos']} curtain, {tot['neg']} negative), {len(drop)} removed")
if apply:
    os.makedirs('duplicates/labels', exist_ok=True)
    for f in drop:
        shutil.move('images/' + f, 'duplicates/' + f)
        l = 'labels/' + f[:-4] + '.txt'
        if os.path.exists(l): shutil.move(l, 'duplicates/labels/' + f[:-4] + '.txt')
    print('moved', len(drop), 'images (+ labels) to duplicates/')
