#!/usr/bin/env python3
"""Turn the example maze CAD file (STEP, mm) into axis-aligned boxes (JSON, metres).

    cd src/maze_nav
    python3 tools/step_to_boxes.py "tools/example maze.step" \
        > test/example_maze.json

No CAD library needed. The file is an assembly of B-rep solids; this walks the assembly
transforms, collects every vertex of each solid and writes its world bounding box. Solids that
aren't boxes (the four ramp wedges, the stair block, the ribbed / studded mats) are reduced
to a "kind" plus their bounding box; fake_maze_lite3.py knows how to draw each kind.
"""
import collections
import json
import re
import sys

import numpy as np

NUM = r'[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?'


def parse(path):
    data = open(path).read().split('DATA;', 1)[1]
    ents = {}
    for m in re.finditer(r'#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|ENDSEC)', data, re.S):
        ents[int(m.group(1))] = ' '.join(m.group(2).split())
    return ents


def main(path):
    ents = parse(path)
    refs = lambda i: [int(x) for x in re.findall(r'#(\d+)', ents[i])]  # noqa: E731
    typ = lambda i: re.match(r'\(?\s*([A-Z_0-9]+)', ents[i]).group(1)  # noqa: E731

    def vec(i):
        return np.array([float(x) for x in re.findall(NUM, ents[i].split(',', 1)[1])][:3])

    def placement(i):
        r = refs(i)
        o = vec(r[0])
        z = vec(r[1]) if len(r) > 1 else np.array([0, 0, 1.0])
        x = vec(r[2]) if len(r) > 2 else np.array([1.0, 0, 0])
        z /= np.linalg.norm(z)
        x -= z * x.dot(z)
        x /= np.linalg.norm(x)
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, o
        return T

    children = collections.defaultdict(list)
    for i, e in ents.items():
        if 'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION' in e:
            child, parent, tr = refs(i)[:3]
            a, b = refs(tr)[:2]
            children[parent].append((child, placement(b) @ np.linalg.inv(placement(a))))
        elif e.startswith('SHAPE_REPRESENTATION_RELATIONSHIP'):
            a, b = refs(i)[:2]
            shape, brep = (a, b) if typ(a) == 'SHAPE_REPRESENTATION' else (b, a)
            children[shape].append((brep, np.eye(4)))

    def vertices(i):
        stack, seen, out = [i], set(), []
        while stack:
            k = stack.pop()
            if k in seen:
                continue
            seen.add(k)
            if typ(k) == 'VERTEX_POINT':
                out.append(vec(refs(k)[0]))
            elif typ(k) != 'CARTESIAN_POINT':
                stack += refs(k)
        return np.array(out)

    solids = []

    def walk(node, T):
        if typ(node) == 'ADVANCED_BREP_SHAPE_REPRESENTATION':
            for item in refs(node)[:-1]:          # last ref is the context
                if typ(item) != 'MANIFOLD_SOLID_BREP':
                    continue
                v = vertices(item)
                w = (T @ np.c_[v, np.ones(len(v))].T).T[:, :3] / 1000.0
                top = w[w[:, 2] > w[:, 2].max() - 1e-4, :2].mean(0)   # where the top is (wedges)
                solids.append((w.min(0), w.max(0), len(np.unique(np.round(v), axis=0)), top))
            return
        for child, Tc in children[node]:
            walk(child, T @ Tc)

    roots = set(children) - {c for kids in children.values() for c, _ in kids}
    for root in roots:
        walk(root, np.eye(4))

    def kind(lo, hi, nverts):
        size = hi - lo
        if nverts == 6:
            return 'wedge'
        if size[2] > 0.3 and min(size[:2]) < 0.1:
            return 'wall' if size[2] < 0.5 else 'rail'
        if nverts == 8:
            return 'tile' if abs(size[2] - 0.1) < 1e-3 and lo[2] < 1e-3 else 'block'
        if size[2] > 0.3:
            return 'stairs'
        return 'mat'

    out = []
    for lo, hi, n, top in solids:
        s = {'kind': kind(lo, hi, n), 'min': np.round(lo, 4).tolist(),
             'max': np.round(hi, 4).tolist()}
        if s['kind'] == 'wedge':
            s['high_xy'] = np.round(top, 4).tolist()
        out.append(s)
    json.dump({'source': path.split('/')[-1], 'units': 'm', 'solids': out}, sys.stdout, indent=1)
    print()


if __name__ == '__main__':
    main(sys.argv[1])
