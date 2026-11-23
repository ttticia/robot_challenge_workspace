"""Write test/challenge_*.json: sim arenas shaped like the competition arena.

    python3 test/make_challenge_mazes.py           # rewrites every challenge_*.json

From the challenge booklet (v5, sections C, E, F; user 2026-11-22): 5 x 3 cells, adjacent cell
centres 1.02 m apart; origin at the start cell's centre, +x the start heading, +y to its
left; the two arenas are mirrors of each other (the maze to the left or to the right of the
start); 4 uneven-terrain cells, one staircase negotiated as part of the route, one hanging
curtain (passable, mounted ~1 m up), rooms with survivors (QR codes on their side walls), an
object of interest and a hazard; the final cell is the exit / drone launch cell, with its
outer wall towards the target left out (a plain opening, never stairs or a curtain). The
layout changes on competition morning, so the interiors below are made up, and so are all
heights (terrain <= 12 cm), except the rolling pin in challenge_a (18 cm, user 2026-11-22)
and what maze_info.md gives for the practice arena (2026-11-23): walls 0.53 m, pyramid stairs
of 3 levels (15 cm rise, 20 cm tread, top 0.45 m) between railings, toy sets taller than the
walls at the back of the rooms.

challenge_a.json (maze to the left), exit +x of the final cell (4, 2):

    j=2   (0,2)-(1,2)   (2,2)={3,2}=(4,2)> exit      { } stairs (3, 2), along x
            |             |                          : curtain between (1, 1) and (2, 1)
    j=1   (0,1)-(1,1) : (2,1)   (3,1) (4,1)
                  |             |     |
    j=0   START-(1,0)---(2,0)---(3,0)-(4,0)
    terrain: (2,0) rolling pin (18 cm, across x: stepped over), (4,0) block field,
             (2,1) ribbed mat, (2,2) low block field
    rooms: (0,2), (1,2) survivors; (3,1) object (free-standing box); (4,1) hazard (cabinet)

challenge_b.json (maze to the left), exit +y of the final cell (2, 2):

    j=2   (0,2)-(1,2)   (2,2)-(3,2)-(4,2)            { } stairs (2, 0), along x
            :     |       ^ exit      |              : curtain between (0, 2) and (0, 1)
    j=1   (0,1) (1,1)-(2,1) (3,1)   (4,1)
                  |         |       |
    j=0   START-(1,0)={2,0}=(3,0)---(4,0)
    terrain: (4,0) block field, (4,1) ramp, (3,2) slats, (1,1) ribbed mat
    rooms: (0,1) survivor behind the curtain, (2,1) survivor, (3,1) object

challenge_a_mirror.json   challenge_a mirrored (maze to the right of the start)
challenge_a_no_curtain.json   challenge_a without the curtain (its passage open)
challenge_a_flat.json     challenge_a without the curtain or the stairs (cell (3, 2) flat): the
                          exit and launch cell without the two features the sim can't cross yet
challenge_b_offset.json   challenge_b with the robot placed 5 cm forward, 4 cm right and
                          5 deg left of the start cell's centre and heading (by hand)

`grid` in each file: cols, rows, cell_size, side, launch_cell (the final cell, where the
robot must park facing the exit) and exit_cell; `features`: where the terrain, stairs,
curtain and rooms are (cell indices, for tests and run logs).
"""
import copy
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CELL = 1.02
COLS, ROWS = 5, 3
TOP = 0.53                                 # wall height (maze_info.md, practice arena)
T = 0.05                                   # wall thickness
H = CELL / 2


def c(i, j):
    return i * CELL, j * CELL


def side_wall(cell, d):
    """Wall box on side d = (di, dj) of cell, overlapping its neighbours at the corners."""
    cx, cy = c(*cell)
    if d[0]:
        x = cx + d[0] * H
        return {'kind': 'wall', 'min': [x - T / 2, cy - H - T / 2, 0.0],
                'max': [x + T / 2, cy + H + T / 2, TOP]}
    y = cy + d[1] * H
    return {'kind': 'wall', 'min': [cx - H - T / 2, y - T / 2, 0.0],
            'max': [cx + H + T / 2, y + T / 2, TOP]}


def walls(passages, exit_side):
    out = []
    for i in range(COLS):
        for j in range(ROWS):
            for d in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                n = (i + d[0], j + d[1])
                inside = 0 <= n[0] < COLS and 0 <= n[1] < ROWS
                if inside and (d[0] < 0 or d[1] < 0):
                    continue                      # each interior side once
                if inside and (((i, j), n) in passages or (n, (i, j)) in passages):
                    continue
                if not inside and ((i, j), d) == exit_side:
                    continue
                out.append(side_wall((i, j), d))
    return out


def box(kind, x0, y0, x1, y1, z0, z1, **extra):
    return dict({'kind': kind, 'min': [round(x0, 4), round(y0, 4), z0],
                 'max': [round(x1, 4), round(y1, 4), z1]}, **extra)


# ------------------------------------------------------------------ cell contents

def slats(cell, along, pitch=0.2, width=0.04, height=0.06):
    """Slats across the cell, running along x or y (the robot crosses them at right angles)."""
    cx, cy = c(*cell)
    out = []
    for k in np.arange(-0.4, 0.41, pitch):
        if along == 'y':
            out.append(box('block', cx + k - width / 2, cy - 0.45, cx + k + width / 2,
                           cy + 0.45, 0.0, height))
        else:
            out.append(box('block', cx - 0.45, cy + k - width / 2, cx + 0.45,
                           cy + k + width / 2, 0.0, height))
    return out


def rolling_pin(cell, across, top=0.18, depth=0.06):
    """The rolling pin (user 2026-11-22): ~18 cm tall, in the middle of the cell, wall to wall,
    running along x or y (the robot crosses it at right angles). fake_maze_lite3's "pin" kind:
    seen by the sensors, stepped over by the body (H-Step)."""
    cx, cy = c(*cell)
    r = H - T / 2                                    # the walls' inner faces
    if across == 'y':
        return [box('pin', cx - depth / 2, cy - r, cx + depth / 2, cy + r, 0.0, top)]
    return [box('pin', cx - r, cy - depth / 2, cx + r, cy + depth / 2, 0.0, top)]


def block_field(cell, rng, size=0.2, heights=(0.03, 0.05, 0.07, 0.09, 0.12)):
    """Rubble: 0.2 m blocks of mixed heights over the cell (5 cm clear of the walls)."""
    cx, cy = c(*cell)
    out = []
    for x0 in np.arange(cx - 0.45, cx + 0.45 - 1e-9, size):
        for y0 in np.arange(cy - 0.45, cy + 0.45 - 1e-9, size):
            h = float(rng.choice(heights))
            out.append(box('block', x0, y0, min(x0 + size, cx + 0.45),
                           min(y0 + size, cy + 0.45), 0.0, h))
    return out


def mat(cell, height=0.03):
    cx, cy = c(*cell)
    return [box('mat', cx - 0.45, cy - 0.45, cx + 0.45, cy + 0.45, 0.0, height)]


def ramp(cell, axis, height=0.10):
    """Up to `height` at the cell centre and down again, along x or y."""
    cx, cy = c(*cell)
    if axis == 'x':
        return [box('wedge', cx - 0.45, cy - 0.45, cx, cy + 0.45, 0.0, height,
                    high_xy=[round(cx, 4), round(cy, 4)]),
                box('wedge', cx, cy - 0.45, cx + 0.45, cy + 0.45, 0.0, height,
                    high_xy=[round(cx, 4), round(cy, 4)])]
    return [box('wedge', cx - 0.45, cy - 0.45, cx + 0.45, cy, 0.0, height,
                high_xy=[round(cx, 4), round(cy, 4)]),
            box('wedge', cx - 0.45, cy, cx + 0.45, cy + 0.45, 0.0, height,
                high_xy=[round(cx, 4), round(cy, 4)])]


def stairs(cell, rise=0.15, tread=0.20, levels=3, rail_top=0.9):
    """Pyramid stairs along x (maze_info.md): `levels` steps of `rise` up to a top tread,
    and down again, every tread `tread` wide (3 levels: 1.0 m long, top 0.45 m), between
    two railings ~0.9 m tall just inside the cell's side edges (open truss frames on the
    real stairs; solid here)."""
    cx, cy = c(*cell)
    top = rise * levels
    half = (2 * levels - 1) * tread / 2
    run = levels * tread
    # the up flight carries the top tread; the down flight starts one level lower (no overlap)
    return [box('steps', cx - half, cy - 0.45, cx - half + run, cy + 0.45, 0.0, top,
                down='-x', treads=levels),
            box('steps', cx - half + run, cy - 0.45, cx + half, cy + 0.45, 0.0,
                top - rise, down='+x', treads=levels - 1),
            box('rail', cx - half, cy - 0.47, cx + half, cy - 0.45, 0.0, rail_top),
            box('rail', cx - half, cy + 0.45, cx + half, cy + 0.47, 0.0, rail_top)]


def curtain(a, b, gap=0.13, top=1.0):
    """Hanging curtain on the side between cells a and b (gap at the bottom, mounted ~1 m
    up); the robot walks through it from a to b."""
    (ax, ay), (bx, by) = c(*a), c(*b)
    mx, my = (ax + bx) / 2, (ay + by) / 2
    if a[1] == b[1]:
        return box('curtain', mx - 0.01, my - 0.49, mx + 0.01, my + 0.49, gap, top,
                   travel=[float(np.sign(bx - ax)), 0.0])
    return box('curtain', mx - 0.49, my - 0.01, mx + 0.49, my + 0.01, gap, top,
               travel=[0.0, float(np.sign(by - ay))])


def object_box(cell, back, size=0.3, height=1.0):
    """A free-standing prop (toy kitchen, shop counter) 0.15 m off the room's back wall,
    taller than the walls like the practice arena's toy sets (~0.8-1.2 m)."""
    cx, cy = c(*cell)
    ox, oy = cx + back[0] * (H - 0.15 - size / 2), cy + back[1] * (H - 0.15 - size / 2)
    return box('wall', ox - size / 2, oy - size / 2, ox + size / 2, oy + size / 2, 0.0, height)


def cabinet(cell, back, depth=0.25, width=0.4, height=0.6):
    """A cabinet against the room's back wall (hazard sticker on it)."""
    cx, cy = c(*cell)
    if back[0]:
        x = cx + back[0] * (H - T / 2)
        return box('wall', min(x, x - back[0] * depth), cy - width / 2,
                   max(x, x - back[0] * depth), cy + width / 2, 0.0, height)
    y = cy + back[1] * (H - T / 2)
    return box('wall', cx - width / 2, min(y, y - back[1] * depth), cx + width / 2,
               max(y, y - back[1] * depth), 0.0, height)


# ------------------------------------------------------------------ arenas

def out_line(exit_side):
    """fake_maze_lite3's "out of the maze" test: 0.4 m past the opened outer side."""
    (i, j), d = exit_side
    if d[0]:
        return {'axis': 'x', 'above' if d[0] > 0 else 'below': round(i * CELL + d[0] * (H + 0.4), 3)}
    return {'axis': 'y', 'above' if d[1] > 0 else 'below': round(j * CELL + d[1] * (H + 0.4), 3)}


def arena(name, passages, exit_side, solids, features, seed_note):
    (i, j), d = exit_side
    m = 2.5                                        # lab floor around the arena
    return {'source': f'make_challenge_mazes.py: {name} (booklet-shaped 5 x 3 arena; '
                      f'layout and heights made up{seed_note})',
            'units': 'm',
            'room': [round(-H - m, 3), round((COLS - 0.5) * CELL + m, 3),
                     round(-H - m, 3), round((ROWS - 0.5) * CELL + m, 3)],
            'start': [0.0, 0.0, 0.0],
            'exit': out_line(exit_side),
            'grid': {'cols': COLS, 'rows': ROWS, 'cell_size': CELL, 'side': 'left',
                     'launch_cell': [i, j], 'exit_cell': [i + d[0], j + d[1]]},
            'features': features,
            'solids': walls(passages, exit_side) + solids}


def challenge_a():
    rng = np.random.default_rng(11)
    passages = {((0, 0), (1, 0)), ((1, 0), (2, 0)), ((2, 0), (3, 0)), ((3, 0), (4, 0)),
                ((4, 0), (4, 1)), ((3, 0), (3, 1)), ((1, 0), (1, 1)), ((1, 1), (0, 1)),
                ((0, 1), (0, 2)), ((0, 2), (1, 2)), ((1, 1), (2, 1)), ((2, 1), (2, 2)),
                ((2, 2), (3, 2)), ((3, 2), (4, 2))}
    solids = (rolling_pin((2, 0), 'y') + block_field((4, 0), rng) + mat((2, 1))
              + block_field((2, 2), rng, heights=(0.03, 0.05)) + stairs((3, 2))
              + [curtain((1, 1), (2, 1)), object_box((3, 1), (0, 1)),
                 cabinet((4, 1), (0, 1))])
    features = {'terrain': [[2, 0], [4, 0], [2, 1], [2, 2]], 'stairs': [3, 2],
                'curtain': [[1, 1], [2, 1]], 'rolling_pin': [2, 0],
                'rooms': {'survivor': [[0, 2], [1, 2]], 'object': [3, 1], 'hazard': [4, 1]}}
    return arena('challenge_a', passages, ((4, 2), (1, 0)), solids, features, '')


def challenge_b():
    rng = np.random.default_rng(23)
    passages = {((0, 0), (1, 0)), ((1, 0), (2, 0)), ((2, 0), (3, 0)), ((3, 0), (4, 0)),
                ((4, 0), (4, 1)), ((4, 1), (4, 2)), ((4, 2), (3, 2)), ((3, 2), (2, 2)),
                ((3, 0), (3, 1)), ((1, 0), (1, 1)), ((1, 1), (2, 1)), ((1, 1), (1, 2)),
                ((1, 2), (0, 2)), ((0, 2), (0, 1))}
    solids = (block_field((4, 0), rng) + ramp((4, 1), 'y') + slats((3, 2), 'y') + mat((1, 1))
              + stairs((2, 0))
              + [curtain((0, 2), (0, 1)), object_box((3, 1), (0, 1))])
    features = {'terrain': [[4, 0], [4, 1], [3, 2], [1, 1]], 'stairs': [2, 0],
                'curtain': [[0, 2], [0, 1]],
                'rooms': {'survivor': [[0, 1], [2, 1]], 'object': [3, 1]}}
    return arena('challenge_b', passages, ((2, 2), (0, 1)), solids, features, '')


def mirrored(a):
    """The other arena: everything reflected in the start heading (y -> -y)."""
    m = copy.deepcopy(a)
    m['source'] = a['source'].replace('challenge_a', 'challenge_a_mirror')
    for s in m['solids']:
        lo, hi = s['min'], s['max']
        lo[1], hi[1] = -hi[1], -lo[1]
        if 'high_xy' in s:
            s['high_xy'][1] = -s['high_xy'][1]
        if 'travel' in s:
            s['travel'][1] = -s['travel'][1]
    r = m['room']
    m['room'] = [r[0], r[1], -r[3], -r[2]]
    e = m['exit']
    if e['axis'] == 'y':
        k, v = next((k, v) for k, v in e.items() if k != 'axis')
        m['exit'] = {'axis': 'y', 'below' if k == 'above' else 'above': -v}
    g = m['grid']
    g['side'] = 'right'
    g['launch_cell'][1] = -g['launch_cell'][1]
    g['exit_cell'][1] = -g['exit_cell'][1]

    def flip(v):
        if isinstance(v, list) and len(v) == 2 and all(isinstance(x, int) for x in v):
            return [v[0], -v[1]]
        if isinstance(v, list):
            return [flip(x) for x in v]
        if isinstance(v, dict):
            return {k: flip(x) for k, x in v.items()}
        return v
    m['features'] = flip(m['features'])
    return m


def offset(a, start):
    m = copy.deepcopy(a)
    m['source'] = a['source'].replace('challenge_b', 'challenge_b_offset') + \
        '; robot placed off the start cell centre / heading'
    m['start'] = [round(v, 6) for v in start]
    return m


def without(a, name, features):
    """challenge_a with features left out ('curtain': its passage stays open; 'stairs': the
    cell is flat floor), to test the rest (exit, launch cell) without them."""
    m = copy.deepcopy(a)
    m['source'] = a['source'].replace('challenge_a', name) + \
        '; without ' + ' or '.join(features)
    drop = []
    if 'curtain' in features:
        drop += [s for s in m['solids'] if s['kind'] == 'curtain']
    if 'stairs' in features:
        cx, cy = c(*m['features']['stairs'])
        drop += [s for s in m['solids'] if s['kind'] in ('steps', 'tile', 'rail')
                 and abs((s['min'][0] + s['max'][0]) / 2 - cx) < H
                 and abs((s['min'][1] + s['max'][1]) / 2 - cy) < H]
    m['solids'] = [s for s in m['solids'] if not any(s is d for d in drop)]
    for f in features:
        del m['features'][f]
    return m


def mazes():
    a, b = challenge_a(), challenge_b()
    return {'challenge_a.json': a, 'challenge_a_mirror.json': mirrored(a),
            'challenge_b.json': b,
            'challenge_b_offset.json': offset(b, (0.05, -0.04, math.radians(5.0))),
            'challenge_a_no_curtain.json': without(a, 'challenge_a_no_curtain', ['curtain']),
            'challenge_a_flat.json': without(a, 'challenge_a_flat', ['curtain', 'stairs'])}


def main():
    for name, m in mazes().items():
        with open(os.path.join(HERE, name), 'w') as f:
            json.dump(m, f, indent=1)
            f.write('\n')
        g = m['grid']
        print(f'wrote {name}: {len(m["solids"])} solids, maze to the {g["side"]}, launch cell '
              f'{g["launch_cell"]}, exit cell {g["exit_cell"]}')


if __name__ == '__main__':
    main()
