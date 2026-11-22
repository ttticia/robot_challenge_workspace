"""Write test/challenge_*.json: the challenge maze shape for fake_maze_lite3.py.

    python3 test/make_challenge_mazes.py           # rewrites every challenge_*.json

The challenge maze (user 2026-11-22): 5 x 3 cells of 1.02 m, start cell (0, 0) in a corner,
the robot facing +x along the 5-cell side. Cell (i, j) is centred on (1.02 i, 1.02 j); the
start cell's centre is the maze frame origin, so the maze frame matches the explorer's cell
grid (cell (0, 0) on the map origin, +i along the start heading). Walls 0.45 m tall on a flat
floor; the interior layout below is made up (nothing was measured).

Interior (passages; every other cell side is a wall):

    j=2   (0,2)-(1,2)   (2,2)-(3,2)---(4,2)
            |             |     |       |
    j=1   (0,1)-(1,1)---(2,1) (3,1)   (4,1)
                  |
    j=0   START-(1,0)---(2,0)---(3,0)---(4,0)
          i=0    i=1     i=2     i=3     i=4

Junctions at (1,0), (1,1) and (3,2); dead ends at (1,2), (3,1), (4,1) and (4,0). Each file
opens one outer side into `grid.exit_cell` (outside the grid). The robot must explore every
cell, then park in `grid.launch_cell` (the cell inside the exit, facing out) and never go out.
    challenge_exit_far.json    +x side of (4,1)  -> launch cell (4, 1), exit cell (5, 1)
    challenge_exit_left.json   +y side of (1,2)  -> launch cell (1, 2), exit cell (1, 3)
    challenge_exit_right.json  -y side of (4,0)  -> launch cell (4, 0), exit cell (4, -1)
    challenge_exit_near.json   -y side of (1,0)  -> launch cell (1, 0), exit cell (1, -1):
                               next to the start, passed early and left for last
    challenge_offset.json      exit_far with the robot started 5 cm forward, 4 cm right and
                               turned 5 deg left (a hand placement): the explorer's grid is
                               then off the real cells by that much
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CELL = 1.02
COLS, ROWS = 5, 3
TOP = 0.45                                # wall height
T = 0.05                                  # wall thickness
PASSAGES = {((0, 0), (1, 0)), ((1, 0), (2, 0)), ((2, 0), (3, 0)), ((3, 0), (4, 0)),
            ((1, 0), (1, 1)), ((0, 1), (1, 1)), ((0, 1), (0, 2)), ((0, 2), (1, 2)),
            ((1, 1), (2, 1)), ((2, 1), (2, 2)), ((2, 2), (3, 2)), ((3, 2), (4, 2)),
            ((4, 2), (4, 1)), ((3, 2), (3, 1))}
EXITS = {'far': ((4, 1), (1, 0)), 'left': ((1, 2), (0, 1)), 'right': ((4, 0), (0, -1)),
         'near': ((1, 0), (0, -1))}


def side_wall(cell, d):
    """Wall box on side d = (di, dj) of cell, overlapping its neighbours at the corners."""
    cx, cy = cell[0] * CELL, cell[1] * CELL
    h = CELL / 2
    if d[0]:
        x = cx + d[0] * h
        return {'kind': 'wall', 'min': [x - T / 2, cy - h - T / 2, 0.0],
                'max': [x + T / 2, cy + h + T / 2, TOP]}
    y = cy + d[1] * h
    return {'kind': 'wall', 'min': [cx - h - T / 2, y - T / 2, 0.0],
            'max': [cx + h + T / 2, y + T / 2, TOP]}


def walls(exit_side):
    out = []
    for i in range(COLS):
        for j in range(ROWS):
            for d in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                n = (i + d[0], j + d[1])
                inside = 0 <= n[0] < COLS and 0 <= n[1] < ROWS
                if inside and (d[0] < 0 or d[1] < 0):
                    continue                      # each interior side once
                if inside and (((i, j), n) in PASSAGES or (n, (i, j)) in PASSAGES):
                    continue
                if not inside and ((i, j), d) == exit_side:
                    continue
                out.append(side_wall((i, j), d))
    return out


def out_line(exit_side):
    """fake_maze_lite3's "out" test: 0.4 m past the opened outer side."""
    (i, j), d = exit_side
    if d[0]:
        v = (i * CELL + d[0] * (CELL / 2 + 0.4))
        return {'axis': 'x', 'above' if d[0] > 0 else 'below': round(v, 3)}
    v = (j * CELL + d[1] * (CELL / 2 + 0.4))
    return {'axis': 'y', 'above' if d[1] > 0 else 'below': round(v, 3)}


def maze(name, start=(0.0, 0.0, 0.0)):
    exit_side = EXITS[name]
    (i, j), d = exit_side
    m = 2.5                                       # lab floor around the maze
    return {'source': f'make_challenge_mazes.py: 5 x 3 challenge maze, exit {name}',
            'units': 'm',
            'room': [round(-CELL / 2 - m, 3), round((COLS - 0.5) * CELL + m, 3),
                     round(-CELL / 2 - m, 3), round((ROWS - 0.5) * CELL + m, 3)],
            'start': [round(v, 6) for v in start],
            'exit': out_line(exit_side),
            'grid': {'cols': COLS, 'rows': ROWS, 'cell_size': CELL,
                     'launch_cell': [i, j], 'exit_cell': [i + d[0], j + d[1]]},
            'solids': walls(exit_side)}


def main():
    files = {f'challenge_exit_{n}.json': maze(n) for n in EXITS}
    files['challenge_offset.json'] = maze('far', (0.05, -0.04, math.radians(5.0)))
    for name, m in files.items():
        with open(os.path.join(HERE, name), 'w') as f:
            json.dump(m, f, indent=1)
            f.write('\n')
        print(f'wrote {name}: {len(m["solids"])} walls, exit cell {m["grid"]["exit_cell"]}')


if __name__ == '__main__':
    main()
