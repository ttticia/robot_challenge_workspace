"""Write test/maze_curtains.json and test/maze_stairs.json: the two presentation mazes
(resource/Presentation, 2026-11-21) for fake_maze_lite3.py.

    python3 test/make_presentation_mazes.py        # rewrites both files (fixed seed)

Both are 3 x 2 cells of 1.0 m (cell (i, j) spans x i-0.5..i+0.5, y j-0.5..j+0.5; +y is the
top of the simplified drawings). Taken from the photos; the heights are GUESSES (nothing was
measured): OSB cells sit on a 0.10 m plate, rooms and the curtain cell are bare lab floor
(z 0), OSB walls stand on the lab floor and reach 0.45 m above the plate (z 0..0.55), extru
walls are their real 0.45 m (z 0..0.45, on the lab floor).

maze_curtains.json (maze_curtains.jpg):
    start (0,1) plate, closed but for its bottom | curtain cell (1,1) lab floor, the curtain
    in the OUTER top wall = the only exit | room (2,1) lab floor, extru walls, toolbox
    (object) by its right wall. Bottom row: (0,0) raised box + ramp on the plate, (1,0)
    slats on the plate (along y, 0.2 m apart).
maze_stairs.json (maze_stairs.jpg):
    top row: normal cell (0,1) plate | block field (1,1) | slats (2,1, along x). Bottom row:
    stairs (0,0): a landing at 0.22 (one 12 cm step up from the plate) and two 11 cm steps
    DOWN past the maze's bottom edge to the lab floor = the exit | room (1,0) lab floor, open
    to the block field, toy shop counter (object) at its bottom | start (2,0) plate.
"""
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PLATE = 0.10
OSB_TOP = PLATE + 0.45
EXTRU_TOP = 0.45
T = 0.05                                  # wall thickness


def tile(i, j, z=PLATE):
    return {'kind': 'tile', 'min': [i - 0.5, j - 0.5, 0.0], 'max': [i + 0.5, j + 0.5, z]}


def hwall(x0, x1, y, top=OSB_TOP, style=None, board_side=None):
    """Wall along x at y (centred on the line)."""
    s = {'kind': 'wall', 'min': [x0, y - T / 2, 0.0], 'max': [x1, y + T / 2, top]}
    if style:
        s['style'] = style
    if board_side is not None:
        s['board_side'] = board_side
    return s


def vwall(x, y0, y1, top=OSB_TOP, style=None, board_side=None):
    """Wall along y at x."""
    s = {'kind': 'wall', 'min': [x - T / 2, y0, 0.0], 'max': [x + T / 2, y1, top]}
    if style:
        s['style'] = style
    if board_side is not None:
        s['board_side'] = board_side
    return s


def block(x0, y0, x1, y1, top):
    return {'kind': 'block', 'min': [x0, y0, PLATE], 'max': [x1, y1, top]}


def slats(i, j, along, pitch=0.2, width=0.04, height=0.06):
    """Slats on cell (i, j)'s plate, running along x or y, `pitch` m apart."""
    out = []
    for k in np.arange(-0.4, 0.41, pitch):
        if along == 'y':
            out.append(block(i + k - width / 2, j - 0.45, i + k + width / 2, j + 0.45,
                             PLATE + height))
        else:
            out.append(block(i - 0.45, j + k - width / 2, i + 0.45, j + k + width / 2,
                             PLATE + height))
    return out


def block_field(i, j, size=0.2, rng=None):
    """Blocks of mixed heights (3-12 cm above the plate) over cell (i, j)."""
    out = []
    for x0 in np.arange(i - 0.5, i + 0.5 - 1e-9, size):
        for y0 in np.arange(j - 0.5, j + 0.5 - 1e-9, size):
            h = float(rng.choice([0.03, 0.05, 0.07, 0.09, 0.12]))
            out.append(block(round(x0, 3), round(y0, 3), round(x0 + size, 3),
                             round(y0 + size, 3), round(PLATE + h, 3)))
    return out


def maze_curtains():
    s = [tile(0, 1), tile(0, 0), tile(1, 0)]
    # (0,0): a raised box with a ramp up to it from the start side (top).
    s.append(block(-0.35, -0.40, 0.25, 0.05, PLATE + 0.12))
    s.append({'kind': 'wedge', 'min': [-0.35, 0.05, PLATE], 'max': [0.25, 0.30, PLATE + 0.12],
              'high_xy': [-0.05, 0.05]})
    s += slats(1, 0, 'y')
    s += [hwall(-0.5, 0.5, 1.5),                      # start cell top
          vwall(-0.5, -0.5, 1.5),                     # left side
          hwall(-0.5, 1.5, -0.5),                     # bottom
          vwall(1.5, -0.5, 0.5),                      # slats cell right
          vwall(0.5, 0.5, 1.5),                       # start | curtain cell
          # room: extru walls on the lab floor (board facing in)
          hwall(1.5, 2.5, 1.5, EXTRU_TOP, 'extru', -1),
          vwall(2.5, 0.5, 1.5, EXTRU_TOP, 'extru', -1),
          hwall(1.5, 2.5, 0.5, EXTRU_TOP, 'extru', 1)]
    # The curtain: outer top wall of the curtain cell, 15 cm gap over the lab floor.
    s.append({'kind': 'curtain', 'min': [0.5, 1.49, 0.15], 'max': [1.5, 1.51, 0.85],
              'travel': [0.0, 1.0]})
    # Toolbox on a stand by the room's right wall (object, 0.3 x 0.6 m).
    s.append({'kind': 'wall', 'min': [2.05, 0.75, 0.0], 'max': [2.35, 1.35, 0.60]})
    return {'source': 'presentation maze_curtains.jpg (2026-11-21), make_presentation_mazes.py;'
                      ' heights guessed',
            'units': 'm', 'room': [-2.5, 4.5, -2.0, 4.0], 'start': [0.0, 1.0, -1.5707963267948966],
            'exit': {'axis': 'y', 'above': 2.0}, 'solids': s}


def maze_stairs(seed=7):
    rng = np.random.default_rng(seed)
    s = [tile(0, 1), tile(1, 1), tile(2, 1), tile(2, 0)]
    s += block_field(1, 1, rng=rng)
    s += slats(2, 1, 'x')
    # Stairs cell: landing 12 cm above the plate, then two 11 cm steps down past the
    # maze's bottom edge (one tread outside at 0.11, lab floor beyond).
    s.append(tile(0, 0, PLATE + 0.12))
    s.append({'kind': 'steps', 'min': [-0.5, -0.80, 0.0], 'max': [0.5, -0.5, 0.11],
              'down': '-y', 'treads': 1})
    s += [hwall(-0.5, 2.5, 1.5),                      # top
          vwall(-0.5, 0.5, 1.5),                      # normal cell left
          vwall(2.5, -0.5, 1.5),                      # right side
          hwall(1.5, 2.5, -0.5),                      # start bottom
          vwall(1.5, -0.5, 0.5),                      # room | start
          # hand rail on the stairs' left (the drawing's wall), down to the bottom step
          {'kind': 'rail', 'min': [-0.53, -0.80, 0.0], 'max': [-0.50, 0.5, 0.90]},
          # room: extru walls on the lab floor
          vwall(0.5, -0.5, 0.5, EXTRU_TOP, 'extru', 1),
          hwall(0.5, 1.5, -0.5, EXTRU_TOP, 'extru', 1)]
    # Toy shop counter at the room's bottom (object, 0.55 x 0.26 m), a hand's width off the
    # wall: an object touching a wall on the map is not found (scene_cells.cell_objects).
    s.append({'kind': 'wall', 'min': [0.75, -0.36, 0.0], 'max': [1.30, -0.10, 0.60]})
    return {'source': 'presentation maze_stairs.jpg (2026-11-21), make_presentation_mazes.py;'
                      ' heights guessed',
            'units': 'm', 'room': [-2.5, 4.5, -2.5, 3.5], 'start': [2.0, 0.0, 1.5707963267948966],
            'exit': {'axis': 'y', 'below': -0.9}, 'solids': s}


def main():
    for name, maze in (('maze_curtains.json', maze_curtains()), ('maze_stairs.json', maze_stairs())):
        with open(os.path.join(HERE, name), 'w') as f:
            json.dump(maze, f, indent=1)
            f.write('\n')
        print(f'wrote {name}: {len(maze["solids"])} solids')


if __name__ == '__main__':
    main()
