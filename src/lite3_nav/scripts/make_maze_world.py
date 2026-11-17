#!/usr/bin/env python3
"""Generate a Gazebo maze world for lite3_nav exploration tests (standard library only).

A seeded depth-first maze (every cell reachable, many dead ends) on a grid of 1.0 m cells with
0.1 m walls, so corridors are 0.9 m clear like the example maze. Extra openings add loops. The
start is cell (0, 0), centred at (0.5, 0.5); the exit is a gap in the outer wall at the cell
farthest (by path) from the start, onto open floor for the exit detector. A room wall surrounds
the maze room_margin metres out: the exit detector needs floor that SLAM has seen as free, and
lidar rays that hit nothing (open ground to the horizon) mark nothing free, so without the room
the floor outside the exit stays unknown (check C, 2026-11-16). The real lab has walls too.

  python3 make_maze_world.py --seed 7 --cols 7 --rows 6 --loops 2 -o worlds/maze_dead_ends.world
"""
import argparse
import random
from collections import deque

CELL, WALL, HEIGHT = 1.0, 0.1, 1.0


def carve(cols, rows, rng, branchiness_newest=.5):
    """Walls as sets: east[(c, r)] between (c, r) and (c+1, r); north[(c, r)] between (c, r) and (c, r+1)."""
    east = {(c, r) for c in range(cols - 1) for r in range(rows)}
    north = {(c, r) for c in range(cols) for r in range(rows - 1)}
    # Growing tree: mostly extend the newest cell (long corridors), sometimes branch from a
    # random one (side passages that end in dead ends).
    seen, active = {(0, 0)}, [(0, 0)]
    while active:
        i = len(active) - 1 if rng.random() < branchiness_newest else rng.randrange(len(active))
        c, r = active[i]
        options = [(c + dc, r + dr) for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))
                   if 0 <= c + dc < cols and 0 <= r + dr < rows and (c + dc, r + dr) not in seen]
        if not options:
            active.pop(i)
            continue
        n = rng.choice(options)
        open_between(east, north, (c, r), n)
        seen.add(n)
        active.append(n)
    return east, north


def open_between(east, north, a, b):
    (c1, r1), (c2, r2) = sorted([a, b])
    (east if r1 == r2 else north).discard((c1, r1))


def neighbours(cell, cols, rows, east, north):
    c, r = cell
    if c + 1 < cols and (c, r) not in east: yield (c + 1, r)
    if c > 0 and (c - 1, r) not in east: yield (c - 1, r)
    if r + 1 < rows and (c, r) not in north: yield (c, r + 1)
    if r > 0 and (c, r - 1) not in north: yield (c, r - 1)


def distances(cols, rows, east, north, start=(0, 0)):
    dist, queue = {start: 0}, deque([start])
    while queue:
        cell = queue.popleft()
        for n in neighbours(cell, cols, rows, east, north):
            if n not in dist:
                dist[n] = dist[cell] + 1
                queue.append(n)
    return dist


def add_loops(cols, rows, east, north, count, rng):
    """Remove interior walls whose two cells are far apart along the maze, making real loops."""
    candidates = sorted(east | north, key=lambda w: rng.random())
    added = 0
    for wall in candidates:
        if added == count:
            break
        group = east if wall in east else north
        c, r = wall
        other = (c + 1, r) if group is east else (c, r + 1)
        if distances(cols, rows, east, north, (c, r)).get(other, 0) >= 6:
            group.discard(wall)
            added += 1


def exit_wall(cols, rows, dist):
    """Outer-wall gap at the boundary cell farthest from the start: (cell, side)."""
    # Only on the far side (at least half the arena's diagonal from the start), so the open
    # floor outside is not in view from the start cell.
    half = ((cols * CELL) ** 2 + (rows * CELL) ** 2) ** .5 / 2
    border = [(c, r) for c in range(cols) for r in range(rows) if (c in (0, cols - 1) or r in (0, rows - 1))
              and ((c * CELL) ** 2 + (r * CELL) ** 2) ** .5 >= half]
    cell = max(border, key=lambda x: (dist[x], x))
    c, r = cell
    side = 'E' if c == cols - 1 else 'N' if r == rows - 1 else 'W' if c == 0 else 'S'
    return cell, side


def segments(cols, rows, east, north, exit_cell, exit_side):
    """Wall boxes (x, y, length_x, length_y), merging runs along each grid line."""
    horizontal = {}   # y index -> set of x cell indices with a wall along that line
    vertical = {}
    for c in range(cols):
        horizontal.setdefault(0, set()).add(c)
        horizontal.setdefault(rows, set()).add(c)
    for r in range(rows):
        vertical.setdefault(0, set()).add(r)
        vertical.setdefault(cols, set()).add(r)
    for c, r in east:
        vertical.setdefault(c + 1, set()).add(r)
    for c, r in north:
        horizontal.setdefault(r + 1, set()).add(c)
    c, r = exit_cell
    {'E': lambda: vertical[cols].discard(r), 'W': lambda: vertical[0].discard(r),
     'N': lambda: horizontal[rows].discard(c), 'S': lambda: horizontal[0].discard(c)}[exit_side]()
    boxes = []
    for line, cells in horizontal.items():
        for a, b in runs(cells):
            boxes.append(((a + b + 1) / 2 * CELL, line * CELL, (b - a + 1) * CELL + WALL, WALL))
    for line, cells in vertical.items():
        for a, b in runs(cells):
            boxes.append((line * CELL, (a + b + 1) / 2 * CELL, WALL, (b - a + 1) * CELL + WALL))
    return boxes


def runs(indices):
    indices = sorted(indices)
    out = []
    for i in indices:
        if out and out[-1][1] == i - 1:
            out[-1][1] = i
        else:
            out.append([i, i])
    return out


def room(cols, rows, margin):
    """Wall boxes of a room margin metres around the maze (none if margin is 0)."""
    if margin <= 0:
        return []
    x0, y0, x1, y1 = -margin, -margin, cols * CELL + margin, rows * CELL + margin
    w, h = x1 - x0 + WALL, y1 - y0 + WALL
    return [((x0 + x1) / 2, y0, w, WALL), ((x0 + x1) / 2, y1, w, WALL),
            (x0, (y0 + y1) / 2, WALL, h), (x1, (y0 + y1) / 2, WALL, h)]


def world(boxes, name):
    models = []
    for i, (x, y, sx, sy) in enumerate(boxes):
        geometry = f'<geometry><box><size>{sx:.3f} {sy:.3f} {HEIGHT:.3f}</size></box></geometry>'
        models.append(
            f"<model name='wall{i}'><static>true</static><pose>{x:.3f} {y:.3f} {HEIGHT / 2:.3f} 0 0 0</pose>"
            f"<link name='l'><collision name='c'>{geometry}</collision><visual name='v'>{geometry}"
            "<material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>"
            "</visual></link></model>")
    return ("<?xml version='1.0'?><sdf version='1.6'><world name='" + name + "'>"
            "<include><uri>model://sun</uri></include><include><uri>model://ground_plane</uri></include>"
            + ''.join(models) + '</world></sdf>\n')


def ascii_map(cols, rows, east, north, exit_cell, exit_side, dist):
    lines = []
    for r in reversed(range(rows)):
        top = '+'
        for c in range(cols):
            gap = (r == rows - 1 and (c, r) == exit_cell and exit_side == 'N') or (r < rows - 1 and (c, r) not in north)
            top += ('  ' if gap else '--') + '+'
        lines.append(top)
        row = ' ' if (exit_side == 'W' and exit_cell == (0, r)) else '|'
        for c in range(cols):
            mark = 'S ' if (c, r) == (0, 0) else 'X ' if (c, r) == exit_cell else 'd ' if (
                c, r) != (0, 0) and len(list(neighbours((c, r), cols, rows, east, north))) == 1 else '  '
            wall = (c == cols - 1 and not (exit_side == 'E' and exit_cell == (c, r))) or (c < cols - 1 and (c, r) in east)
            row += mark + ('|' if wall else ' ')
        lines.append(row)
    lines.append('+' + ''.join(('  ' if exit_side == 'S' and exit_cell == (c, 0) else '--') + '+' for c in range(cols)))
    return '\n'.join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--seed', type=int, default=7)
    p.add_argument('--cols', type=int, default=7)
    p.add_argument('--rows', type=int, default=6)
    p.add_argument('--loops', type=int, default=2)
    p.add_argument('--room-margin', type=float, default=3.0,
                   help='room wall this far around the maze, metres (0: none)')
    p.add_argument('-o', '--output', required=True)
    a = p.parse_args()
    rng = random.Random(a.seed)
    east, north = carve(a.cols, a.rows, rng)
    add_loops(a.cols, a.rows, east, north, a.loops, rng)
    dist = distances(a.cols, a.rows, east, north)
    exit_cell, side = exit_wall(a.cols, a.rows, dist)
    with open(a.output, 'w') as f:
        f.write(world(segments(a.cols, a.rows, east, north, exit_cell, side)
                      + room(a.cols, a.rows, a.room_margin), 'maze_dead_ends'))
    dead = [c for c in dist if c != (0, 0) and len(list(neighbours(c, a.cols, a.rows, east, north))) == 1]
    print(ascii_map(a.cols, a.rows, east, north, exit_cell, side, dist))
    print(f'S start (0.5, 0.5)  X exit cell {exit_cell} side {side}  d dead end ({len(dead)})  '
          f'path to exit {dist[exit_cell]} cells')


if __name__ == '__main__':
    main()
