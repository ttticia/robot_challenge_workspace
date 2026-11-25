"""Which job next (no ROS in here).

The robot stands in a visited cell. `kinds[(cell, d)]` says what is on side d of each visited
cell (sense.py). Jobs, in this order:
  1. visit / look   the NEAREST unexplored thing: an open way into a cell not visited yet
               (plain or uneven terrain), or a room or an object cell not looked into yet
               (object cells are never entered). Nearest = fewest cells to walk, then
               looks before visits, then the way that looks most like a dead end (most
               walls already seen around the cell behind it), then least turning.
  2. greet     only when nothing in 1 is left: a curtain beside the cell the robot is in,
               not faced yet: face it squarely (first camera look)
  3. curtain / stairs   only when nothing in 1 is left: the nearest one not crossed yet
  4. exit      nothing left at all: go to the last cell (the one with the way out) and stop
  5. done      no exit known either: stop
"""
from collections import deque

from maze_explore.grid import nb

WALKABLE = ('free',)


def routes(kinds, visited, closed, links, start):
    """Fewest-cells routes from `start` to every visited cell: {cell: [(d, how, dest), ...]}.
    how = 'walk', or 'curtain' / 'stairs' for a crossing made before."""
    out = {start: []}
    queue = deque([start])
    while queue:
        c = queue.popleft()
        for d in range(4):
            if (c, d) in links:
                how, dest = links[(c, d)]
            elif kinds.get((c, d)) in WALKABLE:
                how, dest = 'walk', nb(c, d)
            else:
                continue
            if dest in visited and dest not in closed and dest not in out:
                out[dest] = out[c] + [(d, how, dest)]
                queue.append(dest)
    return out


def turns(path, facing, final_d):
    ds = [facing] + [s[0] for s in path] + [final_d]
    return sum(min((a - b) % 4, (b - a) % 4) for a, b in zip(ds, ds[1:]))


def next_job(kinds, visited, closed, links, cur, facing, looked, greeted, exit_cell,
             enter_rooms=False, walls_beyond=None):
    """Returns dict(job, path, cell, d). path = steps to walk first (see routes).
    walls_beyond[(cell, d)] = how many of the other three sides of the cell behind side d
    are walls already (0-3), for open ways into cells not visited yet."""
    walls_beyond = walls_beyond or {}
    reach = routes(kinds, visited, closed, links, cur)

    def nearest(want):
        best = None
        for c, path in reach.items():
            for d in range(4):
                job = want(c, d, kinds.get((c, d)))
                if job:
                    # At the same distance a look comes before a visit (2026-11-25, 11:41
                    # run): the robot left two cells without looking into the rooms beside
                    # them, because walking on needed less turning, and had to come back.
                    # Between two ways on at the same distance, the one that looks like a
                    # dead end goes first (user, 2026-11-25): the short branch is finished
                    # while the robot stands next to it, and it never has to come back.
                    cost = (len(path), job == 'visit', -walls_beyond.get((c, d), 0),
                            turns(path, facing, d))
                    if best is None or cost < best[0]:
                        best = (cost, dict(job=job, path=path, cell=c, d=d))
        return best and best[1]

    def explore(c, d, k):
        b = nb(c, d)
        if k == 'free' or (k == 'room' and enter_rooms):
            return 'visit' if b not in visited and b not in closed else None
        if k in ('room', 'object'):
            return 'look' if (c, d) not in looked else None

    def deferred(c, d, k):
        return k if k in ('curtain', 'stairs') and (c, d) not in links else None

    job = nearest(explore)
    if job:
        return job
    # Everything reachable is explored. Curtains and stairs come last (user, 2026-11-25), the
    # first look at a curtain too: it used to be made as soon as the robot stood beside one.
    for d in range(4):
        if kinds.get((cur, d)) == 'curtain' and (cur, d) not in greeted and (cur, d) not in links:
            return dict(job='greet', path=[], cell=cur, d=d)
    job = nearest(deferred)
    if job:
        return job
    if exit_cell is not None and exit_cell in reach:
        return dict(job='exit', path=reach[exit_cell], cell=exit_cell, d=facing)
    return dict(job='done', path=[], cell=cur, d=facing)
