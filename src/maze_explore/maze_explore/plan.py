"""Which job next (no ROS in here).

The robot stands in a visited cell. `kinds[(cell, d)]` says what is on side d of each visited
cell (sense.py). Jobs, in this order:
  1. visit / look   the NEAREST unexplored thing: an open way into a cell not visited yet
               (plain or uneven terrain), or a room or an object cell not looked into yet
               (object cells are never entered). Nearest = fewest cells to walk, then
               looks before visits, then the way that looks most like a dead end (most
               walls already seen around the cell behind it), then least turning.
  2. greet     only when nothing in 1 is left, and only the curtain that 3 would cross
               next, from the cell beside it: face it squarely (first camera look)
  3. stairs / curtain   only when nothing in 1 is left: the nearest one not crossed yet;
               at the same distance the stairs come before the curtain
  4. exit      nothing left at all: go to the last cell (the one with the way out) and stop
  5. done      no exit known either: stop

Junction rules (user, 2026-11-26). At one junction the order is room, path, stairs, curtain
(the job order above at equal distance). A job beside the cell the robot stands in always
beats one it has to walk back through explored cells for, and among those the fewest cells
re-crossed wins (distance is the first key). A room or object cell that has been looked at
is explored: it is not looked at again from another side. An open edge into a cell the
robot has travelled through (visited, or closed like the start cell) is never checked.
Between several ways on at one junction: one that is already seen to be a dead end (walls
on its three other sides) first; then the one furthest from the far row (lowest row along
the start heading; the end cell is past the far row); then most walls seen; least turning.
"""
from collections import deque

from maze_explore.grid import nb

WALKABLE = ('free',)
# the hierarchy at one junction (user, 2026-11-26): rooms, paths, stairs, curtains
RANK = {'look': 0, 'visit': 1, 'stairs': 2, 'curtain': 3}


def routes(kinds, visited, closed, links, start):
    """Fewest-cells routes from `start` to every visited cell: {cell: [(d, how, dest), ...]}.
    how = 'walk', or 'curtain' for a crossing made before. A stairs crossing is never part
    of a route (user, 2026-11-26 night: the stairs are crossed once, never again)."""
    out = {start: []}
    queue = deque([start])
    while queue:
        c = queue.popleft()
        for d in range(4):
            if (c, d) in links:
                how, dest = links[(c, d)]
                if how == 'stairs':
                    continue          # crossed once: not walked back over
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
    looked_cells = {nb(c, d) for c, d in looked}      # looked at once = explored

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
                    # Between ways on that are NOT dead ends, the one furthest from the far
                    # row goes first (user, 2026-11-26): the far row is where the end cell
                    # is, so the robot works towards it and ends the exploring near the
                    # way out. Row = the cell's first index (along the start heading).
                    walls = walls_beyond.get((c, d), 0)
                    row = nb(c, d)[0] if job == 'visit' else 0
                    cost = (len(path), RANK[job], walls < 3, row, -walls,
                            turns(path, facing, d))
                    if best is None or cost < best[0]:
                        best = (cost, dict(job=job, path=path, cell=c, d=d))
        return best and best[1]

    def explore(c, d, k):
        b = nb(c, d)
        if k == 'free' or (k == 'room' and enter_rooms):
            return 'visit' if b not in visited and b not in closed else None
        if k in ('room', 'object'):
            if b in visited or b in closed or b in looked_cells:
                return None                   # travelled through, or looked at already
            return 'look' if (c, d) not in looked else None

    # The maze has one stairs. Once they have been crossed (a stairs link exists) nothing
    # else is taken for stairs to cross (user, 2026-11-26 night).
    stairs_done = any(how == 'stairs' for how, _ in links.values())

    def deferred(c, d, k):
        if k == 'stairs' and stairs_done:
            return None
        return k if k in ('curtain', 'stairs') and (c, d) not in links else None

    job = nearest(explore)
    if job:
        return job
    # Everything reachable is explored. Curtains and stairs come last (user, 2026-11-25), the
    # first look at a curtain too: it used to be made as soon as the robot stood beside one.
    job = nearest(deferred)
    # A curtain is greeted (faced, first camera look) only when it is the next thing to be
    # crossed. Until 2026-11-26 every curtain beside the cell was greeted before anything
    # else in this stage, stairs included: in the 04:42 robot run the robot stood in the
    # start cell with confirmed stairs ahead, spent its time turning to two lidar
    # "curtains" at its sides (neither confirmed by the camera) and the odometry failed
    # before the stairs' turn came.
    if job and job['job'] == 'curtain' and not job['path'] \
            and (job['cell'], job['d']) not in greeted:
        return dict(job, job='greet')
    if job:
        return job
    if exit_cell is not None and exit_cell in reach:
        return dict(job='exit', path=reach[exit_cell], cell=exit_cell, d=facing)
    return dict(job='done', path=[], cell=cur, d=facing)
