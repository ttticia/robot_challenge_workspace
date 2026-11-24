"""The 'extru' maze wall (resource/extru wall.stl) as boxes, for the simulator and tests (no ROS).

The STL (mm, one solid): an aluminium frame 0.90 m long, 0.04 m thick, 0.45 m tall, with a
2 mm grey corrugated board on one face only. Along the wall, from its centre:
  board          |u| <= 0.31, full height, on one face
  inner posts    0.22 <= |u| <= 0.26, full height (behind the board)
  end posts      0.41 <= |u| <= 0.45, full height
  rails          bottom z 0..0.04, top z 0.41..0.45, full length
  side windows   0.31 <= |u| <= 0.41, z 0.04..0.41: open right through the wall
So the wall band (0.30-0.43/0.45 m above the floor) sees only the top rail across each
window, and the floor behind is visible through it.

extru_boxes(lo, hi, board_side): a wall solid's bounding box (min/max, like example_maze.json;
the long axis is the longer of x and y) -> [(x0, y0, z0, x1, y1, z1)] of its parts.
board_side -1 puts the board on the min face of the thin axis, +1 on the max face. A bounding
box that isn't 0.90 m long is scaled along the wall (posts and rails keep their size).
"""

LENGTH = 0.90
HEIGHT = 0.45
RAIL = 0.04
POST = 0.04
BOARD_HALF = 0.31
INNER_POST = (0.22, 0.26)
BOARD_T = 0.002


def extru_boxes(lo, hi, board_side=-1):
    lo, hi = list(lo), list(hi)
    along = 0 if hi[0] - lo[0] >= hi[1] - lo[1] else 1
    across = 1 - along
    c = (lo[along] + hi[along]) / 2
    half = (hi[along] - lo[along]) / 2
    k = half / (LENGTH / 2)                 # scale for a wall that isn't 0.90 m long
    z0, z1 = lo[2], hi[2]
    a0, a1 = lo[across], hi[across]
    if board_side < 0:
        board_a, frame_a = (a0, a0 + BOARD_T), (a0 + BOARD_T, a1)
    else:
        board_a, frame_a = (a1 - BOARD_T, a1), (a0, a1 - BOARD_T)

    def box(u0, u1, a, zz):
        b = [0.0] * 6
        b[along], b[along + 3] = c + u0, c + u1
        b[across], b[across + 3] = a
        b[2], b[5] = zz
        return tuple(b)

    parts = [box(-BOARD_HALF * k, BOARD_HALF * k, board_a, (z0, z1)),
             box(-half, half, frame_a, (z0, z0 + RAIL)),
             box(-half, half, frame_a, (z1 - RAIL, z1))]
    for s in (-1, 1):
        ends = sorted((s * half, s * (half - POST)))
        inner = sorted((s * INNER_POST[0] * k, s * INNER_POST[1] * k))
        parts.append(box(ends[0], ends[1], frame_a, (z0, z1)))
        parts.append(box(inner[0], inner[1], frame_a, (z0, z1)))
    return parts


def windows(lo, hi):
    """[(u0, u1)] of the open side windows along the wall, in the wall's own axis (world)."""
    along = 0 if hi[0] - lo[0] >= hi[1] - lo[1] else 1
    c = (lo[along] + hi[along]) / 2
    half = (hi[along] - lo[along]) / 2
    k = half / (LENGTH / 2)
    return [(c - (half - POST), c - BOARD_HALF * k), (c + BOARD_HALF * k, c + half - POST)]
