"""depth_mask: grey out what lies beyond the room before a frame goes to the vision model.

python3 -m pytest -q test/test_depth_mask.py"""
import math

import numpy as np
import pytest

from scene_understanding.depth_mask import (apply_mask, depth_mask, depth_to_metres,
                                            room_depth_limit)

DEPTH_K = (373.0, 373.0, 320.0, 240.0)       # Orbbec depth, unaligned (qr_utils.DepthAlign)
COLOUR_K = (462.0, 462.0, 320.0, 240.0)
SIZE = (640, 480)


def scene(room=1.6, beyond=4.0, wall_row=200, toy=(260, 380, 80, 200)):
    """Depth (m): the room's back wall / floor below row wall_row, the hall beyond it above,
    a toy set sticking up above the wall (cols toy[0]:toy[1], rows toy[2]:toy[3]) at the
    room's depth, and a dark patch on the toy with no depth."""
    d = np.full((480, 640), beyond, np.float32)
    d[wall_row:] = room
    c0, c1, r0, r1 = toy
    d[r0:r1, c0:c1] = room - 0.3
    d[120:126, 300:306] = 0.0                 # small hole on a shiny part
    return d


def test_room_limit_is_the_far_corner_plus_margin():
    lim = room_depth_limit((0.0, 0.0), (1.02, 0.0), 1.02, 0.25)
    assert lim == pytest.approx(1.02 + 0.51 * math.sqrt(2) + 0.25)


def test_beyond_the_room_is_greyed_and_the_toy_set_kept():
    d = scene()
    mask = depth_mask(d, DEPTH_K, COLOUR_K, SIZE, max_depth=2.0)
    assert mask.shape == (480, 640)
    # colour pixels: the toy set (above the wall line) and the room are kept, the hall isn't
    assert mask[150, 320]                         # toy set, above the wall top
    assert mask[123, 303]                         # its small depth hole, filled
    assert mask[400, 100]                         # room / floor
    assert not mask[30, 40] and not mask[30, 600]  # hall above the walls
    img = np.full((480, 640, 3), 200, np.uint8)
    out = apply_mask(img, mask)
    assert tuple(out[30, 40]) == (128, 128, 128) and tuple(out[400, 100]) == (200, 200, 200)


def test_mask_scales_to_a_bigger_colour_frame():
    mask = depth_mask(scene(), DEPTH_K, COLOUR_K, SIZE, max_depth=2.0)
    big = np.zeros((720, 1280, 3), np.uint8)
    out = apply_mask(big, mask)
    assert out.shape == big.shape and tuple(out[40, 60]) == (128, 128, 128)


def test_no_depth_keeps_nothing_and_mm_depth_converts():
    assert not depth_mask(np.zeros((480, 640)), DEPTH_K, COLOUR_K, SIZE, 2.0).any()
    assert depth_to_metres('16UC1', np.array([[1500]], np.uint16))[0, 0] == pytest.approx(1.5)
    assert depth_to_metres('32FC1', np.array([[1.5]], np.float32))[0, 0] == pytest.approx(1.5)
