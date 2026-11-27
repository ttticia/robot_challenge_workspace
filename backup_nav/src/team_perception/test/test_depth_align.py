"""Unit tests for DepthAlign / DepthAlignState in qr_utils. Run: pytest test/test_depth_align.py."""
import numpy as np
import pytest

from team_perception.qr_utils import DepthAlign, DepthAlignState

# Lite3 Orbbec values (see data_required.md): colour fx 462, depth fx 373, 14 mm apart.
COLOUR_K = (462.0, 462.0, 320.0, 241.0)
DEPTH_K = (373.0, 373.0, 319.0, 242.0)
SIZE = (640, 480)
T = (0.014, 0.0, 0.0)


def test_principal_point_maps_to_principal_point():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K)
    assert a.pixel(320, 241) == pytest.approx((319, 242))


def test_off_centre_pixel_scales_by_focal_ratio():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K)
    u, v = a.pixel(320 + 462, 241)            # 1 rad-ish to the right in normalised coords
    assert u == pytest.approx(319 + 373)
    assert v == pytest.approx(242)


def test_parallax_shrinks_with_distance():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K, T)
    near = a.pixel(320, 241, z=0.5)[0] - 319
    far = a.pixel(320, 241, z=4.0)[0] - 319
    assert near == pytest.approx(373 * 0.014 / 0.5)
    assert far == pytest.approx(373 * 0.014 / 4.0)
    assert near > far > 0


def test_image_size_rescales_input_pixel():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K)
    assert a.pixel(160, 120.5, image_size=(320, 240)) == pytest.approx(a.pixel(320, 241))


def test_assume_aligned_is_plain_resolution_scaling():
    a = DepthAlign.assume_aligned(COLOUR_K, SIZE, (240, 320))   # depth at half resolution
    assert a.pixel(320, 241) == pytest.approx((160, 120.5))
    assert a.pixel(640, 480) == pytest.approx((320, 240))


def test_box_maps_both_corners():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K)
    box = a.box([100, 50, 300, 400])
    assert box[:2] == pytest.approx(list(a.pixel(100, 50)))
    assert box[2:] == pytest.approx(list(a.pixel(300, 400)))


def test_depth_at_uses_parallax_corrected_pixel():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K, T)
    depth = np.zeros((480, 640), np.uint16)
    depth[:] = 1000                           # 1 m: corrected pixel lands ~5 px right of 319
    found = a.depth_at(depth, 320, 241)
    assert found is not None
    z, ud, vd = found
    assert z == pytest.approx(1.0)
    assert ud == pytest.approx(319 + 373 * 0.014 / 1.0)
    assert vd == pytest.approx(242)


def test_depth_at_returns_none_without_depth():
    a = DepthAlign(COLOUR_K, SIZE, DEPTH_K, T)
    assert a.depth_at(np.zeros((480, 640), np.uint16), 320, 241) is None


def test_state_waits_for_camera_infos():
    s = DepthAlignState(depth_info=True)
    assert s.get((480, 640)) == (None, 'no colour camera_info yet')
    s.set_colour(COLOUR_K, SIZE, 'camera_color_optical_frame')
    assert s.get((480, 640)) == (None, 'no depth camera_info yet')


def test_state_asks_for_tf_then_uses_it():
    s = DepthAlignState(depth_info=True)
    s.set_colour(COLOUR_K, SIZE, 'camera_color_optical_frame')
    s.set_depth(DEPTH_K, 'camera_depth_optical_frame')
    assert s.needs_tf() == ('camera_depth_optical_frame', 'camera_color_optical_frame')
    align, why = s.get((480, 640))
    assert align.t == (0.0, 0.0, 0.0) and 'no TF' in why
    s.t = T
    assert s.needs_tf() is None
    align, why = s.get((480, 640))
    assert align.t == T and why == 'ok'


def test_state_same_frame_needs_no_tf():
    s = DepthAlignState(depth_info=True)
    s.set_colour(COLOUR_K, SIZE, 'cam')
    s.set_depth(DEPTH_K, 'cam')
    assert s.needs_tf() is None
    assert s.t == (0.0, 0.0, 0.0)


def test_state_depth_frame_change_resets_tf():
    s = DepthAlignState(depth_info=True)
    s.set_colour(COLOUR_K, SIZE, 'colour')
    s.set_depth(DEPTH_K, 'depth_a')
    s.t = T
    s.set_depth(DEPTH_K, 'depth_b')
    assert s.t is None


def test_state_without_depth_info_assumes_aligned():
    s = DepthAlignState(depth_info=False)
    s.set_colour(COLOUR_K, SIZE, 'colour')
    assert s.needs_tf() is None
    align, why = s.get((480, 640))
    assert why == 'assumed aligned'
    assert align.pixel(320, 241) == pytest.approx((320, 241))
