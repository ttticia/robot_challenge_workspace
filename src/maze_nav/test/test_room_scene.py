"""room_scene's frame choice for a dead-end view (no ROS).

python3 -m pytest -q test/test_room_scene.py"""
from maze_nav.scene_understanding.room_scene import pick_frame, stamp_ns


def test_stamp_strings_parse_to_ns():
    assert stamp_ns('12.000000005') == 12 * 10**9 + 5
    assert stamp_ns('12.5') == 12 * 10**9 + 500_000_000
    assert stamp_ns('7') == 7 * 10**9


def frames():
    # stamp (ns), arrival (s), name
    return [(9 * 10**9, 100.0, 'before'), (10 * 10**9 + 2, 100.2, 'early'),
            (11 * 10**9, 101.0, 'late'), (12 * 10**9, 102.0, 'after')]


def test_newest_frame_stamped_inside_the_hold():
    assert pick_frame(frames(), 10 * 10**9, 11 * 10**9) == ('late', 'stamp')


def test_robot_clock_offset_falls_back_to_arrival_time():
    off = [(s + 50 * 10**9, a, n) for s, a, n in frames()]   # camera clock 50 s ahead
    assert pick_frame(off, 10 * 10**9, 11 * 10**9, 100.1, 101.5) == ('late', 'arrival')


def test_no_frame_in_the_hold():
    assert pick_frame(frames(), 20 * 10**9, 21 * 10**9, 200.0, 201.0) == (None, None)
