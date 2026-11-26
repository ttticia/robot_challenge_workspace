"""room_scene's frame choice for a dead-end view (no ROS).

python3 -m pytest -q test/test_room_scene.py"""
from scene_understanding.room_scene import pick_frame, stamp_ns


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


def test_retry_is_analysed_again_up_to_max_attempts(tmp_path):
    """The explorer's 2nd try at a cell (attempt 2) goes to the VLM again; a 3rd doesn't."""
    import json
    import time
    import rclpy
    from std_msgs.msg import String
    from scene_understanding.room_scene import RoomScene
    rclpy.init(args=['--ros-args', '-p', f'save_dir:={tmp_path}'])
    try:
        node = RoomScene()
        results = []
        node.result_pub.publish = lambda m: results.append(json.loads(m.data))

        def capture(attempt=None):
            c = {'cell': [2, 1], 'cell_centre': [2.0, 1.0], 'view': 'centre', 'last': True,
                 'hold_start': '10.0', 'stamp': '11.0', 'kind': 'dead_end'}
            if attempt is not None:
                c['attempt'] = attempt
            node.on_capture(String(data=json.dumps(c)))

        capture()            # attempt 1 (no field: old explorer)
        capture(1)           # same attempt again: ignored
        capture(2)           # the retry: analysed afresh
        capture(3)           # over max_attempts: ignored
        assert node.done == {((2, 1), 1), ((2, 1), 2)}
        deadline = time.monotonic() + 5.0
        while len(results) < 2 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert sorted(r['attempt'] for r in results) == [1, 2]
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


def test_retry_resends_the_captured_views_once(tmp_path):
    """/scene/retry: the views of attempt 1 go to the VLM again as attempt 2; a 3rd try
    and a retry of a cell never captured are ignored."""
    import json
    import time
    import rclpy
    from std_msgs.msg import String
    from scene_understanding.room_scene import RoomScene
    rclpy.init(args=['--ros-args', '-p', f'save_dir:={tmp_path}'])
    try:
        node = RoomScene()
        results = []
        node.result_pub.publish = lambda m: results.append(json.loads(m.data))
        for view, last in (('centre', False), ('left', False), ('right', True)):
            node.on_capture(String(data=json.dumps({
                'cell': [3, 0], 'cell_centre': [3.0, 0.0], 'view': view, 'last': last,
                'hold_start': '10.0', 'stamp': '11.0', 'kind': 'dead_end'})))
        node.on_retry(String(data=json.dumps({'cell': [3, 0], 'attempt': 2})))
        node.on_retry(String(data=json.dumps({'cell': [3, 0], 'attempt': 2})))   # dup
        node.on_retry(String(data=json.dumps({'cell': [3, 0], 'attempt': 3})))   # > max
        node.on_retry(String(data=json.dumps({'cell': [9, 9], 'attempt': 2})))   # no views
        deadline = time.monotonic() + 5.0
        while len(results) < 2 and time.monotonic() < deadline:
            time.sleep(0.05)
        time.sleep(0.2)
        assert [(r['attempt'], len(r['views'])) for r in results] == [(1, 3), (2, 3)]
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
