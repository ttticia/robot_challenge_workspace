"""Tests for room_scene: frame choice, prompt, result mapping, VLM wiring."""

import json
import time

import cv2
import numpy as np
import pytest

from scene_understanding.room_scene import (
    build_room_prompt, order_views, pick_frame, scene_fields, stamp_ns, stitch,
)
from scene_understanding.scene_prompt import SCENE_PROMPT


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
    off = [(s + 50 * 10**9, a, n) for s, a, n in frames()]
    assert pick_frame(off, 10 * 10**9, 11 * 10**9, 100.1, 101.5) == (
        'late', 'arrival')


def test_no_frame_in_the_hold():
    assert pick_frame(frames(), 20 * 10**9, 21 * 10**9, 200.0, 201.0) == (
        None, None)


def test_views_ordered_left_centre_right_then_others():
    views = [{'view': v} for v in ('centre', '0', 'right', 'left', '1')]
    assert [v['view'] for v in order_views(views)] == [
        'left', 'centre', 'right', '0', '1']


def test_stitch_scales_to_common_height():
    a = np.zeros((480, 640, 3), np.uint8)
    b = np.zeros((240, 320, 3), np.uint8)
    assert stitch([a, b, a]).shape == (240, 320 * 3, 3)


def test_room_prompt_maps_panels_to_directions():
    prompt = build_room_prompt(['left', 'centre', 'right'])
    assert prompt.endswith(SCENE_PROMPT)
    assert 'Panel 1' in prompt and 'direction="left"' in prompt
    assert 'Panel 2' in prompt and 'direction="forward"' in prompt
    assert 'No panel shows' not in prompt


def test_room_prompt_marks_missing_directions_unknown():
    prompt = build_room_prompt(['centre', 'right'])
    assert 'No panel shows left' in prompt


def test_single_view_uses_plain_prompt():
    assert build_room_prompt(['centre']) == SCENE_PROMPT


def scene(scene_type='unknown', matches=(), notes=('', '', '')):
    return {'scene_type': scene_type, 'junction_confirmed': None,
            'paths': [{'direction': d, 'status': 'open', 'notes': n}
                      for d, n in zip(('forward', 'left', 'right'), notes)],
            'mission_matches': list(matches)}


def test_matches_split_into_objects_hazards_signs_humans_qr():
    f = scene_fields(scene('clinic', [
        {'kind': 'object', 'label': 'medical_equipment'},
        {'kind': 'object', 'label': 'medical_equipment'},       # duplicate
        {'kind': 'hazard', 'label': 'high_voltage'},
        {'kind': 'hazard', 'label': 'laser'},           # not reportable
        {'kind': 'survivor', 'label': ''},
        {'kind': 'qr', 'label': '', 'message': 'My friend is in the market.'},
        {'kind': 'qr', 'label': '', 'message': ''},       # unreadable
    ]))
    assert f['scene_type'] == 'clinic'
    assert f['objects'] == ['medical_equipment']
    assert f['hazards'] == ['high_voltage']
    assert f['signs'] == ['laser']
    assert f['humans'] is True
    assert f['qr'] == ['My friend is in the market.']


def test_stairs_and_curtain_notes_set_scene_type():
    f = scene_fields(scene('corridor', notes=('Metal stairs', '', '')))
    assert (f['scene_type'], f['vlm_scene_type']) == ('stairs', 'corridor')
    f = scene_fields(scene('unknown', notes=('Passable curtain', '', '')))
    assert f['scene_type'] == 'curtain'


def test_room_type_wins_over_stairs_note():
    assert scene_fields(scene('kitchen', notes=('Metal stairs', '', '')))[
        'scene_type'] == 'kitchen'


# ---------------------------------------------------------------- ROS node

rclpy = pytest.importorskip('rclpy')


@pytest.fixture
def node_factory(tmp_path):
    from scene_understanding.room_scene import RoomScene
    made = []

    def make(*params):
        args = ['--ros-args', '-p', f'save_dir:={tmp_path}', '-p', 'vlm:=none']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args)
        node = RoomScene()
        made.append(node)
        results = []
        node.result_pub.publish = lambda m: results.append(json.loads(m.data))
        return node, results

    yield make
    for node in made:
        node.destroy_node()
    rclpy.try_shutdown()


def wait_for(results, n, timeout=5.0):
    deadline = time.monotonic() + timeout
    while len(results) < n and time.monotonic() < deadline:
        time.sleep(0.05)


def capture_msg(cell, view, last, attempt=None):
    from std_msgs.msg import String
    c = {'cell': cell, 'cell_centre': [float(cell[0]), float(cell[1])],
         'view': view, 'last': last, 'hold_start': '10.0', 'stamp': '11.0',
         'kind': 'dead_end'}
    if attempt is not None:
        c['attempt'] = attempt
    return String(data=json.dumps(c))


def jpeg_msg(colour, nanosec=500_000_000):
    from sensor_msgs.msg import CompressedImage
    img = np.full((120, 160, 3), colour, np.uint8)
    msg = CompressedImage(format='jpeg')
    msg.data = cv2.imencode('.jpg', img)[1].tobytes()
    msg.header.stamp.sec = 10                  # inside the 10.0-11.0 hold
    msg.header.stamp.nanosec = nanosec
    return msg


def test_retry_is_analysed_again_up_to_max_attempts(node_factory):
    node, results = node_factory()
    node.on_capture(capture_msg([2, 1], 'centre', True))      # attempt 1
    node.on_capture(capture_msg([2, 1], 'centre', True, 1))   # dup: ignored
    node.on_capture(capture_msg([2, 1], 'centre', True, 2))   # retry
    node.on_capture(capture_msg([2, 1], 'centre', True, 3))   # > max
    assert node.done == {((2, 1), 1), ((2, 1), 2)}
    wait_for(results, 2)
    assert sorted(r['attempt'] for r in results) == [1, 2]
    assert all(r['vlm'] == 'none' for r in results)


def test_retry_resends_the_captured_views_once(node_factory):
    from std_msgs.msg import String
    node, results = node_factory()
    for view, last in (('centre', False), ('left', False), ('right', True)):
        node.on_capture(capture_msg([3, 0], view, last))
    for cell, attempt in (([3, 0], 2), ([3, 0], 2), ([3, 0], 3), ([9, 9], 2)):
        node.on_retry(String(data=json.dumps(
            {'cell': cell, 'attempt': attempt})))
    wait_for(results, 2)
    time.sleep(0.2)
    assert [(r['attempt'], len(r['views'])) for r in results] == [
        (1, 3), (2, 3)]


class FakeClient:
    """Stands in for VLMClient / ClaudeVLMClient."""

    def __init__(self, reply):
        self.reply = reply
        self.calls = []

    def describe(self, frame, *, prompt=None):
        self.calls.append((frame.shape, prompt))
        return self.reply


def test_room_views_stitched_into_one_vlm_call(node_factory, tmp_path):
    node, results = node_factory()
    answer = scene('market', [{'kind': 'object', 'label': 'food'},
                              {'kind': 'hazard', 'label': 'flammable'}])
    node.client = FakeClient({'ok': True, 'error': None, 'latency_sec': 1.5,
                              'response': json.dumps(answer)})
    node.vlm_name = 'lab'
    for k, (colour, view) in enumerate(
            ((0, 'centre'), (128, 'left'), (255, 'right'))):
        node.on_image(jpeg_msg(colour, (k + 1) * 200_000_000))
        node.on_capture(capture_msg([4, 2], view, view == 'right'))
    wait_for(results, 1)
    (shape, prompt), = node.client.calls
    assert shape == (120, 160 * 3, 3)
    assert prompt.index('left side wall') < prompt.index('facing into')
    r, = results
    assert [v['view'] for v in r['views']] == ['left', 'centre', 'right']
    assert (r['vlm'], r['scene_type'], r['objects'], r['hazards']) == (
        'lab', 'market', ['food'], ['flammable'])
    assert r['error'] is None and r['image'].endswith('_room.jpg')
    assert r['pos'] == [4.0, 2.0] == r['cell_centre']
    assert [(x['task_type'], x['label'], x['pos'], x['reportable'])
            for x in r['reports']] == [
        ('object', 'food', [4.0, 2.0], True),
        ('hazard', 'flammable', [4.0, 2.0], True)]
    saved, = tmp_path.glob('*/rooms.jsonl')
    assert json.loads(saved.read_text())['vlm_response'] == json.dumps(answer)


def test_vlm_failure_is_published_as_not_analysed(node_factory):
    node, results = node_factory()
    node.client = FakeClient({'ok': False, 'error': 'HTTP 503',
                              'latency_sec': 0.1, 'response': None})
    node.vlm_name = 'lab'
    node.on_image(jpeg_msg(50))
    node.on_capture(capture_msg([1, 1], 'centre', True))
    wait_for(results, 1)
    r, = results
    assert (r['vlm'], r['error'], r['scene_type']) == (
        'none', 'HTTP 503', 'unknown')


def test_room_without_frames_does_not_call_the_vlm(node_factory):
    node, results = node_factory()
    node.client = FakeClient(None)
    node.vlm_name = 'lab'
    node.on_capture(capture_msg([5, 5], 'centre', True))
    wait_for(results, 1)
    assert node.client.calls == []
    assert results[0]['error'] == 'no camera frame for any view'
