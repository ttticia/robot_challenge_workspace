import pytest

from scene_understanding import scene_prompt
from scene_understanding.image_folder_publisher import is_junction_file
from scene_understanding.scene_prompt import (
    build_prompt, build_task_report, cell_centre, HAZARD_LABELS,
    mission_reports, OBJECT_LABELS, parse_scene_json, report_position,
    ROOM_SCENE_TYPES, SCENE_PROMPT, send_task_report, validate_scene_result)


def test_junction_file_names():
    assert is_junction_file('junction01.jpg')
    assert is_junction_file('Junction01.jpg')        # case-insensitive
    assert is_junction_file('/a/b/JUNCTION05.png')
    assert is_junction_file('junction03_extra.jpeg')
    assert not is_junction_file('junction06.jpg')
    assert not is_junction_file('IMG_junction01.jpg')
    assert not is_junction_file('20261106_071404305_iOS.jpg')


def test_build_prompt_detects_mission_clues_automatically():
    prompt = build_prompt()
    assert 'printed human figures' in prompt
    assert 'QR codes' in prompt
    assert '"mission_matches":[]' in prompt
    assert '"scene_type"' in prompt
    assert 'M01' not in prompt


def test_three_view_prompt_maps_each_direction_to_its_image_third():
    prompt = build_prompt(three_view=True)
    assert 'Leftmost third: direction="left"' in prompt
    assert 'Middle third: direction="forward"' in prompt
    assert 'Rightmost third: direction="right"' in prompt
    assert 'Leftmost third' not in build_prompt()


def test_parse_plain_and_fenced():
    assert parse_scene_json('{"scene_type": "junction"}') == {
        'scene_type': 'junction'}
    fenced = 'Here:\n```json\n{"a": {"b": 1}}\n```'
    assert parse_scene_json(fenced) == {'a': {'b': 1}}


def test_parse_invalid():
    assert parse_scene_json(None) is None
    assert parse_scene_json('no json') is None
    assert parse_scene_json('{broken') is None
    assert parse_scene_json('[1, 2]') is None


# ------------------------------------------------------------ task reports
# Booklet examples (task reporting section), with our group id format.

GROUP = 'group04_A7K2'


def test_booklet_examples_are_valid():
    assert build_task_report(GROUP, 'M01', 'clinic', 'medical_equipment',
                             [3.06, 2.04]) == {
        'group_id': GROUP, 'task_id': 'M01', 'task_type': 'object',
        'scene_type': 'clinic', 'label': 'medical_equipment',
        'pos': [3.06, 2.04], 'message': ''}
    build_task_report(GROUP, 'M02', 'kitchen', 'high_voltage', [4.08, 2.04])
    build_task_report(GROUP, 'M03', 'kitchen', pos=[2.04, 3.06])
    qr = build_task_report(GROUP, 'M04', 'kitchen',
                           message='My friend is in the market.')
    assert qr['pos'] is None and qr['label'] == ''
    build_task_report(GROUP, 'M05', 'market', pos=[3.06, 3.06])


def test_vocabulary_matches_booklet():
    assert set(ROOM_SCENE_TYPES) == {
        'clothes_shop', 'fitness_centre', 'kitchen', 'market', 'workshop',
        'clinic', 'cleaning_room', 'furniture_shop'}
    assert set(OBJECT_LABELS) == {'medical_equipment', 'hand_tools', 'food',
                                  'water', 'utensils'}
    assert set(HAZARD_LABELS) == {'high_voltage', 'radioactive', 'chemical',
                                  'biological', 'flammable'}


def test_m01_m02_accept_any_label_of_their_type():
    for label in OBJECT_LABELS:
        build_task_report(GROUP, 'M01', 'workshop', label, [1.02, 0.0])
    for label in HAZARD_LABELS:
        build_task_report(GROUP, 'M02', 'workshop', label, [1.02, 0.0])


@pytest.mark.parametrize('args, reason', [
    (('M01', 'clinic', 'medical_kit', [1.0, 2.0]), 'Invalid label'),
    (('M01', 'clinic', 'high_voltage', [1.0, 2.0]), 'Invalid label'),
    (('M02', 'clinic', 'laser', [1.0, 2.0]), 'Invalid label'),
    (('M03', 'clinic', 'survivor', [1.0, 2.0]), 'Invalid label'),
    (('M01', 'corridor', 'food', [1.0, 2.0]), 'Invalid scene_type'),
    (('M03', '', '', [1.0, 2.0]), 'Invalid scene_type'),
    (('M04', '', '', None, 'hi'), 'Invalid scene_type'),
    (('M04', 'kitchen', '', [1.0, float('inf')], 'hi'), 'finite'),
    (('M04', 'kitchen', '', None, '  '), 'decoded message'),
    (('M05', 'market', '', None), 'finite room/cell centre'),
    (('M05', 'market', '', [1.0, float('nan')]), 'finite room/cell centre'),
    (('M02', 'kitchen', 'flammable', [1.0, 2.0], 'x'), 'message must be empty'),
    (('M06', 'kitchen', '', [1.0, 2.0]), 'Unknown mission'),
])
def test_invalid_reports_rejected(args, reason):
    with pytest.raises(ValueError, match=reason):
        build_task_report(GROUP, *args)


def test_pos_rounded_and_task_type_checked():
    r = build_task_report(GROUP, 'M03', 'kitchen', pos=[2.0412, 3.0587])
    assert r['pos'] == [2.04, 3.06]
    ok, reason = validate_scene_result(dict(r, task_type='object'))
    assert not ok and 'survivor task' in reason
    ok, reason = validate_scene_result(dict(r, group_id='group4'))
    assert not ok


def test_prompt_uses_booklet_vocabulary():
    for word in ROOM_SCENE_TYPES + OBJECT_LABELS + HAZARD_LABELS:
        assert word in SCENE_PROMPT, word
    assert 'medical_kit' not in SCENE_PROMPT
    assert 'kind="sign"' in SCENE_PROMPT


def test_send_task_report_posts_json(monkeypatch):
    sent = {}

    class Reply:
        def raise_for_status(self):
            pass

        def json(self):
            return {'status': 'accepted', 'task_id': 'M05'}

    def post(url, json, timeout):
        sent.update(url=url, json=json, timeout=timeout)
        return Reply()

    import requests
    monkeypatch.setattr(requests, 'post', post)
    report = build_task_report(GROUP, 'M05', 'market', pos=[3.06, 3.06])
    assert send_task_report(report)['status'] == 'accepted'
    assert sent['url'] == scene_prompt.TASK_REPORT_URL
    assert sent['json'] == report


def test_send_task_report_never_sends_invalid(monkeypatch):
    import requests
    monkeypatch.setattr(requests, 'post',
                        lambda *a, **k: pytest.fail('must not send'))
    with pytest.raises(ValueError):
        send_task_report({'task_id': 'M01'})


def test_qr_may_carry_a_position():
    r = build_task_report(GROUP, 'M04', 'kitchen', pos=[2.04, 3.06],
                          message='My friend is in the market.')
    assert r['pos'] == [2.04, 3.06]


@pytest.mark.parametrize('yaw_deg, ahead', [
    (0, [2.04, 1.02]), (88, [1.02, 2.04]), (180, [0.0, 1.02]),
    (-92, [1.02, 0.0]), (40, [2.04, 1.02]), (50, [1.02, 2.04]),
])
def test_report_position_snaps_to_grid(yaw_deg, ahead):
    import math
    # Robot near the centre of cell (1, 1).
    assert report_position(1.1, 0.95, math.radians(yaw_deg), 'robot') == [1.02, 1.02]
    assert report_position(1.1, 0.95, math.radians(yaw_deg), 'ahead') == ahead


def test_cell_centre_and_bad_mode():
    assert cell_centre(-0.15, -0.05) == [0.0, 0.0]
    assert cell_centre(3.5, -1.6) == [3.06, -2.04]
    with pytest.raises(ValueError):
        report_position(0.0, 0.0, 0.0, 'nearest')
    with pytest.raises(ValueError):
        report_position(0.0, 0.0, None, 'ahead')


def test_mission_reports_all_carry_pos():
    scene = {'scene_type': 'Kitchen', 'mission_matches': [
        {'kind': 'object', 'label': 'utensils', 'evidence': 'ladle'},
        {'kind': 'hazard', 'label': 'flammable'},
        {'kind': 'survivor', 'label': ''},
        {'kind': 'qr', 'label': '', 'message': 'My friend is in the market.'},
        {'kind': 'object', 'label': 'medical_kit'},
        {'kind': 'sign', 'label': 'laser'},           # not a task type
    ]}
    reports = mission_reports(scene, [2.04, 1.02])
    assert [r['task_type'] for r in reports] == [
        'object', 'hazard', 'survivor', 'qr', 'object']
    assert all(r['pos'] == [2.04, 1.02] for r in reports)
    assert [r['reportable'] for r in reports] == [True, True, True, True, False]
    assert reports[4]['problems'] == ['label']
    assert reports[2]['task_ids'] == ['M03', 'M05']
    assert reports[0]['scene_type'] == 'kitchen'
    for r, task_id in zip(reports[:4], ('M01', 'M02', 'M03', 'M04')):
        build_task_report(GROUP, task_id, r['scene_type'], r['label'], r['pos'],
                          r['message'])


def test_mission_reports_flag_missing_room_and_pos():
    reports = mission_reports({'scene_type': 'corridor', 'mission_matches': [
        {'kind': 'survivor'}, {'kind': 'qr', 'message': ''}]}, None)
    assert reports[0]['problems'] == ['scene_type', 'pos']
    assert reports[1]['problems'] == ['scene_type', 'pos', 'message']
