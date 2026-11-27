from task_reporter.missions import check, look_cell, Missions


def by_task(reports):
    return {r['task_id']: r for r in reports}


def test_m01_and_m02_use_the_booklet_format():
    m = Missions('group04_AB12')
    m.scene((3, 2), 'workshop', objects=['hand_tools', 'utensils'])
    m.scene((4, 2), 'kitchen', hazards=['high_voltage'])
    r = by_task(m.reports())
    assert r['M01'] == {'group_id': 'group04_AB12', 'task_id': 'M01', 'task_type': 'object',
                        'scene_type': 'workshop', 'label': 'hand_tools', 'pos': [3.06, 2.04],
                        'message': ''}
    assert (r['M02']['task_type'], r['M02']['label'], r['M02']['pos']) == \
        ('hazard', 'high_voltage', [4.08, 2.04])
    assert m.reports() == []                           # nothing changed: nothing sent again


def test_survivor_with_qr_is_m03_and_its_text_is_m04_unchanged():
    m = Missions('g')
    m.scene((2, 3), 'kitchen', humans=True, now=0.0)
    m.qr('Please help me. ', (2, 3))
    r = by_task(m.reports(now=1.0))
    assert (r['M03']['task_type'], r['M03']['scene_type'], r['M03']['pos']) == \
        ('survivor', 'kitchen', [2.04, 3.06])
    assert r['M04']['message'] == 'Please help me. ' and r['M04']['pos'] is None   # server: QR has no pos
    assert r['M04']['task_type'] == 'qr' and 'M05' not in r


def test_no_qr_read_first_survivor_is_m03_second_is_m05():
    m = Missions('g', qr_wait=15.0)
    m.scene((3, 3), 'market', humans=True, now=0.0)
    assert m.reports(now=5.0) == []                    # its QR code may still be read
    r = by_task(m.reports(now=16.0))
    assert set(r) == {'M03'} and r['M03']['scene_type'] == 'market'
    m.scene((1, 2), 'clinic', humans=True, now=20.0)
    r = by_task(m.reports(now=40.0))
    assert set(r) == {'M05'} and r['M05']['pos'] == [1.02, 2.04]


def test_a_qr_read_in_the_second_room_swaps_m03_and_m05():
    m = Missions('g', qr_wait=1.0)
    m.scene((2, 3), 'kitchen', humans=True, now=0.0)
    assert set(by_task(m.reports(now=2.0))) == {'M03'}
    m.scene((3, 3), 'market', humans=True, now=3.0)
    m.qr('Please help me.', (3, 3))
    r = by_task(m.reports(now=10.0))
    assert r['M03']['pos'] == [3.06, 3.06] and r['M05']['pos'] == [2.04, 3.06]
    assert r['M04']['scene_type'] == 'market'


def test_unknown_scene_type_and_labels_are_not_reported():
    m = Missions('g')
    notes = m.scene((1, 1), 'corridor', objects=['hand_tools'], hazards=['fire'])
    assert len(notes) == 2 and m.reports() == []
    m.scene((1, 1), 'workshop')                        # the room is named later
    assert set(by_task(m.reports())) == {'M01'}


def test_qr_read_outside_a_room_still_reports_m04():
    m = Missions('g')
    m.qr('Please help me.', None)
    r = by_task(m.reports())
    assert set(r) == {'M04'} and r['M04']['scene_type'] == 'kitchen'
    assert r['M04']['pos'] is None                     # the QR report has pos null


def test_the_qr_report_never_carries_a_position():
    # 08:01 run, 2026-11-26: HTTP 422 "QR requires nonempty message, null pos, empty label,
    # and empty/permitted scene_type" for an M04 report with a position
    m = Missions('g')
    m.qr('Please help me.', None, near=(2, 1))
    assert by_task(m.reports())['M04']['pos'] is None
    m.qr('Please help me.', (2, 2))
    assert 'M04' not in by_task(m.reports())           # nothing changes when its room becomes known


def test_every_report_has_the_seven_keys_and_a_position():
    m = Missions('group04_grp4', qr_wait=0.0)
    m.scene((1, 2), 'workshop', objects=['hand_tools'], hazards=['radioactive'], humans=True)
    m.scene((3, 0), 'market', humans=True)
    m.qr('Please help me.', (1, 2))
    r = by_task(m.reports(now=1.0))
    assert set(r) == {'M01', 'M02', 'M03', 'M04', 'M05'}
    for p in r.values():
        assert list(p) == ['group_id', 'task_id', 'task_type', 'scene_type', 'label', 'pos',
                           'message']
        if p['task_id'] == 'M04':
            assert p['pos'] is None
        else:
            assert len(p['pos']) == 2 and all(isinstance(v, float) for v in p['pos'])


def test_check_accepts_the_booklet_example_and_refuses_bad_reports():
    good = {'group_id': 'group04_grp4', 'task_id': 'M01', 'task_type': 'object',
            'scene_type': 'workshop', 'label': 'hand_tools', 'pos': [1.02, 2.04], 'message': ''}
    assert check(good) == (True, '')
    for change in ({'pos': None}, {'scene_type': 'junction'}, {'label': 'radioactive'},
                   {'task_type': 'hazard'}, {'group_id': 'group04'}, {'message': 'x'}):
        assert not check(dict(good, **change))[0], change
    qr = dict(good, task_id='M04', task_type='qr', label='', message='Please help me.', pos=None)
    assert check(qr)[0]
    assert not check(dict(qr, pos=[1.02, 2.04]))[0]    # the server rejects a QR report with a position
    assert not check(dict(qr, message=''))[0]


def test_look_cell():
    assert [look_cell((1, 0), d) for d in range(4)] == [(2, 0), (1, 1), (0, 0), (1, -1)]


def test_a_later_sighting_does_not_replace_what_was_reported():
    m = Missions('g')
    m.scene((0, 1), 'workshop', objects=['hand_tools'])
    m.scene((1, -1), 'kitchen', hazards=['radioactive'])
    first = by_task(m.reports())
    assert first['M01']['pos'] == [0.0, 1.02] and first['M02']['label'] == 'radioactive'
    notes = m.scene((2, 2), 'market', objects=['hand_tools', 'food'], hazards=['flammable'])
    assert len(notes) == 3                             # each said once, nothing lost
    assert m.reports() == []                           # one answer per mission: nothing new
    assert m.scene((2, 2), 'market', objects=['hand_tools', 'food'], hazards=['flammable']) == []
    assert len(m.sightings()) == 5


def test_other_objects_are_recorded_but_never_take_m01():
    m = Missions('g')
    notes = m.scene((0, 2), 'clinic', objects=['medical_equipment'])
    assert len(notes) == 1 and m.reports() == []
    assert m.sightings() == [{'cell': [0, 2], 'pos': [0.0, 2.04], 'kind': 'object',
                              'label': 'medical_equipment', 'scene_type': 'clinic'}]
    m.scene((0, 1), 'workshop', objects=['hand_tools'])
    assert by_task(m.reports())['M01']['label'] == 'hand_tools'


def test_the_mission_target_is_a_setting():
    m = Missions('g', m01_object='medical_equipment', m02_hazard='flammable')
    m.scene((0, 1), 'workshop', objects=['hand_tools'], hazards=['radioactive'])
    assert m.reports() == []                           # seen, but not what was asked for
    m.scene((0, 2), 'clinic', objects=['medical_equipment'], hazards=['flammable'])
    r = by_task(m.reports())
    assert (r['M01']['label'], r['M01']['scene_type'], r['M01']['pos']) == \
        ('medical_equipment', 'clinic', [0.0, 2.04])
    assert r['M02']['label'] == 'flammable' and check(dict(r['M02'], group_id='group04_grp4'))[0]


def test_the_same_finding_is_sent_again_when_its_room_gets_a_name():
    m = Missions('g')
    m.scene((0, 1), 'unknown', objects=['hand_tools'])
    assert m.reports() == []
    m.scene((0, 1), 'workshop')
    assert by_task(m.reports())['M01']['scene_type'] == 'workshop'


def test_positions_are_the_looked_at_cell_in_whole_cells_of_1_02():
    m = Missions('g', 1.02, 15.0)
    looked = look_cell((2, -1), 3)                   # standing in (2, -1), looking right
    assert looked == (2, -2)
    m.scene(looked, 'clinic', humans=True, now=0.0)
    m.qr('Please help me.', looked, near=(2, -1))
    got = {r['task_id']: r['pos'] for r in m.reports(now=1.0)}
    assert got['M03'] == [2.04, -2.04] and got['M04'] is None
    for i in range(-6, 7):                           # every cell: an exact multiple, 2 decimals
        assert m.pos((i, -i)) == [float(f'{i * 1.02:.2f}'), float(f'{-i * 1.02:.2f}')]


def test_survivor_with_a_qr_code_seen_is_m03_at_once():
    m = Missions('g', qr_wait=15.0)
    m.scene((1, 2), 'clinic', humans=True, now=0.0)          # found first, no code
    m.scene((0, 1), 'workshop', humans=True, now=1.0, qr=True)
    r = by_task(m.reports(now=2.0))                          # no wait, no code read yet
    assert r['M03']['pos'] == [0.0, 1.02] and r['M05']['pos'] == [1.02, 2.04]
    assert 'M04' not in r
    m.qr('Please help me.', (0, 1))
    r = by_task(m.reports(now=3.0))
    assert set(r) == {'M04'} and r['M04']['message'] == 'Please help me.'


def test_a_qr_code_seen_without_a_survivor_changes_nothing():
    m = Missions('g', qr_wait=1.0)
    m.scene((0, 1), 'workshop', qr=True, now=0.0)
    m.scene((1, 2), 'clinic', humans=True, now=0.0)
    assert set(by_task(m.reports(now=5.0))) == {'M03'}


def test_a_report_the_server_marks_wrong_lets_the_next_sighting_take_the_mission():
    from task_reporter.missions import Missions
    m = Missions('group04_grp4')
    m.scene((1, 0), 'workshop', objects=['hand_tools'], hazards=['chemical'])
    first = {p['task_id']: p for p in m.reports()}
    assert first['M01']['pos'] == [1.02, 0.0]
    m.refused(first['M01'])                            # evaluated wrong by the server
    assert m.reports() == []                           # nothing else known: it stands
    m.scene((2, 1), 'market', objects=['hand_tools'], hazards=['flammable'])
    again = {p['task_id']: p for p in m.reports()}
    assert again['M01']['pos'] == [2.04, 1.02]         # the next sighting is reported instead
    assert 'M02' not in again                          # M02 was fine: its first sighting holds


def test_what_counts_as_successful():
    pytest = __import__('pytest')
    node = pytest.importorskip('task_reporter.task_reporter')
    ok = node.successful
    assert ok('{"status": "accepted", "task_id": "M01", "message": "Submission recorded"}')
    assert ok('{"status":"accepted","evaluation":{"scene_type":true,"pos":true,"label":true},"score":10}')
    assert not ok('{"status":"accepted","evaluation":{"scene_type":false,"pos":true},"score":0}')
    assert not ok('{"status": "rejected", "reason": "Invalid scene_type"}')
    assert not ok('not json')


def test_the_course_follows_maze_side():
    from task_reporter.missions import course_url
    base = 'http://192.168.10.141:8001/task_report'
    assert course_url(base, 'L') == base + '_A'              # maze on the left = course A
    assert course_url(base, 'R') == base + '_B'              # maze on the right = course B
    assert course_url(base, 'r ') == base + '_B'
    assert course_url(base + '_A', 'R') == base + '_A'       # a course named by hand wins
    assert course_url(base, '') == '' and course_url(base, 'X') == ''     # no guess: nothing posted


def test_a_report_refused_for_no_competition_slot_is_kept_and_sent_again():
    from task_reporter.task_reporter import not_now
    # 10:39 run, 2026-11-26: every report refused with this
    assert not_now(409, '{"status":"rejected","reason":"No active competition slot"}')
    assert not_now(503, '')
    assert not not_now(422, '{"reason":"QR requires nonempty message, null pos"}')   # a real fault
