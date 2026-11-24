from scene_understanding.missions import Missions


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
    assert r['M04']['message'] == 'Please help me. ' and r['M04']['pos'] is None
    assert r['M04']['task_type'] == 'qr' and 'M05' not in r


def test_survivor_without_qr_is_m05_after_the_wait():
    m = Missions('g', qr_wait=15.0)
    m.scene((3, 3), 'market', humans=True, now=0.0)
    assert m.reports(now=5.0) == []                    # its QR code may still be read
    r = by_task(m.reports(now=16.0))
    assert set(r) == {'M05'} and r['M05']['scene_type'] == 'market'


def test_a_late_qr_moves_the_survivor_from_m05_to_m03():
    m = Missions('g', qr_wait=1.0)
    m.scene((2, 3), 'kitchen', humans=True, now=0.0)
    assert set(by_task(m.reports(now=2.0))) == {'M05'}
    m.scene((3, 3), 'market', humans=True, now=3.0)
    m.qr('Please help me.', (2, 3))
    r = by_task(m.reports(now=10.0))
    assert r['M03']['pos'] == [2.04, 3.06] and r['M05']['pos'] == [3.06, 3.06]
    assert r['M04']['scene_type'] == 'kitchen'


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
