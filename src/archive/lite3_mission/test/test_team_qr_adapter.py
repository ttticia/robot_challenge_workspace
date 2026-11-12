"""Protect the existing team's QR contract and missing-geometry behaviour."""
import math
import pytest
from lite3_mission.team_qr_adapter import team_qr_event


def test_localized_team_schema_keeps_capture_time_and_evidence():
    detection = dict(text='C', pixel=[1402.5, 690.1], frame='map',
                     position=[3.12, -.84, .41], snapshot='/original/qr.jpg', stamp=1762871234.52)
    event = team_qr_event(detection)
    assert event['kind'] == 'qr' and event['label'] == 'C'
    assert event['position'] == detection['position'] and event['frame_id'] == 'map'
    assert event['stamp'] == dict(sec=1762871234, nanosec=520000000)
    assert event['pixel'] == detection['pixel'] and event['snapshot'] == detection['snapshot']
    assert detection['stamp'] == 1762871234.52  # Source report is not mutated.


def test_camera_frame_and_text_only_reports_are_preserved_without_invented_pose():
    event = team_qr_event(dict(text='A', frame='camera_color_optical_frame',
                              position=[.1, .2, 1.], stamp=20.25))
    assert event['frame_id'] == 'camera_color_optical_frame'
    for fields in [dict(frame='none', position=None), dict(position=[1., 2., 3.]),
                   dict(frame='map', position=[1., 2., 3.], stamp=0)]:
        event = team_qr_event(dict(text='A', **fields))
        assert event['label'] == 'A' and 'position' not in event
    text = '{"robot_actions": ["move_forward"]}'
    event = team_qr_event(dict(text=text))
    assert event['label'] == text and 'robot_actions' not in event


@pytest.mark.parametrize('fields', [dict(text=''), dict(stamp=math.nan), dict(stamp=-1),
                                    dict(stamp=True), dict(stamp=2**31),
                                    dict(position=[1., math.inf, 2.]), dict(position=[True, 1., 2.]),
                                    dict(position=[1., 2.]), dict(pixel=[math.nan, 1.])])
def test_invalid_team_reports_are_rejected(fields):
    with pytest.raises(ValueError):
        team_qr_event(dict(dict(text='A'), **fields))
