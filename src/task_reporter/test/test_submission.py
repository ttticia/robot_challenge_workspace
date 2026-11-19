from unittest.mock import Mock

import pytest
import requests

from task_reporter.submission import TaskSubmitter, validate_report


@pytest.fixture
def qr_report():
    return {
        'group_id': 'group01_A7K2',
        'task_id': 'M04',
        'task_type': 'qr',
        'scene_type': '',
        'label': '',
        'pos': None,
        'message': 'Check the market',
    }


def test_duplicate_and_correction(monkeypatch, qr_report):
    response = Mock(status_code=200)
    response.json.return_value = {
        'group_id': 'group01', 'task_id': 'M04', 'status': 'accepted',
    }
    post = Mock(return_value=response)
    monkeypatch.setattr(requests, 'post', post)
    reporter = TaskSubmitter('http://mock/task_report')

    assert reporter.submit(qr_report)['status'] == 'accepted'
    assert reporter.submit(qr_report)['status'] == 'duplicate'
    corrected = dict(qr_report, message='Check the workshop')
    assert reporter.submit(corrected)['status'] == 'accepted'
    assert post.call_count == 2
    assert post.call_args.kwargs['json'] == corrected


def test_timeout_leaves_server_state_uncertain(monkeypatch, qr_report):
    response = Mock(status_code=200)
    response.json.return_value = {
        'group_id': 'group01', 'task_id': 'M04', 'status': 'accepted',
    }
    post = Mock(side_effect=[response, requests.Timeout(), response])
    monkeypatch.setattr(requests, 'post', post)
    reporter = TaskSubmitter()

    assert reporter.submit(qr_report)['status'] == 'accepted'
    assert reporter.submit(dict(qr_report, message='Correction'))['status'] == 'error'
    assert reporter.submit(qr_report)['status'] == 'accepted'
    assert post.call_count == 3


def test_validation(qr_report):
    validate_report(qr_report)
    object_report = dict(
        qr_report, task_id='M01', task_type='object', scene_type='clinic',
        label='medical_kit', pos=[3.06, 2.04], message='',
    )
    validate_report(object_report)
    with pytest.raises(ValueError, match='finite'):
        validate_report(dict(object_report, pos=[True, 2.04]))
    with pytest.raises(ValueError, match='decoded text'):
        validate_report(dict(qr_report, message=''))
