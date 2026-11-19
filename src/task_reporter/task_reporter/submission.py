"""Validate and submit confirmed results using the scoring API."""

import math
import re

import requests


DEFAULT_URL = 'http://192.168.10.141:8001/task_report'
SCENES = {'kitchen', 'market', 'workshop', 'clinic', 'cleaning_room'}
LABELS = {
    'object': {'medical_kit', 'hand_tools', 'food', 'water', 'utensils'},
    'hazard': {'high_voltage', 'radioactive', 'chemical', 'biological', 'flammable'},
    'survivor': {''},
    'qr': {''},
}
STRING_FIELDS = {'group_id', 'task_id', 'task_type', 'scene_type', 'label', 'message'}


def validate_report(report):
    """Validate the seven fields required for a completed task."""
    if not isinstance(report, dict) or report.keys() != STRING_FIELDS | {'pos'}:
        raise ValueError('Expected exactly the seven task report fields')
    if any(not isinstance(report[key], str) for key in STRING_FIELDS):
        raise ValueError('All fields except pos must be strings')
    if not re.fullmatch(r'group[0-9]{2}_[A-Za-z0-9]{4}', report['group_id']):
        raise ValueError('group_id must use the registered group##_XXXX format')
    if not report['task_id'].strip():
        raise ValueError('task_id is required')

    kind = report['task_type']
    if kind not in LABELS or report['label'] not in LABELS[kind]:
        raise ValueError('Invalid task_type or label')
    scene = report['scene_type']
    pos = report['pos']
    if kind == 'qr':
        if pos is not None or not report['message'].strip() or scene not in SCENES | {''}:
            raise ValueError('QR requires decoded text, null pos and a valid or empty scene_type')
        return
    if scene not in SCENES or report['message']:
        raise ValueError('This task requires a valid scene_type and empty message')
    if not (
        isinstance(pos, list) and len(pos) == 2 and all(
            type(value) is int or (type(value) is float and math.isfinite(value))
            for value in pos
        )
    ):
        raise ValueError('This task requires a finite room/cell centre [x, y]')


class TaskSubmitter:
    """Send reports serially and remember the latest accepted result per task."""

    def __init__(self, url=DEFAULT_URL, timeout=5.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('http_timeout_sec must be positive and finite')
        self.url = url
        self.timeout = timeout
        self.accepted = {}

    def submit(self, report):
        validate_report(report)
        task_id = report['task_id']
        group_id = report['group_id']
        key = (group_id, task_id)
        result = {'group_id': group_id.split('_')[0], 'task_id': task_id}
        previous = self.accepted.get(key)
        if previous == report:
            return dict(result, status='duplicate', message='Already accepted')

        # A lost acknowledgement leaves the server's latest result uncertain.
        self.accepted.pop(key, None)
        try:
            response = requests.post(
                self.url, json=report, timeout=self.timeout, allow_redirects=False,
            )
        except requests.RequestException as exc:
            return dict(result, status='error', reason=f'HTTP request failed ({type(exc).__name__})')

        try:
            ack = response.json()
        except ValueError:
            ack = None
        if isinstance(ack, dict) and ack.get('status') == 'rejected':
            if previous is not None:
                self.accepted[key] = previous
            return dict(result, status='rejected', reason=ack.get('reason', 'Server rejected report'))
        if (
            not 200 <= response.status_code < 300 or not isinstance(ack, dict)
            or ack.get('status') != 'accepted'
            or ack.get('task_id') != task_id or ack.get('group_id') != result['group_id']
        ):
            return dict(result, status='error', reason=f'Invalid acknowledgement (HTTP {response.status_code})')

        self.accepted[key] = report
        return dict(result, status='accepted', message=ack.get('message', 'Submission recorded'))
