"""Unit tests for the perception-server client, with HTTP faked."""

import cv2
import numpy as np
import pytest
import requests

from scene_understanding import vlm_client
from scene_understanding.vlm_client import fit_within, VLMClient, VLMError

ANSWER = {'response': 'A small toy kitchen with a stove and a pot.'}


class FakeResponse:
    """Minimal stand-in for requests.Response."""

    def __init__(self, status_code=200, body=None, text=None, headers=None,
                 reason='OK'):
        self.status_code = status_code
        self.reason = reason
        self._body = body
        self.text = text if text is not None else str(body)
        self.headers = headers or {}

    def json(self):
        if self._body is None:
            raise ValueError('no JSON')
        return self._body


@pytest.fixture
def frame():
    return np.full((960, 1280, 3), 127, np.uint8)


@pytest.fixture
def sent(monkeypatch):
    """Record each POST; set the reply with sent['reply']."""
    record = {'reply': FakeResponse(body=ANSWER), 'calls': 0}

    def fake_post(url, data=None, files=None, timeout=None):
        record.update(url=url, data=data, files=files, timeout=timeout)
        record['calls'] += 1
        reply = record['reply']
        if isinstance(reply, Exception):
            raise reply
        return reply

    monkeypatch.setattr(vlm_client.requests, 'post', fake_post)
    return record


def sent_image(sent):
    jpeg = sent['files']['image'][1]
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)


# ------------------------------------------------------------- the request

def test_request_matches_lab_client(sent, frame):
    VLMClient('group07', server='http://1.2.3.4:8000/',
              read_timeout=60).analyze(frame)
    assert sent['url'] == 'http://1.2.3.4:8000/analyze'
    assert sent['data'] == {'group_id': 'group07',
                            'prompt': 'Describe the image'}
    assert sent['timeout'] == (5.0, 60)
    name, _, mime = sent['files']['image']
    assert name.endswith('.jpg') and mime == 'image/jpeg'
    assert sent_image(sent).shape[:2] == (768, 1024)  # fits 1024x768


def test_custom_prompt(sent, frame):
    VLMClient('g', prompt='Is there a stove?').analyze(frame)
    assert sent['data']['prompt'] == 'Is there a stove?'


@pytest.mark.parametrize('w, h, expected_w, expected_h', [
    (1280, 960, 1024, 768),     # 4:3 landscape: exactly fills the box
    (4032, 3024, 1024, 768),
    (1254, 1254, 768, 768),     # square: limited by height
    (960, 1280, 576, 768),      # portrait: limited by height
    (1920, 1080, 1024, 576),    # 16:9: limited by width
    (640, 480, 640, 480),       # already fits: unchanged, not enlarged
])
def test_fit_within(w, h, expected_w, expected_h):
    out = fit_within(np.zeros((h, w, 3), np.uint8))
    assert out.shape[:2] == (expected_h, expected_w)


def test_fit_within_no_limit():
    big = np.zeros((3024, 4032, 3), np.uint8)
    assert fit_within(big, 0, 0).shape[:2] == (3024, 4032)
    assert fit_within(big, 0, 768).shape[:2] == (768, 1024)


def test_client_size_limits(sent):
    VLMClient('g', max_width=0, max_height=0).analyze(
        np.zeros((960, 1280, 3), np.uint8))
    assert sent_image(sent).shape[:2] == (960, 1280)
    VLMClient('g', max_width=640, max_height=640).analyze(
        np.zeros((960, 1280, 3), np.uint8))
    assert sent_image(sent).shape[:2] == (480, 640)


def test_constructor_validation():
    with pytest.raises(ValueError):
        VLMClient(' ')
    with pytest.raises(ValueError):
        VLMClient('g', prompt='   ')
    with pytest.raises(ValueError):
        VLMClient('g', read_timeout=float('inf'))


# ---------------------------------------------------------------- replies

def test_json_reply_returned_unchanged(sent, frame):
    body = {'group_id': 'g', 'result': {'text': 'hello'}, 'model': 'x'}
    sent['reply'] = FakeResponse(body=body)
    assert VLMClient('g').analyze(frame) == body


def test_text_reply_returned_as_text(sent, frame):
    sent['reply'] = FakeResponse(200, None, text='A kitchen.')
    assert VLMClient('g').analyze(frame) == 'A kitchen.'


# --------------------------------------------------------------- failures

@pytest.mark.parametrize('reply, message', [
    (requests.ConnectTimeout(), 'lab network'),
    (requests.ReadTimeout(), 'did not answer within 75.0s'),
    (requests.ConnectionError('refused'), 'Cannot reach'),
    (FakeResponse(400, {'detail': 'unknown group'}, reason='Bad Request'),
     'HTTP 400 Bad Request.*unknown group'),
    (FakeResponse(500, None, text='Internal error', reason='Server Error'),
     'Internal error'),
])
def test_analyze_errors(sent, frame, reply, message):
    sent['reply'] = reply
    with pytest.raises(VLMError, match=message):
        VLMClient('g').analyze(frame)


def test_retry_after_blocks_next_request(sent, frame):
    c = VLMClient('g')
    sent['reply'] = FakeResponse(429, {'detail': 'busy'},
                                 headers={'Retry-After': '30'},
                                 reason='Too Many Requests')
    with pytest.raises(VLMError, match='retry after 30s'):
        c.analyze(frame)
    sent['reply'] = FakeResponse(body=ANSWER)
    with pytest.raises(VLMError, match='Not sent'):
        c.analyze(frame)
    assert sent['calls'] == 1                 # second call never sent


def test_describe_never_raises(sent, frame):
    sent['reply'] = requests.ConnectionError('refused')
    result = VLMClient('g').describe(frame)
    assert result['ok'] is False and 'Cannot reach' in result['error']
    assert result['response'] is None
    assert result['latency_sec'] >= 0


def test_describe_success(sent, frame):
    result = VLMClient('g').describe(frame)
    assert result['ok'] is True and result['error'] is None
    assert result['response'] == ANSWER


# ------------------------------------------------- source / saved frames

def test_saved_frame_name():
    from scene_understanding.vlm_client import saved_frame_name
    assert saved_frame_name(3, 'kitchen.jpg') == '003_kitchen.jpg'
    assert saved_frame_name(12, '/a/b/My Photo (1).PNG') == \
        '012_My_Photo__1_.jpg'
    assert saved_frame_name(1, None) == '001_camera.jpg'


def test_stamp_key():
    from types import SimpleNamespace
    from scene_understanding.vlm_client import stamp_key
    assert stamp_key(SimpleNamespace(sec=12, nanosec=5)) == '12.000000005'


def test_source_tracker():
    import json
    from types import SimpleNamespace
    from scene_understanding.vlm_client import SourceTracker
    t = SourceTracker(keep=2)
    for i in range(3):
        t.on_message(SimpleNamespace(data=json.dumps(
            {'stamp': f'{i}.000000000', 'file': f'img{i}.jpg'})))
    t.on_message(SimpleNamespace(data='not json'))
    t.on_message(SimpleNamespace(data='{"file": "no stamp"}'))
    assert list(t.sources) == ['1.000000000', '2.000000000']
    assert t.sources['2.000000000']['file'] == 'img2.jpg'
