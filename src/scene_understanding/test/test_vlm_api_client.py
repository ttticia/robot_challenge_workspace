"""Unit tests for the Claude API client, with the Anthropic client faked."""

import base64
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from scene_understanding import vlm_api_client
from scene_understanding.vlm_api_client import (
    ClaudeVLMClient, DEFAULT_MODEL, FALLBACK_BETA)

# Skip only this file until the SDK is installed (pip install anthropic).
# (A module-level pytest.importorskip skips the whole run on pytest 6.)
pytestmark = pytest.mark.skipif(
    vlm_api_client.anthropic is None, reason='anthropic SDK not installed')


def text_block(text):
    return SimpleNamespace(type='text', text=text)


def make_response(blocks, stop_reason='end_turn', model=DEFAULT_MODEL,
                  stop_details=None):
    return SimpleNamespace(content=blocks, stop_reason=stop_reason,
                           model=model, stop_details=stop_details)


class FakeAnthropic:
    """Records create() calls; returns self.reply or raises it."""

    def __init__(self, reply):
        self.reply = reply
        self.calls = []
        self.beta = SimpleNamespace(
            messages=SimpleNamespace(create=self._create))

    def _create(self, **kwargs):
        self.calls.append(kwargs)
        if isinstance(self.reply, Exception):
            raise self.reply
        return self.reply


@pytest.fixture
def frame():
    return np.full((960, 1280, 3), 127, np.uint8)


def client_with(reply, **kwargs):
    fake = FakeAnthropic(reply)
    return ClaudeVLMClient(client=fake, **kwargs), fake


def test_request_shape(frame):
    c, fake = client_with(make_response([text_block('A kitchen.')]))
    c.analyze(frame)
    req = fake.calls[0]
    assert req['model'] == 'claude-opus-5-5'
    assert req['output_config'] == {'effort': 'medium'}
    assert req['betas'] == [FALLBACK_BETA]
    assert req['fallbacks'] == 'default'
    assert req['max_tokens'] == 16000
    image, text = req['messages'][0]['content']
    assert text == {'type': 'text', 'text': 'Describe the image'}
    assert image['source']['media_type'] == 'image/jpeg'
    jpeg = base64.b64decode(image['source']['data'])
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (768, 1024)


def test_custom_prompt_model_effort(frame):
    c, fake = client_with(make_response([text_block('ok')]),
                          prompt='Is there a stove?', model='m', effort='low')
    c.analyze(frame)
    req = fake.calls[0]
    assert req['messages'][0]['content'][1]['text'] == 'Is there a stove?'
    assert req['model'] == 'm' and req['output_config'] == {'effort': 'low'}


def test_text_blocks_joined_and_non_text_ignored(frame):
    blocks = [SimpleNamespace(type='thinking', thinking=''),
              text_block('Line one.'), text_block('Line two.')]
    c, _ = client_with(make_response(blocks, model='claude-opus-4-8'))
    assert c.analyze(frame) == ('Line one.\nLine two.', 'claude-opus-4-8')


def test_refusal_is_an_error(frame):
    c, _ = client_with(make_response(
        [], stop_reason='refusal',
        stop_details=SimpleNamespace(category='cyber')))
    result = c.describe(frame)
    assert result['ok'] is False and 'declined' in result['error']
    assert 'cyber' in result['error']


def test_max_tokens_marks_cut_off(frame):
    c, _ = client_with(make_response([text_block('Long')],
                                     stop_reason='max_tokens'))
    assert 'cut off' in c.analyze(frame)[0]


def test_empty_answer_is_an_error(frame):
    c, _ = client_with(make_response([]))
    assert c.describe(frame)['ok'] is False


def test_missing_credentials(frame):
    c, _ = client_with(TypeError('Could not resolve authentication method'))
    result = c.describe(frame)
    assert result['ok'] is False and 'ANTHROPIC_API_KEY' in result['error']


def test_describe_success(frame):
    c, _ = client_with(make_response([text_block('A toy kitchen.')]))
    result = c.describe(frame)
    assert result['ok'] is True and result['error'] is None
    assert result['response'] == 'A toy kitchen.'
    assert result['model'] == DEFAULT_MODEL
    assert result['latency_sec'] >= 0


def test_blank_prompt_rejected():
    with pytest.raises(ValueError):
        ClaudeVLMClient(prompt='  ', client=FakeAnthropic(None))
