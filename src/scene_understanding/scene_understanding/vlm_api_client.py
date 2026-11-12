"""
Component 3 (Claude API version): describe camera frames with Claude.

Same job as vlm_client.py (the lab-server version), but the VLM is Claude,
called through the official Anthropic Python SDK over the internet.

Setup (once):
    sudo apt install python3-pip
    pip install anthropic
    export ANTHROPIC_API_KEY=sk-ant-...      # e.g. in ~/.bashrc

Frames are shrunk to fit within 1024x768 (keeping the aspect ratio) and sent
as JPEG together with the prompt. ClaudeVLMClient has no ROS dependencies;
the command-line test is a small ROS node that takes frames from
/camera/image_raw:
    ros2 run scene_understanding vlm_api_test --prompt "Describe the image"
"""

import argparse
import base64
import sys
import time

try:
    import anthropic
except ImportError:          # SDK not installed yet; main() explains how
    anthropic = None

from scene_understanding.vlm_client import (
    add_camera_test_args, DEFAULT_PROMPT, encode_jpeg, MAX_HEIGHT, MAX_WIDTH,
    parse_camera_test_args, run_camera_test, VLMError)

DEFAULT_MODEL = 'claude-opus-5-5'
# Effort controls how much the model thinks before answering:
# low | medium | high | xhigh | max. Lower is faster and cheaper.
DEFAULT_EFFORT = 'medium'
# If Claude's safety classifiers decline a request, the API re-runs it on a
# recommended fallback model instead of returning a refusal.
FALLBACK_BETA = 'server-side-fallback-2026-07-01'
INSTALL_HELP = ('The anthropic Python package is not installed. Install it '
                'with:\n  sudo apt install python3-pip\n  pip install anthropic')


def root_cause(error):
    """
    Describe the lowest-level exception behind an SDK connection error.

    "Connection error" alone hides whether DNS, SSL, a proxy or the network
    failed; the original exception says which.
    """
    seen, cause = set(), error
    while True:
        nxt = cause.__cause__ or cause.__context__
        if nxt is None or id(nxt) in seen:
            break
        seen.add(id(nxt))
        cause = nxt
    if cause is error:
        return str(error)
    return f'{type(cause).__name__}: {cause}'


class ClaudeVLMClient:
    """Send camera frames to Claude and return its description."""

    def __init__(self, prompt=DEFAULT_PROMPT, model=DEFAULT_MODEL,
                 effort=DEFAULT_EFFORT, timeout=120.0, max_retries=2,
                 max_width=MAX_WIDTH, max_height=MAX_HEIGHT, client=None):
        """
        Create a client (does not contact the API yet).

        prompt: the question sent with every image.
        model: Claude model id.
        effort: low | medium | high | xhigh | max.
        timeout: seconds to wait for one API request.
        max_retries: automatic retries on rate limits, server errors and
            network errors (done by the SDK, with backoff).
        max_width, max_height: frames are shrunk to fit inside this box.
        client: an existing anthropic.Anthropic client (mainly for tests).
            By default one is created, reading the API key from the
            ANTHROPIC_API_KEY environment variable.
        """
        if client is None and anthropic is None:
            raise ImportError(INSTALL_HELP)
        if not str(prompt).strip():
            raise ValueError('prompt must not be blank.')
        self.prompt = prompt
        self.model = model
        self.effort = effort
        self.max_width = max_width
        self.max_height = max_height
        self.client = client or anthropic.Anthropic(
            timeout=timeout, max_retries=max_retries)

    def analyze(self, frame):
        """
        Describe a BGR numpy image; return (text, served_by_model).

        Raises VLMError on any failure. This blocks for several seconds, so
        a ROS node must not call it directly in a callback.
        """
        image_b64 = base64.standard_b64encode(
            encode_jpeg(frame, self.max_width, self.max_height)
        ).decode('ascii')

        try:
            response = self.client.beta.messages.create(
                model=self.model,
                max_tokens=16000,
                output_config={'effort': self.effort},
                betas=[FALLBACK_BETA],
                fallbacks='default',
                messages=[{
                    'role': 'user',
                    'content': [
                        {'type': 'image',
                         'source': {'type': 'base64',
                                    'media_type': 'image/jpeg',
                                    'data': image_b64}},
                        {'type': 'text', 'text': self.prompt},
                    ],
                }],
            )
        except anthropic.AuthenticationError:
            raise VLMError('Invalid or missing API key. Set ANTHROPIC_API_KEY.')
        except anthropic.PermissionDeniedError as e:
            raise VLMError(f'API key not allowed to do this: {e.message}')
        except anthropic.NotFoundError as e:
            raise VLMError(f'Model "{self.model}" not found: {e.message}')
        except anthropic.RateLimitError as e:
            retry = e.response.headers.get('retry-after', '?')
            raise VLMError(f'Rate limited by the API; retry after {retry}s.')
        except anthropic.BadRequestError as e:
            raise VLMError(f'Bad request: {e.message}')
        except anthropic.APIStatusError as e:
            raise VLMError(f'API error {e.status_code}: {e.message}')
        except anthropic.APITimeoutError as e:
            raise VLMError(
                f'The API did not answer in time ({root_cause(e)}).')
        except anthropic.APIConnectionError as e:
            raise VLMError(
                f'Cannot reach the Anthropic API ({root_cause(e)}). '
                'Check the internet connection, proxy and VPN settings.')
        except TypeError as e:
            # The SDK raises TypeError when it finds no credentials at all.
            raise VLMError(f'No API credentials found ({e}). '
                           'Set ANTHROPIC_API_KEY.')

        if response.stop_reason == 'refusal':
            details = response.stop_details
            category = getattr(details, 'category', None) if details else None
            raise VLMError(f'The model declined this request '
                           f'(category: {category}).')

        text = '\n'.join(block.text for block in response.content
                         if block.type == 'text').strip()
        if response.stop_reason == 'max_tokens':
            text += ' [answer cut off: max_tokens reached]'
        if not text:
            raise VLMError(
                f'Empty answer (stop_reason: {response.stop_reason}).')
        return text, response.model

    def describe(self, frame):
        """
        Analyze a frame and never raise; for use from a ROS node.

        Returns a dict:
          ok (bool), error (str or None), latency_sec (float),
          response (Claude's text answer, or None on failure),
          model (the model that answered; differs from the requested one
          if a fallback ran)
        """
        start = time.monotonic()
        try:
            (response, model), error = self.analyze(frame), None
        except VLMError as e:
            response, model, error = None, None, str(e)
        return {
            'ok': error is None,
            'error': error,
            'latency_sec': round(time.monotonic() - start, 2),
            'response': response,
            'model': model,
        }


def main(argv=None):
    """
    Command-line test: send frames from the camera topic to Claude.

    Same options as vlm_test, with --model/--effort instead of
    --server/--group.
    """
    parser = argparse.ArgumentParser(
        description='Send camera frames to Claude and print its answer.')
    parser.add_argument('--model', default=DEFAULT_MODEL)
    parser.add_argument('--effort', default=DEFAULT_EFFORT,
                        choices=['low', 'medium', 'high', 'xhigh', 'max'])
    parser.add_argument('--timeout', type=float, default=120.0)
    add_camera_test_args(parser)
    args, full_argv = parse_camera_test_args(parser, argv)

    if anthropic is None:
        print(INSTALL_HELP, file=sys.stderr)
        return 1
    try:
        client = ClaudeVLMClient(args.prompt, model=args.model,
                                 effort=args.effort, timeout=args.timeout,
                                 max_width=args.max_width,
                                 max_height=args.max_height)
    except ValueError as e:
        print(f'Error: {e}', file=sys.stderr)
        return 1
    return run_camera_test(client, args, full_argv, 'vlm_api_test',
                           client.model)


if __name__ == '__main__':
    sys.exit(main())
