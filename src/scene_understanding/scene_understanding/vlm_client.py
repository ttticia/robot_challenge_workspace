"""
Component 3: VLM client for the lab perception server.

Sends a camera frame (numpy BGR image from the camera subscription) plus a
text prompt to the LAN perception server and returns its answer.

Server API (same as the lab's client.py):
    POST {server}/analyze   multipart form:
        group_id  your group id
        prompt    the question, e.g. "Describe the image"
        image     the image as a JPEG file
    Busy/rate-limited replies may carry a Retry-After header (seconds).

Frames are shrunk to fit within 1024x768 (keeping the aspect ratio) before
sending.

VLMClient has no ROS dependencies. The command-line test (main) is a small
ROS node that takes frames from /camera/image_raw and sends them:
    ros2 run scene_understanding vlm_test --server http://192.168.10.141:8000 \
        --group groupxx --prompt "Describe the image"

fit_within(), encode_jpeg() and the camera test helpers below are shared
with vlm_api_client.py (the Claude API version).
"""

import argparse
from collections import OrderedDict
from datetime import datetime
import json
import math
import os
import sys
import time

import cv2
import requests

DEFAULT_SERVER = 'http://192.168.10.141:8000'
DEFAULT_PROMPT = 'Describe the image'
# Frames sent to the VLM are shrunk to fit inside this box.
MAX_WIDTH = 1024
MAX_HEIGHT = 768
# Where the camera tests save the frames they send (one subfolder per run).
DEFAULT_SAVE_DIR = '~/ir/prompted_frames'


class VLMError(Exception):
    """The server could not produce a result (unreachable, busy, bad reply)."""


def fit_within(frame, max_width=MAX_WIDTH, max_height=MAX_HEIGHT):
    """
    Shrink a numpy image to fit inside max_width x max_height.

    Keeps the aspect ratio and never enlarges. 1280x960 -> 1024x768,
    1254x1254 -> 768x768, portrait 960x1280 -> 576x768. A limit of 0 means
    no limit on that side.
    """
    h, w = frame.shape[:2]
    scales = [1.0]
    if max_width > 0:
        scales.append(max_width / w)
    if max_height > 0:
        scales.append(max_height / h)
    scale = min(scales)
    if scale >= 1.0:
        return frame
    new_size = (max(1, round(w * scale)), max(1, round(h * scale)))
    return cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)


def encode_jpeg(frame, max_width=MAX_WIDTH, max_height=MAX_HEIGHT,
                quality=90):
    """Fit a BGR numpy image within the size limits; return JPEG bytes."""
    ok, buf = cv2.imencode('.jpg', fit_within(frame, max_width, max_height),
                           [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise VLMError('Could not JPEG-encode the image.')
    return buf.tobytes()


class VLMClient:
    """Send camera frames to the lab perception server."""

    def __init__(self, group_id, prompt=DEFAULT_PROMPT, server=DEFAULT_SERVER,
                 read_timeout=75.0, connect_timeout=5.0,
                 max_width=MAX_WIDTH, max_height=MAX_HEIGHT):
        """
        Create a client (does not contact the server yet).

        group_id: your group id, sent with every request.
        prompt: the question sent with every image.
        server: base URL of the perception server.
        read_timeout: seconds to wait for the answer (model inference).
        connect_timeout: seconds to wait for the connection itself.
        max_width, max_height: frames are shrunk to fit inside this box
            (aspect ratio kept). 0 = no limit on that side.
        """
        if not str(group_id).strip():
            raise ValueError('group_id must not be blank.')
        if not str(prompt).strip():
            raise ValueError('prompt must not be blank.')
        if not math.isfinite(read_timeout) or read_timeout <= 0:
            raise ValueError('read_timeout must be finite and positive.')
        self.group_id = group_id
        self.prompt = prompt
        self.server = server.rstrip('/')
        self.read_timeout = read_timeout
        self.connect_timeout = connect_timeout
        self.max_width = max_width
        self.max_height = max_height
        # time.monotonic() before which the server asked us not to call.
        self.retry_not_before = 0.0

    def analyze(self, frame):
        """
        Send a BGR numpy image to /analyze and return the server's answer.

        Returns the reply parsed as JSON, or the reply text if it is not
        JSON. Raises VLMError on any failure. This blocks for up to
        read_timeout seconds, so a ROS node must not call it directly in a
        callback.
        """
        wait = self.retry_not_before - time.monotonic()
        if wait > 0:
            raise VLMError(
                f'Server asked to retry later; {wait:.0f}s left. Not sent.')

        jpeg = encode_jpeg(frame, self.max_width, self.max_height)
        try:
            r = requests.post(
                f'{self.server}/analyze',
                data={'group_id': self.group_id, 'prompt': self.prompt},
                files={'image': ('frame.jpg', jpeg, 'image/jpeg')},
                timeout=(self.connect_timeout, self.read_timeout))
        except requests.ConnectTimeout:
            raise VLMError(
                f'Could not connect to {self.server} within '
                f'{self.connect_timeout}s. Are you on the lab network?')
        except requests.Timeout:
            raise VLMError(
                f'Server did not answer within {self.read_timeout}s.')
        except requests.RequestException as e:
            raise VLMError(f'Cannot reach {self.server}: {e}')

        if r.status_code >= 400:
            message = f'HTTP {r.status_code} {r.reason}'
            try:
                message += f': {json.dumps(r.json(), ensure_ascii=False)}'
            except ValueError:
                if r.text.strip():
                    message += f': {r.text.strip()[:300]}'
            retry_after = r.headers.get('Retry-After')
            if retry_after:
                try:
                    self.retry_not_before = (
                        time.monotonic() + float(retry_after))
                except ValueError:
                    pass
                message += f' (retry after {retry_after}s)'
            raise VLMError(message)

        try:
            return r.json()
        except ValueError:
            return r.text

    def describe(self, frame):
        """
        Analyze a frame and never raise; for use from a ROS node.

        Returns a dict:
          ok (bool), error (str or None), latency_sec (float),
          response (the server's answer, or None on failure)
        """
        start = time.monotonic()
        try:
            response, error = self.analyze(frame), None
        except VLMError as e:
            response, error = None, str(e)
        return {
            'ok': error is None,
            'error': error,
            'latency_sec': round(time.monotonic() - start, 2),
            'response': response,
        }


def wait_for_new_frame(node, after, timeout):
    """
    Spin the node until a frame arrives later than `after` (monotonic time).

    Returns True if one arrived within `timeout` seconds.
    """
    import rclpy
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.latest_receive_time is not None
                and node.latest_receive_time > after):
            # Frames may have queued up while a request was running; process
            # them so latest_frame is really the newest one.
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0)
            return True
    return False


def add_camera_test_args(parser):
    """Add the command-line options shared by vlm_test and vlm_api_test."""
    parser.add_argument('--prompt', default=DEFAULT_PROMPT)
    parser.add_argument('--max-width', type=int, default=MAX_WIDTH,
                        help='frames are shrunk to fit this width (0 = any)')
    parser.add_argument('--max-height', type=int, default=MAX_HEIGHT,
                        help='frames are shrunk to fit this height (0 = any)')
    parser.add_argument('--count', type=int, default=1,
                        help='number of frames to send (default 1)')
    parser.add_argument('--frame-timeout', type=float, default=10.0,
                        help='seconds to wait for a camera frame')
    parser.add_argument('--save-dir', default=DEFAULT_SAVE_DIR,
                        help='each run saves the frames it sends, plus '
                             'results.jsonl, in a new subfolder here '
                             '("" = do not save)')
    parser.add_argument('--source-topic', default='/camera/image_source',
                        help='topic with the file name of each image '
                             '(published by image_folder_publisher)')


def parse_camera_test_args(parser, argv=None):
    """
    Parse our options; return (args, full argv for rclpy.init).

    argparse sees only our options; rclpy gets the full list so ROS options
    (e.g. --ros-args -p image_topic:=/x) still work.
    """
    from rclpy.utilities import remove_ros_args
    full_argv = sys.argv if argv is None else argv
    return parser.parse_args(remove_ros_args(full_argv)[1:]), full_argv


def stamp_key(stamp):
    """Format a ROS time stamp as "<sec>.<nanosec>" (used to match files)."""
    return f'{stamp.sec}.{stamp.nanosec:09d}'


def saved_frame_name(index, source_file):
    """File name for the index-th sent frame, e.g. 003_kitchen.jpg."""
    stem = os.path.splitext(os.path.basename(source_file))[0] \
        if source_file else 'camera'
    stem = ''.join(c if c.isalnum() or c in '-_.' else '_' for c in stem)
    return f'{index:03d}_{stem}.jpg'


class SourceTracker:
    """
    Remember which file each frame came from, keyed by its stamp.

    Fed by /camera/image_source messages from image_folder_publisher. With a
    real camera nothing is published there and lookups return None.
    """

    def __init__(self, keep=200):
        self.sources = OrderedDict()
        self.keep = keep

    def on_message(self, msg):
        try:
            info = json.loads(msg.data)
            key = info['stamp']
        except (ValueError, KeyError, TypeError):
            return
        self.sources[key] = info
        while len(self.sources) > self.keep:
            self.sources.popitem(last=False)

    def lookup(self, node, key, timeout=0.5):
        """Return the source info for a stamp, waiting briefly if needed."""
        import rclpy
        if not self.sources:          # no folder publisher: don't wait
            return None
        deadline = time.monotonic() + timeout
        while key not in self.sources and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        return self.sources.get(key)


def run_camera_test(client, args, full_argv, node_name, target):
    """
    Send frames from the camera topic to `client` and print each result.

    Each result is printed together with the source image it came from. If
    --save-dir is set, the exact frame sent is saved as NNN_<source>.jpg in
    a new run folder, and a line is appended to results.jsonl there.

    client: any object with describe(frame) -> dict containing 'ok'.
    target: text for the log, e.g. the server URL or model name.
    Returns a process exit code (0 if every request succeeded).
    """
    # ROS imports here so the VLM clients stay usable without ROS.
    import rclpy
    from std_msgs.msg import String
    from scene_understanding.image_subscriber import ImageSubscriber

    run_dir = None
    if args.save_dir:
        run_dir = os.path.join(
            os.path.expanduser(args.save_dir),
            datetime.now().strftime(f'{node_name}_%Y-%m-%d_%H-%M-%S'))
        os.makedirs(run_dir, exist_ok=True)

    rclpy.init(args=full_argv)
    node = ImageSubscriber(node_name=node_name)
    # The rate log is misleading here: the node does not process messages
    # while a request is running, so queued frames arrive in a burst.
    node.rate_timer.cancel()
    tracker = SourceTracker()
    node.create_subscription(String, args.source_topic, tracker.on_message, 10)
    log = node.get_logger()
    if run_dir:
        log.info(f'Saving sent frames and results.jsonl to {run_dir}')
    failures = 0
    last_sent = 0.0
    try:
        for i in range(1, args.count + 1):
            if not wait_for_new_frame(node, last_sent, args.frame_timeout):
                log.error(f'No camera frame within {args.frame_timeout}s. '
                          'Is the camera (image_folder_publisher) running?')
                failures += 1
                break
            frame = node.latest_frame.copy()
            stamp = node.latest_stamp
            last_sent = node.latest_receive_time
            key = stamp_key(stamp)
            source = tracker.lookup(node, key) or {}
            source_file = source.get('file')

            h, w = frame.shape[:2]
            sent = fit_within(frame, args.max_width, args.max_height)
            sh, sw = sent.shape[:2]
            log.info(f'[{i}/{args.count}] Sending '
                     f'{source_file or "camera frame"} ({w}x{h} -> {sw}x{sh},'
                     f' stamp {key}) to {target} with prompt '
                     f'"{args.prompt}"')

            record = {
                'index': i,
                'time': datetime.now().isoformat(timespec='seconds'),
                'source_file': source_file,
                'source_path': source.get('path'),
                'stamp': key,
                'original_size': f'{w}x{h}',
                'sent_size': f'{sw}x{sh}',
                'saved_frame': None,
                'target': target,
                'prompt': args.prompt,
            }
            if run_dir:
                path = os.path.join(run_dir, saved_frame_name(i, source_file))
                cv2.imwrite(path, sent)
                record['saved_frame'] = path

            record.update(client.describe(frame))
            print(json.dumps(record, indent=2, ensure_ascii=False),
                  flush=True)
            if run_dir:
                with open(os.path.join(run_dir, 'results.jsonl'), 'a',
                          encoding='utf-8') as f:
                    f.write(json.dumps(record, ensure_ascii=False) + '\n')
            if not record['ok']:
                failures += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if failures else 0


def main(argv=None):
    """
    Command-line test: send frames from the camera topic to the lab server.

    Waits for a new frame on /camera/image_raw, sends it, prints the answer,
    and repeats --count times. Arguments match the lab client.py, except the
    image comes from the camera subscription instead of --image.
    """
    parser = argparse.ArgumentParser(
        description='Send camera frames to the lab perception server.')
    parser.add_argument('--server', default=DEFAULT_SERVER)
    parser.add_argument('--group', required=True)
    parser.add_argument('--read-timeout', type=float, default=75.0)
    add_camera_test_args(parser)
    args, full_argv = parse_camera_test_args(parser, argv)

    try:
        client = VLMClient(args.group, args.prompt, server=args.server,
                           read_timeout=args.read_timeout,
                           max_width=args.max_width,
                           max_height=args.max_height)
    except ValueError as e:
        print(f'Error: {e}', file=sys.stderr)
        return 1
    return run_camera_test(client, args, full_argv, 'vlm_test',
                           client.server)


if __name__ == '__main__':
    sys.exit(main())
