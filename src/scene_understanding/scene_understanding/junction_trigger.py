"""
Scene understanding triggered by junction detection.

Subscribes to:
  /camera/image_raw
      sensor_msgs/msg/Image

  /junction_detected
      std_msgs/msg/Bool

Publishes:
  /scene_understanding/result
      std_msgs/msg/String containing a scene observation or JSON null

Behaviour:
- Continuously keeps the latest camera frame.
- Waits for /junction_detected.
- Default behaviour is rising-edge triggering:
      False -> True = trigger once
      True -> True  = no new trigger
- Saves the triggered image immediately.
- Uses VLMClient from vlm_client.py.
- Sends the captured image to the Lab 6 VLM server.
- Saves the VLM response into results.jsonl.
- Publishes the result on /scene_understanding/result.
"""

from datetime import datetime
import json
import math
import os
import re
from queue import Queue
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import Bool, String

from scene_understanding.image_subscriber import ImageSubscriber
from scene_understanding.scene_prompt import (
    build_look_prompt, build_prompt, CELL_SIZE, cell_centre, extract_look_result,
    extract_scene_result, look_cell, look_cell_position, mission_reports, REPORT_CELLS,
    report_position, validate_group_id,
)

from scene_understanding.vlm_client import (
    VLMClient,
    DEFAULT_SERVER,
    DEFAULT_SAVE_DIR,
    MAX_HEIGHT,
    MAX_WIDTH,
    fit_within,
    SourceTracker,
    stamp_key,
)


TRIGGER_MODES = (
    'rising_edge',
    'every_true',
)


def look_grid(left, centre, right):
    """The three views of a look as one 2 x 2 image: left | centre over right | empty.

    The server takes at most 1024 x 768. Three 640 x 480 views side by side shrink to
    341 x 256 each; in this grid they stay 512 x 384, which is what makes a small warning
    sign or a poster readable. Every view is kept whole: a version cropped to a seamless
    panorama lost a sign that only one view showed in full.
    """
    return np.concatenate([np.concatenate([left, centre], axis=1),
                           np.concatenate([right, np.zeros_like(right)], axis=1)])


class TriggerLogic:
    """Decide whether a junction message should trigger scene understanding."""

    def __init__(self, mode='rising_edge', cooldown_sec=2.0):
        if mode not in TRIGGER_MODES:
            raise ValueError(
                f'trigger_mode must be one of {TRIGGER_MODES}, '
                f'got "{mode}"'
            )

        self.mode = mode
        self.cooldown_sec = cooldown_sec
        self._previous = False
        self._last_trigger_time = None

    def update(self, detected, now):
        """Return True when the current message should trigger."""

        previous = self._previous
        self._previous = detected

        if not detected:
            return False

        if self.mode == 'rising_edge' and previous:
            return False

        if (
            self._last_trigger_time is not None
            and now - self._last_trigger_time < self.cooldown_sec
        ):
            return False

        self._last_trigger_time = now
        return True


class JunctionTrigger(ImageSubscriber):
    """Keep the latest camera frame and analyse it when triggered."""

    on_demand = False             # camera subscribed only during explorer looks
    look_frame_window = 1.4
    look_settle = 0.0

    def __init__(self):
        # With explorer looks the camera is subscribed only during a look (see
        # explore_status_callback); decided before the base class subscribes.
        self.stream_at_start = True
        self.stream_until = 0.0
        super().__init__(node_name='junction_trigger')
        self.explore_event_topic = self.declare_parameter('explore_event_topic', '').value
        self.on_demand = bool(self.explore_event_topic) and bool(
            self.declare_parameter('subscribe_on_look', True).value)
        # s after an explorer look event in which its frame must arrive (the
        # explorer holds each view 1.5 s)
        self.look_frame_window = float(self.declare_parameter('look_frame_window', 1.4).value)
        # s after a look event before its frame is taken: the first frame after the
        # turn is still motion-blurred (10:03 run, left view)
        self.look_settle = float(self.declare_parameter('look_settle', 0.6).value)
        if self.on_demand:
            self.stop_stream()
        self.explore_pending = None
        self.explore_views = {}
        self.explore_key = None

        # ============================================================
        # Junction trigger parameters
        # ============================================================

        self.declare_parameter(
            'junction_topic',
            '/junction_detected'
        )

        self.declare_parameter(
            'trigger_mode',
            'rising_edge'
        )

        self.declare_parameter(
            'cooldown_sec',
            2.0
        )

        self.declare_parameter(
            'max_frame_age_sec',
            2.0
        )

        junction_topic = self.get_parameter(
            'junction_topic'
        ).value

        trigger_mode = self.get_parameter(
            'trigger_mode'
        ).value

        cooldown_sec = float(
            self.get_parameter(
                'cooldown_sec'
            ).value
        )

        self.max_frame_age = float(
            self.get_parameter(
                'max_frame_age_sec'
            ).value
        )

        self.logic = TriggerLogic(
            trigger_mode,
            cooldown_sec
        )

        self.trigger_count = 0

        # ============================================================
        # VLM parameters
        # ============================================================

        self.declare_parameter(
            'enable_vlm',
            True
        )

        self.declare_parameter(
            'vlm_server',
            DEFAULT_SERVER
        )

        self.declare_parameter(
            'vlm_group',
            'group04_grp4'
        )

        self.declare_parameter(
            'vlm_read_timeout',
            75.0
        )

        self.declare_parameter(
            'vlm_connect_timeout',
            5.0
        )

        # Extra tries when the server answers 5xx or not at all (its own model
        # call times out after 30 s: HTTP 504 "openai_timeout").
        self.declare_parameter('vlm_retries', 1)
        self.declare_parameter('vlm_max_width', MAX_WIDTH)
        self.declare_parameter('vlm_max_height', MAX_HEIGHT)

        # The node's own three-view turn (turn / spin / pose) was removed on 2026-11-26:
        # maze_explore makes the turns and junction_trigger takes its views.
        self.declare_parameter('junction_scan_enabled', False)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # Reported pos: 'ahead' = the cell the robot faces (a room looked into
        # from the cell before it), 'robot' = the cell it stands in.
        self.report_cell = str(self.declare_parameter('report_cell', 'ahead').value)
        self.cell_size = float(self.declare_parameter('cell_size', CELL_SIZE).value)
        if self.report_cell not in REPORT_CELLS:
            raise ValueError(f'report_cell must be one of {REPORT_CELLS}')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.get_parameter('junction_scan_enabled').value:
            raise ValueError('junction_scan_enabled: the junction scan was removed '
                             '(2026-11-26); maze_explore makes the turns')

        # ============================================================
        # Output / saving parameters
        # ============================================================

        self.declare_parameter(
            'result_topic',
            '/scene_understanding/result'
        )

        self.declare_parameter(
            'save_dir',
            DEFAULT_SAVE_DIR
        )

        self.declare_parameter(
            'source_topic',
            '/camera/image_source'
        )

        # ============================================================
        # Source tracking
        #
        # Optional: a publisher of JSON {stamp, file, path} per frame (offline replays).
        # With a real camera this may remain empty.
        # ============================================================

        self.sources = SourceTracker()

        self.create_subscription(
            String,
            self.get_parameter('source_topic').value,
            self.sources.on_message,
            10
        )

        # ============================================================
        # Result publisher
        # ============================================================

        self.result_pub = self.create_publisher(
            String,
            self.get_parameter('result_topic').value,
            10
        )
        # One message per analysed look, in the form task_reporter reads (it turns them
        # into the mission reports and posts them). '' = off.
        room_topic = self.declare_parameter('room_result_topic', '/scene/result').value
        self.room_pub = self.create_publisher(String, room_topic, 10) if room_topic else None

        # ============================================================
        # Internal state
        # ============================================================

        self.vlm = None
        self.vlm_error = 'VLM disabled'
        self.run_dir = None
        self._vlm_busy = threading.Lock()

        # Always create the save directory.
        self._init_save_directory()

        # Initialise VLM only when enabled.
        if self.get_parameter('enable_vlm').value:
            self._init_vlm()

        # ============================================================
        # Junction subscriber
        # ============================================================

        self.junction_sub = self.create_subscription(
            Bool,
            junction_topic,
            self.junction_callback,
            10
        )
        if self.explore_event_topic:
            self.explore_work = Queue()
            threading.Thread(target=self._explore_worker, daemon=True).start()
            self.explore_sub = self.create_subscription(
                String, self.explore_event_topic, self.explore_event_callback, 10,
            )
            if self.on_demand:
                self.status_sub = self.create_subscription(
                    String, self.declare_parameter('explore_status_topic', '/explore/status').value,
                    self.explore_status_callback, 10)
                self.create_timer(0.5, self._stream_watch)
                self.get_logger().info('Camera is subscribed only during explorer looks')
            self.get_logger().info(f'Listening for three-view room looks on {self.explore_event_topic}')

        self.get_logger().info(
            f'Listening on {junction_topic} '
            f'(mode={trigger_mode}, '
            f'cooldown={cooldown_sec}s, '
            f'max_frame_age={self.max_frame_age}s)'
        )

    # ================================================================
    # INITIALISATION
    # ================================================================

    def _init_save_directory(self):
        """Create a directory for triggered images and results."""

        save_dir = self.get_parameter(
            'save_dir'
        ).value

        if not save_dir:
            self.run_dir = None
            return

        self.run_dir = os.path.join(
            os.path.expanduser(save_dir),
            'junctions_'
            + datetime.now().strftime('%Y%m%d_%H%M%S')
        )

        os.makedirs(
            self.run_dir,
            exist_ok=True
        )

        self.get_logger().info(
            f'Saving junction results to {self.run_dir}'
        )

    def _init_vlm(self):
        """
        Initialise VLMClient from vlm_client.py.

        This does NOT send an image.
        The VLM request happens only after a valid junction trigger.
        """

        server = self.get_parameter(
            'vlm_server'
        ).value

        group_id = self.get_parameter(
            'vlm_group'
        ).value

        read_timeout = float(
            self.get_parameter(
                'vlm_read_timeout'
            ).value
        )

        connect_timeout = float(
            self.get_parameter(
                'vlm_connect_timeout'
            ).value
        )

        group_id = group_id.strip()

        try:
            validate_group_id(group_id)
            prompt = build_prompt()
            self.vlm = VLMClient(
                group_id,
                prompt=prompt,
                server=server,
                read_timeout=read_timeout,
                connect_timeout=connect_timeout,
                max_width=self.get_parameter('vlm_max_width').value,
                max_height=self.get_parameter('vlm_max_height').value,
            )

        except Exception as e:
            self.vlm = None
            self.vlm_error = str(e)

            self.get_logger().error(
                f'Could not initialise Lab 6 VLM: {e}'
            )

            return

        self.get_logger().info(
            f'Lab 6 VLM ready '
            f'(server={server}, automatic mission detection)'
        )

    # ================================================================
    # JUNCTION HANDLING
    # ================================================================

    def junction_callback(self, msg):
        """Handle messages received on /junction_detected."""

        triggered_at = time.monotonic()
        if self.logic.update(msg.data, triggered_at):
            self.capture(triggered_at)

    def _want_stream(self, seconds):
        """Keep the camera subscribed for at least `seconds` more."""
        if not self.on_demand:
            return
        self.stream_until = max(self.stream_until, time.monotonic() + seconds)
        if self.start_stream():
            self.get_logger().info('Camera on for an explorer look')

    def explore_status_callback(self, msg):
        """The explorer's status names its job ten times a second. A look job
        with no cells left to walk means the robot is turning to, or holding, the
        three views: the camera is subscribed for exactly that time. It starts
        with the turn to the centre view, so frames flow before the first hold."""
        if re.match(r'RUN .*job=\[look cell .* via 0 cells\]', msg.data):
            self._want_stream(2.0)

    def _stream_watch(self):
        if self.subscription is not None and time.monotonic() > self.stream_until:
            self.stop_stream()
            self.explore_pending = None
            self.get_logger().info('Camera off')

    def explore_event_callback(self, msg):
        """Associate the next RGB frame with the explorer's current look."""
        match = re.fullmatch(
            r'(room|object) (centre|left|right) cell \((-?\d+), (-?\d+)\) side ([0-3])',
            msg.data,
        )
        self.explore_pending = None
        if not match:
            return
        kind, view, i, j, side = match.groups()
        key = (kind, int(i), int(j), int(side))
        if view == 'centre':
            self.explore_key = key
            self.explore_views.clear()
        if key == self.explore_key:
            self._want_stream(3.0)       # in case the status was not heard
            self.explore_pending = (view, time.monotonic())
            self.get_logger().info(f'Explorer look received: {msg.data}')

    def image_callback(self, msg):
        super().image_callback(msg)
        if self.explore_pending is None or self.latest_receive_time is None:
            return
        view, triggered_at = self.explore_pending
        if self.latest_receive_time <= triggered_at + self.look_settle:
            return
        self.explore_pending = None
        if self.latest_receive_time - triggered_at > self.look_frame_window:
            self.get_logger().warning(f'No camera frame arrived during the explorer {view} hold')
            return
        if self.latest_frame is None:
            self.get_logger().warning(f'Camera frame for the explorer {view} view is unreadable')
            return
        self.explore_views[view] = (self.latest_frame.copy(), self.latest_stamp)
        if view == 'centre':
            self.explore_centre_pose = self._robot_position()
            self.explore_started = triggered_at
        if view != 'right':
            return
        layout = ('left', 'centre', 'right')
        if not all(panel in self.explore_views for panel in layout):
            self.get_logger().warning('Explorer look incomplete; skipping three-view request')
            return
        frames = [self.explore_views[panel][0] for panel in layout]
        if any(frame.shape != frames[0].shape for frame in frames[1:]):
            self.get_logger().warning('Camera resolution changed during explorer look')
            return
        kind, i, j, side = self.explore_key
        metadata = {'kind': kind, 'cell': [i, j], 'side': side, 'view_stamps': {
            'center' if panel == 'centre' else panel: stamp_key(self.explore_views[panel][1])
            for panel in layout
        }}
        self.trigger_count += 1
        self.explore_work.put((
            look_grid(*frames), self.explore_views['centre'][1], self.trigger_count,
            None, self.explore_started, metadata, self.explore_centre_pose, frames,
        ))
        self.explore_key = None
        self.explore_views.clear()
        self.get_logger().info(f'Explorer look #{self.trigger_count}: queued left, center, right')

    def _explore_worker(self):
        while True:
            args = self.explore_work.get()
            if args is None:
                return
            self._vlm_busy.acquire()
            if not self.context.ok():
                self._vlm_busy.release()
                return
            self._run_vlm(*args)

    def capture(self, triggered_at):
        """Take a copy of the latest camera frame."""

        if self.latest_frame is None:
            self.get_logger().warning(
                'Junction detected, but no camera image has been '
                'received yet.'
            )
            return

        age = (
            time.monotonic()
            - self.latest_receive_time
        )

        if age > self.max_frame_age:
            self.get_logger().warning(
                f'Junction detected, but latest image is '
                f'{age:.2f}s old '
                f'(limit={self.max_frame_age}s).'
            )
            return

        # Copy frame so incoming camera frames cannot modify it.
        frame = self.latest_frame.copy()

        stamp = self.latest_stamp

        self.trigger_count += 1
        index = self.trigger_count

        h, w = frame.shape[:2]

        source = self.sources.sources.get(
            stamp_key(stamp),
            {}
        )

        source_file = source.get(
            'file'
        )

        self.get_logger().info(
            f'Junction #{index}: '
            f'captured {w}x{h} frame '
            f'(age {age:.2f}s)'
        )

        self.on_junction(
            frame,
            stamp,
            index,
            source_file,
            triggered_at,
        )

    # ================================================================
    # VLM PROCESSING
    # ================================================================

    def on_junction(
        self,
        frame,
        stamp,
        index,
        source_file,
        triggered_at,
    ):
        """Run processing without blocking ROS callbacks."""

        if not self._vlm_busy.acquire(
            blocking=False
        ):
            self.get_logger().warning(
                f'Junction #{index}: '
                'VLM still busy, skipping.'
            )
            return

        thread = threading.Thread(
            target=self._run_vlm,
            args=(
                frame,
                stamp,
                index,
                source_file,
                triggered_at,
            ),
            daemon=True
        )

        thread.start()

    def _robot_position(self):
        """Read the latest robot position available in TF."""
        latest = Time()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, latest,
            )
        except TransformException as map_error:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, latest,
                )
            except TransformException as odom_error:
                return None, f'Map TF: {map_error}; odom TF: {odom_error}'
        translation = transform.transform.translation
        q = transform.transform.rotation
        transform_stamp = transform.header.stamp
        return {
            'frame_id': transform.header.frame_id,
            'stamp': {'sec': transform_stamp.sec, 'nanosec': transform_stamp.nanosec},
            'x': translation.x,
            'y': translation.y,
            'yaw_deg': round(math.degrees(math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))), 1),
        }, None

    def _report_pos(self, position):
        """Robot position (TF) -> (pos to report, robot cell centre)."""
        if position is None:
            return None, None
        x, y = position['x'], position['y']
        pos = report_position(x, y, math.radians(position['yaw_deg']),
                              self.report_cell, self.cell_size)
        return pos, cell_centre(x, y, self.cell_size)

    def _run_vlm(
        self,
        frame,
        stamp,
        index,
        source_file,
        triggered_at,
        explore_look=None,
        centre_pose=None,
        explore_frames=None,
    ):
        """
        Save the image first.

        Then call the Lab 6 VLM if available.

        Always save a JSON result, including when the VLM fails.
        """

        record = {
            'junction': index,
            'stamp': stamp_key(stamp),
            'source_file': source_file,
            'view_yaw_offsets_deg': {'center': 0.0},
            'time': datetime.now().isoformat(
                timespec='seconds'
            ),
            'scene': None,
            'ok': False,
            'response': None,
            'error': None,
        }
        # Position at the trigger (the VLM takes ~10 s).
        record['robot_position'], record['robot_position_error'] = (
            centre_pose if centre_pose is not None else self._robot_position()
        )
        record['pos'], record['robot_cell_centre'] = self._report_pos(
            record['robot_position'])
        record['report_cell'] = self.report_cell
        record['reports'] = []

        three_view = explore_look is not None
        if three_view:
            record['explore_look'] = explore_look
            record['view_layout'] = ['left', 'center', 'right']
            record['image_layout'] = 'grid: left | center / right | empty'
            # The explorer names the cell it stands in and the side it looks through, on
            # its own wall-fitted grid: that gives the looked-at cell exactly, wherever the
            # robot happens to stand.
            record['pos'] = look_cell_position(explore_look['cell'], explore_look['side'],
                                               self.cell_size)
            record['report_cell'] = 'explorer look'
        try:
            # ========================================================
            # STEP 1: SAVE TRIGGERED IMAGE FIRST
            # ========================================================

            if self.run_dir:
                image_path = os.path.join(
                    self.run_dir,
                    f'junction_{index:03d}.jpg'
                )

                if explore_frames is not None:
                    record['saved_views'] = {}
                    for panel, image in zip(record['view_layout'], explore_frames):
                        path = os.path.join(self.run_dir, f'junction_{index:03d}_{panel}.jpg')
                        if cv2.imwrite(path, image):
                            record['saved_views'][panel] = path

                if self.vlm is not None:
                    saved_frame = fit_within(
                        frame,
                        self.vlm.max_width,
                        self.vlm.max_height
                    )
                else:
                    saved_frame = frame

                save_ok = cv2.imwrite(
                    image_path,
                    saved_frame
                )

                if save_ok:
                    record['saved_frame'] = image_path

                    self.get_logger().info(
                        f'Junction #{index}: '
                        f'saved image to {image_path}'
                    )

                else:
                    self.get_logger().error(
                        f'Junction #{index}: '
                        'cv2.imwrite failed'
                    )

            # ========================================================
            # STEP 2: CHECK WHETHER VLM IS AVAILABLE
            # ========================================================

            if self.vlm is None:
                record['error'] = self.vlm_error

                self.get_logger().warning(
                    f'Junction #{index}: '
                    'image saved, but VLM is unavailable.'
                )

            else:
                # ====================================================
                # STEP 3: SEND FRAME TO LAB 6 VLM
                # ====================================================

                self.get_logger().info(
                    f'Junction #{index}: '
                    'sending frame to Lab 6 VLM...'
                )

                if explore_look is not None:
                    prompt = build_look_prompt()
                elif three_view:
                    prompt = build_prompt(three_view=True)
                else:
                    prompt = None
                for attempt in range(1 + max(0, int(self.get_parameter('vlm_retries').value))):
                    # the server refuses a request inside its cooldown (2 s per group)
                    wait = self.vlm.retry_not_before - time.monotonic()
                    if 0 < wait <= 15:
                        time.sleep(wait + 0.2)
                    result = self.vlm.describe(frame, prompt=prompt)
                    error = str(result.get('error') or '')
                    if result['ok'] or not re.search(
                            r'HTTP 5\d\d|HTTP 429|did not answer|timed out', error):
                        break
                    record.setdefault('failed_attempts', []).append(
                        {'error': error, 'latency_sec': result.get('latency_sec')})
                    self.get_logger().warning(
                        f'Junction #{index}: VLM attempt {attempt + 1} failed ({error[:80]})')

                # Copy VLM fields into our record.
                record.update(
                    result
                )

                if result['ok']:
                    response = result['response']
                    scene = (extract_look_result(response) if explore_look is not None
                             else extract_scene_result(response))
                    # Every result carries the reported cell position.
                    scene['pos'] = record['pos']
                    scene['robot_position'] = record['robot_position']
                    for match in scene.get('mission_matches') or []:
                        if isinstance(match, dict):
                            match['pos'] = record['pos']
                    record['reports'] = scene['reports'] = mission_reports(
                        scene, record['pos'])
                    record['scene'] = scene
                    # The parsed scene already contains the model output.
                    record['response'] = (
                        {key: value for key, value in response.items() if key != 'result'}
                        if isinstance(response, dict) and 'result' in response
                        else None
                    )

        except Exception as e:
            record['ok'] = False
            record['error'] = str(e)

            self.get_logger().error(
                f'Junction #{index}: '
                f'processing exception: {e}'
            )

        finally:
            record['total_latency_sec'] = round(time.monotonic() - triggered_at, 2)

            # ========================================================
            # STEP 4: ALWAYS SAVE results.jsonl
            # ========================================================

            if self.run_dir:
                results_path = os.path.join(
                    self.run_dir,
                    'results.jsonl'
                )

                try:
                    with open(
                        results_path,
                        'a',
                        encoding='utf-8'
                    ) as f:
                        f.write(
                            json.dumps(
                                record,
                                ensure_ascii=False
                            )
                            + '\n'
                        )

                    self.get_logger().info(
                        f'Junction #{index}: '
                        'result saved to results.jsonl'
                    )

                except OSError as e:
                    self.get_logger().error(
                        f'Could not write results.jsonl: {e}'
                    )

            # ========================================================
            # STEP 5: PUBLISH RESULT
            # ========================================================

            try:
                result_msg = String()

                result_msg.data = json.dumps(
                    record['scene'],
                    ensure_ascii=False
                )

                self.result_pub.publish(
                    result_msg
                )

            except Exception as e:
                self.get_logger().error(
                    f'Could not publish result: {e}'
                )

            if record.get('ok') and explore_look is not None and self.room_pub is not None:
                scene = record['scene']
                self.room_pub.publish(String(data=json.dumps({
                    'cell': list(look_cell(explore_look['cell'], explore_look['side'])),
                    'cell_centre': record['pos'],
                    'scene_type': scene['scene_type'],
                    'objects': scene['objects'],
                    'hazards': [scene['hazard']] if scene['hazard'] else [],
                    'humans': scene['survivor'],
                    'qr': bool(scene.get('qr')), 'qr_view': scene.get('qr_view', ''),
                    'vlm': 'lab', 'look': explore_look['kind'], 'junction': index,
                })))
            elif explore_look is not None and self.room_pub is not None:
                # No answer for this look: say so, so that the explorer (which waits for
                # the answer before it moves on) and task_reporter need not wait it out.
                self.room_pub.publish(String(data=json.dumps({
                    'cell': list(look_cell(explore_look['cell'], explore_look['side'])),
                    'vlm': 'none', 'look': explore_look['kind'], 'junction': index,
                })))

            # ========================================================
            # STEP 6: TERMINAL LOG
            # ========================================================

            if record.get('ok') and explore_look is not None:
                scene = record['scene']
                found = [m['label'] or m['kind'] for m in scene['mission_matches']]
                self.get_logger().info(
                    f'Junction #{index}: {explore_look["kind"]} look at cell pos {record["pos"]}: '
                    f'scene {scene["scene_type"]}, found {", ".join(found) or "nothing"} '
                    f'({record.get("latency_sec")}s VLM, {record["total_latency_sec"]}s total)')
            elif record.get('ok'):
                summary = ', '.join(
                    f'{path["direction"]}={path["status"]}'
                    for path in record['scene']['paths']
                )
                self.get_logger().info(
                    f'Junction #{index}: {summary} '
                    f'({record.get("latency_sec")}s VLM, {record["total_latency_sec"]}s total)'
                )

            else:
                self.get_logger().warning(
                    f'Junction #{index}: '
                    f'VLM unsuccessful: '
                    f'{record.get("error")}'
                )

            self._vlm_busy.release()


def main(args=None):
    # Own SIGINT/SIGTERM handling instead of rclpy's: the context must stay up
    # after Ctrl+C so a running pose look can still leave Pose mode.
    stop = threading.Event()
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    node = None

    try:
        node = JunctionTrigger()

        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    except ExternalShutdownException:
        pass

    except Exception as e:
        rclpy.logging.get_logger(
            'junction_trigger'
        ).error(
            str(e)
        )

    finally:
        if node is not None:
            if node.explore_event_topic:
                node.explore_work.put(None)
            node.destroy_node()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
