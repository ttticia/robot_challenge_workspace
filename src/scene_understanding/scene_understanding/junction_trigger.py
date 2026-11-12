"""
Component 2: capture the current camera frame when a junction is detected.

Subscribes to:
  /camera/image_raw   (sensor_msgs/Image)  - via ImageSubscriber (component 1)
  /junction_detected  (std_msgs/Bool)      - True while/when at a junction

Publishes:
  /scene_understanding/result  (std_msgs/String)  - JSON, one per trigger;
      "scene" holds the parsed mission-relevant answer (see scene_prompt.py)

Optional: /camera/image_source (from image_folder_publisher) is used only to
record which file was analysed; with a real camera it is simply absent.

On each trigger, a copy of the latest camera frame is taken and passed to
on_junction(), which runs the VLM (Claude API) once in a background thread.
Triggers that arrive while a VLM request is still running are skipped.
The sent frame and the result are saved under save_dir.
"""

from datetime import datetime
import json
import os
import threading
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, String

from scene_understanding.image_subscriber import ImageSubscriber
from scene_understanding.scene_prompt import build_prompt, parse_scene_json
from scene_understanding.vlm_client import (
    DEFAULT_SAVE_DIR, fit_within, SourceTracker, stamp_key)

TRIGGER_MODES = ('rising_edge', 'every_true')


class TriggerLogic:
    """
    Decide which junction messages should trigger scene understanding.

    Plain Python (no ROS) so it can be unit-tested directly.

    Modes:
      rising_edge - trigger only when the value changes False -> True. Use
                    this if the detector publishes True continuously while
                    the robot is at a junction.
      every_true  - trigger on every True message. Use this if the detector
                    publishes a single True per junction.
    In both modes, triggers closer together than cooldown_sec are ignored,
    which filters out a flickering detector.
    """

    def __init__(self, mode='rising_edge', cooldown_sec=2.0):
        if mode not in TRIGGER_MODES:
            raise ValueError(
                f'trigger_mode must be one of {TRIGGER_MODES}, got "{mode}"')
        self.mode = mode
        self.cooldown_sec = cooldown_sec
        self._previous = False
        self._last_trigger_time = None

    def update(self, detected, now):
        """
        Feed one message; return True if it should trigger.

        detected: the Bool message value.
        now: current time in seconds (any monotonic clock).
        """
        previous = self._previous
        self._previous = detected

        if not detected:
            return False
        if self.mode == 'rising_edge' and previous:
            return False
        if (self._last_trigger_time is not None
                and now - self._last_trigger_time < self.cooldown_sec):
            return False
        self._last_trigger_time = now
        return True


class JunctionTrigger(ImageSubscriber):
    """Keep the latest camera frame and snapshot it at each junction."""

    def __init__(self):
        # Reuse component 1 for the camera subscription and conversion.
        super().__init__(node_name='junction_trigger')

        self.declare_parameter('junction_topic', '/junction_detected')
        self.declare_parameter('trigger_mode', 'rising_edge')
        self.declare_parameter('cooldown_sec', 2.0)
        # Refuse frames older than this: describing an old image would
        # describe the wrong place.
        self.declare_parameter('max_frame_age_sec', 2.0)

        junction_topic = self.get_parameter('junction_topic').value
        mode = self.get_parameter('trigger_mode').value
        cooldown = float(self.get_parameter('cooldown_sec').value)
        self.max_frame_age = float(
            self.get_parameter('max_frame_age_sec').value)

        self.logic = TriggerLogic(mode, cooldown)   # raises on a bad mode
        self.trigger_count = 0

        # VLM settings. enable_vlm:=false keeps the node usable without an
        # API key (capture only).
        self.declare_parameter('enable_vlm', True)
        self.declare_parameter('result_topic', '/scene_understanding/result')
        self.declare_parameter('save_dir', DEFAULT_SAVE_DIR)
        # What the mission is looking for, e.g. "a red arrow sign".
        self.declare_parameter('mission_cue', '')
        self.declare_parameter('source_topic', '/camera/image_source')
        self.sources = SourceTracker()
        self.create_subscription(
            String, self.get_parameter('source_topic').value,
            self.sources.on_message, 10)
        self.result_pub = self.create_publisher(
            String, self.get_parameter('result_topic').value, 10)
        self.vlm = None
        self.run_dir = None
        self._vlm_busy = threading.Lock()
        if self.get_parameter('enable_vlm').value:
            self._init_vlm()

        # Junction events are small and must not be lost, so use the
        # default QoS (reliable, keep last 10).
        self.junction_sub = self.create_subscription(
            Bool, junction_topic, self.junction_callback, 10)

        self.get_logger().info(
            f'Listening on {junction_topic} (mode={mode}, '
            f'cooldown={cooldown}s, max_frame_age={self.max_frame_age}s)')

    def _init_vlm(self):
        """Create the Claude client and the folder for saved results."""
        try:
            # Imported here so the node still loads without the SDK.
            from scene_understanding.vlm_api_client import ClaudeVLMClient
            self.vlm = ClaudeVLMClient(prompt=build_prompt(
                self.get_parameter('mission_cue').value))
        except (ImportError, ValueError) as e:
            self.get_logger().error(f'VLM disabled: {e}')
            return
        save_dir = self.get_parameter('save_dir').value
        if save_dir:
            self.run_dir = os.path.join(
                os.path.expanduser(save_dir),
                'junctions_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
            os.makedirs(self.run_dir, exist_ok=True)
        self.get_logger().info(
            f'VLM enabled ({self.vlm.model}); saving to '
            f'{self.run_dir or "(not saving)"}')

    def junction_callback(self, msg: Bool):
        """Run on every /junction_detected message."""
        if self.logic.update(msg.data, time.monotonic()):
            self.capture()

    def capture(self):
        """Snapshot the latest frame, if it is recent enough."""
        if self.latest_frame is None:
            self.get_logger().warning(
                'Junction detected, but no camera image has been received '
                'yet. Is the camera publishing?')
            return

        age = time.monotonic() - self.latest_receive_time
        if age > self.max_frame_age:
            self.get_logger().warning(
                f'Junction detected, but the latest image is {age:.1f}s old '
                f'(limit {self.max_frame_age}s). Skipping.')
            return

        # Copy so later camera frames cannot change the captured one.
        frame = self.latest_frame.copy()
        stamp = self.latest_stamp
        self.trigger_count += 1
        h, w = frame.shape[:2]
        source = self.sources.sources.get(stamp_key(stamp), {})
        self._source_file = source.get('file')
        self.get_logger().info(
            f'Junction #{self.trigger_count}: captured {w}x{h} frame '
            f'{self._source_file or ""} (age {age:.2f}s)')

        self.on_junction(frame, stamp)

    def on_junction(self, frame, stamp):
        """Run the VLM once on the captured frame, without blocking ROS."""
        if self.vlm is None:
            return
        # One request at a time: skip if the previous one is still running.
        if not self._vlm_busy.acquire(blocking=False):
            self.get_logger().warning(
                f'Junction #{self.trigger_count}: VLM still busy, skipping.')
            return
        threading.Thread(
            target=self._run_vlm,
            args=(frame, stamp, self.trigger_count, self._source_file),
            daemon=True).start()

    def _run_vlm(self, frame, stamp, index, source_file):
        """Background thread: call the VLM, publish and save the result."""
        try:
            result = self.vlm.describe(frame)
            scene = parse_scene_json(result['response'])
            if result['ok'] and scene is None:
                self.get_logger().warning(
                    f'Junction #{index}: VLM answer is not valid JSON; '
                    'see "response".')
            record = {
                'junction': index,
                'stamp': stamp_key(stamp),
                'source_file': source_file,
                'time': datetime.now().isoformat(timespec='seconds'),
                'scene': scene,
                **result,
            }
            if self.run_dir:
                path = os.path.join(self.run_dir, f'junction_{index:03d}.jpg')
                cv2.imwrite(path, fit_within(
                    frame, self.vlm.max_width, self.vlm.max_height))
                record['saved_frame'] = path
                with open(os.path.join(self.run_dir, 'results.jsonl'), 'a',
                          encoding='utf-8') as f:
                    f.write(json.dumps(record, ensure_ascii=False) + '\n')
            self.result_pub.publish(
                String(data=json.dumps(record, ensure_ascii=False)))
            if result['ok']:
                summary = (scene or {}).get('summary', '')
                self.get_logger().info(
                    f'Junction #{index}: VLM result in '
                    f'{result["latency_sec"]}s: {summary}')
            else:
                self.get_logger().error(
                    f'Junction #{index}: VLM failed: {result["error"]}')
        finally:
            self._vlm_busy.release()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = JunctionTrigger()
    except ValueError as e:
        rclpy.logging.get_logger('junction_trigger').error(str(e))
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
