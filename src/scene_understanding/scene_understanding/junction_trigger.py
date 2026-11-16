"""
Scene understanding triggered by junction detection.

Subscribes to:
  /camera/image_raw
      sensor_msgs/msg/Image

  /junction_detected
      std_msgs/msg/Bool

Publishes:
  /scene_understanding/result
      std_msgs/msg/String containing JSON

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
    VLMClient,
    DEFAULT_SERVER,
    DEFAULT_SAVE_DIR,
    fit_within,
    SourceTracker,
    stamp_key,
)


TRIGGER_MODES = (
    'rising_edge',
    'every_true',
)


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

    def __init__(self):
        super().__init__(node_name='junction_trigger')

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

        self.declare_parameter(
            'mission_cue',
            ''
        )

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
        # Mainly useful with image_folder_publisher.
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

        # ============================================================
        # Internal state
        # ============================================================

        self.vlm = None
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

        mission_cue = self.get_parameter(
            'mission_cue'
        ).value

        prompt = build_prompt(
            mission_cue
        )

        try:
            self.vlm = VLMClient(
                group_id,
                prompt=prompt,
                server=server,
                read_timeout=read_timeout,
                connect_timeout=connect_timeout,
            )

        except Exception as e:
            self.vlm = None

            self.get_logger().error(
                f'Could not initialise Lab 6 VLM: {e}'
            )

            return

        self.get_logger().info(
            f'Lab 6 VLM ready '
            f'(server={server}, group={group_id})'
        )

    # ================================================================
    # JUNCTION HANDLING
    # ================================================================

    def junction_callback(self, msg):
        """Handle messages received on /junction_detected."""

        should_trigger = self.logic.update(
            msg.data,
            time.monotonic()
        )

        if should_trigger:
            self.capture()

    def capture(self):
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
            source_file
        )

    # ================================================================
    # VLM PROCESSING
    # ================================================================

    def on_junction(
        self,
        frame,
        stamp,
        index,
        source_file
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
                source_file
            ),
            daemon=True
        )

        thread.start()

    def _run_vlm(
        self,
        frame,
        stamp,
        index,
        source_file
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
            'time': datetime.now().isoformat(
                timespec='seconds'
            ),
            'scene': None,
            'ok': False,
            'response': None,
            'error': None,
        }

        try:
            # ========================================================
            # STEP 1: SAVE TRIGGERED IMAGE FIRST
            # ========================================================

            if self.run_dir:
                image_path = os.path.join(
                    self.run_dir,
                    f'junction_{index:03d}.jpg'
                )

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
                record['error'] = (
                    'VLM disabled or unavailable'
                )

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

                result = self.vlm.describe(
                    frame
                )

                # Copy VLM fields into our record.
                record.update(
                    result
                )

                response = result.get(
                    'response'
                )

                scene = None

                # ====================================================
                # IMPORTANT:
                #
                # Lab 6 may return response as either:
                #
                #   dict
                #
                # OR
                #
                #   JSON/string text
                #
                # Do NOT call parse_scene_json() on a dict.
                # ====================================================

                if result.get('ok'):
                    if isinstance(response, dict):
                        scene = response

                    elif isinstance(response, str):
                        scene = parse_scene_json(
                            response
                        )

                    elif response is None:
                        self.get_logger().warning(
                            f'Junction #{index}: '
                            'VLM returned no response.'
                        )

                    else:
                        self.get_logger().warning(
                            f'Junction #{index}: '
                            'unexpected response type: '
                            f'{type(response).__name__}'
                        )

                record['scene'] = scene

                # If it was a successful request but parsing failed,
                # preserve the raw response.
                if (
                    result.get('ok')
                    and scene is None
                    and response is not None
                ):
                    self.get_logger().warning(
                        f'Junction #{index}: '
                        'response could not be parsed into scene JSON; '
                        'raw response is still saved.'
                    )

        except Exception as e:
            record['ok'] = False
            record['error'] = str(e)

            self.get_logger().error(
                f'Junction #{index}: '
                f'processing exception: {e}'
            )

        finally:
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
                    record,
                    ensure_ascii=False
                )

                self.result_pub.publish(
                    result_msg
                )

            except Exception as e:
                self.get_logger().error(
                    f'Could not publish result: {e}'
                )

            # ========================================================
            # STEP 6: TERMINAL LOG
            # ========================================================

            if record.get('ok'):
                scene = record.get(
                    'scene'
                )

                summary = ''

                if isinstance(scene, dict):
                    summary = scene.get(
                        'summary',
                        ''
                    )

                self.get_logger().info(
                    f'Junction #{index}: '
                    f'VLM successful '
                    f'({record.get("latency_sec")}s) '
                    f'{summary}'
                )

            else:
                self.get_logger().warning(
                    f'Junction #{index}: '
                    f'VLM unsuccessful: '
                    f'{record.get("error")}'
                )

            self._vlm_busy.release()


def main(args=None):
    rclpy.init(
        args=args
    )

    node = None

    try:
        node = JunctionTrigger()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

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
            node.destroy_node()

        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
