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
import threading
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import Bool, String

from scene_understanding.image_subscriber import ImageSubscriber
from scene_understanding.scene_prompt import (
    build_prompt, extract_scene_result, validate_group_id,
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

        self.declare_parameter('vlm_max_width', MAX_WIDTH)
        self.declare_parameter('vlm_max_height', MAX_HEIGHT)

        self.declare_parameter('junction_scan_enabled', False)
        self.declare_parameter('junction_scan_angle_deg', 30.0)
        self.declare_parameter('junction_spin_action', '/spin')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan = None
        if self.get_parameter('junction_scan_enabled').value:
            from scene_understanding.junction_scan import JunctionScan

            self.scan = JunctionScan(
                self,
                self.get_parameter('junction_spin_action').value,
                self.get_parameter('junction_scan_angle_deg').value,
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
        transform_stamp = transform.header.stamp
        return {
            'frame_id': transform.header.frame_id,
            'stamp': {'sec': transform_stamp.sec, 'nanosec': transform_stamp.nanosec},
            'x': translation.x,
            'y': translation.y,
        }, None

    def _run_vlm(
        self,
        frame,
        stamp,
        index,
        source_file,
        triggered_at,
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

        three_view = False
        scan_error = None
        try:
            if self.scan is not None:
                started = time.monotonic()
                composite = None
                try:
                    composite = self.scan.capture(frame)
                except Exception as exc:
                    scan_error = exc
                finally:
                    record['scan_latency_sec'] = round(time.monotonic() - started, 2)
                if composite is None:
                    record['view_layout'] = ['center']
                    if scan_error is None:
                        record['scan_warning'] = (
                            f'Nav2 Spin action {self.scan.action_name} unavailable; used center frame'
                        )
                        self.get_logger().warning(record['scan_warning'])
                else:
                    frame = composite
                    three_view = True
                    record['view_layout'] = ['left', 'center', 'right']
                    angle_deg = round(math.degrees(self.scan.angle), 6)
                    record['view_yaw_offsets_deg'].update(left=angle_deg, right=-angle_deg)

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

            if scan_error is not None:
                # The robot may have turned partway, so this frame is diagnostic only.
                raise scan_error

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

                result = (
                    self.vlm.describe(frame, prompt=build_prompt(three_view=True))
                    if three_view else self.vlm.describe(frame)
                )

                # Copy VLM fields into our record.
                record.update(
                    result
                )

                if result['ok']:
                    response = result['response']
                    record['scene'] = extract_scene_result(response)
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
            record['robot_position'], record['robot_position_error'] = self._robot_position()

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

            # ========================================================
            # STEP 6: TERMINAL LOG
            # ========================================================

            if record.get('ok'):
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
