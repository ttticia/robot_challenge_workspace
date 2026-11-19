"""ROS entry point for the competition scoring interface."""

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import os
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from task_reporter.submission import DEFAULT_URL, TaskSubmitter


class TaskReporter(Node):
    """Receive confirmed reports and publish their submission outcomes."""

    def __init__(self):
        super().__init__('task_reporter')
        url = self.declare_parameter('server_url', DEFAULT_URL).value
        timeout = self.declare_parameter('http_timeout_sec', 5.0).value
        results_file = self.declare_parameter(
            'results_file', '~/.ros/task_reporter/results.jsonl'
        ).value
        self.results_file = Path(results_file).expanduser()
        self.results_file.parent.mkdir(parents=True, exist_ok=True)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.submitter = TaskSubmitter(url, timeout)
        self.results = self.create_publisher(String, '/mission/submission_result', 10)
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.create_subscription(String, '/mission/task_completed', self.on_report, 10)

    def on_report(self, message):
        self.worker.submit(self.submit, message.data)

    def submit(self, payload):
        try:
            report = json.loads(payload)
            sent_at = datetime.now(timezone.utc).isoformat()
            position, position_error = self.robot_position()
            outcome = self.submitter.submit(report)
        except ValueError as exc:
            outcome = {'status': 'invalid', 'reason': str(exc)}
        if outcome['status'] in ('accepted', 'rejected', 'error'):
            record = {
                'sent_at': sent_at,
                'report': report,
                'robot_position': position,
                'robot_position_error': position_error,
                'outcome': outcome,
            }
            try:
                fd = os.open(self.results_file, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
                with os.fdopen(fd, 'a', encoding='utf-8') as log:
                    log.write(json.dumps(record, ensure_ascii=False) + '\n')
            except OSError as exc:
                self.get_logger().error(f'Could not write submission results: {exc}')
        if self.context.ok():
            encoded = json.dumps(outcome, ensure_ascii=False)
            self.results.publish(String(data=encoded))
            self.get_logger().info(encoded)

    def robot_position(self):
        """Snapshot the latest robot translation before sending the report."""
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as exc:
            return None, str(exc)
        translation = transform.transform.translation
        stamp = transform.header.stamp
        return {
            'frame_id': transform.header.frame_id,
            'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
            'x': translation.x,
            'y': translation.y,
        }, None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TaskReporter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.worker.shutdown(wait=True, cancel_futures=True)
            node.destroy_node()
        rclpy.try_shutdown()
