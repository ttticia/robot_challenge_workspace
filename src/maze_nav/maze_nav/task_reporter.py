"""task_reporter: room_scene results and QR reads -> /mission/task_completed.

  in : /scene/result  (room_scene, JSON) objects / hazards per checked dead end; results
                      with vlm 'none' (not analysed) are logged and skipped
       /qr/read       (qr_scanner, plain text, every read) QR text
       /scene/capture (maze_explorer, JSON) the dead end the robot is looking at: a QR read
                      within `qr_cell_window` s of a capture gets that cell (QR codes are only
                      in survivor rooms, and are read at dead ends)
  out: /mission/task_completed  std_msgs/String JSON, at most once per (type, what, cell):
       {"type": "object" | "hazard", "what", "cell", "cell_centre", "scene_type"}
       {"type": "qr", "text", "cell", "cell_centre"}   (cell None when not at a dead end)
Survivors: not reported (poster_detector was removed 2026-11-19; nothing detects them).

    ros2 run maze_nav task_reporter
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TaskReporter(Node):
    def __init__(self):
        super().__init__('task_reporter')
        p = self.declare_parameter
        p('scene_result_topic', '/scene/result')
        p('qr_read_topic', '/qr/read')
        p('scene_capture_topic', '/scene/capture')
        p('task_completed_topic', '/mission/task_completed')
        p('qr_cell_window', 30.0)      # s after a dead-end capture a QR read belongs to it
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.qr_window = float(g('qr_cell_window'))
        self.pub = self.create_publisher(String, g('task_completed_topic'), 10)
        self.create_subscription(String, g('scene_result_topic'), self.on_scene_result, 10)
        if g('qr_read_topic'):
            self.create_subscription(String, g('qr_read_topic'), self.on_qr_read, 10)
        if g('scene_capture_topic'):
            self.create_subscription(String, g('scene_capture_topic'), self.on_capture, 10)
        self.reported = set()          # (type, what, cell)
        self.last_capture = None       # (cell, cell_centre, monotonic)
        self.get_logger().info(f'reporting on {g("task_completed_topic")}')

    def report(self, key, payload):
        if key in self.reported:
            return
        self.reported.add(key)
        self.pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'reported: {payload}')

    def on_capture(self, msg):
        try:
            c = json.loads(msg.data)
            self.last_capture = (list(c['cell']), c.get('cell_centre'), time.monotonic())
        except (ValueError, KeyError, TypeError):
            pass

    def on_scene_result(self, msg):
        try:
            r = json.loads(msg.data)
            cell = tuple(r['cell'])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn(f'bad /scene/result: {msg.data[:200]}')
            return
        if r.get('vlm') in (None, 'none'):
            self.get_logger().info(f'cell {list(cell)}: not analysed (no VLM); nothing to '
                                   'report')
            return
        for kind, key in (('object', 'objects'), ('hazard', 'hazards')):
            for what in r.get(key) or []:
                self.report((kind, str(what), cell),
                            {'type': kind, 'what': what, 'cell': list(cell),
                             'cell_centre': r.get('cell_centre'),
                             'scene_type': r.get('scene_type', 'unknown')})

    def on_qr_read(self, msg):
        text = msg.data.strip()
        if not text:
            return
        cell = centre = None
        if self.last_capture is not None and \
                time.monotonic() - self.last_capture[2] <= self.qr_window:
            cell, centre = self.last_capture[0], self.last_capture[1]
        self.report(('qr', text, None), {'type': 'qr', 'text': text, 'cell': cell,
                                         'cell_centre': centre})


def main():
    rclpy.init()
    node = TaskReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
