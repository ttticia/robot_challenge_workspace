"""task_reporter: room results and QR reads -> the five mission reports (booklet section E,
missions as fixed on 2026-11-24; the mapping is in missions.py).

  in : /scene/result   (room_scene, JSON) per analysed room: cell, scene_type, objects,
                       hazards, humans. Results with vlm 'none' (not analysed) are skipped.
       /qr/read        (qr_scanner) the decoded text, passed on exactly as decoded
       /scene/capture  (explorer, JSON) the room the robot is looking at: a QR read within
                       `qr_cell_window` s of a capture belongs to that room
  out: /mission/task_completed (std_msgs/String, the booklet's JSON), one message per report.
       FOR NOW (user, 2026-11-24) only the QR report goes out, and only on this topic:
       `missions` = 'M04' and `post` = false. `missions:=all` sends all five; `post:=true`
       with a `group_id` also sends each one by HTTP POST to `report_url`.
       A report goes out again when its content changes (the server scores the latest valid
       one per task_id), for example M05 becoming M03 once that room's QR code is read.

    ros2 run scene_understanding task_reporter
"""
import json
import queue
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from scene_understanding.missions import Missions


class TaskReporter(Node):
    def __init__(self):
        super().__init__('task_reporter')
        p = self.declare_parameter
        p('scene_result_topic', '/scene/result')
        p('qr_read_topic', '/qr/read')
        p('scene_capture_topic', '/scene/capture')
        p('task_completed_topic', '/mission/task_completed')
        p('qr_cell_window', 30.0)      # s after a room capture a QR read belongs to it
        p('qr_wait', 15.0)             # s a survivor's room waits for its QR code before M05
        p('cell_size', 1.02)
        p('group_id', '')              # group##_XXXX; empty = nothing is posted
        p('report_url', 'http://192.168.10.141:8001/task_report')
        p('post', False)               # HTTP POST to report_url (needs group_id)
        p('missions', 'M04')           # which reports go out: 'all' or e.g. 'M04' / 'M01,M04'
        p('post_timeout', 3.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.qr_window = float(g('qr_cell_window'))
        self.url, self.timeout = g('report_url'), float(g('post_timeout'))
        self.posting = bool(g('post')) and bool(g('group_id'))
        want = str(g('missions')).replace(' ', '')
        self.only = None if want.lower() == 'all' else set(want.split(','))
        self.missions = Missions(g('group_id'), float(g('cell_size')), float(g('qr_wait')))
        self.pub = self.create_publisher(String, g('task_completed_topic'), 10)
        self.create_subscription(String, g('scene_result_topic'), self.on_scene_result, 10)
        if g('qr_read_topic'):
            self.create_subscription(String, g('qr_read_topic'), self.on_qr_read, 10)
        if g('scene_capture_topic'):
            self.create_subscription(String, g('scene_capture_topic'), self.on_capture, 10)
        self.last_capture = None       # (cell, monotonic)
        self.outbox = queue.Queue()
        threading.Thread(target=self.post_loop, daemon=True).start()
        self.create_timer(1.0, self.flush)
        which = 'all missions' if self.only is None else ', '.join(sorted(self.only)) + ' only'
        self.get_logger().info(f'reporting {which} on {g("task_completed_topic")}' + (
            f' and by POST to {self.url} as {g("group_id")}' if self.posting else ' (no POST)'))

    def flush(self):
        for payload in self.missions.reports(time.monotonic()):
            if self.only is not None and payload['task_id'] not in self.only:
                continue
            self.pub.publish(String(data=json.dumps(payload)))
            self.get_logger().info(f'report: {payload}')
            if self.posting:
                self.outbox.put(payload)

    def post_loop(self):
        while True:
            payload = self.outbox.get()
            for attempt in (1, 2, 3):
                try:
                    req = urllib.request.Request(
                        self.url, json.dumps(payload).encode(),
                        {'Content-Type': 'application/json'}, method='POST')
                    with urllib.request.urlopen(req, timeout=self.timeout) as r:
                        answer = r.read().decode(errors='replace')
                    log = self.get_logger().info if '"accepted"' in answer else self.get_logger().error
                    log(f'{payload["task_id"]} server: {answer.strip()[:200]}')
                    break
                except urllib.error.HTTPError as e:      # answered, but refused
                    self.get_logger().error(f'{payload["task_id"]} rejected: HTTP {e.code} '
                                            f'{e.read().decode(errors="replace")[:200]}')
                    break
                except Exception as e:                   # no answer: try again
                    self.get_logger().warn(f'{payload["task_id"]} post failed (try {attempt} '
                                           f'of 3): {e}')
                    time.sleep(1.0)

    def on_capture(self, msg):
        try:
            self.last_capture = (tuple(json.loads(msg.data)['cell']), time.monotonic())
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
        for note in self.missions.scene(cell, r.get('scene_type'), r.get('objects'),
                                        r.get('hazards'), r.get('humans'), time.monotonic()):
            self.get_logger().warn(note)
        self.flush()

    def on_qr_read(self, msg):
        if not msg.data.strip():
            return
        cell = None
        if self.last_capture is not None and \
                time.monotonic() - self.last_capture[1] <= self.qr_window:
            cell = self.last_capture[0]
        self.missions.qr(msg.data, cell)       # the text exactly as decoded
        self.flush()


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
