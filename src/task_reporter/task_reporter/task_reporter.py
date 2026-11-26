"""task_reporter: room results and QR reads -> the five mission reports (booklet section E,
missions as fixed on 2026-11-24; the mapping is in missions.py).

A package of its own (user, 2026-11-24): scene understanding and the QR scanner both work
during a room look and each sends what it found here; this node decides which mission it
is, checks the format (missions.check) and submits. It imports nothing from them.

  in : /scene/result   (junction_trigger or room_scene, JSON) per analysed room: cell,
                       scene_type, objects, hazards, humans. Results with vlm 'none' (not
                       analysed) are skipped.
       /qr/read        (team_perception qr_scanner) the decoded text, passed on exactly as
                       decoded. The QR report (M04) goes out once the VLM
                       (junction_trigger) has given the scene type of the room the code
                       was read in: no VLM result for that room = no M04.
       /explore/event  (maze_explorer) `room|object centre cell (i, j) side d`: the room the
                       robot is looking into. A QR read within `qr_cell_window` s of it
                       belongs to that room (which makes its survivor M03).
       /explore/status (maze_explorer) while its job is a look: the cell looked into, as
                       above. Also the cell the robot stands in: the last resort for a QR
                       read when no look is known at all (the scanner only reads during
                       looks, so this should not happen in a run).
       /scene/capture  (old explorer, JSON with `cell`) same use as /explore/event
  out: /mission/sightings (latched JSON list): everything seen so far, cell by cell,
       whether a mission asks for it or not.
       /mission/task_completed (std_msgs/String), one message per report, and with
       `post:=true` + a `group_id` the same JSON by HTTP POST to `report_url`
       (practice server, live since 2026-11-24: .../task_report_A or _B: the course follows maze_side, L = A, R = B, user 2026-11-26).
       Every report has the seven keys and a position, M04 too (user, 2026-11-24).
       `missions` picks which go out: 'all' (default) or e.g. 'M04'.
       A report goes out again when its content changes (the server scores the latest valid
       one per task_id), for example M03 moving to the room the QR code is read in.

    ros2 run task_reporter task_reporter
"""
import json
import os
import queue
import sys
import re
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from task_reporter.missions import check, course_url, look_cell, Missions


class TaskReporter(Node):
    def __init__(self):
        super().__init__('task_reporter')
        p = self.declare_parameter
        p('scene_result_topic', '/scene/result')
        p('qr_read_topic', '/qr/read')
        p('scene_capture_topic', '/scene/capture')
        p('explore_event_topic', '/explore/event')
        p('explore_status_topic', '/explore/status')
        p('task_completed_topic', '/mission/task_completed')
        p('qr_cell_window', 30.0)      # s after a room capture a QR read belongs to it
        p('qr_wait', 15.0)             # s a survivor's room waits for its QR code before it is sent
        p('cell_size', 1.02)
        p('group_id', '')              # group##_XXXX; empty = nothing is posted
        # The course is not fixed (user, 2026-11-26): it follows maze_side, L = course A,
        # R = course B (missions.course_url). A report_url ending in _A / _B overrides it.
        p('report_url', 'http://192.168.10.141:8001/task_report')
        p('maze_side', '')             # L or R, from explore.launch.py
        p('post', False)               # HTTP POST to report_url (needs group_id)
        p('missions', 'all')           # which reports go out: 'all' or e.g. 'M04' / 'M01,M04'
        p('m01_object', 'hand_tools')  # the object mission M01 asks for (the mission given)
        p('m02_hazard', '')            # the hazard M02 asks for; '' = whichever sign is found
        p('post_timeout', 3.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.qr_window = float(g('qr_cell_window'))
        self.url, self.timeout = course_url(g('report_url'), g('maze_side')), float(g('post_timeout'))
        self.posting = bool(g('post')) and bool(g('group_id')) and bool(self.url)
        if bool(g('post')) and not self.url:
            self.get_logger().error(
                f"no course: maze_side is '{g('maze_side')}' (need L or R) and report_url "
                f"'{g('report_url')}' names none. NOTHING WILL BE POSTED")
        else:
            self.get_logger().info(f"reports go to {self.url} (maze_side '{g('maze_side')}')")
        want = str(g('missions')).replace(' ', '')
        self.only = None if want.lower() == 'all' else set(want.split(','))
        self.missions = Missions(g('group_id'), float(g('cell_size')), float(g('qr_wait')),
                                 g('m01_object'), g('m02_hazard'))
        self.pub = self.create_publisher(String, g('task_completed_topic'), 10)
        # What the server said to each posted report (user, 2026-11-26), JSON: task_id,
        # ok (accepted and, where the reply carries an evaluation, every field right),
        # reply. The explorer counts a mission as done only on ok.
        self.result_pub = self.create_publisher(String, '/mission/result', 10)
        # Every POST and what came back, retries included (user, 2026-11-26), JSON: time,
        # task_id, url, attempt, sent (the payload), http (status code, null = no answer),
        # reply (in full). Latched, so `ros2 run task_reporter report_watch` started late
        # still shows the earlier ones.
        self.server_pub = self.create_publisher(String, '/mission/server_log', QoSProfile(
            depth=200, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.seen_pub = self.create_publisher(String, '/mission/sightings', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, g('scene_result_topic'), self.on_scene_result, 10)
        if g('qr_read_topic'):
            self.create_subscription(String, g('qr_read_topic'), self.on_qr_read, 10)
        if g('scene_capture_topic'):
            self.create_subscription(String, g('scene_capture_topic'), self.on_capture, 10)
        if g('explore_event_topic'):
            self.create_subscription(String, g('explore_event_topic'), self.on_event, 10)
        if g('explore_status_topic'):
            self.create_subscription(String, g('explore_status_topic'), self.on_status, 10)
        self.last_capture = None       # (cell, monotonic)
        self.robot_cell = None
        self.outbox = queue.Queue()
        # Reports the server could not take yet (user, 2026-11-26 night; 10:39 run: every
        # report refused with HTTP 409 "No active competition slot"). They are NOT a wrong
        # answer: the sighting is kept and the latest report per mission is sent again
        # every retry_period s until the server takes it. A newer report for the same
        # mission replaces the waiting one.
        self.retry_period = float(p('retry_period', 5.0).value)
        self.waiting, self.waiting_lock = {}, threading.Lock()
        threading.Thread(target=self.post_loop, daemon=True).start()
        threading.Thread(target=self.retry_loop, daemon=True).start()
        self.create_timer(1.0, self.flush)
        which = 'all missions' if self.only is None else ', '.join(sorted(self.only)) + ' only'
        self.get_logger().info(f'M01 = {g("m01_object")}, M02 = {g("m02_hazard") or "any hazard sign"}')
        self.get_logger().info(f'reporting {which} on {g("task_completed_topic")}' + (
            f' and by POST to {self.url} as {g("group_id")}' if self.posting else ' (no POST)'))

    def flush(self):
        for payload in self.missions.reports(time.monotonic()):
            if self.only is not None and payload['task_id'] not in self.only:
                continue
            self.pub.publish(String(data=json.dumps(payload)))
            self.get_logger().info(f'report: {payload}')
            if not self.posting:
                continue
            valid, why = check(payload)
            if valid:
                with self.waiting_lock:          # a newer report replaces a waiting one
                    if payload['task_id'] in self.waiting:
                        self.waiting[payload['task_id']] = payload
                self.outbox.put(payload)
            else:
                self.get_logger().error(f'{payload["task_id"]} not posted: {why}')

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
                    self.exchange(payload, attempt, r.status, answer)
                    log = self.get_logger().info if '"accepted"' in answer else self.get_logger().error
                    log(f'{payload["task_id"]} server: {answer.strip()}')
                    self.result(payload, successful(answer), answer)
                    with self.waiting_lock:
                        if self.waiting.get(payload['task_id']) == payload:
                            del self.waiting[payload['task_id']]
                    break
                except urllib.error.HTTPError as e:      # answered, but refused
                    answer = e.read().decode(errors='replace')
                    self.exchange(payload, attempt, e.code, answer)
                    if not_now(e.code, answer):
                        self.hold_back(payload, e.code, answer)
                        break
                    self.get_logger().error(f'{payload["task_id"]} rejected: HTTP {e.code} '
                                            f'{answer.strip()}')
                    self.result(payload, False, answer)
                    break
                except Exception as e:                   # no answer: try again
                    self.exchange(payload, attempt, None, f'no answer: {e}')
                    self.get_logger().warn(f'{payload["task_id"]} post failed (try {attempt} '
                                           f'of 3): {e}')
                    time.sleep(1.0)
            else:
                self.get_logger().error(f'{payload["task_id"]} NOT delivered after 3 tries: '
                                        f'{payload}')
                self.result(payload, False, 'not delivered', refuse=False)

    def exchange(self, payload, attempt, code, answer):
        """One POST and its reply, on /mission/server_log (for report_watch)."""
        self.server_pub.publish(String(data=json.dumps({
            'time': time.time(), 'task_id': payload.get('task_id'), 'url': self.url,
            'attempt': attempt, 'sent': payload, 'http': code, 'reply': answer})))

    def hold_back(self, payload, code, answer):
        """The server cannot take reports just now: keep this one and send it again."""
        task = payload['task_id']
        with self.waiting_lock:
            first = task not in self.waiting
            self.waiting[task] = payload
        if first:
            self.get_logger().warn(f'{task} not taken now (HTTP {code} {answer.strip()[:120]}): '
                                   f'kept, sent again every {self.retry_period:.0f} s until it is')
        self.result(payload, False, answer, refuse=False)

    def retry_loop(self):
        while True:
            time.sleep(self.retry_period)
            with self.waiting_lock:
                again = list(self.waiting.values())
            for payload in again:
                self.outbox.put(payload)

    def result(self, payload, ok, answer, refuse=True):
        if not ok and refuse:
            self.missions.refused(payload)
            self.get_logger().warn(f'{payload["task_id"]} not marked successful by the server')
        self.result_pub.publish(String(data=json.dumps(
            {'task_id': payload['task_id'], 'ok': bool(ok), 'reply': answer.strip()[:300]})))

    def on_capture(self, msg):
        try:
            self.last_capture = (tuple(json.loads(msg.data)['cell']), time.monotonic())
        except (ValueError, KeyError, TypeError):
            pass

    def on_event(self, msg):
        # every view of a look (centre, left, right) is of the same cell, the one looked into
        m = re.fullmatch(r'(?:room|object) (?:centre|left|right) cell \((-?\d+), (-?\d+)\) '
                         r'side ([0-3])', msg.data)
        if m:
            i, j, side = (int(v) for v in m.groups())
            self.last_capture = (look_cell((i, j), side), time.monotonic())

    def on_status(self, msg):
        m = re.search(r'cell=\((-?\d+), (-?\d+)\)', msg.data)
        if m:
            self.robot_cell = (int(m.group(1)), int(m.group(2)))
        # While the explorer is on a look (turning to it or holding it) the cell it looks
        # into is known from the status alone, 10 times a second: a QR code read then is
        # placed in THAT cell even if the look's events were missed (user, 2026-11-25:
        # positions are of the cell the robot is looking at, never of where it stands).
        m = re.match(r'RUN .*job=\[look cell \((-?\d+), (-?\d+)\) side ([0-3]) via 0 cells\]',
                     msg.data)
        if m:
            i, j, side = (int(v) for v in m.groups())
            self.last_capture = (look_cell((i, j), side), time.monotonic())

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
                                        r.get('hazards'), r.get('humans'), time.monotonic(),
                                        qr=bool(r.get('qr'))):
            self.get_logger().warn(note)
        # everything seen so far, submitted or not (latched: a late listener gets it too)
        self.seen_pub.publish(String(data=json.dumps(self.missions.sightings())))
        self.flush()

    def on_qr_read(self, msg):
        if not msg.data.strip():
            return
        cell = None
        if self.last_capture is not None and \
                time.monotonic() - self.last_capture[1] <= self.qr_window:
            cell = self.last_capture[0]
        self.missions.qr(msg.data, cell, self.robot_cell)       # the text exactly as decoded
        self.flush()


def not_now(code, answer):
    """An HTTP refusal that is about the server's state, not the report: try again later.
    409 'No active competition slot' (10:39 run, 2026-11-26), and server errors."""
    return code == 409 or code >= 500 or 'slot' in (answer or '').lower()


def successful(answer):
    """The server's reply marks the report as successful: status accepted and, where it
    carries an evaluation (practice mode), every evaluated field right. The competition
    reply has no evaluation: accepted is all there is to go by."""
    try:
        r = json.loads(answer)
    except ValueError:
        return False
    if not isinstance(r, dict) or r.get('status') != 'accepted':
        return False
    marks = r.get('evaluation')
    return all(marks.values()) if isinstance(marks, dict) else True


def main():
    rclpy.init()
    node = TaskReporter()
    # Say why it stops: on 2026-11-24 11:37:00 it ended in the middle of a run, cleanly and
    # without a word, and nobody could tell what had interrupted it.
    try:
        rclpy.spin(node)
        why = 'spin returned'
    except KeyboardInterrupt:
        why = 'interrupted (SIGINT)'
    except ExternalShutdownException:
        why = 'ROS was shut down from outside (SIGINT or SIGTERM to this process)'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'[task_reporter] stopping: {why}; parent pid {os.getppid()}', file=sys.stderr,
          flush=True)


if __name__ == '__main__':
    main()
