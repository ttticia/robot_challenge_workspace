"""room_scene: the camera views of each checked dead end -> one room record for the VLM.

maze_explorer publishes /scene/capture (std_msgs/String JSON) at the end of each hold of a
dead-end look, while still facing the view: the hold facing into the dead end ('centre'),
then each side wall ('left', 'right'; Pose-mode body yaw):
    {cell, cell_centre, view, last, stamp, hold_start, robot, frame,
     tilt_deg, roll_deg, pitch_cmd_deg, camera_pitch_deg}
The explorer never waits for this node.

Frames: the compressed colour topic, the last `buffer_s` s kept as received (nothing is
decoded). For each capture the frame stamped inside [hold_start, stamp] nearest its end is
taken. Camera stamps come from the robot's clock: if none falls inside, the newest frame
that ARRIVED during the hold is taken instead (logged 'by arrival'), else the view has no
frame. A room is complete on `last`, or `room_timeout` s after its newest view (the look was
cut short: stairs, a curtain). Each (cell, attempt) is analysed once; later captures of it
are ignored. `attempt` (default 1) is the explorer's try. With no /scene/result within its
scene_result_timeout (and the cell still a dead end with no object on the map) the explorer
asks again on /scene/retry {cell, attempt: 2}: the views already captured for that cell are
sent to the VLM again as attempt 2 (the robot has usually moved on; no new look). Captures
tagged attempt 2 are analysed afresh too. At most `max_attempts` (2) tries per cell go to
the VLM; later retries / captures are ignored.

Complete rooms go to a worker thread through a queue, so a slow VLM never drops a room.
VLM: NOT wired yet (`vlm` = 'none'): the record is saved and published with
scene_type 'unknown' and vlm 'none', which task_reporter ignores. Saved to
`save_dir`/<YYYY-MM-DD>/<HHMMSS>_cell_<i>_<j>_<view>.jpg + rooms.jsonl.

  /scene/result  std_msgs/String JSON: {cell, cell_centre, attempt, kind (dead_end | room: an
                 object cell looked into from outside), views: [{view, frame, how,
                 tilt_deg, pitch_cmd_deg, camera_pitch_deg}], complete, scene_type,
                 hazards, objects, signs, humans, vlm}

Off by default (maze_nav.launch.py scene_understanding:=false): it is one more colour
subscriber, i.e. one more copy of the camera stream over the robot bridge.

    ros2 run scene_understanding room_scene
"""
import collections
import datetime
import json
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def stamp_ns(text):
    """'<sec>.<nanosec>' (the explorer's stamp strings) -> int ns."""
    sec, _, frac = str(text).partition('.')
    return int(sec) * 10**9 + int((frac + '000000000')[:9])


def pick_frame(frames, start_ns, end_ns, arr_lo=None, arr_hi=None):
    """frames: [(stamp_ns, arrival_s, msg)], oldest first. (msg, how): the frame stamped in
    [start_ns, end_ns] nearest end_ns ('stamp'); else the newest that arrived in
    [arr_lo, arr_hi] ('arrival'); else (None, None)."""
    inside = [f for f in frames if start_ns <= f[0] <= end_ns]
    if inside:
        return max(inside, key=lambda f: f[0])[2], 'stamp'
    if arr_lo is not None:
        arrived = [f for f in frames if arr_lo <= f[1] <= arr_hi]
        if arrived:
            return arrived[-1][2], 'arrival'
    return None, None


class RoomScene(Node):
    def __init__(self):
        super().__init__('room_scene')
        p = self.declare_parameter
        p('scene_capture_topic', '/scene/capture')
        p('camera_topic', '/camera/color/image_raw/compressed')
        p('scene_result_topic', '/scene/result')
        p('scene_retry_topic', '/scene/retry')
        p('buffer_s', 4.0)            # s of camera frames kept
        p('room_timeout', 20.0)        # s after a room's newest view without `last`
        p('save_dir', os.path.expanduser('~/challenge_ws/logs/scene'))
        p('vlm', 'none')               # VLM backend; only 'none' (not wired yet)
        p('max_attempts', 2)           # VLM tries per cell (the explorer retries on no reply)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.max_attempts = int(g('max_attempts'))
        self.buffer_s, self.room_timeout = float(g('buffer_s')), float(g('room_timeout'))
        self.save_dir, self.vlm = g('save_dir'), g('vlm')

        self.frames = collections.deque(maxlen=400)   # (stamp_ns, arrival_s, msg)
        self.rooms = {}                # (cell, attempt) -> {'cell', 'cell_centre', 'attempt',
        #                                'views', 'updated'}
        self.done = set()              # (cell, attempt) already sent to the worker
        self.sent = {}                 # cell -> its newest room sent (views kept for a retry)
        self.work = queue.Queue()
        self.result_pub = self.create_publisher(String, g('scene_result_topic'), 10)
        self.create_subscription(String, g('scene_capture_topic'), self.on_capture, 10)
        if g('scene_retry_topic'):
            self.create_subscription(String, g('scene_retry_topic'), self.on_retry, 10)
        self.create_subscription(CompressedImage, g('camera_topic'), self.on_image,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.check_timeouts)
        threading.Thread(target=self.worker, daemon=True).start()
        if self.vlm != 'none':
            self.get_logger().warn(f'vlm={self.vlm!r}: no such backend yet; using none')
            self.vlm = 'none'
        self.get_logger().info(f'captures on {g("scene_capture_topic")}, frames from '
                               f'{g("camera_topic")}, results on {g("scene_result_topic")}; '
                               f'saving to {self.save_dir}; VLM: {self.vlm} (not wired yet)')

    def on_image(self, msg):
        now = time.monotonic()
        s = msg.header.stamp
        self.frames.append((s.sec * 10**9 + s.nanosec, now, msg))
        while self.frames and now - self.frames[0][1] > self.buffer_s:
            self.frames.popleft()

    def on_capture(self, msg):
        try:
            c = json.loads(msg.data)
            cell = (int(c['cell'][0]), int(c['cell'][1]))
            start, end = stamp_ns(c['hold_start']), stamp_ns(c['stamp'])
            attempt = int(c.get('attempt') or 1)
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(f'bad /scene/capture ({e}): {msg.data[:200]}')
            return
        key = (cell, attempt)
        if attempt > self.max_attempts:
            self.get_logger().warn(f'cell {list(cell)} {c.get("view")}: attempt {attempt} > '
                                   f'max_attempts {self.max_attempts}; not sent to the VLM')
            return
        if key in self.done:
            self.get_logger().info(f'cell {list(cell)} {c.get("view")} (attempt {attempt}): '
                                   'already analysed; ignored')
            return
        now = time.monotonic()
        frame, how = pick_frame(self.frames, start, end, now - (end - start) / 1e9 - 0.2, now)
        if frame is None:
            self.get_logger().warn(f'cell {list(cell)} {c.get("view")}: no camera frame from '
                                   'the hold (camera topic silent?)')
        elif how == 'arrival':
            self.get_logger().info(f'cell {list(cell)} {c.get("view")}: no frame stamped in '
                                   'the hold (robot clock offset?); took one by arrival')
        room = self.rooms.setdefault(key, {'cell': list(cell), 'attempt': attempt,
                                           'cell_centre': c.get('cell_centre'), 'views': []})
        room['updated'] = now
        room['views'].append({'view': c.get('view'), 'msg': frame, 'how': how,
                              'capture': c})
        self.get_logger().info(f'cell {list(cell)}: view {c.get("view")} '
                               f'({len(room["views"])} so far)'
                               + (', last: queued' if c.get('last') else ''))
        if c.get('last'):
            self.finish(key, complete=True)

    def check_timeouts(self):
        now = time.monotonic()
        for key in [k for k, r in self.rooms.items() if now - r['updated'] > self.room_timeout]:
            self.get_logger().warn(f'cell {list(key[0])} (attempt {key[1]}): no last view for '
                                   f'{self.room_timeout:.0f} s (look cut short); queued as is')
            self.finish(key, complete=False)

    def finish(self, key, complete):
        room = self.rooms.pop(key)
        room['complete'] = complete
        self.done.add(key)
        self.sent[key[0]] = room
        self.work.put(room)

    def on_retry(self, msg):
        """/scene/retry {cell, attempt}: no answer for that cell in time; send the views
        already captured to the VLM again as `attempt`."""
        try:
            r = json.loads(msg.data)
            cell = (int(r['cell'][0]), int(r['cell'][1]))
            attempt = int(r.get('attempt') or 2)
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(f'bad /scene/retry ({e}): {msg.data[:200]}')
            return
        if attempt > self.max_attempts:
            self.get_logger().warn(f'cell {list(cell)}: retry attempt {attempt} > max_attempts '
                                   f'{self.max_attempts}; not sent to the VLM')
            return
        if (cell, attempt) in self.done or (cell, attempt) in self.rooms:
            self.get_logger().info(f'cell {list(cell)}: attempt {attempt} already queued; '
                                   'retry ignored')
            return
        prev = self.sent.get(cell)
        if prev is None:
            self.get_logger().warn(f'cell {list(cell)}: retry {attempt}, but no views of it '
                                   'were captured; nothing to send')
            return
        room = dict(prev, attempt=attempt, views=list(prev['views']))
        self.done.add((cell, attempt))
        self.sent[cell] = room
        self.get_logger().info(f'cell {list(cell)}: no answer in time; sending its '
                               f'{len(room["views"])} view(s) again (attempt {attempt})')
        self.work.put(room)

    def worker(self):
        while True:
            room = self.work.get()
            try:
                self.process(room)
            except Exception as e:        # keep the queue alive whatever one room does
                self.get_logger().error(f'cell {room["cell"]}: processing failed: {e!r}')

    def process(self, room):
        day = datetime.datetime.now()
        folder = os.path.join(self.save_dir, day.strftime('%Y-%m-%d'))
        os.makedirs(folder, exist_ok=True)
        views = []
        for v in room['views']:
            name = None
            if v['msg'] is not None:
                ext = 'png' if 'png' in (v['msg'].format or '') else 'jpg'
                name = (f'{day.strftime("%H%M%S")}_cell_{room["cell"][0]}_{room["cell"][1]}'
                        + (f'_try{room["attempt"]}' if room['attempt'] > 1 else '')
                        + f'_{v["view"]}.{ext}')
                with open(os.path.join(folder, name), 'wb') as f:
                    f.write(bytes(v['msg'].data))
            cap = v['capture']
            views.append({'view': v['view'], 'frame': name, 'how': v['how'],
                          **{k: cap.get(k) for k in ('tilt_deg', 'pitch_cmd_deg',
                                                     'camera_pitch_deg', 'robot')}})
        kind = (room['views'][0]['capture'].get('kind') if room['views'] else None) or 'dead_end'
        result = {'cell': room['cell'], 'cell_centre': room['cell_centre'],
                  'attempt': room['attempt'], 'kind': kind,
                  'views': views,
                  'complete': room['complete'], 'scene_type': 'unknown', 'hazards': [],
                  'objects': [], 'signs': [], 'humans': False, 'vlm': self.vlm}
        with open(os.path.join(folder, 'rooms.jsonl'), 'a') as f:
            f.write(json.dumps(result) + '\n')
        self.result_pub.publish(String(data=json.dumps(result)))
        self.get_logger().info(f'cell {room["cell"]}: {len(views)} view(s) saved to {folder} '
                               f'({sum(1 for v in views if v["frame"])} with a frame); '
                               f'VLM {self.vlm}')


def main():
    rclpy.init()
    node = RoomScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
