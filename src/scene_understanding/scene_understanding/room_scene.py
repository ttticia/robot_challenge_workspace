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

Depth mask (2026-11-23, depth_mask.py): toy sets stand taller than the 0.53 m walls, so a
view into one room also shows neighbouring rooms' toys, the other arena and the hall. Each
view is paired with the depth_relay frame nearest it (within depth_max_dt s) and saved
again as <name>_masked.jpg with every pixel beyond the room cell (farthest corner from the
robot + mask_margin) or without depth nearby filled grey; the record's view gets 'masked'
(file name, None if not possible: 'mask_note' says why) and 'mask_keep' (fraction kept).
The vision model should be given the masked frames. depth_mask:=false = off.

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

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from scene_understanding.depth_mask import (apply_mask, depth_mask, depth_to_metres,
                                            room_depth_limit)

try:
    import cv2
except ImportError:
    cv2 = None


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
        p('depth_mask', True)          # grey out what lies beyond the room (depth_mask.py)
        p('depth_topic', '/camera/depth_relay/image_raw')        # local relay: no bridge copy
        p('depth_info_topic', '/camera/depth_relay/camera_info')
        p('colour_info_topic', '/camera/color/camera_info')
        p('depth_max_dt', 0.3)         # s between the colour frame and its depth frame
        p('mask_margin', 0.25)         # m past the room cell's farthest corner kept
        p('cell_size', 1.02)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.max_attempts = int(g('max_attempts'))
        self.buffer_s, self.room_timeout = float(g('buffer_s')), float(g('room_timeout'))
        self.save_dir, self.vlm = g('save_dir'), g('vlm')

        self.frames = collections.deque(maxlen=400)   # (stamp_ns, arrival_s, msg)
        self.depth_frames = collections.deque(maxlen=60)   # (stamp_ns, arrival_s, msg)
        self.depth_k = self.colour_k = self.colour_size = None
        self.mask_on = bool(g('depth_mask')) and cv2 is not None
        self.depth_max_dt = float(g('depth_max_dt'))
        self.mask_margin, self.cell_size = float(g('mask_margin')), float(g('cell_size'))
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
        if self.mask_on:
            self.create_subscription(Image, g('depth_topic'), self.on_depth,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, g('depth_info_topic'), self.on_depth_info,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, g('colour_info_topic'), self.on_colour_info,
                                     qos_profile_sensor_data)
        elif g('depth_mask'):
            self.get_logger().warn('depth_mask: OpenCV missing; frames are not masked')
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

    def on_depth(self, msg):
        now = time.monotonic()
        s = msg.header.stamp
        self.depth_frames.append((s.sec * 10**9 + s.nanosec, now, msg))
        while self.depth_frames and now - self.depth_frames[0][1] > self.buffer_s:
            self.depth_frames.popleft()

    def on_depth_info(self, msg):
        self.depth_k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_colour_info(self, msg):
        self.colour_k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.colour_size = (msg.width, msg.height)

    def depth_for(self, frame):
        """The buffered depth frame stamped nearest the colour frame, within depth_max_dt."""
        if frame is None or not self.depth_frames:
            return None
        s = frame.header.stamp
        t = s.sec * 10**9 + s.nanosec
        best = min(self.depth_frames, key=lambda f: abs(f[0] - t))
        return best[2] if abs(best[0] - t) <= self.depth_max_dt * 1e9 else None

    def masked_frame(self, view, cell_centre):
        """(jpeg bytes, kept fraction, note): the view's frame with everything beyond the
        room greyed out (depth_mask.py), or (None, None, why not)."""
        if not self.mask_on:
            return None, None, 'depth_mask off'
        frame, depth = view.get('msg'), view.get('depth')
        robot = (view.get('capture') or {}).get('robot')
        if frame is None or depth is None:
            return None, None, 'no depth frame near the colour frame'
        if not robot or not cell_centre or self.depth_k is None or self.colour_k is None:
            return None, None, 'no robot pose / cell centre / camera intrinsics'
        img = cv2.imdecode(np.frombuffer(bytes(frame.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None, None, 'colour frame does not decode'
        dtype = np.uint16 if depth.encoding in ('16UC1', 'mono16') else np.float32
        raw = np.frombuffer(bytes(depth.data), dtype).reshape(depth.height, -1)[:, :depth.width]
        limit = room_depth_limit((robot['x'], robot['y']), cell_centre, self.cell_size,
                                 self.mask_margin)
        mask = depth_mask(depth_to_metres(depth.encoding, raw), self.depth_k, self.colour_k,
                          self.colour_size, limit)
        out = apply_mask(img, mask)
        ok, jpg = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return None, None, 'jpeg encode failed'
        return jpg.tobytes(), float(mask.mean()), f'kept depth <= {limit:.2f} m'

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
                              'capture': c, 'depth': self.depth_for(frame)})
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
            name = masked = keep = None
            note = ''
            if v['msg'] is not None:
                ext = 'png' if 'png' in (v['msg'].format or '') else 'jpg'
                name = (f'{day.strftime("%H%M%S")}_cell_{room["cell"][0]}_{room["cell"][1]}'
                        + (f'_try{room["attempt"]}' if room['attempt'] > 1 else '')
                        + f'_{v["view"]}.{ext}')
                with open(os.path.join(folder, name), 'wb') as f:
                    f.write(bytes(v['msg'].data))
                try:
                    jpg, keep, note = self.masked_frame(v, room['cell_centre'])
                except Exception as e:      # never lose the room over the mask
                    jpg, keep, note = None, None, f'mask failed: {e!r}'
                if jpg is not None:
                    masked = name.rsplit('.', 1)[0] + '_masked.jpg'
                    with open(os.path.join(folder, masked), 'wb') as f:
                        f.write(jpg)
            cap = v['capture']
            views.append({'view': v['view'], 'frame': name, 'how': v['how'],
                          'masked': masked, 'mask_note': note,
                          'mask_keep': None if keep is None else round(keep, 3),
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
