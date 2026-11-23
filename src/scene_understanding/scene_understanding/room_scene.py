"""
room_scene: the views of each scanned dead end -> one VLM call -> /scene/result.

Same interface as challenge_ws's room_scene (maze_nav's maze_explorer and
task_reporter work with it unchanged), with the VLM wired to this package's
lab-server client (vlm_client.VLMClient).

Room scanning (done by maze_explorer, never by this node): at a dead end the
explorer holds still facing into it ('centre'), then turns to each side wall
('left', 'right'). At the end of each hold it publishes /scene/capture
(std_msgs/String JSON):
    {cell, cell_centre, view, last, stamp, hold_start, kind, attempt, robot,
     tilt_deg, pitch_cmd_deg, camera_pitch_deg, ...}
The explorer never waits for this node.

Frames: the last `buffer_s` s of the compressed colour topic, kept as
received. For each capture the frame stamped inside [hold_start, stamp]
nearest its end is taken; if none (robot clock offset), the newest that
ARRIVED during the hold ('by arrival'); else the view has no frame. A room is
complete on `last`, or `room_timeout` s after its newest view (look cut
short). Each (cell, attempt) is analysed once, at most `max_attempts` tries
per cell. /scene/retry {cell, attempt} sends the views already captured for
that cell again (the explorer asks when no answer came in time).

Analysis (worker thread, one room at a time, so a slow VLM never drops a
room): the views are decoded and stitched left | centre | right (other views
in capture order), and sent in ONE request with a panel-mapped scene prompt
(scene_prompt.SCENE_PROMPT). The reply is mapped to what the explorer's
scene_verdict() and task_reporter read:
    objects  mission_matches of kind object (labels)
    hazards  mission_matches of kind hazard with a reportable label
             (REPORTED_HAZARDS); other warning signs go to `signs`
    humans   any survivor match
    qr       decoded QR messages
    scene_type  the VLM's, except: not a room type and a path note names
             stairs / a curtain -> 'stairs' / 'curtain' (the explorer then
             queues the cell as a candidate instead of a dead end). The VLM's
             own value is kept in vlm_scene_type.
A failed or unparsable VLM reply is published with vlm 'none' and an
`error`: the explorer treats that as no answer (and retries), task_reporter
ignores it.

`vlm`: 'lab' (lab perception server, vlm_server / vlm_group) or 'none'
(save and publish only, scene_type 'unknown').

  /scene/result  std_msgs/String JSON: {cell, cell_centre, attempt, kind,
                 views: [{view, frame, how, tilt_deg, pitch_cmd_deg,
                 camera_pitch_deg, robot}], image, complete, scene_type,
                 vlm_scene_type, hazards, objects, signs, humans, qr, paths,
                 pos (= cell_centre), reports (scene_prompt.mission_reports:
                 one per mission match, each with pos), vlm, latency_sec,
                 error}

Saved to `save_dir`/<YYYY-MM-DD>/<HHMMSS>_cell_<i>_<j>[_try<n>]_<view>.jpg,
the stitched image (..._room.jpg) and rooms.jsonl (with the raw VLM reply).

    ros2 run scene_understanding room_scene
    ros2 run scene_understanding room_scene --ros-args \
        -p vlm_server:=http://192.168.10.141:8000 -p vlm_group:=group04_grp4
"""

import collections
import datetime
import json
import os
import queue
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from scene_understanding.scene_prompt import (
    HAZARD_LABELS, mission_reports, ROOM_SCENE_TYPES as REPORT_ROOM_TYPES,
    SCENE_PROMPT, extract_scene_result, validate_group_id,
)
from scene_understanding.vlm_client import (
    DEFAULT_SERVER, MAX_HEIGHT, MAX_WIDTH, VLMClient,
)

VLM_BACKENDS = ('lab', 'none')
# Panel order in the stitched image, left to right; other views follow.
PANEL_ORDER = ('left', 'centre', 'right')
# Which path direction each panel stands for in the prompt.
PANEL_DIRECTION = {'left': 'left', 'centre': 'forward', 'right': 'right'}
PANEL_MEANING = {
    'left': "the cell's left side wall (robot turned left)",
    'centre': 'facing into the cell from its open side',
    'right': "the cell's right side wall (robot turned right)",
}
# Hazard labels the scoring server accepts; other signs are kept in `signs`.
REPORTED_HAZARDS = set(HAZARD_LABELS)
# scene_type values that are rooms (the booklet's eight room types).
ROOM_SCENE_TYPES = set(REPORT_ROOM_TYPES)


def stamp_ns(text):
    """Convert '<sec>.<nanosec>' (the explorer's stamp strings) to int ns."""
    sec, _, frac = str(text).partition('.')
    return int(sec) * 10**9 + int((frac + '000000000')[:9])


def pick_frame(frames, start_ns, end_ns, arr_lo=None, arr_hi=None):
    """
    Pick the camera frame for one view hold.

    frames: [(stamp_ns, arrival_s, msg)], oldest first. Returns (msg, how):
    the frame stamped in [start_ns, end_ns] nearest end_ns ('stamp'); else
    the newest that arrived in [arr_lo, arr_hi] ('arrival'); else
    (None, None).
    """
    inside = [f for f in frames if start_ns <= f[0] <= end_ns]
    if inside:
        return max(inside, key=lambda f: f[0])[2], 'stamp'
    if arr_lo is not None:
        arrived = [f for f in frames if arr_lo <= f[1] <= arr_hi]
        if arrived:
            return arrived[-1][2], 'arrival'
    return None, None


def order_views(views):
    """Return view dicts sorted left, centre, right, then the rest as captured."""
    rank = {name: k for k, name in enumerate(PANEL_ORDER)}
    return sorted(views, key=lambda v: rank.get(v.get('view'), len(rank)))


def stitch(images):
    """Scale BGR images to a common height and join them left to right."""
    height = min(img.shape[0] for img in images)
    scaled = [
        img if img.shape[0] == height else cv2.resize(
            img, (max(1, round(img.shape[1] * height / img.shape[0])), height),
            interpolation=cv2.INTER_AREA)
        for img in images
    ]
    return np.concatenate(scaled, axis=1)


def build_room_prompt(view_names):
    """Build the scene prompt for the panels of one scanned dead end."""
    if len(view_names) == 1:
        return SCENE_PROMPT
    lines = [
        f'The image contains {len(view_names)} equal-width panels from one '
        'scan of a maze dead-end cell (a possible room), left to right:',
    ]
    mapped = []
    for k, name in enumerate(view_names, 1):
        meaning = PANEL_MEANING.get(name, f'view "{name}"')
        direction = PANEL_DIRECTION.get(name)
        lines.append(f'- Panel {k}: {meaning}'
                     + (f'; use it for direction="{direction}".'
                        if direction else '.'))
        if direction:
            mapped.append(direction)
    missing = [d for d in ('forward', 'left', 'right') if d not in mapped]
    if missing:
        lines.append('No panel shows ' + ', '.join(missing)
                     + '; use status "unknown" for it.')
    lines.append('Use only the corresponding panel for each path status and '
                 'note. Panel edges are camera boundaries. Use all panels for '
                 'scene_type and mission evidence; a match seen in several '
                 'panels is one candidate.')
    return '\n'.join(lines) + '\n' + SCENE_PROMPT


def scene_fields(scene):
    """
    Map a parsed scene observation to the /scene/result fields.

    Returns a dict with scene_type, vlm_scene_type, objects, hazards, signs,
    humans, qr and paths.
    """
    objects, hazards, signs, qr = [], [], [], []
    humans = False
    for m in scene.get('mission_matches') or []:
        if not isinstance(m, dict):
            continue
        kind = str(m.get('kind') or '').strip().lower()
        label = str(m.get('label') or '').strip().lower()
        if kind == 'object' and label and label not in objects:
            objects.append(label)
        elif kind == 'hazard' and label:
            target = hazards if label in REPORTED_HAZARDS else signs
            if label not in target:
                target.append(label)
        elif kind == 'survivor':
            humans = True
        elif kind == 'qr':
            message = str(m.get('message') or '').strip()
            if message and message not in qr:
                qr.append(message)
    paths = [p for p in scene.get('paths') or [] if isinstance(p, dict)]
    vlm_type = str(scene.get('scene_type') or 'unknown').strip().lower()
    scene_type = vlm_type
    if vlm_type not in ROOM_SCENE_TYPES:
        notes = ' '.join(str(p.get('notes') or '') for p in paths).lower()
        if 'stair' in notes:
            scene_type = 'stairs'
        elif 'curtain' in notes:
            scene_type = 'curtain'
    return {'scene_type': scene_type, 'vlm_scene_type': vlm_type,
            'objects': objects, 'hazards': hazards, 'signs': signs,
            'humans': humans, 'qr': qr, 'paths': paths}


class RoomScene(Node):
    """Collect each scanned dead end's views and analyse them as one room."""

    def __init__(self):
        super().__init__('room_scene')
        p = self.declare_parameter
        p('scene_capture_topic', '/scene/capture')
        p('camera_topic', '/camera/color/image_raw/compressed')
        p('scene_result_topic', '/scene/result')
        p('scene_retry_topic', '/scene/retry')
        p('buffer_s', 4.0)          # s of camera frames kept
        p('room_timeout', 20.0)     # s after a room's newest view without last
        p('save_dir', os.path.expanduser(
            '~/test_scene_understanding/src/scene_understanding/'
            'room_scene_results'))
        p('max_attempts', 2)        # VLM tries per cell
        p('vlm', 'lab')             # lab | none
        p('vlm_server', DEFAULT_SERVER)
        p('vlm_group', 'group04_grp4')
        p('vlm_read_timeout', 75.0)
        p('vlm_connect_timeout', 5.0)
        p('vlm_max_width', MAX_WIDTH)
        p('vlm_max_height', MAX_HEIGHT)

        def g(name):
            return self.get_parameter(name).value

        self.max_attempts = int(g('max_attempts'))
        self.buffer_s = float(g('buffer_s'))
        self.room_timeout = float(g('room_timeout'))
        self.save_dir = os.path.expanduser(g('save_dir'))
        self.vlm_name, self.client = self.make_client(g)

        self.frames = collections.deque(maxlen=400)  # (stamp_ns, arrival, msg)
        self.rooms = {}       # (cell, attempt) -> room being collected
        self.done = set()     # (cell, attempt) already sent to the worker
        self.sent = {}        # cell -> its newest room sent (kept for a retry)
        self.work = queue.Queue()
        self.result_pub = self.create_publisher(
            String, g('scene_result_topic'), 10)
        self.create_subscription(
            String, g('scene_capture_topic'), self.on_capture, 10)
        if g('scene_retry_topic'):
            self.create_subscription(
                String, g('scene_retry_topic'), self.on_retry, 10)
        self.create_subscription(
            CompressedImage, g('camera_topic'), self.on_image,
            qos_profile_sensor_data)
        self.create_timer(1.0, self.check_timeouts)
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(
            f'captures on {g("scene_capture_topic")}, frames from '
            f'{g("camera_topic")}, results on {g("scene_result_topic")}; '
            f'saving to {self.save_dir}; VLM: {self.vlm_name}')

    def make_client(self, g):
        """Return (vlm name for results, client or None) from parameters."""
        backend = str(g('vlm')).strip().lower()
        if backend not in VLM_BACKENDS:
            self.get_logger().warn(
                f'vlm={backend!r}: not one of {VLM_BACKENDS}; using none')
            return 'none', None
        if backend == 'lab':
            try:
                validate_group_id(g('vlm_group'))
                return 'lab', VLMClient(
                    group_id=g('vlm_group'), prompt=SCENE_PROMPT,
                    server=g('vlm_server'),
                    read_timeout=float(g('vlm_read_timeout')),
                    connect_timeout=float(g('vlm_connect_timeout')),
                    max_width=int(g('vlm_max_width')),
                    max_height=int(g('vlm_max_height')))
            except ValueError as e:
                self.get_logger().error(f'lab VLM not usable ({e}); using none')
                return 'none', None
        return 'none', None

    def on_image(self, msg):
        """Keep the last buffer_s s of camera frames (not decoded)."""
        now = time.monotonic()
        s = msg.header.stamp
        self.frames.append((s.sec * 10**9 + s.nanosec, now, msg))
        while self.frames and now - self.frames[0][1] > self.buffer_s:
            self.frames.popleft()

    def on_capture(self, msg):
        """Handle one /scene/capture view: pick its frame, add it to its room."""
        try:
            c = json.loads(msg.data)
            cell = (int(c['cell'][0]), int(c['cell'][1]))
            start, end = stamp_ns(c['hold_start']), stamp_ns(c['stamp'])
            attempt = int(c.get('attempt') or 1)
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(
                f'bad /scene/capture ({e}): {msg.data[:200]}')
            return
        key = (cell, attempt)
        where = f'cell {list(cell)} {c.get("view")}'
        if attempt > self.max_attempts:
            self.get_logger().warn(
                f'{where}: attempt {attempt} > max_attempts '
                f'{self.max_attempts}; not sent to the VLM')
            return
        if key in self.done:
            self.get_logger().info(
                f'{where} (attempt {attempt}): already analysed; ignored')
            return
        now = time.monotonic()
        frame, how = pick_frame(self.frames, start, end,
                                now - (end - start) / 1e9 - 0.2, now)
        if frame is None:
            self.get_logger().warn(
                f'{where}: no camera frame from the hold '
                '(camera topic silent?)')
        elif how == 'arrival':
            self.get_logger().info(
                f'{where}: no frame stamped in the hold (robot clock '
                'offset?); took one by arrival')
        room = self.rooms.setdefault(key, {
            'cell': list(cell), 'attempt': attempt,
            'cell_centre': c.get('cell_centre'), 'views': []})
        room['updated'] = now
        room['views'].append({'view': c.get('view'), 'msg': frame,
                              'how': how, 'capture': c})
        self.get_logger().info(
            f'cell {list(cell)}: view {c.get("view")} '
            f'({len(room["views"])} so far)'
            + (', last: queued' if c.get('last') else ''))
        if c.get('last'):
            self.finish(key, complete=True)

    def check_timeouts(self):
        """Queue rooms whose look was cut short (no last view in time)."""
        now = time.monotonic()
        for key in [k for k, r in self.rooms.items()
                    if now - r['updated'] > self.room_timeout]:
            self.get_logger().warn(
                f'cell {list(key[0])} (attempt {key[1]}): no last view for '
                f'{self.room_timeout:.0f} s (look cut short); queued as is')
            self.finish(key, complete=False)

    def finish(self, key, complete):
        """Hand a collected room to the worker."""
        room = self.rooms.pop(key)
        room['complete'] = complete
        self.done.add(key)
        self.sent[key[0]] = room
        self.work.put(room)

    def on_retry(self, msg):
        """Resend a cell's captured views as a new attempt (/scene/retry)."""
        try:
            r = json.loads(msg.data)
            cell = (int(r['cell'][0]), int(r['cell'][1]))
            attempt = int(r.get('attempt') or 2)
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.get_logger().warn(f'bad /scene/retry ({e}): {msg.data[:200]}')
            return
        if attempt > self.max_attempts:
            self.get_logger().warn(
                f'cell {list(cell)}: retry attempt {attempt} > max_attempts '
                f'{self.max_attempts}; not sent to the VLM')
            return
        if (cell, attempt) in self.done or (cell, attempt) in self.rooms:
            self.get_logger().info(
                f'cell {list(cell)}: attempt {attempt} already queued; '
                'retry ignored')
            return
        prev = self.sent.get(cell)
        if prev is None:
            self.get_logger().warn(
                f'cell {list(cell)}: retry {attempt}, but no views of it '
                'were captured; nothing to send')
            return
        room = dict(prev, attempt=attempt, views=list(prev['views']))
        self.done.add((cell, attempt))
        self.sent[cell] = room
        self.get_logger().info(
            f'cell {list(cell)}: no answer in time; sending its '
            f'{len(room["views"])} view(s) again (attempt {attempt})')
        self.work.put(room)

    def worker(self):
        """Process queued rooms one at a time, whatever one room does."""
        while True:
            room = self.work.get()
            try:
                self.process(room)
            except Exception as e:  # noqa: B902 - keep the queue alive
                self.get_logger().error(
                    f'cell {room["cell"]}: processing failed: {e!r}')

    def analyse(self, image, names):
        """
        Send one room image (its panels stitched in `names` order) to the VLM.

        Returns a dict with vlm, latency_sec, error, response and, on
        success, the scene_fields() fields.
        """
        out = {'vlm': 'none', 'latency_sec': None, 'error': None,
               'response': None}
        if self.client is None:
            return out
        if image is None:
            out['error'] = 'no camera frame for any view'
            return out
        reply = self.client.describe(image, prompt=build_room_prompt(names))
        out['latency_sec'] = reply.get('latency_sec')
        out['response'] = reply.get('response')
        if not reply.get('ok'):
            out['error'] = reply.get('error') or 'VLM request failed'
            return out
        try:
            scene = extract_scene_result(reply.get('response'))
        except ValueError as e:
            out['error'] = str(e)
            return out
        out.update(scene_fields(scene))
        out['scene'] = scene
        out['vlm'] = self.vlm_name
        return out

    def process(self, room):
        """Save a room's frames, analyse them and publish /scene/result."""
        day = datetime.datetime.now()
        folder = os.path.join(self.save_dir, day.strftime('%Y-%m-%d'))
        os.makedirs(folder, exist_ok=True)
        base = (f'{day.strftime("%H%M%S")}_cell_{room["cell"][0]}_'
                f'{room["cell"][1]}'
                + (f'_try{room["attempt"]}' if room['attempt'] > 1 else ''))
        views, panels = [], []
        for v in order_views(room['views']):
            name = None
            if v['msg'] is not None:
                data = bytes(v['msg'].data)
                ext = 'png' if 'png' in (v['msg'].format or '') else 'jpg'
                name = f'{base}_{v["view"]}.{ext}'
                with open(os.path.join(folder, name), 'wb') as f:
                    f.write(data)
                img = cv2.imdecode(np.frombuffer(data, np.uint8),
                                   cv2.IMREAD_COLOR)
                if img is None:
                    self.get_logger().warn(
                        f'cell {room["cell"]} {v["view"]}: frame did not '
                        'decode; left out of the VLM image')
                else:
                    panels.append((str(v['view']), img))
            cap = v['capture']
            views.append({'view': v['view'], 'frame': name, 'how': v['how'],
                          **{k: cap.get(k) for k in (
                              'tilt_deg', 'pitch_cmd_deg',
                              'camera_pitch_deg', 'robot')}})
        image = image_name = None
        if panels:
            image = stitch([img for _, img in panels])
            image_name = f'{base}_room.jpg'
            if len(panels) > 1:
                cv2.imwrite(os.path.join(folder, image_name), image)
            else:
                image_name = next(v['frame'] for v in views if v['frame'])

        analysis = self.analyse(image, [name for name, _ in panels])
        kind = ((room['views'][0]['capture'].get('kind')
                 if room['views'] else None) or 'dead_end')
        result = {
            'cell': room['cell'], 'cell_centre': room['cell_centre'],
            'attempt': room['attempt'], 'kind': kind, 'views': views,
            'image': image_name, 'complete': room['complete'],
            'scene_type': 'unknown', 'vlm_scene_type': None, 'hazards': [],
            'objects': [], 'signs': [], 'humans': False, 'qr': [],
            'paths': [],
        }
        response = analysis.pop('response')
        scene = analysis.pop('scene', None)
        result.update(analysis)
        # Every result carries the position: the scanned cell's centre.
        result['pos'] = room['cell_centre']
        result['reports'] = mission_reports(scene, result['pos']) if scene else []
        with open(os.path.join(folder, 'rooms.jsonl'), 'a') as f:
            f.write(json.dumps(dict(result, vlm_response=response),
                               default=str) + '\n')
        self.result_pub.publish(String(data=json.dumps(result)))
        summary = (f'error: {result["error"]}' if result['error'] else
                   f'scene_type {result["scene_type"]!r}, objects '
                   f'{result["objects"]}, hazards {result["hazards"]}, '
                   f'signs {result["signs"]}, humans {result["humans"]}')
        self.get_logger().info(
            f'cell {room["cell"]} (attempt {room["attempt"]}): {len(panels)}'
            f'/{len(views)} view(s) to VLM {result["vlm"]}; {summary}')


def main():
    """Run the room_scene node."""
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
