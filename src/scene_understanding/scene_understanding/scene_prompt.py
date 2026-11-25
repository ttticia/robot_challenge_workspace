import json
import math
import re

SCENE_PROMPT = """Return one compact JSON observation of the camera frame, even if no mission matches.
For a single camera frame, describe forward, left and right from the robot's
perspective. For a three-panel image, use the panel mapping above. Use open
for a visible continuing route or a pass-through curtain as described below.
Use blocked when a visible obstruction prevents passage. Use partially_blocked
when an obstruction narrows the route and the remaining clearance is uncertain.
Use unknown when there is insufficient evidence to assess the route.
In notes, name only the decisive
visible cue in at most six words, or use "" if status is self-explanatory.
Mention an obstacle only once unless it affects multiple directions. Include
terrain only when it affects a path. Do not infer distance, clearance beyond
the frame, or movement commands.
When clearly visible metal stairs lie on a route, write "Metal stairs" as that
path's note. Report the path status according to the visible obstruction.

In this arena, a recognizable flexible curtain or freely hanging paper sheet
across a passage is a pass-through marker. If it is the only visible obstruction,
set status to open and use notes like "Passable curtain; view beyond obscured".
Apply this rule even if the marker hides the view behind it. If you see a
separate obstacle, assess the route around that obstacle. If you cannot tell
whether the hanging material is flexible or rigid, use unknown.

Check all categories in every frame, even when the room is unknown. Props
may be miniature or toy versions, printed pictures or simulated features;
treat them as the real thing.
Room cues for scene_type (use exactly these names): kitchen (cooking or
food-prep equipment), market (displayed goods, produce or stalls), workshop
(workbenches or tools), clinic (medical equipment, beds or signage),
cleaning_room (cleaning supplies or equipment), clothes_shop (clothing on
racks or hangers, mannequins), fitness_centre (exercise equipment such as
dumbbells, treadmills, exercise bikes or mats), furniture_shop (displayed
sofas, beds, tables, chairs or cabinets). Name a room only from its cues.
Otherwise use open_area, corridor, junction or unknown. Use null for
uncertain junction status.

Object labels (use exactly these): medical_equipment (first aid kit,
bandages, stethoscope, syringe, medicine, stretcher, wheelchair), hand_tools
(wrench, hammer, pliers, screwdriver, saw), food (fruit, bread, packaged or
canned food), water (water bottles, jugs or dispensers), utensils (cutlery,
ladle, spatula, cooking pots and pans).
Hazard labels (use exactly these; only for a warning sign that shows the
symbol): high_voltage (lightning bolt), radioactive (radiation trefoil),
chemical (corrosive liquid on a hand or surface, or a flask), biological
(biohazard symbol), flammable (flame). Other warning signs are not
reportable hazards: report them only as sightings with kind="sign" and the
visual symbol as the note, e.g. exclamation mark, horseshoe magnet, antenna
with waves, laser starburst. If a symbol is ambiguous, use a sighting
instead of guessing a hazard label.

Any recognizable depiction of a human (photo, poster or printed human figures,
including cutouts) counts as a survivor, even if partly cropped or the room
is unknown. Use kind="survivor" and label="". People physically present are
reported as sightings when there is no evidence they are survivor markers.
For QR codes, use kind="qr" and label="" for an exactly decoded message.
Use sightings with kind="qr" and a short note when a code is visible but
unreadable. Never invent QR text.

List each supported candidate in mission_matches with kind (object, hazard,
survivor or qr), label (an object or hazard label above; "" for survivor and
qr), a short evidence phrase, and message (empty except for QR). List each
distinct candidate once. Use [] if none are supported.
Use optional sightings with kind and a short note for inconclusive clues.
Omit sightings when empty. Do not assign task IDs or infer match from room alone.

Output only minified JSON. Use these required fields at the top level:
{"scene_type":"unknown","junction_confirmed":null,"paths":[
{"direction":"forward","status":"unknown","notes":""},
{"direction":"left","status":"unknown","notes":""},
{"direction":"right","status":"unknown","notes":""}],"mission_matches":[]}
"""

# Prompt for the explorer's room / object looks. It asks only for what the five missions
# score (scene type, objects, hazard sign, survivor) and for a four-key answer. All five
# object labels are asked for, not only M01's hand_tools: the 10:54 run named a room
# "clinic" from its medical equipment and yet recorded no object (user, 2026-11-24). (QR codes
# are left to qr_scanner, which decodes them exactly; removed on the user's word 2026-11-24): the
# reply time is mostly the length of the answer. Measured on the lab server 2026-11-24 on
# the 10:03 run's frames: 2.4-9 s, against 18-22 s for SCENE_PROMPT with its path notes.
LOOK_PROMPT = """The image is a grid of three views from a small robot's camera inside a search-and-rescue training arena: top left = looking left, top right = looking straight ahead, bottom left = looking right (bottom right is empty). The views overlap, so the same thing may appear twice. Props may be toys, miniatures or printed pictures: treat them as the real thing. Ignore everything outside the arena's low grey partition walls (the hall, real people, furniture behind the walls).
Answer with minified JSON only, with exactly these keys:
{"scene_type":"unknown","objects":[],"hazard":"","survivor":false}
scene_type: the room that the props set up, one of kitchen (cooking or food-preparation equipment), market (displayed goods, produce, stalls, shelves of products), workshop (workbench, tools), clinic (medical equipment, bed), cleaning_room (cleaning supplies or equipment), clothes_shop (clothes on racks or hangers, mannequins), fitness_centre (exercise equipment), furniture_shop (sofas, beds, tables, chairs or cabinets on display). Use "unknown" if no such props are visible.
objects: every kind of these that is visible, by exactly these names: medical_equipment (first aid kit, bandages, stethoscope, syringe, medicine, stretcher, wheelchair, hospital bed, drip stand), hand_tools (wrench, hammer, pliers, screwdriver, saw), food (fruit, bread, packaged or canned food), water (water bottles, jugs, dispensers), utensils (cutlery, ladle, spatula, pots, pans). [] if none.
hazard: the symbol on a warning sign, one of high_voltage (lightning bolt), radioactive (trefoil), chemical (corrosion or flask), biological (biohazard), flammable (flame). Use "" if there is no warning sign or it shows another symbol.
survivor: true if a photo, poster or cut-out showing a person is displayed inside the arena.
"""

# Competition reporting vocabulary (challenge booklet, task reporting section).
ROOM_SCENE_TYPES = ('clothes_shop', 'fitness_centre', 'kitchen', 'market',
                    'workshop', 'clinic', 'cleaning_room', 'furniture_shop')
# The VLM may also describe a frame as one of these; they are not reportable.
OTHER_SCENE_TYPES = ('corridor', 'open_area', 'junction', 'unknown')
OBJECT_LABELS = ('medical_equipment', 'hand_tools', 'food', 'water',
                 'utensils')
HAZARD_LABELS = ('high_voltage', 'radioactive', 'chemical', 'biological',
                 'flammable')
M01_OBJECT = 'hand_tools'       # M01's target is fixed (booklet update 2026-11-24)
# Allowed label per task_type ('' only for survivor and qr).
TASK_LABELS = {
    'object': OBJECT_LABELS,
    'hazard': HAZARD_LABELS,
    'survivor': ('',),
    'qr': ('',),
}
# Mission types and task_ids are fixed; the specific object / hazard for
# M01 / M02 is announced by the organizers, so the label is not fixed here.
MISSIONS = {
    'M01': 'object',      # identify the object and its room / location
    'M02': 'hazard',      # identify the hazard sign and its room / location
    'M03': 'survivor',    # first survivor's location
    'M04': 'qr',          # decoded QR message (pos null)
    'M05': 'survivor',    # second survivor's location
}
# Candidate task_ids per task_type (M03 / M05 are two different survivors).
TASK_IDS = {'object': ('M01',), 'hazard': ('M02',),
            'survivor': ('M03', 'M05'), 'qr': ('M04',)}
TASK_REPORT_URL = 'http://192.168.10.141:8001/task_report_B'    # _B = course B
# Arena grid: cell centres 1.02 m apart, (0, 0) = start cell centre, +x the
# robot's initial forward direction, +y its left.
CELL_SIZE = 1.02
REPORT_CELLS = ('ahead', 'robot')
REPORT_FIELDS = ('group_id', 'task_id', 'task_type', 'scene_type', 'label',
                 'pos', 'message')
# Backwards-compatible name: scene types accepted in a task report.
VALID_SCENE_TYPES = set(ROOM_SCENE_TYPES)
PATH_STATUSES = ('open', 'partially_blocked', 'blocked', 'unknown')


def build_prompt(three_view=False):
    """Build the automatic scene observation prompt."""
    if three_view:
        return (
            'The image contains three equal-width panels from the same junction.\n'
            'For paths, use this fixed mapping by position in the stitched image:\n'
            '- Leftmost third: direction="left" (robot turned left).\n'
            '- Middle third: direction="forward" (original robot heading).\n'
            '- Rightmost third: direction="right" (robot turned right).\n'
            'Use only the corresponding panel for each path status and note. '
            'Do not mirror or swap the panel labels. Panel edges are camera boundaries. '
            'Use all panels for mission evidence.\n' + SCENE_PROMPT
        )
    return SCENE_PROMPT


def build_look_prompt():
    """Prompt for a 2 x 2 grid of the explorer's left, centre and right views."""
    return LOOK_PROMPT


def extract_look_result(response):
    """Unwrap the answer to LOOK_PROMPT into a scene observation.

    Returns the four answers, cleaned to the booklet's vocabulary, plus the
    `mission_matches` list that mission_reports() reads (and an empty `paths`,
    which a look does not ask for). Raises ValueError for an unusable reply.
    """
    data = parse_scene_json(response)
    if data is not None and data.get('success') is False:
        raise ValueError('Lab VLM server reported failure')
    if data is not None and 'result' in data:
        data = parse_scene_json(data['result'])
    if data is None or 'scene_type' not in data:
        raise ValueError('VLM reply did not contain a look observation')

    def flag(key):
        value = data.get(key)
        return value is True or str(value).strip().lower() == 'true'

    scene_type = str(data.get('scene_type') or '').strip().lower()
    hazard = str(data.get('hazard') or '').strip().lower()
    seen = data.get('objects')
    seen = [str(o).strip().lower() for o in seen] if isinstance(seen, list) else []
    if flag('hand_tools'):                       # the earlier form of the answer
        seen.append('hand_tools')
    objects = [label for label in OBJECT_LABELS if label in seen]
    scene = {
        'scene_type': scene_type if scene_type in ROOM_SCENE_TYPES else 'unknown',
        'objects': objects,
        'hand_tools': 'hand_tools' in objects,
        'hazard': hazard if hazard in HAZARD_LABELS else '',
        'survivor': flag('survivor'),
        'paths': [],
        'mission_matches': [],
    }
    add = scene['mission_matches'].append
    for label in objects:
        add({'kind': 'object', 'label': label, 'evidence': label.replace('_', ' ') + ' seen',
             'message': ''})
    if scene['hazard']:
        add({'kind': 'hazard', 'label': scene['hazard'], 'evidence': 'warning sign', 'message': ''})
    if scene['survivor']:
        add({'kind': 'survivor', 'label': '', 'evidence': 'picture of a person', 'message': ''})
    return scene


def look_cell(cell, side):
    """The cell looked into from `cell` through `side` (0 = the start heading, 1 = left,
    2 = back, 3 = right)."""
    di, dj = ((1, 0), (0, 1), (-1, 0), (0, -1))[int(side) % 4]
    return (int(cell[0]) + di, int(cell[1]) + dj)


def look_cell_position(cell, side, cell_size=CELL_SIZE):
    """Arena position [x, y] of that cell's centre, 2 decimals."""
    i, j = look_cell(cell, side)
    return [round(i * cell_size, 2), round(j * cell_size, 2)]


def parse_scene_json(response):
    """Return the VLM response as a dict, or None if invalid."""
    if response is None:
        return None
    if isinstance(response, dict):
        return response
    if not isinstance(response, str) or not response.strip():
        return None

    start, end = response.find('{'), response.rfind('}')
    if start < 0 or end <= start:
        return None

    try:
        data = json.loads(response[start:end + 1])
    except (ValueError, json.JSONDecodeError):
        return None

    return data if isinstance(data, dict) else None


def validate_group_id(group_id):
    """Check the registered team identifier format."""
    if not re.fullmatch(r'group[0-9]{2}_[A-Za-z0-9]{4}', group_id):
        raise ValueError('vlm_group must use the registered group##_XXXX format')


def validate_scene_result(data):
    """
    Check a task report against the booklet's fixed vocabulary.

    Returns (valid, reason). Valid means the scoring server should accept it
    as a correctly formatted message with permitted values (it does not mean
    the result is correct):
      object / hazard: label from OBJECT_LABELS / HAZARD_LABELS, room
        scene_type, finite pos [x, y], empty message;
      survivor: empty label, room scene_type, finite pos [x, y], empty message;
      qr: empty label, room scene_type, pos [x, y] or null, the decoded
        message.
    """
    strings = set(REPORT_FIELDS) - {'pos'}
    if not isinstance(data, dict) or set(data) != set(REPORT_FIELDS):
        return False, 'Result does not contain exactly the required fields'
    if not all(isinstance(data[key], str) for key in strings):
        return False, 'All fields except pos must be strings'
    try:
        validate_group_id(data['group_id'])
    except ValueError as exc:
        return False, str(exc)
    kind = MISSIONS.get(data['task_id'])
    if kind is None:
        return False, f'Unknown task_id {data["task_id"]!r}'
    if data['task_type'] != kind:
        return False, f'{data["task_id"]} is a {kind} task, not {data["task_type"]!r}'
    if data['label'] not in TASK_LABELS[kind]:
        allowed = ', '.join(repr(v) for v in TASK_LABELS[kind])
        return False, f'Invalid label for {kind}: use one of {allowed}'
    if data['scene_type'] not in ROOM_SCENE_TYPES:
        return False, 'Invalid scene_type: use one of ' + ', '.join(ROOM_SCENE_TYPES)
    pos = data['pos']
    finite = (
        isinstance(pos, list) and len(pos) == 2
        and all(
            type(value) is int or (type(value) is float and math.isfinite(value))
            for value in pos
        )
    )
    if kind == 'qr':
        if not data['message'].strip():
            return False, 'QR requires the decoded message'
        # The format allows null here and only the message is scored; our reports
        # always carry a position all the same (user, 2026-11-24).
        if pos is not None and not finite:
            return False, 'QR pos must be a finite [x, y] or null'
        return True, ''
    if data['message']:
        return False, 'message must be empty for this task'
    if not finite:
        return False, 'This task requires a finite room/cell centre [x, y]'
    return True, ''


def extract_scene_result(response):
    """Unwrap the lab response and check its navigation observations."""
    data = parse_scene_json(response)
    if data is not None and data.get('success') is False:
        raise ValueError('Lab VLM server reported failure')
    if data is not None and 'result' in data:
        data = parse_scene_json(data['result'])
    if data is None:
        raise ValueError('VLM reply did not contain a scene observation')

    paths = data.get('paths')
    if not isinstance(paths, list) or not paths:
        raise ValueError('Scene observation must include path observations')
    for path in paths:
        if (
            not isinstance(path, dict)
            or path.get('direction') not in ('forward', 'left', 'right')
            or path.get('status') not in PATH_STATUSES
            or not isinstance(path.get('notes'), str)
        ):
            raise ValueError('Each path requires a direction, status and notes')
    return data


def build_task_report(group_id, task_id, scene_type, label='', pos=None,
                      message=''):
    """
    Build and validate the scoring payload once a task is confirmed.

    scene_type: one of ROOM_SCENE_TYPES (required for every task, QR too).
    label: an OBJECT_LABELS entry for M01, a HAZARD_LABELS entry for M02,
        '' for survivors and QR.
    pos: [x, y] centre of the cell (m, arena frame: origin at the start cell
        centre, +x the initial forward direction, +y left), rounded to 2
        decimals; required except for QR (where it may be None).
    message: the decoded QR text for M04, else ''.
    Raises ValueError with the reason when the server would reject it.
    """
    kind = MISSIONS.get(task_id)
    if kind is None:
        raise ValueError(f'Unknown mission ID: {task_id}')
    if pos is not None:
        pos = [round(float(value), 2) for value in pos]
    report = {
        'group_id': group_id,
        'task_id': task_id,
        'task_type': kind,
        'scene_type': scene_type,
        'label': label,
        'pos': pos,
        'message': message,
    }
    valid, error = validate_scene_result(report)
    if not valid:
        raise ValueError(error)
    return report


def cell_centre(x, y, cell_size=CELL_SIZE):
    """Centre [x, y] of the arena cell containing (x, y), 2 decimals."""
    return [round(round(x / cell_size) * cell_size, 2),
            round(round(y / cell_size) * cell_size, 2)]


def report_position(x, y, yaw=None, mode='ahead', cell_size=CELL_SIZE):
    """
    Cell centre to report for a scene seen from robot pose (x, y, yaw).

    mode 'robot': the cell the robot stands in. mode 'ahead': the next cell
    along the heading (yaw snapped to the nearest grid direction), i.e. the
    room looked into from the cell before it. Arena frame, 2 decimals.
    """
    if mode not in REPORT_CELLS:
        raise ValueError(f'report_cell must be one of {REPORT_CELLS}, got {mode!r}')
    i, j = round(x / cell_size), round(y / cell_size)
    if mode == 'ahead':
        if yaw is None or not math.isfinite(yaw):
            raise ValueError('report_cell ahead needs the robot yaw')
        di, dj = ((1, 0), (0, 1), (-1, 0), (0, -1))[round(yaw / (math.pi / 2)) % 4]
        i, j = i + di, j + dj
    return [round(i * cell_size, 2), round(j * cell_size, 2)]


def mission_reports(scene, pos):
    """
    Turn a scene observation's mission_matches into task report fields.

    Every entry carries `pos` (the reported cell centre, or None if unknown)
    and the candidate task_ids for its type. `reportable` is True when the
    scoring server would accept it once a task_id is chosen; `problems`
    lists what is missing otherwise (label, scene_type, pos, message).
    """
    reports = []
    scene_type = str(scene.get('scene_type') or '').strip().lower()
    for match in scene.get('mission_matches') or []:
        if not isinstance(match, dict):
            continue
        kind = str(match.get('kind') or '').strip().lower()
        if kind not in TASK_LABELS:
            continue
        label = (str(match.get('label') or '').strip().lower()
                 if kind in ('object', 'hazard') else '')
        message = str(match.get('message') or '').strip() if kind == 'qr' else ''
        problems = []
        if label not in TASK_LABELS[kind]:
            problems.append('label')
        if scene_type not in ROOM_SCENE_TYPES:
            problems.append('scene_type')
        if pos is None:
            problems.append('pos')
        if kind == 'qr' and not message:
            problems.append('message')
        task_ids = list(TASK_IDS[kind])
        if kind == 'object' and label != M01_OBJECT:
            # seen and recorded, but M01 asks for the hand tools only (booklet 2026-11-24)
            task_ids = []
            problems.append('no mission asks for this object')
        reports.append({
            'task_type': kind, 'task_ids': task_ids,
            'scene_type': scene_type, 'label': label,
            'pos': list(pos) if pos is not None else None,
            'message': message, 'evidence': str(match.get('evidence') or ''),
            'reportable': not problems, 'problems': problems,
        })
    return reports


def send_task_report(report, url=TASK_REPORT_URL, timeout=5.0):
    """
    POST one validated task report to the scoring server.

    Returns the server's JSON reply ({"status": "accepted" | "rejected",
    ...}). Only the latest valid submission per task_id is scored, so a
    corrected report may be sent again. Raises ValueError for an invalid
    report (nothing is sent) and requests exceptions on network errors.
    """
    import requests

    valid, error = validate_scene_result(report)
    if not valid:
        raise ValueError(error)
    reply = requests.post(url, json=report, timeout=timeout)
    reply.raise_for_status()
    return reply.json()
