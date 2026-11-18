import json
import math
import re

SCENE_PROMPT = """Return one compact JSON observation of the camera frame, even if no mission matches.
Describe forward, left and right relative to the camera. Use open only for a
visible continuing route, partially_blocked or blocked for visible obstructions,
and unknown when the route cannot be seen. In notes, name only the decisive
visible cue in at most six words, or use "" if status is self-explanatory.
Mention an obstacle only once unless it affects multiple directions. Include
terrain only when it affects a path. Do not infer distance, clearance beyond
the frame, or movement commands. A hanging sign, curtain or cutout can obscure
the route even when floor is visible underneath. Use unknown if clearance or
the route behind it is unclear, or blocked if it visibly blocks the route.

Check all categories in every frame, even when the room is unknown.
Room cues for scene_type: kitchen (cooking or food-prep equipment), market
(displayed goods or stalls), workshop (workbenches or tools), clinic (medical
equipment or signage), cleaning_room (cleaning supplies or equipment). Use
open_area, corridor, junction or unknown when appropriate. Use null for
uncertain junction status.

Object labels: medical_kit (first aid supplies), hand_tools (e.g. wrench,
hammer, pliers, screwdriver), food, water, utensils (e.g. cutlery, ladle).
Hazard labels from the reporting requirements: high_voltage, radioactive,
chemical, biological, flammable. Also recognize all ten reference signs by
their symbol inside a yellow triangle:
- radiation trefoil -> radioactive; lightning bolt -> high_voltage;
  exclamation mark -> general_warning; horseshoe magnet -> magnetic_field;
  antenna with waves -> non_ionizing_radiation.
- biohazard symbol -> biological; starburst with horizontal beam -> laser;
  cresting wave -> wave_warning; sun-like rays around a ring -> sunburst_warning;
  three linked circles around a centre -> three_circle_warning.
Use chemical or flammable when the symbol or text clearly supports it.
Use the visual label for an ambiguous symbol instead of guessing its meaning.

Any recognizable depiction of a human (photo, poster or printed human figures,
including cutouts) counts as a survivor, even if partly cropped or the room
is unknown. Use kind="survivor" and label="". People physically present are
reported as sightings when there is no evidence they are survivor markers.
For QR codes, use kind="qr" and label="" for an exactly decoded message.
Use sightings with kind="qr" and a short note when a code is visible but
unreadable. Never invent QR text.

List each supported candidate in mission_matches with kind (object, hazard,
survivor or qr), label, a short evidence phrase, and message (empty except
for QR). List each distinct candidate once. Use [] if none are supported.
Use optional sightings with kind and a short note for inconclusive clues.
Omit sightings when empty. Do not assign task IDs or infer match from room alone.

Output only minified JSON. Use these required fields at the top level:
{"scene_type":"unknown","junction_confirmed":null,"paths":[
{"direction":"forward","status":"unknown","notes":""},
{"direction":"left","status":"unknown","notes":""},
{"direction":"right","status":"unknown","notes":""}],"mission_matches":[]}
"""

# Task type and label used by the competition reporting helper.
MISSIONS = {
    'M01': ('object', 'medical_kit'),
    'M02': ('hazard', 'high_voltage'),
    'M03': ('survivor', ''),
    'M04': ('qr', ''),
    'M05': ('survivor', ''),
}
VALID_SCENE_TYPES = {'', 'kitchen', 'market', 'workshop', 'clinic', 'cleaning_room'}
PATH_STATUSES = ('open', 'partially_blocked', 'blocked', 'unknown')


def build_prompt(three_view=False):
    """Build the automatic scene observation prompt."""
    if three_view:
        return (
            "The image is three equal-width panels ordered left, center, right. "
            "They show separate headings from one junction. Assess directions "
            "relative to the original center heading and use all three panels for "
            "mission evidence. Panel edges are camera boundaries.\n" + SCENE_PROMPT
        )
    return SCENE_PROMPT


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
    """Validate a completed task report."""
    strings = {'group_id', 'task_id', 'task_type', 'scene_type', 'label', 'message'}
    if not isinstance(data, dict) or data.keys() != strings | {'pos'}:
        return False, 'Result does not contain exactly the required fields'
    if not all(isinstance(data[key], str) for key in strings):
        return False, 'All fields except pos must be strings'
    kind = data['task_type']
    try:
        validate_group_id(data['group_id'])
    except ValueError as exc:
        return False, str(exc)
    mission = MISSIONS.get(data['task_id'])
    if mission is None or kind != mission[0]:
        return False, 'Unknown mission or task_type does not match task_id'
    if data['label'] != mission[1]:
        return False, 'Label does not match the mission'
    if data['scene_type'] not in VALID_SCENE_TYPES:
        return False, 'Invalid scene_type'
    pos = data['pos']
    if kind == 'qr':
        if not data['message'].strip() or data['label'] or pos is not None:
            return False, 'QR requires a decoded message, empty label and null pos'
        return True, ''
    if not data['scene_type']:
        return False, 'No supported scene_type was detected'
    if data['message']:
        return False, 'message must be empty for this task'
    if not (
        isinstance(pos, list) and len(pos) == 2
        and all(
            type(value) is int or (type(value) is float and math.isfinite(value))
            for value in pos
        )
    ):
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


def build_task_report(group_id, task_id, scene_type='', pos=None, message=''):
    """Build the scoring payload after a caller confirms task completion."""
    mission = MISSIONS.get(task_id)
    if mission is None:
        raise ValueError(f'Unknown mission ID: {task_id}')
    report = {
        'group_id': group_id,
        'task_id': task_id,
        'task_type': mission[0],
        'scene_type': scene_type,
        'label': mission[1],
        'pos': pos,
        'message': message,
    }
    valid, error = validate_scene_result(report)
    if not valid:
        raise ValueError(error)
    return report
