import json

SCENE_PROMPT = '''You are the scene-understanding module of a quadruped robot dog operating inside a competition arena.

Analyse ONLY what is visibly supported by the image. Do not give navigation commands, recommendations, or invent objects, hazards, survivors, rooms, QR messages, or positions.

Current mission:
group_id: {group_id}
task_id: {task_id}
task_type: {task_type}
mission cue: {mission_cue}

Allowed task_type values:
object, hazard, survivor, qr

Allowed scene_type values:
kitchen, market, workshop, clinic, cleaning_room

Allowed label values:
medical_kit, hand_tools, food, water, utensils, high_voltage, radioactive, chemical, biological, flammable

Use only the allowed vocabulary above. If something cannot be determined reliably, use the required empty value instead of guessing.

Task-specific rules:
- object: identify label and scene_type; message=""
- hazard: identify label and scene_type; message=""
- survivor: identify scene_type; label=""; message=""
- qr: identify message; label=""; scene_type=""; pos=null

Position:
"pos" is the estimated arena position [x, y]. Do NOT estimate coordinates from the image alone. If reliable position information is unavailable, use null.

Reply with ONLY one JSON object, no markdown or extra text, using exactly these keys:
{{
  "group_id": "{group_id}",
  "task_id": "{task_id}",
  "task_type": "{task_type}",
  "scene_type": "",
  "label": "",
  "pos": null,
  "message": ""
}}

Rules:
- unused string fields = ""
- unused/unknown pos = null
- do not use "unknown", "none", or "N/A"
- do not add extra keys
'''

NO_CUE = 'none specified'

VALID_TASK_TYPES = {'object', 'hazard', 'survivor', 'qr'}
VALID_SCENE_TYPES = {'', 'kitchen', 'market', 'workshop', 'clinic', 'cleaning_room'}
VALID_LABELS = {
    '', 'medical_kit', 'hand_tools', 'food', 'water', 'utensils',
    'high_voltage', 'radioactive', 'chemical', 'biological', 'flammable'
}


def build_prompt(mission_cue='', group_id='group04_XXXX', task_id='', task_type=''):
    """Build the competition scene-understanding prompt."""
    return SCENE_PROMPT.format(
        mission_cue=mission_cue.strip() or NO_CUE,
        group_id=group_id.strip(),
        task_id=task_id.strip(),
        task_type=task_type.strip().lower()
    )


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


def validate_scene_result(data):
    """Validate the competition result fields and vocabulary."""
    required = {'group_id', 'task_id', 'task_type', 'scene_type', 'label', 'pos', 'message'}

    if not isinstance(data, dict):
        return False, 'Result is not a JSON object'
    if set(data.keys()) != required:
        return False, 'Result does not contain exactly the required fields'
    if data['task_type'] not in VALID_TASK_TYPES:
        return False, f'Invalid task_type: {data["task_type"]}'
    if data['scene_type'] not in VALID_SCENE_TYPES:
        return False, f'Invalid scene_type: {data["scene_type"]}'
    if data['label'] not in VALID_LABELS:
        return False, f'Invalid label: {data["label"]}'

    pos = data['pos']
    if pos is not None and not (
        isinstance(pos, list) and len(pos) == 2 and
        all(isinstance(v, (int, float)) for v in pos)
    ):
        return False, 'pos must be [x, y] or null'

    if not all(isinstance(data[k], str) for k in ['group_id', 'task_id', 'message']):
        return False, 'group_id, task_id and message must be strings'

    return True, ''