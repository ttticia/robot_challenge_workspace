"""
Mission-focused VLM prompt and parsing of its JSON answer.

No ROS dependencies. The VLM only describes the scene; it must never issue
motion commands. Decisions are left to the mission/state machine.
"""

import json

SCENE_PROMPT = '''You are the scene-understanding module of a quadruped robot \
dog exploring a maze. The image is from its forward-facing camera, taken when \
a junction was detected. Report only what is visible; do not guess. Describe \
directions from the robot's viewpoint. Do NOT give motion commands or \
recommendations; another module decides what to do.

Mission cue to look for: {mission_cue}

Reply with ONLY one JSON object, no markdown, using exactly these keys:
{{
  "scene_type": "junction | corridor | dead_end | open_area | doorway | \
other",
  "junction_confirmed": true | false,
  "terrain": {{"type": "e.g. carpet, tiles, concrete, grass, gravel",
               "traversable": true | false, "notes": ""}},
  "paths": [{{"direction": "left | forward_left | forward | forward_right | \
right", "status": "open | blocked | partially_blocked | unknown", \
"notes": ""}}],
  "hazards": [{{"type": "e.g. step, hole, obstacle, wet floor, person", \
"location": "", "severity": "low | medium | high"}}],
  "objects": [{{"label": "", "location": "", "mission_relevant": true | \
false}}],
  "signs_or_markings": [{{"content": "text, arrow or symbol", \
"location": "", "meaning": ""}}],
  "mission_cue": {{"matched": true | false | null, "evidence": ""}},
  "confidence": 0.0,
  "summary": "one short sentence for the mission manager"
}}
Use [] for empty lists. Use null for mission_cue.matched if no cue is given. \
confidence is 0.0-1.0 for the whole answer.'''

NO_CUE = 'none specified'


def build_prompt(mission_cue=''):
    """Return the scene prompt for an optional mission cue."""
    return SCENE_PROMPT.format(mission_cue=mission_cue.strip() or NO_CUE)


def parse_scene_json(text):
    """
    Extract the JSON object from the VLM answer; return a dict or None.

    Tolerates markdown code fences or text around the object.
    """
    if not text:
        return None
    start, end = text.find('{'), text.rfind('}')
    if start < 0 or end <= start:
        return None
    try:
        data = json.loads(text[start:end + 1])
    except ValueError:
        return None
    return data if isinstance(data, dict) else None
