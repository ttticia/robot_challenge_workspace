"""Which mission report each observation belongs to (no ROS in here).

Booklet as updated 2026-11-24: five fixed missions, the same in practice, prelims and finals.

  M01  object    the hand tools: scene type, label hand_tools, position
  M02  hazard    the hazard warning sign: scene type, its label, position
  M03  survivor  the conscious survivor = the one WITH a QR code: scene type, position
  M04  qr        that QR code's text, exactly as decoded, with pos null (the server
                 refuses a position) and the scene type the VLM gave the room it was read
                 in: sent once that room's VLM result is in, never with a guessed type
  M05  survivor  the other survivor, WITHOUT a QR code: scene type, position

Each arena has exactly two survivors and one QR code. Positions are cell centres in the
arena frame (origin = centre of the start cell, +X along the start heading, +Y to the left),
so a cell (i, j) reports as [i * cell_size, j * cell_size].

One answer per mission (booklet: "Each mission has exactly one intended target and one
correct response", and the server keeps only the latest report per task_id). So:
  - M01 answers for the object the mission asks for and for nothing else. That target is a
    setting (`m01_object`, default hand_tools: the missions may still change, user
    2026-11-24). Other objects seen (say medical_equipment in a clinic) are recorded in
    `seen` and never submitted.
  - M02 answers for a hazard sign: any label, or only `m02_hazard` when that is set.
  - Survivors: the room the QR code is read in is M03 and the other survivor M05. Next
    best: the room where scene understanding saw a survivor AND a QR code (seen, not
    read) is M03 at once. With neither, the first survivor found is M03 and the second M05 (user,
    2026-11-25; on 2026-11-24 10:54 a lone survivor went out as M05 and scored 0).
  - A report the server marks as not successful (rejected, or evaluated with a field
    wrong: practice mode) lets go of M01 / M02: the next sighting elsewhere is reported
    in its place (`refused`, user 2026-11-26).
  - The first sighting holds the mission. The same target seen again elsewhere does not
    replace it. A report is sent again only when the SAME finding is known better (its
    room is named, a survivor's QR code is read).
"""
import math
import re

SCENE_TYPES = ('clothes_shop', 'fitness_centre', 'kitchen', 'market', 'workshop', 'clinic',
               'cleaning_room', 'furniture_shop')
HAZARDS = ('high_voltage', 'radioactive', 'chemical', 'biological', 'flammable')
M01_LABEL = 'hand_tools'
OBJECTS = ('medical_equipment', 'hand_tools', 'food', 'water', 'utensils')
TASK_TYPES = {'M01': 'object', 'M02': 'hazard', 'M03': 'survivor', 'M04': 'qr', 'M05': 'survivor'}
KEYS = ('group_id', 'task_id', 'task_type', 'scene_type', 'label', 'pos', 'message')


def look_cell(cell, side):
    """The cell looked into from `cell` through `side` (0 = the start heading, 1 = left,
    2 = back, 3 = right)."""
    di, dj = ((1, 0), (0, 1), (-1, 0), (0, -1))[int(side) % 4]
    return (int(cell[0]) + di, int(cell[1]) + dj)


def check(report):
    """Is this report fit to submit? Returns (ok, reason).

    The booklet's format (seven keys, the fixed vocabulary). M01, M02, M03 and M05 must
    carry a position; the QR report M04 must NOT (pos null: the server rejects it otherwise)."""
    if not isinstance(report, dict) or tuple(report) != KEYS:
        return False, 'not exactly the seven keys, in order: ' + ', '.join(KEYS)
    if not all(isinstance(report[k], str) for k in KEYS if k != 'pos'):
        return False, 'every field except pos must be a string'
    if not re.fullmatch(r'group[0-9]{2}_[A-Za-z0-9]{4}', report['group_id']):
        return False, f'group_id {report["group_id"]!r} is not group##_XXXX'
    kind = TASK_TYPES.get(report['task_id'])
    if kind is None:
        return False, f'unknown task_id {report["task_id"]!r}'
    if report['task_type'] != kind:
        return False, f'{report["task_id"]} is a {kind} task, not {report["task_type"]!r}'
    if report['scene_type'] not in SCENE_TYPES:
        return False, f'scene_type {report["scene_type"]!r} is not in the booklet list'
    allowed = {'object': OBJECTS, 'hazard': HAZARDS}.get(kind, ('',))
    if report['label'] not in allowed:
        return False, f'label {report["label"]!r} is not valid for a {kind} task'
    pos = report['pos']
    if kind == 'qr':
        # The server refuses a QR report with a position (08:01 run, 2026-11-26, HTTP 422:
        # "QR requires nonempty message, null pos, empty label, and empty/permitted
        # scene_type"). Until then every report carried one (user, 2026-11-24).
        if pos is not None:
            return False, 'the QR report must have pos null (the server rejects a position)'
    elif not (isinstance(pos, list) and len(pos) == 2 and all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            for v in pos)):
        return False, 'pos must be [x, y] (every report carries a position)'
    if kind == 'qr' and not report['message'].strip():
        return False, 'the QR report needs the decoded message'
    if kind != 'qr' and report['message']:
        return False, 'message must be empty for this task'
    return True, ''


COURSES = {'L': 'A', 'R': 'B'}       # maze on the left = course A, on the right = course B


def course_url(report_url, maze_side):
    """The scoring server's address for this run's course (user, 2026-11-26): the course
    follows maze_side, L = A, R = B, as '<report_url>_A' / '<report_url>_B'. A report_url
    that already ends in _A or _B is a course chosen by hand and is used as it is.
    Returns '' when neither says which course: nothing must be posted to a guessed one."""
    url = str(report_url or '').strip().rstrip('/')
    if url.endswith(('_A', '_B')):
        return url
    course = COURSES.get(str(maze_side or '').strip().upper()[:1])
    return f'{url}_{course}' if url and course else ''


class Missions:
    def __init__(self, group_id='', cell_size=1.02, qr_wait=15.0, m01_object=M01_LABEL,
                 m02_hazard=''):
        self.group_id, self.cell_size, self.qr_wait = group_id, cell_size, qr_wait
        if m01_object not in OBJECTS:
            raise ValueError(f'm01_object {m01_object!r} is not one of {OBJECTS}')
        if m02_hazard and m02_hazard not in HAZARDS:
            raise ValueError(f'm02_hazard {m02_hazard!r} is not one of {HAZARDS}')
        self.m01_object = m01_object       # the object mission M01 asks for
        self.m02_hazard = m02_hazard       # the hazard M02 asks for; '' = whichever is found
        self.scenes = {}       # cell -> scene type (one of SCENE_TYPES)
        self.survivors = {}    # cell -> time a survivor was first seen there
        self.objects = {}      # cell with the hand tools -> True
        self.hazards = {}      # cell -> hazard label
        self.qr_text = None
        self.qr_cell = None    # the room the code was read in (this is what makes M03)
        self.qr_near = None    # where the robot stood at the read: only M04's position
        self.qr_seen = None    # room where a survivor AND a QR code were seen (not read)
        self.sent = {}         # task_id -> last payload handed out
        self.seen = {}         # (cell, 'object' | 'hazard' | 'survivor', label) -> first time
        self.bad = {'M01': set(), 'M02': set()}   # cells whose report the server marked wrong

    # ---- observations ---------------------------------------------------------------------
    def scene(self, cell, scene_type, objects=(), hazards=(), humans=False, now=0.0,
              qr=False):
        """One analysed room. Returns notes about anything left out.
        qr = a QR code was SEEN there (by the VLM; not read)."""
        cell, notes = tuple(cell), []
        if qr and humans and self.qr_seen is None:
            self.qr_seen = cell            # survivor + code in one room: the M03 survivor
        if scene_type in SCENE_TYPES:
            self.scenes[cell] = scene_type
        else:
            notes.append(f'cell {list(cell)}: scene type {scene_type!r} is not in the booklet '
                         'list; nothing in this room can be reported until it is')
        for o in objects or ():
            if o not in OBJECTS:
                notes.append(f'cell {list(cell)}: object {o!r} is not a booklet label')
                continue
            new = self.note((cell, 'object', o), now)
            if o != self.m01_object:
                if new:
                    notes.append(f'cell {list(cell)}: {o} seen and recorded; not submitted '
                                 f'(mission M01 asks for {self.m01_object})')
                continue
            first = next(iter(self.objects), cell)
            if first != cell and new:
                notes.append(f'cell {list(cell)}: {o} seen again; M01 stays with the first '
                             f'sighting in cell {list(first)}')
            self.objects.setdefault(cell, True)
        for h in hazards or ():
            if h not in HAZARDS:
                notes.append(f'cell {list(cell)}: hazard {h!r} is not a booklet label')
                continue
            new = self.note((cell, 'hazard', h), now)
            if self.m02_hazard and h != self.m02_hazard:
                if new:
                    notes.append(f'cell {list(cell)}: hazard {h} seen and recorded; not '
                                 f'submitted (mission M02 asks for {self.m02_hazard})')
                continue
            first = next(iter(self.hazards), cell)
            if first != cell and new:
                notes.append(f'cell {list(cell)}: hazard {h} seen; M02 stays with the first '
                             f'sighting ({self.hazards[first]} in cell {list(first)})')
            if first == cell:
                self.hazards[cell] = h
            else:
                self.hazards.setdefault(cell, h)
        if humans:
            self.note((cell, 'survivor', ''), now)
            self.survivors.setdefault(cell, now)
        return notes

    def note(self, key, now):
        """Remember a sighting; True when it is new."""
        if key in self.seen:
            return False
        self.seen[key] = now
        return True

    def sightings(self):
        """Everything seen so far, submitted or not: one dict per (cell, kind, label)."""
        return [{'cell': list(cell), 'pos': self.pos(cell), 'kind': kind, 'label': label,
                 'scene_type': self.scenes.get(cell, 'unknown')}
                for (cell, kind, label) in self.seen]

    def qr(self, text, cell, near=None):
        """A decoded QR code; cell = the room it was read in, or None if not known.
        near = the cell the robot stood in, used for M04's position when no room is known."""
        if self.qr_text is not None and text != self.qr_text and cell is None:
            return                      # a second, unplaced code: keep the first
        self.qr_text = text
        if cell is not None:
            self.qr_cell = tuple(cell)
        elif near is not None and self.qr_near is None:
            self.qr_near = tuple(near)

    def refused(self, payload):
        """The server did not mark this report as successful (user, 2026-11-26). For M01
        and M02 the sighting it came from gives up the mission: the next one seen elsewhere
        is reported in its place (until then the server keeps this report). Survivors are
        left as they are: there are only two, and a code read later re-sorts them."""
        task = payload.get('task_id')
        if task in self.bad and self.sent.get(task) == payload:
            pos = payload.get('pos') or [0, 0]
            self.bad[task].add((round(pos[0] / self.cell_size), round(pos[1] / self.cell_size)))

    # ---- reports --------------------------------------------------------------------------
    def pos(self, cell):
        return [round(cell[0] * self.cell_size, 2), round(cell[1] * self.cell_size, 2)]

    def payload(self, task_id, task_type, scene_type, label='', pos=None, message=''):
        return {'group_id': self.group_id, 'task_id': task_id, 'task_type': task_type,
                'scene_type': scene_type, 'label': label, 'pos': pos, 'message': message}

    def reports(self, now=0.0):
        """Every report that is new or has changed since it was last handed out."""
        want = {}
        # the first sighting holds the mission: a later one elsewhere does not replace it
        for cell in [c for c in self.objects if c not in self.bad['M01']][:1]:
            if cell in self.scenes:
                want['M01'] = self.payload('M01', 'object', self.scenes[cell], self.m01_object,
                                           self.pos(cell))
        for cell, label in [i for i in self.hazards.items() if i[0] not in self.bad['M02']][:1]:
            if cell in self.scenes:
                want['M02'] = self.payload('M02', 'hazard', self.scenes[cell], label,
                                           self.pos(cell))
        with_qr = without_qr = None
        if self.qr_cell is not None:                   # the QR code marks the M03 survivor
            with_qr = self.qr_cell
            others = [c for c in self.survivors if c != self.qr_cell]
            without_qr = others[0] if others else None
        elif self.qr_seen is not None:                 # code seen beside a survivor (user,
            with_qr = self.qr_seen                     # 2026-11-25): M03 at once, no wait
            others = [c for c in self.survivors if c != self.qr_seen]
            without_qr = others[0] if others else None
        else:                                          # no QR placed (yet)
            settled = [c for c, t in self.survivors.items() if now - t >= self.qr_wait]
            # In the order found (user, 2026-11-25): the first survivor is M03, the second
            # M05. A code read later puts M03 in its room and both go out again.
            if settled:
                with_qr = settled[0]
            if len(settled) >= 2:
                without_qr = settled[1]
        if with_qr in self.scenes:
            want['M03'] = self.payload('M03', 'survivor', self.scenes[with_qr], '',
                                       self.pos(with_qr))
        if without_qr in self.scenes:
            want['M05'] = self.payload('M05', 'survivor', self.scenes[without_qr], '',
                                       self.pos(without_qr))
        # M04 carries the scene type the VLM gave the room the code was read in (user,
        # 2026-11-26: nothing hard-coded; the 11:47 run sent 'kitchen', a fixed fallback,
        # for a code read in a cleaning room). It waits for that room's VLM result; a code
        # read outside any known room uses the room where the VLM saw a survivor with a
        # QR code, and with neither it is not sent.
        qr_room = self.qr_cell if self.qr_cell is not None else self.qr_seen
        if self.qr_text is not None and qr_room in self.scenes:
            want['M04'] = self.payload('M04', 'qr', self.scenes[qr_room], '', None,
                                       self.qr_text)   # pos null
        out = [p for t, p in sorted(want.items()) if self.sent.get(t) != p]
        for p in out:
            self.sent[p['task_id']] = p
        return out
