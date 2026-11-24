"""Which mission report each observation belongs to (no ROS in here).

Booklet as updated 2026-11-24: five fixed missions, the same in practice, prelims and finals.

  M01  object    the hand tools: scene type, label hand_tools, position
  M02  hazard    the hazard warning sign: scene type, its label, position
  M03  survivor  the conscious survivor = the one WITH a QR code: scene type, position
  M04  qr        that QR code's text, exactly as decoded (no position)
  M05  survivor  the other survivor, WITHOUT a QR code: scene type, position

Each arena has exactly two survivors and one QR code. Positions are cell centres in the
arena frame (origin = centre of the start cell, +X along the start heading, +Y to the left),
so a cell (i, j) reports as [i * cell_size, j * cell_size]. The server scores the latest
valid report per task_id, so a report is sent again whenever a better answer is known.
"""
SCENE_TYPES = ('clothes_shop', 'fitness_centre', 'kitchen', 'market', 'workshop', 'clinic',
               'cleaning_room', 'furniture_shop')
HAZARDS = ('high_voltage', 'radioactive', 'chemical', 'biological', 'flammable')
M01_LABEL = 'hand_tools'


class Missions:
    def __init__(self, group_id='', cell_size=1.02, qr_wait=15.0):
        self.group_id, self.cell_size, self.qr_wait = group_id, cell_size, qr_wait
        self.scenes = {}       # cell -> scene type (one of SCENE_TYPES)
        self.survivors = {}    # cell -> time a survivor was first seen there
        self.objects = {}      # cell with the hand tools -> True
        self.hazards = {}      # cell -> hazard label
        self.qr_text = None
        self.qr_cell = None
        self.sent = {}         # task_id -> last payload handed out

    # ---- observations ---------------------------------------------------------------------
    def scene(self, cell, scene_type, objects=(), hazards=(), humans=False, now=0.0):
        """One analysed room. Returns notes about anything left out."""
        cell, notes = tuple(cell), []
        if scene_type in SCENE_TYPES:
            self.scenes[cell] = scene_type
        else:
            notes.append(f'cell {list(cell)}: scene type {scene_type!r} is not in the booklet '
                         'list; nothing in this room can be reported until it is')
        for o in objects or ():
            if o == M01_LABEL:
                self.objects[cell] = True
        for h in hazards or ():
            if h in HAZARDS:
                self.hazards[cell] = h
            else:
                notes.append(f'cell {list(cell)}: hazard {h!r} is not a booklet label')
        if humans:
            self.survivors.setdefault(cell, now)
        return notes

    def qr(self, text, cell):
        """A decoded QR code; cell = the room it was read in, or None if not known."""
        if self.qr_text is not None and text != self.qr_text and cell is None:
            return                      # a second, unplaced code: keep the first
        self.qr_text = text
        if cell is not None:
            self.qr_cell = tuple(cell)

    # ---- reports --------------------------------------------------------------------------
    def pos(self, cell):
        return [round(cell[0] * self.cell_size, 2), round(cell[1] * self.cell_size, 2)]

    def payload(self, task_id, task_type, scene_type, label='', pos=None, message=''):
        return {'group_id': self.group_id, 'task_id': task_id, 'task_type': task_type,
                'scene_type': scene_type, 'label': label, 'pos': pos, 'message': message}

    def reports(self, now=0.0):
        """Every report that is new or has changed since it was last handed out."""
        want = {}
        for cell in self.objects:
            if cell in self.scenes:
                want['M01'] = self.payload('M01', 'object', self.scenes[cell], M01_LABEL,
                                           self.pos(cell))
        for cell, label in self.hazards.items():
            if cell in self.scenes:
                want['M02'] = self.payload('M02', 'hazard', self.scenes[cell], label,
                                           self.pos(cell))
        with_qr = without_qr = None
        if self.qr_cell is not None:                   # the QR code marks the M03 survivor
            with_qr = self.qr_cell
            others = [c for c in self.survivors if c != self.qr_cell]
            without_qr = others[0] if others else None
        else:                                          # no QR placed (yet)
            settled = [c for c, t in self.survivors.items() if now - t >= self.qr_wait]
            if settled:
                without_qr = settled[0]                # a survivor and no code: M05
            if len(settled) >= 2:
                with_qr = settled[1]                   # two survivors, no code read: a guess
        if with_qr in self.scenes:
            want['M03'] = self.payload('M03', 'survivor', self.scenes[with_qr], '',
                                       self.pos(with_qr))
        if without_qr in self.scenes:
            want['M05'] = self.payload('M05', 'survivor', self.scenes[without_qr], '',
                                       self.pos(without_qr))
        if self.qr_text is not None:
            # Only the message is scored, but the server rejects an unknown scene type.
            scene = self.scenes.get(self.qr_cell) or next(iter(self.scenes.values()), 'kitchen')
            want['M04'] = self.payload('M04', 'qr', scene, '', None, self.qr_text)
        out = [p for t, p in sorted(want.items()) if self.sent.get(t) != p]
        for p in out:
            self.sent[p['task_id']] = p
        return out
