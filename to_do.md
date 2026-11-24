# To do

Open work for the maze run. Strategy and interfaces are in `AGENTS.md`; this file is the
task list. Competition day: 27 November 2026.

## 1. Scene understanding integration (keep the dog out of object cells)

**Rule (user, 2026-11-23): the dog must not go into a cell that contains objects** (toy
sets such as the market stall and trolley, and any other mission prop). In `run1` it
walked into the market cell and pushed into the props, because the side was classified as
a curtain (debug notes in section 2).

**Goal:** before `maze_explore` enters or pushes into a cell it has not been in, the AI
server looks at it. A cell reported as a room or as holding objects is closed and never
entered; the same answer feeds the mission reports.

### How the AI server is triggered today (`~/test_scene_understanding`, old stack)

- Server: lab perception server, `POST http://192.168.10.141:8000/analyze`, multipart
  form: `group_id` (`group04_grp4`), `prompt`, `image` (JPEG, shrunk to fit 1024x768).
  A busy reply may carry `Retry-After`. Client: `scene_understanding/vlm_client.py`
  (`VLMClient.analyze`), no ROS in it.
- Quick check from the robot computer:
  `ros2 run scene_understanding vlm_test --server http://192.168.10.141:8000 --group group04_grp4 --prompt "Describe the image" --count 1`
- In a run, `room_scene` does the call:
  1. The explorer stops in the cell before a dead end and holds three views: `centre`,
     then `left` and `right`. At the end of each hold it publishes `/scene/capture`
     (`std_msgs/String`, JSON): `{cell, cell_centre, view, last, stamp, hold_start, kind,
     attempt, robot, ...}`.
  2. `room_scene` keeps the last seconds of `/camera/color/image_raw/compressed`, picks the
     frame inside each hold, stitches left | centre | right and sends **one** request with
     `scene_prompt.SCENE_PROMPT`.
  3. It publishes `/scene/result` (JSON): `scene_type` (one of the eight room types, or
     `corridor` / `open_area` / `junction` / `unknown` / `stairs` / `curtain`), `objects`,
     `hazards`, `signs`, `humans`, `qr`, `pos` (the scanned cell's centre), `reports`.
     A failed call comes back with `vlm: "none"` and an `error`.
  4. The old explorer read the verdict: a room type or any object -> the dead end is
     blacklisted; stairs / curtain -> candidate; no answer in 15 s -> `/scene/retry`
     `{cell, attempt}`, at most 2 tries.
- `challenge_ws/src/scene_understanding/room_scene.py` is the older copy: same topics, but
  `vlm` is fixed to `none`.

### Tasks

- [ ] **Bring the VLM-wired `room_scene` into `challenge_ws`.** Take `room_scene.py`,
      `vlm_client.py`, `scene_prompt.py` and their tests from
      `~/test_scene_understanding/src/scene_understanding`. Merge, do not overwrite: the
      `challenge_ws` copy has `depth_mask.py` and depth options the other lacks.
- [ ] **Explorer publishes `/scene/capture`.** `maze_explore` already holds `centre`,
      `left`, `right` on a room look and publishes `room centre|left|right cell (i, j) side
      d` on `/explore/event`. Publish the JSON capture at the end of each hold as well
      (`cell` = the cell looked into, `cell_centre` = `[i * 1.02, j * 1.02]`, `hold_start`,
      `stamp`, `view`, `last` on the third view, `kind`, `attempt`).
- [ ] **Explorer reads `/scene/result`.** New ROS-free helper (in `plan.py` or next to it)
      that turns a result into a verdict for the cell:
      - room type, or any of `objects` / `hazards` / `humans` not empty -> **closed, never
        entered**;
      - `stairs` / `curtain` -> confirms that candidate;
      - no answer in 15 s -> one `/scene/retry`; still nothing -> **stay out** (the safe
        default) and carry on with other jobs.
- [ ] **Look before entering, not only at dead ends.** Today only a geometric dead end
      (`room`) gets a look. Add a look for (a) any unvisited cell whose interior shows
      returns in the height map, and (b) a `curtain` side before the push. No `visit` or
      `curtain` job into such a cell until its verdict allows it.
- [ ] **Launch.** Add `room_scene` to `explore.launch.py` behind `scene_understanding:=true`
      with `vlm:=lab`, `vlm_server`, `vlm_group`. Check the number of subscribers on the
      compressed colour topic stays within the bridge limit (about 4 copies per stream,
      `AGENTS.md` section 7).
- [ ] **Reports.** `task_reporter`: send the `reports` of each result to
      `http://192.168.10.141:8001/task_report` in the booklet's format (also an open item
      in `AGENTS.md`).
- [ ] **Tests.**
      - Unit: verdict helper (room, objects, stairs, curtain, error, timeout).
      - Offline: send the `run1` frames around 150 s (market stall and trolley) through
        `vlm_test` and check the answer names a market / objects.
      - Simulator has no camera: add a fake `/scene/result` publisher keyed by cell so the
        keep-out logic can be run in `run_sim.sh`.
      - Robot: `drive:=false` first, watching `/scene/result` and `/explore/status`.
- [ ] **Before the run:** confirm the server answers from the robot computer (`vlm_test`
      above) and measure the reply time; it sets how long the dog stands at each look.

## 2. From the `run1` debug (2026-11-23)

Done in code 2026-11-23, checked by unit tests and an offline replay of `run1`, **not yet
run on the robot or in the simulator**:

- [x] Grid fit: the wide fit (30 deg / 0.40 m) is retried at every planning step until it
      works once. Replay: it works on the map at 23 s, before the first move (+6 deg,
      0.23 m, quality 0.82).
- [x] Height rule: a cell with something 0.38 m or taller inside is kind `object`: looked
      at, never entered. Replay: the market cell reads `object` from 47 s on (entered at
      105 s in the run); the four cells walked through read clean.
- [x] Obstacle stop stays on during a curtain push (`push_stop`, default true).
- [ ] Simulator run and a `drive:=false` robot run with these changes.

Open. Curtains are ignored for now (user, 2026-11-23): with `push_stop` on, the fabric
stops the push, so no curtain is crossed.

- [ ] Curtain rule: tall returns across the whole edge, nothing mapped as a wall behind;
      confirm with the curtain YOLO model or the AI server before a push. In the replay
      the 0.9 m frame walls beside cells (0, 0) and (2, 0) also read as `curtain`.
- [ ] Push: judge pose age from `/odom` (the `map -> odom` stamp stops while scans are
      muted, which made the push stop for 1 s every 2.4 s).
- [ ] Push: abort when the body pitches or sinks, and on a total time limit that keeps
      counting through stale-pose stops.
- [ ] `rec_test`: also record `/commands/joy_raw`, so a manual stop shows in the bag.
