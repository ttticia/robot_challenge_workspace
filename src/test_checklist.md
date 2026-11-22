# Test checklist: changes of 2026-11-21/22

Everything below was changed by several parallel work streams and is on the `integration` branch
of the team repo. All of it passes the unit tests. Very little has run on the robot yet.
Tick items off here (`[x]`, with the date and the run) as they pass, and move failures to
`to_do.md`. The design is in `bt.md` (D1–D11); `agent.md` §3 describes the behaviour.

Status at the 2026-11-22 wrap-up push:
- Unit tests: **the full suites were not rerun on the final tree.** Each session's last result:
  src-33 full maze_nav 478 passed, 0 failed; src-68 curtain + rolling-pin tests 99 passed, queue
  tests 62 passed; src-8b exit / challenge-maze / timeout tests pass; scene_understanding 6
  passed. team_perception has 1 known failure (`test_qr_unread`, older than these changes).
  First item below: run the full suites.
- Simulator: the example maze (`maze_cols:=0`, `depth_rate:=0`) reached `EXITED` in 128–309 s,
  slower than before because of the junction scans and queue checks. The curtain maze reached
  `EXITED`. Minimum wall clearance 4.2 cm. On `challenge_a`, two clean runs (src-8b; src-e2's
  rolling-pin rerun) both stuck at the curtain: `AT_CURTAIN` at (1.70, 1.0), "not through
  after 120 s". No `challenge_*` maze has reached `AT_EXIT` yet.
- Robot: nothing from this list yet.

## 1. Before the robot (simulator and unit tests)

How to run: `skills.md` recipes 2 and 3. Use a free ROS domain; other sessions may have sims running.

- [ ] Full unit suites pass on the final tree (maze_nav, scene_understanding, team_perception).
- [ ] **Grid exit, every `challenge_*` maze** (`src/maze_nav/test/challenge_*.json`, made by
      `make_challenge_mazes.py`; being reworked on 2026-11-22, list in `agent.md` §9): each ends
      `AT_EXIT` in the maze's launch cell, and the sim never logs `OUT OF THE MAZE`. Not passed
      yet. The 22:00–22:45 runs on 2026-11-22 were polluted (leftover simulators on the same
      domain), so their failures prove nothing. The one clean `challenge_a` run got stuck at the
      curtain on the route (`AT_CURTAIN` after 120 s, timed out). Run `challenge_a_no_curtain` and
      `challenge_a_flat` (no curtain / stairs) first, to test the grid exit on its own.
- [ ] Rolling pin (18 cm, across `challenge_a` cell (2, 0)): exploration walks over it with
      H-Step and the sensors don't read it as a wall, stairs or curtain. The 22:30 run never left
      the start cell (CPU load 27: SENSOR STALE), so it's unverified (`to_do.md` obstacles).
- [ ] Facing the exit at `AT_EXIT` works in the simulator (only unit-tested so far).
- [ ] Example maze and curtain maze still reach `EXITED` with `maze_cols:=0`.
- [ ] Presentation mazes (`maze_stairs.json`, `maze_curtains.json`) run to the end.
- [ ] Simulator depth camera: at 640 px / 5 Hz it delays odometry about 1.5 s and no map forms.
      Find a rate or resolution that works, so the stair line-up can run in the simulator
      (until then `depth_rate:=0`: no stairs in the simulator).
- [ ] A released curtain candidate is actually inspected. In the simulator the check sometimes
      ended "not in front of the robot from here" when the spot 1.2 m in front couldn't be
      reached.
- [ ] Toy-market scene test (`object_maze.json` + market photos, `to_do.md` items 8–13): rerun
      once the VLM is connected; expected `scene_type` `market`.

## 2. On the robot

Robot-run recipe: `skills.md` recipe 6. Record a bag of each run (`rec_maze`). Add the new topics
by hand (`agent.md` §12: `record.sh` doesn't have them yet).

### Startup (D1, D8, D9)
- [ ] Launch with the robot **lying down**: `cmd_vel_bridge` and `wall_scan` log an ERROR every
      5 s, nothing is sent to stand it up, no gait / floor / map / `PAUSED`. Stand it up with the
      handheld controller: startup carries on by itself.
- [ ] Launched standing: Move mode, then H-Step gait sent (`gait sent`), then `PAUSED`.
- [ ] H-Step is steady enough for a full run (it was unsteady once, 2026-11-12). Else `gait:=rug`.
- [ ] After `/maze/start`: 10 cm creep, then straight into `EXPLORING` (no start scan).

### Maze grid and exit (grid fit, launch cell)
- [ ] Start placement: robot square in the start cell. The grid fit only corrects up to 8° /
      0.35 m from the start-pose grid.
- [ ] Log shows `grid fitted`; `/maze/grid` matches the real walls; `scene_trigger` follows it
      (junction and dead-end cells line up with real cells).
- [ ] No goal is sent outside the grid.
- [ ] The exit is chosen only once the grid is explored, and it is the real opening, not a start-cell
      side that read open early. `/maze/launch_cell` says `found`, then `at`.
- [ ] `TO_EXIT` → creeps to the launch cell's centre → turns to face the exit → `AT_EXIT`,
      standing still, facing out.
- [ ] Real layout: the challenge mazes' interior walls are made up. Update `make_challenge_mazes.py`
      (`PASSAGES` / `EXITS`) once the real maze is known.

### Junctions (D2, D5, D6, D7)
- [ ] **The Lite3 follows `/cmd_vel` `linear.y` in auto mode** (never tested). Check first with a
      small sideways command in an open area.
- [ ] At a junction: faces each unvisited way, left-most first, moves sideways onto the cell's
      centre line, holds 1 s with QR on. No sideways move after a turn cut short by > 10°; the
      sideways move stops at an unmapped wall.
- [ ] It doesn't leave until every way is scanned, then leaves by the nearest unexplored way.
- [ ] An object room seen in the scan is marked explored (no goal, no look-in later).

### Candidate queue (D3, D10)
- [ ] Possible stairs or a curtain seen in a junction / dead-end look are queued, not handled on
      the spot (`/stairs/candidate`, `/curtain/candidate`).
- [ ] After the junction scan, or before `NO_EXIT`, candidates are checked nearest first: drive to
      1.2 m in front, face, release, `HANDOVER` within 15 s.
- [ ] A curtain at a junction, or in a corner cell beside an open way, is held back until that
      way is explored (`curtain_branch.Deferral.candidates`, detectors' `hold_while_exploring`).
- [ ] Stairs are only worked in the final queue pass, and until then their cell stays walled on
      `/maze/map` (no Nav2 goal drives onto them). Unit-tested only; not run in the simulator.
- [ ] Possible false stairs from the ramp wedges (`to_do.md` item 11): check on the real ramp.

### Stairs and AI walk (D4)
- [ ] Line-up as before (`AT_STAIRS`, 0.45 m from the first riser, ±2.5°).
- [ ] **AI walk distance 1.3 m** is right for the real stairs (going up; check going down too).
- [ ] **After the walk the code sends auto mode itself** (AI exit → manual → Move → gait → auto).
      This is the first time the code sends auto mode instead of the operator. Keep a hand on the
      controller. Check the robot really takes `/cmd_vel` again and `/maze/ai_status` says `DONE`.
- [ ] A walk that ends early, or `NOT LINED UP`, holds until `/maze/start`.

### Dead ends and scene understanding
- [ ] `scene_understanding:=false`: dead end blacklisted at once after the look (as before).
- [ ] `scene_understanding:=true`: explorer waits for `/scene/result`. Room → blacklisted; stairs /
      curtain → queued; nothing → left open. No answer in 15 s → map re-check, `/scene/retry`
      (max 2 tries), then blacklisted.
- [ ] The VLM backend is wired (organisers' AI server address and format still needed, `to_do.md`
      item 8). Until then every verdict is `unknown`.
- [ ] Camera frame stamps vs local clock (`by arrival` in the log = offset); one more colour
      subscriber doesn't saturate the bridge.
- [ ] A dead end met during a junction scan gets only one view (`to_do.md` item 9).

### Floor tracking and curtains (depth floor, FloorTracker)
- [ ] `/maze/floor_est` follows the real floor: steady when walking on flat floor (bob filtered),
      follows ramps and mats.
- [ ] Start-cell calibration: logged standing height and depth camera tilt look right.
- [ ] Curtain gap measured against the local floor at the curtain line (`curtains.py`
      `gap_at_line`): no false curtains at walls with a cavity (`to_do.md` curtains).
- [ ] Practice curtain (1–3 cm slit between the two halves, 15–20 cm gap under the hem, possibly
      blocked by a raised platform behind it): found by the lidar, and confirmed by the lie-down
      check. Split-curtain joining (C1, `curtains.py`) is off by default (`slit_max 0`). Try it
      on the real slit.
- [ ] YOLO curtain detector: record training bags with `rec_curtain` (new preset; dry run only so
      far), then follow the data steps in `to_do.md` (## curtains). Robot test with
      `curtain_yolo:=true curtain_inspect_mode:=yolo` only once a model passes them.
      The user's decision: curtain detection moves to YOLO then. Until then lidar + lie-down stay
      the defaults.

### Troubleshooting tools
- [ ] `troubleshoot:=true`: `telemetry_snapshot` writes map + state snapshots to `~/live_debug/current`
      without loading the CPU noticeably (`snapshot_camera:=event`).
- [ ] `touble_shooting/live_params.py` changes a parameter on the running nodes.
- [ ] `touble_shooting/trigger_scene.py` triggers a manual dead-end / scene look.
