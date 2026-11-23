What to test after the 2026-11-21/22 changes (simulator and robot): `test_checklist.md`.

## stairs
reduce stair detection CPU usage. 

stairs were missed in 2nd last run. need to make stair check more robust. 

Stairs last + curtains after the other way (built 2026-11-22, unit tests only, NOT run in sim or on the robot):
- Stairs are crossed only once nothing else is left to explore. stair_detector `hold_while_exploring` (default true) holds stairs seen while exploring, even confirmed ones right in front, instead of handing over. maze_explorer queues them and works them only in the final queue pass. Until then the cell up from the first riser is drawn as wall on /maze/map (Nav2's static layer). Test next: sim maze with stairs mid-maze. The robot should stop short of them, explore everything else, then come back and cross. Check that the walled cell lines up with the real stairs (first riser on a cell edge?) and that the robot is never walled in.
- A curtain at a junction, or in a corner cell (one solid wall, the curtain, one open way, the way in), is noted but held back until the cell's other way is explored. curtain_detector `hold_while_exploring` (default true) now publishes candidates seen while exploring. Test next: sim maze like the user's sketch (corridor, curtain ahead, room to the right). The robot should go right first, then come back for the curtain. Watch for extra false curtain candidates while exploring (side windows): they now enter the queue.

## curtains
some walls have a cavity that could be read as curtain gap. when checking for curtain look for curtain gap that extends >10cm behind the supposed curtain.

### YOLO curtain detector (built 2026-11-22, OFF: no data, no model)
`maze_nav/curtain_yolo.py` (launch `curtain_yolo:=false`): YOLO curtain boxes in the colour camera, placed with the depth relay, on `/curtain/yolo`. It runs only in junction / dead-end looks and CURTAIN, never while EXPLORING. `curtain_detector` `inspect_mode` `yolo` (launch `curtain_inspect_mode:=yolo`): stand 1 m in front of a released candidate for 3 s; confirmed if ≥ 3 frames have a box (conf ≥ 0.5) on it. Once trained, this should replace the lie-down gap check as the curtain verification. Until then: `lie`. Missing data, in order:
1. Record bags with the colour camera + depth + /tf_static: `rec_curtain <name>` (ROSBAG_GUIDE 2.5; it records raw /livox/points by itself without maze_nav, plus odometry, IMU and the controller), with the competition curtains, approaching from ~2 m to 0.3 m (detection range 1.5–0.3 m, user 2026-11-23), square on and 30–45° off, under the lab lights, lit and in shadow, still and swaying. At least 10 bags, several sessions and days. Also negatives: plain walls, extrusion walls with cavities/windows, doorways, slats, open floor, people. Both curtain sides.
2. Frames + draft labels: `nice -n 19 python3 src/maze_nav/tools/curtain_yolo_frames.py <bag> ~/challenge_ws/datasets/curtain --every 10 --lidar --cloud-topic /livox/points [--max-gap 0.25 for a curtain ~20 cm up]` for every bag (frames named by bag, so all go into one dataset).
3. Review EVERY label in CVAT / Label Studio / labelImg (YOLO format, class 0 = curtain): fix the drafts, box the curtains the depth draft missed, and delete boxes on walls. Target ≥ 1000 curtain boxes and as many negative frames.
4. Train: `python3 src/maze_nav/tools/train_curtain_yolo.py ~/challenge_ws/datasets/curtain --install` (yolo11n, imgsz 416, split by bag; installs `~/.cache/team_perception/curtain_yolo.pt` + an OpenVINO export). Copy it to the robot's `~/.cache/team_perception/`.
5. Accept only if, on bags NOT used in training: recall ≥ 0.9 for curtains 0.3–1.5 m away, and no box on the negatives at conf 0.5. Check CPU on the robot (~180 ms a frame at 416 on torch here; use the OpenVINO model). Check that one more colour subscriber doesn't saturate the bridge.
6. Robot test with `curtain_yolo:=true curtain_inspect_mode:=yolo` on the spec curtain and on a cavity wall. If it holds up, make `yolo` the default verification and keep `lie` as the fallback.

**Dataset state 2026-11-23** (`~/challenge_ws/datasets/curtain`, NOT in git; README.md there): 396 frames after de-duplication (`dedup_frames.py 0.45`), every label hand-checked: 205 curtain / 191 negative. Robot bags: curtain_test1, curtain_lab_45deg_lit_01, curtain_slit_square_lit_01, curtain_bag1 (degraded recording), curtain_bag2 (163 curtain views in all), curtain_n1 (negatives only); plus phone_IMG_1671/1672 (42 curtain frames from phone videos in maze_examples/curtains - training only, never val/test). Checked: no missing/orphan/invalid labels, `--cv --dry-run` gives 5 folds (17-50 curtain frames in each val), test_curtain_yolo.py passes. Nothing blocks a first training run. Before / around it:
- [ ] Commit `tools/train_curtain_yolo.py` + `test/test_curtain_yolo.py` (bag-safe split: `--val-bags`, `--test-bags`, `--cv`, `--dry-run`, phone frames train-only).
- [ ] Back up the dataset again (zip images/ labels/ manifest.csv README.md data.yaml + scripts; the 2026-11-23 zips predate curtain_bag2 and the phone frames).
- [ ] Training time: CPU here ~2 h per fold at imgsz 320 / 80 epochs (estimate), `--cv` = 5 folds. Quick check first, one fold: `nice -n 19 python3 src/maze_nav/tools/train_curtain_yolo.py ~/challenge_ws/datasets/curtain --val-bags curtain_lab_45deg_lit_01 --imgsz 320 --epochs 50`; full `--cv --imgsz 320` overnight or on a GPU machine. Never while the robot stack runs.
- [ ] Final test bag: record one more curtain bag and keep it out of everything (`--test-bags`) for the acceptance test (step 5).
- [ ] Evaluation script (needed to judge the model, not to train): recall per distance band from manifest.csv `ahead_m` (0.3-1.5 m) and false positives on negatives at conf 0.5, per held-out bag.
- Known limits (expect a prototype, good for pre-labelling the next bags, not yet for `curtain_inspect_mode:=yolo`): few distinct robot-camera curtain views, few at 1.2-1.5 m, one day's light, no white-wall / window negatives. Horizontal flips are already done online (`fliplr=0.5`); no offline flipped copies (no new information, would inflate val).
- Lightweight: keep yolo11n; compare imgsz 320 vs 416 on `--cv` recall; deploy the OpenVINO export (INT8 optional, check recall); 2 Hz only in looks is enough.

### practice-maze review (2026-11-22, photos in challenge_files/maze_examples): open work
User decisions: walls ~0.45 m, cells ~1.02 m, tallest ramp block ~10 cm. Practice curtain: two halves with a 1-3 cm slit between them; gap under the hem 15-20 cm, but a raised platform (terrain) behind the curtain can block it.
- **DECISION (user 2026-11-22): curtain detection switches to YOLO (CV)** (section above). Why: with a platform behind the curtain the lidar / depth gap check sees no floor behind the hem (no 'curtain' bins), and the lie-down check can't measure the gap either. Until the model is trained and accepted (steps 1-6 above), the lidar gap detection + lie-down check stay the defaults; then switch with `curtain_yolo:=true curtain_inspect_mode:=yolo` and make those the launch defaults.
- Gap 15-20 cm vs the current thresholds: `curtain_max_gap` 0.15 (detection) calls >= 15 cm a hurdle ('high'); the lie-down check confirms 12-20 cm. For the practice curtain run with `curtain_max_gap:=0.22` (or `curtain_lab:=true`, 0.25) until YOLO takes over; check the false-curtain risk on cavity walls first.
- YOLO dataset (steps 1-3 above) must include: both halves hanging closed (1-3 cm slit), parted / swaying, and the curtain in front of a raised platform (the case lidar can't do).
- C1 done but OFF: `curtains.py` `join_runs` / `slit_max` joins the two halves of a split curtain across the slit (test_curtains::test_split_curtain_halves_join_into_one). Default `slit_max = 0` (off): at 0.20 m, side-wall pieces along a corridor joined into a false curtain (test_rolling_pin). The real slit is only 1-3 cm, so `slit_max` ~0.05 may be enough and safer; also only join when the joined segment faces the robot (normal within ~45 deg of the heading). Low priority now that curtains go to YOLO. Also new: bin kind 'low' (seen under, gap 1-5 cm): logged only, never a curtain or candidate (extru side windows read the same).
- C2 (wall band): no change. Measured walls 0.45 m and terrain <= ~0.10-0.18 m sit below the 0.30-0.45 m band; re-check once the heights below are measured.
- NOT DONE (approved, ran out of time):
  - S1 `practice_maze` in make_challenge_mazes.py: 0.45 m walls, 1.02 m cells, ramp unit, slat box, block field, tube on a tray (tray 'tile' + src-e2's 'pin', pin z from the tray top), checker-plate stairs with truss rails, split curtain (hem 15-20 cm, 1-3 cm slit) in front of the raised slat cell, props against walls, open exit side.
  - S2 sim shapes in fake_maze_lite3.py: curtain as two halves + 1-3 cm slit (two 'curtain' solids), OSB panels with hand-hole slots, open truss stair rails ('rail'). The sim has no colour camera, so YOLO curtain detection can't be sim-tested; it needs bags (YOLO steps 1-5).
  - C3 objects against walls (scene_cells.cell_objects): count a blob sticking > ~0.3 m into a cell from the fitted wall line as a POSSIBLE object, and block / room-look it only after scene understanding confirms (src-6d's /scene/result flow).
  - C5 curtain onto terrain: skip / shorten the post-crossing wiggle when /maze/terrain says rough under or just past the curtain (curtain_detector WIGGLING / CREEPING). Note what changed in its doc when done.
- Measurements still needed (tape measure at the practice arena): wall panel top height and any gap under panels; ramp platform / wedge tops; tallest block; tube diameter + tray height; slat rail height and spacing; stairs (steps, rise, tread, width, up-only or up-and-down, handrail height); curtain hem height (15-20 cm, user) and the platform height behind it, width of each half, frame height; OSB hand-hole slot size and height; competition layout vs the practice setup.
- Other flags from the photos: shiny checker-plate stair treads (depth noise: record a bag), truss handrails instead of panel walls at the stairs, curtain opens straight onto the slat cell, survivor posters present (poster detection is off), a second arena ~1.5 m away and glass / daylight (depth IR) around.

## posture
Check whether the robot's posture can be published to ROS. The motion host already sends `robot_basic_state` (1 sitting, 5 sit->stand, 6 standing, 7 stand->sit, 8 lose-control protection, ...) in its 50 Hz `RobotStateUpload` packet (code 0x0901, offset 0; robot_info.md §6.4.3), but `lite3_domain_bridge_4` only turns that packet into /leg_odom*, so ROS never sees it (robot_info.md §3.3). Same packet: `robot_gait_state` (offset 4) and `robot_motion_state` (offset 172: 0 still, 1 stepping).
1. Ask whoever runs the robot's driver container (agent.md §12) whether the bridge can publish basic / gait / motion state (e.g. std_msgs/Int32 or one String JSON topic). We can't read the UDP packet ourselves: it goes to the bridge.
2. If yes: cmd_vel_bridge's lying / standing check (core.posture on base_z - floor) and stair_detector's FloorTracker standing gate (/maze/lie_status) can use it instead of a floor-based height. The gait state would also confirm gait changes, which today can only be seen on the robot or the handheld controller.

## turning
Costmap and path planning behaviour needs to be tuned. need to run with ros-bag and make turning avoid inner wall without inflating costmap too much.

## Navigation

fast-lio delay
Frequently running into issues where map is not updating fast enough. Look into
1. how to change launch stack to accommodate for buffer/loading time
2. make map loading independent of time (possibly leveraging laplace transform)
3. does running robot too long/overheating cause issues? add imu hz check to gauge if robot needs cooldown

robot not reliably sensing walls
check new map. identify why robot not detecting walls properly. is it linked to map delay?

maze grid, exit and launch cell (src-8b, 2026-11-22; agent.md "Maze grid", §9)
Done (unit-tested, 23 tests in test_exit_cell.py + test_challenge_mazes.py): 5 x 3 grid of 1.02 m;
no goals outside it; exit chosen on the final map (never the start cell); park in the launch cell:
creep to its centre, turn to face out, AT_EXIT, /maze/launch_cell; grid refitted to the walls every
5 s (/maze/grid, scene_trigger follows); maze_side left / right; sim depth camera; challenge_*
arenas shaped like the booklet's (made-up interiors).
NOT done / not confirmed:
1. No full sim run has passed yet on the challenge_* arenas. The 22:00-22:45 runs were polluted
   (two orphaned sims on the same domain: duplicate lidar / odom), so their results (wrong maze side,
   Nav2 down) prove nothing. run_sim.sh now refuses a busy domain and cleans up however it ends.
   Clean run 22:58, challenge_a, maze_side left: grid fit fine (score ~1.0, yaw within 2 deg); the run
   got STUCK AT THE CURTAIN on the route (CURTAIN -> AT_CURTAIN "not through after 120 s", waiting for
   /maze/start) and timed out, so the exit / launch cell was never reached. Look at curtain_detector's
   walk-through in the sim (challenge_a's curtain: 13 cm gap, 1 m mount, between (1, 1) and (2, 1)).
   Also logged: 0 cm from a wall at maze (0.24, 1.36).
   FIRST FULL PASS (23:2x): challenge_a_flat (no curtain, no stairs): explored everything, looked
   into the object room without entering, exit chosen on the final map ([4, 2] +x, 73 replans),
   crept 20 cm to the centre after Nav2 stopped short, AT_EXIT 0.03 m off centre, 1 deg off facing
   the exit, never left the maze. challenge_a_no_curtain: stopped AT_STAIRS "NOT LINED UP after 45 s"
   (0.85 m from the first riser, 21 deg off): the stair line-up needs work (and the AI-mode crossing
   can't be simulated). Rerun:
   src/maze_nav/test/run_sim.sh 900 /tmp/sim_a challenge_a.json  (and _a_mirror, _b, _b_offset)
2. maze_side auto (experimental; until fixed, set maze_side:=left / right for your arena). Weak
   spot: while undecided, goals on both sides are allowed, so the robot can leave the maze and
   decide from a cell outside the walls. PLAN: decide from the map instead of the robot's cell.
   The maze side has interior walls in its rows (j = +1, +2 or -1, -2); the other side is open lab
   floor (free, no walls) or unseen. Each replan, near the start-pose grid: count wall pixels
   (>= OCCUPIED_MIN) in the row band on each side, excluding the start row and the outer outline
   lines; decide when one side has >= N wall pixels on >= K replans in a row AND the other has
   < M wall pixels with free floor seen there. Keep the cell rule only as a cross-check (log a
   WARN when they disagree).
   TUNE during the troubleshooting run (troubleshoot:=true, ~/live_debug/current snapshots):
   log both sides' wall / free counts every replan (add them to the snapshot state), then pick
   N, M and K from the real map: margins well clear of what the open side shows (lab walls, the
   start cell's own walls, SLAM speckle). Check both arenas (or the practice arena mirrored).
   The watching agent may ask for a run reset ("RESET REQUESTED", nav_launch.md section 5), e.g. to try
   maze_side:=auto vs the set side, or after a wrong side decision, re-placing the robot square.
   Then sim: challenge_a / challenge_a_mirror with maze_side:=auto must both pass.
3. Robot tests: "grid fitted" in the log (yaw / shift sensible), the chosen exit, AT_EXIT facing out
   (the turn and the creep to the centre are unit-tested only), /maze/launch_cell. Place the robot
   square in the start cell: the grid fit only corrects <= 8 deg / 0.35 m (precision ~0.5 deg).
4. The challenge_* interiors are made up: add the practice arena's layout to make_challenge_mazes.py
   once it is set up.


## scanning
Pose-mode yaw left and right at dead ends / rooms is implemented (the start scan is off by default since D9). NEEDS TESTING ON ROBOT:
1. does the group bridge pass the yaw axis value (`size`) through?
2. does Pose mode work from auto mode (pose_manual:=false), or only via manual?
3. measure body yaw at full axis (log prints what /odom saw) and set pose_max_yaw_deg (currently 20)
4. check it returns to Move mode + gait and Nav2 velocity resumes after the look


## Perception
1. QR code reading always on, but QR detection and survivor detection only at dead ends/rooms. 
2. check feasibility of using yolo for stair detection. should be able to detect at 2-0.5m away (curtains: built, see ## curtains)

## scene understanding (VLM) in room looks
Built and OFF by default (`scene_understanding:=true` to use it); how it works is in `agent.md` §3 (Dead ends).
1. VLM backend in room_scene.process (decide: on-robot model vs Claude API dev-only; needs internet).
2. Survivors: nothing detects them since poster_detector was removed (11-19); scene_type for survivor reports only once the VLM is in.
3. Measure pose_max_pitch_deg (20 = guess). At a 30 deg threshold the compensation is always clamped to it.
4. Robot tests: Pose-mode pitch with yaw 0 at a dead end; camera frame stamps vs local clock (`by arrival` in the log = offset); whether one more colour subscriber saturates the bridge (scene_understanding:=true).
5. Roll on cambered floor: logged only (no roll axis).
6. logs/scene/ is pruned by the daily >7 d cron (only cpu/ and junctions/ are kept).
7. Room look-in (2026-11-21): object cells (toy kitchen etc.) are looked into from an open neighbour (hold + left/right walls, perception on, never entered) + Nav2 KeepoutFilter on the object cells. Sim-tested only (object_maze.json). Depends on block_objects being right: a false object cell gets a pointless look and is lethal for Nav2 (08:32 run: NO_EXIT) - add the "unblock before NO_EXIT" fallback. Sim 10:45 (object_maze): look-in from [1, 1] worked and never entered, but squaring up failed (-77 deg turn blocked both ways: Nav2 stopped the robot 6-16 cm off the cell centre, the cell's own walls block the turn; not the painted room), so the centre view faced 77 deg away (turning in a 1 m cell, see ## turning).

Toy-market sim test (2026-11-22, paused, fix during the actual run): object_maze.json with market-sized boxes (stand, counter, trolley) in dead-end cell grid [2, 1] + a fake colour camera playing the market photos (scene_understanding/test/data/photo_*). Hold -> retry -> blacklist worked, but:
8. AI not connected: room_scene still has vlm 'none', so the market came back scene_type 'unknown', no objects. Needs the organisers' AI server address + request/response format (scene-understanding lab, group id group04_grp4) and the client code from ~/test_scene_understanding (not on this machine). Expected answer for this room: scene_type 'market'.
9. Dead end met during a junction scan gets only the junction's single frame (facing in), no left/right wall views: photo 2 (counter) and photo 3 (trolley, right wall) never captured; QR codes / signs on side walls would be missed. Give such a dead end the full dead-end look (hold + left + right) before leaving the junction.
10. block_objects saw no object in the market cell: the toys stand against the walls and merge into them on /map, so the "object on the map -> room" shortcut on a timeout never fires. Expect the same for real props against walls.
11. Possible false stairs next to the market: stair_detector reported 2 risers of 12 cm twice (camera at 1.76 m during the junction scan, then lidar), lined up 0.37 m from them and handed over to AI mode (transfer_interfaces not found in the sim). Probably the ramp wedges in the cell before the market (maze (-1, 1)), not the toys - check on the real ramp.
12. room_scene saves into the real logs dir (~/challenge_ws/logs/scene/); save_dir isn't a launch argument, so sim test images land there too.

## obstacles
evaluate feasibility of adjusting gait based on obstacle height/terrain type. Some terrains, like slats or barriers, work better with rug mode h-step.
Obstacles up to 15 cm (checked 2026-11-20): nav already treats them as walkable (wall band starts 0.30 m above floor), stair detector needs >= 2 risers so a single 15 cm block/barrier is not stairs. Open: does RUG general gait clear 15 cm? (foot step height unmeasured, robot_info.md open item 7); H-Step made the robot unsteady 2026-11-12. Test on a 15 cm block at max_speed before relying on it.
Rolling pin (~18 cm tall, across the middle of a cell, user 2026-11-22): exploration walks over it with H-Step (default gait; the user says H-Step clears it normally). Checked offline in maze_nav/test/test_rolling_pin.py (bar on the floor and roller on stands): not in wall_scan / SLAM / costmaps, no rising or descending stairs (lidar or camera), no curtain, floor under the feet stays the floor, terrain 'obstacle' ahead -> 0.10 m/s cap. In the sim: challenge_a.json cell (2, 0) (replaces the slats there), fake_maze_lite3 "pin" kind (seen, stepped over: no ride-up, no collision). Not checked yet: the robot stopping or turning with the pin under its belly (goals are cell centres); sim run on challenge_a with the pin (22:30, domain 93): NO_EXIT, UNVERIFIED. The robot never left the start cell (track ends at x 0.045 m). cmd_vel_bridge saw SENSOR STALE over and over, then SENSOR FAULT ("ignoring Nav2 until restart"), while the machine was overloaded (load ~27 on 20 cores; other sims ran on domain 121, which had duplicate simulators). It says nothing about the pin. Rerun (22:53, sim_pin2): timed out with no end state, UNVERIFIED. The pin cell (2, 0) was never reached: the robot went (1,0) -> (1,1) -> (0,1) -> (1,1) -> through the curtain into (2,1), then got stuck at (1.70, 1.0). curtain_detector ended WIGGLING -> STUCK "not through after 120 s" -> AT_CURTAIN, while the curtains.py work (C1/C5) was in flux. Bash ran run_sim.sh while it was being edited (syntax error at line 92), so it never got to its cleanup and the sim and launch were left running (src-82 killed them). Next: once the curtain work is settled, rerun on a quiet machine (`run_sim.sh 900 <log> challenge_a.json <free domain>`) and check the crossing of (2, 0) in track.csv: speed cap 0.10 m/s, no stop or turn on the pin, no stair or curtain hits there. Open (2026-11-22 22:30): test_rolling_pin.py::test_pin_is_not_a_curtain fails for both shapes (it passed earlier) since the curtains.py split-curtain change (C1): a "curtain" along the side wall (y -0.42, x 0.5-1.7) 0.1 m into the walk. Check whether a plain corridor without the pin does the same.

### terrain-adaptive gait (LOW PRIORITY)
Plan (2026-11-21, not started). Only discrete gaits via /simple_cmd (no SDK mode), so the levers are gait choice + speed cap.
0. FIRST: block test 5/10/15 cm, RUG general vs H-Step, 0.10 and 0.20 m/s, log /imu/data + /leg_odom2. If RUG general clears 15 cm, skip gait switching and only slow down.
1. Look-ahead: cheap check on /livox/points_crop (not in stairs_lidar.detect, ~700 ms/pass): 5 cm cells in a 0.4 x 0.4-1.0 m strip ahead, max z minus wall_scan floor -> /maze/terrain (max step, roughness, slope). Check the crop keeps floor points ahead.
2. Body fallback: /imu/data roll/pitch wobble over ~1 s, /leg_odom2 speed / commanded < 0.5 = stumbling.
3. Levels: even (< 5 cm) RUG general, normal speed; rough (5-10 cm) RUG general, ~0.6x speed; obstacle (10-15 cm, slats/barriers, or body fallback) H-Step at 0.10 m/s; > 15 cm left to Nav2 / stair check.
4. Switching in cmd_vel_bridge (owns /simple_cmd), behind terrain_gait:=false: remember the obstacle in odom, switch up ~0.6 m before it, back only once the hind legs are past (obstacle + 0.55 m + 0.2 m) and >= 5 s in H-Step; stop ~0.5 s, Move mode -> gait.
Open: H-Step unsteady 2026-11-12 (cause unknown); does a gait change mid-run drop auto mode?; Grip/Flat gaits untested, left out.
