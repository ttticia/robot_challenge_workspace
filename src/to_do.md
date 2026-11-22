## stairs
reduce stair detection CPU usage. 

stairs were missed in 2nd last run. need to make stair check more robust. 

## curtains
some walls have a cavity that could be read as curtain gap. when checking for curtain look for curtain gap that extends >10cm behind the supposed curtain.

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


## scanning
when scanning after launch and at a dead end/room, go into pose mode, yaw left and right.
Implemented (start scan + dead-end side walls; QR-room look-around still uses Nav2 spins). NEEDS TESTING ON ROBOT:
1. does the group bridge pass the yaw axis value (`size`) through?
2. does Pose mode work from auto mode (pose_manual:=false), or only via manual?
3. measure body yaw at full axis (log prints what /odom saw) and set pose_max_yaw_deg (currently 20)
4. check it returns to Move mode + gait and Nav2 velocity resumes after the look


## Perception
1. QR code reading always on, but QR detection and survivor detection only at dead ends/rooms. 
2. check feasibility of using yolo for stair and curtain detection. should be able to detect at 2-0.5m away

## scene understanding (VLM) in room looks
Built 2026-11-21, OFF by default (`scene_understanding:=false`; dead-end looks + junction follow-up run regardless, explorer never waits):
- explorer: one /scene/capture per dead-end view (facing in, left wall, right wall) at the end of its hold, still posed; tagged with dead-end cell + centre, `last` on the final view. Junction-way views are not captured.
- room_scene: frame stamped inside the hold (fallback: by arrival, robot clock offset), room complete on `last` or 20 s timeout, each cell once, saved to logs/scene/<date>/ + rooms.jsonl.
- task_reporter: QR text (/qr/read is plain text) + the dead-end cell; objects/hazards only from analysed results.
- checked dead-end cells never entered (frontier filter + goal cancel; a confirmed curtain on the edge re-opens it).
- tilt > 30 deg -> Pose-mode pitch on every view of that dead end (TF odom->base_link, zeroed at /maze/start).

STILL TO DO:
1. VLM backend in room_scene.process (decide: on-robot model vs Claude API dev-only; needs internet).
2. Survivors: nothing detects them since poster_detector was removed (11-19); scene_type for survivor reports only once the VLM is in.
3. Measure pose_max_pitch_deg (20 = guess). At a 30 deg threshold the compensation is always clamped to it.
4. Robot tests: Pose-mode pitch with yaw 0 at a dead end; camera frame stamps vs local clock (`by arrival` in the log = offset); whether one more colour subscriber saturates the bridge (scene_understanding:=true).
5. Roll on cambered floor: logged only (no roll axis).
6. logs/scene/ is pruned by the daily >7 d cron (only cpu/ and junctions/ are kept).
7. Room look-in (2026-11-21): object cells (toy kitchen etc.) are looked into from an open neighbour (hold + left/right walls, perception on, never entered) + Nav2 KeepoutFilter on the object cells. Sim-tested only (object_maze.json). Depends on block_objects being right: a false object cell gets a pointless look and is lethal for Nav2 (08:32 run: NO_EXIT) - add the "unblock before NO_EXIT" fallback. Sim 10:45 (object_maze): look-in from [1, 1] worked and never entered, but squaring up failed (-77 deg turn blocked both ways: Nav2 stopped the robot 6-16 cm off the cell centre, the cell's own walls block the turn; not the painted room), so the centre view faced 77 deg away; same risk for junction dead-end visits (turning in a 1 m cell, see ## turning).

## obstacles
evaluate feasibility of adjusting gait based on obstacle height/terrain type. Some terrains, like slats or barriers, work better with rug mode h-step.
Obstacles up to 15 cm (checked 2026-11-20): nav already treats them as walkable (wall band starts 0.30 m above floor), stair detector needs >= 2 risers so a single 15 cm block/barrier is not stairs. Open: does RUG general gait clear 15 cm? (foot step height unmeasured, robot_info.md open item 7); H-Step made the robot unsteady 2026-11-12. Test on a 15 cm block at max_speed before relying on it.

### terrain-adaptive gait (LOW PRIORITY)
Plan (2026-11-21, not started). Only discrete gaits via /simple_cmd (no SDK mode), so the levers are gait choice + speed cap.
0. FIRST: block test 5/10/15 cm, RUG general vs H-Step, 0.10 and 0.20 m/s, log /imu/data + /leg_odom2. If RUG general clears 15 cm, skip gait switching and only slow down.
1. Look-ahead: cheap check on /livox/points_crop (not in stairs_lidar.detect, ~700 ms/pass): 5 cm cells in a 0.4 x 0.4-1.0 m strip ahead, max z minus wall_scan floor -> /maze/terrain (max step, roughness, slope). Check the crop keeps floor points ahead.
2. Body fallback: /imu/data roll/pitch wobble over ~1 s, /leg_odom2 speed / commanded < 0.5 = stumbling.
3. Levels: even (< 5 cm) RUG general, normal speed; rough (5-10 cm) RUG general, ~0.6x speed; obstacle (10-15 cm, slats/barriers, or body fallback) H-Step at 0.10 m/s; > 15 cm left to Nav2 / stair check.
4. Switching in cmd_vel_bridge (owns /simple_cmd), behind terrain_gait:=false: remember the obstacle in odom, switch up ~0.6 m before it, back only once the hind legs are past (obstacle + 0.55 m + 0.2 m) and >= 5 s in H-Step; stop ~0.5 s, Move mode -> gait.
Open: H-Step unsteady 2026-11-12 (cause unknown); does a gait change mid-run drop auto mode?; Grip/Flat gaits untested, left out.
