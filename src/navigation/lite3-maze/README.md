# Lite3 rough-maze autonomy

Run this program before selecting `lite3_auto`. It explores an unknown maze,
remembers visited 1 m cells, backtracks toward remaining reachable frontiers,
and stops in a sustained open clearing. Navigation does not read the simulator's
maze geometry, terrain labels, route, or ground-truth pose. It does not pause
at every cell to look for QR codes or people.

Application roll/pitch limits are disabled (`maximum_tilt_deg: 0.0`). Fresh
sensors, localization and swept-body collision checks remain active. The physical Auto transition requests the documented Move mode. RUG General
is the normal gait request; raised roughness requests RUG High-Step, with a
5-second hold and at least 3 seconds between factory gait changes. This changes
neither the robot firmware nor its built-in protections.

## Run MuJoCo with RViz

From this folder:

```bash
./scripts/build.sh
./scripts/run_sim.sh
```

This opens MuJoCo and RViz together. In another terminal, switch the simulated
robot to Auto:

```bash
./scripts/mode.sh auto
```

Pause with `./scripts/mode.sh manual`; Auto resumes the same exploration. Close
the MuJoCo window or press Ctrl+C in the simulator terminal to stop the session.
The robot starts standing. There is no additional arming service to call.

The simulator initially waits in Manual, matching the requested hardware
workflow. For a flat comparison use `./scripts/run_sim.sh --flat`. Headless
verification uses `--headless --no-rgb --no-rviz`. Default local ROS domain is
**31**, restricted to loopback. Hardware uses the explicit domain you supply.

RViz shows the occupancy map, live 3D cloud/scan, planned path, current goal,
orange body footprint, translucent visited cells, navigation state, camera and YOLO people image.
The map starts empty and fills from sensors. `./scripts/run_rviz.sh` opens a
separate view. Logs are in `logs/current/`. The YOLO people view shows annotated camera images; QR and scene outputs remain
placeholders until their detector topics are connected.

The local simulator can reuse the existing sibling project's Python environment.
For a separate installation run `./scripts/setup_sim.sh`. ROS 2 Humble,
`colcon`, `rmw_cyclonedds_cpp`, RViz, NumPy and SciPy must be installed. Vendor
model/policy files are included in `vendor/lite3`; source and licensing are in
[docs/SOURCES.md](docs/SOURCES.md).

## Run on Lite3

Source the robot's **existing** ROS/transfer/course overlays first, then:

```bash
cd /path/to/ir_challenge_revised
./scripts/setup_vision.sh  # first installation: local CPU inference dependencies
./scripts/build.sh
./scripts/run_robot.sh YOUR_ROS_DOMAIN
```

In the lab control terminal:

```bash
lite3_auto
```

The listener recognizes the lab command on `/simple_cmd` and begins as soon as
live mapping/TF and sensors are ready. `lite3_manual` pauses and sends zero once,
then releases velocity publishing. After the clearing is confirmed, completion
remains latched. In Manual, `/maze/reset` (`std_srvs/srv/Trigger`) clears mission
history for a new run while retaining the map.

The default hardware launcher starts the previously inspected course
`lite3_bringup` and `lite3_navigation` include files with SLAM enabled. It excludes
the top-level course joystick/mux and sends no Nav2 goals; this program produces
`/cmd_vel` directly through the existing transfer bridge. If your working SLAM
stack is already running, use:

```bash
./scripts/run_robot.sh YOUR_ROS_DOMAIN stack:=false
./scripts/run_rviz.sh robot YOUR_ROS_DOMAIN
```

The expected live interfaces are `/map` in `map`, dynamic `map -> base_link` TF,
`/imu/data`, `/livox/points`, `/cmd_vel`, and `/simple_cmd` with
`transfer_interfaces/msg/MotionSimpleCMD`. `scripts/inspect_robot.sh DOMAIN`
prints the actual graph and message definition without moving the robot.
Do not run a second velocity-producing navigation/mux stack alongside this one.
A command publisher conflict is reported in RViz and stops autonomous velocity.
The documented STOP command also disables the mission. The independent velocity
stream publishes at 50 Hz and expires commands after 200 ms if planning stalls.

Hardware parameters are in
[src/lite3_maze/config/robot.yaml](src/lite3_maze/config/robot.yaml), including
speed, frame/topic names and nominal footprint. The initial physical speed is
0.12 m/s. These dimensions are a nominal body envelope; the moving feet must be
measured on the actual robot. The course include layout, live transfer bridge,
rough-ground mapping and factory gait response have **not** been tested on your
physical robot in this session. The lab layout is checked when launched and a
clear error identifies a different installed layout. A custom configuration can
be supplied with `config:=/absolute/path/to/robot.yaml`.

A normalized `std_msgs/String` input `/robot_mode` also accepts `lite3_auto` and
`lite3_manual`, allowing an alternate mode adapter if the lab topic differs.
It is a software adapter; it does not itself change the physical robot mode.
`mode_command_topic:=...` and `mode_command_type:=...` configure the course input.
The bridge sends only the two requested RUG gait codes when Auto is active.

## YOLO people detection and perception outputs

| Output | Type | Initial behavior |
| --- | --- | --- |
| `/maze/qr/detections` | `std_msgs/msg/String` JSON | `source: placeholder`, empty payload |
| `/maze/people/detections` | `std_msgs/msg/String` JSON | CPU YOLO person boxes, confidence and count |
| `/maze/scene/detections` | `std_msgs/msg/String` JSON | `source: placeholder`, empty payload |

YOLOv5n runs at up to 2 Hz in a background worker with a single pending frame.
It accepts raw simulation images or compressed hardware images, preserves camera
capture stamps and publishes person boxes/confidences/counts. `/n_humans` is an
Int32 compatibility output. `/maze/people/debug_image` and its `/compressed`
variant show annotated frames in RViz. `/maze/yolo/objects` contains all COCO
object detections; `/maze/yolo/status` reports camera/inference health.

Each stable detection envelope includes schema version, kind, ROS timestamp,
source, input topic, and `localized: false`. No object position is invented from
a 2D box. The packaged model runs locally on CPU without runtime downloads.
Generic COCO person detection has not been evaluated for challenge posters,
prone figures or mannequins; the simulated maze does not contain realistic people.

Pass `yolo:=false` to retain the empty people placeholder or to use an external
people detector. The default adapter input is `/maze/yolo/people` when YOLO runs.
Once a detector topic is agreed, pass `qr_input:=/qr/detections`,
`people_input:=/your_people_json_topic`, or `scene_input:=/your_scene_json_topic`.
Output topics can be changed with launch arguments `qr_output:=...`,
`people_output:=...`, and `scene_output:=...` (or the corresponding
`detections` node parameters `qr_output_topic`, `people_output_topic`,
`scene_output_topic`). Detector JSON is
preserved inside `payload`. An unconnected input reports `waiting_for_detector`.
These adapters never command navigation or stop the robot for an inspection.

The linked repository has a working QR pipeline. Its `yolo_node.py` was a stub;
this package now supplies the requested CPU YOLO implementation independently.
Run the team's QR scanner alongside the maze program and connect
`qr_input:=/qr/detections`. QR and scene placeholder publishers remain available.

## How exploration and exit detection work

A fine occupancy grid supplies reachable frontiers and A* paths. Unknown cells
remain blocked. Planning clearance includes the whole rotating body. A 0.64 x 0.46 m oriented
rectangle checks the movement horizon;
a brief recentering movement is allowed only when it sheds an existing overlap,
enters no new blocked cells and ends clear. The physical profile consumes the
course SLAM map and TF. The simulator uses IMU heading and local scan ICP,
without loop closure.

The 1 m grid is anchored at the starting robot position/heading for visit memory
and RViz. It is not official cell numbering and does not require a route map.
Place the robot within the maze before selecting Auto. The exit needs an
observed free disk of radius 0.9 m, at least 3 m of travel, at least three visited
cells, enclosed-start evidence and 2 seconds of sustained clearing evidence.
Unknown map space and an exhausted frontier list cannot establish success.
A broad open room inside the maze could resemble the clearing; change
`exit_radius` if the actual course permits interior rooms wider than the cell
corridors. No QR exit marker is assumed.

The downloaded robot guides are reconciled with the implementation in
[docs/ROBOT_INTERFACE.md](docs/ROBOT_INTERFACE.md), including the ROS/raw-axis sign
difference, factory/SDK mode separation, sensor mounts, timing and body dimensions.

## Verification

```bash
source scripts/env.sh local
python3 -m pytest -q tests/test_core.py
ROS_DOMAIN_ID=89 python3 tests/ros_contract.py
python3 scripts/inspect_bag.py /home/vicdeo/ir_challenge/rosbags/teleop_run1_0.db3
# With the headless simulator running:
python3 scripts/evaluate_sim.py --pause-test --require-yolo
```

The supplied bag has 52.225 seconds, 37,273 messages and 522 Livox point clouds.
Its dynamic `/tf` topic contains no messages, and body IMU/leg odometry use uptime
stamps while LiDAR/camera use epoch stamps. All 522 clouds (10,439,808 finite XYZ points) decode successfully with the
actual Livox field layout. The bag also supplies recorded RGB for YOLO inference;
it does not verify autonomous completion. The program does
not invent TF or rewrite capture times to make the recorded sensors look live.

The example rough world has different seeded tile heights in all sixteen cells,
plus an open exit. `scripts/generate_terrain.py --maximum-height 0.065` regenerates
it. It is an example, not measured competition terrain. MuJoCo runs the vendor
ONNX locomotion policy; it does **not** emulate the factory RUG General/High-Step
controllers. Navigation and requested gait transitions are visible, while real
factory gait performance remains a hardware validation item.

New test records and screenshots are under [docs/validation](docs/validation),
with a readable [validation record](docs/VALIDATION.md).
`python3 scripts/package_source.py` creates `dist/lite3-maze-source.tar.gz` with
source, models, licenses and checksums; build/install trees and Python environments
are excluded.
