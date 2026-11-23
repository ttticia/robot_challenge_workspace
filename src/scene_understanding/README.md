# Scene Understanding

The `scene_understanding` ROS 2 package receives camera images, detects junction
trigger events, and asks a vision-language model (VLM) to classify the scene.
Its primary runtime node, `junction_trigger`, publishes a compact JSON scene
observation that can be used by the rest of the competition stack.

The package supports:

- Two VLM backends: the lab HTTP perception server or Anthropic Claude
- Rising-edge or repeated-`true` junction triggering
- Live camera input or offline image-folder playback
- Optional three-view junction captures: the whole robot turns 20 deg left
  and right slowly on `twist_mux` (default), with Nav2's `Spin` action, or
  only the body yaws in Lite3 Pose mode (the feet stay put; at most about
  17 deg)
- `room_scene`: scene understanding for challenge_ws's maze run (its
  dead-end room scans; same topics as challenge_ws's own `room_scene`, VLM
  wired). See "Running with challenge_ws navigation"
- Frame-rate monitoring and frame-age validation
- Atomic-by-line JSONL result logging and camera-frame saving
- Robot position lookup through TF
- Unit tests for trigger logic, prompting, image transport, and VLM clients

## Package layout

| File | Purpose |
| --- | --- |
| `image_subscriber.py` | Converts `sensor_msgs/msg/Image` frames to OpenCV images |
| `image_folder_publisher.py` | Publishes saved images for offline testing |
| `junction_trigger.py` | Coordinates triggering, VLM calls, saving, TF lookup, and output |
| `timed_turn.py` | Left/center/right junction views by turning the robot slowly on `twist_mux` (default) |
| `junction_scan.py` | Left/center/right junction views by turning the robot with Nav2 `Spin` |
| `pose_scan.py` | Left/center/right junction views by yawing the body in Lite3 Pose mode |
| `room_scene.py` | VLM for challenge_ws's dead-end room scans (`/scene/capture` -> `/scene/result`) |
| `vlm_client.py` | HTTP client for the lab perception server |
| `vlm_api_client.py` | Anthropic Claude API client |
| `scene_prompt.py` | Scene prompt, mission match parsing, and task report validation |

## Build and source

Requirements include ROS 2 Humble, Python 3, OpenCV, `cv_bridge`, NumPy,
requests, TF2, and the ROS `nav2_msgs` and `action_msgs` packages. The Pose-mode
scan also needs the robot's `transfer_interfaces` message package (from
`~/ros2_ws`); this workspace finds it when it was built with `~/ros2_ws`
sourced. Check with
`python3 -c "from transfer_interfaces.msg import MotionSimpleCMD"`.

From the workspace root:

```bash
cd ~/test_scene_understanding
source /opt/ros/humble/setup.bash
colcon build --packages-select scene_understanding --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=<your-domain>
```

If using the Claude backend, install the Anthropic SDK and set an API key:

```bash
pip install anthropic
export ANTHROPIC_API_KEY='your-key'
```

Never commit API keys or credentials to the workspace.

## ROS interfaces

### `junction_trigger`

Subscriptions:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Latest camera frame |
| `/junction_detected` | `std_msgs/msg/Bool` | Junction detector trigger |
| `/camera/image_source` | `std_msgs/msg/String` | Optional folder-source metadata |

The source-topic message is JSON with `stamp`, `file`, and `path`. This lets
the trigger associate a camera frame with its original file.

Publisher:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/scene_understanding/result` | `std_msgs/msg/String` | Parsed scene JSON with `pos`, `robot_position` and `reports`, or JSON `null` |
| `/nav_cmd_vel` | `geometry_msgs/msg/Twist` | Turn commands to `twist_mux` (turn scan only) |
| `/simple_cmd` | `transfer_interfaces/msg/MotionSimpleCMD` | Robot mode and Pose-mode yaw commands (Pose scan only) |

The Nav2 scan (`junction_scan_method:=spin`) uses the action client
`nav2_msgs/action/Spin` at `/spin` by default.

## Basic execution

Start the live scene-understanding node:

```bash
ros2 run scene_understanding junction_trigger
```

With the lab server:

```bash
ros2 run scene_understanding junction_trigger \
  --ros-args \
  -p enable_vlm:=true \
  -p vlm_server:=http://192.168.10.141:8000 \
  -p vlm_group:=group04_grp4
```

Set the group ID to the registered value for the team. It must match:

```text
group##_XXXX
```

For a live camera topic and best-effort camera QoS:

```bash
ros2 run scene_understanding junction_trigger \
  --ros-args \
  -p image_topic:=/camera/image_raw \
  -p reliable:=false
```

Monitor the output:

```bash
ros2 topic echo /scene_understanding/result
```

## Offline image-folder testing

Start the folder publisher:

```bash
ros2 run scene_understanding image_folder_publisher \
  --ros-args \
  -p image_dir:=/absolute/path/to/images \
  -p rate:=1.0 \
  -p loop:=true \
  -p max_size:=1280 \
  -p publish_junction:=true
```

In another terminal, start scene understanding:

```bash
ros2 run scene_understanding junction_trigger \
  --ros-args \
  -p vlm_server:=http://192.168.10.141:8000 \
  -p vlm_group:=group04_grp4
```

The folder publisher creates `/junction_detected=true` for files whose names
start with `junction01` through `junction05`, and `false` for other files.
Override the prefixes and trigger behavior with ROS parameters:

```bash
ros2 run scene_understanding image_folder_publisher \
  --ros-args \
  -p image_dir:=/absolute/path/to/images \
  -p junction_prefixes:="['my_junction']" \
  -p separate_junctions:=true
```

To send a trigger manually:

```bash
ros2 topic pub --once /junction_detected std_msgs/msg/Bool "{data: true}"
```

## VLM command-line tests

Lab server:

```bash
ros2 run scene_understanding vlm_test \
  --server http://192.168.10.141:8000 \
  --group group04_grp4 \
  --prompt "Describe the image" \
  --count 1
```

Claude API:

```bash
ros2 run scene_understanding vlm_api_test \
  --prompt "Describe the image" \
  --model claude-opus-5-5 \
  --effort medium \
  --count 1
```

Both commands wait for a new frame on `/camera/image_raw`, fit it within
1024 x 768 by default, and optionally save the sent frame and JSONL result.
Use `--source-topic /camera/image_source` to associate frames with folder
publisher filenames.

## Junction three-view scan

Enable the scan to add a left and a right view to each junction:

```bash
ros2 run scene_understanding junction_trigger \
  --ros-args \
  -p enable_vlm:=true \
  -p reliable:=false \
  -p image_topic:=/camera/color/image_raw \
  -p junction_scan_enabled:=true
```

The scan never runs while challenge_ws's `maze_explorer` is running
(`/maze/status` has a publisher; `junction_scan_block_topic`): the record
then gets a `scan_warning` and only the center frame.

The startup log says `Junction three-view scan: turn`. The views are stitched
left, center, right in that fixed order and sent with the three-panel prompt.
The trigger frame is the center view.

### Slow turn (default, `junction_scan_method:=turn`)

The whole robot turns in place (needs `lite3_launcher` running, for
`twist_mux` and TF):

1. Turn 20 deg left (`junction_scan_angle_deg`), take a fresh frame.
2. Turn back to the trigger heading.
3. Turn 20 deg right, take a fresh frame.
4. Turn back to the trigger heading.

Each turn is sent at `junction_turn_rate` (0.4 rad/s) for angle / rate
seconds on `/nav_cmd_vel` (`twist_mux`'s navigation input, so the joystick
overrides it), then zero. It then holds zero until the TF heading is steady
(at least `junction_turn_settle_sec`, 1.5 s: Fast-LIO odometry arrives about
1.2 s late), measures, and corrects any error over 3 deg with up to two short
turns. A turn more than 25 deg beyond the scan angle is refused (a TF jump,
or the robot already far off) and the scan stops with an error.
`view_yaw_measured_deg` in the record is the settled angle of each side view.
A scan takes about 10-15 s.

There is no collision check: stand the robot near the middle of the cell,
with its corners (about 0.36 m from its centre) clear of walls. The robot
must be in auto (navigation) mode.

### Nav2 Spin (`junction_scan_method:=spin`)

The same four turns with Nav2's `Spin` action. Spin steers by the late
odometry and overshoots: asked for 20 deg, the robot reached about 73 deg
(2026-11-23 04:52). Nav2 also stops a turn with `Collision Ahead` when its
local costmap predicts the footprint touching anything. Not recommended on
this robot.

### Pose mode (`junction_scan_method:=pose`)

Use this when the robot must not turn: it covers less (the body yaws at most
about 17 deg, the hardware limit at full axis).

The feet stay planted and only the body (with the camera) yaws, so the robot
is not turned round and Nav2 is not involved. The node sends the robot
commands itself on `/simple_cmd`, with the same codes and order as
challenge_ws's `cmd_vel_bridge`:

1. Manual mode, then Pose mode, 0.5 s apart.
2. Body yaw left (`junction_pose_yaw_deg`, 16 deg by default = full axis),
   repeated at 10 Hz. After `junction_pose_settle_sec` (2 s) a fresh frame is
   taken. Then the same to the right.
3. Body back to center, then Move mode, then navigation (auto) mode.

Step 3 always runs, also when the camera stops or on Ctrl+C during a scan.
The gait is not re-sent after Move mode.

Measured on the robot (2026-11-23): 8 deg asked gave 8.2 deg; 16 deg (full
axis) gave about 17 deg. Pose mode cannot yaw the body further than that.
challenge_ws limits the axis to 80 % (`junction_pose_axis_limit:=0.8`, about
13 deg) because the full axis made the motors buzz with the body against a
wall. Keep the body clear of walls, or use that limit.

`view_yaw_measured_deg` in the record is the furthest body yaw TF showed
towards each side during the look. Fast-LIO odometry lags the body by about
1 s, so it is approximate; it is `null` without TF (no `lite3_launcher`).

The node refuses to send anything, and uses the center frame with a
`scan_warning`, when:

- nothing subscribes to `/simple_cmd` (robot bridge not on this domain),
- another node publishes `/simple_cmd` (for example challenge_ws's
  `cmd_vel_bridge` during a maze run),
- `transfer_interfaces` is not found.

Before testing: the robot must be standing and not in AI mode, keep the
handheld controller ready, and note that it ends in auto mode (it then follows
`/cmd_vel` from `lite3_launcher`'s `twist_mux`).

Trigger by hand (send `false` before each `true`):

```bash
ros2 topic pub --once /junction_detected std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /junction_detected std_msgs/msg/Bool "{data: true}"
```

## Parameters

### Image and trigger parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `image_topic` | `/camera/image_raw` | Camera image topic |
| `reliable` | `true` | Use reliable QoS; set `false` for a best-effort camera |
| `junction_topic` | `/junction_detected` | Junction trigger topic |
| `trigger_mode` | `rising_edge` | `rising_edge` or `every_true` |
| `cooldown_sec` | `2.0` | Minimum interval between accepted triggers |
| `max_frame_age_sec` | `2.0` | Maximum age of the latest camera frame |
| `source_topic` | `/camera/image_source` | Optional source-file metadata topic |
| `result_topic` | `/scene_understanding/result` | Scene result topic |

### VLM parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `enable_vlm` | `true` | Enable processing and HTTP VLM requests |
| `vlm_server` | `http://192.168.10.141:8000` | Lab perception server base URL |
| `vlm_group` | `group04_grp4` | Registered team group identifier |
| `vlm_read_timeout` | `75.0` | VLM response timeout in seconds |
| `vlm_connect_timeout` | `5.0` | Server connection timeout in seconds |
| `vlm_max_width` | `1024` | Maximum sent frame width; `0` means unlimited |
| `vlm_max_height` | `768` | Maximum sent frame height; `0` means unlimited |

The package's Claude CLI uses `--model`, `--effort`, `--timeout`, `--prompt`,
`--count`, `--frame-timeout`, `--save-dir`, `--source-topic`, `--max-width`,
and `--max-height`. The `effort` choices are `low`, `medium`, `high`,
`xhigh`, and `max`.

### Scan and TF parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `junction_scan_enabled` | `false` | Enable the three-view scan |
| `junction_scan_method` | `turn` | `turn` (slow turns on `twist_mux`), `spin` (Nav2 Spin) or `pose` (body yaw, Pose mode) |
| `junction_turn_topic` | `/nav_cmd_vel` | Turn: velocity topic (`twist_mux` navigation input) |
| `junction_turn_rate` | `0.4` | Turn: rad/s while turning (0.1 to 1.0) |
| `junction_turn_settle_sec` | `1.5` | Turn: seconds at zero before the heading is trusted |
| `junction_scan_block_topic` | `/maze/status` | No scan while this topic has a publisher (maze run); `''` = off |
| `junction_pose_yaw_deg` | `16.0` | Pose: body yaw for the side views; limited by max yaw x axis limit |
| `junction_pose_max_yaw_deg` | `16.0` | Pose: body yaw at full axis (measured about 16-17 deg) |
| `junction_pose_axis_limit` | `1.0` | Pose: fraction of the full axis never exceeded (`0.8` = challenge_ws limit) |
| `junction_pose_settle_sec` | `2.0` | Pose: seconds after each yaw command before the frame |
| `junction_pose_manual` | `true` | Pose: manual mode before, auto mode after the look |
| `junction_scan_angle_deg` | `20.0` | Turn / Spin: turn angle from the center heading |
| `junction_spin_action` | `/spin` | Spin: Nav2 `Spin` action name |
| `map_frame` | `map` | Preferred robot position frame |
| `odom_frame` | `odom` | Fallback robot position frame |
| `base_frame` | `base_link` | Robot base frame |
| `report_cell` | `ahead` | Reported `pos`: `ahead` = the cell the robot faces (a room looked into from the cell before it), `robot` = the cell it stands in |
| `cell_size` | `1.02` | Arena cell pitch (m) for snapping `pos` to cell centres |

### Output parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `save_dir` | `~/test_scene_understanding/src/scene_understanding/junction_test_results` | Root for timestamped junction runs |

To disable saved images and `results.jsonl`, pass an empty string, quoted:
`-p "save_dir:=''"` (a bare `save_dir:=` makes ROS fail to start).
The node always creates a timestamped run folder while saving is enabled.

## Scene JSON

The prompt requests a JSON object containing:

- `scene_type`: a room type from the competition vocabulary (`clothes_shop`,
  `fitness_centre`, `kitchen`, `market`, `workshop`, `clinic`,
  `cleaning_room`, `furniture_shop`), or `corridor`, `open_area`, `junction`,
  `unknown` (not reportable)
- `junction_confirmed`: `true`, `false`, or `null`
- `paths`: observations for `forward`, `left`, and `right`, each with
  `status` and `notes`
- `mission_matches`: supported objects (`medical_equipment`, `hand_tools`,
  `food`, `water`, `utensils`), hazards (`high_voltage`, `radioactive`,
  `chemical`, `biological`, `flammable`), survivors, or decoded QR codes
- Other warning signs (exclamation mark, magnet, laser, ...) are not
  reportable hazards; they appear only in `sightings` with `kind="sign"`
- Optional inconclusive `sightings`

Path status values are `open`, `partially_blocked`, `blocked`, or `unknown`.
Metal stairs, pass-through curtains, and route obstructions are assessed only
from the visible evidence; the model must not infer movement commands or
distance beyond the frame.

Every published scene also carries its position:

- `pos`: the reported cell centre `[x, y]` (see `report_cell`), from the
  robot pose at the trigger, snapped to the 1.02 m grid. The map frame is the
  arena frame when SLAM starts with the robot on the start cell centre,
  facing the initial forward direction.
- `robot_position`: the raw TF pose (`frame_id`, `x`, `y`, `yaw_deg`).
- each entry of `mission_matches` gets the same `pos`.
- `reports`: one entry per mission match, ready for `build_task_report`:
  `task_type`, candidate `task_ids`, `scene_type`, `label`, `pos`,
  `message`, `evidence`, `reportable` and `problems` (what the scoring
  server would reject: `label`, `scene_type`, `pos`, `message`).

`pos` is `null` (and listed in `problems`) when TF has no `map` or `odom`
pose, for example without `lite3_launcher`.

The trigger publishes `null` when no parsed scene is available. A successful
`/scene_understanding/result` message is JSON text, not a native ROS JSON type.

## Task reports

`scene_prompt.py` builds and checks the scoring-server payload with the
competition's fixed vocabulary. Nothing is sent automatically.

| task_id | task_type | label | scene_type | pos | message |
| --- | --- | --- | --- | --- | --- |
| M01 | object | an object label | room type | `[x, y]` | `""` |
| M02 | hazard | a hazard label | room type | `[x, y]` | `""` |
| M03, M05 | survivor | `""` | room type | `[x, y]` | `""` |
| M04 | qr | `""` | room type | `[x, y]` (or `null`) | decoded QR text |

The M01 object and M02 hazard are announced by the organizers, so any label
of the right type is accepted. `pos` is the centre of the cell, in metres:
origin at the start cell centre, +x the robot's initial forward direction,
+y its left; cell centres are 1.02 m apart, and a position within 0.5 m of
the true centre on both axes counts. Only the latest valid report per
task_id is scored, so a corrected report can be sent again.

```python
from scene_understanding.scene_prompt import build_task_report, send_task_report

report = build_task_report('group04_XXXX', 'M01', 'clinic', 'medical_equipment', [3.06, 2.04])
send_task_report(report)   # POST http://192.168.10.141:8001/task_report
```

`build_task_report` raises `ValueError` with the reason the server would
reject a report (for example `Invalid scene_type`); `send_task_report`
refuses invalid reports and returns the server's JSON reply.

## Saved output

Each run creates a folder similar to:

```text
junction_test_results/junctions_YYYYMMDD_HHMMSS/
├── junction_001.jpg
├── junction_002.jpg
└── results.jsonl
```

Each JSONL record includes:

- Junction index and camera timestamp
- Source filename, if available
- Captured or stitched image path
- View layout and yaw offsets
- VLM response and scene observation
- Latency and processing status
- Scan warnings or errors
- Robot position at the trigger (with yaw) and TF lookup status
- `pos` (reported cell centre), `robot_cell_centre`, `report_cell`, and
  `reports` (one per mission match, each with `pos`)

Robot position lookup prefers `map -> base_link`, then falls back to
`odom -> base_link`. A missing transform is recorded instead of inventing a
position.

## Trigger behavior

The default `rising_edge` mode triggers once on the `False -> True` transition
and also accepts the first `True` message. Holding `True` does not repeatedly
trigger.

`every_true` accepts every `True` message subject to `cooldown_sec`.
The cooldown applies after a trigger, and no trigger is accepted at a frame age
above `max_frame_age_sec`.

VLM processing runs in a background thread with one-at-a-time locking. If a
new junction arrives while analysis is busy, that junction is skipped.

## Running with challenge_ws navigation

Scene understanding runs beside challenge_ws's maze run (`maze_nav`) without
changing anything in challenge_ws: challenge_ws drives the robot and does the
room scans, this package's `room_scene` analyses them. challenge_ws is still
changing every day, so this section only relies on its topic interface and
launch arguments; when they disagree with this text, its `agent.md`,
`src/maze_nav/README.md` and `src/maze_nav/nav_launch.md` are current.

### How the two fit together

1. challenge_ws's `scene_trigger` notices a dead end in the cell ahead
   (`/scene/trigger`) and its `maze_explorer` stops in the cell before it
   (`DEAD_END`). Junction scans that face a dead end, and room look-ins at
   object cells, do the same.
2. The explorer holds facing in (`centre`), then looks at the `left` and
   `right` walls (whole-robot turns or Pose-mode body yaw, depending on
   challenge_ws's current settings), and publishes one `/scene/capture` per
   view, but only with `scene_understanding:=true`.
3. This `room_scene` takes the camera frame from each hold, stitches the
   views left, centre, right, sends them to the lab VLM in one request and
   publishes `/scene/result` with `scene_type`, `objects`, `hazards`,
   `signs`, `humans`, `qr`, `pos` (the scanned cell's centre) and `reports`
   (one per mission match, each with `pos`).
4. The explorer reads the verdict: a room (or any object) -> the dead end is
   blacklisted, stairs / curtain -> candidate queue, otherwise left open. No
   answer within its `scene_result_timeout` (15 s) -> map re-check, then
   `/scene/retry` (this node resends the stored views), at most 2 tries.
5. challenge_ws's `task_reporter` turns results into
   `/mission/task_completed`. Nothing sends reports to the scoring server
   automatically; `reports` in each result are ready for `build_task_report`
   / `send_task_report` (see Task reports).

`scene_understanding:=true` also starts challenge_ws's own `room_scene`
(no VLM: results say `"vlm": "none"`, which the explorer and
`task_reporter` ignore). Both run; this one gives the real verdicts.

| Topic | Type | Direction |
| --- | --- | --- |
| `/scene/capture` | `std_msgs/msg/String` (JSON) | in, from `maze_explorer` |
| `/scene/retry` | `std_msgs/msg/String` (JSON) | in, from `maze_explorer` |
| `/camera/color_relay/image_raw/compressed` (or `/camera/color/image_raw/compressed`) | `sensor_msgs/msg/CompressedImage` | in |
| `/scene/result` | `std_msgs/msg/String` (JSON) | out, to `maze_explorer` and `task_reporter` |

### Commands (real robot, domain 4)

Two packages are called `scene_understanding` (challenge_ws's and this
one). **Source only one workspace per terminal**, or the launch may start
the wrong `room_scene`.

Before: robot standing square on the start cell (the cell grid and the
arena frame for `pos` start there), someone holding the handheld
controller, nothing else running SLAM / Nav2 / `lite3_launcher` /
`junction_trigger` on domain 4 (`ros2 node list`). Check that no one else
(another person or Claude session) is running on domain 4.

```bash
# terminal 1: challenge_ws navigation (source ONLY challenge_ws)
cd ~/challenge_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=4
ros2 launch maze_nav maze_nav.launch.py scene_understanding:=true
# wait for "gait sent" in the log, then: lite3_auto

# terminal 2: this room_scene (source ONLY this workspace)
cd ~/test_scene_understanding
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=4
ros2 run scene_understanding room_scene --ros-args -r __node:=room_scene_vlm \
  -p camera_topic:=/camera/color_relay/image_raw/compressed \
  -p vlm_server:=http://192.168.10.141:8000 -p vlm_group:=group04_grp4

# terminal 3: watch
ros2 topic echo /maze/status
ros2 topic echo /scene/result

# when /maze/status reads PAUSED: start recording, then go
rec_maze -- /scene/capture /scene/result /scene/retry /scene/trigger
ros2 topic pub --once /maze/start std_msgs/msg/Empty
```

- `-r __node:=room_scene_vlm`: challenge_ws's `room_scene` already uses the
  name `room_scene`.
- `camera_topic`: with `perception:=true` (challenge_ws's default) its
  `colour_relay` already brings the camera's JPEG stream over once; use that
  copy. With `perception:=false`, leave the default
  (`/camera/color/image_raw/compressed`, one more copy over the robot bridge).
- `rec_maze` does not record the `/scene/*` topics by itself; add them after
  `--`.

After the run: Ctrl+C both, then `lite3_manual` (challenge_ws `skills.md`
§6). Results: `room_scene_results/<date>/` (one jpg per view, the stitched
`_room.jpg`, `rooms.jsonl` with the raw VLM reply).

### Testing the VLM step without driving

With the launch running (terminal 1, `go:=false` if the robot must not
move) and this `room_scene` (terminal 2), point the robot at a room by hand,
then send one capture of the cell ahead, with no motion:

```bash
# terminal 4: source challenge_ws (trigger_scene is a maze_nav tool)
ros2 run maze_nav trigger_scene capture                  # the cell the robot faces
ros2 run maze_nav trigger_scene capture --cell 3 1 --hold 2.0
```

Each cell is analysed once; use another `--cell` to repeat. `ros2 run
maze_nav trigger_scene` (without `capture`) runs the full dead-end look
(the robot moves; only while the explorer is `EXPLORING`). See challenge_ws
`src/maze_nav/nav_launch.md`, "Manually triggering a dead-end scan or scene
understanding".

In the simulator (never domain 4): challenge_ws `skills.md` §3-4, with
`scene_understanding:=true`, and this node started with
`-p use_sim_time:=true` on the same domain. The simulator has no colour
camera unless one is played in (challenge_ws `to_do.md`, toy-market test).

### Things to know

- **Latency vs. the explorer's timeout.** The lab VLM takes about 6-16 s;
  the explorer waits 15 s, then retries once. A slow answer can still arrive
  and be used, but expect some `attempt: 2` results.
- **Room types.** The explorer counts `kitchen`, `market`, `workshop`,
  `clinic` and `cleaning_room` (or any object) as a room. A dead end seen as
  `clothes_shop`, `fitness_centre` or `furniture_shop` with no object is left
  open, not blacklisted, until challenge_ws adds those names.
- **`pos` comes from challenge_ws's cell grid** (`cell_centre` in each
  capture), fitted to the mapped walls; it is the arena frame only when the
  robot started square on the start cell centre.
- **No depth mask here.** challenge_ws's own `room_scene` greys out pixels
  beyond the room (`<name>_masked.jpg`); this one sends the unmasked frames,
  so toys in neighbouring rooms or the hall can show in the image.
- **Do not run `junction_trigger` during a maze run.** challenge_ws's
  `scene_trigger` pulses `/junction_detected` at every dead end. Its scan is
  blocked while `/maze/status` has a publisher (`junction_scan_block_topic`;
  center frame only), but it would still send a second VLM request for every
  dead end.

## Testing

Run the package tests from the workspace:

```bash
cd ~/test_scene_understanding
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/scene_understanding
python3 -m pytest -q test
```

Targeted tests:

```bash
python3 -m pytest -q test/test_junction_trigger.py
python3 -m pytest -q test/test_scene_prompt.py
python3 -m pytest -q test/test_vlm_client.py
python3 -m pytest -q test/test_junction_scan.py
python3 -m pytest -q test/test_timed_turn.py
python3 -m pytest -q test/test_pose_scan.py
python3 -m pytest -q test/test_room_scene.py
```

Tests cover trigger edges and cooldown, prompt and mission-result validation,
HTTP request formatting and image resizing, and junction scan action behavior.
Tests that require ROS or Nav2 should use the package's ROS environment and a
separate test domain.

## Troubleshooting

### No camera frames

Check the publisher and QoS:

```bash
ros2 topic info -v /camera/image_raw
ros2 topic hz /camera/image_raw
```

Set `reliable:=false` if the camera driver publishes best-effort frames.
Confirm `image_topic` matches the real camera.

### Junction never triggers

Check `ros2 topic echo /junction_detected`, the configured
`trigger_mode`, and `cooldown_sec`. In `rising_edge` mode, send a `False`
message before a new `True` message.

### VLM requests fail

Check network reachability, server address, group ID, and server logs. The lab
server default is `http://192.168.10.141:8000`. HTTP 429 responses include a
retry delay. The node may throttle later requests when the server asks it to.

For Claude, verify `ANTHROPIC_API_KEY`, SDK installation, model availability,
and network/proxy access.

### Old frame warning

Increase `max_frame_age_sec` only after confirming camera freshness, or repair
the camera timing and QoS. A held frame should not be used as a live junction
view.

### Pose scan does not move the robot

Read `scan_warning` in `results.jsonl`: it says why (no `/simple_cmd`
subscriber, another `/simple_cmd` publisher, `transfer_interfaces` missing).
No `scan_warning` and no `view_layout` means `junction_scan_enabled` was not
set. If the robot stays in Pose mode (for example after the node was killed),
switch it back with the handheld controller.

### Scan action rejected or unavailable

Confirm `/spin` exists and Nav2 is healthy:

```bash
ros2 action list
ros2 action info /spin
```

Start with `junction_scan_enabled:=false` for a center-only test. Verify
turning behavior and recovery on an unobstructed, supervised test route.
