# Lite3 competition mission layer

This package extends the completed ROS labs. The robot's installed `lite3_launcher`
continues to own SLAM/localization, TF and Nav2. The factory motion controller stays
in place. No simulator policy, SDK binary, driver restart or firmware change is needed.

Default operation is **shadow mode**: candidate goals and observations only. It
creates no Nav2 action client, drive-lease publisher or publisher on the robot's final `/cmd_vel`.
Execution requires explicit launch options, healthy data and manual arming.

## Integration with this team's QR scanner

This branch adds navigation alongside the existing `src/team_perception` package.
All files present at base commit `d84078d` are preserved. The existing scanner
publishes `std_msgs/msg/String` JSON on `/qr/detections`; it publishes no robot
motion commands. Its decoder, camera subscriptions, CSV logs and snapshots stay
in place. The adapter lives entirely in the added navigation package.

From this repository, after sourcing ROS Humble and the usual lab overlays:

```bash
colcon build --packages-select team_perception lite3_mission --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=4
# Keep the team's existing scanner running, or start one if none is running:
ros2 run team_perception qr_scanner
# Separate terminal with the same overlays and ROS_DOMAIN_ID:
ros2 launch lite3_mission mission.launch.py team_qr:=true qr:=false
```

The mission starts in preview mode. The adapter converts the team's `text`,
`frame`, `position` and floating-point `stamp` fields into the mission observation
schema and preserves snapshot/pixel evidence. It never commands movement or
interprets decoded text as robot instructions. Existing map positions stay in
map; camera-frame points are transformed by the reporter at the original capture
time. Missing depth, timestamp or frame leaves an unlocalized text observation.
The upstream floating-point timestamp has finite precision, which the adapter
cannot recover. Original QR CSV files and snapshots remain in the scanner's own
log directory; mission reports are separate in `~/.ros/lite3_mission/`.

The scanner's aligned-depth setting, RGB/depth timing, calibration and TF still
need verification on the robot. The adapter trusts geometry provided by that
scanner; it cannot establish that measurements are calibrated. Reporting a QR
does not yet trigger a navigation pause or route change.

`team_qr:=true` starts only the adapter, so it can attach to an already running
scanner. Our optional standalone decoder remains disabled. Enabling both
`team_qr` and `qr` is rejected to avoid duplicate decoding/reporting.

See `TEAM_VALIDATION.md` for the tests actually run on this branch. To upload:

```bash
git diff --cached --name-status
git commit -m "Add autonomous navigation alongside team QR perception"
git push -u origin feat/autonomous-navigation
```

Review the feature branch in a pull request before merging into the group branch.
The build and preview above do not enable physical navigation; use the supervised
execution procedure below after checking the actual lab command routing.

## Upload and build

Upload the source archive to **Group 4's `.74` container**, not the shared `.62`
system container. On RI_network the group address is `192.168.10.74`; on the
robot hotspot the documented group address is `192.168.1.74`. Confirm addresses
and exclusive control on the test day. Credentials are not included in this bundle.

```bash
mkdir -p ~/competition_ws/src
tar -xzf ~/lite3_mission-source.tar.gz -C ~/competition_ws/src
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/competition_ws
colcon build --symlink-install --packages-select lite3_mission
source install/setup.bash
export ROS_DOMAIN_ID=4
```

Package dependencies are standard ROS 2 Humble/Nav2, TF2, NumPy, SciPy, OpenCV,
YAML and ZBar/PyZBar. The course stack normally provides most of these. Inspect dependencies
with `rosdep check --from-paths src/lite3_mission --ignore-src`; install missing
dependencies in the group container only with the lab's normal authorization.
No MuJoCo, Gazebo, ONNX Runtime or Ultralytics dependency is required by the mission.

## First launch: preview

Start your working lab SLAM/Nav2 stack normally. Then in a separate terminal:

```bash
source ~/competition_ws/install/setup.bash
export ROS_DOMAIN_ID=4
ros2 launch lite3_mission mission.launch.py
```

Inspect `/mission/candidate_goal` in RViz, `/mission/status`, and
`/mission/gate_status`. The default candidate is **0.5 m relative to the current
initial body heading**. It is not a competition route. `mode:=explore` selects
reachable map frontiers automatically. The cell grid is not used as a route oracle.

## Supervised execution

Coordinate command ownership with both groups sharing the robot. Complete the
course's handset initialization and select the appropriate supported gait. The
course aliases `lite3_auto` / `lite3_manual` remain the mechanism for enabling and
returning control; this package never toggles robot modes automatically.

The final Nav2 velocity output must be `/mission/nav_cmd_vel`, and the mission
gate must be the only final `/cmd_vel` publisher. **Do not start a second lab
navigation stack alongside an existing one.** If starting the lab stack anew,
the optional wrapper scopes a velocity remap without editing course files:

```bash
ros2 launch lite3_mission lab_navigation.launch.py slam:=True
# Or: slam:=False map:=/absolute/path/to/your/map.yaml
```

The course source was not supplied locally, so the wrapper is checked against the
documented launch interface, not its unpublished implementation. A stack with an
extra velocity smoother or custom remaps may require adjusting its **final output**.
Before arming inspect `ros2 topic info /cmd_vel -v` and
`ros2 topic info /mission/nav_cmd_vel -v`. The gate refuses ownership conflicts,
missing input or multiple input producers; it does not silently bypass this check.

Restart the mission nodes with execution enabled:

```bash
ros2 launch lite3_mission mission.launch.py execution:=true drive:=true
# After checking the preview, health, control mode and clear path:
ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: true}'
# Stop this mission and cancel its current goal:
ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: false}'
```

Keep the handset STOP available. A software zero-velocity command is not a physical
emergency stop. Return to manual mode after the trial as taught in the labs.

The gate caps the **vector norm** at 0.10 m/s and yaw at 0.25 rad/s by default,
limits acceleration, and stops immediately on a stale command (0.25 s), expired
mission lease (0.30 s), disarming or ownership conflict. These software bounds do
not establish the robot's physical stopping distance or firmware watchdog behaviour.
Mission health also requires an active Nav2 BT navigator, a fresh map-to-base TF,
valid/fresh body IMU and acceptable tilt. Clock/frame problems must be fixed in the owning lab stack;
stamping all incoming messages with "now" is not a synchronization solution.
If the installed stack has a velocity smoother, check its own input timeout as
well: a stream that repeats an old velocity still looks fresh to a downstream
Twist receiver. The on-robot command chain and physical stopping behaviour must
be checked during the first supervised trial.

## Tuning

Parameters are in `config/robot.yaml`. Change one parameter at a time and retain
the successful configuration. Runtime limit changes are range-checked:

```bash
ros2 param set /velocity_gate max_linear_m_s 0.05
ros2 param set /mission frontier_clearance_m 0.25
```

Disarm before changing mission mode or waypoint lists. Waypoints are flat
`[x,y,yaw, x,y,yaw, ...]` arrays; yaw is in radians. `waypoint_frame` is `map` or
`initial_base`. Mission frontier clearance is an approximate width filter;
**Nav2 remains responsible for full footprint/path collision checks**. Tune the
installed Nav2 footprint, inflation and local controller for the measured robot
and 80 cm passages. Do not treat the simulator foot envelope as a measurement.
Frontier goals retain the current body heading by default (`frontier_yaw_mode:
keep_current`), avoiding an unnecessary demand for an in-place turn. This does
not replace Nav2's footprint checks or make a differential-drive controller
holonomic. Three consecutive failed frontier goals disarm the mission by default.

## Perception and reporting

`/mission/observations` accepts JSON with `kind` (`qr`, `survivor`, `room`, `target`),
`label`, `confidence`, capture `stamp` (`sec`, `nanosec`), `frame_id`, and optionally
`position: [x,y,z]`. Localized observations require actual 3D measurement. They
are transformed at **capture time**, deduplicated by kind/label/proximity, and
written atomically to unique reports in `~/.ros/lite3_mission/`.

The existing Lab 7 `/detections` PointStamped interface is also accepted. Its
default label is `target/lab_detection`; it is not automatically called a survivor.
Configure `point_kind` and `point_label` when connecting a real validated detector.
YOLO person **counts** from Lab 3 do not provide 3D coordinates. A live RGB/depth
localizer or the existing lab detection node must supply them. Camera/depth images
must not be assumed registered just because both are 640x480.

Enable QR decoding with `qr:=true`. With `qr_size_m: 0.0` it records text without
inventing a position. For PnP localization supply calibrated camera info and the
measured **black QR symbol side length**, excluding the white quiet zone. The
camera frame and image dimensions must match calibration.

Optional Lab 6 scene integration uses your **existing** client script:
set `scene_client.ros__parameters.client_path` in an override YAML, then launch
with `scene:=true config_file:=/absolute/path/to/override.yaml`. It reuses the
documented CLI, resizes within 1024x768, permits one in-flight request and handles
timeouts/structured JSON. The client file is not in the handouts and is not bundled.
Service availability and real client compatibility need checking in the lab.
Scene room/survivor classifications are retained as **unlocalized evidence** until
geometry is supplied. VLM text never directly commands the robot.

Cell indices remain null until `cell_grid_enabled` is explicitly enabled with a
measured cell origin and size. Map coordinates alone do not establish competition
cell numbering. Hazard identification is excluded. Stair/curtain pushing actions,
factory gait switching, mannequin-detector training and full-maze scoring are not
implemented by this package.

## Local validation

Pure tests: `python3 -m pytest test/test_core.py`. ROS integration tests use a
separate loopback-only domain and explicitly distinguish a test action server
from real Nav2. See the workspace verification record for the actual checks run.
The real Nav2 integration fixture uses a **planar kinematic base**, not quadruped
dynamics. It validates planner/controller/action wiring and limits, not foot
placement, rough terrain, slip, stair climbing or physical stopping distance.
The archive contains source/configuration, not machine-specific build/install
trees, passwords or model weights. Hardware validation still begins with a brief
health/ownership check; uploading code alone does not validate calibration.
