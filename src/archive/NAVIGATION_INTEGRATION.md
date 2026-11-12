> Updated hardware findings and commands: [HARDWARE_NAVIGATION.md](/home/user/challenge_ws/HARDWARE_NAVIGATION.md). The earlier missing-bridge diagnosis below was superseded by lazy discovery and QoS findings.

# Navigation alongside team perception

The existing navigation package from `~/competition_nav_ws/src/lite3_mission`
has been copied into this workspace. Existing `team_perception` source is unchanged.
The GitHub connector returned 404 for the supplied repository; these changes are
local and have not been pushed. This local Git checkout has no configured remote.

## Current behavior

Selected behavior: navigate to a configured goal independently of detection.
Navigation does not wait for a QR detection. The scanner and QR reporting adapter
are optional and do not choose, cancel, or modify navigation goals.

The QR scanner publishes `/qr/detections`; `team_qr_adapter` forwards observations
to the mission reporter. Nav2 handles navigation goals and obstacle avoidance.
QR observations are recorded; they do not currently stop navigation or change goals.
The default waypoint is 0.5 metres forward relative to the initial robot heading.
It is a trial goal, not a configured competition route.

## Build and preview

In each terminal, source ROS and your normal robot overlays first, then:

```bash
cd ~/challenge_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select team_perception lite3_mission --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=4
```

Keep the working lab localization/SLAM and Nav2 stack running. For navigation
alone, launch `ros2 launch lite3_mission mission.launch.py mode:=waypoints`.
To also record QR detections, start the scanner only if it is not already running:

```bash
ros2 run team_perception qr_scanner
```

In a separate sourced terminal:

```bash
ros2 launch lite3_mission mission.launch.py team_qr:=true qr:=false
```

This mission launch defaults to preview: no navigation goals are sent and its
velocity gate does not publish drive commands. Inspect `/mission/status`,
`/mission/gate_status`, `/mission/candidate_goal` and `/mission/observations`.
Reports are saved under `~/.ros/lite3_mission/`.

## Choose the goal

For an actual destination, create `~/challenge_ws/goal.yaml` using measured map
coordinates. This example is illustrative; replace it with a reachable goal:

```yaml
mission:
  ros__parameters:
    waypoint_frame: map
    waypoints: [1.0, 0.0, 0.0]
```

Each triple is `[x_metres, y_metres, yaw_radians]`. With `waypoint_frame: map`,
coordinates refer to the active map; establish the robot's map pose first.
With `waypoint_frame: initial_base`, the goal is relative to the robot pose when
armed (forward x, left y). The default remains a 0.5 metre forward trial.

Preview your configured destination with:

```bash
ros2 launch lite3_mission mission.launch.py mode:=waypoints config_file:=$HOME/challenge_ws/goal.yaml
```

Add `team_qr:=true` if you want QR reporting alongside navigation. Once the
movement checks below are complete, use the same `config_file` and add
`execution:=true drive:=true`, then arm. The mission disarms after all configured
goals succeed. Disarm before changing the goal or restarting a route.

## Before a supervised movement trial

Verify live `/map`, map-to-base TF, `/imu/data`, active `/bt_navigator`, and the
actual Nav2 velocity chain. The final upstream velocity output must feed
`/mission/nav_cmd_vel`; the mission velocity gate must be the sole publisher on
`/cmd_vel`. Inspect both topics with `ros2 topic info TOPIC -v`.
The installed lab stack includes a twist mux, so verify its output and any
velocity smoother rather than assuming the controller output is final.
Do not launch a second base/SLAM/Nav2 stack alongside an existing one.

Once routing is verified, replace the preview mission launch with:

```bash
ros2 launch lite3_mission mission.launch.py team_qr:=true qr:=false execution:=true drive:=true
```

This still requires manual arming. With the robot in the course-supported auto
mode, the trial path clear, and the handset STOP available:

```bash
ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: true}'
# Disarm and cancel:
ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: false}'
```

Default command limits are 0.10 m/s translation and 0.25 rad/s yaw. Physical
stopping behavior and the real command chain have not been tested in this session.
Route configuration and additional options are in `src/lite3_mission/README.md`
and `src/lite3_mission/config/robot.yaml`.

## Validation in this session

- Colcon build: both packages passed.
- Core, adapter, and existing QR helper tests: 28 passed, 1 expected failure.
- Mission launch argument loading passed.
- ROS contract suite could not start its first ROS test: its launcher requires
  `rmw_cyclonedds_cpp`, which is absent from this ROS installation. This is not a
  passing ROS integration run.
- No robot motion or remote GitHub write was performed.

`src/lite3_mission/TEAM_VALIDATION.md` was copied with the package and describes
an earlier environment; its results are not verification of this workspace.
