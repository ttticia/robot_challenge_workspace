# Navigation on the robot — 12 November 2026

The user requested H-Step and maze navigation, then requested **keep stationary**.
No gait, auto-mode or velocity command was sent in this session. A prepared
alternate UDP sender was stopped; use the existing group domain bridge.

## Findings and fixes

The earlier conclusion that the motion bridge was absent was incorrect. The
existing `lite3_domain_bridge_4` creates its command subscriptions lazily after
discovering a publisher. Both `/cmd_vel` and `/simple_cmd` subscriptions request
RELIABLE + TRANSIENT_LOCAL QoS. The old dangling motion-transfer launch link does
not establish whether the shared driver bridge is running.

- Added a reliable, transient-local output option to the mission velocity gate.
  The robot profile enables it. History depth is one and message lifespan is
  bounded by the command timeout (0.25 s by default), limiting replay on reconnect.
- UDP-only Fast DDS transport allowed the real Nav2 stack to reach active state;
  the earlier run stalled during a controller lifecycle service response.
- Added `hardware_navigation.launch.py`, generating a separate conservative Nav2
  profile from the installed lab configuration with the installed BT plugin list.
- Isolated Fast-LIO's sensor `config_file` launch scope from the mission override.
  Without isolation the combined launch passed an empty profile and Fast-LIO crashed.
- The generated profile keeps the 2D obstacle layer and removes a voxel layer
  only when it consumes the identical LaserScan source. This avoids failed
  raytracing when the flattened scan falls below the voxel grid zero height.
  Independent 3D sources and the configured footprint are preserved.
- Made the room wrapper's mission mode and configuration selectable.
- Updated isolated ROS tests to use installed Fast DDS on loopback/domain 89.
  Added coverage for bridge-compatible command QoS and expired command retention.

These changes do not establish physical stopping distance or validate high gait
in narrow passages. The current body/foot swept footprint still needs hardware
verification. Do not reduce clearance merely to bypass `START_NOT_CLEAR`.

## Stationary preview

Stop any earlier lab base/SLAM/Nav2 stack first; this launcher starts those nodes.

```bash
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/challenge_ws
colcon build --packages-select lite3_mission --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=4
ros2 launch lite3_mission hardware_navigation.launch.py
```

Default mode is frontier exploration. Execution and drive output are disabled.
Detection is independent and optional (`team_qr:=true` attaches the QR adapter).
Use `mode:=waypoints config_file:=/absolute/path/goal.yaml` for a known destination.
All map and TF ownership remains with the existing lab components.

## H-Step, when movement is permitted again

The manufacturer's documented high-stepping RUG gait is command `0x21010407`,
with gait-state value 13. This is high stepping, not the high-speed flat gait.
Source: https://developer.deeprobotics.cn/docs/lite3/interfaces/control/state/

The following is prepared for a future supervised trial. It was **not executed**:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/challenge_ws/src/lite3_mission/config/fastdds_udp.xml
# Robot must already be standing in the supported locomotion mode; no stand/sit toggle.
ros2 topic pub --once --qos-depth 1 --qos-durability transient_local --keep-alive 0.5   /simple_cmd transfer_interfaces/msg/MotionSimpleCMD   '{cmd_code: 0x21010407, size: 0, type: 0}'
```

Check H-Step on the handset. No gait-state topic was exposed through the group
bridge, so publishing a request alone cannot confirm the resulting gait.
`lite3_auto` selects automatic mode; it does not supply navigation goals.

Only after the standing pose, clear route, H-Step, current map/TF, active Nav2,
command routing and handset STOP have been checked, restart the same launcher
with `execution:=true drive:=true`, then arm via `/mission/set_armed`. These
steps remain deferred under the user's stationary instruction.

## Maze exit

Frontier exploration selects new mapped approaches; it does not recognize a
maze exit. An opening into a large clear area is a candidate geometric cue, but
can also describe an internal room. Exit identification and automatic stopping
at that opening are not implemented or hardware-validated yet.

## Evidence

- Updated software tests: `hardware_testing/tests_updated.log` (37 passed, 1 expected failure).
- Nav2 activation with real sensors: `hardware_testing/nav2_udp.log`.
- Combined launcher: `hardware_testing/hardware_combined.log`.
- Latest stationary snapshot: `hardware_testing/stationary_acceptance.json`.
- Earlier camera/map snapshots: `hardware_testing/camera.jpg`, `map_udp.png`.
- Sender packet tests: `~/hardware_support/motion_ws/test_sender.py` (loopback only).

The alternate sender was derived from manufacturer source at commit
`e4ab5dc1c7f3ff46274162a1ef302253936578aa` of
https://github.com/DeepRoboticsLab/Lite3_ROS/tree/ros2-foxy
and is retained for review, not used by the hardware launcher.
