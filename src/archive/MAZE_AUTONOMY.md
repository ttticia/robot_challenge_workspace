# Autonomous maze navigation — operator-start workflow

## Workspace contents

- `src/team_perception`: existing QR scanner and CPU YOLOv5n people detection.
- `src/lite3_mission`: online SLAM/Nav2 integration, frontier exploration and open-area exit detection.
- `models/yolov5n.onnx`: people detector weights.
- `runs/`: small session logs, QR snapshots/CSV and a saved map.
- `recordings/`, `ROS_bags/`: older bag files. The current launch does not record a rosbag.

## Start

The robot should be standing. Launch the stack once, using the normal lab overlays:

```bash
source /opt/ros/humble/setup.bash
source ~/lite3_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/challenge_ws/install/setup.bash
export ROS_DOMAIN_ID=4
ros2 launch lite3_mission maze.launch.py record:=false
```

It starts SLAM, Nav2, QR and people detection, requests H-Step, then waits. In your other terminal:

```bash
lite3_auto
```

H-Step (`0x21010407`) is requested **before** the session waits for your command.
After receiving **your** auto-mode command, it arms online exploration. Auto mode
is the last mode command before driving; gait is not changed after entering auto. The coordinator never sends `lite3_auto` itself. It does not switch
to a lower gait during the run. No gait-state feedback is exposed through the
current bridge, so H-Step is a command request rather than a measured confirmation.

No saved map or preset route is required. The robot chooses frontiers from the
map as it grows. Navigation faults pause driving while SLAM and perception stay
running. `lite3_manual` or software STOP disarms the mission and prevents automatic
rearming. Run `lite3_auto` again to request H-Step and resume explicitly.

To stop the launch, press Ctrl+C in its terminal. At shutdown the mission is
disarmed, manual mode is requested if the operator started a run, and a standalone
map is saved. Existing recordings are not modified.

## Exit and terrain behavior

The geometric exit cue requires an observed enclosed region (free radius <=0.8m),
2m displacement from the starting point, then >=1.2m of observed free radius around
the robot for three seconds. Unknown map cells and map edges do not count as open
space. Thresholds are in `src/lite3_mission/config/maze.yaml`. A sufficiently large
internal room can look like the stated open-area exit to this geometric detector.

Factory H-Step handles foot motion. Stair and curtain traversal is not validated
end to end by this software; Nav2 is a 2D planner and does not classify fabric or
calculate footholds. A curtain mapped as a wall can still block exploration.

QR text and people boxes are published while navigation continues. Unregistered
depth is not treated as aligned RGB depth, so no invented 3D positions are stored.
People detection uses the COCO person class and can also detect a person in a
printed image.

## Validation of the operator start change

A loopback ROS test confirms no automatic-mode command or arming occurs before
an operator `lite3_auto`; H-Step is requested once before arming; and manual-mode
takeover disarms without automatic rearming. Both packages build successfully.
The preceding combined integration tests passed 42 tests with one existing
expected failure. This does not establish a successful complete maze traversal.
