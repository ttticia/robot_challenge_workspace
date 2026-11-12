> Updated hardware findings and commands: [HARDWARE_NAVIGATION.md](/home/user/challenge_ws/HARDWARE_NAVIGATION.md). The earlier missing-bridge diagnosis below was superseded by lazy discovery and QoS findings.

# Hardware preflight — 12 November 2026

Ran in the group container (192.168.1.74 / 192.168.10.74), ROS domain 4,
using Fast DDS. The user confirmed a supervisor with handset STOP, standing
robot in the maze and exclusive group control. No drive commands, mode-switch
commands or navigation action goals were sent during these checks.

## Passed observations

- Sensor bridge discovered; body IMU, Livox IMU, custom LiDAR and point clouds
  received continuously. LiDAR stamps were within about 0.3 seconds of host time.
- Static base-to-LiDAR and base-to-camera transforms present.
- Started existing Fast-LIO, odometry relay and pointcloud-to-laserscan components.
  Fast-LIO initialized and the relay obtained the required static transform.
- About 10 Hz odometry and scans received. One scan had 723 beams, 371 finite
  returns and nearest return 0.732 m. These are sample observations, not clearance
  guarantees for the robot footprint.
- SLAM Toolbox started and published a map; exploration mission ran in preview.
- Sample odometry variation over 12 seconds: 0.019 m x, 0.014 m y, 0.010 m z.
  Whether this represents drift or physical motion was not independently verified.

## Blockers and limitations

1. No subscribers discovered for `/cmd_vel`, `/simple_cmd` or `/complex_cmd` on
   domain 4. No command bridge was started. The installed old
   `lite3_bringup/launch/motion_transfer.launch.py` is a dangling symlink to
   `/home/user/ros2_ws/src/lite3_bringup/launch/motion_transfer.launch.py`.
   Old bytecode mentions package `transfer`, executable `jetson2motion`;
   this is historical evidence, not a verified supported startup procedure.
   Restore the supported lab motion path or have the lab operator enable its
   domain bridge, then verify command subscribers before any mode switch.
2. Nav2 controller configured its MPPI plugin, but its change-state response
   timed out. Controller was inactive and planner, behavior server, navigator
   and velocity smoother were unconfigured at the final sample. Nav2 was not
   ready for a movement trial.
3. Exploration reported `SHADOW_START_NOT_CLEAR`. The robot cell was free,
   but distance to non-free/unknown map cells was only 0.05 m, below the current
   0.25 m frontier clearance requirement. Investigate map coverage, scan height,
   self-filtering and pose registration; do not simply lower clearance to arm.
4. Body IMU stamps use a different time base (~1065 seconds in an initial
   sample) from wall-clock LiDAR time. The mission currently checks IMU freshness
   by receipt time, so this is not itself an arming rejection. Do not use these
   body IMU stamps for synchronized geometry without establishing clock mapping.
5. The exit is described as an opening into a large clear area. Frontier
   exploration exists, but an exit-recognition/stop condition is not implemented.
   A large internal room can resemble the exit; dimensions or an independent
   exit cue are needed to validate that behavior.

## Files

- `preflight.json`: map, mission and gate state, odometry sample and topic routing.
- `nav2_preflight.json`: lifecycle states and robot-cell map clearance.
- `map_preview.png`: partial map snapshot, white free / black occupied / gray unknown.
- `base.log`, `slam.log`, `preview.log`, `nav2.log`: process output.
- `nav2_trial.yaml`: separate conservative profile generated from the lab config;
  the installed lab config was not overwritten.

All test launchers started in this session were sent SIGINT for shutdown.
The pre-existing domain bridge was left untouched. This is sensor/mapping
hardware validation, not a successful movement or maze-exit test.
