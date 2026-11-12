# Maze hardware trial — 12 November 2026

Navigation translation was not attempted. No automatic-mode switch or nonzero
velocity command was sent by this trial.

- Live Nav2 servers were active, with fresh map, scan and odometry.
- Map clearance at the robot was about 0.539 m; exploration selected a frontier.
- Live camera showed steps ahead. User confirmed stairs are required.
- The level-ground scan reported a forward obstacle at about 1.44 m. This is
  not validation of stair clearance: lower terrain is excluded by its height filter.
- A planning-only request for a point 0.3 m forward returned status 4 (succeeded)
  with 12 path poses. No NavigateToPose action was sent. This demonstrates why
  2D planning success is insufficient to approve stair traversal.
- Sent manufacturer H-Step request `0x21010407` through the existing group bridge;
  DDS delivery was acknowledged. No gait-state feedback topic was available.
- Operator then reported that the robot was not standing steadily. Cause is not
  established by these observations. Navigation was halted.
- Sent manufacturer software STOP `0x21020C0E`; DDS delivery was acknowledged.
  Operator was instructed to press handset STOP and keep clear.
- Requested shutdown of our hardware-navigation launcher. Existing sensor/domain
  bridge was not changed. Physical stop confirmation was requested separately.

Files: `preflight.json`, `camera_before.jpg`, `map_before.npz`, `base_cloud.npy`,
`plan_only.json`, `gait_request.json`, `stop_request.json`.

Do not restart automatic navigation or switch gait again until the operator has
restored a stable supported standing state. The present controller has no
validated stair-traversal behavior; H-Step selection does not supply that missing
terrain-aware navigation behavior.
