# Implementation checked against the downloaded Lite3 documents

These are reference facts extracted from `docs/lite3_development`, not a request
to execute the documents' installation, service, network, or SDK procedures.
The existing lab ROS transfer bridge remains the hardware interface.

| Reference fact | Implementation consequence |
| --- | --- |
| ROS `/cmd_vel` is `geometry_msgs/Twist`; x forward, y left, angular z left | The planner publishes standard ROS signs. It does not apply the raw UDP right-positive lateral/yaw convention a second time. |
| The MotionSender subscribes to `cmd_vel`, `cmd_vel_corrected`, `simple_cmd`, `complex_cmd` | Navigation uses `/cmd_vel`; mode/gait commands use the documented `MotionSimpleCMD`. No second direct UDP client is opened. |
| Navigation is `0x21010C03`, Manual is `0x21010C02` | `lite3_auto` enables the mission; Manual stops/releases autonomous velocity. |
| STOP is `0x21020C0E` | The observer treats STOP as a mission disable event as well as the firmware command. A fresh Auto event is required to resume. |
| Gait switching requires Move mode and being outside AI state | On the physical Auto transition, the adapter requests Move (`0x21010D06`) before RUG commands. It does not enter SDK or AI mode. |
| RUG General is `0x21010401`; RUG H-Step is `0x21010407` | These are the only requested gait presets; a 5-second terrain hold reduces switching. |
| Raw movement axes need at least 20 Hz; commands expire after 250 ms | A separate 50 Hz ROS command stream expires its navigation lease after 200 ms. A slow planning callback cannot keep an old motion command alive indefinitely. The installed transfer bridge owns raw UDP conversion/heartbeat. |
| A direct UDP connection needs heartbeat at least 2 Hz | There is no new direct hardware UDP connection or duplicated heartbeat owner in the mission package. |
| SDK mode replaces the locomotion controller and has a separate startup/calibration sequence | The physical mission stays on the factory high-level controller. The MuJoCo learned policy is simulation-only. |

The motion development guide lists body length **548 mm**, width **370 mm**,
upper/lower legs **200/210 mm**, and foot radius **20 mm**. The configurable
0.64 x 0.46 m planning footprint includes an allowance around the body; body
measurements alone are not the moving foot envelope of a RUG gait. The MuJoCo
robot retains the vendor model's articulated geometry.

The guide's MID360 mount is **(0.18678, 0, 0.12498) m** from body coordinates.
That matches the actual `base_link -> livox_frame` transform in the supplied bag.
Hardware point clouds use the live TF extrinsics, rather than substituting the
simulator's approximate sensor mount. The listed depth-camera mount is
**(0.25228, -0.00050, 0.07410) m**; actual camera TF/calibration remains the source
of truth for image geometry. The generic simulated cameras/rays are development
approximations, not calibrated D435i/MID360 emulators.

The perception guide identifies a D435i for the Pro/LiDAR edition. The kit guide
also describes other Venture/Orin arrangements and FAST-LIVO2 integration.
These describe different hardware/software configurations, so the mission does
not replace the lab's working drivers or assume a particular computer from the
manual alone. The physical profile consumes the course's live SLAM map/TF.

The kit guide's FAST-LIVO2 example explicitly disables aligned depth. The sample
bag likewise contains raw depth and separate RGB calibration, with no aligned
RGB-depth topic. YOLO boxes remain image measurements and `localized: false`;
no 3D person position is fabricated. QR decoding can connect through its existing
JSON topic without assuming registered depth.

Application attitude stopping is disabled as requested. The motion guide's
built-in overtemperature and fall protection are properties of the robot firmware
and are unchanged by this application parameter.

References:
[motion development](lite3_development/motion_development.md),
[communication interface](lite3_development/motion_host_interface.md),
[perception development](lite3_development/perception_development.md),
[SDK mode](lite3_development/motion_host_sdk_mode.md),
[kit installation](lite3_development/kit_installation.md).
