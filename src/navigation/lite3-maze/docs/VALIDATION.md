# Validation — 13 November 2026

This record covers the new `ir_challenge_revised` implementation. Earlier results
from the sibling project are not used as proof of this package.

## Local tests

- Seven core checks cover observed scan translation, unobservable geometry,
  unknown/body-corner collision rejection, A* connectivity and bounds,
  recentering from the captured corner failure, false exit rejection, Livox field
  decoding/padded rows/endianness, full turning clearance at frontier goals, and
  capture-time-bounded recovery after delayed scans.
- The isolated ROS physical-interface fixture exercises external `/map` and TF,
  Auto start, Manual stop/release, configured physical velocity, raised-terrain
  High-Step selection, and stop on stale localization. It also checks the
  documented Auto/Move/RUG/STOP command adapter, and independent 50 Hz command
  publication with a 200 ms planning lease. No robot is connected.
- The real YOLO model is tested through compressed ROS camera messages, the
  detection adapter, people count and annotated RViz image. A positive test image
  yielded a person detection around 0.79 confidence.
- Source compilation, shell syntax, colcon build and launch argument discovery
  pass. Model licenses and source provenance are included.

See `validation/core_tests.txt`, `validation/ros_contract.log` and
`validation/yolo_positive.json`.

## Sample bag

All **522** `/livox/points` clouds decode with the actual field layout:
**10,439,808** finite XYZ points. The bag has 37,273 messages over 52.225 seconds.
Its dynamic `/tf` topic is empty; body IMU/leg odometry stamps use uptime while
camera/Livox stamps use epoch time. No capture timestamp is rewritten.

YOLO processed 52 RGB frames sampled at one-second intervals, with mean CPU
inference around **39 ms** per frame. One sampled frame produced a person box
at about 0.49 confidence. These are model outputs, not a measurement of detection
accuracy or calibrated 3D positions. See `bag_inspection.json` and
`validation/yolo_bag.json`.

## Closed-loop MuJoCo

The example world contains sixteen 1 m cells with different seeded roughness,
an enclosed start, and a single opening to a large clearing. The navigator uses
sensor data only; geometry, the known route and ground truth are excluded from
control. Ground truth is consumed by the separate evaluator and physics metrics.

The completed RGB/YOLO run at a target 1.5 simulation/wall factor visited all
sixteen maze cells, travelled about **18.4 m**, confirmed the clearing and
published zero thereafter. It exercised Manual pause/resume and produced 161
YOLO frames during the mission. Settling after the completion report was about
**3.3 mm**. The physics observer recorded **zero wall-contact samples** at 20 ms
sampling, maximum roll/pitch about **6.1 degrees**, and no policy fault. This is
not a proof of zero contact between samples.

See `validation/rough_yolo_run.json`, `validation/rough_yolo_physics.json` and
`validation/rviz_yolo_live.png`. Earlier local headless and RGB runs also reached
the clearing. A graphics run exposed insufficient room to turn beside a frontier;
planning now reserves the full rotating footprint. A default-speed run exposed
a fixed ICP inter-scan displacement gate; it now scales with capture-time
separation while retaining residual, correspondence and observability checks.
The final default-speed run also completed all sixteen cells. It travelled about
**19.6 m** in **136.8 wall seconds**, with a Manual pause/resume during the test,
222 YOLO frames processed, zero motion commands after completion, and about
**1.6 cm** settling. Physics ran at essentially real time. Maximum roll/pitch was
about **5.6 degrees**, with zero sampled wall contacts and no policy fault. See
`validation/default_speed_run.json`, `validation/default_speed_physics.json`, and
`validation/rviz_default_exit.png`.

## Scope limits

The supplied vendor ONNX locomotion policy is a simulation controller. It does
not emulate factory RUG General/High-Step. The example terrain is not a measured
competition surface. Physical gait response, installed course launcher layout,
actual moving foot clearance and stopping distance require an on-robot run.
The physical map/TF path passes the ROS fixture; the actual course stack is not
installed on this laptop. Application tilt stops are disabled; firmware fall and
overtemperature protection are unchanged.

The 0.9 m clearing radius assumes no internal area is as open as the exit. The
threshold is configurable. Generic YOLO person recognition has not been evaluated
for the challenge's photos, mannequins or prone figures. QR and scene outputs
remain placeholders until the chosen detector inputs are connected.
