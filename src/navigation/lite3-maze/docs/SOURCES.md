# Source provenance

The new package was built in `/home/vicdeo/ir_challenge_revised`. Reused navigation,
cell bookkeeping and simulation bridge code came from the sibling local project
`/home/vicdeo/ir_challenge`; its earlier validation claims were not carried over
as validation of this package. New run records are under `docs/validation`.

The model meshes and ONNX locomotion policy are from
[DeepRoboticsLab/Lite3_rl_deploy](https://github.com/DeepRoboticsLab/Lite3_rl_deploy),
commit `1c4a8f14c9c531903fa5a26783e3f9b99d1039d9`. The vendor license is retained
in `vendor/lite3/LICENSE`. The generated flat MJCF retains the vendor robot's
inertias, joints, actuators and sensors. The example terrain generator adds
seeded tiles with different height ranges in each 1 m cell.

[The supplied Motion Host Communication Interface](https://alidocs.dingtalk.com/i/p/OlnXRxOLAljEbGLp/docs/kDnRL6jAJM3d9LPdSG4dpq6AWyMoPYe1)
and the supplied gait table establish RUG General `0x21010401` and RUG High-Step
`0x21010407`, type 0. A local reference copy is available at
`docs/lite3_development/motion_host_interface.md`. The manual separates
Navigation mode (`0x21010C03`) from vendor AI state and specifies gait switching
in Move mode outside AI state. This program uses the existing ROS transfer
bridge, without creating a second direct UDP controller or changing firmware.

The local course handouts `ROS_Lab_Exercise3.pdf`, `ROS_Lab_Exercise4.pdf` under
`/home/vicdeo/ir_challenge/Previous Labs` establish `/simple_cmd`,
`transfer_interfaces/msg/MotionSimpleCMD`, Navigation `0x21010C03`, Manual
`0x21010C02`, and the `lite3_auto`/`lite3_manual` workflow. They are references,
not instructions to execute their hardware setup procedures.

The [team repository](https://github.com/ttticia/robot_challenge_workspace) was
read through the GitHub connector. Its QR scanner publishes JSON Strings on
`/qr/detections`. As inspected on 13 November 2026, its
[`yolo_node.py`](https://github.com/ttticia/robot_challenge_workspace/blob/main/src/team_perception/team_perception/yolo_node.py)
only prints a greeting. The original stub is not used as a detector; the new package now supplies CPU YOLO inference.
Detector code is kept in the team workspace; this package provides optional
JSON input adapters and clearly labelled empty placeholder publishers.


The newly added CPU vision backend uses the packaged YOLOv5n v7.0 ONNX model
from [Ultralytics YOLOv5 releases](https://github.com/ultralytics/yolov5/releases/tag/v7.0).
Its license is retained at `src/lite3_maze/models/YOLOv5_LICENSE.txt`. The local
ONNX detector/letterbox/NMS implementation was reused from the sibling project's
completed `team_perception` extension. See the upstream
[ONNX export/inference documentation](https://docs.ultralytics.com/yolov5/tutorials/model-export/).
The upstream team's original QR node is unchanged. Their placeholder YOLO file
is not presented as a functioning detector; the `lite3_maze` package supplies it.
