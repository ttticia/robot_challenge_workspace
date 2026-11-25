# curtain_cv: curtain detection developer guide

This branch (`curtain_cv` of robot_challenge_workspace) holds **only curtain detection** for
the DEEP Robotics Lite3: the lidar gap detector, the lie-down inspection, the YOLO curtain
model in the colour camera, the dataset (labels, frames, manifest), the trained model and the
tools to record, label, train, evaluate, export and mine data. Navigation, stairs, scene
understanding, perception (QR, depth relay) and hardware notes were removed on 2026-11-24;
they live on `integration` / `navigation`.

**Do not merge this branch into `integration` or `navigation`:** that would delete their
navigation code. Bring curtain changes over file by file (`git checkout curtain_cv -- <path>`)
or cherry-pick commits that only touch curtain files.

**Rules for coding agents:** if another session is editing a file you need, check in with it
first. The task list is `src/to_do.md` (## curtains). Keep this file up to date when you add a
tool or change how the model is built or deployed.

## 1. Use this model

**Deploy the OpenVINO INT8 320x256 export:**
`curtain CV/model/curtain_int8_320x256_openvino_model/` (3.3 MB). On the dev laptop
(i7-13700H) it runs ~23 ms a frame against ~78 ms for the PyTorch model, with the same
accuracy at conf 0.5 (val: 19/23 in-range curtain frames, false boxes on 2/51 negatives;
test: 24/29, 1/57). It was exported with the camera's 4:3 shape; Ultralytics uses the
model's own 256x320 input whatever `imgsz` says (checked: imgsz 320 and 416 give the same
results), so `curtain_yolo`'s `imgsz` can stay at its default. Set it to 320 anyway to be
explicit. Don't use a square export (416x416): it pads every frame with grey the model never
saw and gave 8/51 false boxes.

The model is NOT accepted yet (`src/to_do.md`: recall ≥ 0.9 for curtains 0.3-1.5 m away and
no false boxes at conf 0.5; it has 0.83 and 2/51). Use it for robot tests and for
pre-labelling new recordings, keep the lie-down check (`curtain_inspect_mode:=lie`) as the
verification in real runs.

Source weights: `curtain CV/model/curtain_v3_best.pt` (yolo11n, imgsz 416, best epoch 44 of
64, trained 2026-11-24; settings and per-epoch scores next to it).

## 2. Load it onto the robot

The robot computer is `192.168.10.74` (team/lab network; SSH or VS Code Remote; FileBrowser
on port 8080). Our nodes run there with ROS 2 Humble, `ROS_DOMAIN_ID=4`.

```bash
# On the robot computer, once: the runtime for the exported model (CPU only)
pip install --user openvino==2026.4.1          # plus ultralytics==8.4.210 if it isn't installed
python3 -c "import openvino, ultralytics; print(openvino.__version__, ultralytics.__version__)"

# From the dev laptop: copy the model into curtain_yolo's model folder
scp -r "curtain CV/model/curtain_int8_320x256_openvino_model" \
    <user>@192.168.10.74:~/.cache/team_perception/

# On the robot: check it loads and time it (one frame at a time, like the node)
python3 - <<'EOF'
import time, numpy as np
from ultralytics import YOLO
m = YOLO('/home/<user>/.cache/team_perception/curtain_int8_320x256_openvino_model', task='detect')
im = np.zeros((480, 640, 3), np.uint8); m.predict(im, imgsz=320, verbose=False)
t = time.time(); [m.predict(im, imgsz=320, verbose=False) for _ in range(50)]
print(f'{(time.time() - t) / 50 * 1000:.0f} ms/frame')
EOF
```

`<user>` is the robot computer's login. Check numpy on the robot first if `pip` wants to
upgrade it: ROS Humble's SciPy / Matplotlib are built for NumPy 1.x (on the dev laptop the
export tools live in their own venv for that reason, see section 5).

Run it, full stack (from `integration` / `navigation`, where the launch file lives):

```bash
ros2 launch maze_nav maze_nav.launch.py curtain_yolo:=true \
    curtain_yolo_model:=curtain_int8_320x256_openvino_model      # bare name -> ~/.cache/team_perception/
# then, to let the model verify curtain candidates instead of the lie-down check:
#   curtain_inspect_mode:=yolo
```

Run it alone (this branch; no explorer, so make it run on every frame, and read the camera's
own depth instead of the main stack's depth relay):

```bash
ros2 run maze_nav curtain_yolo --ros-args -p model:=curtain_int8_320x256_openvino_model \
    -p imgsz:=320 -p status_topic:='' \
    -p depth_topic:=/camera/depth/image_raw -p depth_info_topic:=/camera/depth/camera_info
ros2 topic echo /curtain/yolo_status          # RUNNING, or UNAVAILABLE: why
ros2 topic echo /curtain/yolo                 # one JSON per frame: boxes, range, odom centre
```

The depth topic names are the camera driver's as recorded in the bags (`/camera/depth/
image_raw`); check the camera-info name with `ros2 topic list | grep depth` on the robot.
Without depth the node still publishes boxes, with `range` / `centre` null.

Before relying on it: time it on the robot (the laptop's 23 ms will be slower there), check
that one more colour-camera subscriber doesn't saturate the robot bridge, and run the robot
test in `src/to_do.md` (the spec curtain and a wall with a cavity).

## 3. What is here

```
agent.md                     this file
ROS_bags/                    record.sh / aliases.sh (rec_curtain), ROSBAG_GUIDE.md (2.5: curtain bags)
curtain CV/                  the dataset and model (images and model tracked, see .gitignore)
├── images/images/*.jpg      717 frames (16 recordings), + *.json: labelling-tool annotations
├── images/labels/*.txt      YOLO labels, class 0 = curtain; empty file = checked negative
├── images/classes.txt
├── manifest.csv             per frame: time, depth straight ahead (ahead_m), draft label
├── extract_frames.py        frames out of colour-only bags (any image topic), with dedup
└── model/                   trained weights, INT8 320x256 export, args, results, val/test lists
src/to_do.md                 curtain task list, results so far, next steps (most impactful first)
src/maze_nav/                ROS package (name kept so the robot stack's imports still match)
├── maze_nav/
│   ├── curtains.py          lidar curtain geometry: walls with a low gap, tracking, diagnosis
│   ├── curtain_detector.py  node: candidates, hold / release with the explorer, inspection
│   ├── curtain_inspect.py   lie-down (and Pose-mode) gap check plan
│   ├── curtain_branch.py    curtain held back until the cell's other way is explored
│   ├── curtain_yolo.py      node: YOLO boxes in the colour camera, placed with depth
│   ├── core.py, stairs.py   shared geometry (grid, cloud parsing, wall check, camera TF)
├── touble_shooting/live_params.py   live parameter overrides used by curtain_detector
├── tools/                   dataset, training and evaluation tools (section 4)
└── test/                    test_curtains, test_curtain_diag, test_curtain_inspect,
                             test_curtain_yolo (+ extru_wall.py helper)
```

`curtain_detector` and `curtain_yolo` are the package's only nodes on this branch (no launch
file: the full launch, the explorer, the lidar relay and the depth relay are on `integration`).
Topics: `/curtain/candidate` (held candidates), `/maze/curtain_release` (explorer releases one),
`/curtain/curtains` (latched confirmed curtains, odom), `/curtain/yolo`, `/curtain/yolo_status`.

## 4. Building the model: data to deployment

The dataset folder used for training (`datasets/curtain_v3` on the dev laptop) is NOT in git;
it is rebuilt from `curtain CV/`. Steps, each a tool in `src/maze_nav/tools/` unless noted:

1. **Record** with the colour camera, depth and TF: `rec_curtain <name>` (ROS_bags/record.sh,
   ROSBAG_GUIDE 2.5). Teleop only. From ~2 m to 0.3 m, square on and 30-45° off, lit and in
   shadow, still and swaying; negatives-only bags too (walls, panels, people, props).
2. **Extract frames:** `curtain_yolo_frames.py <bag> <dataset> --every 10 --lidar` (frames,
   draft labels from depth/lidar, `manifest.csv` with `ahead_m`). Colour-only bags (no depth):
   `python3 "curtain CV/extract_frames.py" <bag> --hz 3 --dedup 0.45` (frames + times, no
   drafts, no `ahead_m`). Frames are named `<bag>_<SSSS>_<cc>.jpg`; the bag name is the split
   unit, so frames of one recording never sit in both train and val.
3. **Remove near-repeats:** `--dedup 0.45` compares frames within 3 s; a second pass over the
   whole recording catches views seen again later (2026-11-24: moved to `<bag>/similar/`).
4. **Label** every frame in the labelling tool (YOLO format, class 0 = curtain): one box per
   curtain from rod (or frame top) to hem, both halves together; far curtains too. Build the
   label `.txt` files from the tool's `.json` (each shape -> its bounding box): the tool's own
   export dropped boxes drawn as a rotated box or a quadrilateral.
5. **Train:** `train_curtain_yolo.py <dataset> --val-bags a,b --test-bags c` (yolo11n, imgsz
   416, CPU ~3 min an epoch for ~1000 frames; split by recording; `--cv` = leave one recording
   out; `--dry-run` shows the split). Don't store mirrored copies: training already flips half
   the images (`fliplr=0.5`); the copies doubled the epoch time for little gain. Keep the
   laptop awake (Windows: never sleep on power): sleep paused a run for over an hour.
6. **Evaluate:** `eval_curtain_yolo.py <dataset> [--list split_test.txt] [--cv]`: per
   recording, curtain frames DETECTED in range (label box on the image centre column and
   `ahead_m` 0.3-1.5 m), out of range and unknown range, at conf 0.25 and 0.5, plus false
   boxes: on negatives, and off-curtain boxes on curtain frames. Scored by detection, not box
   overlap (since 2026-11-25): a frame counts when a model box lies on the curtain (≥ half of
   the box inside a label box; `--on`), because the robot only needs to know a curtain is there.
   A box on one curtain half counts; the old IoU ≥ 0.5 count is printed next to it. The headline
   mAP / recall count far curtains the robot ignores; the in-range detection score is the
   acceptance number. Review sheets of each missed / false frame: `curtain_v3/review/
   make_review.py` on the dev laptop (same rule).
7. **Export and compare:** `bench_curtain_export.py <dataset>` (OpenVINO FP16 / INT8 with the
   camera's 4:3 shape, timing and the step 6 scores for each). Run it in `~/venvs/yolo-export`
   (section 5).
8. **Mine hard negatives:** `mine_curtain_negatives.py <dataset> <out> --pool <frames dir>
   [--pool ...] --val-bags a,b --test-bags c`: runs the model over frames not in the dataset
   (and the dataset's negatives) and writes `candidates.csv` (fill `decision`: neg / curtain /
   skip), review sheets and copies of the frames. Frames of val / test recordings go to val /
   test only, never into training.

GPU: the dev laptop's Intel Iris Xe is not usable for training (PyTorch XPU detects it but the
first computation hangs or takes ~50 s; `torch.xpu.is_available()` segfaults in WSL). Train on
the CPU, or on an NVIDIA machine with `--device 0`.

## 5. Python environments (dev laptop, WSL Ubuntu 22.04)

- System `python3` (ROS Humble sourced): ultralytics 8.4.210, torch 2.14.1+cpu; training,
  evaluation, mining, `extract_frames.py` (needs `rosbag2_py`).
- `~/venvs/yolo-export` (made with `--system-site-packages`): openvino 2026.4.1, nncf 3.4.0,
  onnx, plus its own scipy 1.15 / matplotlib 3.10 (the system ones are built for NumPy 1.x and
  break nncf and the INT8 calibration). Run it with ROS's paths removed:
  `env -u PYTHONPATH ~/venvs/yolo-export/bin/python ...`
- `~/venvs/yolo-xpu`: the Intel GPU attempt, not usable (see above).

## 6. Tests

```bash
(cd src/maze_nav && python3 -m pytest -q test)
```
