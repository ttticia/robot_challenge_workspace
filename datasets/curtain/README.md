# Curtain YOLO dataset (2026-11-23)

One class: `0 curtain` (YOLO format, `labels/<image>.txt`, normalised `x y w h`).

| Folder | What |
|---|---|
| `images/`, `labels/` | **Training set**: 638 labelled frames, 298 with a curtain, 340 negatives (empty label file = checked, no curtain) |
| `duplicates/` (+ `duplicates/labels/`) | Not on disk: 1338 near-repeat frames removed by `dedup_frames.py 0.45` were deleted 2026-11-23 to save space (still in git at commit 044e17c, except the 75 unlabelled phone repeats). `dedup_frames.py` recreates the folder when it runs. |
| `excluded/` | 51 frames with no usable label (camera pressed into the fabric, only an unclear sliver of curtain, or a sideways view) |
| `manifest.csv` | every extracted frame: time, depth straight ahead (m), draft yes/no and the depth detector's reason |
| `data.yaml` | YOLO config |

Bags (competition curtains): `curtain_test1`, `curtain_lab_45deg_lit_01`, `curtain_slit_square_lit_01`,
`curtain_bag1` (recorded with a degraded bridge: uneven timing), `curtain_bag2`, `curtain_n1` (negatives only) and
`scene_check` (2026-11-23 robot run over the block field in the practice arena: the curtain at 1-3 m behind
the blocks, often cut off at the frame top; 45 curtain + 5 negative; colour only, no depth, so no drafts;
frames at 3 Hz, then de-duplicated) and
`demo` (`runs/ROS_bags/demo`, 2026-11-23 robot run through the competition arena: curtain from behind the
block field to pushing through it, twice, plus stairs, grey walls and props as negatives; 20 curtain + 102
negative; colour only, 3 Hz, de-duplicated, the 347 near-repeats not kept; currently in `split_train.txt`).
`img_20261124_035654` (`runs/ROS_bags/img_20261124_035654`, 2026-11-24, colour only, hand-carried camera view outside
the arena: people, chairs, dark red table skirts, grey panels and close-ups of an OSB board; 40 negatives,
no curtain; 3 Hz, de-duplicated, the 132 near-repeats not kept; in `split_train.txt`).
Frames are named `<bag>_<SSSS>_<cc>.jpg` (seconds, hundredths into the bag).

**Phone frames** (`phone_crop_IMG_1671/1672/1679/1680_*`, 70 curtain + 3 negative): hand-held phone videos
(1671/1672 from `maze_examples/curtains`; 1679/1680 from `datasets/`: curtain behind the block field and roller
in the practice arena, far to close-up), NOT the robot camera: different lens and colours, brighter, no depth.
Every 10th frame, portrait 720x1280 resized to 360x640, de-duplicated, then cut to a 4:3 landscape crop
(360x270, full width, hem at ~60 % of the height) so the framing is like the robot view: curtain cut off at the
top, no rod. 2 all-fabric crops left out. Boxes on the 1671/1672 crops were tightened to the fabric 2026-11-23.
The portrait originals (75 frames, older looser labels) are in `phone_portrait/`, not in the training set.
Training only - `train_curtain_yolo.py` never puts `phone_*` in val, test or a CV fold.

Labelling rules: one box per curtain from the rod (or frame top) to the hem, both halves together;
curtains far in the background are boxed too. Boxes were placed by eye on gridded images and
checked drawn on each frame, so expect a few percent error at the edges.

Training: `python3 src/maze_nav/tools/train_curtain_yolo.py <this folder> --cv` (leave-one-bag-out score), then `--val-bags <bag> --install`; `--test-bags` keeps an acceptance bag out of both, `--dry-run` shows the split.
Val is always whole bags with curtains in them (a negatives-only val set is refused). Acceptance (to_do.md): recall
>= 0.9 for curtains 0.3-1.5 m away on bags not used in training, no boxes on negatives at conf 0.5.
