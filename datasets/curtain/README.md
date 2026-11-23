# Curtain YOLO dataset (2026-11-23)

One class: `0 curtain` (YOLO format, `labels/<image>.txt`, normalised `x y w h`).

| Folder | What |
|---|---|
| `images/`, `labels/` | **Training set**: 446 frames, 250 with a curtain, 196 negatives (empty label file = checked, no curtain) |
| `duplicates/` (+ `duplicates/labels/`) | 1263 near-repeat frames removed by `dedup_frames.py 0.45` (kept for reference, not for training) |
| `excluded/` | 43 frames with no usable label (camera pressed into the fabric, or only an unclear sliver of curtain) |
| `manifest.csv` | every extracted frame: time, depth straight ahead (m), draft yes/no and the depth detector's reason |
| `data.yaml` | YOLO config |

Bags (competition curtains): `curtain_test1`, `curtain_lab_45deg_lit_01`, `curtain_slit_square_lit_01`,
`curtain_bag1` (recorded with a degraded bridge: uneven timing), `curtain_bag2`, `curtain_n1` (negatives only) and
`scene_check` (2026-11-23 robot run over the block field in the practice arena: the curtain at 1-3 m behind
the blocks, often cut off at the frame top; 45 curtain + 5 negative; colour only, no depth, so no drafts;
frames at 3 Hz, then de-duplicated).
Frames are named `<bag>_<SSSS>_<cc>.jpg` (seconds, hundredths into the bag).

**Phone frames** (`phone_IMG_1671_*`, `phone_IMG_1672_*`): from hand-held phone videos in
`maze_examples/curtains` (portrait 720x1280 resized to 360x640, every 10th frame, then de-duplicated),
NOT the robot camera: higher viewpoint, different lens and colours, no depth. 42 curtain + 1 negative.
Training only - `train_curtain_yolo.py` never puts `phone_*` in val, test or a CV fold.

Labelling rules: one box per curtain from the rod (or frame top) to the hem, both halves together;
curtains far in the background are boxed too. Boxes were placed by eye on gridded images and
checked drawn on each frame, so expect a few percent error at the edges.

Training: `python3 src/maze_nav/tools/train_curtain_yolo.py <this folder> --cv` (leave-one-bag-out score), then `--val-bags <bag> --install`; `--test-bags` keeps an acceptance bag out of both, `--dry-run` shows the split.
Val is always whole bags with curtains in them (a negatives-only val set is refused). Acceptance (to_do.md): recall
>= 0.9 for curtains 0.3-1.5 m away on bags not used in training, no boxes on negatives at conf 0.5.
