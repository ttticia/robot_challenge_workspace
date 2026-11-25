# curtain_v3: training set, run and review of the v3 curtain model (2026-11-24)

The dataset, training run and evaluation behind the deployed model
(`curtain CV/model/curtain_int8_320x256_openvino_model`). Images are not in git (duplicates):
run `python3 datasets/curtain_v3/rebuild_images.py` from the repo root to restore `images/`
from `curtain CV/images/images/` and regenerate the mirrored copies. It also rewrites the split
lists, which hold the dev laptop's absolute paths.

| Path | What |
|---|---|
| `labels/` | YOLO labels for all 1233 frames: 717 originals (built from the labelling tool's `.json`, each shape -> its bounding box) + 516 mirrored copies of the training frames |
| `split_train.txt`, `split_val.txt`, `split_test.txt`, `split.yaml` | train 1032 (516 + mirrors), val 115 (`curtain_lab_45deg_lit_01`, `curtain_test1`), test 86 (`manual_20261124_031525`); split by recording |
| `manifest.csv` | per frame: time, depth straight ahead (`ahead_m`, for the 0.3-1.5 m in-range score) |
| `runs/curtain/` | training run: `args.yaml`, `results.csv` (per epoch), curves, val batches, `weights/best.pt` (epoch 44 of 64) + `last.pt` |
| `runs/curtain/export/` | OpenVINO exports: FP16 / INT8 at 416x320 and 320x256 (4:3, use INT8 320x256), FP16 416x416 (square: more false boxes, don't use); `bench.log` = speed + accuracy of each |
| `runs/curtain/compare_runs.png` | run 1 (23 Nov) vs run 2 (24 Nov) vs v3: mAP50 by epoch, in-range recall, false boxes |
| `review/` | missed (14), false (5) and correct (79) detections of the INT8 320x256 model on val + test at conf 0.5, scored by detection: `missed_*.jpg`, `false_*.jpg`, `correct_*.jpg`, `review.csv` (one row per frame, verdict column to fill in), `make_review.py` to regenerate |
| `mining/` | hard-negative mining over frames not in the dataset (`candidates.csv` with a decision column, `sheets/`); the frame copies (`mining/images/`) are not in git |
| `train.log` | the training console log |

Results (val, INT8 320x256, conf 0.5, by detection): 20/23 in-range curtains detected, off-curtain
boxes on 3/64 curtain frames, false boxes on 2/51 negatives; test 24/29, 1/57. Not accepted yet:
see `src/to_do.md` (## curtains) and `agent.md`.
