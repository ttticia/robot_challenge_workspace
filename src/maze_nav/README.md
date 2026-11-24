# maze_nav (curtain_cv branch): curtain detection only

This branch keeps only the curtain part of `maze_nav`; the full package (wall scan, SLAM,
Nav2, explorer, stairs, launch file) is on `integration` / `navigation`. The package name is
unchanged so the curtain modules can be copied back as they are.

- `curtain_detector`: walls with a low gap in the lidar (`curtains.py`), candidates held and
  released with the explorer, lie-down / Pose-mode / YOLO inspection (`curtain_inspect.py`).
- `curtain_yolo`: YOLO curtain boxes in the colour camera, placed with depth, on `/curtain/yolo`.
  Use the OpenVINO INT8 320x256 export in `curtain CV/model/`.
- `tools/`: frame extraction, training, in-range evaluation, OpenVINO export benchmark,
  hard-negative mining.

How to build the dataset, train, evaluate, export and deploy: `agent.md` at the repo root.
Tests: `python3 -m pytest -q test`.
