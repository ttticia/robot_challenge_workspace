Staging for colour-only bags (no depth, so curtain_yolo_frames.py can't run on them).
extract_colour.py <bag> <this folder> <hz> -> images/ + manifest.csv; then
python3 ../curtain/dedup_frames.py 0.45 apply here, label (sc_boxes.json, sheet.py --grid),
copy into ../curtain. scene_check (2026-11-23) is merged; this folder can be deleted.
