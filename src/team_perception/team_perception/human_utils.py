"""
Person detection with Ultralytics YOLO, with no ROS imports so it can be tested anywhere.

YOLO's COCO class 0 ('person') fires on real people AND on pictures of people
(posters, printouts, screens), which is what the challenge's human targets are.

The model weights (default yolo11n.pt, ~5 MB) are looked up in ~/.cache/team_perception/
(install_deps.sh downloads them there) instead of the current folder.

Pulled from team_perception_external (Robot-Dog-Grp4) to run on the robot computer. Changes
for sharing the CPU with navigation:
  - torch is capped to `threads` intra-op threads. Ultralytics' select_device() resets torch
    to min(8, cpus - 1) threads on the first predict(), which on the robot (10 vCPUs) made
    one 640x480 frame cost ~285 ms of CPU (8 spinning OpenMP threads) instead of ~95 ms.
  - YOLO_OFFLINE=1 by default: ultralytics otherwise does DNS lookups at import (online
    check) and may try to send usage events; the robot has no internet.
"""
import os

import numpy as np

PERSON = 0
MODEL_DIR = os.path.expanduser('~/.cache/team_perception')


def resolve_model(model):
    """Bare names like 'yolo11n.pt' live in MODEL_DIR; paths are used as given."""
    if os.sep in model or os.path.exists(model):
        return os.path.expanduser(model)
    os.makedirs(MODEL_DIR, exist_ok=True)
    return os.path.join(MODEL_DIR, model)


class HumanDetector:
    """Wraps a YOLO model and returns person boxes only."""

    def __init__(self, model='yolo11n.pt', conf=0.4, imgsz=640, device='cpu', threads=2):
        os.environ.setdefault('YOLO_OFFLINE', '1')
        import torch                   # imported here: slow (~1 s)
        from ultralytics import YOLO
        self.torch = torch
        self.threads = int(threads)    # <= 0: leave torch / ultralytics to choose
        self.model = YOLO(resolve_model(model))
        self.conf = conf
        self.imgsz = imgsz
        self.device = device
        if self.threads > 0:
            try:
                torch.set_num_interop_threads(1)
            except RuntimeError:       # only allowed once per process, before parallel work
                pass
        # Warm-up: the first predict() sets the predictor up (and resets torch's thread
        # count, see the module docstring), so cap the threads after it.
        self.detect(np.zeros((imgsz, imgsz, 3), np.uint8))

    def detect(self, bgr):
        """Return [(box[x1, y1, x2, y2] float32, confidence)] for people in a BGR image."""
        r = self.model.predict(bgr, classes=[PERSON], conf=self.conf, imgsz=self.imgsz,
                               device=self.device, verbose=False)[0]
        if self.threads > 0 and self.torch.get_num_threads() != self.threads:
            self.torch.set_num_threads(self.threads)
        boxes = r.boxes.xyxy.cpu().numpy().astype(np.float32)
        confs = r.boxes.conf.cpu().numpy()
        return [(b, float(c)) for b, c in zip(boxes, confs)]


def box_depth_pixel(box):
    """Pixel to sample depth for a person box: centre, a bit above middle (torso)."""
    x1, y1, x2, y2 = box
    return (x1 + x2) / 2.0, y1 + 0.4 * (y2 - y1)
