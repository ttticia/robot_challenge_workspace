"""
Person detection with Ultralytics YOLO, with no ROS imports so it can be tested anywhere.

YOLO's COCO class 0 ('person') fires on real people AND on pictures of people
(posters, printouts, screens), which is what the challenge's human targets are.

The model weights (default yolo11n.pt, ~5 MB) are looked up in ~/challenge_ws/models/team_perception/
(install_deps.sh downloads them there) instead of the current folder.

Pulled from team_perception_external (Robot-Dog-Grp4) to run on the robot computer. Changes
for sharing the CPU with navigation:
  - torch is capped to `threads` intra-op threads. Ultralytics' select_device() resets torch
    to min(8, cpus - 1) threads on the first predict(), which on the robot (10 vCPUs) made
    one 640x480 frame cost ~285 ms of CPU (8 spinning OpenMP threads) instead of ~95 ms.
  - OpenVINO models (a `*_openvino_model/` folder, install_deps.sh exports one) run on
    Intel's CPU runtime instead of torch. Ultralytics compiles them with a LATENCY hint
    (4 threads here); they are recompiled with INFERENCE_NUM_THREADS = `threads`. Their
    input size is fixed at export, so `imgsz` is taken from the model's metadata.yaml.
    Robot, one 640x480 frame, CPU time: torch 640 ~124 ms, torch 416 ~60 ms.
    If the OpenVINO folder is missing, the .pt next to it is used (`fallback` says why).
  - YOLO_OFFLINE=1 by default: ultralytics otherwise does DNS lookups at import (online
    check) and may try to send usage events; the robot has no internet.
"""
import os

import numpy as np

PERSON = 0
# Inside the workspace (user, 2026-11-24: nothing is written outside ~/challenge_ws at run
# time). models/ is not under logs/, so the daily log clean-up leaves it alone.
MODEL_DIR = os.path.expanduser('~/challenge_ws/models/team_perception')
# Ultralytics writes its settings file on import: keep that in the workspace too.
os.environ.setdefault('YOLO_CONFIG_DIR', os.path.expanduser('~/challenge_ws/models/ultralytics'))
OPENVINO_SUFFIX = '_openvino_model'


def resolve_model(model):
    """Bare names like 'yolo11n.pt' live in MODEL_DIR; paths are used as given."""
    if os.sep in model or os.path.exists(model):
        return os.path.expanduser(model)
    os.makedirs(MODEL_DIR, exist_ok=True)
    return os.path.join(MODEL_DIR, model)


def is_openvino(path):
    return os.path.basename(str(path).rstrip('/')).endswith(OPENVINO_SUFFIX)


def openvino_imgsz(path):
    """[h, w] an OpenVINO export was made for (its metadata.yaml), or None."""
    meta = os.path.join(path, 'metadata.yaml')
    try:
        import yaml
        with open(meta) as f:
            size = yaml.safe_load(f).get('imgsz')
    except (OSError, AttributeError, ImportError, ValueError):
        return None
    if isinstance(size, int):
        return [size, size]
    if isinstance(size, (list, tuple)) and len(size) == 2:
        return [int(size[0]), int(size[1])]
    return None


def parse_imgsz(imgsz):
    """640, '640', '320,416' or [320, 416] -> int or [h, w] for ultralytics."""
    if isinstance(imgsz, str):
        parts = [int(p) for p in imgsz.replace('x', ',').split(',') if p.strip()]
        return parts[0] if len(parts) == 1 else parts[:2]
    if isinstance(imgsz, (list, tuple)):
        return [int(imgsz[0]), int(imgsz[1])]
    return int(imgsz)


class HumanDetector:
    """Wraps a YOLO model and returns person boxes only."""

    def __init__(self, model='yolo11n.pt', conf=0.4, imgsz=640, device='cpu', threads=2):
        os.environ.setdefault('YOLO_OFFLINE', '1')
        import torch                   # imported here: slow (~1 s)
        from ultralytics import YOLO
        self.torch = torch
        self.threads = int(threads)    # <= 0: leave torch / ultralytics / OpenVINO to choose
        path = resolve_model(model)
        self.fallback = None           # why the requested model wasn't used, if it wasn't
        if is_openvino(path) and not os.path.isdir(path):
            pt = path.rstrip('/')[:-len(OPENVINO_SUFFIX)] + '.pt'
            self.fallback = (f'{path} not found (export it: bash install_deps.sh); '
                             f'using {pt} on torch')
            path = pt
        self.openvino = is_openvino(path)
        self.imgsz = parse_imgsz(imgsz)
        if self.openvino:
            self.imgsz = openvino_imgsz(path) or self.imgsz
        self.model = YOLO(path, task='detect')
        self.path = path
        self.conf = conf
        self.device = device
        if self.threads > 0 and not self.openvino:
            try:
                torch.set_num_interop_threads(1)
            except RuntimeError:       # only allowed once per process, before parallel work
                pass
        # Warm-up: the first predict() sets the predictor up (and resets torch's thread
        # count, see the module docstring), so cap the threads after it.
        h, w = (self.imgsz if isinstance(self.imgsz, list) else (self.imgsz, self.imgsz))
        self.detect(np.zeros((h, w, 3), np.uint8))
        if self.openvino and self.threads > 0:
            self.cap_openvino_threads()

    def cap_openvino_threads(self):
        """Recompile the OpenVINO model with `threads` inference threads (and keep it so
        for any later recompile ultralytics does)."""
        import glob
        from functools import partial
        import openvino as ov
        backend = self.model.predictor.model.backend
        xml = (self.path if os.path.isfile(self.path)
               else sorted(glob.glob(os.path.join(self.path, '*.xml')))[0])
        cfg = {'PERFORMANCE_HINT': 'LATENCY', 'INFERENCE_NUM_THREADS': self.threads}
        core = ov.Core()
        backend.compile_model = partial(core.compile_model, device_name='CPU', config=cfg)
        backend.ov_compiled_model = backend.compile_model(core.read_model(xml))

    def detect(self, bgr):
        """Return [(box[x1, y1, x2, y2] float32, confidence)] for people in a BGR image."""
        r = self.model.predict(bgr, classes=[PERSON], conf=self.conf, imgsz=self.imgsz,
                               device=self.device, verbose=False)[0]
        if (self.threads > 0 and not self.openvino
                and self.torch.get_num_threads() != self.threads):
            self.torch.set_num_threads(self.threads)
        boxes = r.boxes.xyxy.cpu().numpy().astype(np.float32)
        confs = r.boxes.conf.cpu().numpy()
        return [(b, float(c)) for b, c in zip(boxes, confs)]


def box_depth_pixel(box):
    """Pixel to sample depth for a person box: centre, a bit above middle (torso)."""
    x1, y1, x2, y2 = box
    return (x1 + x2) / 2.0, y1 + 0.4 * (y2 - y1)
