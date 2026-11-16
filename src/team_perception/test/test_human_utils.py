"""Tests for human_utils (YOLO person detector). The YOLO tests need torch, ultralytics and
the weights in ~/.cache/team_perception/ (install_deps.sh); they are skipped without them."""
import os

import cv2
import numpy as np
import pytest

from team_perception.human_utils import MODEL_DIR, box_depth_pixel, resolve_model

IMAGES = os.path.join(os.path.dirname(__file__), 'testing_data', 'test_images')
WEIGHTS = os.path.join(MODEL_DIR, 'yolo11n.pt')


def test_box_depth_pixel_is_centre_above_middle():
    assert box_depth_pixel((100, 200, 300, 400)) == (200.0, 280.0)


def test_resolve_model():
    assert resolve_model('yolo11n.pt') == WEIGHTS
    assert resolve_model('/opt/w/best.pt') == '/opt/w/best.pt'
    assert resolve_model('~/w/best.pt') == os.path.expanduser('~/w/best.pt')


@pytest.fixture(scope='module')
def detector():
    pytest.importorskip('ultralytics')
    if not os.path.exists(WEIGHTS):
        pytest.skip(f'no weights at {WEIGHTS} (run install_deps.sh)')
    from team_perception.human_utils import HumanDetector
    return HumanDetector('yolo11n.pt', conf=0.4, imgsz=640, device='cpu', threads=2)


def test_thread_cap_survives_ultralytics_setup(detector):
    # ultralytics' select_device() resets torch to ~8 threads on the first predict()
    import torch
    detector.detect(np.zeros((480, 640, 3), np.uint8))
    assert torch.get_num_threads() == 2


def test_no_people_in_blank_image(detector):
    assert detector.detect(np.zeros((480, 640, 3), np.uint8)) == []


def test_finds_person_in_test_image(detector):
    img = cv2.imread(os.path.join(IMAGES, 'test_c.jpg'))
    dets = detector.detect(cv2.resize(img, (640, 480)))
    assert dets, 'expected a person in test_c.jpg'
    box, conf = dets[0]
    assert box.shape == (4,) and 0.4 <= conf <= 1.0
