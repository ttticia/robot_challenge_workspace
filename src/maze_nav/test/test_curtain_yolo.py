"""curtain_yolo (YOLO curtain boxes placed with depth) and the 'yolo' inspection, no ROS, no
model: the geometry, the hit matching, the InspectPlan YOLO phase and the confirm / reject."""
import json
import math
import os
import sys
from types import SimpleNamespace

import numpy as np

from maze_nav import curtain_yolo as Y
from maze_nav.curtain_detector import CurtainDetector
from maze_nav.curtain_inspect import InspectParams, InspectPlan

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import train_curtain_yolo as T  # noqa: E402

K = (300.0, 300.0, 160.0, 120.0)          # 320 x 240 colour camera


def test_points_in_box_projects_with_the_colour_intrinsics():
    pts = np.array([[0.0, 0.0, 1.0],       # image centre (160, 120)
                    [0.5, 0.0, 1.0],       # u = 310: right edge
                    [0.0, 0.0, -1.0]])     # behind the camera
    m = Y.points_in_box(pts, K, (100, 80, 220, 160), shrink=0.1)
    assert m.tolist() == [True, False, False]


def test_place_takes_the_fabric_not_what_shows_through_the_gap():
    rng = np.random.default_rng(0)
    fabric = np.column_stack([rng.normal(1.2, 0.01, 60), rng.normal(0.1, 0.05, 60),
                              rng.uniform(0.1, 1.0, 60)])
    behind = np.column_stack([np.full(20, 2.5), np.zeros(20), np.full(20, 0.02)])
    rng_, lat = Y.place(np.vstack([fabric, behind]))
    assert abs(rng_ - 1.2) < 0.03 and abs(lat - 0.1) < 0.03
    assert Y.place(fabric[:5]) is None


def test_to_odom_and_matching_hits():
    c = Y.to_odom((1.0, 0.0), ((2.0, 1.0), math.pi / 2))
    assert np.allclose(c, (2.0, 2.0))
    dets = [{'conf': 0.9, 'centre': [2.1, 2.0]}, {'conf': 0.3, 'centre': [2.0, 2.0]},
            {'conf': 0.9, 'centre': None}, {'conf': 0.9, 'centre': [4.0, 2.0]}]
    assert Y.matching_hits(dets, (2.0, 2.0), 0.6, 0.5) == [dets[0]]


def test_missing_model_is_reported():
    try:
        Y.CurtainYolo('/nonexistent/curtain.pt')
    except FileNotFoundError as e:
        assert 'train_curtain_yolo' in str(e)
    else:
        raise AssertionError('no error for a missing model')


def test_inspect_plan_yolo_holds_standing_then_ends():
    p = InspectParams(look_mode='yolo', yolo_hold=3.0, yolo_distance=1.0, settle=0.5)
    plan = InspectPlan((2.0, 0.0), (1.0, 0.0), 0.0, p)
    assert np.allclose(plan.target, (1.0, 0.0))            # 1 m in front, not 0.5
    plan.phase, plan.since = 'SETTLE', 0.0
    s = plan.tick(1.0, (1.0, 0.0), 0.0)
    assert s.phase == 'YOLO' and s.measuring and s.lie is None and s.pose is None
    assert plan.tick(2.0, (1.0, 0.0), 0.0).phase == 'YOLO'
    s = plan.tick(4.5, (1.0, 0.0), 0.0)
    assert s.done and plan.result.startswith('YOLO look')


class Det:
    for _n in ('on_yolo', 'decide_yolo'):
        locals()[_n] = getattr(CurtainDetector, _n)

    def __init__(self):
        self.yolo_cfg = {'conf': 0.5, 'need': 3, 'match': 0.6}
        self.yolo_frames, self.yolo_hits = 0, 0
        self.inspection = SimpleNamespace(phase='YOLO')
        self.inspect_target = {'centre': np.array([2.0, 0.0]), 'p1': np.array([2.0, -0.4]),
                               'p2': np.array([2.0, 0.4]), 'n': np.array([1.0, 0.0]),
                               'gap': 0.1}
        self.rejected, self.confirmed = [], []

    def reject(self, centre, why):
        self.rejected.append(why)

    def frame_tf(self):
        return (0.0, 0.0, 0.0)

    def publish_curtains(self):
        pass

    def curtains_odom(self):
        return []

    def after_confirm(self, cur, why):
        self.confirmed.append(why)

    @property
    def tracker(self):
        return SimpleNamespace(confirm=lambda c, t: SimpleNamespace(id=5))


def frame(*centres):
    return SimpleNamespace(data=json.dumps({'detections': [
        {'conf': 0.8, 'centre': list(c)} for c in centres]}))


def test_yolo_confirms_with_enough_hits_on_the_candidate():
    d = Det()
    for _ in range(3):
        d.on_yolo(frame((2.05, 0.1)))
    d.on_yolo(frame())                                      # a frame with no box
    assert (d.yolo_frames, d.yolo_hits) == (4, 3)
    d.decide_yolo(d.inspect_target, (1.0, 0.0), 'YOLO look (3 s)')
    assert d.confirmed and not d.rejected


def test_yolo_rejects_few_hits_or_no_frames():
    d = Det()
    d.on_yolo(frame((2.0, 0.0)))
    d.on_yolo(frame((4.0, 0.0)))                            # a box elsewhere
    d.decide_yolo(d.inspect_target, (1.0, 0.0), 'YOLO look (3 s)')
    assert d.rejected and '1 of 2' in d.rejected[0]
    d = Det()
    d.decide_yolo(d.inspect_target, (1.0, 0.0), 'YOLO look (3 s)')
    assert 'no /curtain/yolo frames' in d.rejected[0]


def test_frames_outside_the_yolo_phase_are_not_counted():
    d = Det()
    d.inspection.phase = 'GOTO'
    d.on_yolo(frame((2.0, 0.0)))
    assert d.yolo_frames == 0


def test_training_split_keeps_each_bag_on_one_side():
    imgs = [f'/d/images/bag{b}_{t}.jpg' for b in range(5) for t in range(10)]
    train, val = T.split_by_bag(imgs, 0.2)
    assert {T.bag_of(i) for i in train}.isdisjoint({T.bag_of(i) for i in val})
    assert len(train) + len(val) == 50 and 0 < len(val) < 25
