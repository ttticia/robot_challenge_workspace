"""run_gate (pause by /maze/status), human_utils model/imgsz helpers, robot.launch cpu_prefix."""
import importlib.util
import os

from team_perception.human_utils import is_openvino, openvino_imgsz, parse_imgsz
from team_perception.run_gate import DEFAULT_PAUSE, RunGate, state_of

HERE = os.path.dirname(os.path.abspath(__file__))


def load_launch():
    path = os.path.join(HERE, '..', 'launch', 'robot.launch.py')
    spec = importlib.util.spec_from_file_location('robot_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_state_of():
    assert state_of('STAIRS: stair_detector drives') == 'STAIRS'
    assert state_of('exploring') == 'EXPLORING'


def test_gate_runs_until_status_seen():
    g = RunGate(DEFAULT_PAUSE)
    assert g.active                      # perception on its own: never paused


def test_gate_pauses_and_resumes():
    g = RunGate(DEFAULT_PAUSE)
    for s, want in (('STARTING', False), ('PAUSED: ready', True), ('EXPLORING: go', True),
                    ('TURNING', True), ('JUNCTION: looking', True), ('DEAD_END', True),
                    ('STAIRS: x', False), ('AT_STAIRS', False), ('CURTAIN: y', False),
                    ('AT_CURTAIN', False), ('EXPLORING', True)):
        g.on_status(s)
        assert g.active is want, s


def test_gate_off_with_blank_list():
    g = RunGate([''])
    g.on_status('STAIRS')
    assert g.active


def test_parse_imgsz():
    assert parse_imgsz(640) == 640
    assert parse_imgsz('416') == 416
    assert parse_imgsz('320,416') == [320, 416]
    assert parse_imgsz([320, 416]) == [320, 416]


def test_openvino_helpers(tmp_path):
    d = tmp_path / 'yolo11n_openvino_model'
    d.mkdir()
    (d / 'metadata.yaml').write_text('imgsz:\n- 320\n- 416\n')
    assert is_openvino(str(d)) and is_openvino(str(d) + '/')
    assert not is_openvino(str(tmp_path / 'yolo11n.pt'))
    assert openvino_imgsz(str(d)) == [320, 416]
    assert openvino_imgsz(str(tmp_path)) is None


def test_cpu_prefix():
    mod = load_launch()
    have = sorted(os.sched_getaffinity(0))
    auto = mod.cpu_prefix('10', 'auto')
    if len(have) >= 6:
        assert auto == f'nice -n 10 taskset -c {have[4]}-{have[-1]}'
    else:
        assert auto == 'nice -n 10'
    assert mod.cpu_prefix('0', 'all') is None
    assert mod.cpu_prefix('5', '2-3') == 'nice -n 5 taskset -c 2-3'


def test_perceive_overrides_status():
    g = RunGate(DEFAULT_PAUSE)
    g.on_status('EXPLORING: go')
    assert g.active                      # no /maze/perceive yet: old rule
    g.on_perceive(False)
    assert not g.active                  # dead ends only: off while exploring
    g.on_status('DEAD_END: holding')
    g.on_perceive(True)
    assert g.active
    g.on_perceive(False)
    g.on_status('EXPLORING')
    assert not g.active


def test_running_ignores_perceive():
    """QR reading (running) stays on outside the dead-end window, off in pause_states."""
    g = RunGate(DEFAULT_PAUSE)
    assert g.running
    g.on_status('EXPLORING')
    g.on_perceive(False)
    assert g.running and not g.active
    g.on_status('STAIRS: climbing')
    assert not g.running
    g.on_status('CURTAIN')
    assert not g.running
    g.on_status('DEAD_END: holding')
    g.on_perceive(True)
    assert g.running and g.active


def test_gate_follows_the_v2_explorer_looks():
    g = RunGate(DEFAULT_PAUSE)
    look = 'RUN cell=(1, 0) job=[look cell (1, 0) side 3 via 0 cells] act=hold '
    g.on_explore_status('RUN cell=(0, 0) job=[visit cell (0, 0) side 0 via 0 cells] act=drive ')
    assert g.perceive is False and not g.active        # known explorer, no look: off
    g.on_explore_status('RUN cell=(0, 0) job=[look cell (1, 0) side 3 via 1 cells] act=drive ')
    assert not g.active                                # still walking to the look
    g.on_explore_status(look)
    assert g.active
    g.look_seen -= 10.0                                # the explorer went quiet mid-look
    assert not g.active
    g.on_explore_status(look)
    g.on_explore_status('FAULT cell=(1, 0) job=[look cell (1, 0) side 3 via 0 cells] act=- x')
    assert not g.active
