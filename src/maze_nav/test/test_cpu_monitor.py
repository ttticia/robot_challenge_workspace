"""Unit tests for maze_nav.cpu_monitor (no ROS needed): python3 -m pytest -q test"""
from maze_nav.cpu_monitor import percentile, proc_name, summarise


def test_proc_name_prefers_ros_node_name():
    cmd = ['/usr/bin/python3', '/x/lib/team_perception/human_detector', '--ros-args',
           '-r', '__node:=human_detector', '--params-file', '/tmp/p.yaml']
    assert proc_name(cmd, 'python3') == 'human_detector'


def test_proc_name_skips_interpreter():
    assert proc_name(['/usr/bin/python3.10', '-u', '/x/lib/maze_nav/wall_scan'], 'python3') \
        == 'wall_scan'
    assert proc_name(['/opt/ros/humble/lib/nav2_planner/planner_server'], 'p') \
        == 'planner_server'


def test_proc_name_fallback_for_kernel_threads():
    assert proc_name([], 'kworker/0:1') == 'kworker/0:1'


def test_percentile():
    assert percentile([], 95) == 0.0
    assert percentile(list(range(1, 101)), 95) == 95
    assert percentile([5.0], 50) == 5.0


def test_summary_counts_missing_samples_as_zero_and_sorts():
    text = summarise({'_total_': [300.0] * 4, 'yolo': [200.0, 200.0], 'idle': [0.1]}, 4, 10)
    lines = text.splitlines()
    assert lines[2].startswith('_total_') and '300.0' in lines[2]
    assert lines[3].startswith('yolo') and '100.0' in lines[3]   # mean over 4 samples
    assert not any(line.startswith('idle') for line in lines)
