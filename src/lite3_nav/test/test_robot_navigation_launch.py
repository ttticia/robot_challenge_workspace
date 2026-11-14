"""Offline checks of launch/robot_navigation.launch.py and its configs (no ROS graph needed)."""
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_launch():
    spec = importlib.util.spec_from_file_location('robot_navigation_launch',
                                                  ROOT / 'launch' / 'robot_navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('domain,robot,ok', [
    ('4', True, True), ('77', False, True), ('0', False, True), (None, False, True),
    ('89', False, False), ('89', True, False),   # never against the simulator
    ('77', True, False), (None, True, False),     # robot needs domain 4
    ('4', False, False),                          # a dry run must not reach the robot
])
def test_domain_guard(domain, robot, ok):
    assert (load_launch().check_domain(domain, robot) is None) == ok


def test_nav2_profile_caps_frames_and_footprint():
    data = yaml.safe_load((ROOT / 'config' / 'nav2_robot.yaml').read_text())
    assert 'amcl' not in data and 'map_server' not in data
    smoother = data['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [0.1, 0.0, 0.25] and smoother['min_velocity'] == [-0.1, 0.0, -0.25]
    follow = data['controller_server']['ros__parameters']['FollowPath']
    assert follow['desired_linear_vel'] <= 0.1 and follow['rotate_to_heading_angular_vel'] <= 0.25
    assert data['behavior_server']['ros__parameters']['max_rotational_vel'] <= 0.25
    for name in ('local_costmap', 'global_costmap'):
        params = data[name][name]['ros__parameters']
        assert params['robot_base_frame'] == 'base_link'
        assert yaml.safe_load(params['footprint']) == [[0.31, 0.19], [0.31, -0.19], [-0.31, -0.19], [-0.31, 0.19]]
        assert 'voxel_layer' not in params['plugins']
        assert params['obstacle_layer']['scan']['topic'] == '/scan'
    assert data['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'map'
    assert data['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_subscribe_transient_local']
    assert data['local_costmap']['local_costmap']['ros__parameters']['rolling_window']
    assert data['planner_server']['ros__parameters']['GridBased']['allow_unknown']
    for node in data.values():
        params = node.get('ros__parameters') or next(iter(node.values()))['ros__parameters']
        assert params.get('use_sim_time') is False


def test_slam_and_gate_overrides():
    slam = yaml.safe_load((ROOT / 'config' / 'slam_robot.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert slam['base_frame'] == 'base_link' and slam['scan_topic'] == '/scan' and slam['mode'] == 'mapping'
    assert slam['use_sim_time'] is False
    gate = yaml.safe_load((ROOT / 'config' / 'robot_own_nav2.yaml').read_text())
    assert gate == {'velocity_gate': {'ros__parameters': {'expected_input_publishers': 2}}}
