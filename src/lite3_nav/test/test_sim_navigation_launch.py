"""Offline checks of launch/sim_navigation.launch.py (no ROS graph needed)."""
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_launch():
    spec = importlib.util.spec_from_file_location('sim_navigation_launch',
                                                  ROOT / 'launch' / 'sim_navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sim_runs_the_robot_nav2_tuning_with_sim_time_and_speed():
    launch = load_launch()
    source = ROOT / 'config' / 'nav2_robot.yaml'
    out = Path(launch.sim_nav2_params(source, '/x/nav2_bt_no_backup.xml', .25))
    try:
        data = yaml.safe_load(out.read_text())
        original = yaml.safe_load(source.read_text())
        bt = data['bt_navigator']['ros__parameters']
        assert bt.pop('default_nav_to_pose_bt_xml') == bt.pop('default_nav_through_poses_bt_xml') \
            == '/x/nav2_bt_no_backup.xml'
        assert data['controller_server']['ros__parameters']['FollowPath'].pop('desired_linear_vel') == .25
        original['controller_server']['ros__parameters']['FollowPath'].pop('desired_linear_vel')
        smoother = data['velocity_smoother']['ros__parameters']
        assert smoother['max_velocity'][0] == .25 and smoother['min_velocity'][0] == -.25
        smoother['max_velocity'][0], smoother['min_velocity'][0] = .1, -.1

        def sim_time(node):
            if isinstance(node, dict):
                params = node.get('ros__parameters')
                if isinstance(params, dict):
                    assert params.pop('use_sim_time') is True
                for value in node.values():
                    sim_time(value)
        sim_time(data)

        def drop(node):
            if isinstance(node, dict):
                if isinstance(node.get('ros__parameters'), dict):
                    node['ros__parameters'].pop('use_sim_time', None)
                for value in node.values():
                    drop(value)
        drop(original)
        assert data == original  # nothing else changed
    finally:
        out.unlink()


def test_sim_speed_is_bounded():
    with pytest.raises(RuntimeError):
        load_launch().sim_nav2_params(ROOT / 'config' / 'nav2_robot.yaml', '/x.xml', .5)


def test_rpp_collision_check_can_be_switched_off_in_the_sim_only():
    launch = load_launch()
    source = ROOT / 'config' / 'nav2_robot.yaml'
    assert yaml.safe_load(source.read_text())['controller_server']['ros__parameters'][
        'FollowPath']['use_collision_detection'] is True  # the robot keeps it on
    for collision in (True, False):
        out = Path(launch.sim_nav2_params(source, '/x.xml', .25, collision))
        try:
            follow = yaml.safe_load(out.read_text())['controller_server']['ros__parameters']['FollowPath']
            assert follow['use_collision_detection'] is collision
        finally:
            out.unlink()
    with pytest.raises(RuntimeError):
        launch._flag('maybe')
    assert launch._flag(' False ') is False and launch._flag('true') is True
