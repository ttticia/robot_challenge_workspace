"""Static checks: no ROS graph needed (rclpy must be importable: source ROS first)."""
import ast
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
TEAM = ROOT.parent / 'lite3_mission'


def test_modules_import_and_use_team_helpers():
    import lite3_mission.core as core
    import lite3_nav.mission as mission
    import lite3_nav.velocity_gate as gate
    assert mission.select_frontier is core.select_frontier
    assert gate.VelocityLimiter is core.VelocityLimiter
    assert callable(mission.main) and callable(gate.main)


def test_no_relative_imports():
    for name in ('mission.py', 'velocity_gate.py'):
        tree = ast.parse((ROOT / 'lite3_nav' / name).read_text())
        assert not [n for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level], name


def flatten(data, prefix=''):
    out = {}
    for key, value in data.items():
        if isinstance(value, dict):
            out.update(flatten(value, prefix + key + '.'))
        else:
            out[prefix + key] = value
    return out


def test_robot_yaml_keeps_every_team_value():
    team = flatten(yaml.safe_load((TEAM / 'config' / 'robot.yaml').read_text()))
    ours = flatten(yaml.safe_load((ROOT / 'config' / 'robot.yaml').read_text()))
    assert {k: ours.get(k) for k in team} == team
    assert set(ours) - set(team) == {
        'mission.ros__parameters.nav2_active_timeout_sec',
        'mission.ros__parameters.nav2_health_check',
        'velocity_gate.ros__parameters.output_transient_local',
        'velocity_gate.ros__parameters.expected_input_publishers'}


def test_sim_yaml_only_overrides_known_nodes():
    sim = yaml.safe_load((ROOT / 'config' / 'sim.yaml').read_text())
    assert set(sim) <= {'mission', 'velocity_gate', 'observations'}
