"""Static checks: no ROS graph needed (rclpy must be importable: source ROS first)."""
import ast
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
# Frozen copy of the team's original robot.yaml (lite3_mission main e725625).
TEAM_ROBOT_YAML = ROOT / 'test' / 'team_robot_e725625.yaml'


def test_modules_import_and_use_own_helpers():
    import lite3_nav.core as core
    import lite3_nav.mission as mission
    import lite3_nav.velocity_gate as gate
    assert mission.select_frontier is core.select_frontier
    assert gate.VelocityLimiter is core.VelocityLimiter
    assert callable(mission.main) and callable(gate.main)


def test_no_lite3_mission_dependency():
    # Self-contained: no import, launched executable, share lookup, package dependency or
    # PYTHONPATH entry for lite3_mission. Provenance comments and the observations report
    # directory (~/.ros/lite3_mission, the team's value) may still name it.
    pattern = re.compile(r"(import|from)\s+lite3_mission|package=.lite3_mission|"
                         r"get_package_share_directory\(.lite3_mission|depend>lite3_mission|"
                         r"\.\./lite3_mission")
    files = [*ROOT.glob('lite3_nav/*.py'), *ROOT.glob('launch/*.py'), ROOT / 'package.xml',
             ROOT / 'setup.py', ROOT / 'test' / 'run_tests.sh']
    for path in files:
        assert not pattern.search(path.read_text()), path


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
    team = flatten(yaml.safe_load(TEAM_ROBOT_YAML.read_text()))
    ours = flatten(yaml.safe_load((ROOT / 'config' / 'robot.yaml').read_text()))
    # Deliberate changes to team values (course terrain, 2026-11-15): tilt stop 12 -> 22 deg.
    changed = {'mission.ros__parameters.maximum_tilt_deg': 22.0}
    assert {k: ours.get(k) for k in team} == {**team, **changed}
    assert set(ours) - set(team) == {
        'mission.ros__parameters.nav2_active_timeout_sec',
        'mission.ros__parameters.nav2_health_check',
        'mission.ros__parameters.frontier_ranking',
        'mission.ros__parameters.frontier_planner_radius_m', 'mission.ros__parameters.minimum_goal_motion_m',
        'mission.ros__parameters.minimum_goal_hop_m', 'mission.ros__parameters.tilt_hold_sec',
        'mission.ros__parameters.goal_progress_m', 'mission.ros__parameters.goal_max_sec',
        'mission.ros__parameters.explore_budget_sec', 'mission.ros__parameters.exit_when_seen',
        'mission.ros__parameters.start_routine', 'mission.ros__parameters.start_creep_m',
        'mission.ros__parameters.start_creep_speed', 'mission.ros__parameters.start_scan_deg',
        'mission.ros__parameters.auto_stand_up', 'mission.ros__parameters.stand_timeout_sec',
        'mission.ros__parameters.stand_settle_sec', 'velocity_gate.ros__parameters.simple_cmd_allowed_node',
        'velocity_gate.ros__parameters.output_transient_local',
        'velocity_gate.ros__parameters.expected_input_publishers'} | {
        'mission.ros__parameters.' + k for k in EXIT_KEYS if k != 'final_cell_xy'}


EXIT_KEYS = ('exit_detection_enabled', 'exit_open_radius', 'exit_near_radius', 'exit_min_displacement',
             'exit_enclosed_radius', 'exit_dwell_sec', 'exit_frontiers_last', 'final_cell_xy',
             'final_cell_radius')


def test_exit_defaults_match_the_node():
    import lite3_nav.mission as mission
    import inspect
    source = inspect.getsource(mission.Mission.__init__)
    ours = yaml.safe_load((ROOT / 'config' / 'robot.yaml').read_text())['mission']['ros__parameters']
    # final_cell_xy stays out of the file: an empty list there loads as None, not [].
    assert 'final_cell_xy' not in ours and ours['exit_open_radius'] > 0.45  # more than half a 0.9 m corridor
    for key in EXIT_KEYS:
        assert key + '=' in source, key


def test_sim_yaml_only_overrides_known_nodes():
    sim = yaml.safe_load((ROOT / 'config' / 'sim.yaml').read_text())
    assert set(sim) <= {'mission', 'velocity_gate', 'observations'}


def test_maze_world_matches_its_generator(tmp_path):
    # worlds/maze_dead_ends.world is generated; regenerate it rather than editing it by hand.
    import subprocess
    import sys
    out = tmp_path / 'maze.world'
    subprocess.run([sys.executable, str(ROOT / 'scripts' / 'make_maze_world.py'), '--seed', '36',
                    '--cols', '7', '--rows', '6', '--loops', '2', '-o', str(out)],
                   check=True, capture_output=True)
    assert out.read_text() == (ROOT / 'worlds' / 'maze_dead_ends.world').read_text()
    setup = (ROOT / 'setup.py').read_text()
    assert "('share/lite3_nav/worlds', glob('worlds/*.world'))" in setup
