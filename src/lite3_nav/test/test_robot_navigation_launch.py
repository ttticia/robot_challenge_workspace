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
        if name == 'local_costmap':  # the global one plans with the turning circle (robot_radius)
            assert yaml.safe_load(params['footprint']) == FOOTPRINT
        assert 'voxel_layer' not in params['plugins']
        assert params['obstacle_layer']['scan']['topic'] == '/scan'
    assert data['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'map'
    assert data['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_subscribe_transient_local']
    assert data['local_costmap']['local_costmap']['ros__parameters']['rolling_window']
    assert data['planner_server']['ros__parameters']['GridBased']['allow_unknown']
    for node in data.values():
        params = node.get('ros__parameters') or next(iter(node.values()))['ros__parameters']
        assert params.get('use_sim_time') is False


CORRIDOR = 0.9
# Lite3 body 0.548 x 0.370 m: lab Nav2 length 0.61 m, 1.5 cm a side for the feet.
FOOTPRINT = [[0.305, 0.2], [0.305, -0.2], [-0.305, -0.2], [-0.305, 0.2]]


def test_nav2_leaves_a_free_lane_in_a_0p9_m_corridor():
    import math
    data = yaml.safe_load((ROOT / 'config' / 'nav2_robot.yaml').read_text())
    inscribed = min(max(abs(x) for x, _ in FOOTPRINT), max(abs(y) for _, y in FOOTPRINT))
    circumscribed = max(math.hypot(x, y) for x, y in FOOTPRINT)
    assert circumscribed < CORRIDOR / 2 - 0.05   # turning on the spot near the centre fits
    length = max(x for x, _ in FOOTPRINT) - min(x for x, _ in FOOTPRINT)
    width = max(y for _, y in FOOTPRINT) - min(y for _, y in FOOTPRINT)
    assert (length + width) / math.sqrt(2) < CORRIDOR - 0.15  # diagonal at a junction leaves room
    local = data['local_costmap']['local_costmap']['ros__parameters']
    inflation = local['inflation_layer']
    # Local (RPP collision check): the real rectangle, 0.3 m / 6 as found in the sim.
    assert inscribed < inflation['inflation_radius'] <= 0.35
    assert inflation['cost_scaling_factor'] >= 5.0
    assert CORRIDOR - 2 * inscribed >= 0.4                       # plannable (non-lethal) lane
    assert CORRIDOR - 2 * inflation['inflation_radius'] >= 0.25  # zero-cost middle
    # Global (planner): the turning circle, so plans keep the centre where it can turn on the spot.
    glob = data['global_costmap']['global_costmap']['ros__parameters']
    assert 'footprint' not in glob
    radius = glob['robot_radius']
    assert circumscribed - 0.02 <= radius <= circumscribed
    assert CORRIDOR - 2 * radius >= 0.18                         # plannable lane on the centre line
    assert radius < glob['inflation_layer']['inflation_radius'] <= CORRIDOR / 2


def test_nav2_planner_controller_follow_maze_nav():
    import math
    data = yaml.safe_load((ROOT / 'config' / 'nav2_robot.yaml').read_text())
    planner = data['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_smac_planner/SmacPlanner2D' and planner['cost_travel_multiplier'] > 1
    glob = data['global_costmap']['global_costmap']['ros__parameters']
    # Smac (Humble) must never plan from outside the costmap: rolling window centred on the robot.
    assert glob['rolling_window'] and glob['width'] >= 16 and glob['height'] >= 16
    ctrl = data['controller_server']['ros__parameters']
    follow = ctrl['FollowPath']
    assert follow['use_rotate_to_heading'] and follow['rotate_to_heading_min_angle'] <= 0.6
    assert follow['lookahead_dist'] <= 0.4 and not follow['allow_reversing']
    progress = ctrl['progress_checker']
    # A 180 deg turn at the 0.25 rad/s cap must not count as "no progress".
    assert progress['movement_time_allowance'] > math.pi / follow['rotate_to_heading_angular_vel'] + 5


def test_slam_and_gate_overrides():
    slam = yaml.safe_load((ROOT / 'config' / 'slam_robot.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert slam['base_frame'] == 'base_link' and slam['scan_topic'] == '/scan' and slam['mode'] == 'mapping'
    assert slam['use_sim_time'] is False
    gate = yaml.safe_load((ROOT / 'config' / 'robot_own_nav2.yaml').read_text())
    assert gate == {'velocity_gate': {'ros__parameters': {'expected_input_publishers': 2}}}


def test_wall_scan_feeds_slam_only_when_enabled(monkeypatch):
    launch = load_launch()
    assert launch.slam_overrides(True) == {'use_sim_time': False, 'scan_topic': '/wall_scan'}
    assert launch.slam_overrides(False) == {'use_sim_time': False}
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda name: str(ROOT))  # not installed here
    args = {a.name: a.default_value[0].text for a in launch.generate_launch_description().entities
            if type(a).__name__ == 'DeclareLaunchArgument'}
    assert args['wall_scan'] == 'true' and args['costmap_scan_topic'] == '/scan'
    assert args['bt_xml'].endswith('config/nav2_bt_short_backup.xml')
    assert (args['band_low'], args['band_high'], args['floor_z']) == ('0.30', '0.43', 'nan')


def test_backup_is_only_a_short_last_recovery():
    import xml.etree.ElementTree as ET
    data = yaml.safe_load((ROOT / 'config' / 'nav2_robot.yaml').read_text())
    behavior = data['behavior_server']['ros__parameters']
    assert behavior['behavior_plugins'] == ['spin', 'backup', 'drive_on_heading', 'wait']
    assert behavior['backup']['plugin'] == 'nav2_behaviors/BackUp'
    assert behavior['drive_on_heading']['plugin'] == 'nav2_behaviors/DriveOnHeading'  # start creep
    tree = ET.parse(ROOT / 'config' / 'nav2_bt_short_backup.xml')
    tags = {element.tag for element in tree.iter()}
    assert {'Spin', 'Wait', 'BackUp', 'ComputePathToPose', 'FollowPath'} <= tags
    backups = list(tree.iter('BackUp'))
    assert len(backups) == 1 and float(backups[0].get('backup_dist')) <= 0.15
    assert float(backups[0].get('backup_speed')) <= 0.05
    robin = next(tree.iter('RoundRobin'))
    assert list(robin)[-1].tag == 'BackUp'  # last resort, after clearing, spin and wait
    # The old tree stays available (bt_xml:=.../nav2_bt_no_backup.xml) and still has none.
    old = ET.parse(ROOT / 'config' / 'nav2_bt_no_backup.xml')
    assert 'BackUp' not in {element.tag for element in old.iter()}


def test_bt_xml_rewrite():
    launch = load_launch()
    source = str(ROOT / 'config' / 'nav2_robot.yaml')
    out = launch.nav2_params_for_scan(source, '/scan', '/x/nav2_bt_no_backup.xml')
    try:
        data = yaml.safe_load(Path(out).read_text())
        original = yaml.safe_load(Path(source).read_text())
        bt = data['bt_navigator']['ros__parameters']
        assert bt.pop('default_nav_to_pose_bt_xml') == bt.pop('default_nav_through_poses_bt_xml') \
            == '/x/nav2_bt_no_backup.xml'
        assert data == original  # the scan topic stays /scan
    finally:
        Path(out).unlink()


def test_costmap_scan_topic_rewrite():
    launch = load_launch()
    source = str(ROOT / 'config' / 'nav2_robot.yaml')
    assert launch.nav2_params_for_scan(source, '/scan') == source
    out = launch.nav2_params_for_scan(source, '/wall_scan')
    try:
        data = yaml.safe_load(Path(out).read_text())
        original = yaml.safe_load(Path(source).read_text())
        for name in ('local_costmap', 'global_costmap'):
            assert data[name][name]['ros__parameters']['obstacle_layer']['scan']['topic'] == '/wall_scan'
            original[name][name]['ros__parameters']['obstacle_layer']['scan']['topic'] = '/wall_scan'
        assert data == original
    finally:
        Path(out).unlink()
