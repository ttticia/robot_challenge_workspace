"""Run the lite3_nav mission layer in the lite3_sim Gazebo simulator.

ros2 launch lite3_nav sim_navigation.launch.py              # explore arena_1, armed manually
ros2 launch lite3_nav sim_navigation.launch.py rviz:=false world:=$(ros2 pkg prefix lite3_sim)/share/lite3_sim/worlds/arena_2.world
ros2 launch lite3_nav sim_navigation.launch.py sensors:=nav   # navigation-only, faster Gazebo
                                                   # (no camera: do not combine with team_qr)
Then: ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: true}'

nav2:=robot (default) runs Nav2 with the robot's own tuning (config/nav2_robot.yaml: Smac 2D,
RPP, corridor inflation, the short-BackUp behaviour tree) under sim time and at sim_speed (default
0.25 m/s; the robot itself is capped at 0.1), so the sim tests what will drive the robot.
rpp_collision:=false turns off the path follower's own look-ahead collision check in the sim only
(a diagnostic for "detected collision ahead" aborts; the robot always keeps it on).
nav2:=sim keeps lite3_sim's own Nav2 file (DWB, NavFn, stock tree).

Needs lite3_sim with the nav_cmd_vel_topic, sensors and nav2_params arguments.
lite3_sim's Nav2 output is remapped to /mission/nav_cmd_vel, so the mission velocity gate is
the only /cmd_vel publisher, exactly as on the robot. Domain 89, localhost only; domain 4 is
refused (it is the real robot).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import tempfile

from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _domain_guard(context):
    dom = LaunchConfiguration('domain_id').perform(context).strip()
    if dom == '4':
        raise RuntimeError('domain_id 4 is the real robot: the sim must not run on it')
    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', dom),
               SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')]
    # Same UDP-only Fast DDS transport as on the robot: with the default shared-memory
    # transport, service responses (Nav2 lifecycle get_state) are intermittently lost.
    profile = LaunchConfiguration('fastdds_profile').perform(context).strip()
    if profile:
        actions.append(SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', profile))
    return actions


PASS_SIM = ('world', 'x_pose', 'y_pose', 'gui', 'rviz', 'domain_id', 'sensors')


def sim_nav2_params(source, bt_xml, linear, collision=True):
    """Temporary copy of nav2_robot.yaml for the simulator: sim time on every node, the
    short-BackUp tree, and `linear` m/s instead of the robot's 0.1 cap. Everything else unchanged."""
    import yaml
    if not 0.05 <= linear <= 0.25:
        raise RuntimeError('sim_speed must be 0.05-0.25 m/s (the sim velocity gate caps 0.25)')
    data = yaml.safe_load(Path(source).read_text())

    def walk(node):
        if isinstance(node, dict):
            if isinstance(node.get('ros__parameters'), dict):
                node['ros__parameters']['use_sim_time'] = True
            for value in list(node.values()):
                walk(value)
    walk(data)
    bt = data['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml'] = bt['default_nav_through_poses_bt_xml'] = bt_xml
    follow = data['controller_server']['ros__parameters']['FollowPath']
    follow['desired_linear_vel'] = linear
    if not collision:
        follow['use_collision_detection'] = False
    smoother = data['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][0], smoother['min_velocity'][0] = linear, -linear
    out = tempfile.NamedTemporaryFile('w', prefix='lite3_nav_sim_nav2_', suffix='.yaml', delete=False)
    with out:
        yaml.safe_dump(data, out, sort_keys=False)
    return out.name


def _flag(text):
    value = text.strip().lower()
    if value not in ('true', 'false'):
        raise RuntimeError('rpp_collision must be true or false')
    return value == 'true'


def _sim(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    sim = Path(get_package_share_directory('lite3_sim')) / 'launch' / 'sim_nav.launch.py'
    share = Path(get_package_share_directory('lite3_nav'))
    profile = arg('nav2').strip()
    if profile not in ('robot', 'sim'):
        raise RuntimeError('nav2 must be robot or sim')
    extra = []
    if profile == 'robot':
        extra = [('nav2_params', sim_nav2_params(share / 'config' / 'nav2_robot.yaml',
                                                 str(share / 'config' / 'nav2_bt_short_backup.xml'),
                                                 float(arg('sim_speed')),
                                                 _flag(arg('rpp_collision'))))]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim)),
        launch_arguments=[(n, arg(n)) for n in PASS_SIM]
        + [('nav_cmd_vel_topic', '/mission/nav_cmd_vel')] + extra)]


def generate_launch_description():
    share = Path(get_package_share_directory('lite3_nav'))
    mission = share / 'launch' / 'mission.launch.py'
    arg = LaunchConfiguration
    declared = [DeclareLaunchArgument(name, default_value=value) for name, value in [
        ('world', str(Path(get_package_share_directory('lite3_sim')) / 'worlds' / 'arena_1.world')), ('x_pose', '0.75'),
        ('y_pose', '0.75'), ('gui', 'false'), ('rviz', 'true'), ('domain_id', '89'),
        # full, or nav = navigation-only (no camera, lighter lidar) when Gazebo is too slow
        ('sensors', 'full'),
        ('mode', 'explore'), ('execution', 'true'), ('drive', 'true'), ('team_qr', 'false'),
        ('config_file', ''), ('fastdds_profile', str(share / 'config' / 'fastdds_udp.xml')),
        ('mission_delay', '25.0'),
        # robot = nav2_robot.yaml (the robot's tuning) at sim_speed; sim = lite3_sim's own file
        ('nav2', 'robot'), ('sim_speed', '0.25'),
        # false = sim-only diagnostic: RPP's own collision check off (robot keeps it on)
        ('rpp_collision', 'true')]]
    return LaunchDescription(declared + [
        OpaqueFunction(function=_domain_guard),
        OpaqueFunction(function=_sim),
        # Mission nodes start once SLAM/Nav2 have had time to come up (sim spawn ~15 s).
        TimerAction(period=arg('mission_delay'), actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mission)),
            launch_arguments=[('profile', 'sim'), ('mode', arg('mode')),
                              ('execution', arg('execution')), ('drive', arg('drive')),
                              ('team_qr', arg('team_qr')), ('config_file', arg('config_file'))])]),
    ])
