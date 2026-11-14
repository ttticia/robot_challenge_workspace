"""Run the lite3_nav mission layer (plus the team's unchanged lite3_mission nodes) in the lite3_sim Gazebo simulator.

ros2 launch lite3_nav sim_navigation.launch.py              # explore arena_1, armed manually
ros2 launch lite3_nav sim_navigation.launch.py rviz:=false world:=$HOME/arena/arena_2.world
ros2 launch lite3_nav sim_navigation.launch.py sensors:=nav   # navigation-only, faster Gazebo
                                                   # (no camera: do not combine with team_qr)
Then: ros2 service call /mission/set_armed std_srvs/srv/SetBool '{data: true}'

Needs Rey's local lite3_sim (nav_cmd_vel_topic and sensors arguments; not in git).
lite3_sim's Nav2 output is remapped to /mission/nav_cmd_vel, so the mission velocity gate is
the only /cmd_vel publisher, exactly as on the robot. Domain 89, localhost only; domain 4 is
refused (it is the real robot).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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


def generate_launch_description():
    sim = Path(get_package_share_directory('lite3_sim')) / 'launch' / 'sim_nav.launch.py'
    share = Path(get_package_share_directory('lite3_nav'))
    mission = share / 'launch' / 'mission.launch.py'
    arg = LaunchConfiguration
    pass_sim = ('world', 'x_pose', 'y_pose', 'gui', 'rviz', 'domain_id', 'sensors')
    declared = [DeclareLaunchArgument(name, default_value=value) for name, value in [
        ('world', str(Path.home() / 'arena' / 'arena_1.world')), ('x_pose', '0.75'),
        ('y_pose', '0.75'), ('gui', 'false'), ('rviz', 'true'), ('domain_id', '89'),
        # full, or nav = navigation-only (no camera, lighter lidar) when Gazebo is too slow
        ('sensors', 'full'),
        ('mode', 'explore'), ('execution', 'true'), ('drive', 'true'), ('team_qr', 'false'),
        ('config_file', ''), ('fastdds_profile', str(share / 'config' / 'fastdds_udp.xml')),
        ('mission_delay', '25.0')]]
    return LaunchDescription(declared + [
        OpaqueFunction(function=_domain_guard),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim)),
            launch_arguments=[(n, arg(n)) for n in pass_sim]
            + [('nav_cmd_vel_topic', '/mission/nav_cmd_vel')]),
        # Mission nodes start once SLAM/Nav2 have had time to come up (sim spawn ~15 s).
        TimerAction(period=arg('mission_delay'), actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mission)),
            launch_arguments=[('profile', 'sim'), ('mode', arg('mode')),
                              ('execution', arg('execution')), ('drive', arg('drive')),
                              ('team_qr', arg('team_qr')), ('config_file', arg('config_file'))])]),
    ])
