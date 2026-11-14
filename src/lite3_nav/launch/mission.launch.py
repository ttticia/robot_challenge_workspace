"""Mission nodes only. The lab stack continues to own SLAM/localization/Nav2.

Same arguments as lite3_mission/launch/mission.launch.py, plus profile:=sim. mission and
velocity_gate run from lite3_nav (same node names, topics and services); observations, qr,
scene_client and team_qr_adapter run unchanged from the team's lite3_mission package.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def nodes(context):
    root=Path(get_package_share_directory('lite3_nav'))
    team=Path(get_package_share_directory('lite3_mission'))
    profile=LaunchConfiguration('profile').perform(context)
    if profile not in ('robot','mujoco','sim'):raise ValueError('profile must be robot, mujoco or sim')
    parameters=[str(root/'config/robot.yaml')]
    if profile=='mujoco':parameters.append(str(team/'config/mujoco.yaml'))
    if profile=='sim':parameters.append(str(root/'config/sim.yaml'))
    override=LaunchConfiguration('config_file').perform(context)
    if override:parameters.append(override)
    execute=LaunchConfiguration('execution').perform(context).lower()=='true'
    drive=LaunchConfiguration('drive').perform(context).lower()=='true'
    if (LaunchConfiguration('team_qr').perform(context).lower()=='true'
            and LaunchConfiguration('qr').perform(context).lower()=='true'):
        raise ValueError('Use team_qr for the existing team scanner or qr for the standalone decoder, one at a time')
    mode=LaunchConfiguration('mode').perform(context)
    return [Node(package='lite3_nav',executable='mission',name='mission',output='screen',
                 parameters=parameters+[{'execution_enabled':execute,'mode':mode}]),
            Node(package='lite3_nav',executable='velocity_gate',name='velocity_gate',output='screen',
                 parameters=parameters+[{'drive_output_enabled':drive}]),
            Node(package='lite3_mission',executable='observations',name='observations',output='screen',parameters=parameters),
            Node(package='lite3_mission',executable='qr',name='qr',output='screen',parameters=parameters,
                 condition=IfCondition(LaunchConfiguration('qr'))),
            Node(package='lite3_mission',executable='scene_client',name='scene_client',output='screen',parameters=parameters,
                 condition=IfCondition(LaunchConfiguration('scene'))),
            Node(package='lite3_mission',executable='team_qr_adapter',name='team_qr_adapter',output='screen',
                 parameters=parameters,condition=IfCondition(LaunchConfiguration('team_qr')))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile',default_value='robot'),
        DeclareLaunchArgument('config_file',default_value=''),
        DeclareLaunchArgument('mode',default_value='waypoints'),
        DeclareLaunchArgument('execution',default_value='false'),
        DeclareLaunchArgument('drive',default_value='false'),
        DeclareLaunchArgument('qr',default_value='false'),
        DeclareLaunchArgument('team_qr',default_value='false'),
        DeclareLaunchArgument('scene',default_value='false'),
        OpaqueFunction(function=nodes)])
