"""Installed lab SLAM/Nav2 with the final mux output routed through our gate.

Includes lab base/navigation components without the lab's unguarded mux or
joystick launcher. Course source/configuration files are never changed.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(context):
    course = Path(get_package_share_directory('lite3_launcher'))
    mission = Path(get_package_share_directory('lite3_mission'))
    values = {key: LaunchConfiguration(key).perform(context)
              for key in ('slam', 'map', 'params_file')}
    if not values['map']:
        values.pop('map')
    if not values['params_file']:
        values.pop('params_file')
    values.update(use_sim_time='false', use_composition='False')
    return [
        # Fast-LIO also uses a launch argument named config_file. Keep its
        # sensor profile separate from the mission override of the same name.
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            course / 'launch/include/lite3_bringup.launch.py')),
            launch_arguments={'config_file':'mid360.yaml',
                              'config_path':str(course/'config')}.items())]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            course / 'launch/include/lite3_navigation.launch.py')), launch_arguments=values.items()),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             parameters=[str(course / 'config/twist_mux_topics.yaml')],
             remappings=[('cmd_vel_out', '/mission/nav_cmd_vel')], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            mission / 'launch/mission.launch.py')), launch_arguments={
                'execution': LaunchConfiguration('execution').perform(context),
                'drive': LaunchConfiguration('drive').perform(context),
                'team_qr': LaunchConfiguration('team_qr').perform(context),
                'qr': 'false', 'mode': LaunchConfiguration('mode').perform(context),
                'config_file': (LaunchConfiguration('config_file').perform(context)
                                or str(mission / 'config/room_trial.yaml'))}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('execution', default_value='false'),
        DeclareLaunchArgument('drive', default_value='false'),
        DeclareLaunchArgument('team_qr', default_value='false'),
        DeclareLaunchArgument('mode', default_value='waypoints'),
        DeclareLaunchArgument('config_file', default_value=''),
        OpaqueFunction(function=include)])
