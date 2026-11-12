"""Optional wrapper for the installed course launcher; does not edit its files.

Final output ownership is checked by the velocity gate before arming. A lab
stack with extra remaps/smoothers may require selecting its final output topic.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def include(context):
    file=Path(get_package_share_directory('lite3_launcher'))/'launch/lite3.launch.py'
    values={'slam':LaunchConfiguration('slam').perform(context)}
    map_file=LaunchConfiguration('map').perform(context)
    if map_file:values['map']=map_file
    # Scope this remap to the lab launch; the mission velocity gate remains outside.
    return [GroupAction([
        SetRemap(src='/cmd_vel',dst='/mission/nav_cmd_vel'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(file)),launch_arguments=values.items())])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('slam',default_value='True'),
                              DeclareLaunchArgument('map',default_value=''),OpaqueFunction(function=include)])
