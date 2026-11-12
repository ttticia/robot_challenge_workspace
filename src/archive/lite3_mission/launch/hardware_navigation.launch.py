"""Group-4 hardware startup; preview by default, no gait or auto-mode switches."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from lite3_mission.room_config import room_parameters


def include(context):
    root=Path(get_package_share_directory('lite3_mission'))
    values={k:LaunchConfiguration(k).perform(context) for k in
            ('slam','map','params_file','execution','drive','team_qr','mode','config_file')}
    actions=[]
    if not values['params_file']:
        course=Path(get_package_share_directory('lite3_launcher'))
        standard=Path(get_package_share_directory('nav2_bringup'))/'params/nav2_params.yaml'
        data=room_parameters(yaml.safe_load((course/'config/nav2_params.yaml').read_text()),root/'config/room_to_pose.xml')
        data['bt_navigator']['ros__parameters']['plugin_lib_names']=yaml.safe_load(standard.read_text())['bt_navigator']['ros__parameters']['plugin_lib_names']
        with tempfile.NamedTemporaryFile(mode='w',prefix='lite3-hardware-',suffix='.yaml',delete=False) as stream:
            yaml.safe_dump(data,stream,sort_keys=False)
            generated=Path(stream.name)
        values['params_file']=str(generated)
        def cleanup(context):
            generated.unlink(missing_ok=True)
            return []
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/robot_room.launch.py')),
                   launch_arguments=values.items()))
    return actions


def generate_launch_description():
    root=Path(get_package_share_directory('lite3_mission'))
    defaults={'slam':'True','map':'','params_file':'','execution':'false',
              'drive':'false','team_qr':'false','mode':'explore','config_file':''}
    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
        SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',str(root/'config/fastdds_udp.xml')),
        OpaqueFunction(function=include)])
