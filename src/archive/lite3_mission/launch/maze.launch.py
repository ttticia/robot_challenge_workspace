"""Online maze navigation after operator lite3_auto; recording is opt-in."""
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,ExecuteProcess,RegisterEventHandler,EmitEvent,SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    root=Path(get_package_share_directory('lite3_mission'))
    value=LaunchConfiguration('run').perform(context).lower()
    if value not in ('true','false'):raise ValueError('run must be true or false')
    run=value=='true'
    record_value=LaunchConfiguration('record').perform(context).lower()
    if record_value not in ('true','false'):raise ValueError('record must be true or false')
    record=record_value=='true'
    folder=Path(LaunchConfiguration('output').perform(context)).expanduser();folder.mkdir(parents=True,exist_ok=True)
    model=str(Path(LaunchConfiguration('people_model').perform(context)).expanduser())
    if not Path(model).is_file():raise FileNotFoundError('Download the people model first: '+model)
    coordinator=Node(package='lite3_mission',executable='maze_session',output='screen',parameters=[dict(
        autostart=run,run_directory=str(folder),maximum_run_sec=float(LaunchConfiguration('maximum_run_sec').perform(context)),
        no_progress_timeout_sec=30.)])
    scanner=Node(package='team_perception',executable='qr_scanner',output='screen',parameters=[dict(
        image_transport='compressed',use_depth=False,rate_hz=5.,publish_debug=False,log_dir=str(folder/'qr_evidence'))])
    people=Node(package='team_perception',executable='yolo_node',output='screen',parameters=[dict(
        model_path=model,image_transport='compressed',max_inference_hz=2.,publish_debug=False)])
    topics=['/map','/map_updates','/tf','/tf_static','/scan','/odom','/imu/data',
            '/camera/color/image_raw/compressed','/camera/color/camera_info','/qr/detections',
            '/people/detections','/mission/observations','/mission/report','/mission/status',
            '/mission/gate_status','/mission/exit','/mission/session','/cmd_vel']
    required=[(coordinator,'Maze session finished'),(scanner,'QR scanner stopped'),
              (people,'People detector stopped')]
    recorders=[]
    if record:
        bag=ExecuteProcess(cmd=['ros2','bag','record','-o',str(folder/'slam_qr_people'),
                               '--max-bag-size','268435456',*topics],output='screen')
        required.append((bag,'Recorder stopped'));recorders.append(bag)
    stops=[RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[EmitEvent(event=Shutdown(reason=reason))]))
           for node,reason in required]
    return [*stops,*recorders,scanner,people,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/hardware_navigation.launch.py')),
            launch_arguments=dict(slam='True',mode='explore',execution=str(run).lower(),drive=str(run).lower(),
                team_qr='true',config_file=str(root/'config/maze.yaml')).items()),coordinator]


def generate_launch_description():
    root=Path(get_package_share_directory('lite3_mission'))
    stamp=datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S')
    return LaunchDescription([
        DeclareLaunchArgument('run',default_value='true',description='Prepare H-Step, wait for operator lite3_auto, then arm exploration'),
        DeclareLaunchArgument('record',default_value='false',description='Optional rosbag; disabled by default'),
        DeclareLaunchArgument('output',default_value=str(Path.home()/'challenge_ws/runs'/('maze_'+stamp+'_SGT'))),
        DeclareLaunchArgument('people_model',default_value=str(Path.home()/'challenge_ws/models/yolov5n.onnx')),
        DeclareLaunchArgument('maximum_run_sec',default_value='600'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',str(root/'config/fastdds_udp.xml')),
        OpaqueFunction(function=start)])
