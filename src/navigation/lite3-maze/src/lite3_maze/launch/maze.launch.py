from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    sim = value('simulation').lower() == 'true'
    share = Path(get_package_share_directory('lite3_maze'))
    config = value('config') or str(share/'config'/('simulation.yaml' if sim else 'robot.yaml'))
    nodes = []
    if not sim and value('stack').lower() == 'true':
        course = Path(get_package_share_directory('lite3_launcher'))
        # Use the inspected course includes, excluding its top-level joystick/mux.
        # This navigator is the sole /cmd_vel producer; no Nav2 goals are sent.
        for name in ('lite3_bringup', 'lite3_navigation'):
            file = course/'launch/include'/f'{name}.launch.py'
            if not file.is_file():
                raise RuntimeError('Course launcher layout differs: '+str(file)+
                    '. Start your working SLAM stack and pass stack:=false.')
            nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(file)),
                launch_arguments=dict(slam='True',use_sim_time='false',use_composition='False').items()))
    yolo = value('yolo').lower() == 'true'
    people_input = value('people_input') or ('/maze/yolo/people' if yolo else '')
    if yolo:
        nodes.append(Node(package='lite3_maze', executable='yolo',
            prefix=value('perception_python'), parameters=[dict(use_sim_time=sim,
                image_topic=value('image_topic') or ('/camera/image_raw' if sim else '/camera/color/image_raw'),
                image_transport='raw' if sim else 'compressed')], output='screen'))
    nodes += [Node(package='lite3_maze', executable='navigator', parameters=[config], output='screen'),
        Node(package='lite3_maze', executable='mode_bridge', parameters=[dict(use_sim_time=sim,
            simulation=sim, command_topic=value('mode_command_topic'), command_type=value('mode_command_type'),
            send_gait_commands=not sim)], output='screen'),
        Node(package='lite3_maze', executable='detections', parameters=[dict(use_sim_time=sim,
            qr_input_topic=value('qr_input'), people_input_topic=people_input,
            scene_input_topic=value('scene_input'), qr_output_topic=value('qr_output'),
            people_output_topic=value('people_output'), scene_output_topic=value('scene_output'))], output='screen')]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('stack', default_value='false'),
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('yolo', default_value='true'),
        DeclareLaunchArgument('perception_python', default_value=''),
        DeclareLaunchArgument('image_topic', default_value=''),
        DeclareLaunchArgument('mode_command_topic', default_value='/simple_cmd'),
        DeclareLaunchArgument('mode_command_type', default_value='transfer_interfaces/msg/MotionSimpleCMD'),
        DeclareLaunchArgument('qr_input', default_value=''),
        DeclareLaunchArgument('people_input', default_value=''),
        DeclareLaunchArgument('scene_input', default_value=''),
        DeclareLaunchArgument('qr_output', default_value='/maze/qr/detections'),
        DeclareLaunchArgument('people_output', default_value='/maze/people/detections'),
        DeclareLaunchArgument('scene_output', default_value='/maze/scene/detections'),
        OpaqueFunction(function=setup)])
