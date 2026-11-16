"""
Start the whole perception pipeline on the robot: qr_scanner, human_detector, poster_detector.

    ros2 launch team_perception robot.launch.py
    ros2 launch team_perception robot.launch.py depth_topic:=/camera/aligned_depth/image_raw \
        depth_camera_info_topic:=''        # depth already aligned by the driver
    ros2 launch team_perception robot.launch.py posters:=false   # QR scanner only
    ros2 launch team_perception robot.launch.py offboard:=true   # YOLO on an external computer

YOLO person detection (human_detector) runs here on the robot's CPU, reading the camera's
JPEG stream directly, capped to yolo_threads torch threads and yolo_rate_hz inferences a
second so it leaves the CPU to navigation. It publishes /humans/detections, which
poster_detector judges against the depth image.

offboard:=true is the old split: human_detector is NOT started; image_throttle sends a
reduced-rate JPEG stream to an external computer running
`ros2 launch team_perception_external external.launch.py` (Robot-Dog-Grp4 repo).

Every argument has a default that matches the robot's Orbbec camera. Check the real topic
names with `ros2 topic list` before a run. poster_detector needs depth ALIGNED to the
colour image or depth_camera_info_topic (the Lite3's depth is NOT aligned; the nodes then
map colour pixels into it, a temporary fix), localisation (TF from the camera to map_frame)
and the static map.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ARGS = {
    'image_topic': ('/camera/color/image_raw', 'colour image topic (raw name)'),
    'image_transport': ('compressed', "'raw', or 'compressed' = <image_topic>/compressed"),
    'depth_topic': ('/camera/depth/image_raw', 'depth image (aligned to colour or not)'),
    'depth_camera_info_topic': ('/camera/depth/camera_info',
                                "depth intrinsics, to map colour pixels into unaligned depth; "
                                "'' = depth_topic is already aligned to colour"),
    'camera_info_topic': ('/camera/color/camera_info', 'colour camera intrinsics'),
    'use_depth': ('true', 'place QR codes in the map using depth'),
    'map_topic': ('/map', 'static map (nav_msgs/OccupancyGrid, transient local)'),
    'map_frame': ('map', 'frame that positions are reported in'),
    'map_mode': ('veto', "poster_detector map test: 'require', 'veto' or 'off'"),
    'yolo_rate_hz': ('5.0', 'human_detector: max YOLO inferences per second'),
    'yolo_threads': ('2', 'human_detector: torch CPU threads (0 = torch default, ~8 cores)'),
    'yolo_imgsz': ('640', 'human_detector: YOLO input size (smaller = faster, misses far '
                          'people)'),
    'yolo_conf': ('0.4', 'human_detector: minimum person confidence'),
    'yolo_model': ('yolo11n.pt', 'human_detector: weights, bare name -> '
                                 '~/.cache/team_perception/, or a path'),
    'offboard': ('false', 'true = YOLO on an external computer: run image_throttle instead '
                          'of human_detector'),
    'offboard_topic': ('/camera/color/image_offboard/compressed',
                       'offboard: throttled JPEG stream for the external computer'),
    'offboard_rate_hz': ('10.0', 'offboard: frames per second sent to the external computer'),
    'qr': ('true', 'run qr_scanner'),
    'posters': ('true', 'run human_detector (or image_throttle) + poster_detector'),
}


def generate_launch_description():
    c = {name: LaunchConfiguration(name) for name in ARGS}
    camera = {'image_topic': c['image_topic'], 'image_transport': c['image_transport'],
              'depth_topic': c['depth_topic'], 'camera_info_topic': c['camera_info_topic'],
              'depth_camera_info_topic': c['depth_camera_info_topic'],
              'use_depth': c['use_depth'], 'map_frame': c['map_frame']}

    def posters_with(offboard):
        """IfCondition: posters:=true and offboard:=<offboard>."""
        return IfCondition(PythonExpression([
            "'", c['posters'], "'.lower() in ('true', '1') and ",
            '' if offboard else 'not ', "'", c['offboard'], "'.lower() in ('true', '1')"]))

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, (default, desc) in ARGS.items()],
        Node(package='team_perception', executable='qr_scanner', name='qr_scanner',
             output='screen', parameters=[camera], condition=IfCondition(c['qr'])),
        Node(package='team_perception', executable='image_throttle', name='image_throttle',
             output='screen',
             parameters=[{'in_topic': [c['image_topic'], '/compressed'],
                          'out_topic': c['offboard_topic'],
                          'rate_hz': c['offboard_rate_hz']}],
             condition=posters_with(offboard=True)),
        Node(package='team_perception', executable='human_detector', name='human_detector',
             output='screen',
             parameters=[{'image_topic': c['image_topic'],
                          'image_transport': c['image_transport'],
                          'use_depth': False,      # poster_detector places the posters
                          'model': c['yolo_model'], 'conf': c['yolo_conf'],
                          'imgsz': c['yolo_imgsz'], 'rate_hz': c['yolo_rate_hz'],
                          'threads': c['yolo_threads']}],
             condition=posters_with(offboard=False)),
        Node(package='team_perception', executable='poster_detector', name='poster_detector',
             output='screen',
             parameters=[{'depth_topic': c['depth_topic'],
                          'depth_camera_info_topic': c['depth_camera_info_topic'],
                          'camera_info_topic': c['camera_info_topic'],
                          'map_topic': c['map_topic'], 'map_frame': c['map_frame'],
                          'map_mode': c['map_mode']}],
             condition=IfCondition(c['posters'])),
    ])
