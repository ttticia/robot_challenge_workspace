"""
Start the whole perception pipeline on the robot: qr_scanner, human_detector, poster_detector.

    ros2 launch team_perception robot.launch.py
    ros2 launch team_perception robot.launch.py depth_topic:=/camera/aligned_depth/image_raw \
        depth_camera_info_topic:=''        # depth already aligned by the driver
    ros2 launch team_perception robot.launch.py posters:=false   # QR scanner only
    ros2 launch team_perception robot.launch.py depth_relay:=false   # each node reads raw depth

Depth goes through ONE depth_relay (depth_relay:=true): it subscribes the camera's raw depth
once and republishes it at relay_rate_hz (10) on /camera/depth_relay/*, and qr_scanner /
poster_detector read that. The raw stream comes from lite3_domain_bridge_4 (another VM) and
every subscribing process costs the bridge a full 18 MB/s copy (depth_relay_node.py).
maze_nav.launch.py starts its own relay and passes depth_relay:=false with the relay topics.

YOLO person detection (human_detector) runs here on the robot's CPU, reading the camera's
JPEG stream directly, capped to yolo_threads torch threads and yolo_rate_hz inferences a
second so it leaves the CPU to navigation. It publishes /humans/detections, which
poster_detector judges against the depth image.

Every argument has a default that matches the robot's Orbbec camera. Check the real topic
names with `ros2 topic list` before a run. poster_detector needs depth ALIGNED to the
colour image or depth_camera_info_topic (the Lite3's depth is NOT aligned; the nodes then
map colour pixels into it, a temporary fix), localisation (TF from the camera to map_frame)
and the static map.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
    'depth_relay': ('true', 'read depth through one depth_relay (one copy from the bridge); '
                            'false = qr_scanner and poster_detector subscribe depth_topic '
                            'themselves'),
    'relay_rate_hz': ('10.0', 'depth_relay: max depth frames a second passed on'),
    'qr': ('true', 'run qr_scanner'),
    'posters': ('true', 'run human_detector + poster_detector'),
}


RELAY_DEPTH = '/camera/depth_relay/image_raw'
RELAY_INFO = '/camera/depth_relay/camera_info'


def relay_topics(depth_topic, info_topic):
    """(depth, info) topics the consumers should read behind a depth_relay."""
    return RELAY_DEPTH, (RELAY_INFO if info_topic else '')


def nodes(context):
    c = {name: LaunchConfiguration(name) for name in ARGS}
    v = {name: c[name].perform(context) for name in ('depth_topic', 'depth_camera_info_topic',
                                                     'depth_relay', 'relay_rate_hz')}
    relay = v['depth_relay'].lower() in ('true', '1')
    depth, info = v['depth_topic'], v['depth_camera_info_topic']
    out = []
    if relay:
        out.append(Node(package='team_perception', executable='depth_relay', name='depth_relay',
                        output='screen',
                        parameters=[{'in_depth_topic': depth, 'in_info_topic': info,
                                     'out_depth_topic': RELAY_DEPTH,
                                     'out_info_topic': RELAY_INFO,
                                     'rate_hz': float(v['relay_rate_hz'])}]))
        depth, info = relay_topics(depth, info)
    camera = {'image_topic': c['image_topic'], 'image_transport': c['image_transport'],
              'depth_topic': depth, 'camera_info_topic': c['camera_info_topic'],
              'depth_camera_info_topic': info,
              'use_depth': c['use_depth'], 'map_frame': c['map_frame']}
    out += [
        Node(package='team_perception', executable='qr_scanner', name='qr_scanner',
             output='screen', parameters=[camera], condition=IfCondition(c['qr'])),
        Node(package='team_perception', executable='human_detector', name='human_detector',
             output='screen',
             parameters=[{'image_topic': c['image_topic'],
                          'image_transport': c['image_transport'],
                          'use_depth': False,      # poster_detector places the posters
                          'model': c['yolo_model'], 'conf': c['yolo_conf'],
                          'imgsz': c['yolo_imgsz'], 'rate_hz': c['yolo_rate_hz'],
                          'threads': c['yolo_threads']}],
             condition=IfCondition(c['posters'])),
        Node(package='team_perception', executable='poster_detector', name='poster_detector',
             output='screen',
             parameters=[{'depth_topic': depth,
                          'depth_camera_info_topic': info,
                          'camera_info_topic': c['camera_info_topic'],
                          'map_topic': c['map_topic'], 'map_frame': c['map_frame'],
                          'map_mode': c['map_mode']}],
             condition=IfCondition(c['posters'])),
    ]
    return out


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, (default, desc) in ARGS.items()],
        OpaqueFunction(function=nodes),
    ])
