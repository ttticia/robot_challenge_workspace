"""
Start perception on the robot: qr_scanner (QR codes) only.

    ros2 launch team_perception robot.launch.py
    ros2 launch team_perception robot.launch.py depth_topic:=/camera/aligned_depth/image_raw \
        depth_camera_info_topic:=''        # depth already aligned by the driver
    ros2 launch team_perception robot.launch.py depth_relay:=false   # qr_scanner reads raw depth

Human / survivor poster detection (human_detector + poster_detector) was removed from this
launch on 2026-11-19: each extra camera subscriber is another full copy from
lite3_domain_bridge_4, and with them running the bridge fell behind (IMU 23 Hz instead of
200, lidar 1.1 s late, so SLAM never built a map). The node code and `ros2 run` entry points
are still in the package.

Depth goes through ONE depth_relay (depth_relay:=true): it subscribes the camera's raw depth
once and republishes it at relay_rate_hz (10) on /camera/depth_relay/*, and qr_scanner reads
that. The raw stream comes from lite3_domain_bridge_4 (another VM) and every subscribing
process costs the bridge a full 18 MB/s copy (depth_relay_node.py). maze_nav.launch.py starts
its own relay and passes depth_relay:=false with the relay topics.

Every argument has a default that matches the robot's Orbbec camera. Check the real topic
names with `ros2 topic list` before a run.

CPU (shared with navigation on the robot computer):
  - every node here gets THREAD_ENV (OMP / OpenBLAS / MKL / OpenCV pools = 1 thread):
    idle maths pools otherwise spin on all cores (system time) for each small numpy/cv2 call.
  - qr_scanner runs under `nice -n <nice>` and `taskset -c <cpus>`: only on those cores, at
    lower priority, so Fast-LIO, wall_scan and the DDS receive threads are never queued behind
    it. cpus:=auto = all but the first 4 cores; cpus:=all / nice:=0 = off. depth_relay is
    left alone (stairs/curtains need it).
  - qr_scanner READS codes everywhere (plain decodes, roam_rate_hz 5) but DETECTS
    (full scans, /qr/unread -> FACING_QR) only at dead ends, while maze_explorer's
    /maze/perceive is True (run_gate.py). Without maze_explorer it always detects.
"""
import os

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
    'map_frame': ('map', 'frame that positions are reported in'),
    'depth_relay': ('true', 'read depth through one depth_relay (one copy from the bridge); '
                            'false = qr_scanner subscribes depth_topic '
                            'themselves'),
    'relay_rate_hz': ('10.0', 'depth_relay: max depth frames a second passed on'),
    'qr': ('true', 'run qr_scanner'),
    'nice': ('10', 'perception nodes run at this nice level (0 = normal priority)'),
    'cpus': ('auto', "perception nodes only on these cores (taskset list, e.g. '4-9'); "
                     "auto = all but the first 4, all = no limit"),
}

# Maths / image thread pools: one thread each (see the module doc).
# (exec_name on the niced nodes keeps their log / death-banner label, not 'nice-N'.)
THREAD_ENV = {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1',
              'OPENCV_FOR_THREADS_NUM': '1'}


def cpu_prefix(nice, cpus):
    """'nice -n 10 taskset -c 4-9' for the perception nodes, or None when both are off."""
    have = sorted(os.sched_getaffinity(0))
    cpus = str(cpus).strip().lower()
    if cpus == 'auto':
        cpus = f'{have[4]}-{have[-1]}' if len(have) >= 6 else 'all'
    parts = []
    if str(nice).strip() not in ('', '0'):
        parts.append(f'nice -n {int(nice)}')
    if cpus not in ('', 'all'):
        parts.append(f'taskset -c {cpus}')
    return ' '.join(parts) or None


RELAY_DEPTH = '/camera/depth_relay/image_raw'
RELAY_INFO = '/camera/depth_relay/camera_info'


def relay_topics(depth_topic, info_topic):
    """(depth, info) topics the consumers should read behind a depth_relay."""
    return RELAY_DEPTH, (RELAY_INFO if info_topic else '')


def nodes(context):
    c = {name: LaunchConfiguration(name) for name in ARGS}
    v = {name: c[name].perform(context) for name in ('depth_topic', 'depth_camera_info_topic',
                                                     'depth_relay', 'relay_rate_hz', 'nice',
                                                     'cpus')}
    prefix = cpu_prefix(v['nice'], v['cpus'])
    relay = v['depth_relay'].lower() in ('true', '1')
    depth, info = v['depth_topic'], v['depth_camera_info_topic']
    out = []
    if relay:
        out.append(Node(package='team_perception', executable='depth_relay', name='depth_relay',
                        output='screen', additional_env=THREAD_ENV,
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
             exec_name='qr_scanner', output='screen', parameters=[camera], condition=IfCondition(c['qr']),
             prefix=prefix, additional_env=THREAD_ENV),
    ]
    return out


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, (default, desc) in ARGS.items()],
        OpaqueFunction(function=nodes),
    ])
