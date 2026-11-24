"""
Start perception on the robot: qr_scanner, READING QR codes only (no detection, no posters).

    ros2 launch team_perception robot.launch.py
    ros2 launch team_perception robot.launch.py depth_topic:=/camera/aligned_depth/image_raw \
        depth_camera_info_topic:=''        # depth already aligned by the driver
    ros2 launch team_perception robot.launch.py depth_relay:=false   # qr_scanner reads raw depth

Survivor posters (human_detector + poster_detector): NOT started (removed again 2026-11-23,
user). The nodes and `ros2 run` entry points stay in the package; poster_detector now follows
maze_explorer's /maze/grid and places a poster in the room cell on the camera's side of its
wall (poster_utils.MazeCells.room_cell) should it come back.
QR codes (user 2026-11-23): READ only (qr_detect:=false -> qr_scanner detect false): plain
decodes at roam_rate_hz while the run is under way; no /qr/unread / FACING_QR, no full scans.
qr_detect:=true = the old dead-end detection.
Colour: ONE colour_relay (depth_relay_node with compressed:=true) takes the JPEG stream once;
qr_scanner (and maze_nav's room_scene) read its local copy
(/camera/color_relay/image_raw/compressed, <= colour_rate_hz). colour_relay:=false =
qr_scanner reads image_topic itself.

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
  - qr_scanner only READS codes (plain decodes, roam_rate_hz 5) while the run is under way;
    with qr_detect:=true it also DETECTS (full scans, /qr/unread -> FACING_QR) at dead ends,
    while maze_explorer's /maze/perceive is True (run_gate.py).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ros2 launch writes each node's parameters to a temporary file. Keep those in the workspace
# too (user, 2026-11-24: no files outside ~/challenge_ws at run time); the daily log clean-up
# removes them after a week.
import tempfile  # noqa: E402
_TMP = os.path.expanduser('~/challenge_ws/logs/tmp')
os.makedirs(_TMP, exist_ok=True)
tempfile.tempdir = _TMP
os.environ['TMPDIR'] = _TMP

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
    'qr_detect': ('false', 'QR detection (full scans + /qr/unread for FACING_QR); false = '
                           'only read codes'),
    'colour_relay': ('true', 'take the compressed colour stream once (colour_relay) for '
                             'qr_scanner (and room_scene)'),
    'colour_rate_hz': ('10.0', 'colour_relay: max colour frames a second passed on'),
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


RELAY_COLOUR = '/camera/color_relay/image_raw'      # + /compressed
RELAY_DEPTH = '/camera/depth_relay/image_raw'
RELAY_INFO = '/camera/depth_relay/camera_info'


def relay_topics(depth_topic, info_topic):
    """(depth, info) topics the consumers should read behind a depth_relay."""
    return RELAY_DEPTH, (RELAY_INFO if info_topic else '')


def nodes(context):
    c = {name: LaunchConfiguration(name) for name in ARGS}
    v = {name: c[name].perform(context) for name in ('depth_topic', 'depth_camera_info_topic',
                                                     'depth_relay', 'relay_rate_hz', 'nice',
                                                     'cpus', 'image_topic', 'image_transport',
                                                     'colour_relay', 'colour_rate_hz',
                                                     'qr_detect')}
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
    image_topic = v['image_topic']
    if v['colour_relay'].lower() in ('true', '1') and v['image_transport'] == 'compressed':
        out.append(Node(package='team_perception', executable='depth_relay', name='colour_relay',
                        output='screen', additional_env=THREAD_ENV,
                        parameters=[{'in_depth_topic': image_topic.rstrip('/') + '/compressed',
                                     'in_info_topic': '',
                                     'out_depth_topic': RELAY_COLOUR + '/compressed',
                                     'out_info_topic': '', 'compressed': True,
                                     'rate_hz': float(v['colour_rate_hz'])}]))
        image_topic = RELAY_COLOUR
    camera = {'image_topic': image_topic, 'image_transport': c['image_transport'],
              'depth_topic': depth, 'camera_info_topic': c['camera_info_topic'],
              'depth_camera_info_topic': info,
              'use_depth': c['use_depth'], 'map_frame': c['map_frame']}
    qr_params = dict(camera, detect=v['qr_detect'].lower() in ('true', '1'))
    out += [
        Node(package='team_perception', executable='qr_scanner', name='qr_scanner',
             exec_name='qr_scanner', output='screen', parameters=[qr_params], condition=IfCondition(c['qr']),
             prefix=prefix, additional_env=THREAD_ENV),
    ]
    return out


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, (default, desc) in ARGS.items()],
        OpaqueFunction(function=nodes),
    ])
