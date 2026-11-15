#!/usr/bin/env python3
"""Generate the sim URDFs from the DeepRoboticsLab Lite3 URDF, matching the real robot's interface.

Usage: make_sim_urdf.py <path/to/Lite3.urdf> <out_dir>
Writes <out_dir>/lite3_sim.urdf (camera 320x240 @ 5 Hz), lite3_sim_full.urdf (640x480 @ 10 Hz)
and lite3_sim_nav.urdf (navigation-only: no camera sensor, lighter lidar, IMU 100 Hz).

Frames / topics follow the real robot (robot_info.md):
- root link base_link (= body centre, ~0.34 m above ground); odom -> base_link from planar_move
- livox_frame (15 deg nose-down) -> /livox/points (PointCloud2)
- camera_link (20 deg nose-down) -> camera_{color,depth}_frame -> *_optical_frame
  /camera/color/image_raw (+ /compressed), /camera/color/camera_info,
  /camera/color/depth/image_m (float metres; depth_to_mm.py republishes /camera/depth/image_raw 16UC1 mm),
  /camera/depth/points
- /imu/data in base_link
Sim-only simplifications: legs frozen in a standing pose, body slides (planar_move),
leg collisions removed, frictionless foot spheres + one body box.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

SRC, OUT_DIR = sys.argv[1], sys.argv[2]

HIP_Y_Q = -0.7          # standing pose (joint values, rad); source axes are (0,-1,0) so pitch = -q
KNEE_Q = 1.36
THIGH_LEN, SHANK_LEN, FOOT_R = 0.20, 0.21012, 0.022
BASE_Z = THIGH_LEN * math.cos(-HIP_Y_Q) + SHANK_LEN * math.cos(-HIP_Y_Q - KNEE_Q) + FOOT_R
OPT_RPY = f'{-math.pi/2:.6f} 0 {-math.pi/2:.6f}'
RENAME = {'TORSO': 'base_link', 'MID360_LINK': 'livox_frame', 'D435I_LINK': 'camera_link'}


def build(cam_w, cam_h, cam_hz, camera=True, lidar_vertical=16, imu_hz=200):
    """camera=False drops only the depth/RGB sensor (its links and frames stay, so TF is unchanged)."""
    tree = ET.parse(SRC)
    robot = tree.getroot()
    robot.set('name', 'lite3')
    for m in robot.findall('mujoco'):
        robot.remove(m)

    for mesh in robot.iter('mesh'):
        mesh.set('filename', 'package://lite3_sim/meshes/' + mesh.get('filename').split('/')[-1])

    # real-robot frame names
    for link in robot.findall('link'):
        if link.get('name') in RENAME:
            link.set('name', RENAME[link.get('name')])
    for j in robot.findall('joint'):
        for tag in ('parent', 'child'):
            e = j.find(tag)
            if e is not None and e.get('link') in RENAME:
                e.set('link', RENAME[e.get('link')])

    # IMU frame on the real robot is base_link itself
    for link in robot.findall("link[@name='IMU_LINK']"):
        robot.remove(link)
    for j in robot.findall('joint'):
        if j.find('child') is not None and j.find('child').get('link') == 'IMU_LINK':
            robot.remove(j)

    # collisions: keep foot spheres (frictionless) + one body box; feet carry the robot (gravity on)
    for link in robot.findall('link'):
        if link.get('name', '').endswith('_FOOT'):
            continue
        for c in link.findall('collision'):
            link.remove(c)
    base = robot.find("link[@name='base_link']")
    col = ET.SubElement(base, 'collision')
    ET.SubElement(col, 'origin', xyz='0 0 0.01', rpy='0 0 0')
    ET.SubElement(ET.SubElement(col, 'geometry'), 'box', size='0.5 0.2 0.12')

    # freeze joints in the standing pose
    for j in robot.findall('joint'):
        if j.get('type') in ('revolute', 'continuous', 'prismatic'):
            name = j.get('name')
            j.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics', 'safety_controller'):
                for e in j.findall(tag):
                    j.remove(e)
            org = j.find('origin')
            if org is None:
                org = ET.SubElement(j, 'origin', xyz='0 0 0')
            if name.endswith('HipY_joint'):
                org.set('rpy', f'0 {-HIP_Y_Q:.4f} 0')
            elif name.endswith('Knee_joint'):
                org.set('rpy', f'0 {-KNEE_Q:.4f} 0')
        j.attrib.pop('dont_collapse', None)

    def add(parent, child, xyz='0 0 0', rpy='0 0 0'):
        ET.SubElement(robot, 'link', name=child)
        j = ET.SubElement(robot, 'joint', name=f'{child}_joint', type='fixed')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=xyz, rpy=rpy)

    # camera frame tree as on the robot (robot_info.md section 5)
    add('camera_link', 'camera_depth_frame')
    add('camera_depth_frame', 'camera_color_frame', '0.00184 -0.01409 0.00013')
    add('camera_depth_frame', 'camera_depth_optical_frame', rpy=OPT_RPY)
    add('camera_color_frame', 'camera_color_optical_frame', rpy=OPT_RPY)

    gazebo = f"""<root>
  <gazebo reference="base_link"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
  <gazebo reference="FL_FOOT"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
  <gazebo reference="FR_FOOT"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
  <gazebo reference="HL_FOOT"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
  <gazebo reference="HR_FOOT"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
  <gazebo>
    <plugin name="lite3_planar_move" filename="libgazebo_ros_planar_move.so">
      <ros><remapping>cmd_vel:=cmd_vel</remapping><remapping>odom:=odom</remapping></ros>
      <update_rate>50</update_rate>
      <publish_rate>30</publish_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.001</covariance_yaw>
    </plugin>
  </gazebo>
  <gazebo reference="livox_frame">
    <sensor name="mid360" type="ray">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal><samples>720</samples><resolution>1</resolution>
            <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
          <vertical><samples>{lidar_vertical}</samples><resolution>1</resolution>
            <min_angle>-0.122</min_angle><max_angle>0.907</max_angle></vertical>
        </scan>
        <range><min>0.1</min><max>40.0</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
      </ray>
      <plugin name="livox_driver" filename="libgazebo_ros_ray_sensor.so">
        <ros><namespace>/livox</namespace><remapping>~/out:=points</remapping></ros>
        <output_type>sensor_msgs/PointCloud2</output_type>
        <frame_name>livox_frame</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  {{CAMERA}}
  <gazebo reference="base_link">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>{imu_hz}</update_rate>
      <plugin name="imu_driver" filename="libgazebo_ros_imu_sensor.so">
        <ros><namespace>/imu</namespace><remapping>~/out:=data</remapping></ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
        <frame_name>base_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</root>"""
    camera_xml = f"""<gazebo reference="camera_link">
    <sensor name="orbbec" type="depth">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>{cam_hz}</update_rate>
      <camera>
        <horizontal_fov>1.2</horizontal_fov>
        <image><width>{cam_w}</width><height>{cam_h}</height><format>R8G8B8</format></image>
        <clip><near>0.1</near><far>10.0</far></clip>
      </camera>
      <plugin name="orbbec_driver" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/camera</namespace>
          <remapping>color/depth/image_raw:=color/depth/image_m</remapping>
          <remapping>color/depth/camera_info:=color/depth/camera_info_m</remapping>
          <remapping>color/points:=depth/points</remapping>
        </ros>
        <camera_name>color</camera_name>
        <frame_name>camera_color_optical_frame</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>"""
    gazebo = gazebo.replace('{CAMERA}', camera_xml if camera else '')
    for el in ET.fromstring(gazebo):
        robot.append(el)
    ET.indent(tree, space='  ')
    return tree


VARIANTS = {
    'lite3_sim.urdf': dict(cam_w=320, cam_h=240, cam_hz=5),
    'lite3_sim_full.urdf': dict(cam_w=640, cam_h=480, cam_hz=10),
    # sensors:=nav: navigation-only testing. No camera sensor (no /camera/* topics; frames kept),
    # MID360 8 instead of 16 vertical rays (same 720 x 0.5 deg horizontal, 10 Hz), IMU 100 Hz.
    'lite3_sim_nav.urdf': dict(cam_w=320, cam_h=240, cam_hz=5, camera=False, lidar_vertical=8, imu_hz=100),
}
for fname, kw in VARIANTS.items():
    out = os.path.join(OUT_DIR, fname)
    # no <?xml encoding?> declaration: spawn_entity.py rejects it in robot_description
    build(**kw).write(out, encoding='unicode', xml_declaration=False)
    print(f'wrote {out} ' + (f"(camera {kw['cam_w']}x{kw['cam_h']}@{kw['cam_hz']}Hz)" if kw.get('camera', True)
                             else '(no camera, lidar 720x8, imu 100 Hz)'))
with open(os.path.join(OUT_DIR, 'base_height.txt'), 'w') as f:
    f.write(f'{BASE_Z:.4f}\n')
print(f'base_link height {BASE_Z:.3f} m')
