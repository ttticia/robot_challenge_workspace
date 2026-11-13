#!/usr/bin/env python3
"""Real ONNX inference through compressed ROS camera -> people JSON + RViz image."""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage,Image
from std_msgs.msg import String,Int32
from lite3_maze.yolo import Yolo
from lite3_maze.detections import Detections

p=argparse.ArgumentParser();p.add_argument('image',type=Path);p.add_argument('--output',type=Path,default=Path('docs/validation/yolo_positive.json'));a=p.parse_args()
frame=cv2.imread(str(a.image));assert frame is not None
rclpy.init(args=['--ros-args','-p','people_input_topic:=/maze/yolo/people'])
yolo=Yolo();adapter=Detections();fixture=Node('yolo_camera_fixture')
pub=fixture.create_publisher(CompressedImage,'/camera/color/image_raw/compressed',10)
people=[];debug=[];counts=[]
fixture.create_subscription(String,'/maze/people/detections',lambda m:people.append(json.loads(m.data)),10)
fixture.create_subscription(Image,'/maze/people/debug_image',lambda m:debug.append(m),10)
fixture.create_subscription(Int32,'/n_humans',lambda m:counts.append(m.data),10)
start=time.monotonic();next_frame=0
try:
 while time.monotonic()-start<6:
  now=time.monotonic()
  if now>next_frame:
   m=CompressedImage();m.header.frame_id='camera_color_optical_frame';m.header.stamp=fixture.get_clock().now().to_msg()
   m.format='jpeg';m.data=cv2.imencode('.jpg',frame)[1].tobytes();pub.publish(m);next_frame=now+.5
  for node in (yolo,adapter,fixture):rclpy.spin_once(node,timeout_sec=0)
  yolo.update()
  positives=[m for m in people if m['source']=='detector' and m['payload']['count']>0]
  if positives and debug and counts and counts[-1]>0:break
  time.sleep(.002)
 assert positives,'Expected a positive person detection'
 assert debug and debug[-1].encoding=='bgr8','RViz annotated image missing'
 result=positives[-1]
 assert result['payload']['frame_id']=='camera_color_optical_frame'
 assert result['localized'] is False
 a.output.write_text(json.dumps(result,indent=2)+'\n')
 pixels=yolo.bridge.imgmsg_to_cv2(debug[-1],desired_encoding='bgr8')
 cv2.imwrite(str(a.output.with_suffix('.jpg')),pixels)
 print('PASS: YOLO person detection, compressed camera, stable people output, count, RViz annotation')
finally:
 yolo.destroy_node();adapter.destroy_node();fixture.destroy_node();rclpy.shutdown()
