#!/usr/bin/env python3
"""Exercise the physical map/TF path on isolated local ROS, without a robot."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu,PointCloud2,PointField
from std_msgs.msg import Bool,String,Header
from tf2_ros import TransformBroadcaster
from lite3_maze.navigator import Navigator

rclpy.init(args=['--ros-args','-p','localization_source:=tf','-p','map_frame:=map',
                '-p','forward_speed:=0.12','-p','reverse_speed:=0.12'])
node=Navigator();fixture=Node('physical_interface_fixture')
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
mode=fixture.create_publisher(Bool,'/maze/auto_mode',latched)
mp=fixture.create_publisher(OccupancyGrid,'/map',latched)
ip=fixture.create_publisher(Imu,'/imu/data',10)
cp=fixture.create_publisher(PointCloud2,'/points',10)
tf=TransformBroadcaster(fixture)
commands=[];statuses=[]
fixture.create_subscription(Twist,'/cmd_vel',lambda m:commands.append([m.linear.x,m.linear.y,m.angular.z]),10)
fixture.create_subscription(String,'/maze/status',lambda m:statuses.append(json.loads(m.data)),10)
a=np.full((100,150),-1,dtype=np.int8)
a[38:62,13:130]=0;a[37,12:130]=100;a[62,12:130]=100;a[37:63,12]=100
m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=.04;m.info.width=150;m.info.height=100
m.info.origin.position.x=-1.;m.info.origin.position.y=-2.;m.info.origin.orientation.w=1.;m.data=a.ravel().tolist()
points=np.array([[x,y,-.34] for x in np.linspace(-.6,.6,8) for y in np.linspace(-.6,.6,8)],dtype='<f4')
points=np.vstack([points,[[.3,0.,-.25],[.4,.1,-.25],[.5,-.1,-.25],[.6,0.,-.25]]]).astype('<f4')
cloud=PointCloud2(header=Header(frame_id='base_link'),height=1,width=len(points),point_step=12,row_step=len(points)*12,
 fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))],data=points.tobytes())
imu=Imu();imu.orientation.w=1.

def spin(seconds, send_tf=True):
 end=time.monotonic()+seconds;next_pub=0.;next_update=0.
 while time.monotonic()<end:
  now=time.monotonic()
  if now>next_pub:
   stamp=fixture.get_clock().now().to_msg();imu.header.stamp=stamp;ip.publish(imu)
   cloud.header.stamp=stamp;cp.publish(cloud);m.header.stamp=stamp;mp.publish(m)
   if send_tf:
    t=TransformStamped();t.header=Header(frame_id='map',stamp=stamp);t.child_frame_id='base_link';t.transform.rotation.w=1.;tf.sendTransform(t)
   next_pub=now+.05
  for _ in range(4):
   rclpy.spin_once(node,timeout_sec=0);rclpy.spin_once(fixture,timeout_sec=0)
  if now>next_update:node.update();next_update=now+.05
  time.sleep(.002)

try:
 spin(1.5)
 assert node.terrain=='high_roughness','Raised terrain must select RUG High-Step'
 assert not commands,'Manual mode must not continually overwrite operator velocity'
 mode.publish(Bool(data=True));spin(1.5)
 assert any(abs(c[0])+abs(c[1])>0 for c in commands),statuses[-1] if statuses else None
 assert max(np.linalg.norm(c[:2]) for c in commands)<=.12001
 mode.publish(Bool(data=False));spin(.5)
 assert commands[-1]==[0.,0.,0.]
 count=len(commands);spin(.3)
 assert len(commands)==count,'Manual stop must release the velocity topic'
 mode.publish(Bool(data=True));spin(.5);spin(1.2,send_tf=False)
 assert commands[-1]==[0.,0.,0.],'Stale localization must stop movement'
 print('PASS: physical map/TF input, Auto starts, Manual stops/releases, configured speed, stale TF stops')
finally:
 node.halt();node.destroy_node();fixture.destroy_node();rclpy.shutdown()

# Protocol adapter contract: documented Move on Auto, two RUG codes, STOP disables.
from types import SimpleNamespace
from lite3_maze.mode_bridge import ModeBridge,AUTO,STOP,MOVE,GAITS
rclpy.init(args=[])
b=ModeBridge()
class Publisher:
 def __init__(self):self.messages=[]
 def publish(self,m):self.messages.append(m)
class Command:
 def __init__(self):self.cmd_code=0;self.type=0
b.command_pub=Publisher();b.cls=Command
try:
 b.command(SimpleNamespace(cmd_code=AUTO))
 assert b.auto and b.command_pub.messages[-1].cmd_code==MOVE
 b.pending_gait='rug_high_step';b.tick()
 assert b.command_pub.messages[-1].cmd_code==GAITS['rug_high_step']
 b.command(SimpleNamespace(cmd_code=STOP))
 assert not b.auto
 print('PASS: documented Auto/Move, RUG High-Step, STOP mode observer')
finally:
 b.destroy_node();rclpy.shutdown()

# Planning can block; the command stream must still expire its previous lease.
from lite3_maze.command_stream import CommandStream
p=Publisher();stream=CommandStream(p)
try:
 cmd=Twist();cmd.linear.x=.12
 stream.set(cmd);time.sleep(.10)
 assert any(m.linear.x>0 for m in p.messages)
 time.sleep(.18)
 assert p.messages[-1].linear.x==0.,'A stalled planner must not keep its old command alive'
 stream.halt();count=len(p.messages);time.sleep(.06)
 assert len(p.messages)==count,'Manual mode must release command publishing'
 print('PASS: independent 50 Hz stream, 200 ms planning lease expiry, Manual release')
finally:
 stream.close()
