#!/usr/bin/env python3
"""Test-only planar kinematic base; it is not a quadruped physics benchmark."""
import json
import math
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist,TransformStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster


class Fixture(Node):
    def __init__(self):
        super().__init__('planar_test_fixture')
        self.xy=np.zeros(2);self.yaw=0.
        self.command=np.zeros(3);self.command_time=-1e9
        self.start=self.last=time.monotonic();self.steps=0
        self.create_subscription(Twist,'/cmd_vel',self.on_command,1)
        self.clock=self.create_publisher(Clock,'/clock',10)
        self.odom=self.create_publisher(Odometry,'/odom',10)
        self.imu=self.create_publisher(Imu,'/imu/data',10)
        self.scan=self.create_publisher(LaserScan,'/scan',10)
        self.map=self.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pose=self.create_publisher(String,'/test/pose',10)
        self.tf=TransformBroadcaster(self)
        self.static=StaticTransformBroadcaster(self)
        transforms=[]
        for parent,child,z in [('map','odom',0.),('base_link','laser',.12)]:
            tf=TransformStamped();tf.header.frame_id=parent;tf.child_frame_id=child
            tf.transform.translation.z=z;tf.transform.rotation.w=1.;transforms.append(tf)
        self.static.sendTransform(transforms)
        self.map_msg=OccupancyGrid()
        self.map_msg.header.frame_id='map'
        self.map_msg.info.resolution=.04;self.map_msg.info.width=self.map_msg.info.height=100
        self.map_msg.info.origin.position.x=self.map_msg.info.origin.position.y=-2.
        self.map_msg.info.origin.orientation.w=1.
        grid=np.zeros((100,100),np.int8);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
        self.explore='--explore' in sys.argv
        self.complete_grid=grid.copy()
        if self.explore:grid[1:-1,80:-1]=-1
        self.map_msg.data=grid.ravel().tolist()

    def on_command(self,msg):
        command=np.array([msg.linear.x,msg.linear.y,msg.angular.z])
        if np.isfinite(command).all():self.command=command;self.command_time=time.monotonic()

    def tick(self):
        now=time.monotonic();dt=min(now-self.last,.05);self.last=now
        velocity=self.command if now-self.command_time<.3 else np.zeros(3)
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        self.xy+=np.array([[c,-s],[s,c]])@velocity[:2]*dt
        self.yaw+=float(velocity[2])*dt
        ns=int((1000.+now-self.start)*1e9)
        stamp=Time(sec=ns//10**9,nanosec=ns%10**9)
        self.clock.publish(Clock(clock=stamp))
        tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='odom';tf.child_frame_id='base_link'
        tf.transform.translation.x,tf.transform.translation.y=map(float,self.xy)
        tf.transform.rotation.z,tf.transform.rotation.w=math.sin(self.yaw/2),math.cos(self.yaw/2)
        self.tf.sendTransform(tf)
        odom=Odometry(header=tf.header,child_frame_id='base_link')
        odom.pose.pose.position.x,odom.pose.pose.position.y=map(float,self.xy)
        odom.pose.pose.orientation=tf.transform.rotation
        odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z=map(float,velocity)
        self.odom.publish(odom)
        imu=Imu();imu.header.stamp=stamp;imu.header.frame_id='base_link';imu.orientation=tf.transform.rotation
        imu.linear_acceleration.z=9.81;self.imu.publish(imu)
        self.pose.publish(String(data=json.dumps({'xy':self.xy.tolist(),'yaw':self.yaw,'velocity':velocity.tolist()})))
        self.steps+=1
        if self.steps%5==0:
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser'
            scan.angle_min=-math.pi;scan.angle_max=math.pi-2*math.pi/360;scan.angle_increment=2*math.pi/360
            scan.range_min=.05;scan.range_max=6.;scan.scan_time=.1
            directions=np.linspace(-math.pi,math.pi,360,endpoint=False)+self.yaw
            distances=[]
            for angle in directions:
                vx,vy=math.cos(angle),math.sin(angle)
                hits=[((1.96 if vx>0 else -1.96)-self.xy[0])/vx if abs(vx)>1e-6 else math.inf,
                      ((1.96 if vy>0 else -1.96)-self.xy[1])/vy if abs(vy)>1e-6 else math.inf]
                distances.append(float(min(hits)))
            scan.ranges=distances;self.scan.publish(scan)
        if self.steps%25==1:
            if self.explore and self.xy[0]>.6:
                self.map_msg.data=self.complete_grid.ravel().tolist()
            self.map_msg.header.stamp=stamp;self.map.publish(self.map_msg)


if __name__=='__main__':
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Fixture();deadline=time.monotonic()
    try:
        while rclpy.ok():
            for _ in range(8):rclpy.spin_once(node,timeout_sec=0)
            if time.monotonic()>=deadline:node.tick();deadline=time.monotonic()+.02
            time.sleep(.002)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
