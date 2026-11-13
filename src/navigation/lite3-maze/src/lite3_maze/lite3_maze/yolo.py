"""Bounded CPU person detection, independent of maze control and robot mode."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Int32
from .object_detector import Detector
from .runtime import run


class Yolo(Node):
    def __init__(self):
        super().__init__('maze_yolo')
        for key,value in dict(image_topic='/camera/color/image_raw', image_transport='compressed',
            model_path=str(Path(get_package_share_directory('lite3_maze'))/'models/yolov5n.onnx'),
            confidence=.4, rate_hz=2.).items():
            self.declare_parameter(key,value)
        self.detector=Detector(self.get_parameter('model_path').value,self.get_parameter('confidence').value)
        self.bridge=CvBridge()
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.future=self.latest=None
        self.period=1./max(.1,self.get_parameter('rate_hz').value)
        self.received=self.last=self.last_status=-float('inf')
        self.received_frames=self.processed_frames=0
        self.latency=None
        self.objects=self.create_publisher(String,'/maze/yolo/objects',10)
        self.people=self.create_publisher(String,'/maze/yolo/people',10)
        self.count=self.create_publisher(Int32,'/n_humans',10)
        self.debug=self.create_publisher(Image,'/maze/people/debug_image',1)
        self.jpeg=self.create_publisher(CompressedImage,'/maze/people/debug_image/compressed',1)
        self.status=self.create_publisher(String,'/maze/yolo/status',1)
        compressed=self.get_parameter('image_transport').value=='compressed'
        topic=self.get_parameter('image_topic').value+('/compressed' if compressed else '')
        self.create_subscription(CompressedImage if compressed else Image,topic,self.image,qos_profile_sensor_data)
        self.get_logger().info('YOLOv5n CPU person detector: '+topic)

    def image(self,msg):
        self.latest=msg
        self.received=time.monotonic()
        self.received_frames+=1

    def infer(self,msg):
        start=time.monotonic()
        if isinstance(msg,CompressedImage):
            frame=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
        else:
            frame=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        if frame is None:raise ValueError('Cannot decode camera frame')
        return msg.header,frame,self.detector.detect(frame),time.monotonic()-start

    def update(self):
        now=time.monotonic()
        if self.future and self.future.done():
            try:
                header,frame,objects,self.latency=self.future.result()
                self.processed_frames+=1
                people=[o for o in objects if o['label']=='person']
                base=dict(frame_id=header.frame_id,stamp=dict(sec=header.stamp.sec,nanosec=header.stamp.nanosec),
                    image_width=frame.shape[1],image_height=frame.shape[0],model='yolov5n',localized=False,
                    box_format='xyxy_pixels')
                self.objects.publish(String(data=json.dumps(dict(base,objects=objects))))
                self.people.publish(String(data=json.dumps(dict(base,count=len(people),people=people))))
                self.count.publish(Int32(data=len(people)))
                if self.debug.get_subscription_count() or self.jpeg.get_subscription_count():
                    for obj in people:
                        x1,y1,x2,y2=map(int,obj['box'])
                        cv2.rectangle(frame,(x1,y1),(x2,y2),(40,240,60),2)
                        cv2.putText(frame,'person %.2f'%obj['confidence'],(x1,max(14,y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX,.45,(40,240,60),1)
                    cv2.putText(frame,'YOLO people: '+str(len(people)),(10,22),
                        cv2.FONT_HERSHEY_SIMPLEX,.6,(40,240,60),2)
                    if self.debug.get_subscription_count():
                        output=self.bridge.cv2_to_imgmsg(frame,encoding='bgr8');output.header=header
                        self.debug.publish(output)
                    if self.jpeg.get_subscription_count():
                        self.jpeg.publish(CompressedImage(header=header,format='jpeg',
                            data=cv2.imencode('.jpg',frame)[1].tobytes()))
            except Exception as exc:
                self.get_logger().error('YOLO frame failed: '+str(exc))
            self.future=None
        if self.future is None and self.latest is not None and now-self.received<2. and now-self.last>=self.period:
            msg,self.latest=self.latest,None
            self.last=now
            self.future=self.pool.submit(self.infer,msg)
        if now-self.last_status>1.:
            self.last_status=now
            self.status.publish(String(data=json.dumps(dict(
                state='RUNNING' if now-self.received<2. else 'WAITING_FOR_CAMERA',
                received_frames=self.received_frames,processed_frames=self.processed_frames,
                inference_seconds=self.latency,provider='CPUExecutionProvider'))))

    def destroy_node(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main():
    run(Yolo)
