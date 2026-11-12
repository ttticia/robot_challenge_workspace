"""People detections from camera images, independent of navigation."""
import json
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CompressedImage
from std_msgs.msg import String
from .people import PeopleDetector


class PeopleNode(Node):
    def __init__(self,**kwargs):
        super().__init__('people_detector',**kwargs)
        defaults=dict(model_path=str(Path.home()/'challenge_ws/models/yolov5n.onnx'),
                      image_topic='/camera/color/image_raw',image_transport='compressed',
                      confidence=.5,max_inference_hz=2.,publish_debug=True)
        for key,value in defaults.items():self.declare_parameter(key,value)
        get=lambda k:self.get_parameter(k).value
        if not 0.<get('confidence')<=1. or not .1<=get('max_inference_hz')<=10.:raise ValueError('Invalid detector rate/confidence')
        self.detector=PeopleDetector(get('model_path'),get('confidence'))
        self.latest=None
        self.transport=get('image_transport')
        if self.transport not in ('compressed','raw'):raise ValueError('image_transport must be compressed/raw')
        self.create_subscription(CompressedImage if self.transport=='compressed' else Image,
            get('image_topic')+('/compressed' if self.transport=='compressed' else ''),self.receive,qos_profile_sensor_data)
        self.events=self.create_publisher(String,'/people/detections',10)
        self.observations=self.create_publisher(String,'/mission/observations',10)
        self.debug=self.create_publisher(CompressedImage,'/people/debug_image/compressed',1) if get('publish_debug') else None
        self.create_timer(1./get('max_inference_hz'),self.process)
        self.get_logger().info('People detector ready (YOLOv5n ONNX, COCO person class)')

    def receive(self,msg):self.latest=msg

    def process(self):
        if self.latest is None:return
        msg=self.latest;self.latest=None
        try:
            if self.transport=='compressed':frame=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
            else:
                from cv_bridge import CvBridge
                frame=CvBridge().imgmsg_to_cv2(msg,'bgr8')
            if frame is None:raise ValueError('Image decode failed')
            boxes=self.detector.detect(frame)
            stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec)
            event=dict(stamp=stamp,frame_id=msg.header.frame_id,count=len(boxes),detections=boxes,
                       model='yolov5n-coco',image_size=[frame.shape[1],frame.shape[0]])
            self.events.publish(String(data=json.dumps(event,allow_nan=False)))
            for box in boxes:
                self.observations.publish(String(data=json.dumps(dict(kind='target',label='person',
                    confidence=box['confidence'],stamp=stamp,frame_id=msg.header.frame_id,
                    pixel_bbox=box['bbox_xyxy'],source='team_perception_people'),allow_nan=False)))
                if self.debug:
                    x1,y1,x2,y2=map(int,box['bbox_xyxy']);cv2.rectangle(frame,(x1,y1),(x2,y2),(0,220,0),2)
                    cv2.putText(frame,'person %.2f'%box['confidence'],(x1,max(15,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,220,0),1)
            if self.debug:
                ok,jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,70])
                if ok:self.debug.publish(CompressedImage(header=msg.header,format='jpeg',data=jpeg.tobytes()))
        except (ValueError,RuntimeError,cv2.error) as exc:self.get_logger().warning('People frame rejected: '+str(exc))


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    import signal,threading
    stop=threading.Event()
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,lambda *_:stop.set())
    node=None
    try:
        node=PeopleNode()
        while rclpy.ok() and not stop.is_set():rclpy.spin_once(node,timeout_sec=.05)
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
