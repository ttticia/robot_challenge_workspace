"""Decode actual camera pixels; optional calibrated PnP, never ground-truth labels."""
import json
import time
import cv2
import numpy as np
from pyzbar.pyzbar import decode,ZBarSymbol
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from .vision import bgr_image,qr_position


class QR(Node):
    def __init__(self,**kwargs):
        super().__init__('qr',**kwargs)
        for key,value in dict(image_topic='/camera/color/image_raw',camera_info_topic='/camera/color/camera_info',
                              max_decode_hz=2.,qr_size_m=0.).items():self.declare_parameter(key,value)
        if self.value('max_decode_hz')<=0:raise ValueError('max_decode_hz must be positive')
        self.info=None
        self.last=-float('inf')
        self.publisher=self.create_publisher(String,'/mission/observations',10)
        self.create_subscription(CameraInfo,self.value('camera_info_topic'),lambda m:setattr(self,'info',m),qos_profile_sensor_data)
        self.create_subscription(Image,self.value('image_topic'),self.image,qos_profile_sensor_data)

    def value(self,name):return self.get_parameter(name).value

    def image(self,msg):
        now=time.monotonic()
        if now-self.last<1/self.value('max_decode_hz'):return
        self.last=now
        try:
            frame=bgr_image(msg.data,msg.width,msg.height,msg.step,msg.encoding)
            # Ubuntu OpenCV builds may omit QUIRC; ZBar provides actual decoding.
            for symbol in decode(frame,symbols=[ZBarSymbol.QRCODE]):
                label=symbol.data.decode('utf-8',errors='replace')
                if not label:continue
                corners=np.array([[p.x,p.y] for p in symbol.polygon],dtype=float)
                if len(corners)!=4:continue
                centre=corners.mean(axis=0)
                corners=corners[np.argsort(np.arctan2(corners[:,1]-centre[1],corners[:,0]-centre[0]))]
                corners=np.roll(corners,-int(np.argmin(corners.sum(axis=1))),axis=0)
                event=dict(kind='qr',label=label,frame_id=msg.header.frame_id,confidence=1.,
                           stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec),
                           corners=corners.tolist(),evidence='camera QR decode')
                if self.value('qr_size_m')>0 and self.info is not None:
                    if (self.info.width,self.info.height,self.info.header.frame_id)!=(msg.width,msg.height,msg.header.frame_id):
                        event['localization_error']='Image/calibration dimensions or frame do not match'
                    elif self.info.distortion_model not in ('plumb_bob','rational_polynomial'):
                        event['localization_error']='Unsupported distortion model'
                    else:
                        try:
                            error,position=qr_position(corners,self.info.k,self.info.d,self.value('qr_size_m'))
                            event.update(position=position,reprojection_error_px=error)
                        except ValueError as exc:event['localization_error']=str(exc)
                self.publisher.publish(String(data=json.dumps(event)))
        except (ValueError,cv2.error) as exc:self.get_logger().warning('QR frame rejected: '+str(exc))


def main():
    from .runtime import initialize
    stop=initialize()
    node=QR()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=0)
            time.sleep(.002)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
