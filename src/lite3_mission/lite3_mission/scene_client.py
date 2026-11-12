"""Run the course's existing Lab 6 client, with one request in flight.

The client source is not in the uploaded handouts. Its path must be supplied;
the documented CLI is reused instead of inventing a server request protocol.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .vision import bgr_image,parse_client_output

PROMPT='''Return a single JSON object with keys room_type (kitchen, living_room, bedroom,
bathroom, other, or unknown), survivor_present (boolean), confidence (0 to 1),
and evidence (a short list of visible cues). Judge only this image. A survivor
may be a competition mannequin; uncertainty must be reflected. Do not invent
positions, invisible people or robot commands. No hazard assessment is requested.'''


class SceneClient(Node):
    def __init__(self,**kwargs):
        super().__init__('scene_client',**kwargs)
        for key,value in dict(image_topic='/camera/color/image_raw',client_path='',server='http://192.168.10.141:8000',
                              group='group04',minimum_interval_sec=5.,request_timeout_sec=20.).items():self.declare_parameter(key,value)
        self.client=Path(self.value('client_path')).expanduser()
        if not self.client.is_file():raise ValueError('Set client_path to your existing Lab 6 client.py; scene integration is optional')
        if self.value('minimum_interval_sec')<1 or not 1<=self.value('request_timeout_sec')<=120:
            raise ValueError('Invalid scene request timing')
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.future=None
        self.latest=None
        self.latest_received=-float('inf')
        self.last_stamp=None
        self.last=-float('inf')
        self.publisher=self.create_publisher(String,'/mission/observations',10)
        self.create_subscription(Image,self.value('image_topic'),self.image,qos_profile_sensor_data)
        self.create_timer(.2,self.tick)

    def value(self,name):return self.get_parameter(name).value

    def image(self,msg):
        self.latest=msg
        self.latest_received=time.monotonic()

    def request(self,msg):
        image=bgr_image(msg.data,msg.width,msg.height,msg.step,msg.encoding)
        scale=min(1.,1024/image.shape[1],768/image.shape[0])
        if scale<1:image=cv2.resize(image,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
        with tempfile.TemporaryDirectory(prefix='lite3_scene_') as directory:
            image_path=Path(directory)/'capture.jpg'
            prompt_path=Path(directory)/'prompt.txt'
            if not cv2.imwrite(str(image_path),image):raise ValueError('Could not encode scene image')
            prompt_path.write_text(PROMPT)
            result=subprocess.run([sys.executable,str(self.client),'--server',self.value('server'),
                '--group',self.value('group'),'--image',str(image_path),'--prompt-file',str(prompt_path)],
                capture_output=True,text=True,timeout=self.value('request_timeout_sec'),
                env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
            if result.returncode:raise ValueError('Lab scene client failed: '+result.stderr[-400:])
            return msg,parse_client_output(result.stdout)

    def tick(self):
        if self.future and self.future.done():
            try:
                msg,result=self.future.result()
                base=dict(frame_id=msg.header.frame_id,stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec),
                          confidence=result['confidence'],evidence=str(result['evidence']),
                          localization_status='unlocalized_image_classification')
                if result['room_type'] not in ('other','unknown'):
                    self.publisher.publish(String(data=json.dumps(dict(base,kind='room',label=result['room_type']))))
                if result['survivor_present']:
                    self.publisher.publish(String(data=json.dumps(dict(base,kind='survivor',label='possible_survivor'))))
            except Exception as exc:self.get_logger().warning('Scene request failed: '+str(exc))
            self.future=None
        stamp=(self.latest.header.stamp.sec,self.latest.header.stamp.nanosec) if self.latest is not None else None
        if (self.future is None and self.latest is not None and time.monotonic()-self.latest_received<2.
                and stamp!=self.last_stamp and time.monotonic()-self.last>=self.value('minimum_interval_sec')):
            self.last=time.monotonic()
            self.last_stamp=stamp
            self.future=self.pool.submit(self.request,self.latest)

    def close(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
        self.destroy_node()


def main():
    from .runtime import initialize
    stop=initialize()
    node=SceneClient()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=0)
            time.sleep(.002)
    except KeyboardInterrupt:pass
    finally:
        node.close()
        if rclpy.ok():rclpy.shutdown()
