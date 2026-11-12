"""Transform timestamped observations through the lab TF tree and deduplicate."""
import json
from pathlib import Path
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from .core import EventStore, transform_point


class Observations(Node):
    def __init__(self,**kwargs):
        super().__init__('observations',**kwargs)
        for key,value in dict(map_frame='map',input_topic='/mission/observations',point_topic='/detections',
                              point_kind='target',point_label='lab_detection',dedup_radius_m=.25,
                              cell_grid_enabled=False,cell_size_m=.8,cell_origin_x=0.,cell_origin_y=0.,
                              report_directory='~/.ros/lite3_mission').items():self.declare_parameter(key,value)
        origin=[self.value('cell_origin_x'),self.value('cell_origin_y')] if self.value('cell_grid_enabled') else None
        self.store=EventStore(self.value('dedup_radius_m'),self.value('cell_size_m'),origin)
        folder=Path(self.value('report_directory')).expanduser()
        folder.mkdir(parents=True,exist_ok=True)
        self.path=folder/(time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]+'.json')
        self.buffer=Buffer(cache_time=Duration(seconds=60.))
        self.listener=TransformListener(self.buffer,self)
        self.pending=[]
        self.unlocalized=[]
        self.publisher=self.create_publisher(String,'/mission/report',10)
        self.create_subscription(String,self.value('input_topic'),self.on_json,10)
        self.create_subscription(PointStamped,self.value('point_topic'),self.on_point,qos_profile_sensor_data)
        self.create_timer(.2,self.process)
        self.get_logger().info('Mission report: '+str(self.path))

    def value(self,name):return self.get_parameter(name).value

    def on_point(self,msg):
        self.enqueue(dict(kind=self.value('point_kind'),label=self.value('point_label'),
                          frame_id=msg.header.frame_id,stamp=dict(sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec),
                          position=[msg.point.x,msg.point.y,msg.point.z],confidence=1.))

    def on_json(self,msg):
        try:
            def invalid(value):raise ValueError('Non-finite JSON number: '+value)
            event=json.loads(msg.data,parse_constant=invalid)
            if not isinstance(event,dict):raise ValueError('Observation must be an object')
            self.enqueue(event)
        except (ValueError,TypeError) as exc:self.get_logger().warning('Rejected observation: '+str(exc))

    def enqueue(self,event):
        if (event.get('kind') not in ('qr','survivor','room','target') or not isinstance(event.get('label'),str)
                or not 0<len(event['label'])<=2048):
            raise ValueError('Invalid observation label or kind')
        confidence=float(event.get('confidence',1.))
        if not 0.<=confidence<=1.:raise ValueError('Invalid observation confidence')
        if 'position' not in event:
            # Preserve decoded text/scene evidence, but do not invent a map position.
            self.unlocalized.append(event)
            self.unlocalized=self.unlocalized[-100:]
            self.write_report()
            return
        self.pending.append((time.monotonic(),event))
        self.pending=self.pending[-100:]

    def process(self):
        remaining=[]
        changed=False
        for started,event in self.pending:
            try:
                event=dict(event)
                frame=event.get('frame_id','')
                if not frame:raise ValueError('Missing observation frame')
                stamp=event.get('stamp')
                if not isinstance(stamp,dict) or not (stamp.get('sec',0) or stamp.get('nanosec',0)):
                    raise ValueError('Capture timestamp is required; latest TF is not substituted')
                if frame!=self.value('map_frame'):
                    tf=self.buffer.lookup_transform(self.value('map_frame'),frame,
                        Time(seconds=int(stamp['sec']),nanoseconds=int(stamp.get('nanosec',0))))
                    p,q=tf.transform.translation,tf.transform.rotation
                    event['position']=transform_point(event['position'],[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
                event['frame_id']=self.value('map_frame')
                self.store.add(event)
                changed=True
            except TransformException:
                if time.monotonic()-started<5.:remaining.append((started,event))
                else:self.get_logger().warning('Observation expired while waiting for capture-time TF')
            except (ValueError,KeyError,TypeError) as exc:
                self.get_logger().warning('Rejected observation: '+str(exc))
        self.pending=remaining
        if changed:self.write_report()

    def write_report(self):
        report=dict(frame_id=self.value('map_frame'),cell_grid_calibrated=self.value('cell_grid_enabled'),
                    records=self.store.records,unlocalized_observations=self.unlocalized)
        text=json.dumps(report,indent=2,allow_nan=False)
        temporary=self.path.with_suffix('.tmp')
        temporary.write_text(text+'\n')
        temporary.replace(self.path)
        self.publisher.publish(String(data=text))


def main():
    from .runtime import initialize
    stop=initialize()
    node=Observations()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=0)
            time.sleep(.002)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
