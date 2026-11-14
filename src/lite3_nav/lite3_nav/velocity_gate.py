# Fork of lite3_mission/velocity_gate.py (main e725625) with:
#  - output_transient_local (default true): /cmd_vel published RELIABLE + TRANSIENT_LOCAL with
#    lifespan = command_timeout_sec, as the Lite3 domain bridge's /cmd_vel subscription requires.
#  - expected_input_publishers (default 1, range 1-4) instead of exactly one; publishers are
#    counted per node, not per endpoint (Nav2 behavior_server opens several on one topic).
#  - Gate status reason renamed: EXPECTED_ONE_NAV2_INPUT_PUBLISHER_GOT_<n>
#    -> EXPECTED_<k>_NAV2_INPUT_PUBLISHERS_GOT_<n>.
#  - core and runtime helpers are imported from the team's lite3_mission package unchanged
#    (absolute imports instead of relative ones).
"""Independent wall-clock command timeout and limiter; hardware output is opt-in."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy,QoSProfile,ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from lite3_mission.core import VelocityLimiter


class VelocityGate(Node):
    def __init__(self,**kwargs):
        super().__init__('velocity_gate',**kwargs)
        defaults=dict(drive_output_enabled=False,input_topic='/mission/nav_cmd_vel',output_topic='/cmd_vel',
                      command_timeout_sec=.25,lease_timeout_sec=.3,max_linear_m_s=.1,max_angular_rad_s=.25,
                      acceleration_m_s2=.2,angular_acceleration_rad_s2=.5,
                      output_transient_local=True,expected_input_publishers=1)
        read_only=('drive_output_enabled','input_topic','output_topic','output_transient_local','expected_input_publishers')
        for key,value in defaults.items():
            self.declare_parameter(key,value,ParameterDescriptor(read_only=key in read_only))
        result=self.validate([])
        if not result.successful:raise ValueError(result.reason)
        if self.value('input_topic')==self.value('output_topic'):
            raise ValueError('Gate input and final output must be different topics')
        self.add_on_set_parameters_callback(self.validate)
        self.drive=self.value('drive_output_enabled')
        self.name=self.get_name()
        if not 1<=self.value('expected_input_publishers')<=4:
            raise ValueError('expected_input_publishers must be 1-4')
        # The Lite3 domain bridge subscribes to /cmd_vel RELIABLE + TRANSIENT_LOCAL and never
        # receives from a VOLATILE publisher. This profile also matches VOLATILE subscribers
        # (Gazebo). The lifespan stops a reconnecting subscriber replaying an old command.
        output_qos=(QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               lifespan=Duration(seconds=float(self.value('command_timeout_sec'))))
                    if self.value('output_transient_local') else 1)
        self.publisher=self.create_publisher(Twist,self.value('output_topic') if self.drive else '/mission/limited_cmd_vel',output_qos)
        self.status=self.create_publisher(String,'/mission/gate_status',10)
        self.create_subscription(Twist,self.value('input_topic'),self.command,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Bool,'/mission/drive_lease',self.lease,1)
        self.target=np.zeros(3)
        self.command_time=self.lease_time=-math.inf
        self.lease_value=False
        self.last_tick=time.monotonic()
        self.last_graph=self.last_status=0.
        self.conflicts=[]
        self.sources=0
        self.lease_sources=0
        self.last_active=False
        self.last_output=np.zeros(3)
        self.limiter=VelocityLimiter()

    def value(self,name):return self.get_parameter(name).value

    def validate(self,parameters):
        values={p.name:p.value for p in parameters}
        for key,low,high in [('max_linear_m_s',.01,.3),('max_angular_rad_s',.02,.8),
                             ('acceleration_m_s2',.02,1.),('angular_acceleration_rad_s2',.05,2.),
                             ('command_timeout_sec',.05,.5),('lease_timeout_sec',.1,.5)]:
            value=values.get(key,self.value(key))
            if not isinstance(value,(float,int)) or not math.isfinite(value) or not low<=value<=high:
                return SetParametersResult(successful=False,reason=key+' outside supported bounds')
        return SetParametersResult(successful=True)

    def command(self,msg):
        target=np.array([msg.linear.x,msg.linear.y,msg.angular.z])
        if np.isfinite(target).all():
            self.target=target
            self.command_time=time.monotonic()
        else:
            self.target[:]=0.
            self.command_time=-math.inf

    def lease(self,msg):
        self.lease_value=bool(msg.data)
        self.lease_time=time.monotonic()

    def refresh_graph(self):
        conflicts=[]
        for topic in (self.value('output_topic'),'/cmd_vel_corrected','/simple_cmd','/complex_cmd'):
            for endpoint in self.get_publishers_info_by_topic(topic):
                if not (topic==self.value('output_topic') and endpoint.node_name==self.name and endpoint.node_namespace==self.get_namespace()):
                    conflicts.append(topic+':'+endpoint.node_name)
        self.conflicts=conflicts
        # More than one input producer is also ambiguous, even when bounded.
        # Count producing nodes, not endpoints: Nav2's behavior_server opens one publisher per
        # behavior plugin on the same topic.
        self.sources=len({(e.node_namespace,e.node_name) for e in self.get_publishers_info_by_topic(self.value('input_topic'))})
        self.lease_sources=self.count_publishers('/mission/drive_lease')

    def tick(self):
        now=time.monotonic()
        if now-self.last_graph>=.25:
            self.refresh_graph()
            self.last_graph=now
        if not self.drive:reason='PREVIEW_ONLY'
        elif self.conflicts:reason='COMPETING_COMMAND_SOURCE: '+','.join(self.conflicts)
        elif self.sources!=self.value('expected_input_publishers'):
            reason='EXPECTED_'+str(self.value('expected_input_publishers'))+'_NAV2_INPUT_PUBLISHERS_GOT_'+str(self.sources)
        elif self.lease_sources!=1:reason='EXPECTED_ONE_MISSION_LEASE_PUBLISHER_GOT_'+str(self.lease_sources)
        else:reason='READY'
        ready=reason=='READY'
        lease_ok=self.lease_value and now-self.lease_time<=self.value('lease_timeout_sec')
        command_ok=now-self.command_time<=self.value('command_timeout_sec')
        permitted=ready and lease_ok and command_ok
        self.limiter.linear=self.value('max_linear_m_s')
        self.limiter.angular=self.value('max_angular_rad_s')
        self.limiter.acceleration=self.value('acceleration_m_s2')
        self.limiter.angular_acceleration=self.value('angular_acceleration_rad_s2')
        output=self.limiter.apply(self.target,now-self.last_tick,permitted)
        self.last_tick=now
        msg=Twist()
        msg.linear.x,msg.linear.y,msg.angular.z=map(float,output)
        # Never fight another controller with a stream of zero commands while
        # idle. If ownership changes during motion, send one immediate stop.
        if not self.drive or not self.conflicts or self.last_active:
            self.publisher.publish(msg)
        self.last_active=permitted
        self.last_output=output
        if now-self.last_status>=.2:
            self.last_status=now
            self.status.publish(String(data=json.dumps(dict(ready_to_arm=ready,reason=reason,
                drive_output_enabled=self.drive,lease_fresh=lease_ok,command_fresh=command_ok,
                output=output.tolist(),input_publishers=self.sources))))

    def stop(self):
        if self.drive and self.last_active:
            for _ in range(3):self.publisher.publish(Twist())


def main():
    from lite3_mission.runtime import initialize
    stop=initialize()
    node=VelocityGate()
    deadline=time.monotonic()
    try:
        while rclpy.ok() and not stop.is_set():
            for _ in range(8):rclpy.spin_once(node,timeout_sec=0)
            now=time.monotonic()
            if now>=deadline:
                node.tick()
                deadline=now+.05
            time.sleep(.002)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.stop()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
