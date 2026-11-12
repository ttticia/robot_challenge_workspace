"""Wait for the operator auto command, select H-Step, then arm exploration."""
import json,math,shutil,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from transfer_interfaces.msg import MotionSimpleCMD


class MazeSession(Node):
    def __init__(self,**kwargs):
        super().__init__('maze_session',**kwargs)
        for k,v in dict(autostart=False,run_directory='',startup_timeout_sec=90.,maximum_run_sec=600.,
                        no_progress_timeout_sec=30.,minimum_free_disk_mb=512).items():self.declare_parameter(k,v)
        self.folder=Path(self.get_parameter('run_directory').value).expanduser();self.folder.mkdir(parents=True,exist_ok=True)
        self.state={};self.gate={}
        self.phase='PREPARE_HIGH_STEP' if self.get_parameter('autostart').value else 'PREVIEW'
        self.started=time.monotonic();self.mode_pub=None;self.mode_time=None;self.future=None
        self.done=False;self.auto_requested=False;self.result=None;self.run_started=None
        self.last_pose=None;self.last_progress=time.monotonic()
        self.create_subscription(String,'/mission/status',self.status,10)
        self.create_subscription(String,'/mission/gate_status',lambda m:setattr(self,'gate',json.loads(m.data)),10)
        self.create_subscription(MotionSimpleCMD,'/simple_cmd',self.operator_command,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client=self.create_client(SetBool,'/mission/set_armed')
        self.events=self.create_publisher(String,'/mission/session',10)
        self.create_timer(.2,self.tick)
        self.event('session_started',phase=self.phase)


    def event(self,kind,**data):
        e=dict(event=kind,stamp_ns=self.get_clock().now().nanoseconds,**data)
        with (self.folder/'session.jsonl').open('a') as f:f.write(json.dumps(e)+'\n')
        self.events.publish(String(data=json.dumps(e)))
        self.get_logger().info(json.dumps(e))

    def status(self,msg):
        self.state=json.loads(msg.data)
        pose=self.state.get('pose')
        if pose:
            if (self.last_pose is None or math.hypot(pose[0]-self.last_pose[0],pose[1]-self.last_pose[1])>.03
                    or abs(math.atan2(math.sin(pose[2]-self.last_pose[2]),math.cos(pose[2]-self.last_pose[2])))>.05):
                self.last_pose=pose;self.last_progress=time.monotonic()

    def new_mode_publisher(self):
        return self.create_publisher(MotionSimpleCMD,'/simple_cmd',QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,lifespan=Duration(seconds=.5)))

    def finish(self,reason):
        if self.done:return
        self.result=reason;self.done=True
        self.event('session_finished',reason=reason,mission=self.state)

    def operator_command(self,msg):
        if self.done or self.phase=='PREVIEW':return
        if msg.cmd_code in (0x21010C02,0x21020C0E):
            if self.client.service_is_ready():self.client.call_async(SetBool.Request(data=False))
            if self.mode_pub:self.destroy_publisher(self.mode_pub);self.mode_pub=None
            self.future=None;self.phase='WAIT_AUTO'
            self.event('operator_manual_or_stop')
        elif msg.cmd_code==0x21010C03 and self.phase in ('WAIT_AUTO','PAUSED'):
            self.auto_requested=True;self.started=time.monotonic();self.phase='ARMING'
            self.event('operator_auto_received')

    def tick(self):
        if self.done:return
        now=time.monotonic()
        if shutil.disk_usage(self.folder).free<int(self.get_parameter('minimum_free_disk_mb').value)*1024**2:
            self.finish('DISK_SPACE_LIMIT');return
        if self.phase in ('PREVIEW','WAIT_AUTO'):return
        if self.phase in ('PREPARE_HIGH_STEP','HIGH_STEP_REQUEST','ARMING') and now-self.started>self.get_parameter('startup_timeout_sec').value:
            self.phase='PAUSED';self.event('navigation_paused',reason='STARTUP_TIMEOUT: '+str(self.state.get('state')))
            if self.mode_pub:self.destroy_publisher(self.mode_pub);self.mode_pub=None
            return
        if self.run_started is not None and now-self.run_started>self.get_parameter('maximum_run_sec').value:
            self.finish('RUN_TIME_LIMIT');return
        if self.phase=='PREPARE_HIGH_STEP':
            self.mode_pub=self.new_mode_publisher();self.phase='HIGH_STEP_REQUEST';self.mode_time=now
        elif self.phase=='HIGH_STEP_REQUEST':
            if now-self.mode_time>5.:
                self.phase='PAUSED';self.event('navigation_paused',reason='HIGH_STEP_BRIDGE_UNAVAILABLE')
                self.destroy_publisher(self.mode_pub);self.mode_pub=None;return
            # One subscription is our own command listener; also require a receiver.
            if self.mode_pub.get_subscription_count()>=2:
                self.mode_pub.publish(MotionSimpleCMD(cmd_code=0x21010407,size=0,type=0))
                self.mode_time=now;self.phase='MODE_SETTLING';self.event('high_step_requested',code='0x21010407')
        elif self.phase=='MODE_SETTLING':
            if now-self.mode_time>.7:
                self.destroy_publisher(self.mode_pub);self.mode_pub=None;self.phase='WAIT_AUTO'
                self.get_logger().info('H-Step requested. Ready: run lite3_auto in your terminal.')
        elif self.phase=='ARMING':
            if self.future is not None:
                if not self.future.done():return
                response=self.future.result();self.future=None
                if response.success:
                    self.phase='RUNNING';self.last_progress=now;self.armed_time=now
                    if self.run_started is None:self.run_started=now
                    self.event('exploration_armed',gait='H-Step');return
            if self.gate.get('ready_to_arm') and self.client.service_is_ready():
                self.future=self.client.call_async(SetBool.Request(data=True))
        elif self.phase=='RUNNING':
            if now-self.armed_time<1.:return
            if not self.state.get('armed',False):
                reason=self.state.get('state','MISSION_DISARMED')
                if self.state.get('exit_found') or reason=='EXIT_FOUND':self.finish('EXIT_FOUND')
                else:
                    self.phase='PAUSED';self.event('navigation_paused',reason=reason)
                return
            if now-self.last_progress>self.get_parameter('no_progress_timeout_sec').value:
                if self.client.service_is_ready():self.client.call_async(SetBool.Request(data=False))
                self.phase='PAUSED';self.event('navigation_paused',reason='NO_MEASURED_PROGRESS')
        elif self.phase=='PAUSED':
            # New operator lite3_auto, or an explicit arm request, can resume.
            # Never switch out of H-Step or automatically fight teleoperation.
            if self.state.get('armed',False):
                self.phase='RUNNING';self.armed_time=now;self.last_progress=now
                self.event('exploration_resumed')

    def cleanup(self):
        self.done=True
        if self.mode_pub:self.destroy_publisher(self.mode_pub);self.mode_pub=None
        if self.client.service_is_ready():
            f=self.client.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(self,f,timeout_sec=2.)
        if self.auto_requested:
            pub=self.new_mode_publisher();end=time.monotonic()+1.
            while pub.get_subscription_count()<1 and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
            pub.publish(MotionSimpleCMD(cmd_code=0x21010C02,size=0,type=0))
            pub.wait_for_all_acked(Duration(seconds=.5));self.destroy_publisher(pub)
        # Save a standalone map before launch shutdown; no recorder is required.
        import subprocess
        try:subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(self.folder/'maze_map'),
                            '--ros-args','-p','save_map_timeout:=3.0'],timeout=5,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        except (OSError,subprocess.TimeoutExpired):pass
        (self.folder/'result.json').write_text(json.dumps(dict(reason=self.result or 'USER_STOP',mission=self.state),indent=2))


def main():
    from .runtime import initialize
    stop=initialize();node=MazeSession()
    try:
        while rclpy.ok() and not stop.is_set() and not node.done:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        if rclpy.ok():node.cleanup()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
