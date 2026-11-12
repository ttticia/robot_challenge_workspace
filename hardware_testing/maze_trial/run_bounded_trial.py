"""Supervise one existing mission. Uses mission arming, never publishes velocity."""
import json, math, os, time, signal, argparse
from pathlib import Path
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from transfer_interfaces.msg import MotionSimpleCMD

assert os.environ.get('ROS_DOMAIN_ID')=='4'
p=argparse.ArgumentParser();p.add_argument('--explore',action='store_true');a=p.parse_args()
rclpy.init();n=rclpy.create_node('bounded_hardware_trial');subs=[];state={};received={};odom=[];commands=[];tilts=[];events=[];shutdown=[False]
for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,lambda *_:shutdown.__setitem__(0,True))
def record(t,m):
    received[t]=time.monotonic()
    if t in ('/mission/status','/mission/gate_status'):state[t]=json.loads(m.data)
    elif t=='/odom':
        q=m.pose.pose.orientation
        odom.append((time.monotonic(),m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
    elif t=='/imu/data':
        q=m.orientation
        roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        tilts.append(max(abs(roll),abs(pitch)))
    elif t=='/cmd_vel':commands.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z))
for t,c in [('/mission/status',String),('/mission/gate_status',String),('/odom',Odometry),('/scan',LaserScan),('/imu/data',Imu),('/cmd_vel',Twist)]:
    subs.append(n.create_subscription(c,t,lambda m,t=t:record(t,m),qos_profile_sensor_data if t not in ('/mission/status','/mission/gate_status') else 10))
client=n.create_client(SetBool,'/mission/set_armed')
def spin(sec):
    end=time.monotonic()+sec
    while time.monotonic()<end and not shutdown[0]:rclpy.spin_once(n,timeout_sec=.02)
def event(k,**data):
    item=dict(event=k,wall_time=time.time(),**data);events.append(item);print(json.dumps(item),flush=True)
def arm(value):
    if not client.wait_for_service(timeout_sec=2):raise RuntimeError('Mission arm service unavailable')
    f=client.call_async(SetBool.Request(data=value));rclpy.spin_until_future_complete(n,f,timeout_sec=2)
    if not f.done():raise RuntimeError('Mission arm service timed out')
    event('arm_response',armed=value,success=f.result().success,message=f.result().message)
    if value and not f.result().success:raise RuntimeError(f.result().message)
def mode(code):
    pub=n.create_publisher(MotionSimpleCMD,'/simple_cmd',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,lifespan=Duration(seconds=.5)))
    try:
        end=time.monotonic()+3
        while time.monotonic()<end and pub.get_subscription_count()<1:spin(.05)
        if pub.get_subscription_count()!=1:raise RuntimeError('Expected one compatible mode command bridge')
        pub.publish(MotionSimpleCMD(cmd_code=code,size=0,type=0));ack=pub.wait_for_all_acked(Duration(seconds=1));event('mode_request',code=hex(code),bridge_ack=ack)
        if not ack:raise RuntimeError('Mode delivery not acknowledged')
        spin(.3)
    finally:n.destroy_publisher(pub)
result='NOT_STARTED';switched=False;armed=False;start=None;endpose=None;maximum_displacement=0.
try:
    spin(6)
    for t in ['/odom','/scan','/imu/data','/mission/status','/mission/gate_status']:
        if time.monotonic()-received.get(t,-math.inf)>.75:raise RuntimeError('Missing/stale '+t)
    if not odom or not tilts or tilts[-1]>math.radians(8):raise RuntimeError('Robot pose/tilt not ready')
    pubs=n.get_publishers_info_by_topic('/cmd_vel')
    if len(pubs)!=1 or pubs[0].node_name!='velocity_gate':raise RuntimeError('Unexpected velocity publisher ownership')
    bridge=[e for e in n.get_subscriptions_info_by_topic('/cmd_vel') if e.node_name!=n.get_name()]
    if len(bridge)!=1 or bridge[0].node_name!='lite3_domain_bridge_4':raise RuntimeError('Unexpected velocity subscriber route')
    if state.get('/mission/status',{}).get('armed'):raise RuntimeError('Mission was already armed')
    if not state.get('/mission/gate_status',{}).get('ready_to_arm'):raise RuntimeError('Gate not ready: '+str(state.get('/mission/gate_status')))
    if any(math.hypot(c[1],c[2])>1e-4 or abs(c[3])>1e-4 for c in commands):raise RuntimeError('Nonzero command before arming')
    mode(0x21010C03);switched=True;spin(1.5)
    if not state.get('/mission/gate_status',{}).get('ready_to_arm'):raise RuntimeError('Gate not ready after mode switch')
    start=odom[-1];command_start=len(commands);arm(True);armed=True;began=time.monotonic();result='MONITORING';nonzero=False
    event('trial_started',start_pose=start[1:],duration_limit_sec=45 if a.explore else 12)
    while time.monotonic()-began<(45 if a.explore else 12) and not shutdown[0]:
        spin(.05);now=time.monotonic()
        for t in ['/odom','/scan','/imu/data']:
            if now-received.get(t,-math.inf)>.5:raise RuntimeError('Sensor went stale: '+t)
        if now-received.get('/mission/status',-math.inf)>1.:raise RuntimeError('Mission status stale')
        if now-received.get('/mission/gate_status',-math.inf)>.7:raise RuntimeError('Gate status stale')
        if tilts[-1]>math.radians(10):raise RuntimeError('Tilt exceeds 10 degrees')
        endpose=odom[-1];displacement=math.hypot(endpose[1]-start[1],endpose[2]-start[2]);maximum_displacement=max(maximum_displacement,displacement)
        if displacement>(1.0 if a.explore else .22):raise RuntimeError('Trial travel bound reached')
        if commands[command_start:]:
            c=commands[-1];nonzero=nonzero or math.hypot(c[1],c[2])>.001 or abs(c[3])>.001
            if math.hypot(c[1],c[2])>.031 or abs(c[3])>.101:raise RuntimeError('Command exceeds configured trial limits')
        if now-began>6 and maximum_displacement<.015 and not a.explore:raise RuntimeError('No measurable translation after six seconds')
        s=state.get('/mission/status',{})
        if now-began>1 and not s.get('armed',False):result=s.get('state','DISARMED');break
    else:result='TIME_BOUND_STOP' if not shutdown[0] else 'INTERRUPTED'
    event('trial_finished',result=result,maximum_displacement=maximum_displacement,nonzero_command_observed=nonzero)
except Exception as exc:
    result='ABORTED: '+str(exc);event('trial_aborted',reason=str(exc))
finally:
    try:arm(False)
    except Exception as exc:event('disarm_failed',reason=str(exc))
    spin(.5)
    if switched:
        try:mode(0x21010C02)
        except Exception as exc:event('manual_mode_failed',reason=str(exc))
    spin(.5)
    report=dict(result=result,events=events,start_pose=start,end_pose=endpose,maximum_displacement_m=maximum_displacement,
                commands=commands,odom=odom,maximum_tilt_deg=math.degrees(max(tilts)) if tilts else None,last_state=state)
    Path(__file__).with_name('explore_result.json' if a.explore else 'movement_result.json').write_text(json.dumps(report,indent=2))
    n.destroy_node();rclpy.shutdown()
