"""Record SLAM/QR and run the existing autonomous exploration mission."""
import os,time,json,signal,subprocess,math,shutil
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from std_srvs.srv import SetBool
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from transfer_interfaces.msg import MotionSimpleCMD
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
stamp=time.strftime('%Y%m%d_%H%M%S',time.gmtime(time.time()+8*3600))
root=Path('/home/user/challenge_ws/recordings')/('maze_'+stamp+'_SGT');root.mkdir()
Path('/home/user/challenge_ws/recordings/LATEST_RUN').write_text(str(root))
processes={};logs=[];stop=[False];events=[];data={};odom=[];vel=[];qr=[]
def event(name,**v):
 x=dict(event=name,time=time.time(),**v);events.append(x);print(json.dumps(x),flush=True);(root/'status.json').write_text(json.dumps(dict(directory=str(root),events=events),indent=2))
def launch(name,cmd):
 f=(root/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True);processes[name]=p;return p
for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,lambda *_:stop.__setitem__(0,True))
launch('rosbag',['ros2','bag','record','-o',str(root/'slam_qr'),'/map','/map_updates','/tf','/tf_static','/scan','/odom','/imu/data','/camera/color/image_raw/compressed','/camera/color/camera_info','/qr/detections','/qr/debug_image/compressed','/mission/observations','/mission/status','/mission/gate_status','/cmd_vel'])
launch('qr',['ros2','run','team_perception','qr_scanner','--ros-args','-p','image_topic:=/camera/color/image_raw','-p','image_transport:=compressed','-p','use_depth:=false','-p','log_dir:='+str(root/'qr_evidence')])
launch('navigation',['ros2','launch','lite3_mission','hardware_navigation.launch.py','mode:=explore','execution:=true','drive:=true','team_qr:=true','config_file:=/home/user/challenge_ws/hardware_testing/record_explore.yaml'])
event('recording_started',path=str(root/'slam_qr'),planned_recording_seconds=120)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=rclpy.create_node('maze_recording_session');subs=[]
def observe(topic,m):
 if topic in ['/mission/status','/mission/gate_status']:data[topic]=json.loads(m.data)
 elif topic=='/odom':odom.append([time.monotonic(),m.pose.pose.position.x,m.pose.pose.position.y])
 elif topic=='/cmd_vel':vel.append([time.monotonic(),m.linear.x,m.linear.y,m.angular.z])
 elif topic=='/qr/detections':qr.append(m.data)
 elif topic=='/camera/color/image_raw/compressed':
  if not (root/'camera_start.jpg').exists():(root/'camera_start.jpg').write_bytes(m.data)
for t,c in [('/mission/status',String),('/mission/gate_status',String),('/odom',Odometry),('/cmd_vel',Twist),('/qr/detections',String),('/camera/color/image_raw/compressed',CompressedImage)]:subs.append(n.create_subscription(c,t,lambda m,t=t:observe(t,m),qos_profile_sensor_data if t in ['/odom','/cmd_vel','/camera/color/image_raw/compressed'] else 10))
client=n.create_client(SetBool,'/mission/set_armed')
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end and not stop[0]:rclpy.spin_once(n,timeout_sec=.05)
def armed(value):
 if not client.wait_for_service(timeout_sec=2):return False
 f=client.call_async(SetBool.Request(data=value));rclpy.spin_until_future_complete(n,f,timeout_sec=2)
 ok=f.done() and f.result().success;event('arming' if value else 'disarming',success=ok,message=f.result().message if f.done() else 'timeout');return ok
def mode(code):
 p=n.create_publisher(MotionSimpleCMD,'/simple_cmd',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,lifespan=Duration(seconds=.5)))
 try:
  end=time.monotonic()+3
  while time.monotonic()<end and p.get_subscription_count()<1:spin(.05)
  if p.get_subscription_count()!=1:raise RuntimeError('No unique mode-command bridge')
  p.publish(MotionSimpleCMD(cmd_code=code,size=0,type=0));ack=p.wait_for_all_acked(Duration(seconds=1));event('mode_request',code=hex(code),bridge_acknowledged=ack);spin(.3)
  return ack
 finally:n.destroy_publisher(p)
began=time.monotonic();auto=False;active=False
try:
 deadline=time.monotonic()+20
 while time.monotonic()<deadline and not stop[0]:
  spin(.1)
  if data.get('/mission/status',{}).get('pose') and data.get('/mission/gate_status',{}).get('ready_to_arm'):break
 if stop[0]:raise RuntimeError('Stopped by user')
 if data.get('/mission/status',{}).get('pose') and data.get('/mission/gate_status',{}).get('ready_to_arm'):
  auto=mode(0x21010C03);spin(1.0)
  if auto:
   for _ in range(8):
    active=armed(True)
    if active or stop[0]:break
    spin(1.)
 else:event('navigation_not_ready',state=data)
 while time.monotonic()-began<120 and not stop[0]:
  spin(.2)
  if processes['rosbag'].poll() is not None:raise RuntimeError('Recorder exited')
  if shutil.disk_usage(root).free<1024**3:raise RuntimeError('Stopping before disk space falls below 1GB')
  if active and time.monotonic()-began>15:
   distance=math.hypot(odom[-1][1]-odom[0][1],odom[-1][2]-odom[0][2]) if len(odom)>1 else 0
   if not any(e['event']=='motion_summary' for e in events):event('motion_summary',odom_net_displacement_m=distance,velocity_messages=len(vel),qr_detections=len(qr))
  if time.monotonic()-began>90 and active:
   armed(False);active=False
   if auto:mode(0x21010C02);auto=False
   event('exploration_window_finished')
except Exception as exc:event('session_error',reason=str(exc))
finally:
 try:armed(False)
 except Exception as exc:event('disarm_error',reason=str(exc))
 if auto:
  try:mode(0x21010C02)
  except Exception as exc:event('manual_mode_error',reason=str(exc))
 # Save the current map before stopping the mapper.
 if not stop[0]:
  try:subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(root/'maze_map'),'--ros-args','-p','save_map_timeout:=5.0'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=8)
  except Exception:pass
 for name in ['navigation','qr','rosbag']:
  p=processes.get(name)
  if p and p.poll() is None:p.send_signal(signal.SIGINT)
 for p in processes.values():
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:p.terminate()
 net=math.hypot(odom[-1][1]-odom[0][1],odom[-1][2]-odom[0][2]) if len(odom)>1 else 0
 event('session_finished',bag=str(root/'slam_qr'),odom_net_displacement_m=net,qr_detections=len(qr),nonzero_velocity_messages=sum(math.hypot(v[1],v[2])>.001 or abs(v[3])>.001 for v in vel),last_mission_state=data)
 (root/'telemetry.json').write_text(json.dumps(dict(odom=odom,cmd_vel=vel,qr=qr),indent=2))
 n.destroy_node();rclpy.shutdown()
