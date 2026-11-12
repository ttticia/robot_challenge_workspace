"""Observe hardware state without publishing any command or action goal."""
import json,time
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('hardware_stationary_final');data={};subs=[];counts={}
for t in ['/mission/status','/mission/gate_status']:
    subs.append(n.create_subscription(String,t,lambda m,t=t:data.update({t:json.loads(m.data)}),10))
for t,c in [('/map',OccupancyGrid),('/odom',Odometry),('/scan',LaserScan)]:
    def cb(m,t=t):counts[t]=counts.get(t,0)+1
    q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if t=='/map' else qos_profile_sensor_data
    subs.append(n.create_subscription(c,t,cb,q))
clients={k:n.create_client(GetState,'/'+k+'/get_state') for k in ['bt_navigator','controller_server','planner_server','velocity_smoother']};futures={}
end=time.monotonic()+8
while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=.1)
    for k,c in clients.items():
        if k not in futures and c.service_is_ready():futures[k]=c.call_async(GetState.Request())
for k,f in futures.items():
    if f.done():data[k]=f.result().current_state.label
for t in ['/cmd_vel','/simple_cmd']:data[t+'_publishers']=[e.node_name for e in n.get_publishers_info_by_topic(t)]
data['received_counts']=counts;data['alternate_sender_running']=any(k=='motion_sender' for k,_ in n.get_node_names_and_namespaces())
Path(__file__).with_name('stationary_acceptance.json').write_text(json.dumps(data,indent=2))
print(json.dumps(data,indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
