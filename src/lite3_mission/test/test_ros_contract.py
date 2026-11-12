"""ROS contract tests on an isolated loopback domain; no physical robot access.

Run explicitly with LITE3_ROS_TESTS=1 after sourcing ROS. The action server here
is a test double; successful tests are not presented as actual Nav2 planning.
"""
import json
import math
import os
import time
import numpy as np
import pytest

pytestmark=pytest.mark.skipif(os.getenv('LITE3_ROS_TESTS')!='1',reason='Explicit isolated ROS test run required')


@pytest.fixture
def context():
    if os.getenv('ROS_DOMAIN_ID')!='89' or 'cyclonedds-local.xml' not in os.getenv('CYCLONEDDS_URI',''):
        raise RuntimeError('ROS tests require isolated loopback domain 89; use scripts/test-mission.sh')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=['--ros-args','--disable-external-lib-logs','--disable-rosout-logs'],signal_handler_options=SignalHandlerOptions.NO)
    nodes=[]
    yield nodes
    for node in reversed(nodes):node.destroy_node()
    if rclpy.ok():rclpy.shutdown()


def spin(nodes,seconds,step=None):
    import rclpy
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if step:step()
        for node in nodes:
            for _ in range(8):rclpy.spin_once(node,timeout_sec=0)
        time.sleep(.003)


def make_world(nodes):
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from builtin_interfaces.msg import Time as MessageTime
    node=rclpy.create_node('test_world')
    nodes.append(node)
    map_pub=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    imu_pub=node.create_publisher(Imu,'/imu/data',10)
    broadcaster=TransformBroadcaster(node)
    clock=node.create_publisher(Clock,'/clock',10)
    started=time.monotonic()
    msg=OccupancyGrid()
    msg.header.frame_id='map'
    msg.info.width=msg.info.height=100
    msg.info.resolution=.04
    msg.info.origin.position.x=msg.info.origin.position.y=-2.
    msg.info.origin.orientation.w=1.
    msg.data=[0]*10000
    last=[0.]
    def publish():
        if time.monotonic()-last[0]<.05:return
        last[0]=time.monotonic()
        ns=int((1000.+time.monotonic()-started)*1e9)
        msg.header.stamp=MessageTime(sec=ns//10**9,nanosec=ns%10**9)
        clock.publish(Clock(clock=msg.header.stamp))
        map_pub.publish(msg)
        imu=Imu()
        imu.header.stamp=msg.header.stamp
        imu.header.frame_id='base_link'
        imu.orientation.w=1.
        imu_pub.publish(imu)
        tf=TransformStamped()
        tf.header.stamp=msg.header.stamp
        tf.header.frame_id='map'
        tf.child_frame_id='base_link'
        tf.transform.rotation.w=1.
        broadcaster.sendTransform(tf)
    return node,publish


def test_shadow_has_no_action_client_or_hardware_publisher(context):
    from lite3_mission.mission import Mission
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseStamped
    world,publish=make_world(context)
    mission=Mission(parameter_overrides=[Parameter('require_imu',value=False),Parameter('use_sim_time',value=True)])
    context.append(mission)
    goals=[]
    sub=world.create_subscription(PoseStamped,'/mission/candidate_goal',lambda m:goals.append(m),10)
    spin(context,2.,publish)
    assert mission.client is None and not mission.armed
    assert goals and goals[-1].pose.position.x==pytest.approx(.5)
    assert world.count_publishers('/cmd_vel')==0
    assert world.count_publishers('/mission/drive_lease')==0


def test_gate_limits_vector_and_stops_on_input_or_lease_timeout(context):
    import rclpy
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from lite3_mission.velocity_gate import VelocityGate
    node=rclpy.create_node('gate_test_io')
    context.append(node)
    gate=VelocityGate(parameter_overrides=[Parameter('drive_output_enabled',value=True),
        Parameter('input_topic',value='/test/nav_cmd'),Parameter('output_topic',value='/test/final_cmd')])
    context.append(gate)
    cmd=node.create_publisher(Twist,'/test/nav_cmd',1)
    lease=node.create_publisher(Bool,'/mission/drive_lease',1)
    outputs=[]
    sub=node.create_subscription(Twist,'/test/final_cmd',lambda m:outputs.append([m.linear.x,m.linear.y,m.angular.z]),10)
    last=[0.]
    def publish(command=True,lease_ok=True):
        if time.monotonic()-last[0]>.04:
            last[0]=time.monotonic()
            if command:
                msg=Twist();msg.linear.x=1.;msg.linear.y=1.;msg.angular.z=2.;cmd.publish(msg)
            if lease_ok:lease.publish(Bool(data=True))
        gate.tick()
    spin(context,2.,publish)
    assert outputs and np.linalg.norm(outputs[-1][:2])==pytest.approx(.1,abs=.002), (gate.sources,gate.conflicts,gate.lease_value,gate.target)
    assert outputs[-1][2]==pytest.approx(.25,abs=.002)
    spin(context,.65,lambda:publish(command=False))
    assert np.linalg.norm(outputs[-1])==0.
    spin(context,1.,publish)
    spin(context,.65,lambda:publish(lease_ok=False))
    assert np.linalg.norm(outputs[-1])==0.
    rogue=node.create_publisher(Twist,'/test/final_cmd',1)
    spin(context,.6,publish)
    assert gate.conflicts and not gate.last_output.any()


def test_capture_time_transform_and_report_dedup(context,tmp_path):
    from lite3_mission.observations import Observations
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import TransformStamped
    node=Observations(parameter_overrides=[Parameter('report_directory',value=str(tmp_path))])
    context.append(node)
    tf=TransformStamped()
    tf.header.frame_id='map';tf.child_frame_id='camera'
    tf.transform.rotation.w=1.
    tf.header.stamp.sec=1;tf.transform.translation.x=1.
    node.buffer.set_transform(tf,'test')
    tf.header.stamp.sec=2;tf.transform.translation.x=5.
    node.buffer.set_transform(tf,'test')
    event=dict(kind='qr',label='checkpoint_A',frame_id='camera',stamp={'sec':1,'nanosec':0},position=[.2,0.,1.])
    node.enqueue(event);node.process()
    node.enqueue(event);node.process()
    result=json.loads(node.path.read_text())
    assert len(result['records'])==1
    assert result['records'][0]['position']==pytest.approx([1.2,0.,1.])
    assert result['records'][0]['observations']==2


def test_existing_team_scanner_to_adapter_to_report(context,tmp_path):
    """Run the unchanged team's real scanner on generated QR pixels, no camera."""
    import signal
    import subprocess
    import sys
    import rclpy
    import qrcode
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Image
    from lite3_mission.observations import Observations
    pytest.importorskip('team_perception.qr_scanner_node')
    observer=rclpy.create_node('team_qr_pipeline_test');context.append(observer)
    report=Observations(parameter_overrides=[Parameter('report_directory',value=str(tmp_path/'reports'))])
    context.append(report)
    camera=observer.create_publisher(Image,'/test/team_camera',10)
    commands=[
        [sys.executable,'-c','from team_perception.qr_scanner_node import main; main()',
         '--ros-args','-p','use_depth:=false','-p','publish_debug:=false',
         '-p','image_topic:=/test/team_camera','-p','log_dir:='+str(tmp_path/'original_qr_logs')],
        [sys.executable,'-c','from lite3_mission.team_qr_adapter import main; main()']]
    processes=[]
    try:
        for command in commands:
            processes.append(subprocess.Popen(command,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                                              start_new_session=True))
        pixels=np.array(qrcode.make('TEAM:checkpoint=C').convert('RGB'))
        image=Image();image.header.frame_id='camera_color_optical_frame';image.header.stamp.sec=123
        image.height,image.width=pixels.shape[:2];image.encoding='rgb8';image.step=image.width*3
        image.data=pixels.tobytes()
        last=[0.]
        def publish():
            assert all(p.poll() is None for p in processes),'QR scanner or adapter exited early'
            if time.monotonic()-last[0]>.2:
                last[0]=time.monotonic();camera.publish(image)
        spin(context,5.,publish)
        evidence=[e for e in report.unlocalized if e['label']=='TEAM:checkpoint=C']
        assert evidence and evidence[0]['source']=='team_perception'
        assert evidence[0]['stamp']==dict(sec=123,nanosec=0)
        assert 'position' not in evidence[0]
        assert list((tmp_path/'original_qr_logs').glob('*.csv'))
        assert list((tmp_path/'original_qr_logs').glob('*.jpg'))
        assert observer.count_publishers('/cmd_vel')==0
    finally:
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2.)


def test_cancel_while_goal_acceptance_pending(context):
    import threading
    import rclpy
    from rclpy.action import ActionServer,GoalResponse,CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from nav2_msgs.action import NavigateToPose
    from std_srvs.srv import SetBool
    from lite3_mission.mission import Mission
    world,publish=make_world(context)
    server_node=rclpy.create_node('test_nav2_action_server')
    context.append(server_node)
    accepted=[];cancelled=[]
    def goal_callback(goal):
        time.sleep(.25)
        accepted.append(goal)
        return GoalResponse.ACCEPT
    def execute(handle):
        until=time.monotonic()+3.
        while time.monotonic()<until:
            if handle.is_cancel_requested:
                cancelled.append(True);handle.canceled();return NavigateToPose.Result()
            time.sleep(.01)
        handle.succeed();return NavigateToPose.Result()
    server=ActionServer(server_node,NavigateToPose,'/navigate_to_pose',execute_callback=execute,
        goal_callback=goal_callback,cancel_callback=lambda goal:CancelResponse.ACCEPT,
        callback_group=ReentrantCallbackGroup())
    mission=Mission(parameter_overrides=[Parameter('execution_enabled',value=True),
        Parameter('require_velocity_gate',value=False),Parameter('require_imu',value=False),
        Parameter('require_nav2_active',value=False),Parameter('use_sim_time',value=True)])
    context.append(mission)
    spin([world,mission],1.2,publish)
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(server_node)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    try:
        spin([world,mission],.5,publish)
        response=mission.arm(SetBool.Request(data=True),SetBool.Response())
        assert response.success,response.message
        # Submit directly to ensure disarm happens inside the server's acceptance delay.
        mission.dispatch((.5,0.,0.))
        mission.disarm('TEST_DISARM_DURING_ACCEPTANCE')
        spin([world,mission],1.5,publish)
        assert accepted and cancelled
        assert not mission.armed and mission.goal_handle is None and mission.pending is None
    finally:
        executor.shutdown(timeout_sec=4.)
        worker.join(timeout=4.)
        server.destroy()


def test_real_qr_pixels_produce_text_without_inventing_position(context):
    import rclpy
    import qrcode
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from lite3_mission.qr import QR
    node=rclpy.create_node('qr_test_observer');context.append(node)
    qr=QR();context.append(qr)
    events=[]
    sub=node.create_subscription(String,'/mission/observations',lambda m:events.append(json.loads(m.data)),10)
    pixels=np.array(qrcode.make('LITE3:checkpoint=test').convert('RGB'))
    msg=Image();msg.header.frame_id='camera_color_optical_frame';msg.header.stamp.sec=1
    msg.height,msg.width=pixels.shape[:2];msg.encoding='rgb8';msg.step=msg.width*3;msg.data=pixels.tobytes()
    spin(context,.4)
    qr.image(msg)
    spin(context,.4)
    assert events and events[-1]['label']=='LITE3:checkpoint=test'
    assert 'position' not in events[-1]  # Physical QR size was not provided.


def test_scene_adapter_uses_documented_client_cli_without_network(context,tmp_path):
    from lite3_mission.scene_client import SceneClient
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Image
    client=tmp_path/'client.py'
    client.write_text('''import argparse,json,cv2
p=argparse.ArgumentParser()
for name in ['server','group','image','prompt-file']:p.add_argument('--'+name,required=True)
a=p.parse_args();im=cv2.imread(a.image)
assert im.shape[1]<=1024 and im.shape[0]<=768
assert a.group=='group04'
print('HTTP 200 OK')
print(json.dumps({'success':True,'result':json.dumps({'room_type':'kitchen','survivor_present':False,'confidence':0.9,'evidence':['sink']})}))
''')
    scene=SceneClient(parameter_overrides=[Parameter('client_path',value=str(client))])
    context.append(scene)
    image=np.zeros((900,1200,3),np.uint8)
    msg=Image();msg.height=900;msg.width=1200;msg.encoding='bgr8';msg.step=3600;msg.data=image.tobytes()
    _,result=scene.request(msg)
    assert result['room_type']=='kitchen' and not result['survivor_present']
    scene.pool.shutdown(wait=True)
