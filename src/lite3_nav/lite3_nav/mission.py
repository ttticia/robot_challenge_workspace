# Fork of lite3_mission/mission.py (main e725625) with:
#  - nav2_health_check (lifecycle | action_server | either, default lifecycle) and
#    nav2_active_timeout_sec (default 3.0, the value main hard-codes); fresh goal feedback
#    from an accepted goal always counts as Nav2 active.
#  - A lost lifecycle get_state response is dropped after 2 s instead of blocking all later queries.
#  - Freshness checks (IMU, lifecycle replies, goal timeout) use sim time under use_sim_time,
#    wall/monotonic time otherwise (unchanged on the robot).
#  - The 10 Hz tick (and the drive lease it publishes) runs on a steady clock even under sim time.
#  - core and runtime helpers are imported from the team's lite3_mission package unchanged
#    (absolute imports instead of relative ones).
"""Nav2 goal coordinator. Shadow mode creates no action client or drive lease."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from tf2_ros import Buffer, TransformListener, TransformException
from lite3_mission.core import Grid, relative_goal, select_frontier, yaw_from_quaternion


class Mission(Node):
    def __init__(self,**kwargs):
        super().__init__('mission',**kwargs)
        defaults = dict(execution_enabled=False, mode='waypoints', map_topic='/map', map_frame='map',
                        base_frame='base_link', imu_topic='/imu/data', nav_action='/navigate_to_pose',
                        pose_timeout_sec=.75, imu_timeout_sec=.5, maximum_tilt_deg=12.,
                        require_imu=True, require_velocity_gate=True, require_nav2_active=True,
                        nav2_lifecycle_node='/bt_navigator',goal_timeout_sec=45.,nav2_active_timeout_sec=3.,
                        frontier_clearance_m=.25, frontier_standoff_m=.32, minimum_frontier_size_m=.12,
                        blacklist_duration_sec=30., waypoints=[.5,0.,0.], waypoint_frame='initial_base',
                        map_transient_local=True,maximum_consecutive_failures=3,frontier_yaw_mode='keep_current',
                        nav2_health_check='lifecycle')
        immutable = {'execution_enabled','map_topic','map_frame','base_frame','imu_topic','nav_action',
                     'map_transient_local','require_velocity_gate','require_nav2_active','nav2_lifecycle_node',
                     'nav2_health_check'}
        for key,value in defaults.items():
            self.declare_parameter(key,value,ParameterDescriptor(read_only=key in immutable))
        result = self.validate([])
        if not result.successful:
            raise ValueError(result.reason)
        self.add_on_set_parameters_callback(self.validate)
        self.execute = self.value('execution_enabled')
        self.grid = self.map_error = self.pose = self.anchor = None
        self.imu_received = self.gate_received = -math.inf
        self.tilt = math.inf
        self.gate = {}
        self.armed = False
        self.epoch = self.waypoint_index = 0
        self.consecutive_failures=0
        self.goal_handle = self.pending = None
        self.goal = None
        self.cancel_requested = False
        self.goal_started = self.last_preview = self.last_status = 0.
        self.blacklist = []
        self.note = 'SHADOW' if not self.execute else 'DISARMED'
        self.buffer = Buffer(cache_time=Duration(seconds=30.))
        self.listener = TransformListener(self.buffer,self)
        map_qos = QoSProfile(depth=1,durability=(DurabilityPolicy.TRANSIENT_LOCAL if self.value('map_transient_local')
                                               else DurabilityPolicy.VOLATILE))
        self.create_subscription(OccupancyGrid,self.value('map_topic'),self.on_map,map_qos)
        self.create_subscription(Imu,self.value('imu_topic'),self.on_imu,qos_profile_sensor_data)
        self.create_subscription(String,'/mission/gate_status',self.on_gate,10)
        self.preview = self.create_publisher(PoseStamped,'/mission/candidate_goal',1)
        self.status = self.create_publisher(String,'/mission/status',10)
        self.lease = self.create_publisher(Bool,'/mission/drive_lease',1) if self.execute else None
        self.create_service(SetBool,'/mission/set_armed',self.arm)
        self.client = ActionClient(self,NavigateToPose,self.value('nav_action')) if self.execute else None
        # Without an action client (shadow mode) only the lifecycle check is meaningful.
        self.health_mode=self.value('nav2_health_check') if self.execute else 'lifecycle'
        self.lifecycle=(self.create_client(GetState,self.value('nav2_lifecycle_node')+'/get_state')
                        if self.execute and self.value('require_nav2_active') and self.health_mode!='action_server'
                        else None)
        self.lifecycle_future=None
        self.lifecycle_active=False
        self.lifecycle_received=self.lifecycle_requested=self.feedback_received=-math.inf
        # Steady (wall) timer even under use_sim_time: the drive lease it publishes is checked by
        # the gate on the wall clock, and a slow simulator must not starve it. Freshness checks
        # inside tick use sensor_now(). On the robot (no sim time) this is unchanged.
        self.timer = self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Mission ready: '+('explicit arming required' if self.execute else 'SHADOW: no Nav2 goals sent'))

    def value(self,name):
        return self.get_parameter(name).value

    def sensor_now(self):
        """Clock for sensor freshness: wall (monotonic) on the robot; sim time under use_sim_time,
        so a frozen simulator (no new IMU because no sim time passed) is not reported as stale."""
        if self.get_parameter('use_sim_time').value:
            return self.get_clock().now().nanoseconds*1e-9
        return time.monotonic()

    def nav2_active(self):
        """Nav2 liveness for arming and while armed (nav2_health_check: lifecycle | action_server | either).

        A goal Nav2 accepted and is still sending feedback for proves bt_navigator is active in any
        mode. action_server alone is weaker: in Nav2 Humble the navigate_to_pose server exists from
        on_configure and only rejects goals until on_activate, so a configured-but-inactive
        bt_navigator also looks ready; its goals are then rejected and count as failed goals.
        Ages use sensor_now(): the requests come from the tick timer, which runs on sim time.
        """
        now=self.sensor_now()
        limit=self.value('nav2_active_timeout_sec')
        if self.goal_handle and now-self.feedback_received<=limit:
            return True
        lifecycle_ok=self.lifecycle_active and now-self.lifecycle_received<=limit
        server_ok=self.client is not None and self.client.server_is_ready()
        if self.health_mode=='action_server':return server_ok
        if self.health_mode=='either':return lifecycle_ok or server_ok
        return lifecycle_ok

    def on_feedback(self,unused):
        self.feedback_received=self.sensor_now()

    def validate(self,parameters):
        values = {p.name:p.value for p in parameters}
        def v(name):return values.get(name,self.value(name))
        try:
            if v('mode') not in ('waypoints','explore') or v('waypoint_frame') not in ('map','initial_base'):
                raise ValueError('mode must be waypoints/explore; waypoint_frame map/initial_base')
            if v('nav2_health_check') not in ('lifecycle','action_server','either'):
                raise ValueError('nav2_health_check must be lifecycle, action_server or either')
            if v('frontier_yaw_mode') not in ('keep_current','face_frontier'):
                raise ValueError('frontier_yaw_mode must be keep_current or face_frontier')
            if not 1<=v('maximum_consecutive_failures')<=10:
                raise ValueError('maximum_consecutive_failures must be 1–10')
            waypoints = v('waypoints')
            if len(waypoints)%3 or not np.isfinite(waypoints).all():
                raise ValueError('waypoints must be a flat list of x,y,yaw triples in radians')
            for name,low,high in [('pose_timeout_sec',.1,5.),('imu_timeout_sec',.1,5.),
                                  ('maximum_tilt_deg',1.,30.),('goal_timeout_sec',5.,600.),
                                  ('nav2_active_timeout_sec',1.,30.),
                                  ('frontier_clearance_m',.1,.6),('frontier_standoff_m',.15,1.),
                                  ('minimum_frontier_size_m',.04,2.),('blacklist_duration_sec',1.,600.)]:
                if not low <= float(v(name)) <= high:
                    raise ValueError(name+' outside supported bounds')
            if getattr(self,'armed',False) and any(p.name in ('mode','waypoints','waypoint_frame') for p in parameters):
                raise ValueError('Disarm before changing the mission or waypoint list')
        except (ValueError,TypeError) as exc:
            return SetParametersResult(successful=False,reason=str(exc))
        if parameters and hasattr(self,'waypoint_index') and any(p.name in ('mode','waypoints','waypoint_frame') for p in parameters):
            self.waypoint_index=0
            self.consecutive_failures=0
            self.anchor=None
        return SetParametersResult(successful=True)

    def on_map(self,msg):
        try:
            if msg.header.frame_id != self.value('map_frame'):
                raise ValueError('Map frame '+msg.header.frame_id+' does not match map_frame')
            q=msg.info.origin.orientation
            self.grid=Grid(np.asarray(msg.data).reshape(msg.info.height,msg.info.width),msg.info.resolution,
                           (msg.info.origin.position.x,msg.info.origin.position.y),yaw_from_quaternion([q.x,q.y,q.z,q.w]))
            self.map_error=None
        except (ValueError,TypeError) as exc:
            self.grid=None
            self.map_error=str(exc)

    def on_imu(self,msg):
        q=msg.orientation
        try:
            yaw_from_quaternion([q.x,q.y,q.z,q.w])
            roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
            pitch=math.asin(np.clip(2*(q.w*q.y-q.z*q.x),-1.,1.))
            self.tilt=max(abs(roll),abs(pitch))
            self.imu_received=self.sensor_now()
        except ValueError:
            self.tilt=math.inf

    def on_gate(self,msg):
        try:
            self.gate=json.loads(msg.data)
            self.gate_received=time.monotonic()
        except (ValueError,TypeError):
            self.gate={}

    def health(self):
        if self.grid is None:
            return self.map_error or 'WAITING_FOR_MAP'
        try:
            tf=self.buffer.lookup_transform(self.value('map_frame'),self.value('base_frame'),Time())
            age=(self.get_clock().now().nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)*1e-9
            if age < -.1 or age > self.value('pose_timeout_sec'):
                return 'POSE_TIMESTAMP_STALE_OR_FUTURE'
            p,q=tf.transform.translation,tf.transform.rotation
            self.pose=(p.x,p.y,yaw_from_quaternion([q.x,q.y,q.z,q.w]))
        except (TransformException,ValueError):
            return 'WAITING_FOR_POSE_TF'
        if self.value('require_imu'):
            if self.sensor_now()-self.imu_received > self.value('imu_timeout_sec'):
                return 'IMU_STALE_OR_INVALID'
            if self.tilt > math.radians(self.value('maximum_tilt_deg')):
                return 'EXCESS_TILT'
        return None

    def arm(self,request,response):
        if not request.data:
            self.disarm('DISARMED_BY_OPERATOR')
            response.success=True
            response.message='Disarmed; this mission goal is being cancelled'
            return response
        reason=self.health()
        if not self.execute:reason='Shadow mode cannot arm; relaunch with execution:=true'
        elif not reason and self.value('require_velocity_gate'):
            if time.monotonic()-self.gate_received > .75 or not self.gate.get('ready_to_arm',False):
                reason='VELOCITY_GATE_NOT_READY: '+str(self.gate.get('reason','no gate status'))
        if not reason and not self.client.server_is_ready():reason='NAV2_ACTION_SERVER_UNAVAILABLE'
        if not reason and self.value('require_nav2_active') and not self.nav2_active():
            reason='NAV2_BT_NAVIGATOR_NOT_ACTIVE'
        if self.pending or self.goal_handle:reason=reason or 'WAIT_FOR_PREVIOUS_GOAL_TO_FINISH_CANCELLING'
        response.success=reason is None
        response.message=reason or 'Armed; Nav2 goals may now execute'
        if response.success:
            self.armed=True
            self.epoch+=1
            self.anchor=self.pose
            self.waypoint_index=0
            self.consecutive_failures=0
            self.note='ARMED'
        return response

    def disarm(self,reason):
        self.armed=False
        self.epoch+=1
        self.note=reason
        if self.lease:self.lease.publish(Bool(data=False))
        if self.goal_handle and not self.cancel_requested:
            self.cancel_requested=True
            self.goal_handle.cancel_goal_async()

    def choose_goal(self):
        if self.value('mode')=='waypoints':
            values=self.value('waypoints')
            index=3*self.waypoint_index
            if index>=len(values):return None,'MISSION_COMPLETE'
            if self.anchor is None:self.anchor=self.pose
            goal=tuple(values[index:index+3])
            return (relative_goal(self.anchor,goal) if self.value('waypoint_frame')=='initial_base' else goal),'WAYPOINT'
        now=time.monotonic()
        self.blacklist=[(p,until) for p,until in self.blacklist if until>now]
        goal,state=select_frontier(self.grid,self.pose[:2],self.value('frontier_clearance_m'),
                               self.value('frontier_standoff_m'),self.value('minimum_frontier_size_m'),
                               [p for p,until in self.blacklist])
        if goal and self.value('frontier_yaw_mode')=='keep_current':goal=(goal[0],goal[1],self.pose[2])
        return goal,state

    def stamped_goal(self,goal):
        msg=PoseStamped()
        msg.header.frame_id=self.value('map_frame')
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=map(float,goal[:2])
        msg.pose.orientation.z,msg.pose.orientation.w=math.sin(goal[2]/2),math.cos(goal[2]/2)
        return msg

    def dispatch(self,goal):
        epoch=self.epoch
        self.goal=goal
        self.goal_started=self.sensor_now()
        message=NavigateToPose.Goal(pose=self.stamped_goal(goal))
        self.pending=self.client.send_goal_async(message,feedback_callback=self.on_feedback)
        self.pending.add_done_callback(lambda future:self.accepted(future,epoch))

    def accepted(self,future,epoch):
        self.pending=None
        try:handle=future.result()
        except Exception as exc:
            self.disarm('GOAL_SEND_FAILED: '+str(exc))
            return
        if not handle.accepted:
            self.failed_goal('NAV2_REJECTED_GOAL')
            return
        self.goal_handle=handle
        self.feedback_received=self.sensor_now()  # accepted = bt_navigator is active
        self.cancel_requested=False
        handle.get_result_async().add_done_callback(lambda future:self.finished(future,epoch))
        if not self.armed or epoch!=self.epoch:
            self.cancel_requested=True
            handle.cancel_goal_async()

    def finished(self,future,epoch):
        self.goal_handle=None
        self.cancel_requested=False
        try:status=future.result().status
        except Exception as exc:
            self.disarm('RESULT_FAILED: '+str(exc))
            return
        if epoch!=self.epoch or not self.armed:return
        if status==GoalStatus.STATUS_SUCCEEDED:
            self.consecutive_failures=0
            self.note='GOAL_SUCCEEDED'
            if self.value('mode')=='waypoints':self.waypoint_index+=1
            elif self.goal:self.blacklist.append((self.goal[:2],time.monotonic()+self.value('blacklist_duration_sec')))
        else:self.failed_goal('NAV2_GOAL_FAILED_'+str(status))

    def failed_goal(self,reason):
        self.consecutive_failures+=1
        if self.consecutive_failures>=self.value('maximum_consecutive_failures'):
            self.disarm('CONSECUTIVE_GOAL_FAILURES: '+reason)
            return
        if self.value('mode')=='explore' and self.goal:
            self.blacklist.append((self.goal[:2],time.monotonic()+self.value('blacklist_duration_sec')))
            self.note=reason
        else:self.disarm(reason)

    def tick(self):
        now=time.monotonic()
        sensor_now=self.sensor_now()
        if self.lifecycle_future is not None and sensor_now-self.lifecycle_requested>2.:
            # A lost service response (seen with Fast DDS on WSL and on the lab network) would
            # otherwise block every later query and leave Nav2 looking inactive forever.
            self.lifecycle.remove_pending_request(self.lifecycle_future)
            self.lifecycle_future=None
        if self.lifecycle and self.lifecycle_future is None and sensor_now-self.lifecycle_requested>1. and self.lifecycle.service_is_ready():
            self.lifecycle_requested=sensor_now
            self.lifecycle_future=self.lifecycle.call_async(GetState.Request())
            self.lifecycle_future.add_done_callback(self.lifecycle_result)
        reason=self.health()
        if self.armed and self.value('require_nav2_active') and not self.nav2_active():
            reason=reason or 'NAV2_BT_NAVIGATOR_NOT_ACTIVE'
        if self.armed and reason:self.disarm(reason)
        if self.armed and (self.pending or self.goal_handle) and sensor_now-self.goal_started>self.value('goal_timeout_sec'):
            self.disarm('GOAL_TIMEOUT')
        # Only a live, accepted goal receives a drive lease. Loss of this process
        # expires at the independent gate; Nav2 alone cannot keep driving.
        if self.lease:
            self.lease.publish(Bool(data=bool(self.execute and self.armed and self.goal_handle and not self.cancel_requested and not reason)))
        if not reason and not self.pending and not self.goal_handle and now-self.last_preview>=1.:
            self.last_preview=now
            goal,state=self.choose_goal()
            if goal:
                self.preview.publish(self.stamped_goal(goal))
                if self.execute and self.armed:
                    if self.client.server_is_ready():self.dispatch(goal)
                    else:self.disarm('NAV2_ACTION_SERVER_UNAVAILABLE')
            elif self.armed:self.disarm(state)
            if not self.execute:self.note='SHADOW_'+state
        if now-self.last_status>=.5:
            self.last_status=now
            message={'state':reason or self.note,'armed':self.armed,'execution_enabled':self.execute,
                     'active_goal':self.goal if (self.pending or self.goal_handle) else None,
                     'pose':self.pose,'waypoint_index':self.waypoint_index,'mode':self.value('mode')}
            self.status.publish(String(data=json.dumps(message)))

    def lifecycle_result(self,future):
        self.lifecycle_future=None
        try:
            self.lifecycle_active=future.result().current_state.id==State.PRIMARY_STATE_ACTIVE
            self.lifecycle_received=self.sensor_now()
        except Exception:
            self.lifecycle_active=False


def main():
    from lite3_mission.runtime import initialize
    stop=initialize()
    node=Mission()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=0)
            time.sleep(.002)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            node.disarm('SHUTDOWN')
            end=time.monotonic()+.5
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0);time.sleep(.005)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
