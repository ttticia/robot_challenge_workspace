# Fork of lite3_mission/mission.py (main e725625) with:
#  - nav2_health_check (lifecycle | action_server | either, default lifecycle) and
#    nav2_active_timeout_sec (default 3.0, the value main hard-codes); fresh goal feedback
#    from an accepted goal always counts as Nav2 active.
#  - A lost lifecycle get_state response is dropped after 2 s instead of blocking all later queries.
#  - Freshness checks (IMU, lifecycle replies, goal timeout) use sim time under use_sim_time,
#    wall/monotonic time otherwise (unchanged on the robot).
#  - The 10 Hz tick (and the drive lease it publishes) runs on a steady clock even under sim time.
#  - core and runtime helpers come from lite3_nav's own unchanged copies (lite3_nav.core, .runtime)
#    with absolute imports instead of relative ones.
#  - START_NOT_CLEAR retry from the nearest clear cell (nearest_clear_start).
#  - Exit / final cell (lite3_nav.exit_detect, borrowed from maze_nav, origin/navigation 419f205):
#    explore mode ends as EXITED once the robot is out through the arena opening (or, in any mode,
#    within final_cell_radius of final_cell_xy): goal cancelled, drive lease dropped, disarmed,
#    no more goals and no re-arming until relaunch or a mode change. Frontiers on the open floor
#    beyond the exit are only chosen once nothing else is left (exit_frontiers_last).
#  - GOAL_TIMEOUT no longer ends the run: the stuck goal is cancelled and counted as a failed goal
#    (explore: blacklisted, next frontier; disarm after maximum_consecutive_failures in a row).
#  - final_cell_xy loaded as None (an empty list in a params file) counts as unset.
#  - frontier_ranking: path_length (default; lite3_nav.frontier, nearest along free space, from
#    maze_nav) or score (the team's select_frontier: size over straight-line distance).
#  - Start routine (start_routine, from maze_nav): on the first arming in explore mode the robot
#    creeps start_creep_m straight ahead (Nav2's drive_on_heading behavior; a NavigateToPose goal
#    that short is already inside the goal tolerance), then turns start_scan_deg left, twice that
#    right and back (Nav2's spin behavior) so SLAM sees the start cell before frontiers are chosen.
#    Arming is the start signal. A failed or timed-out step is logged and skipped, never counted
#    as a failed goal.
#  - auto_stand_up (robot.yaml true, sim.yaml false): arming first checks the posture from the
#    lidar (lite3_nav.stand) and, only if the robot is clearly lying, sends the stand/sit toggle
#    0x21010202 on /simple_cmd once; once it reads as standing the mission arms itself as soon as
#    the normal arming checks pass (map, pose, gate, Nav2), within stand_settle_sec. A standing
#    robot never gets the toggle. robot_info.md 6.3 left standing to the operator; the team chose
#    auto stand-up on 2026-11-15 because the handheld controller is ignored in auto mode.
"""Nav2 goal coordinator. Shadow mode creates no action client or drive lease."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
from builtin_interfaces.msg import Duration as DurationMsg
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from tf2_ros import Buffer, TransformListener, TransformException
from lite3_nav.core import Grid, relative_goal, select_frontier, yaw_from_quaternion
from scipy.ndimage import distance_transform_edt
from lite3_nav.exit_detect import ExitDetector, exit_masked_grid, seen_free_clearance
from lite3_nav.frontier import select_frontier_by_path
from lite3_nav.stand import STAND_TOGGLE, StandUp, body_height, quaternion_matrix
from lite3_nav.wall_band import cloud_xyz


def nearest_clear_start(grid, xy, clearance, max_shift=.6):
    """Closest cell that passes select_frontier's start check, within max_shift metres."""
    known_free = (grid.data >= 0) & (grid.data <= 25)
    safe = known_free & (distance_transform_edt(known_free)*grid.resolution > clearance)
    cells = np.argwhere(safe)
    if not len(cells):
        return None
    points = grid.world(cells[:, ::-1])
    distance = np.linalg.norm(points-np.asarray(xy), axis=1)
    i = int(np.argmin(distance))
    return None if distance[i] > max_shift else (float(points[i][0]), float(points[i][1]))

class Mission(Node):
    def __init__(self,**kwargs):
        super().__init__('mission',**kwargs)
        defaults = dict(execution_enabled=False, mode='waypoints', map_topic='/map', map_frame='map',
                        base_frame='base_link', imu_topic='/imu/data', nav_action='/navigate_to_pose',
                        pose_timeout_sec=.75, imu_timeout_sec=.5, maximum_tilt_deg=12.,
                        require_imu=True, require_velocity_gate=True, require_nav2_active=True,
                        nav2_lifecycle_node='/bt_navigator',goal_timeout_sec=45.,nav2_active_timeout_sec=3.,
                        frontier_clearance_m=.25, frontier_standoff_m=.32, minimum_frontier_size_m=.12,
                        frontier_planner_radius_m=0., minimum_goal_motion_m=.1, minimum_goal_hop_m=0.,
                        blacklist_duration_sec=30., waypoints=[.5,0.,0.], waypoint_frame='initial_base',
                        map_transient_local=True,maximum_consecutive_failures=3,frontier_yaw_mode='keep_current',
                        nav2_health_check='lifecycle', frontier_ranking='path_length',
                        # Exit / final cell (lite3_nav.exit_detect); defaults = maze_nav's.
                        exit_detection_enabled=True, exit_open_radius=1.1, exit_near_radius=.8,
                        exit_min_displacement=1.5, exit_enclosed_radius=.7, exit_dwell_sec=2.,
                        exit_frontiers_last=True, final_cell_xy=[], final_cell_radius=.4,
                        # Start routine (explore mode, first arming only).
                        start_routine=True, start_creep_m=.1, start_scan_deg=45.,
                        start_creep_speed=.05, spin_action='/spin', drive_on_heading_action='/drive_on_heading',
                        # Auto stand-up on arming (lite3_nav.stand).
                        auto_stand_up=False, stand_cloud_topic='/livox/points', simple_cmd_topic='/simple_cmd',
                        stand_timeout_sec=15., stand_settle_sec=30.)
        immutable = {'execution_enabled','map_topic','map_frame','base_frame','imu_topic','nav_action',
                     'map_transient_local','require_velocity_gate','require_nav2_active','nav2_lifecycle_node',
                     'nav2_health_check','spin_action','drive_on_heading_action',
                     'auto_stand_up','stand_cloud_topic','simple_cmd_topic'}
        for key,value in defaults.items():
            # final_cell_xy: empty list (unset) or [x, y]; an empty default has no fixed ROS type.
            self.declare_parameter(key,value,ParameterDescriptor(read_only=key in immutable,
                                                                 dynamic_typing=key=='final_cell_xy'))
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
        self.unresolvable = []  # frontier goals reached without moving: skipped for the whole run
        self.exit = ExitDetector()
        self.exited = False
        self.run_started = False  # first arming of this mission: the exit test's start pose
        self.last_exit_check = -math.inf
        self.last_exit_log = -math.inf
        self.clearance_cache = (None, None)  # (grid, seen_free_clearance(grid))
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
        # Start routine behaviors (Nav2 behavior_server): kind -> action client.
        self.behaviors = ({'START_SCAN':ActionClient(self,Spin,self.value('spin_action')),
                           'START_CREEP':ActionClient(self,DriveOnHeading,self.value('drive_on_heading_action'))}
                          if self.execute else {})
        self.start_steps = []  # start routine still to run: [(kind, value)]
        self.goal_kind = 'explore'  # 'start' while a creep or scan turn runs
        # Auto stand-up: self.stand runs the posture check, then settle_until bounds the wait for
        # the normal arming checks. Neither phase is armed: no lease, no goals.
        self.stand = self.settle_until = self.stand_cloud = None
        self.simple_cmd_pub = self.simple_cmd_type = None
        if self.execute and self.value('auto_stand_up'):
            try:
                from transfer_interfaces.msg import MotionSimpleCMD
                self.simple_cmd_type = MotionSimpleCMD
                # Created at startup: the Lite3 domain bridge only subscribes once it sees a publisher.
                self.simple_cmd_pub = self.create_publisher(MotionSimpleCMD,self.value('simple_cmd_topic'),
                    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,lifespan=Duration(seconds=1.)))
            except ImportError:
                self.get_logger().error('auto_stand_up: transfer_interfaces not found; arming is refused '
                                        '(source the workspace that has it, or set auto_stand_up false)')
            self.create_subscription(PointCloud2,self.value('stand_cloud_topic'),self.on_stand_cloud,
                                     qos_profile_sensor_data)
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
            if v('frontier_ranking') not in ('path_length','score'):
                raise ValueError('frontier_ranking must be path_length or score')
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
                                  ('frontier_planner_radius_m',0.,.5),('minimum_goal_motion_m',0.,.5),
                                  ('minimum_goal_hop_m',0.,1.),
                                  ('minimum_frontier_size_m',.04,2.),('blacklist_duration_sec',1.,600.),
                                  ('exit_open_radius',.5,3.),('exit_near_radius',.2,3.),
                                  ('exit_min_displacement',0.,20.),('exit_enclosed_radius',.2,2.),
                                  ('exit_dwell_sec',0.,30.),('final_cell_radius',.1,2.),
                                  ('start_creep_m',0.,.5),('start_scan_deg',0.,90.),
                                  ('start_creep_speed',.02,.1),
                                  ('stand_timeout_sec',5.,60.),('stand_settle_sec',5.,120.)]:
                if not low <= float(v(name)) <= high:
                    raise ValueError(name+' outside supported bounds')
            final=list(v('final_cell_xy') or [])  # [] in a params file loads as None
            if final and (len(final)!=2 or any(isinstance(x,(bool,str,bytes)) for x in final)
                          or not np.isfinite(np.asarray(final,float)).all()):
                raise ValueError('final_cell_xy must be empty or [x, y] in the map frame')
            if getattr(self,'armed',False) and any(p.name in ('mode','waypoints','waypoint_frame') for p in parameters):
                raise ValueError('Disarm before changing the mission or waypoint list')
        except (ValueError,TypeError) as exc:
            return SetParametersResult(successful=False,reason=str(exc))
        if parameters and hasattr(self,'waypoint_index') and any(p.name in ('mode','waypoints','waypoint_frame') for p in parameters):
            self.waypoint_index=0
            self.consecutive_failures=0
            self.anchor=None
            if hasattr(self,'exit'):  # a new mission: the exit may be found again
                self.exited=self.run_started=False
                self.exit.reset()
                self.start_steps=[]
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

    def arm(self,request,response,stood=False):
        if not request.data:
            self.disarm('DISARMED_BY_OPERATOR')
            response.success=True
            response.message='Disarmed; this mission goal is being cancelled'
            return response
        if self.execute and self.value('auto_stand_up') and not stood and not self.armed:
            return self.arm_after_standing(response)
        reason=self.health()
        if not self.execute:reason='Shadow mode cannot arm; relaunch with execution:=true'
        elif not reason and self.value('require_velocity_gate'):
            if time.monotonic()-self.gate_received > .75 or not self.gate.get('ready_to_arm',False):
                reason='VELOCITY_GATE_NOT_READY: '+str(self.gate.get('reason','no gate status'))
        if not reason and not self.client.server_is_ready():reason='NAV2_ACTION_SERVER_UNAVAILABLE'
        if not reason and self.value('require_nav2_active') and not self.nav2_active():
            reason='NAV2_BT_NAVIGATOR_NOT_ACTIVE'
        if self.pending or self.goal_handle:reason=reason or 'WAIT_FOR_PREVIOUS_GOAL_TO_FINISH_CANCELLING'
        if self.exited:reason='EXITED: run finished at the exit; relaunch or change mode to run again'
        response.success=reason is None
        response.message=reason or 'Armed; Nav2 goals may now execute'
        if response.success:
            self.armed=True
            self.epoch+=1
            self.anchor=self.pose
            self.waypoint_index=0
            self.consecutive_failures=0
            if not self.run_started:  # exit_min_displacement counts from where the run starts;
                self.exit.reset(self.pose)  # re-arming mid-run keeps it
                self.run_started=True
                self.unresolvable=[]
                if self.value('mode')=='explore' and self.value('start_routine'):
                    self.start_steps=self.start_plan()
            self.note='ARMED'
        return response

    def arm_after_standing(self,response):
        """Arming with auto_stand_up: posture check (and at most one stand toggle) first; tick
        then arms through arm(stood=True) once the normal checks pass."""
        reason=('EXITED: run finished at the exit; relaunch or change mode to run again' if self.exited
                else 'AUTO_STAND_UP_UNAVAILABLE: transfer_interfaces not installed' if self.simple_cmd_pub is None
                else 'STAND_UP_ALREADY_RUNNING' if self.stand is not None or self.settle_until is not None
                else None)
        response.success=reason is None
        response.message=reason or 'Checking posture: stands up if lying, then arms once standing and ready'
        if response.success:
            self.stand=StandUp(time.monotonic(),timeout=self.value('stand_timeout_sec'))
            self.stand_cloud=None
            self.note='STAND_UP_CHECK'
        return response

    def on_stand_cloud(self,msg):
        if self.stand is not None:self.stand_cloud=msg  # only the latest, only while needed

    def cloud_height(self,msg):
        try:
            tf=self.buffer.lookup_transform(self.value('base_frame'),msg.header.frame_id,Time())
            xyz=cloud_xyz(msg.data,msg.fields,msg.point_step,msg.row_step,msg.width,msg.height,msg.is_bigendian)
        except (TransformException,ValueError,KeyError):
            return None
        t,q=tf.transform.translation,tf.transform.rotation
        return body_height(xyz@quaternion_matrix(q.x,q.y,q.z,q.w).T+np.array([t.x,t.y,t.z]))

    def stand_tick(self,now):
        cloud,self.stand_cloud=self.stand_cloud,None
        if cloud is None:
            if self.stand.expired(now):
                self.stand_failed('STAND_STATE_UNKNOWN: no usable cloud on '+self.value('stand_cloud_topic'))
            return
        height=self.cloud_height(cloud)
        action=self.stand.feed(height,now)
        if action=='send':
            if self.simple_cmd_pub.get_subscription_count()==0:
                self.stand_failed('STAND_UP_NO_SUBSCRIBER: nobody listens on '+self.value('simple_cmd_topic'))
                return
            self.simple_cmd_pub.publish(self.simple_cmd_type(cmd_code=STAND_TOGGLE,size=0,type=0))
            self.get_logger().warn(f'robot lying (base_link {height:.2f} m above the floor): stand toggle sent')
            self.note='STANDING_UP'
        elif action=='done':
            self.get_logger().info(f'standing (base_link {height:.2f} m above the floor)')
            self.stand=None
            self.settle_until=now+self.value('stand_settle_sec')
            self.note='STANDING_WAITING_TO_ARM'
        elif action.startswith('fail:'):
            self.stand_failed(action[5:]+('' if height is None else f' (base_link {height:.2f} m above the floor)'))

    def stand_failed(self,reason):
        self.stand=None
        self.note=reason
        self.get_logger().error('auto stand-up: '+reason+'; not armed')

    def settle_tick(self,now):
        response=self.arm(SetBool.Request(data=True),SetBool.Response(),stood=True)
        if response.success:
            self.settle_until=None
            self.get_logger().info('armed after standing up')
        elif now>self.settle_until:
            self.settle_until=None
            self.note='NOT_ARMED_AFTER_STAND_UP: '+response.message
        else:
            self.note='STANDING_WAITING_TO_ARM: '+response.message

    def start_plan(self):
        """Creep straight ahead, then look left, right and back to the heading it had (maze_nav)."""
        steps=[('START_CREEP',self.value('start_creep_m'))] if self.value('start_creep_m')>0 else []
        turn=math.radians(self.value('start_scan_deg'))
        if turn>0:steps+=[('START_SCAN',turn),('START_SCAN',-2*turn),('START_SCAN',turn)]
        return steps

    def start_step(self):
        kind,value=self.start_steps.pop(0)
        self.note=kind
        if self.behaviors[kind].server_is_ready():
            self.dispatch_behavior(kind,value)
        else:
            self.get_logger().warn(f'Nav2 behavior for {kind} not available: skipped')
            self.start_steps=[step for step in self.start_steps if step[0]!=kind]

    def disarm(self,reason):
        self.stand=self.settle_until=None  # also ends a stand-up phase
        self.armed=False
        self.epoch+=1
        self.note=reason
        if self.lease:self.lease.publish(Bool(data=False))
        if self.goal_handle and not self.cancel_requested:
            self.cancel_requested=True
            self.goal_handle.cancel_goal_async()

    def choose_goal(self):
        if self.exited:return None,'EXITED'  # the run is over: no goal, no frontier past the exit
        if self.value('mode')=='waypoints':
            values=self.value('waypoints')
            index=3*self.waypoint_index
            if index>=len(values):return None,'MISSION_COMPLETE'
            if self.anchor is None:self.anchor=self.pose
            goal=tuple(values[index:index+3])
            return (relative_goal(self.anchor,goal) if self.value('waypoint_frame')=='initial_base' else goal),'WAYPOINT'
        now=time.monotonic()
        self.blacklist=[(p,until) for p,until in self.blacklist if until>now]
        # Frontiers on the open floor beyond the exit only once the arena has nothing else left.
        masked=(exit_masked_grid(self.grid,self.value('exit_open_radius'),self.exit.start or self.pose[:2],
                                 self.value('exit_min_displacement'),self.clearance())
                if self.value('exit_detection_enabled') and self.value('exit_frontiers_last') else None)
        goal,state=self.frontier_on(masked if masked is not None else self.grid)
        if goal is None and masked is not None:
            goal,state=self.frontier_on(self.grid)
            if goal:state='EXIT_FRONTIER'
        if goal and self.value('frontier_yaw_mode')=='keep_current':goal=(goal[0],goal[1],self.pose[2])
        return goal,state

    def frontier_on(self,grid):
        args=(self.value('frontier_clearance_m'),self.value('frontier_standoff_m'),
              self.value('minimum_frontier_size_m'),[p for p,until in self.blacklist])
        pick=select_frontier_by_path if self.value('frontier_ranking')=='path_length' else select_frontier
        extra={'planner_radius':self.value('frontier_planner_radius_m'),'ignored':list(self.unresolvable),
               'minimum_hop':self.value('minimum_goal_hop_m')} if pick is select_frontier_by_path else {}
        goal,state=pick(grid,self.pose[:2],*args,**extra)
        if state=='START_NOT_CLEAR':
            start=nearest_clear_start(grid,self.pose[:2],args[0])
            if start is not None:goal,state=pick(grid,start,*args,**extra)
        return goal,state

    def clearance(self):
        """seen_free_clearance of the current map, computed once per map message."""
        if self.clearance_cache[0] is not self.grid:
            self.clearance_cache=(self.grid,seen_free_clearance(self.grid))
        return self.clearance_cache[1]

    def check_exit(self):
        """Exit / final-cell test (at most 2 Hz). Returns True once the run has ended there."""
        if self.exited:return True
        if self.execute and not self.run_started:return False  # not armed yet: no start pose
        now=self.sensor_now()
        if now-self.last_exit_check<.5:return False
        self.last_exit_check=now
        explore=self.value('mode')=='explore' and self.value('exit_detection_enabled')
        final=list(self.value('final_cell_xy') or [])
        if not explore and not final:return False
        e=self.exit
        e.open_radius,e.near_radius=self.value('exit_open_radius'),self.value('exit_near_radius')
        e.min_displacement,e.enclosed_radius=self.value('exit_min_displacement'),self.value('exit_enclosed_radius')
        e.dwell,e.final_cell_radius=self.value('exit_dwell_sec'),self.value('final_cell_radius')
        e.final_cell_xy=tuple(float(x) for x in final) if final else None
        found=e.update(self.grid,self.pose[:2],now,self.clearance(),open_test=explore)
        # Exit diagnostics in the launch log (1 Hz) whenever open floor is getting close to the
        # exit test, so an overshoot can be read from the log without echoing /mission/status.
        if explore and not found and e.open_nearby>=.7*e.open_radius and now-self.last_exit_log>=1.:
            self.last_exit_log=now
            self.get_logger().info(f'exit_check: open_nearby={e.open_nearby:.2f}/{e.open_radius:.2f} '
                                   f'clearance={e.clearance:.2f} enclosed_seen={e.seen_enclosed} '
                                   f'held={0. if e.since is None else now-e.since:.1f}/{e.dwell:.1f}s '
                                   f'at ({self.pose[0]:.2f}, {self.pose[1]:.2f})')
        if found:
            self.exited=True
            self.get_logger().info(f'EXITED ({e.reason}) at ({self.pose[0]:.2f}, {self.pose[1]:.2f}): '
                                   'cancelling the goal, holding position, no more goals')
        return self.exited

    def stamped_goal(self,goal):
        msg=PoseStamped()
        msg.header.frame_id=self.value('map_frame')
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=map(float,goal[:2])
        msg.pose.orientation.z,msg.pose.orientation.w=math.sin(goal[2]/2),math.cos(goal[2]/2)
        return msg

    def dispatch(self,goal,kind='explore'):
        epoch=self.epoch
        self.goal_from=tuple(self.pose) if self.pose else None  # minimum_goal_motion_m
        self.goal=goal
        self.goal_kind=kind
        self.goal_started=self.sensor_now()
        message=NavigateToPose.Goal(pose=self.stamped_goal(goal))
        self.pending=self.client.send_goal_async(message,feedback_callback=self.on_feedback)
        self.pending.add_done_callback(lambda future:self.accepted(future,epoch))

    def dispatch_behavior(self,kind,value):
        epoch=self.epoch
        self.goal=None
        self.goal_kind='start'
        self.goal_started=self.sensor_now()
        if kind=='START_SCAN':
            message=Spin.Goal(target_yaw=float(value),time_allowance=DurationMsg(sec=20))
        else:
            message=DriveOnHeading.Goal(speed=float(self.value('start_creep_speed')),
                                        time_allowance=DurationMsg(sec=20))
            message.target.x=float(value)
        self.pending=self.behaviors[kind].send_goal_async(message,feedback_callback=self.on_feedback)
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
        if self.goal_kind=='start':  # start routine: a failed step is skipped, not counted
            if status!=GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f'{self.note} failed (status {status}); skipped')
            return
        if status==GoalStatus.STATUS_SUCCEEDED and self.value('mode')=='explore' and self.goal \
                and self.idle_success():
            # Nav2 "reached" a frontier goal without the robot moving or turning (sim 2026-11-15:
            # it alternated between two such frontiers forever). Being there did not clear the
            # frontier, so skip goals near it for the rest of the run and explore elsewhere;
            # not a failed goal (Nav2 did nothing wrong).
            self.unresolvable.append(self.goal[:2])
            self.note='FRONTIER_UNRESOLVABLE'
            self.get_logger().warn(f'frontier goal ({self.goal[0]:.2f}, {self.goal[1]:.2f}) reached '
                                   'without moving; skipping it for this run')
            return
        if status==GoalStatus.STATUS_SUCCEEDED:
            self.consecutive_failures=0
            self.note='GOAL_SUCCEEDED'
            if self.value('mode')=='waypoints':self.waypoint_index+=1
            elif self.goal:self.blacklist.append((self.goal[:2],time.monotonic()+self.value('blacklist_duration_sec')))
        else:self.failed_goal('NAV2_GOAL_FAILED_'+str(status))

    def idle_success(self):
        start=getattr(self,'goal_from',None)
        if start is None or not self.pose:return False
        moved=math.hypot(self.pose[0]-start[0],self.pose[1]-start[1])
        turned=abs(math.atan2(math.sin(self.pose[2]-start[2]),math.cos(self.pose[2]-start[2])))
        return moved<self.value('minimum_goal_motion_m') and turned<.5

    def abandon_goal(self,reason):
        """Cancel a goal Nav2 is stuck on and count it as a failed goal: in explore mode it is
        blacklisted and the next frontier tried; disarm after maximum_consecutive_failures."""
        self.epoch+=1  # the old goal's accepted/finished callbacks are then ignored
        if self.goal_handle and not self.cancel_requested:
            self.cancel_requested=True
            self.goal_handle.cancel_goal_async()
        self.goal_started=self.sensor_now()  # a still-pending send does not time out again at once
        self.failed_goal(reason)

    def failed_goal(self,reason):
        if self.goal_kind=='start':
            self.get_logger().warn(f'{self.note}: {reason}; skipped')
            return
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
        if self.stand is not None:self.stand_tick(now)
        elif self.settle_until is not None:self.settle_tick(now)
        standing_up=self.stand is not None or self.settle_until is not None
        reason=self.health()
        if self.armed and self.value('require_nav2_active') and not self.nav2_active():
            reason=reason or 'NAV2_BT_NAVIGATOR_NOT_ACTIVE'
        if self.armed and reason:self.disarm(reason)
        if not reason and self.check_exit():
            if self.armed:self.disarm('EXITED')  # cancels the goal; the lease drops below
            elif not self.execute:self.note='SHADOW_EXITED'
        if self.armed and (self.pending or self.goal_handle) and not self.cancel_requested \
                and sensor_now-self.goal_started>self.value('goal_timeout_sec'):
            self.abandon_goal('GOAL_TIMEOUT')
        # Only a live, accepted goal receives a drive lease. Loss of this process
        # expires at the independent gate; Nav2 alone cannot keep driving.
        if self.lease:
            self.lease.publish(Bool(data=bool(self.execute and self.armed and self.goal_handle and not self.cancel_requested and not reason)))
        if not reason and self.execute and self.armed and self.start_steps and not self.pending \
                and not self.goal_handle:
            self.start_step()
        elif not reason and not self.pending and not self.goal_handle and now-self.last_preview>=1.:
            self.last_preview=now
            goal,state=self.choose_goal()
            if goal:
                self.preview.publish(self.stamped_goal(goal))
                if self.execute and self.armed:
                    if self.client.server_is_ready():self.dispatch(goal)
                    else:self.disarm('NAV2_ACTION_SERVER_UNAVAILABLE')
            elif self.armed and state=='NO_REACHABLE_FRONTIER' and self.blacklist:
                # Only blacklisted frontiers left (e.g. a small one whose approach points all
                # lie near a failed goal): the blacklist only orders the work, so retry them now
                # (next preview, ~1 s) rather than end the run or idle until it expires; a goal
                # that keeps failing still disarms via maximum_consecutive_failures.
                self.blacklist.clear()
                self.note='RETRYING_BLACKLISTED_FRONTIERS'
            elif self.armed:self.disarm(state)
            if not self.execute:self.note='SHADOW_'+state
        if now-self.last_status>=.5:
            self.last_status=now
            message={'state':self.note if standing_up else reason or self.note,'armed':self.armed,'execution_enabled':self.execute,
                     'active_goal':self.goal if (self.pending or self.goal_handle) else None,
                     'pose':self.pose,'waypoint_index':self.waypoint_index,'mode':self.value('mode'),
                     'exited':self.exited,'exit_reason':self.exit.reason or None,
                     # Exit detector inputs (see exit_detect.ExitDetector): OPEN_FLOOR needs
                     # seen_enclosed, open_nearby >= exit_open_radius and that held for exit_dwell_sec.
                     'exit_check':{'clearance':round(self.exit.clearance,2),
                                   'open_nearby':round(self.exit.open_nearby,2),
                                   'seen_enclosed':self.exit.seen_enclosed,
                                   'held_sec':None if self.exit.since is None else round(sensor_now-self.exit.since,1)}}
            self.status.publish(String(data=json.dumps(message)))

    def lifecycle_result(self,future):
        self.lifecycle_future=None
        try:
            self.lifecycle_active=future.result().current_state.id==State.PRIMARY_STATE_ACTIVE
            self.lifecycle_received=self.sensor_now()
        except Exception:
            self.lifecycle_active=False


def main():
    from lite3_nav.runtime import initialize
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
