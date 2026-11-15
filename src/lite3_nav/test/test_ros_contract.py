"""ROS contract tests for the lite3_nav forks on isolated loopback domain 88 (not the sim's 89); no robot access.

Run with test/run_tests.sh (sets LITE3_ROS_TESTS=1, Cyclone DDS loopback profile, domain 88).
The action server is a test double, not Nav2.
"""
import json
import math
import os
import threading
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.getenv('LITE3_ROS_TESTS') != '1',
                                reason='Explicit isolated ROS test run required')


@pytest.fixture
def context():
    if os.getenv('ROS_DOMAIN_ID') != '88' or 'cyclonedds-local.xml' not in os.getenv('CYCLONEDDS_URI', ''):
        raise RuntimeError('ROS tests require isolated loopback domain 88; use test/run_tests.sh')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=['--ros-args', '--disable-external-lib-logs', '--disable-rosout-logs'],
               signal_handler_options=SignalHandlerOptions.NO)
    nodes = []
    yield nodes
    for node in reversed(nodes):
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def spin(nodes, seconds, step=None):
    import rclpy
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if step:
            step()
        for node in nodes:
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0)
        time.sleep(.003)


def make_world(nodes):
    import rclpy
    from builtin_interfaces.msg import Time as MessageTime
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rosgraph_msgs.msg import Clock
    from tf2_ros import TransformBroadcaster
    node = rclpy.create_node('test_world')
    nodes.append(node)
    map_pub = node.create_publisher(OccupancyGrid, '/map',
                                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clock = node.create_publisher(Clock, '/clock', 10)
    broadcaster = TransformBroadcaster(node)
    started = time.monotonic()
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 100
    msg.info.resolution = .04
    msg.info.origin.position.x = msg.info.origin.position.y = -2.
    msg.info.origin.orientation.w = 1.
    msg.data = [0] * 10000
    last = [0.]

    def publish():
        if time.monotonic() - last[0] < .05:
            return
        last[0] = time.monotonic()
        ns = int((1000. + time.monotonic() - started) * 1e9)
        msg.header.stamp = MessageTime(sec=ns // 10**9, nanosec=ns % 10**9)
        clock.publish(Clock(clock=msg.header.stamp))
        map_pub.publish(msg)
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.rotation.w = 1.
        broadcaster.sendTransform(tf)
    return node, publish


def test_invalid_health_check_is_rejected(context):
    from rclpy.parameter import Parameter
    from lite3_nav.mission import Mission
    with pytest.raises(Exception):
        Mission(parameter_overrides=[Parameter('nav2_health_check', value='bogus')])


def test_shadow_mission_has_no_action_client(context):
    from rclpy.parameter import Parameter
    from lite3_nav.mission import Mission
    world, publish = make_world(context)
    mission = Mission(parameter_overrides=[Parameter('require_imu', value=False),
                                           Parameter('use_sim_time', value=True)])
    context.append(mission)
    spin(context, 1.5, publish)
    assert mission.client is None and mission.lifecycle is None and not mission.armed
    assert world.count_publishers('/mission/drive_lease') == 0


def test_arms_with_action_server_health_check(context):
    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from std_srvs.srv import SetBool
    from lite3_nav.mission import Mission
    world, publish = make_world(context)
    server_node = rclpy.create_node('test_nav2_action_server')
    context.append(server_node)

    def execute(handle):
        until = time.monotonic() + 2.
        while time.monotonic() < until and not handle.is_cancel_requested:
            time.sleep(.01)
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()
    server = ActionServer(server_node, NavigateToPose, '/navigate_to_pose', execute_callback=execute,
                          goal_callback=lambda goal: GoalResponse.ACCEPT,
                          cancel_callback=lambda goal: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    mission = Mission(parameter_overrides=[
        Parameter('execution_enabled', value=True), Parameter('require_velocity_gate', value=False),
        Parameter('require_imu', value=False), Parameter('require_nav2_active', value=True),
        Parameter('nav2_health_check', value='action_server'), Parameter('use_sim_time', value=True)])
    context.append(mission)
    # action_server mode needs no lifecycle client.
    assert mission.lifecycle is None
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(server_node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        spin([world, mission], 1.5, publish)
        response = mission.arm(SetBool.Request(data=True), SetBool.Response())
        assert response.success, response.message
        spin([world, mission], 1., publish)
        assert mission.armed and (mission.goal_handle or mission.pending)
        mission.disarm('TEST_DONE')
        spin([world, mission], 1., publish)
    finally:
        executor.shutdown(timeout_sec=4.)
        worker.join(timeout=4.)
        server.destroy()


def test_final_cell_ends_the_run_cancels_and_refuses_rearming(context):
    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from std_srvs.srv import SetBool
    from lite3_nav.mission import Mission
    world, publish = make_world(context)
    server_node = rclpy.create_node('test_nav2_exit_server')
    context.append(server_node)
    cancelled = []

    def execute(handle):
        until = time.monotonic() + 5.
        while time.monotonic() < until and not handle.is_cancel_requested:
            time.sleep(.01)
        if handle.is_cancel_requested:
            cancelled.append(True)
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()
    server = ActionServer(server_node, NavigateToPose, '/navigate_to_pose', execute_callback=execute,
                          goal_callback=lambda goal: GoalResponse.ACCEPT,
                          cancel_callback=lambda goal: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    # Waypoint 0.5 m ahead; the final cell is set to "here" only once the goal runs.
    mission = Mission(parameter_overrides=[
        Parameter('execution_enabled', value=True), Parameter('require_velocity_gate', value=False),
        Parameter('require_imu', value=False), Parameter('nav2_health_check', value='action_server'),
        Parameter('use_sim_time', value=True), Parameter('final_cell_radius', value=.2)])
    context.append(mission)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(server_node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        spin([world, mission], 1.5, publish)
        assert mission.arm(SetBool.Request(data=True), SetBool.Response()).success
        spin([world, mission], 1., publish)
        assert mission.armed and mission.goal_handle and not mission.exited
        bad = mission.set_parameters([Parameter('final_cell_xy', value=[1., float('nan')])])[0]
        assert not bad.successful
        assert mission.set_parameters([Parameter('final_cell_xy', value=[0.05, 0.])])[0].successful
        spin([world, mission], 1.5, publish)
        assert mission.exited and not mission.armed and mission.note == 'EXITED'
        assert mission.exit.reason == 'FINAL_CELL' and cancelled and mission.goal_handle is None
        response = mission.arm(SetBool.Request(data=True), SetBool.Response())
        assert not response.success and response.message.startswith('EXITED')
        assert mission.choose_goal() == (None, 'EXITED')
    finally:
        executor.shutdown(timeout_sec=4.)
        worker.join(timeout=4.)
        server.destroy()


def test_mission_loads_the_real_params_files(context):
    # Same files, same order as the launch files: robot.yaml, then sim.yaml on top.
    from pathlib import Path
    from lite3_nav.mission import Mission
    config = Path(__file__).resolve().parents[1] / 'config'
    mission = Mission(cli_args=['--ros-args', '--params-file', str(config / 'robot.yaml'),
                                '--params-file', str(config / 'sim.yaml')])
    context.append(mission)
    assert mission.value('goal_timeout_sec') == 45. and mission.value('nav2_health_check') == 'either'
    assert list(mission.value('final_cell_xy') or []) == [] and mission.check_exit() is False
    assert mission.value('frontier_ranking') == 'path_length'
    assert mission.value('auto_stand_up') is False  # robot.yaml true, sim.yaml false


def test_auto_stand_up_refuses_without_transfer_interfaces(context):
    import importlib.util
    from rclpy.parameter import Parameter
    from std_srvs.srv import SetBool
    from lite3_nav.mission import Mission
    if importlib.util.find_spec('transfer_interfaces'):
        pytest.skip('transfer_interfaces installed here')
    mission = Mission(parameter_overrides=[Parameter('execution_enabled', value=True),
                                           Parameter('auto_stand_up', value=True)])
    context.append(mission)
    response = mission.arm(SetBool.Request(data=True), SetBool.Response())
    assert not response.success and response.message.startswith('AUTO_STAND_UP_UNAVAILABLE')
    assert mission.stand is None and not mission.armed


def test_gate_allows_only_the_named_simple_cmd_publisher(context):
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool
    from lite3_nav.velocity_gate import VelocityGate
    allowed = rclpy.create_node('mission_stand_test')
    other = rclpy.create_node('someone_else')
    context.extend([allowed, other])
    gate = VelocityGate(parameter_overrides=[
        Parameter('drive_output_enabled', value=True), Parameter('simple_cmd_allowed_node', value='mission_stand_test'),
        Parameter('input_topic', value='/test/nav_cmd'), Parameter('output_topic', value='/test/final_cmd')])
    context.append(gate)
    allowed.create_publisher(Bool, '/simple_cmd', 1)  # the gate checks publishers, not types
    spin(context, 1., gate.tick)
    assert not any(c.startswith('/simple_cmd') for c in gate.conflicts), gate.conflicts
    other.create_publisher(Bool, '/simple_cmd', 1)
    spin(context, 1., gate.tick)
    assert gate.conflicts == ['/simple_cmd:someone_else']


def test_frontier_ranking_is_validated(context):
    from rclpy.parameter import Parameter
    from lite3_nav.mission import Mission
    mission = Mission()
    context.append(mission)
    assert not mission.set_parameters([Parameter('frontier_ranking', value='bogus')])[0].successful
    assert mission.set_parameters([Parameter('frontier_ranking', value='score')])[0].successful


def test_goal_timeout_blacklists_and_keeps_exploring(context):
    from rclpy.parameter import Parameter
    from lite3_nav.mission import Mission
    mission = Mission(parameter_overrides=[Parameter('mode', value='explore'),
                                           Parameter('require_imu', value=False)])
    context.append(mission)
    cancels = []

    class Handle:
        def cancel_goal_async(self):
            cancels.append(True)
    mission.armed, mission.goal, mission.goal_handle = True, (1., 2., 0.), Handle()
    epoch = mission.epoch
    mission.abandon_goal('GOAL_TIMEOUT')
    assert cancels == [True] and mission.cancel_requested and mission.epoch == epoch + 1
    assert mission.armed and mission.note == 'GOAL_TIMEOUT' and mission.consecutive_failures == 1
    assert [p for p, _ in mission.blacklist] == [(1., 2.)]
    mission.abandon_goal('GOAL_TIMEOUT')  # cancel already requested: not sent twice
    assert cancels == [True] and mission.armed
    mission.abandon_goal('GOAL_TIMEOUT')  # third in a row (maximum_consecutive_failures 3)
    assert not mission.armed and mission.note.startswith('CONSECUTIVE_GOAL_FAILURES')


def test_start_routine_creeps_then_scans_then_explores(context):
    import rclpy
    from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from std_srvs.srv import SetBool
    from lite3_nav.mission import Mission
    world, publish = make_world(context)
    server_node = rclpy.create_node('test_nav2_start_servers')
    context.append(server_node)
    calls = []

    def server(kind, action, name):
        def execute(handle):
            goal = handle.request
            calls.append((kind, round(goal.target.x, 3) if kind == 'creep' else
                          round(goal.target_yaw, 3) if kind == 'scan' else None))
            handle.succeed()
            return action.Result()
        return ActionServer(server_node, action, name, execute_callback=execute,
                            goal_callback=lambda goal: GoalResponse.ACCEPT,
                            cancel_callback=lambda goal: CancelResponse.ACCEPT,
                            callback_group=ReentrantCallbackGroup())
    servers = [server('nav', NavigateToPose, '/navigate_to_pose'),
               server('creep', DriveOnHeading, '/drive_on_heading'), server('scan', Spin, '/spin')]
    # The test map is all seen free floor: no frontier, so exploring ends at once afterwards.
    mission = Mission(parameter_overrides=[
        Parameter('execution_enabled', value=True), Parameter('require_velocity_gate', value=False),
        Parameter('require_imu', value=False), Parameter('nav2_health_check', value='action_server'),
        Parameter('use_sim_time', value=True), Parameter('mode', value='explore'),
        Parameter('exit_detection_enabled', value=False)])
    context.append(mission)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(server_node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        spin([world, mission], 1.5, publish)
        assert mission.arm(SetBool.Request(data=True), SetBool.Response()).success
        spin([world, mission], 3., publish)
        turn = round(math.pi / 4, 3)
        assert calls == [('creep', .1), ('scan', turn), ('scan', round(-math.pi / 2, 3)), ('scan', turn)]
        assert not mission.armed and mission.note == 'NO_REACHABLE_FRONTIER' and not mission.start_steps
        # Re-arming in the same run does not repeat the start routine.
        mission.arm(SetBool.Request(data=True), SetBool.Response())
        spin([world, mission], 1., publish)
        assert len(calls) == 4
    finally:
        executor.shutdown(timeout_sec=4.)
        worker.join(timeout=4.)
        for item in servers:
            item.destroy()


def test_gate_status_names_expected_publisher_count(context):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from lite3_nav.velocity_gate import VelocityGate
    node = rclpy.create_node('gate_test_count')
    context.append(node)
    gate = VelocityGate(parameter_overrides=[
        Parameter('drive_output_enabled', value=True), Parameter('expected_input_publishers', value=2),
        Parameter('input_topic', value='/test/nav_cmd'), Parameter('output_topic', value='/test/final_cmd')])
    context.append(gate)
    node.create_publisher(Twist, '/test/nav_cmd', 1)
    statuses = []
    node.create_subscription(String, '/mission/gate_status', lambda m: statuses.append(json.loads(m.data)), 10)
    spin(context, 1.2, gate.tick)
    assert statuses and statuses[-1]['reason'] == 'EXPECTED_2_NAV2_INPUT_PUBLISHERS_GOT_1'
    assert not statuses[-1]['ready_to_arm']


def test_gate_transient_local_output_limits_and_times_out(context):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool
    from lite3_nav.velocity_gate import VelocityGate
    node = rclpy.create_node('gate_test_io')
    context.append(node)
    gate = VelocityGate(parameter_overrides=[
        Parameter('drive_output_enabled', value=True),
        Parameter('input_topic', value='/test/nav_cmd'), Parameter('output_topic', value='/test/final_cmd')])
    context.append(gate)
    cmd = node.create_publisher(Twist, '/test/nav_cmd', 1)
    lease = node.create_publisher(Bool, '/mission/drive_lease', 1)
    outputs = []
    # Same profile as the Lite3 domain bridge subscription: only a TRANSIENT_LOCAL publisher matches.
    node.create_subscription(
        Twist, '/test/final_cmd', lambda m: outputs.append([m.linear.x, m.linear.y, m.angular.z]),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    last = [0.]

    def publish(command=True):
        if time.monotonic() - last[0] > .04:
            last[0] = time.monotonic()
            if command:
                msg = Twist()
                msg.linear.x = msg.linear.y = 1.
                msg.angular.z = 2.
                cmd.publish(msg)
            lease.publish(Bool(data=True))
        gate.tick()
    spin(context, 2., publish)
    assert outputs and np.linalg.norm(outputs[-1][:2]) == pytest.approx(.1, abs=.002)
    assert outputs[-1][2] == pytest.approx(.25, abs=.002)
    spin(context, .65, lambda: publish(command=False))
    assert np.linalg.norm(outputs[-1]) == 0.


class PushRig:
    """Gate with push input enabled plus the publishers the mission would own."""

    def __init__(self, context, **overrides):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, String
        from lite3_nav.velocity_gate import VelocityGate
        self.node = rclpy.create_node('gate_test_push')
        context.append(self.node)
        params = dict(drive_output_enabled=True, push_input_enabled=True, input_topic='/test/nav_cmd',
                      output_topic='/test/final_cmd', push_input_topic='/test/push_cmd',
                      push_lease_topic='/test/push_lease')
        params.update(overrides)
        self.gate = VelocityGate(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        context.append(self.gate)
        self.context = context
        self.nav = self.node.create_publisher(Twist, '/test/nav_cmd', 1)
        self.push = self.node.create_publisher(Twist, '/test/push_cmd', 1)
        self.lease = self.node.create_publisher(Bool, '/mission/drive_lease', 1)
        self.push_lease = self.node.create_publisher(Bool, '/test/push_lease', 1)
        self.outputs, self.statuses = [], []
        self.node.create_subscription(
            Twist, '/test/final_cmd', lambda m: self.outputs.append([m.linear.x, m.linear.y, m.angular.z]),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String, '/mission/gate_status',
                                      lambda m: self.statuses.append(json.loads(m.data)), 10)
        self.last = 0.

    def run(self, seconds, push=None, nav=None, push_lease=False, lease=False):
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool

        def step():
            if time.monotonic() - self.last > .04:
                self.last = time.monotonic()
                if push is not None:
                    msg = Twist()
                    msg.linear.x, msg.linear.y, msg.angular.z = push
                    self.push.publish(msg)
                if nav is not None:
                    msg = Twist()
                    msg.linear.x, msg.linear.y, msg.angular.z = nav
                    self.nav.publish(msg)
                self.lease.publish(Bool(data=lease))
                self.push_lease.publish(Bool(data=push_lease))
            self.gate.tick()
        spin(self.context, seconds, step)
        return np.array(self.outputs[-1]) if self.outputs else None


def test_push_clamped_straight_and_stops_when_stale(context):
    rig = PushRig(context)
    out = rig.run(2., push=(1., 1., 2.), push_lease=True)
    assert out is not None and out[0] == pytest.approx(.1, abs=.002) and out[1] == 0. and out[2] == 0., rig.statuses[-1]
    assert rig.statuses[-1]['input'] == 'push' and rig.statuses[-1]['push'] == 'PUSH'
    out = rig.run(.65, push_lease=True)  # push commands stop arriving
    assert not out.any() and rig.statuses[-1]['push'] == 'PUSH_COMMAND_STALE'


def test_push_respects_lower_push_limit_and_reverse(context):
    rig = PushRig(context, max_push_linear_m_s=.05)
    out = rig.run(1.5, push=(-1., 0., 0.), push_lease=True)
    assert out[0] == pytest.approx(-.05, abs=.002) and not out[1:].any()


def test_push_rejected_without_push_lease(context):
    rig = PushRig(context)
    out = rig.run(1.5, push=(.1, 0., 0.))
    assert out is None or not out.any()
    assert rig.statuses[-1]['push'] == 'NO_PUSH_LEASE' and rig.statuses[-1]['input'] == 'none'
    # The ordinary drive lease never authorises the push input.
    out = rig.run(1., push=(.1, 0., 0.), lease=True)
    assert out is None or not out.any()


def test_push_rejected_while_nav2_commanding(context):
    rig = PushRig(context)
    out = rig.run(1.5, push=(.1, 0., 0.), nav=(.1, 0., 0.), push_lease=True)
    assert out is None or not out.any()
    assert rig.statuses[-1]['push'] == 'NAV2_COMMANDING'


def test_push_lease_blocks_nav2_input_and_both_leases_stop(context):
    rig = PushRig(context)
    out = rig.run(1.5, nav=(.1, 0., 0.), lease=True)
    assert out[0] == pytest.approx(.1, abs=.002) and rig.statuses[-1]['input'] == 'nav2'
    out = rig.run(.5, nav=(.1, 0., 0.), lease=True, push_lease=True)
    assert not out.any() and rig.statuses[-1]['input'] == 'none'
    assert rig.statuses[-1]['push'] == 'BOTH_LEASES_HELD'


def test_push_rejected_with_second_push_publisher(context):
    import rclpy
    from geometry_msgs.msg import Twist
    rig = PushRig(context)
    rogue = rclpy.create_node('rogue_pusher')
    context.append(rogue)
    rogue.create_publisher(Twist, '/test/push_cmd', 1)
    out = rig.run(1.5, push=(.1, 0., 0.), push_lease=True)
    assert out is None or not out.any()
    assert rig.statuses[-1]['push'] == 'MULTIPLE_PUSH_PUBLISHERS'


def test_push_disabled_by_default(context):
    from rclpy.parameter import Parameter
    from lite3_nav.velocity_gate import VelocityGate
    gate = VelocityGate(parameter_overrides=[Parameter('drive_output_enabled', value=True)])
    context.append(gate)
    assert not gate.push_enabled
    assert gate.push_permitted(time.monotonic(), True, False, False) == (False, '')


@pytest.mark.parametrize('floor_z', [0.0, float('nan')])
def test_wall_scan_keeps_only_the_wall_band(context, floor_z):
    """wall_scan (borrowed from maze_nav): wall 1 m ahead in the 0.30-0.43 m band is seen; a
    0.15 m ramp 1.5 m to the left and the floor are not. Floor given, or measured at startup."""
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import LaserScan, PointCloud2, PointField
    from tf2_ros import StaticTransformBroadcaster
    from lite3_nav.wall_scan import WallScan
    node = rclpy.create_node('wall_scan_test_world')
    context.append(node)
    static = StaticTransformBroadcaster(node)
    transforms = []
    for child, z in (('base_link', .35), ('livox_frame', .45)):   # standing robot, lidar on top
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'odom', child
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.
        transforms.append(tf)
    static.sendTransform(transforms)
    rng = np.random.default_rng(1)
    ring = rng.uniform(-2.4, 2.4, (6000, 2))
    ring = ring[np.hypot(ring[:, 0], ring[:, 1]) > .5]
    floor = np.c_[ring, rng.normal(0., .003, len(ring))]
    wall = np.c_[np.full(400, 1.), rng.uniform(-.3, .3, 400), rng.uniform(0., .45, 400)]
    ramp = np.c_[rng.uniform(-.2, .2, 200), np.full(200, 1.5), np.full(200, .15)]
    points = (np.r_[floor, wall, ramp] - [0., 0., .45]).astype(np.float32)   # in livox_frame
    cloud = PointCloud2()
    cloud.header.frame_id = 'livox_frame'
    cloud.height, cloud.width = 1, len(points)
    cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate('xyz')]
    cloud.point_step, cloud.row_step, cloud.is_dense = 12, 12 * len(points), True
    cloud.data = points.tobytes()
    cloud_pub = node.create_publisher(PointCloud2, '/test/points', 10)
    scans = []
    node.create_subscription(LaserScan, '/test/wall_scan', scans.append, 10)
    scan = WallScan(parameter_overrides=[Parameter('cloud_topic', value='/test/points'),
                                         Parameter('scan_topic', value='/test/wall_scan'),
                                         Parameter('floor_z', value=floor_z)])
    context.append(scan)
    last = [0.]

    def publish():
        if time.monotonic() - last[0] > .1:
            last[0] = time.monotonic()
            cloud.header.stamp = node.get_clock().now().to_msg()
            cloud_pub.publish(cloud)
    spin(context, 2.5, publish)
    assert scans, 'no /wall_scan'
    assert scan.floor == pytest.approx(0., abs=.02)
    msg = scans[-1]
    assert msg.header.frame_id == 'base_link'
    ranges = np.asarray(msg.ranges)
    beam = lambda a: int(round((a - msg.angle_min) / msg.angle_increment)) % len(ranges)  # noqa: E731
    assert ranges[beam(0.)] == pytest.approx(1., abs=.02)
    assert np.isinf(ranges[beam(math.pi / 2)])               # ramp below the band
    hit = np.flatnonzero(np.isfinite(ranges))
    angles = msg.angle_min + hit * msg.angle_increment
    assert len(hit) and (np.abs(angles) < math.atan(.3 / 1.) + .02).all()   # only the wall, no floor
    assert (ranges[hit] < 1.05).all()
