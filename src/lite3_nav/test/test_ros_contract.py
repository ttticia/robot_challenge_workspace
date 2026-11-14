"""ROS contract tests for the lite3_nav forks on isolated loopback domain 89; no robot access.

Run with test/run_tests.sh (sets LITE3_ROS_TESTS=1, Cyclone DDS loopback profile, domain 89).
The action server is a test double, not Nav2.
"""
import json
import os
import threading
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.getenv('LITE3_ROS_TESTS') != '1',
                                reason='Explicit isolated ROS test run required')


@pytest.fixture
def context():
    if os.getenv('ROS_DOMAIN_ID') != '89' or 'cyclonedds-local.xml' not in os.getenv('CYCLONEDDS_URI', ''):
        raise RuntimeError('ROS tests require isolated loopback domain 89; use test/run_tests.sh')
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
