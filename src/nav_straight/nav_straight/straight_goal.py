"""Send Nav2 one straight path from where the robot stands, then hand back to manual.

    ros2 run nav_straight straight_goal        (normally started by nav_straight.launch.py)

Nav2 does the driving, steering and obstacle stop (config/nav2_params.yaml). This node:
  1. Waits for /odom and for the Nav2 lifecycle manager to report active.
  2. Refuses to start if another node already publishes /cmd_vel (one publisher rule).
  3. Counts down start_delay seconds.
  4. Sends controller_server's /follow_path a straight path in the odom frame:
     max_distance metres along the current heading.
  5. When the action ends (goal reached, or aborted because RPP saw a collision ahead,
     or the progress checker timed out), it sends manual mode so the handheld controller
     drives again. If /livox/points or /odom stops for sensor_timeout, or on Ctrl+C, it
     cancels the goal first (Nav2 Humble's controller would wait forever on stale TF;
     cmd_vel_bridge has already stopped the robot by then).

It never switches the robot to auto mode. The operator does that (lite3_auto).
"""
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

try:
    from transfer_interfaces.msg import MotionSimpleCMD
except ImportError:  # built off-robot without ~/lite3_ws
    MotionSimpleCMD = None

MANUAL_MODE = 0x21010C02
CMD_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     lifespan=Duration(seconds=0.3))
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
          GoalStatus.STATUS_CANCELED: 'canceled'}


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def straight_line(x0, y0, yaw, length, step=0.05):
    """Points (x, y) from (x0, y0) along yaw, every `step` metres, ending exactly at length."""
    n = max(1, int(math.ceil(length / step - 1e-9)))
    return [(x0 + min(i * step, length) * math.cos(yaw),
             y0 + min(i * step, length) * math.sin(yaw)) for i in range(n + 1)]


class StraightGoal(Node):
    def __init__(self):
        super().__init__('straight_goal')
        p = self.declare_parameter
        p('max_distance', 5.0)       # m, length of the straight path
        p('start_delay', 3.0)        # s countdown once Nav2 is up
        p('drive', True)             # false: dry run, no /cmd_vel check, no manual mode
        p('manual_on_finish', True)  # send manual mode after the run
        p('sensor_timeout', 0.5)     # s without cloud or odom while driving -> cancel
        p('odom_topic', '/odom')
        p('cloud_topic', '/livox/points')
        p('lifecycle_manager', 'lifecycle_manager_navigation')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in ('max_distance', 'start_delay', 'drive', 'sensor_timeout',
                                      'manual_on_finish', 'lifecycle_manager')}
        if self.cfg['max_distance'] <= 0:
            raise ValueError('max_distance must be > 0 (Nav2 needs a path end)')

        self.pose = None
        self.odom_frame = 'odom'
        self.odom_time = self.cloud_time = None
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        self.create_subscription(PointCloud2, g('cloud_topic'), self.on_cloud,
                                 qos_profile_sensor_data)
        self.client = ActionClient(self, FollowPath, 'follow_path')
        self.is_active = self.create_client(Trigger, f'/{self.cfg["lifecycle_manager"]}/is_active')
        self.path_pub = self.create_publisher(Path, '~/path', 1)
        self.mode_pub = None
        if self.cfg['drive'] and self.cfg['manual_on_finish']:
            if MotionSimpleCMD is None:
                self.get_logger().warn('transfer_interfaces not found: cannot send manual '
                                       'mode at the end. Run lite3_manual yourself.')
            else:
                self.mode_pub = self.create_publisher(MotionSimpleCMD, '/simple_cmd', CMD_QOS)

        self.start = None
        self.goal_handle = None
        self.done = False

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_from_quat(q.x, q.y, q.z, q.w))
        self.odom_frame = msg.header.frame_id or 'odom'
        self.odom_time = self.get_clock().now()

    def on_cloud(self, _msg):
        self.cloud_time = self.get_clock().now()

    def stale_sensors(self):
        now = self.get_clock().now()
        limit = Duration(seconds=float(self.cfg['sensor_timeout']))
        return [name for name, t in (('cloud', self.cloud_time), ('odom', self.odom_time))
                if t is None or now - t > limit]

    def travelled(self):
        if self.start is None or self.pose is None:
            return 0.0
        return math.hypot(self.pose[0] - self.start[0], self.pose[1] - self.start[1])

    # ------------------------------------------------------------ waiting
    def spin_for(self, seconds):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until_ready(self):
        log = self.get_logger()
        while rclpy.ok() and self.pose is None:
            log.info('waiting for /odom', throttle_duration_sec=3.0)
            rclpy.spin_once(self, timeout_sec=0.1)
        while rclpy.ok():
            if self.is_active.wait_for_service(timeout_sec=0.5):
                fut = self.is_active.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
                if fut.done() and fut.result() is not None and fut.result().success:
                    break
            log.info('waiting for Nav2 to become active', throttle_duration_sec=3.0)
            self.spin_for(0.5)
        while rclpy.ok() and not self.client.wait_for_server(timeout_sec=1.0):
            log.info('waiting for /follow_path', throttle_duration_sec=3.0)

    # --------------------------------------------------------------- run
    def run(self):
        log = self.get_logger()
        c = self.cfg
        self.wait_until_ready()
        if c['drive'] and self.count_publishers('/cmd_vel') > 1:
            log.error('another node publishes /cmd_vel (twist_mux, teleop, another Nav2?). '
                      'Not starting. Stop it and relaunch.')
            return
        log.info(f'Nav2 ready; starting in {c["start_delay"]:.0f} s '
                 '(robot must be standing and in auto mode)')
        self.spin_for(float(c['start_delay']))

        x0, y0, yaw0 = self.start = self.pose
        path = Path()
        path.header.frame_id = self.odom_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in straight_line(x0, y0, yaw0, c['max_distance']):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw0 / 2), math.cos(yaw0 / 2)
            path.poses.append(ps)
        self.path_pub.publish(path)

        goal = FollowPath.Goal(path=path, controller_id='FollowPath',
                               goal_checker_id='goal_checker')
        log.info(f'GO: {c["max_distance"]:.1f} m straight from x={x0:.2f} y={y0:.2f} '
                 f'yaw={math.degrees(yaw0):.1f} deg')
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, fut)
        self.goal_handle = fut.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.finish('Nav2 rejected the goal')
            return
        res = self.goal_handle.get_result_async()
        while rclpy.ok() and not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            stale = self.stale_sensors()
            if stale:
                self.cancel()
                self.finish(f'sensor timeout (no {"/".join(stale)} for '
                            f'> {self.cfg["sensor_timeout"]} s)')
                return
        status = res.result().status if res.result() else GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED:
            reason = f'reached max_distance {c["max_distance"]:.1f} m'
        elif status == GoalStatus.STATUS_ABORTED:
            reason = ('Nav2 aborted: obstacle ahead, robot held by collision_monitor, or '
                      'controller error (see controller_server log)')
        else:
            reason = f'goal {STATUS.get(status, status)}'
        self.finish(reason)

    def on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'driving: {self.travelled():.2f} m done, '
                               f'{fb.distance_to_goal:.2f} m to go, v {fb.speed:.2f} m/s',
                               throttle_duration_sec=1.0)

    def cancel(self):
        if self.goal_handle is not None and not self.done:
            fut = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    def finish(self, reason):
        if self.done:
            return
        self.done = True
        self.get_logger().info(f'STOPPED: {reason}. Travelled {self.travelled():.2f} m.')
        if self.mode_pub is not None:
            m = MotionSimpleCMD()
            m.cmd_code, m.size, m.type = MANUAL_MODE, 0, 0
            self.mode_pub.publish(m)
            self.get_logger().info('sent manual mode: handheld controller has control')
            self.spin_for(0.3)  # let the reliable publisher deliver


def main():
    # Handle Ctrl+C ourselves so the goal is cancelled and manual mode still goes out.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = StraightGoal()
    try:
        node.run()
        while rclpy.ok():          # idle so the launch keeps SLAM running
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        if node.start is not None:
            node.cancel()
            node.finish('shutdown (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
