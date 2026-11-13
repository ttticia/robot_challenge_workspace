#!/usr/bin/env python3
"""Read-only run evaluation, plus operator Auto/Manual transitions for the test."""
import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Observer(Node):
    def __init__(self, pause_test=False):
        super().__init__('maze_sim_evaluation')
        self.status = None
        self.states, self.placeholders = Counter(), set()
        self.detectors_seen = set()
        self.yolo_status = None
        self.create_subscription(String, '/maze/yolo/status', self.yolo, 10)
        self.truth, self.command = None, None
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.create_subscription(String, '/maze/status', self.report, 10)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.pose, 10)
        self.create_subscription(Twist, '/cmd_vel', self.velocity, 10)
        for kind in ('qr','people','scene'):
            self.create_subscription(String, '/maze/'+kind+'/detections', lambda m,k=kind:self.detection(k,m), 10)
        self.pause_test = pause_test
        self.paused = self.resumed = self.manual_seen = False
        self.pause_time = None
        self.started = False
        self.last_mode = -math.inf
        self.exit_since = None
        self.exit_poses = []
        self.zero_after_exit = True

    def detection(self, kind, msg):
        source=json.loads(msg.data)['source']
        if source == 'placeholder':
            self.placeholders.add(kind)
        elif source == 'detector':
            self.detectors_seen.add(kind)

    def yolo(self, msg):
        self.yolo_status=json.loads(msg.data)

    def report(self, msg):
        self.status = json.loads(msg.data)
        self.states[self.status['state']] += 1

    def pose(self, msg):
        p=msg.pose.pose.position
        self.truth=[p.x,p.y,p.z]
        if self.exit_since is not None:
            self.exit_poses.append(self.truth[:2])

    def velocity(self, msg):
        self.command=[msg.linear.x,msg.linear.y,msg.angular.z]
        if self.exit_since is not None and any(abs(x)>1e-6 for x in self.command):
            self.zero_after_exit=False

    def mode(self, text):
        self.mode_pub.publish(String(data=text))
        self.last_mode=time.monotonic()

    def tick(self):
        if not self.status:return False
        now=time.monotonic()
        if not self.started:
            if not self.status['auto'] and now-self.last_mode>1:self.mode('lite3_auto')
            self.started = self.status['auto']
        if self.pause_test and self.started and not self.paused and len(self.status['cells']['visited'])>=3:
            self.mode('lite3_manual');self.paused=True;self.pause_time=now
        if self.paused and not self.resumed:
            if self.status['state']=='WAIT_FOR_LITE3_AUTO':self.manual_seen=True
            if now-self.pause_time>2. and self.manual_seen:
                self.mode('lite3_auto');self.resumed=True
        if self.status['state']=='MAZE_EXIT_REACHED':
            if self.exit_since is None:self.exit_since=now
            return now-self.exit_since>3.
        return False


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timeout',type=float,default=300)
    p.add_argument('--pause-test',action='store_true')
    p.add_argument('--require-yolo',action='store_true')
    p.add_argument('--output',type=Path,default=Path('logs/current/evaluation.json'))
    a=p.parse_args()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Observer(a.pause_test);start=time.monotonic();done=False
    try:
        while time.monotonic()-start<a.timeout and not done:
            rclpy.spin_once(n,timeout_sec=.01);done=n.tick()
        drift=math.inf
        if n.exit_poses:
            drift=max(math.dist(x,n.exit_poses[-1]) for x in n.exit_poses)
        result=dict(completed=done, wall_seconds=time.monotonic()-start, states=dict(n.states),
            placeholders_seen=sorted(n.placeholders),manual_pause_seen=n.manual_seen,
            detectors_seen=sorted(n.detectors_seen),yolo_status=n.yolo_status,
            resumed=n.resumed,final_status=n.status,final_truth_position=n.truth,
            command_zero_after_exit=n.zero_after_exit, exit_drift_m=drift if math.isfinite(drift) else None)
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
        success=done and n.zero_after_exit and drift<.08 and (not a.pause_test or n.manual_seen and n.resumed)
        if a.require_yolo:
            success &= 'people' in n.detectors_seen and n.yolo_status is not None and n.yolo_status['processed_frames']>10
        return 0 if success else 1
    except KeyboardInterrupt:
        return 130
    finally:
        n.mode('lite3_manual');n.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
