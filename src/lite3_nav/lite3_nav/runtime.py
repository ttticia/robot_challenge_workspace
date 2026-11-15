# Unchanged copy of the team package file lite3_mission/lite3_mission/runtime.py (main e725625) so lite3_nav is
# self-contained (origin/navigation removes lite3_mission). Relative imports are lite3_nav's copies.
"""Defer process signals so repeated launch Ctrl+C cannot interrupt cleanup."""
import signal
import threading
import rclpy
from rclpy.signals import SignalHandlerOptions


def initialize():
    stop=threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def request_stop(*unused):stop.set()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,request_stop)
    return stop
