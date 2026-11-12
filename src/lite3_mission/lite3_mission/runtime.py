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
