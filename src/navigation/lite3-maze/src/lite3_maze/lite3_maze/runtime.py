"""Monotonic pacing keeps watchdogs independent of wall-clock adjustments."""
import signal
import threading
import time
import rclpy
from rclpy.signals import SignalHandlerOptions


def run(cls):
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = cls()
    deadline = time.monotonic()
    try:
        while rclpy.ok() and not stop.is_set():
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0)
            if hasattr(node, 'update') and time.monotonic() >= deadline:
                node.update()
                deadline = time.monotonic()+.05
            time.sleep(.002)
    finally:
        if hasattr(node, 'halt'):
            node.halt()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
