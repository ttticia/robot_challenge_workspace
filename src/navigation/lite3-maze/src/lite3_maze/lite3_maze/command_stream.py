"""50 Hz command stream with a monotonic lease, independent of map planning."""
import threading
import time
from geometry_msgs.msg import Twist


class CommandStream:
    def __init__(self, publisher, lease=.2):
        self.publisher, self.lease = publisher, lease
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.enabled = False
        self.deadline = 0.
        self.command = Twist()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def set(self, command):
        with self.lock:
            self.command = command
            self.deadline = time.monotonic()+self.lease
            self.enabled = True

    def halt(self):
        with self.lock:
            self.enabled = False
            self.publisher.publish(Twist())

    def loop(self):
        while not self.stop.wait(.02):
            with self.lock:
                if self.enabled:
                    self.publisher.publish(self.command if time.monotonic()<self.deadline else Twist())

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.)
        self.halt()
