# numpy's OpenBLAS keeps ~20 worker threads busy-waiting after every matrix multiply, even a
# tiny one (wall_scan's 3x3 cloud rotation): wall_scan used ~190% CPU instead of ~10%, enough
# to stall the simulator's lidar and trip cmd_vel_bridge's sensor watchdog. The variable is
# read once, when numpy is first imported; this package __init__ runs before any maze_nav
# module (entry points import maze_nav.<node>), so it is set in time for every node, test
# and `ros2 run`. setdefault: an explicit OPENBLAS_NUM_THREADS from the shell still wins.
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
