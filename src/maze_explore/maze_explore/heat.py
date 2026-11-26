"""This computer's heat and load: read it, check it before a run, log it during one.

Why (2026-11-26): under load the processor reaches its 100 C limit within seconds and is
throttled; Fast-LIO then falls behind, the sensor feed reads late and the odometry jumps.

    python3 -m maze_explore.heat                 one reading and a GO / NO-GO
    python3 -m maze_explore.heat --log FILE      one CSV line every 2 s until stopped

No ROS in here.
"""
import glob
import os
import subprocess
import sys
import time

HOT = 75        # C: above this before a run there is no headroom left
BUSY = ('rviz2', 'fake_maze_lite3', 'gzserver', 'gazebo', 'pytest')   # must not run beside a robot run


def temperature():
    """Hottest thermal zone, C (None when the kernel shows none)."""
    best = None
    for f in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
        try:
            v = int(open(f).read()) / 1000.0
        except (OSError, ValueError):
            continue
        best = v if best is None else max(best, v)
    return best


def throttled_ms():
    """Total time the processor package has been thermally throttled since boot, ms."""
    try:
        return int(open('/sys/devices/system/cpu/cpu0/thermal_throttle/'
                        'package_throttle_total_time_ms').read())
    except (OSError, ValueError):
        return None


def load():
    try:
        return os.getloadavg()[0]
    except OSError:
        return None


def turbo_on():
    try:
        return open('/sys/devices/system/cpu/intel_pstate/no_turbo').read().strip() == '0'
    except OSError:
        return None


def busy():
    """[(name, cpu %)] of heavy programs that should not run beside the robot stack."""
    try:
        out = subprocess.run(['ps', '-eo', 'pcpu,args'], capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    found = []
    for line in out.splitlines()[1:]:
        parts = line.split(None, 1)
        if len(parts) < 2:
            continue
        for name in BUSY:
            if name in parts[1] and 'maze_explore.heat' not in parts[1]:
                found.append((name, float(parts[0])))
                break
    return found


def check(wait=3.0):
    """(ok, lines): is this computer cool and idle enough to start a run?"""
    lines, ok = [], True
    t0, before = temperature(), throttled_ms()
    time.sleep(wait)
    t1, after = temperature(), throttled_ms()
    temp = max(v for v in (t0, t1) if v is not None) if (t0 is not None or t1 is not None) else None
    if temp is None:
        lines.append('  ?      no temperature sensor readable')
    elif temp >= HOT:
        ok = False
        lines.append(f'  NO-GO  processor {temp:.0f} C (needs under {HOT} C before a run): let it cool, '
                     'close what is running, check the fan')
    else:
        lines.append(f'  ok     processor {temp:.0f} C')
    if before is not None and after is not None and after > before:
        ok = False
        lines.append(f'  NO-GO  the processor is being throttled right now ({after - before} ms in the last '
                     f'{wait:.0f} s): Fast-LIO will fall behind')
    seen = {}
    for name, cpu in busy():
        seen[name] = seen.get(name, 0.0) + cpu
    for name, cpu in sorted(seen.items()):
        ok = False
        lines.append(f'  NO-GO  {name} is running ({cpu:.0f} % of a core): close it before the run')
    if not seen:
        lines.append('  ok     no RViz, simulator or test run on this computer')
    ld = load()
    if ld is not None:
        lines.append(f"  {'ok    ' if ld < 3.0 else 'NOTICE'} load {ld:.1f}" + ('' if ld < 3.0 else ' (busy before the stack even starts)'))
    if turbo_on():
        lines.append('  NOTICE turbo boost is on: more heat spikes (host setting, see AGENTS.md)')
    return ok, lines


def log(path, period=2.0):
    """Append 'time, temp C, throttled ms in the period, load, turbo' every `period` s."""
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    new = not os.path.exists(path)
    last = throttled_ms()
    with open(path, 'a', buffering=1) as f:
        if new:
            f.write('time,unix,temp_c,throttled_ms,load1,turbo\n')
        while True:
            time.sleep(period)
            now = throttled_ms()
            d = '' if now is None or last is None else now - last
            last = now
            t, ld = temperature(), load()
            f.write(f"{time.strftime('%H:%M:%S')},{time.time():.1f},{'' if t is None else round(t)},{d},"
                    f"{'' if ld is None else round(ld, 2)},{int(bool(turbo_on()))}\n")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ['--log'] and len(argv) > 1:
        try:
            log(argv[1])
        except KeyboardInterrupt:
            pass
        return 0
    ok, lines = check()
    print('this computer:')
    print('\n'.join(lines))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
