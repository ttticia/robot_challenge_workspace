"""cpu_monitor: log CPU use per process during a run, and summarise it when the run ends.

Started by maze_nav.launch.py (cpu_log:=true). Not a ROS node: it reads /proc through psutil,
so it costs almost nothing and keeps working if the ROS graph is in trouble.

Every `period` seconds it samples every process on the computer and writes the busy ones to
    ~/challenge_ws/logs/cpu/run_<time>.csv
        wall_time, name, pid, cpu_pct (100 = one full core), rss_mb
plus a `_total_` row per sample (whole computer, 100 = one core, so 1000 = all 10 threads
busy). ROS processes are named after their node (`__node:=` in the command line), the rest
after their executable. On exit (Ctrl+C) it prints, and saves next to the CSV as
run_<time>_summary.txt, the mean / p95 / max per name, highest mean first.

While running it warns when the whole computer stays above `warn_pct` of all cores for
`warn_samples` samples in a row: navigation (controller, SLAM) then starts missing deadlines.

    ros2 run maze_nav cpu_monitor                      # by hand, Ctrl+C for the summary
    ros2 run maze_nav cpu_monitor --period 1 --out /tmp/cpu
"""
import argparse
import csv
import math
import os
import time

KEEP_PCT = 1.0          # per-sample rows are written only for processes above this
INTERPRETERS = ('python', 'python3', 'bash', 'sh', 'ros2')


def proc_name(cmdline, fallback):
    """Node name from `__node:=<name>`, else the executable's basename (skipping python3)."""
    for arg in cmdline:
        if arg.startswith('__node:='):
            return arg.split(':=', 1)[1]
    for arg in cmdline:
        base = os.path.basename(arg)
        if not base or arg.startswith('-'):
            continue
        if base.split('.')[0].rstrip('0123456789.') in INTERPRETERS:
            continue
        return base
    return fallback


def percentile(values, q):
    """q-th percentile (0-100) by nearest rank; 0.0 for no values."""
    if not values:
        return 0.0
    s = sorted(values)
    k = max(0, min(len(s) - 1, math.ceil(q / 100.0 * len(s)) - 1))
    return s[k]


def summarise(samples, n_samples, ncpu):
    """Text table from {name: [cpu_pct per sample]}; absent samples count as 0."""
    rows = []
    for name, vals in samples.items():
        full = vals + [0.0] * (n_samples - len(vals))
        rows.append((sum(full) / max(1, n_samples), percentile(full, 95), max(full), name))
    rows.sort(reverse=True)
    lines = [f'{n_samples} samples, {ncpu} logical CPUs (100 % = one core, '
             f'{100 * ncpu} % = all)',
             f'{"process":32s} {"mean%":>7s} {"p95%":>7s} {"max%":>7s}']
    for mean, p95, mx, name in rows:
        if name != '_total_' and mean < 0.5 and mx < 5:
            continue
        lines.append(f'{name:32s} {mean:7.1f} {p95:7.1f} {mx:7.1f}')
    return '\n'.join(lines)


def main(argv=None):
    import psutil

    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--period', type=float, default=2.0)
    ap.add_argument('--out', default=os.path.expanduser('~/challenge_ws/logs/cpu'))
    ap.add_argument('--warn-pct', type=float, default=85.0)
    ap.add_argument('--warn-samples', type=int, default=3)
    args, _ = ap.parse_known_args(argv)      # ignore --ros-args ... from launch

    ncpu = psutil.cpu_count() or 1
    os.makedirs(args.out, exist_ok=True)
    stem = os.path.join(args.out, time.strftime('run_%Y%m%d_%H%M%S'))
    me = os.getpid()
    procs = {}                                # pid -> (Process, name)
    samples = {}                              # name -> [cpu_pct per sample]
    n = 0
    hot = 0
    psutil.cpu_percent(None)
    print(f'[cpu_monitor] logging every {args.period:g} s to {stem}.csv', flush=True)
    with open(stem + '.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['wall_time', 'name', 'pid', 'cpu_pct', 'rss_mb'])
        try:
            while True:
                time.sleep(args.period)
                now = time.time()
                live = set()
                per_name = {}
                for p in psutil.process_iter(['pid']):
                    pid = p.info['pid']
                    if pid == me:
                        continue
                    live.add(pid)
                    try:
                        if pid not in procs:
                            name = proc_name(p.cmdline(), p.name())
                            p.cpu_percent(None)       # first call only primes the counter
                            procs[pid] = (p, name)
                            continue
                        proc, name = procs[pid]
                        pct = proc.cpu_percent(None)
                        rss = proc.memory_info().rss / 1e6
                    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                        continue
                    per_name[name] = per_name.get(name, 0.0) + pct
                    if pct >= KEEP_PCT:
                        w.writerow([f'{now:.1f}', name, pid, f'{pct:.1f}', f'{rss:.0f}'])
                for pid in set(procs) - live:
                    del procs[pid]
                total = psutil.cpu_percent(None) * ncpu    # 100 = one core
                w.writerow([f'{now:.1f}', '_total_', '', f'{total:.1f}', ''])
                f.flush()
                n += 1
                for name, pct in per_name.items():
                    vals = samples.setdefault(name, [])
                    vals.extend([0.0] * (n - 1 - len(vals)))     # samples it was absent
                    vals.append(pct)
                samples.setdefault('_total_', []).append(total)
                hot = hot + 1 if total >= args.warn_pct * ncpu else 0   # both in core-%
                if hot == args.warn_samples:
                    top = sorted(per_name.items(), key=lambda kv: -kv[1])[:4]
                    print(f'[cpu_monitor] WARNING computer above {args.warn_pct:g} % of all '
                          f'cores for {hot * args.period:g} s; top: '
                          + ', '.join(f'{k} {v:.0f}%' for k, v in top), flush=True)
        except KeyboardInterrupt:
            pass
    text = summarise(samples, n, ncpu)
    with open(stem + '_summary.txt', 'w') as f:
        f.write(text + '\n')
    print(f'[cpu_monitor] summary ({stem}_summary.txt):\n{text}', flush=True)


if __name__ == '__main__':
    main()
