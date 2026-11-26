"""report_watch: every report task_reporter POSTs to the scoring server and the reply, in a
terminal of its own (user, 2026-11-26).

    ros2 run task_reporter report_watch            # in a second terminal, any time
    ros2 run task_reporter report_watch --no-retry # one line per waiting resend instead of all
    ros2 run task_reporter report_watch --history  # the reports of earlier runs, from the logs
    ros2 run task_reporter report_watch --history 3          # ... of the last 3 runs only
    ros2 run task_reporter report_watch --history 11-26-10-39  # ... of the runs whose log
                                                             # folder name contains that

--history reads the task_reporter logs in ~/challenge_ws/logs/ros (one folder per
explore.launch.py run, plus task_reporter_*.log of `ros2 run`; kept 7 days) and prints each
run's reports with the reply each one got, then the latest score per mission. A report the
server answered with 409 before 2026-11-26 night was not sent again.

Listens to /mission/server_log (latched, so the reports sent before it started show too).
Green = accepted with every evaluated field right, yellow = accepted with a field wrong or
not taken yet (HTTP 409 / server error, task_reporter sends it again), red = rejected or no
answer. Light: a few small strings per report, no camera, no lidar.
"""
import ast
import glob
import json
import os
import re
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

COLOUR = sys.stdout.isatty()
GREEN, YELLOW, RED, BOLD, DIM, END = (
    ('\033[32m', '\033[33m', '\033[31m', '\033[1m', '\033[2m', '\033[0m') if COLOUR
    else ('',) * 6)


def verdict(code, reply, resend=True):
    """(colour, word) for one reply."""
    if code is None:
        return RED, 'NO ANSWER'
    try:
        r = json.loads(reply)
    except ValueError:
        r = None
    if code == 409 or code >= 500:
        return YELLOW, f'NOT TAKEN NOW (HTTP {code}' + (', will resend)' if resend else ', not resent)')
    if code >= 300:
        return RED, f'REJECTED (HTTP {code})'
    if isinstance(r, dict) and r.get('status') == 'accepted':
        marks = r.get('evaluation')
        if isinstance(marks, dict) and not all(marks.values()):
            wrong = ', '.join(k for k, v in marks.items() if not v)
            return YELLOW, f'ACCEPTED, WRONG: {wrong}'
        return GREEN, 'ACCEPTED'
    return RED, f'HTTP {code}, NOT ACCEPTED'


def pretty(text):
    try:
        return json.dumps(json.loads(text), ensure_ascii=False)
    except (ValueError, TypeError):
        return str(text).strip()


LOG_DIR = os.path.expanduser('~/challenge_ws/logs/ros')
LINE = re.compile(r'^\[(\w+)\] \[(\d+(?:\.\d+)?)\] \[task_reporter\]: (.*)$')
REPLY = [  # (pattern, http code or None = in the pattern) for the node's own log lines
    (re.compile(r'(M0\d) server: (.*)$'), 200),
    (re.compile(r'(M0\d) rejected: HTTP (\d+) (.*)$'), None),
    (re.compile(r'(M0\d) not taken now \(HTTP (\d+) (.*)\): kept'), None),
    (re.compile(r'(M0\d) post failed \(try \d of 3\): (.*)$'), 0),
    (re.compile(r'(M0\d) not posted: (.*)$'), -1),
]


def read_log(path):
    """One task_reporter log -> (url, [entry]); entry = dict(time, task_id, sent, http,
    reply), http None = no reply logged (not posted, or the log ends first)."""
    url, entries, pending = None, [], {}     # pending: task_id -> entries waiting a reply
    last = {}                                # task_id -> last payload (resends log no report)
    with open(path, errors='replace') as f:
        for line in f:
            m = LINE.match(line.rstrip('\n'))
            if not m:
                continue
            t, text = float(m.group(2)), m.group(3)
            u = re.search(r'reports go to (\S+)|by POST to (\S+) as', text)
            if u:
                url = url or u.group(1) or u.group(2)
            if 'no POST' in text:
                url = url or '(no POST)'
            if text.startswith('report: '):
                try:
                    sent = ast.literal_eval(text[len('report: '):])
                except (ValueError, SyntaxError):
                    continue
                e = {'time': t, 'task_id': sent.get('task_id'), 'sent': sent,
                     'http': None, 'reply': ''}
                entries.append(e)
                pending.setdefault(e['task_id'], []).append(e)
                last[e['task_id']] = sent
                continue
            for pat, code in REPLY:
                r = pat.match(text)
                if not r:
                    continue
                task = r.group(1)
                if code is None:
                    code, reply = int(r.group(2)), r.group(3)
                else:
                    reply = r.group(2)
                waiting = pending.get(task) or []
                if waiting:
                    e = waiting.pop(0)
                else:                    # a resend: same report again, new answer
                    e = {'time': t, 'task_id': task, 'sent': last.get(task), 'http': None,
                         'reply': '', 'resend': True}
                    entries.append(e)
                e['http'], e['reply'], e['reply_time'] = code, reply, t
                e['kept'] = 'not taken now' in pat.pattern   # this node resends it
                break
    return url, entries


def history(select):
    paths = glob.glob(os.path.join(LOG_DIR, '*', 'task_reporter*.log')) + \
        glob.glob(os.path.join(LOG_DIR, 'task_reporter_*.log'))
    runs = []
    for p in paths:
        url, entries = read_log(p)
        if entries:
            runs.append((entries[0]['time'], p, url, entries))
    runs.sort()
    if select is None:
        pass
    elif select.isdigit():
        runs = runs[-int(select):]
    else:
        runs = [r for r in runs if select in r[1]]
    if not runs:
        print(f'no reports in the task_reporter logs under {LOG_DIR}'
              + (f' matching {select!r}' if select else ''))
        return
    for start, path, url, entries in runs:
        name = os.path.relpath(path, LOG_DIR)
        print(f'{BOLD}=== run {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start))}'
              f'  {name}{END}\n    to {url or "?"}\n')
        best = {}
        for n, e in enumerate(entries, 1):
            stamp = time.strftime('%H:%M:%S', time.localtime(e['time']))
            if e['http'] is None and url == '(no POST)':
                colour, word = DIM, 'NOT POSTED (posting was off in that run)'
            elif e['http'] is None:
                colour, word = DIM, 'NO REPLY IN THE LOG'
            elif e['http'] == -1:
                colour, word = RED, 'NOT POSTED (format check)'
            elif e['http'] == 0:
                colour, word = RED, 'NO ANSWER'
            else:
                colour, word = verdict(e['http'], e['reply'], e.get('kept', False))
            again = ' (resent)' if e.get('resend') else ''
            print(f'{BOLD}{stamp}  #{n}  {e["task_id"]}{END}{again}\n'
                  f'  sent : {pretty(json.dumps(e["sent"]))}\n'
                  f'  reply: {colour}{word}{END}  {pretty(e["reply"])}\n')
            try:
                r = json.loads(e['reply'])
                if isinstance(r, dict) and 'score' in r:
                    best[e['task_id']] = f'{r["score"]}/{r.get("max_score", "?")}'
            except (ValueError, TypeError):
                pass
        if best:
            print('    latest score per mission: '
                  + ', '.join(f'{k} {v}' for k, v in sorted(best.items())) + '\n')


class ReportWatch(Node):
    def __init__(self, show_retries=True):
        super().__init__('report_watch')
        self.show_retries = show_retries
        self.last = {}          # task_id -> (sent, http, reply) of the last line printed
        self.count = 0
        self.create_subscription(String, '/mission/server_log', self.on_log, QoSProfile(
            depth=200, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        print(f'{DIM}waiting for reports on /mission/server_log ...{END}', flush=True)

    def on_log(self, msg):
        try:
            e = json.loads(msg.data)
        except ValueError:
            print(msg.data, flush=True)
            return
        key = (json.dumps(e.get('sent'), sort_keys=True), e.get('http'), e.get('reply'))
        task = e.get('task_id')
        if not self.show_retries and self.last.get(task) == key:
            return              # the same report and the same answer as last time
        self.last[task] = key
        self.count += 1
        colour, word = verdict(e.get('http'), e.get('reply') or '')
        stamp = time.strftime('%H:%M:%S', time.localtime(e.get('time', time.time())))
        attempt = e.get('attempt', 1)
        tries = f' (try {attempt})' if attempt and attempt > 1 else ''
        print(f'{BOLD}{stamp}  #{self.count}  {task}{END}  -> {e.get("url")}{tries}\n'
              f'  sent : {pretty(json.dumps(e.get("sent")))}\n'
              f'  reply: {colour}{word}{END}  {pretty(e.get("reply"))}\n', flush=True)


def main():
    if '--history' in sys.argv:
        i = sys.argv.index('--history')
        arg = sys.argv[i + 1] if i + 1 < len(sys.argv) and \
            not sys.argv[i + 1].startswith('-') else None
        history(arg)
        return
    rclpy.init(args=sys.argv)
    node = ReportWatch(show_retries='--no-retry' not in sys.argv)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
