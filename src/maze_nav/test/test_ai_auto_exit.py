"""cmd_vel_bridge AI mode at the stairs (user 2026-11-21): walk done -> leave AI mode by
itself (auto mode included) and report DONE; a walk that ended early holds. No ROS."""
from types import SimpleNamespace

from maze_nav.cmd_vel_bridge import AI_EXIT, AUTO_MODE, CmdVelBridge, MOVE_MODE


class Fake:
    for _n in ('end_walk', 'exit_ai', 'run_ai_steps', 'ai_tick', 'set_ai_status', 'send_simple',
               'on_start'):
        locals()[_n] = getattr(CmdVelBridge, _n)

    def __init__(self):
        self.sent, self.statuses, self.logs = [], [], []
        log = SimpleNamespace(info=self.logs.append, warn=self.logs.append,
                              error=self.logs.append)
        self.get_logger = lambda: log
        self.simple_cmd = lambda cmd_code, size, type: cmd_code
        self.simple_pub = SimpleNamespace(publish=self.sent.append,
                                          get_subscription_count=lambda: 1)
        self.ai_status_pub = SimpleNamespace(publish=lambda m: self.statuses.append(m.data))
        self.pub = SimpleNamespace(publish=lambda m: None)
        self.ai_status, self.ai_mode, self.ai_exit_wanted, self.ai_auto = None, True, False, False
        self.ai_auto_exit, self.drive, self.ai_settle = True, True, 0.0
        self.gait_code, self.gait_req = 0x21010401, 'rug'
        self.walk = {'start': (0.0, 0.0)}
        self.ai_steps, self.ai_kind = [], None

    def simple_publisher(self):
        return True

    def run_steps(self):
        for _ in range(10):
            self.ai_next = 0.0
            self.ai_tick()


def test_walk_done_leaves_ai_mode_with_auto_mode_and_reports_done():
    b = Fake()
    b.end_walk('1.30 m done', done=True)
    b.run_steps()
    assert b.sent[0] == AI_EXIT and MOVE_MODE in b.sent and b.sent[-1] == AUTO_MODE
    assert not b.ai_mode and b.statuses[-1].startswith('DONE')


def test_walk_ended_early_holds_for_the_operator():
    b = Fake()
    b.end_walk('no progress for 8 s', warn=True)
    b.run_steps()
    assert b.sent == [] and b.ai_mode and b.statuses[-1].startswith('HOLDING')
    b.on_start(None)                                   # the operator's /maze/start
    b.run_steps()
    assert b.sent[0] == AI_EXIT and AUTO_MODE not in b.sent
    assert not b.ai_mode and b.statuses[-1] == 'IDLE'
