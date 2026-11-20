"""explorer.at_dead_end: /maze/perceive is True only while looking at a dead end."""
from types import SimpleNamespace

from maze_nav.explorer import Explorer


def at(state, junction=None):
    return Explorer.at_dead_end(SimpleNamespace(state=state, junction=junction))


def look(names, i, hold_until=None, dead_end=None):
    return {'steps': [{'name': n} for n in names], 'i': i, 'hold_until': hold_until,
            'dead_end': dead_end}


def test_states():
    for s in ('DEAD_END', 'FACING_QR', 'LOOKING'):
        assert at(s)
    for s in ('EXPLORING', 'TURNING', 'SCANNING', 'PAUSED', 'STAIRS', 'CURTAIN', 'CREEPING'):
        assert not at(s)
    assert not at('JUNCTION', None)


def test_junction_look_steps():
    names = ['junction way 1', 'dead end left wall', 'dead end right wall', 'junction way 2']
    assert not at('JUNCTION', look(names, 1))            # facing an open way
    assert at('JUNCTION', look(names, 2))                # scanning a dead end's side wall
    assert at('JUNCTION', look(names, 3))
    assert not at('JUNCTION', look(names, 4))
    # holding at way 1 because it faces a dead end: its side-wall steps come next
    assert at('JUNCTION', look(names, 1, hold_until=5.0))
    assert at('JUNCTION', look(names[:1], 1, dead_end=[{'name': 'dead end left wall'}]))


def test_junction_follow_up():
    g = 'dead end [2, 1]'
    j = look(['junction way 1'], 1)
    for do, group, want in (('goto', g, False), ('align', g, True), ('hold', g, True),
                            ('goto', 'rightmost', False)):
        j['todo'] = [{'do': do, 'group': group}]
        assert at('JUNCTION', j) is want, (do, group)
    j['todo'] = []
    assert not at('JUNCTION', j)
