"""Make a node's parameters changeable while it runs (`ros2 param set`).

Each node reads its parameters once in __init__, mostly into a cfg dict. Without a callback
`ros2 param set` stores the new value but the node keeps using the old one. attach() adds
an on-set callback that:

  * updates the entry in each of the node's cfg dicts holding that name (code that reads
    cfg[name] when it runs sees the new value at once);
  * calls a setter for values copied elsewhere at startup (dataclass fields, tuples,
    helper objects);
  * REJECTS everything else, with a reason (`ros2 param set` prints it): a parameter
    read only at startup (topics, frames, timer rates) would be stored but ignored, which
    is worse than a clear "restart the node";
  * rejects `locked` names: safety or identity values never changed mid-run;
  * logs a warning and publishes one JSON line on /troubleshoot/param_change, so the
    telemetry snapshots and events.log record every change.

A set is all or nothing: if one name is rejected or a setter raises, nothing changes.

Call it at the END of __init__: rclpy also runs on-set callbacks for declare_parameter.
"""
import json
import time

from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

CHANGE_TOPIC = '/troubleshoot/param_change'


def _plain(v):
    """Parameter value as JSON-safe Python (arrays come back as array.array)."""
    if isinstance(v, (list, tuple)) or type(v).__name__ == 'array':
        return [_plain(x) for x in v]
    return v


def value(node, name):
    """A parameter's value for setters: the new one while a set is being applied (rclpy
    stores it only after the callback accepts), else the stored one."""
    pending = getattr(node, '_live_pending', None) or {}
    return pending[name] if name in pending else _plain(node.get_parameter(name).value)


def attach(node, cfgs=(), setters=None, locked=()):
    """Register the live-param callback on `node`.

    cfgs:    dicts {param name: value} the node reads at run time (self.cfg, ...).
    setters: {param name: fn(new_value)} for values copied elsewhere at startup. Called
             after the cfg dicts are updated; a setter combining several params (e.g. a
             footprint tuple) reads the others with value(node, name), which already
             returns the new values of the same set.
    locked:  names never changeable live, even if they are in a cfg dict.
    """
    cfgs = [c for c in cfgs if c is not None]
    setters = dict(setters or {})
    locked = set(locked)
    pub = node.create_publisher(String, CHANGE_TOPIC, 10)
    log = node.get_logger()

    def live(name):
        return name in setters or any(name in c for c in cfgs)

    def on_set(params):
        for prm in params:
            if prm.name in locked:
                return SetParametersResult(
                    successful=False, reason=f'{prm.name}: locked, not changeable while running')
            if not live(prm.name):
                return SetParametersResult(
                    successful=False,
                    reason=f'{prm.name}: read only at startup; restart {node.get_name()} '
                           'with the new value (launch arg or -p)')
        undo = []            # (dict, name, old value)
        changes = []
        node._live_pending = {prm.name: _plain(prm.value) for prm in params}
        try:
            for prm in params:
                name, value = prm.name, _plain(prm.value)
                old = _plain(node.get_parameter(name).value)
                for c in cfgs:
                    if name in c:
                        undo.append((c, name, c[name]))
                        c[name] = value
                changes.append((name, old, value))
            for name, old, value in changes:
                if name in setters:
                    setters[name](value)
        except Exception as e:   # noqa: BLE001 - any setter failure rejects the whole set
            for c, name, old in reversed(undo):
                c[name] = old
            # setters already run keep the new value: re-run them with the old ones
            node._live_pending = None
            for name, old, _ in changes:
                if name in setters:
                    try:
                        setters[name](old)
                    except Exception:   # noqa: BLE001
                        pass
            return SetParametersResult(successful=False, reason=f'rejected: {e}')
        finally:
            node._live_pending = None
        for name, old, value in changes:
            log.warning(f'live param {name}: {old!r} -> {value!r}')
            pub.publish(String(data=json.dumps({
                'node': node.get_name(), 'name': name, 'old': old, 'new': value,
                'wall_time': time.time()})))
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(on_set)
    return on_set
