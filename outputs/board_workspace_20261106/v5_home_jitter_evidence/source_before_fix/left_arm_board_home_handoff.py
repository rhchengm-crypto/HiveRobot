"""Explicit Home recovery on the existing arm/serial connection.

SIGUSR1 only requests Home; SIGTERM/SIGINT and physical E-stop are untouched.
The Web server must verify the advertised capability before sending SIGUSR1.
"""
from __future__ import annotations

import json
import signal
import sys
import time

from left_arm_fault_support import BoardWatchdog


def run_home_on_existing_arm(arm, legacy, home_main=None):
    if home_main is None:
        from left_arm_v2_8 import main as home_main
    cls = legacy.LeftArmV2
    if not isinstance(arm, cls):
        raise TypeError('Home handoff requires the existing LeftArmV2 instance')
    missing = object()
    previous_new = cls.__dict__.get('__new__', missing)
    previous_init = cls.__init__
    previous_close = arm.__dict__.get('close', missing)
    previous_argv = sys.argv
    previous_gravity = getattr(arm, 'board_gravity_feedforward', False)
    previous_deadbands = getattr(legacy, 'HOME_JOINT_DEADBANDS_DEG', missing)
    try:
        # v2.8 performs multiple v2.6 passes. Each receives this same object;
        # constructors, enable caches and serial ports are not recreated.
        cls.__new__ = staticmethod(lambda _cls, *args, **kwargs: arm)
        cls.__init__ = lambda self, *args, **kwargs: None
        arm.close = lambda: None
        arm.board_gravity_feedforward = False  # Preserve the trained Home controller.
        sys.argv = ['left_arm_v2_8.py', 'home', '--deadband-deg', '.5',
                    '--max-delta-deg', '180', '--prehome-clearance', '--execute']
        home_main()
    finally:
        cls.__init__ = previous_init
        if previous_new is missing:
            delattr(cls, '__new__')
        else:
            cls.__new__ = previous_new
        if previous_close is missing:
            delattr(arm, 'close')
        else:
            arm.close = previous_close
        sys.argv = previous_argv
        arm.board_gravity_feedforward = previous_gravity
        if previous_deadbands is not missing:
            legacy.HOME_JOINT_DEADBANDS_DEG = previous_deadbands
    return {'ok': True, 'motion_issued': True, 'recovered_action': 'home',
            'same_serial_session': True, 'automatic_trajectory_resume': False}


def install_home_handoff(supervisor, arm, legacy, home_main=None):
    supervisor.home_requested = False
    supervisor.home_active = False
    supervisor.home_handoff_available = False
    supervisor.previous_home_signal = None
    if hasattr(signal, 'SIGUSR1') and getattr(arm, 'allow_home_handoff', False) is True:
        supervisor.previous_home_signal = signal.getsignal(signal.SIGUSR1)
        signal.signal(signal.SIGUSR1, lambda signum, frame: setattr(supervisor, 'home_requested', True))
        supervisor.home_handoff_available = True
    supervisor.home_callback = lambda: run_home_on_existing_arm(arm, legacy, home_main)
    supervisor.home_guard = BoardWatchdog(arm, supervisor, .3, 5.,
        context={'stage': 'explicit_home_handoff'}, check_every_calls=1, record_only=True)
    supervisor.next_home_guard = 0.


def close_home_handoff(supervisor):
    if getattr(supervisor, 'home_handoff_available', False):
        signal.signal(signal.SIGUSR1, supervisor.previous_home_signal)
        supervisor.home_handoff_available = False


def home_command_observed(supervisor, name, kp, kd):
    if not getattr(supervisor, 'home_active', False):
        return
    supervisor.gains[name] = {'kp': float(kp), 'kd': float(kd)}
    now = supervisor.clock()
    if now >= supervisor.next_home_guard:
        supervisor.next_home_guard = now+.08
        reference = {n: supervisor.last_commands.get(n, {}).get('q', supervisor.snapshot[n]['pos'])
                     for n in supervisor.names}
        supervisor.home_guard(reference)


def process_home_request(supervisor):
    """Called only from the support loop, never inside a signal handler."""
    if not getattr(supervisor, 'home_requested', False):
        return None
    supervisor.home_requested = False
    supervisor.step()
    print('board home handoff= '+json.dumps({'state': 'starting', 'same_serial_session': True}), flush=True)
    # A fresh feedback snapshot is maintained by the Home command observer.
    supervisor.latched = False
    supervisor.home_active = True
    try:
        result = supervisor.home_callback()
    except Exception as exc:
        supervisor.home_active = False
        try:
            supervisor.observe(supervisor.arm.read_status(supervisor.names))
        except Exception:
            pass  # Keep the last valid Home feedback, not the old board target.
        supervisor.latch(exc)
        print('board home handoff= '+json.dumps({'state': 'failed-protective-support',
                                                'error': str(exc)}), flush=True)
        return None
    supervisor.home_active = False
    print('board home handoff= '+json.dumps({'state': 'complete', 'same_serial_session': True}), flush=True)
    return result
