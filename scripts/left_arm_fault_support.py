"""Same-process latched support after a software trajectory stop.

No motor enables/disables, no trajectory continuation, no learning writes.
This cannot provide support after loss of power, a disabled motor or a failed
communications link. Explicit SIGTERM/SIGKILL and physical E-stop are not
intercepted. The process intentionally remains alive during protective hold.
"""
from __future__ import annotations

import copy
from collections import deque
import json
import math
import time

from left_arm_v3_1_model_control import MIT_TAU_MAX, clamp


class MotionStop(RuntimeError):
    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


def clean_feedback(status, names):
    result = {}
    for name in names:
        row = {k: float(status[name][k]) for k in ('pos', 'vel', 'tau')}
        if not all(math.isfinite(v) for v in row.values()) or abs(row['tau']) > MIT_TAU_MAX[name]:
            raise ValueError('invalid support feedback for '+name)
        result[name] = row
    return result


class FaultSupport:
    def __init__(self, arm, names, gains, *, clock=time.monotonic, sleep=time.sleep,
                 emit=None):
        self.arm = arm
        self.names = tuple(names)
        self.gains = {n: {'kp': float(gains[n]['kp']), 'kd': float(gains[n]['kd'])} for n in names}
        if any(not 0 <= g['kp'] <= 500 or not 0 <= g['kd'] <= 5 for g in self.gains.values()):
            raise ValueError('invalid support gains')
        self.clock, self.sleep = clock, sleep
        self.emit = emit or (lambda row: print('board protective hold= '+json.dumps(row), flush=True))
        self.active = False
        self.latched = False
        self.snapshot = None
        self.snapshot_at = None
        self.targets = {}
        self.support_tau = {}
        self.bridge_tau = {}
        self.failure_count = 0
        self.logging_failure_count = 0
        self.started = None
        self.next_read = 0.
        self.next_log = 0.
        self.last_feedback_at = None
        self.latest_velocity = {}
        self.last_io_failures = []
        self.last_commands = {}
        self.max_drift_deg = {n: 0. for n in names}

    def observe(self, status):
        if self.latched:
            return  # Never move a latched target to follow a drifting arm.
        self.snapshot = clean_feedback(status, self.names)
        self.snapshot_at = self.clock()

    def activate(self, status):
        self.observe(status)
        self.active = True

    def latch(self, cause):
        if self.latched:
            return
        if not self.active or self.snapshot is None:
            raise RuntimeError('support was not armed before the motion')
        self.started = self.clock()
        self.reason = str(cause)
        self.diagnostics = getattr(cause, 'diagnostics', {})
        self.feedback_age_ms = (self.started-self.snapshot_at)*1000
        for n in self.names:
            row = self.snapshot[n]
            self.targets[n] = row['pos']
            self.support_tau[n] = row['tau']
            # Same measured-load/damping bridge as the existing Model Hold:
            # at q_target=q_measured, the first net torque is measured_tau.
            self.bridge_tau[n] = clamp(row['tau']+self.gains[n]['kd']*row['vel'],
                                       -MIT_TAU_MAX[n], MIT_TAU_MAX[n])
        self.latched = True
        self.step()  # Refresh support before logging or other cleanup.
        trace = getattr(cause, 'trace', None)
        if trace:
            try:
                print('board watchdog trace= '+json.dumps(trace), flush=True)
            except Exception:
                self.logging_failure_count += 1

    def step(self):
        if not self.latched:
            raise RuntimeError('support not latched')
        now = self.clock()
        elapsed = now-self.started
        # Decay only the initial damping cancellation, never the load support.
        alpha = min(1., max(0., elapsed/.75))
        alpha = alpha*alpha*(3-2*alpha)
        failures = []
        for n in self.names:
            gain = self.gains[n]
            tau = self.bridge_tau[n]*(1-alpha)+self.support_tau[n]*alpha
            try:
                self.arm.ctrl.controlMIT(self.arm.motors[n], gain['kp'], gain['kd'],
                                         self.targets[n], 0., tau)
            except Exception as exc:
                # Continue refreshing the other joints; never auto re-enable
                # a faulted motor or deliberately remove their support.
                failures.append({'joint': n, 'error': str(exc)})
        if now >= self.next_read:
            self.next_read = now+.25
            try:
                feedback = clean_feedback(self.arm.read_status(self.names), self.names)
                self.last_feedback_at = self.clock()
                self.latest_velocity = {n: feedback[n]['vel'] for n in self.names}
                for n in self.names:
                    self.max_drift_deg[n] = max(self.max_drift_deg[n],
                        abs(math.degrees(feedback[n]['pos']-self.targets[n])))
            except Exception as exc:
                failures.append({'feedback_error': str(exc)})
        self.failure_count += len(failures)
        if failures:
            self.last_io_failures = failures
        if now >= self.next_log:
            self.next_log = now+1.
            report = {'state': 'degraded' if self.failure_count else 'support-commanding',
                       'driver_enable_verified': False,
                       'physical_load_support_verified': False,
                       'home_handoff_available': getattr(self, 'home_handoff_available', False),
                       'motion_aborted': True, 'automatic_resume': False,
                       'reason': self.reason, 'diagnostics': self.diagnostics,
                       'hold_elapsed_s': elapsed, 'snapshot_age_at_entry_ms': self.feedback_age_ms,
                       'targets_rad': dict(self.targets), 'load_support_tau': dict(self.support_tau),
                       'max_drift_deg': dict(self.max_drift_deg),
                       'latest_velocity_rad_s': dict(self.latest_velocity),
                       'feedback_age_ms': None if self.last_feedback_at is None else
                                          (self.clock()-self.last_feedback_at)*1000.,
                       'io_failure_count': self.failure_count, 'io_failures': failures,
                       'last_io_failures': self.last_io_failures,
                       'requires_operator_attention': True}
            try:
                self.emit(report)
            except Exception:
                # A closed browser/log pipe must not unwind the load-support loop.
                self.logging_failure_count += 1

    def run(self, cause):
        self.latch(cause)
        while True:
            if getattr(self, 'home_requested', False):
                from left_arm_board_home_handoff import process_home_request
                result = process_home_request(self)
                if result is not None:
                    return result
            self.step()
            self.sleep(.01)


class BoardWatchdog:
    def __init__(self, arm, support, max_velocity, max_error_deg, *,
                 inject_after_checks=None, context=None, check_every_calls=8,
                 record_only=False):
        self.arm, self.support = arm, support
        self.max_velocity, self.max_error_deg = max_velocity, max_error_deg
        self.calls = self.checks = 0
        self.inject_after_checks = inject_after_checks
        self.context = context or {}
        self.samples = deque(maxlen=32)
        self.check_every_calls = int(check_every_calls)
        if self.check_every_calls < 1:
            raise ValueError('invalid feedback check interval')
        self.record_only = bool(record_only)
        self.next_audit_log = 0.
        self.warning_log_failures = 0
        self.velocity_warning_active = dict.fromkeys(support.names, False)
        self.velocity_warning_count = dict.fromkeys(support.names, 0)
        self.position_warning_active = dict.fromkeys(support.names, False)

    def _log(self, prefix, data):
        try:
            print(prefix+json.dumps(data),flush=True)
        except Exception:
            self.warning_log_failures += 1

    def __call__(self, reference):
        self.calls += 1
        if (self.calls-1) % self.check_every_calls != 0:
            return
        self.checks += 1
        feedback_read_started_wall_s = time.time()
        status = self.arm.read_status(self.support.names)
        feedback_read_finished_wall_s = time.time()
        self.support.observe(status)
        now = self.support.clock()
        sample_timing = {'feedback_read_started_wall_s': feedback_read_started_wall_s,
                         'feedback_read_finished_wall_s': feedback_read_finished_wall_s,
                         'feedback_sample_monotonic_s': now}
        self.samples.append({'time':now,'feedback':copy.deepcopy(self.support.snapshot),
                             **sample_timing,
                             'reference_rad':dict(reference),
                             'last_sent_commands':copy.deepcopy(self.support.last_commands)})
        violations = []
        changed = False
        for n,row in self.support.snapshot.items():
            error = abs(math.degrees(row['pos']-reference[n]))
            diagnostic = {**self.context,'joint':n,'position_rad':row['pos'],
                'velocity_rad_s':row['vel'],'tau':row['tau'],'reference_rad':reference[n],
                'error_deg':error,'velocity_limit_rad_s':self.max_velocity,
                'error_limit_deg':self.max_error_deg}
            speed = abs(row['vel']) > self.max_velocity
            tracking = error > self.max_error_deg
            if (speed or tracking) and not self.record_only:
                message = (f'board velocity watchdog: {n} {row["vel"]:.3f}rad/s' if speed else
                           f'board tracking watchdog: {n} {error:.2f}deg')
                fault=MotionStop(message,diagnostic)
                fault.trace=list(self.samples)
                raise fault
            if speed or tracking:
                violations.append({**diagnostic,'velocity_exceeded':speed,'tracking_exceeded':tracking})
            if speed and not self.velocity_warning_active[n]:
                self.velocity_warning_count[n] += 1
            changed = changed or speed != self.velocity_warning_active[n] or tracking != self.position_warning_active[n]
            self.velocity_warning_active[n]=speed
            self.position_warning_active[n]=tracking
        if self.record_only and (now >= self.next_audit_log or changed):
            self.next_audit_log=now+.5
            self._log('board tracking audit= ',{**self.context,'mode':'record-only',
                **sample_timing,
                'protection_triggered':False,'trajectory_continues':True,
                'feedback':self.support.snapshot,'reference_rad':dict(reference),
                'error_deg':{n:math.degrees(row['pos']-reference[n]) for n,row in self.support.snapshot.items()},
                'violations':violations,'last_sent_commands':self.support.last_commands})
        if self.inject_after_checks is not None and self.checks >= self.inject_after_checks:
            raise MotionStop('injected software velocity-watchdog stop; no fake motor feedback',
                             {**self.context,'test_injected':True,'feedback':copy.deepcopy(status)})


def segment_timing(delta_rad, feedback_velocity_limit=.2):
    """Quintic velocity and acceleration budgets, not motor gain tuning."""
    delta = abs(float(delta_rad))
    limit = float(feedback_velocity_limit)
    if not math.isfinite(delta) or not math.isfinite(limit) or limit <= 0:
        raise ValueError('invalid timing input')
    reference_limit = .35*limit
    acceleration_limit = .1
    duration = max(.8, 1.875*delta/reference_limit, math.sqrt(5.7735026919*delta/acceleration_limit))
    return {'seconds': duration, 'reference_peak_velocity_rad_s': 1.875*delta/duration,
            'reference_peak_acceleration_rad_s2': 5.7735026919*delta/duration**2,
            'feedback_velocity_limit_rad_s': limit}
