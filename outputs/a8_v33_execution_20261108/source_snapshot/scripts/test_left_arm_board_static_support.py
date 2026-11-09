"""Offline v33 regression: seven simulated joints, no SDK or serial access.

The plant solves a static spring/gravity equilibrium, not motor dynamics.
Receive callbacks, packet clocks, observation filtering, the actual board
support baseline, and (in the integration test) execute's command capture and
segment bridge run normally. Geometry planning and the motion primitive are
replaced with deterministic in-memory fixtures.
"""
import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import copy
from contextlib import redirect_stdout
import io
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np

import left_arm_board_grasp as board
import left_arm_static_support as static
from left_arm_trajectory_support import gravity_envelope


NAMES = ('shoulder_front', 'shoulder_side', 'shoulder_rotate', 'elbow',
         'arm_roll', 'wrist_side', 'wrist')


class ReplyMotor:
    def __init__(self, name, tau=0.):
        self.name, self.pos, self.vel, self.tau = name, 0., 0., tau

    def recv_data(self, q, dq, tau):
        self.pos, self.vel, self.tau = q, dq, tau

    def getPosition(self):
        return self.pos

    def getVelocity(self):
        return self.vel

    def getTorque(self):
        return self.tau


class StaticPlant:
    """Kp*(reference-position)+FF = entry_load+true_slope*position."""
    def __init__(self, modeled_slope=.4):
        self.gains = {n: {'kp': 20., 'kd': 1.} for n in NAMES}
        self.entry = {n: .2+.05*i for i, n in enumerate(NAMES)}
        self.true_slope = {n: 3.+.2*i for i, n in enumerate(NAMES)}
        self.modeled_slope = modeled_slope
        self.model = types.SimpleNamespace(
            names=NAMES, n=7, signs=np.ones(7),
            encoder_to_model=lambda q: np.asarray(q, dtype=float),
            gravity_torques=lambda q: modeled_slope*np.asarray(q))

    def equilibrium(self, commands):
        result = {}
        for name, command in commands.items():
            position = ((command['kp']*command['q']+command['tau_ff']-self.entry[name])
                        / (command['kp']+self.true_slope[name]))
            result[name] = dict(pos=position, vel=0.,
                                tau=self.entry[name]+self.true_slope[name]*position)
        return result

    def baseline(self, *, hardware=True, limits=(-2., 2.)):
        zero = dict.fromkeys(NAMES, 0.)
        status = {n: dict(pos=0., vel=0., tau=self.entry[n]) for n in NAMES}
        return board.TrajectorySupportBaseline(
            self.model, zero, self.entry, status, self.gains,
            dict.fromkeys(NAMES, limits), dict.fromkeys(NAMES, 8.) if hardware else None)

    def command(self, baseline, target):
        status = {n: dict(pos=target[n], vel=0., tau=self.entry[n]) for n in NAMES}
        support, _ = baseline.command(target, status, self.gains)
        return {n: dict(self.gains[n], q=target[n], dq=0., tau_ff=support[n]['end_tau'])
                for n in NAMES}


class PacketWindows:
    def __init__(self):
        self.now = 10.
        self.motors = {n: ReplyMotor(n) for n in NAMES}
        assert static.install_feedback_clocks(self.motors)['complete']

    def frames(self, commands, status):
        result = []
        for _ in range(17):
            self.now += .02
            with patch('left_arm_static_support.time.monotonic', return_value=self.now):
                for name, motor in self.motors.items():
                    row = status[name]
                    motor.recv_data(row['pos'], row['vel'], row['tau'])
            frame = dict(event='motor_frame', monotonic_s=self.now,
                         commands=copy.deepcopy(commands), feedback=copy.deepcopy(status),
                         context={'stage': 'irrelevant-label'})
            frame.update(static.feedback_clock_fields(self.motors, self.now))
            result.append(frame)
        return result

    def observation(self, commands, status, target):
        rows = self.frames(commands, status)
        observation = static.stable_support_observation(rows, NAMES, target, self.now+.005)
        return observation, rows


class BoardStaticSupportTests(unittest.TestCase):
    def setUp(self):
        self.plant = StaticPlant()
        self.baseline = self.plant.baseline()
        self.target = dict.fromkeys(NAMES, .25)
        self.windows = PacketWindows()

    def learn(self):
        command = self.plant.command(self.baseline, self.target)
        status = self.plant.equilibrium(command)
        observation, _ = self.windows.observation(command, status, self.target)
        self.assertTrue(observation['accepted'], observation)
        update = self.baseline.observe_static_support(observation, self.plant.gains)
        return command, status, observation, update

    def test_repeating_identical_q_and_gains_preserves_old_static_error(self):
        command = self.plant.command(self.baseline, self.target)
        initial = self.plant.equilibrium(command)
        self.assertGreater(max(abs(initial[n]['pos']-self.target[n]) for n in NAMES), .02)
        for _ in range(10):
            repeated = self.plant.command(self.baseline, self.target)
            self.assertEqual(repeated, command)
            self.assertEqual(self.plant.equilibrium(repeated), initial)
        self.assertEqual(self.baseline.static_bias, dict.fromkeys(NAMES, 0.))

    def test_fresh_windows_learn_absolute_residual_and_reduce_equilibrium_error(self):
        anchor = copy.deepcopy(self.baseline.anchor)
        original_gains, original_target = copy.deepcopy(self.plant.gains), dict(self.target)
        original_limits = copy.deepcopy(self.baseline.model_limits)
        initial_error = None
        previous = dict(self.baseline.static_bias)
        for _ in range(14):
            command, status, observation, update = self.learn()
            error = max(abs(status[n]['pos']-self.target[n]) for n in NAMES)
            initial_error = error if initial_error is None else initial_error
            self.assertTrue(update['updated'])
            for name in NAMES:
                residual = ((self.plant.true_slope[name]-self.plant.modeled_slope)
                            * status[name]['pos'])
                self.assertAlmostEqual(update['observed_residual_tau'][name], residual)
                self.assertAlmostEqual(self.baseline.static_bias[name],
                                       .5*(previous[name]+residual))
                self.assertEqual(command[name]['q'], original_target[name])
                self.assertEqual((command[name]['kp'], command[name]['kd']), (20., 1.))
            previous = dict(self.baseline.static_bias)
        final = self.plant.equilibrium(self.plant.command(self.baseline, self.target))
        self.assertLess(max(abs(final[n]['pos']-self.target[n]) for n in NAMES), initial_error*.002)
        self.assertEqual(self.baseline.anchor, anchor)
        self.assertEqual(self.baseline.initial_gravity, dict.fromkeys(NAMES, 0.))
        self.assertEqual(self.baseline.model_limits, original_limits)
        self.assertEqual(self.plant.gains, original_gains)
        self.assertEqual(self.target, original_target)

    def test_duplicate_and_older_windows_cannot_learn_twice(self):
        _, _, observation, _ = self.learn()
        saved = copy.deepcopy(self.baseline.static_bias)
        for age in (0., -.01):
            repeated = copy.deepcopy(observation)
            repeated['window_end_s'] += age
            result = self.baseline.observe_static_support(repeated, self.plant.gains)
            self.assertFalse(result['updated'])
            self.assertEqual(result['reason'], 'observation-already-consumed')
            self.assertEqual(self.baseline.static_bias, saved)

    def test_unverified_packets_and_moving_references_never_learn(self):
        command = self.plant.command(self.baseline, self.target)
        status = self.plant.equilibrium(command)
        for mutation, expected in (('clocks', 'individual-feedback-freshness-unverified'),
                                   ('q', 'moving-or-mismatched-reference'),
                                   ('dq', 'moving-or-mismatched-reference')):
            with self.subTest(mutation=mutation):
                rows = self.windows.frames(command, status)
                if mutation == 'clocks':
                    rows[8]['individual_feedback_freshness_verified'] = False
                else:
                    rows[8]['commands'][NAMES[0]][mutation] += .01
                observed = static.stable_support_observation(rows, NAMES, self.target, self.windows.now+.005)
                self.assertEqual(observed['reason'], expected)
                self.assertFalse(self.baseline.observe_static_support(observed, self.plant.gains)['updated'])
                self.assertEqual(self.baseline.static_bias, dict.fromkeys(NAMES, 0.))

    def test_no_hardware_bounds_or_invalid_load_cannot_update(self):
        command = self.plant.command(self.baseline, self.target)
        observed, _ = self.windows.observation(command, self.plant.equilibrium(command), self.target)
        no_bounds = self.plant.baseline(hardware=False)
        self.assertEqual(no_bounds.observe_static_support(observed, self.plant.gains)['reason'],
                         'hardware-torque-bounds-unavailable')
        for mutation in ('nan-time', 'over-range', 'nan-torque'):
            invalid = copy.deepcopy(observed)
            if mutation == 'nan-time':
                invalid['window_end_s'] = float('nan')
            else:
                invalid['torques'][NAMES[0]] = 8.01 if mutation == 'over-range' else float('nan')
            self.assertFalse(self.baseline.observe_static_support(invalid, self.plant.gains)['updated'])
            self.assertEqual(self.baseline.last_static_observation, float('-inf'))
            self.assertEqual(self.baseline.static_bias, dict.fromkeys(NAMES, 0.))

    def test_envelope_headroom_anti_windup_and_live_range_never_shrinks(self):
        model = types.SimpleNamespace(names=NAMES, n=7, signs=np.ones(7),
            encoder_to_model=lambda q: np.asarray(q), gravity_torques=lambda q: .99*np.asarray(q))
        zero = dict.fromkeys(NAMES, 0.)
        status = {n: dict(pos=0., vel=0., tau=0.) for n in NAMES}
        baseline = board.TrajectorySupportBaseline(model, zero, zero, status,
            self.plant.gains, dict.fromkeys(NAMES, (-.99, .99)), dict.fromkeys(NAMES, 1.))
        original_base = copy.deepcopy(baseline.base_limits)
        observations = []
        for sign in (1., -1., 1.):
            for _ in range(8):
                command = {n: dict(self.plant.gains[n], q=0., dq=0., tau_ff=0.) for n in NAMES}
                feedback = {n: dict(pos=0., vel=0., tau=.8*sign) for n in NAMES}
                observed, _ = self.windows.observation(command, feedback, zero)
                old_range = dict(baseline.limits)
                report = baseline.observe_static_support(observed, self.plant.gains)
                observations.append(report)
                for name in NAMES:
                    self.assertAlmostEqual(baseline.static_bias[name], .01*sign)
                    self.assertTrue(report['joints'][name]['hardware_envelope_headroom_clamped'])
                    self.assertTrue(report['joints'][name]['saturated'])
                    self.assertLessEqual(baseline.limits[name][0], old_range[name][0])
                    self.assertGreaterEqual(baseline.limits[name][1], old_range[name][1])
                    self.assertGreaterEqual(baseline.limits[name][0], -1.)
                    self.assertLessEqual(baseline.limits[name][1], 1.)
                for corner in (-1., 1.):
                    output, _ = baseline.command(dict.fromkeys(NAMES, corner), status, self.plant.gains)
                    self.assertTrue(all(abs(row['end_tau']) <= 1.+1e-12 for row in output.values()))
        self.assertEqual(baseline.base_limits, original_base)
        self.assertEqual(baseline.model_limits, original_base)
        self.assertEqual(baseline.anchor, zero)
        self.assertTrue(all(bounds == (-1., 1.) for bounds in baseline.limits.values()))

    def test_new_learned_endpoint_bridges_from_literal_previous_command(self):
        old, status, _, _ = self.learn()
        support, _ = self.baseline.command(self.target, status, self.plant.gains)
        measured = {n: status[n]['pos'] for n in NAMES}
        bridged, audit = board.bridge_segment_feedforward(old, measured,
            dict.fromkeys(NAMES, 0.), self.plant.gains, support,
            self.baseline.hardware_limits, reference_starts=self.target)
        for name in NAMES:
            self.assertEqual(bridged[name]['start_tau'], old[name]['tau_ff'])
            self.assertEqual(bridged[name]['end_tau'], support[name]['end_tau'])
            self.assertEqual(bridged[name]['ramp'], 'smoothstep')
            self.assertEqual(bridged[name]['ramp_fraction'], 1.)
            self.assertAlmostEqual(audit[name]['previous_total_tau'], audit[name]['new_total_tau'])
        seconds = static.slew_duration({n: old[n]['tau_ff'] for n in NAMES},
                                      {n: bridged[n]['end_tau'] for n in NAMES}, .1)
        for name in NAMES:
            self.assertLessEqual(1.5*abs(bridged[name]['end_tau']-bridged[name]['start_tau'])/seconds, 1.+1e-12)

    def test_unloading_to_zero_uses_actual_support_timing_and_full_smoothstep(self):
        # Regress the all-zero-estimate early return: a zero desired residual
        # still needs time to unload the nonzero command already on the wire.
        previous = {n: dict(self.plant.gains[n], q=self.target[n], dq=0.,
                            tau_ff=(-1. if i % 2 else 1.)*(1.+.2*i))
                    for i, n in enumerate(NAMES)}
        desired = {n: dict(start_tau=0., end_tau=0., ramp_fraction=.25,
                           ramp='smoothstep') for n in NAMES}
        timing = {'seconds': .1, 'feedback_velocity_limit_rad_s': .2}
        originals = copy.deepcopy((previous, desired, timing))
        result = board.support_baseline_timing(timing, .02, desired, previous)
        self.assertAlmostEqual(result['seconds'], 1.5*2.2)
        self.assertEqual(result['support_baseline_slew_limit_Nm_s'], 1.)
        self.assertEqual(result['feedback_velocity_limit_rad_s'], .2)
        self.assertAlmostEqual(result['reference_peak_velocity_rad_s'], 1.875*.02/result['seconds'])
        starts, _ = board.bridge_segment_feedforward(previous, self.target,
            dict.fromkeys(NAMES, 0.), self.plant.gains, desired,
            self.baseline.hardware_limits, reference_starts=self.target)
        for name in NAMES:
            self.assertEqual(starts[name]['start_tau'], previous[name]['tau_ff'])
            self.assertEqual(starts[name]['end_tau'], 0.)
            self.assertEqual(starts[name]['ramp_fraction'], 1.)
            for fraction in (0., .25, .5, .75, 1.):
                derivative = abs(previous[name]['tau_ff']*(6*fraction-6*fraction*fraction)/result['seconds'])
                self.assertLessEqual(derivative, 1.+1e-12)
        self.assertEqual((previous, desired, timing), originals)


class MemoryTrace:
    def __init__(self, path):
        self.path = Path('offline-in-memory-only')
        self.rows, self.dropped, self.error, self.capture_errors = [], 0, None, 0

    def submit(self, row):
        self.rows.append(copy.deepcopy(row))


class SimulatedArm:
    """In-memory primitive; every control call goes to ReplyMotor only."""
    def __init__(self, plant, clock):
        self.plant, self.clock = plant, clock
        self.motors = {n: ReplyMotor(n, plant.entry[n]) for n in NAMES}
        self.ctrl = types.SimpleNamespace(controlMIT=self.control)
        self.board_gravity_limits = dict.fromkeys(NAMES, (-2., 2.))
        self.moves, self.commands, self.enables = [], [], []

    def control(self, motor, kp, kd, q, dq, tau):
        command = dict(kp=kp, kd=kd, q=q, dq=dq, tau_ff=tau)
        state = self.plant.equilibrium({motor.name: command})[motor.name]
        motor.recv_data(state['pos'], state['vel'], state['tau'])
        self.commands.append((motor.name, command))

    def read_status(self, names):
        return {n: dict(pos=self.motors[n].pos, vel=self.motors[n].vel,
                        tau=self.motors[n].tau) for n in names}

    def enable(self, names):
        self.enables.append(tuple(names))

    def move_targets_with_holds(self, target, **kwargs):
        actual = {n: self.motors[n].pos for n in NAMES}
        reference = kwargs.get('move_reference_starts', actual)
        support = kwargs['move_tau_ff']
        if 'move_tau_ff_initializer' in kwargs:
            support = kwargs['move_tau_ff_initializer'](actual)
        record = dict(target=dict(target), gains=copy.deepcopy(kwargs['move_gains']),
                      support=copy.deepcopy(support), reference=dict(reference),
                      seconds=kwargs['seconds_per_step'], first_commands={})
        self.moves.append(record)
        # Include the literal interpolation-origin command, a smooth ramp, and
        # >.4s terminal dwell. This tests execute's real capture/observation path.
        for step in range(32):
            fraction = step/31
            smooth = fraction*fraction*(3-2*fraction)
            self.clock[0] += kwargs['seconds_per_step']/31 if step else .001
            for name in NAMES:
                row = support[name]
                q = reference[name]+smooth*(target[name]-reference[name])
                ff = row['start_tau']+smooth*(row['end_tau']-row['start_tau'])
                gains = kwargs['move_gains'][name]
                self.ctrl.controlMIT(self.motors[name], gains['kp'], gains['kd'], q, 0., ff)
                if step == 0:
                    record['first_commands'][name] = self.commands[-1][1]
        for _ in range(26):
            self.clock[0] += .02
            for name in NAMES:
                gains = kwargs['move_gains'][name]
                self.ctrl.controlMIT(self.motors[name], gains['kp'], gains['kd'], target[name],
                                     0., support[name]['end_tau'])


class ExecuteStaticSupportIntegrationTests(unittest.TestCase):
    def test_execute_real_receive_capture_learning_and_next_segment_bridge(self):
        # Zero modeled gravity isolates underestimated feedforward from the
        # separately tested dynamics adapter; actual plant gravity remains nonzero.
        plant, clock = StaticPlant(modeled_slope=0.), [100.]
        arm = SimulatedArm(plant, clock)
        legacy = types.SimpleNamespace(HOME_GAINS=copy.deepcopy(plant.gains))
        target = dict.fromkeys(NAMES, .25)
        route = dict(stage='approach', start_rad=dict.fromkeys(NAMES, 0.),
                     target_board_mm=[0., 0., 50.], precomputed_before_motion=True,
                     endpoint_settle_enabled=False, waypoints=[
                         dict(stage='source_above' if i == 0 else 'contact',
                              encoder_rad=dict(target), target_board_mm=[0., 0., 120. if i == 0 else 50.])
                         for i in range(4)])
        route['planned_gravity_envelope'] = gravity_envelope(plant.model, route)
        untouched = copy.deepcopy(route)
        helpers = types.ModuleType('left_arm_v2_8_move_library')
        helpers.claw_home_while_holding_arm = lambda *a, **k: self.fail('claw must remain untouched')
        helpers.close_claw_while_holding_arm = helpers.claw_home_while_holding_arm
        cfg = {'motion_feedforward_policy': 'single-entry-gravity-anchor'}
        with patch.dict(sys.modules, {'left_arm_v2_8_move_library': helpers}), \
                patch('left_arm_motion_trace.MotionTrace', MemoryTrace), \
                patch('left_arm_board_grasp.time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(board, 'check_planned_gripper_clearance'), \
                patch.object(board, 'check_transit_edge'), \
                patch.object(board, 'board_motion_gains', side_effect=lambda *a: (copy.deepcopy(plant.gains), None)), \
                redirect_stdout(io.StringIO()):
            result = board.execute(arm, legacy, route, geometry=(plant.model, cfg))
        self.assertTrue(result['ok'])
        self.assertEqual(route, untouched)
        self.assertEqual(arm.enables, [NAMES])
        self.assertEqual(len(arm.moves), 4)
        observations = [r['diagnostics'] for r in arm._board_trace_recorder.rows
                        if r['event'] == 'quasistatic_support_observation']
        self.assertEqual(len(observations), 4)
        self.assertTrue(all(r['observation']['accepted'] and r['updated'] for r in observations))
        frames = [r for r in arm._board_trace_recorder.rows if r['event'] == 'motor_frame']
        self.assertGreater(len(frames), 100)
        self.assertTrue(all(r['individual_feedback_freshness_verified'] for r in frames))
        for frame in frames:
            for name in NAMES:
                self.assertEqual((frame['commands'][name]['kp'], frame['commands'][name]['kd']), (20., 1.))
        for previous, current in zip(arm.moves, arm.moves[1:]):
            self.assertEqual(current['target'], target)
            self.assertEqual(current['reference'], target)
            self.assertEqual(current['gains'], plant.gains)
            for name in NAMES:
                self.assertEqual(current['first_commands'][name]['q'], target[name])
                self.assertEqual(current['first_commands'][name]['tau_ff'], previous['support'][name]['end_tau'])
                self.assertGreater(current['support'][name]['end_tau'], current['support'][name]['start_tau'])
                self.assertEqual(current['support'][name]['ramp_fraction'], 1.)
                self.assertGreaterEqual(current['seconds'],
                    1.5*(current['support'][name]['end_tau']-current['support'][name]['start_tau'])-1e-12)
        first = plant.equilibrium({n: dict(plant.gains[n], q=target[n], dq=0., tau_ff=plant.entry[n]) for n in NAMES})
        old_error = max(abs(first[n]['pos']-target[n]) for n in NAMES)
        new_error = max(abs(result['final'][n]['pos']-target[n]) for n in NAMES)
        self.assertLess(new_error, old_error*.25)
        for name in NAMES:
            terminal = result['endpoint']['model_hold_motion_gains'][name]
            self.assertEqual(terminal['q'], target[name])
            self.assertEqual(terminal['tau_ff'], arm.moves[-1]['support'][name]['end_tau'])
            report = result['endpoint']['quasistatic_support']
            self.assertAlmostEqual(report['last_completed_segment_bias_tau'][name],
                                   observations[-2]['bias_tau'][name])
            self.assertAlmostEqual(report['estimated_bias_tau'][name], observations[-1]['bias_tau'][name])
            self.assertAlmostEqual(report['pending_unapplied_bias_tau'][name],
                                   observations[-1]['bias_tau'][name]-observations[-2]['bias_tau'][name])
            self.assertGreater(report['pending_unapplied_bias_tau'][name], 0.)
        self.assertEqual(arm._board_trace_recorder.capture_errors, 0)


if __name__ == '__main__':
    unittest.main()
