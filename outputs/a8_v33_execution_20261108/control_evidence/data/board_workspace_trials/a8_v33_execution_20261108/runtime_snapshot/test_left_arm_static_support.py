"""Offline unit tests: fake receive callbacks only, no serial/SDK imports."""
import copy
import math
import unittest
from unittest.mock import patch

from left_arm_static_support import (bounded_support_update, feedback_clock_fields,
    install_feedback_clocks, slew_duration, stable_support_observation)


NAMES = ('shoulder_front', 'shoulder_side', 'shoulder_rotate', 'elbow',
         'arm_roll', 'wrist_side', 'wrist')
TARGETS = dict.fromkeys(NAMES, .3)


class FakeMotor:
    def __init__(self):
        self.pos = self.vel = self.tau = 0.
        self.calls = 0

    def recv_data(self, q, dq, tau):
        self.pos, self.vel, self.tau = q, dq, tau
        self.calls += 1
        return 'received'

    def getPosition(self):
        return self.pos

    def getVelocity(self):
        return self.vel

    def getTorque(self):
        return self.tau


def frames():
    result = []
    for i in range(17):
        now = 10.+i*.02
        row = {'event': 'motor_frame', 'monotonic_s': now,
               'commands': {n: {'q': .3, 'dq': 0., 'kp': 36., 'kd': 4.5,
                                'tau_ff': .2} for n in NAMES},
               'feedback': {n: {'pos': .301, 'vel': .007, 'tau': .25} for n in NAMES},
               'writer_error': None, 'queue_dropped_frames_before_submit': 0,
               'invalid_feedback_fields': [],
               'individual_feedback_freshness_verified': True,
               'feedback_received_monotonic_s': dict.fromkeys(NAMES, now-.002),
               'feedback_sequence': dict.fromkeys(NAMES, i+1),
               'feedback_age_s': dict.fromkeys(NAMES, .002)}
        result.append(row)
    return result


class PacketClockTests(unittest.TestCase):
    def test_actual_packet_only_and_getters_do_not_refresh(self):
        motor = FakeMotor()
        self.assertTrue(install_feedback_clocks({'joint': motor})['complete'])
        self.assertFalse(feedback_clock_fields({'joint': motor}, 1.)['individual_feedback_freshness_verified'])
        with patch('left_arm_static_support.time.monotonic', return_value=1.):
            self.assertEqual(motor.recv_data(.3, .01, .2), 'received')
        self.assertTrue(feedback_clock_fields({'joint': motor}, 1.05)['individual_feedback_freshness_verified'])
        with patch('left_arm_static_support.time.monotonic', return_value=9.):
            self.assertEqual((motor.getPosition(), motor.getVelocity(), motor.getTorque()), (.3, .01, .2))
        self.assertEqual(motor._board_feedback_monotonic_s, 1.)
        self.assertEqual(motor._board_feedback_sequence, 1)
        self.assertFalse(feedback_clock_fields({'joint': motor}, 1.101)['individual_feedback_freshness_verified'])

    def test_repeated_installation_not_double_wrapped(self):
        motor = FakeMotor()
        install_feedback_clocks({'joint': motor})
        hook = motor.recv_data
        report = install_feedback_clocks({'joint': motor})
        self.assertEqual(report['already_installed'], ['joint'])
        self.assertIs(motor.recv_data, hook)
        with patch('left_arm_static_support.time.monotonic', side_effect=[1., 2.]):
            motor.recv_data(.1, 0., 0.)
            motor.recv_data(.2, 0., 0.)
        self.assertEqual(motor.calls, 2)
        self.assertEqual(motor._board_feedback_sequence, 2)
        untouched = FakeMotor()
        self.assertFalse(hasattr(untouched, '_board_feedback_sequence'))

    def test_failed_receive_not_marked_fresh(self):
        motor = FakeMotor()
        def fail(*args):
            raise ValueError('failed receive')
        motor.recv_data = fail
        install_feedback_clocks({'joint': motor})
        with self.assertRaises(ValueError):
            motor.recv_data(0., 0., 0.)
        self.assertEqual(motor._board_feedback_sequence, 0)
        self.assertFalse(feedback_clock_fields({'joint': motor}, 1.)['individual_feedback_freshness_verified'])

    def test_missing_clock_and_future_clock_fail_closed(self):
        motor = FakeMotor()
        self.assertFalse(feedback_clock_fields({'joint': motor}, 1.)['individual_feedback_freshness_verified'])
        install_feedback_clocks({'joint': motor})
        with patch('left_arm_static_support.time.monotonic', return_value=2.):
            motor.recv_data(0., 0., 0.)
        self.assertFalse(feedback_clock_fields({'joint': motor}, 1.)['individual_feedback_freshness_verified'])

    def test_missing_or_replaced_receiver_reported(self):
        self.assertFalse(install_feedback_clocks({'joint': object()})['complete'])
        motor = FakeMotor()
        install_feedback_clocks({'joint': motor})
        motor.recv_data = lambda *args: None
        self.assertFalse(install_feedback_clocks({'joint': motor})['complete'])
        self.assertFalse(feedback_clock_fields({'joint': motor}, 1.)['individual_feedback_freshness_verified'])


class ObservationTests(unittest.TestCase):
    def observe(self, rows=None, now=10.33, targets=None, names=NAMES):
        return stable_support_observation(frames() if rows is None else rows,
                                           names, TARGETS if targets is None else targets, now)

    def test_stable_fresh_observation_and_inputs_unchanged(self):
        rows = frames()
        original = copy.deepcopy(rows)
        result = self.observe(rows)
        self.assertTrue(result['accepted'], result)
        self.assertEqual(result['positions'], dict.fromkeys(NAMES, .301))
        self.assertEqual(result['velocities'], dict.fromkeys(NAMES, .007))
        self.assertEqual(result['torques'], dict.fromkeys(NAMES, .25))
        self.assertAlmostEqual(result['window_end_s'], 10.32)
        self.assertEqual(rows, original)

    def test_stage_label_does_not_affect_acceptance(self):
        rows = frames()
        for i, row in enumerate(rows):
            row['context'] = {'stage': 'anything-'+str(i)}
        self.assertTrue(self.observe(rows)['accepted'])

    def test_frozen_cached_packets_rejected_despite_new_frames(self):
        rows = frames()
        for row in rows:
            row['feedback_sequence'] = dict.fromkeys(NAMES, 1)
            row['feedback_received_monotonic_s'] = dict.fromkeys(NAMES, 9.998)
            row['feedback_age_s'] = dict.fromkeys(NAMES, row['monotonic_s']-9.998)
        self.assertFalse(self.observe(rows)['accepted'])

    def test_fabricated_getter_clock_without_new_sequence_rejected(self):
        rows = frames()
        for row in rows:
            row['feedback_sequence'] = dict.fromkeys(NAMES, 1)
        result = self.observe(rows)
        self.assertFalse(result['accepted'])
        self.assertIn('clock', result['reason'])

    def test_expired_latest_frame_rejected(self):
        self.assertFalse(self.observe(now=10.43)['accepted'])

    def test_individual_stale_axis_rejected(self):
        rows = frames()
        for row in rows:
            row['feedback_received_monotonic_s'][NAMES[0]] -= .11
            row['feedback_age_s'][NAMES[0]] += .11
        self.assertFalse(self.observe(rows)['accepted'])

    def test_latest_packet_age_is_checked_against_now(self):
        rows = frames()
        for row in rows:
            row['feedback_received_monotonic_s'] = dict.fromkeys(NAMES, row['monotonic_s']-.09)
            row['feedback_age_s'] = dict.fromkeys(NAMES, .09)
        self.assertFalse(self.observe(rows, now=10.35)['accepted'])

    def test_moving_reference_or_dq_rejected(self):
        for field, value in (('q', .301), ('dq', .001)):
            with self.subTest(field=field):
                rows = frames()
                rows[8]['commands'][NAMES[0]][field] = value
                self.assertEqual(self.observe(rows)['reason'], 'moving-or-mismatched-reference')

    def test_high_velocity_rejected(self):
        rows = frames()
        rows[8]['feedback'][NAMES[0]]['vel'] = -.051
        self.assertEqual(self.observe(rows)['reason'], 'joint-not-stationary')

    def test_drift_rejected_even_with_zero_velocity_feedback(self):
        rows = frames()
        for i, row in enumerate(rows):
            row['feedback'][NAMES[0]]['pos'] += i*math.radians(.2)/16
            row['feedback'][NAMES[0]]['vel'] = 0.
        self.assertEqual(self.observe(rows)['reason'], 'position-drift-exceeds-window-limit')

    def test_gain_and_ff_changes_rejected(self):
        rows = frames()
        rows[8]['commands'][NAMES[0]]['kp'] += .1
        self.assertEqual(self.observe(rows)['reason'], 'gains-changed-within-window')
        rows = frames()
        rows[8]['commands'][NAMES[0]]['tau_ff'] += .101
        self.assertEqual(self.observe(rows)['reason'], 'feedforward-changing-within-window')

    def test_torque_span_absolute_and_relative_limits(self):
        rows = frames()
        rows[8]['feedback'][NAMES[0]]['tau'] += .151
        self.assertEqual(self.observe(rows)['reason'], 'load-not-stable')
        rows = frames()
        for row in rows:
            row['feedback'][NAMES[0]]['tau'] = -10.
        rows[8]['feedback'][NAMES[0]]['tau'] = -10.49
        self.assertTrue(self.observe(rows)['accepted'])
        rows[8]['feedback'][NAMES[0]]['tau'] = -10.51
        self.assertFalse(self.observe(rows)['accepted'])

    def test_drop_writer_invalid_and_unverified_rejected(self):
        for key, value in (('queue_dropped_frames_before_submit', 1),
                           ('writer_error', 'disk error'), ('invalid_feedback_fields', ['joint.pos']),
                           ('individual_feedback_freshness_verified', False)):
            with self.subTest(key=key):
                rows = frames()
                rows[8][key] = value
                self.assertFalse(self.observe(rows)['accepted'])

    def test_missing_joint_nonfinite_and_missing_clocks_rejected(self):
        for mutation in ('joint', 'nonfinite', 'clock'):
            rows = frames()
            if mutation == 'joint':
                rows[8]['feedback'].pop(NAMES[0])
            elif mutation == 'nonfinite':
                rows[8]['feedback'][NAMES[0]]['tau'] = float('nan')
            else:
                rows[8].pop('feedback_sequence')
            self.assertFalse(self.observe(rows)['accepted'])

    def test_minimum_frames_duration_and_continuity(self):
        self.assertFalse(self.observe(frames()[-5:])['accepted'])
        self.assertFalse(self.observe(frames()[-10:])['accepted'])
        rows = frames()
        del rows[4:11]
        self.assertEqual(self.observe(rows)['reason'], 'feedback-frame-gap')
        rows = frames()
        rows[8], rows[9] = rows[9], rows[8]
        self.assertEqual(self.observe(rows)['reason'], 'nonincreasing-sample-times')
        self.assertFalse(self.observe(names=NAMES[:-1])['accepted'])

    def test_repeated_cache_frame_allowed_when_packets_progress_over_window(self):
        rows = frames()
        for i in range(1, len(rows), 2):
            rows[i]['feedback_received_monotonic_s'] = dict(rows[i-1]['feedback_received_monotonic_s'])
            rows[i]['feedback_sequence'] = dict(rows[i-1]['feedback_sequence'])
            rows[i]['feedback_age_s'] = dict.fromkeys(NAMES, .022)
        self.assertTrue(self.observe(rows)['accepted'])


class BoundedUpdateTests(unittest.TestCase):
    def test_absolute_residual_not_added_repeatedly(self):
        gains = {'joint': {'kp': 100., 'kd': 4.5}}
        limits = {'joint': 28.}
        current = {'joint': 0.}
        observed = {'joint': -4.}
        first = bounded_support_update(current, observed, gains, limits)
        self.assertEqual(first['bias']['joint'], -2.)
        bias = first['bias']
        for _ in range(100):
            bias = bounded_support_update(bias, observed, gains, limits)['bias']
        self.assertAlmostEqual(bias['joint'], -4.)
        self.assertEqual(current, {'joint': 0.})
        self.assertEqual(observed, {'joint': -4.})

    def test_caps_and_anti_windup(self):
        gains = {'a': {'kp': 0.}, 'b': {'kp': 36.}, 'c': {'kp': 90.}}
        limits = {'a': 10., 'b': 10., 'c': 28.}
        updated = bounded_support_update(dict.fromkeys(gains, 0.),
            {'a': 100., 'b': -100., 'c': 100.}, gains, limits, alpha=1.)
        self.assertEqual(updated['bias'], {'a': .1, 'b': -2.5, 'c': 7.})
        self.assertTrue(all(v['saturated'] for v in updated['audit'].values()))
        self.assertFalse(any(v['entry_anchor_modified'] or v['joint_target_modified']
                             for v in updated['audit'].values()))

    def test_bad_inputs_rejected(self):
        arguments = ({'joint': 0.}, {'joint': .2}, {'joint': {'kp': 36.}}, {'joint': 10.})
        for alpha in (float('nan'), -1., 2.):
            with self.assertRaises(ValueError):
                bounded_support_update(*arguments, alpha=alpha)
        for replacement, index in (({'joint': float('inf')}, 1), ({'joint': -1.}, 3)):
            changed = list(arguments)
            changed[index] = replacement
            with self.assertRaises(ValueError):
                bounded_support_update(*changed)


class SlewTests(unittest.TestCase):
    def test_smoothstep_derivative_at_most_one_Nm_per_second(self):
        start, end = {'a': -2., 'b': 1.}, {'a': 2., 'b': .5}
        duration = slew_duration(start, end, .5)
        self.assertEqual(duration, 6.)
        for i in range(1001):
            fraction = i/1000.
            for name in start:
                derivative = abs((end[name]-start[name])*(6*fraction-6*fraction*fraction)/duration)
                self.assertLessEqual(derivative, 1.+1e-12)

    def test_minimum_and_rate_and_bad_inputs(self):
        self.assertEqual(slew_duration({'j': 0.}, {'j': 0.}, 2.), 2.)
        self.assertEqual(slew_duration({'j': 0.}, {'j': 4.}, 1., rate=2.), 3.)
        for kwargs in ({'minseconds': -1.}, {'minseconds': 1., 'rate': 0.}):
            with self.assertRaises(ValueError):
                slew_duration({'j': 0.}, {'j': 1.}, **kwargs)


if __name__ == '__main__':
    unittest.main()
