"""Packet freshness and bounded quasi-static support observations.

This module has no serial, controller, model, file, or network dependencies.
Only ``install_feedback_clocks`` changes objects, by instrumenting their
instance-level SDK receive callbacks. It never issues a motor command.

Merge ``feedback_clock_fields(motors, frame['monotonic_s'])`` into an existing
motor_frame before collecting observations. All three clock fields are
name-to-value mappings; the aggregate freshness flag is a boolean. Getters
are intentionally untouched, so repeatedly reading a frozen cache is stale.

``bounded_support_update`` estimates an ABSOLUTE session-local residual, not
an increment to add repeatedly. The caller owns observation deduplication,
total feedforward/hardware clipping, and the actual continuous command ramp.
No joint target, entry anchor, or persisted adaptive model is changed here.
"""
from __future__ import annotations

from collections.abc import Mapping
import functools
import math
from statistics import median
import time


MAX_FEEDBACK_AGE_S = .1
MIN_WINDOW_S = .25
MAX_LOOKBACK_S = .4
MIN_WINDOW_FRAMES = 6
MAX_FRAME_GAP_S = .1
MAX_SPEED_RAD_S = .05
MAX_POSITION_SPAN_RAD = math.radians(.12)
MAX_REFERENCE_ERROR_RAD = 1e-5
MAX_FF_SPAN_NM = .1
_EPS = 1e-9


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _number(value, label):
    if not _finite(value):
        raise ValueError(label+' must be finite')
    return float(value)


def install_feedback_clocks(motors):
    """Wrap each instance's recv_data(q,dq,tau) after a successful real update.

    The SDK dispatches its decoded reply through this method. Existing cached
    values are not stamped at installation. Duplicate installation is a no-op;
    a replaced hook or an uninstrumentable motor is explicitly reported.
    """
    if not isinstance(motors, Mapping) or not motors:
        raise ValueError('motors must be a nonempty name-to-instance mapping')
    installed, already, unavailable = [], [], {}
    for name, motor in motors.items():
        original = getattr(motor, 'recv_data', None)
        wrapper = getattr(motor, '_board_feedback_recv_wrapper', None)
        if wrapper is not None:
            if original is wrapper:
                already.append(name)
            else:
                unavailable[name] = 'previously installed receive hook has been replaced'
            continue
        if not callable(original):
            unavailable[name] = 'recv_data callback is unavailable'
            continue

        def make_wrapper(instance, callback):
            @functools.wraps(callback)
            def received(*args, **kwargs):
                result = callback(*args, **kwargs)
                now = time.monotonic()
                previous = getattr(instance, '_board_feedback_packet_state', (None, 0))
                sequence = previous[1]+1
                instance._board_feedback_monotonic_s = now
                instance._board_feedback_sequence = sequence
                # A single tuple assignment lets readers observe a consistent
                # timestamp/sequence pair even with a separate receive thread.
                instance._board_feedback_packet_state = (now, sequence)
                return result
            return received

        hook = make_wrapper(motor, original)
        try:
            motor._board_feedback_monotonic_s = None
            motor._board_feedback_sequence = 0
            motor._board_feedback_packet_state = (None, 0)
            motor.recv_data = hook
            motor._board_feedback_recv_wrapper = hook
        except (AttributeError, TypeError) as exc:
            # Do not leave an untracked receiver replacement behind if the
            # object accepts recv_data but rejects its instrumentation marker.
            try:
                if getattr(motor, 'recv_data', None) is hook:
                    motor.recv_data = original
            except (AttributeError, TypeError):
                pass
            unavailable[name] = type(exc).__name__+': receive instrumentation unavailable'
            continue
        installed.append(name)
    return {'installed': installed, 'already_installed': already,
            'unavailable': unavailable, 'complete': not unavailable,
            'clock_source': 'successful instance recv_data callback; getters are not stamped'}


def feedback_clock_fields(motors, now_s):
    """Return frame-level name->clock maps and a fail-closed freshness flag."""
    if not isinstance(motors, Mapping) or not motors:
        raise ValueError('motors must be a nonempty name-to-instance mapping')
    now_s = _number(now_s, 'now_s')
    received, sequences, ages, verified = {}, {}, {}, {}
    for name, motor in motors.items():
        hook = getattr(motor, '_board_feedback_recv_wrapper', None)
        installed = hook is not None and getattr(motor, 'recv_data', None) is hook
        stamp, sequence = getattr(motor, '_board_feedback_packet_state', (None, None))
        valid = (installed and _finite(stamp) and isinstance(sequence, int)
                 and not isinstance(sequence, bool) and sequence > 0)
        age = now_s-float(stamp) if valid else None
        received[name] = float(stamp) if _finite(stamp) else None
        sequences[name] = sequence if isinstance(sequence, int) and not isinstance(sequence, bool) else None
        ages[name] = age
        verified[name] = bool(valid and -_EPS <= age <= MAX_FEEDBACK_AGE_S+_EPS)
    return {'feedback_received_monotonic_s': received,
            'feedback_sequence': sequences, 'feedback_age_s': ages,
            'individual_feedback_freshness': verified,
            'individual_feedback_freshness_verified': all(verified.values())}


def stable_support_observation(samples, names, expected_targets, now_s):
    """Accept only a recent, continuous, independently fresh seven-axis window.

    Input rows use the existing motor_frame schema plus feedback_clock_fields.
    No trace-stage label is consulted. The caller may invoke this repeatedly;
    accepted window_end_s and sequence metadata support caller-side deduplication.
    Rejections return diagnostics instead of commanding or raising a motion stop.
    """
    def reject(reason, **diagnostics):
        return {'accepted': False, 'reason': reason, **diagnostics}

    names = tuple(names)
    if len(names) != 7 or len(set(names)) != 7:
        return reject('seven-distinct-joints-required')
    if not _finite(now_s):
        return reject('nonfinite-observation-time')
    if not isinstance(expected_targets, Mapping) or any(
            name not in expected_targets or not _finite(expected_targets[name]) for name in names):
        return reject('missing-or-nonfinite-expected-target')
    samples = list(samples)
    if not samples:
        return reject('no-feedback-samples')
    if any(not isinstance(row, Mapping) or not _finite(row.get('monotonic_s')) for row in samples):
        return reject('invalid-sample-time')
    times = [float(row['monotonic_s']) for row in samples]
    if any(b <= a for a, b in zip(times, times[1:])):
        return reject('nonincreasing-sample-times')
    age = float(now_s)-times[-1]
    if not -_EPS <= age <= MAX_FEEDBACK_AGE_S+_EPS:
        return reject('latest-feedback-frame-stale', latest_age_s=age)
    window = [row for row in samples if row['monotonic_s'] >= float(now_s)-MAX_LOOKBACK_S-_EPS]
    if len(window) < MIN_WINDOW_FRAMES:
        return reject('insufficient-feedback-frames', frames=len(window))
    duration = window[-1]['monotonic_s']-window[0]['monotonic_s']
    if duration+_EPS < MIN_WINDOW_S:
        return reject('insufficient-stable-window', duration_s=duration, frames=len(window))
    if any(b['monotonic_s']-a['monotonic_s'] > MAX_FRAME_GAP_S+_EPS
           for a, b in zip(window, window[1:])):
        return reject('feedback-frame-gap')

    positions = {n: [] for n in names}
    velocities = {n: [] for n in names}
    torques = {n: [] for n in names}
    feedforward = {n: [] for n in names}
    gain_reference, first_clocks, previous_clocks = {}, {}, {}
    for row in window:
        if row.get('event', 'motor_frame') != 'motor_frame':
            return reject('non-control-frame-in-window')
        if row.get('writer_error') or row.get('error'):
            return reject('control-writer-error')
        if any(row.get(key, 0) for key in ('queue_dropped_frames_before_submit',
                                         'queue_dropped_frames', 'dropped_frames', 'capture_errors')):
            return reject('dropped-or-invalid-control-frames')
        if row.get('invalid_feedback_fields'):
            return reject('invalid-feedback-fields')
        if row.get('individual_feedback_freshness_verified') is not True:
            return reject('individual-feedback-freshness-unverified')
        commands, feedback = row.get('commands'), row.get('feedback')
        if not isinstance(commands, Mapping) or not isinstance(feedback, Mapping):
            return reject('missing-control-or-feedback')
        clock_maps = [row.get(key) for key in ('feedback_received_monotonic_s',
                                              'feedback_sequence', 'feedback_age_s')]
        if not all(isinstance(value, Mapping) for value in clock_maps):
            return reject('missing-individual-feedback-clocks')
        stamps, sequences, recorded_ages = clock_maps
        for name in names:
            command, state = commands.get(name), feedback.get(name)
            if (not isinstance(command, Mapping) or not isinstance(state, Mapping)
                    or not all(_finite(command.get(k)) for k in ('q', 'dq', 'kp', 'kd', 'tau_ff'))
                    or not all(_finite(state.get(k)) for k in ('pos', 'vel', 'tau'))):
                return reject('missing-or-nonfinite-control-fields', joint=name)
            if (abs(command['q']-expected_targets[name]) > MAX_REFERENCE_ERROR_RAD+_EPS
                    or abs(command['dq']) > MAX_REFERENCE_ERROR_RAD+_EPS):
                return reject('moving-or-mismatched-reference', joint=name)
            gains = (command['kp'], command['kd'])
            if gains[0] < 0 or gains[1] < 0:
                return reject('invalid-gains', joint=name)
            if name in gain_reference and gains != gain_reference[name]:
                return reject('gains-changed-within-window', joint=name)
            gain_reference[name] = gains
            if abs(state['vel']) > MAX_SPEED_RAD_S+_EPS:
                return reject('joint-not-stationary', joint=name)
            stamp, sequence, reported_age = stamps.get(name), sequences.get(name), recorded_ages.get(name)
            if (not _finite(stamp) or not _finite(reported_age) or not isinstance(sequence, int)
                    or isinstance(sequence, bool) or sequence <= 0):
                return reject('invalid-individual-feedback-clock', joint=name)
            actual_age = row['monotonic_s']-stamp
            if (not -_EPS <= actual_age <= MAX_FEEDBACK_AGE_S+_EPS
                    or abs(reported_age-actual_age) > 1e-6):
                return reject('stale-or-inconsistent-individual-feedback', joint=name)
            if name in previous_clocks:
                old_stamp, old_sequence = previous_clocks[name]
                if (sequence < old_sequence or stamp < old_stamp or
                        (sequence == old_sequence and stamp != old_stamp) or
                        (sequence > old_sequence and stamp <= old_stamp)):
                    return reject('nonmonotonic-individual-feedback-clock', joint=name)
            first_clocks.setdefault(name, (stamp, sequence))
            previous_clocks[name] = (stamp, sequence)
            positions[name].append(float(state['pos']))
            velocities[name].append(float(state['vel']))
            torques[name].append(float(state['tau']))
            feedforward[name].append(float(command['tau_ff']))

    spans, medians = {}, {'positions': {}, 'velocities': {}, 'torques': {}}
    for name in names:
        first_stamp, first_sequence = first_clocks[name]
        last_stamp, last_sequence = previous_clocks[name]
        if last_sequence <= first_sequence or last_stamp <= first_stamp:
            return reject('frozen-individual-feedback', joint=name)
        if not -_EPS <= float(now_s)-last_stamp <= MAX_FEEDBACK_AGE_S+_EPS:
            return reject('latest-individual-feedback-stale', joint=name)
        pos_span = max(positions[name])-min(positions[name])
        ff_span = max(feedforward[name])-min(feedforward[name])
        tau_span = max(torques[name])-min(torques[name])
        tau_median = float(median(torques[name]))
        tau_limit = max(.15, .05*abs(tau_median))
        if pos_span > MAX_POSITION_SPAN_RAD+_EPS:
            return reject('position-drift-exceeds-window-limit', joint=name,
                          position_span_deg=math.degrees(pos_span))
        if ff_span > MAX_FF_SPAN_NM+_EPS:
            return reject('feedforward-changing-within-window', joint=name, ff_span_Nm=ff_span)
        if tau_span > tau_limit+_EPS:
            return reject('load-not-stable', joint=name, torque_span_Nm=tau_span,
                          torque_span_limit_Nm=tau_limit)
        medians['positions'][name] = float(median(positions[name]))
        medians['velocities'][name] = float(median(velocities[name]))
        medians['torques'][name] = tau_median
        spans[name] = {'position_span_deg': math.degrees(pos_span), 'ff_span_Nm': ff_span,
                       'torque_span_Nm': tau_span, 'torque_span_limit_Nm': tau_limit}
    return {'accepted': True, 'reason': 'stable-fresh-seven-axis-support-window',
            **medians, 'window_start_s': float(window[0]['monotonic_s']),
            'window_end_s': float(window[-1]['monotonic_s']),
            'duration_s': float(duration), 'frames': len(window), 'spans': spans,
            'feedback_received_monotonic_s': {n: previous_clocks[n][0] for n in names},
            'feedback_sequence': {n: previous_clocks[n][1] for n in names},
            'feedback_age_s': {n: float(now_s)-previous_clocks[n][0] for n in names},
            'feedback_sequence_start': {n: first_clocks[n][1] for n in names},
            'individual_feedback_freshness_verified': True,
            'deduplication_required_by_caller': True}


def bounded_support_update(current_bias, observed_residual, gains, hardware_limits, alpha=.5):
    """Filter toward an absolute residual observation with explicit anti-windup.

    Observations are not additive increments. Repeated identical observations
    approach the same bounded estimate; they never accumulate without bound.
    The caller must not reuse one observation/window as fresh evidence.
    """
    alpha = _number(alpha, 'alpha')
    if not 0 <= alpha <= 1:
        raise ValueError('alpha must be in [0,1]')
    if (not isinstance(current_bias, Mapping) or not current_bias
            or not isinstance(observed_residual, Mapping)
            or set(current_bias) != set(observed_residual)):
        raise ValueError('current and observed residual must have the same nonempty joint keys')
    result, audit = {}, {}
    for name, previous in current_bias.items():
        previous = _number(previous, 'current bias for '+name)
        observed = _number(observed_residual[name], 'observed residual for '+name)
        if name not in gains or name not in hardware_limits:
            raise ValueError('missing gains or hardware limit for '+name)
        kp = _number(gains[name]['kp'], 'Kp for '+name)
        maximum = _number(hardware_limits[name], 'hardware limit for '+name)
        if kp < 0 or maximum <= 0:
            raise ValueError('nonnegative Kp and positive hardware limit required for '+name)
        cap = min(.25*maximum, max(.1, kp*math.radians(5.)))
        clipped_previous = min(cap, max(-cap, previous))
        desired = min(cap, max(-cap, observed))
        raw = clipped_previous+alpha*(desired-clipped_previous)
        applied = min(cap, max(-cap, raw))
        result[name] = applied
        audit[name] = {'previous_bias_Nm': previous, 'observed_residual_Nm': observed,
                       'bounded_observation_Nm': desired, 'bias_limit_Nm': cap,
                       'bias_Nm': applied, 'change_Nm': applied-previous,
                       'alpha': alpha, 'saturated': desired != observed or clipped_previous != previous or applied != raw,
                       'entry_anchor_modified': False, 'joint_target_modified': False,
                       'observation_semantics': 'absolute session residual, not an additive increment'}
    return {'bias': result, 'audit': audit}


def slew_duration(start_ff, end_ff, minseconds, rate=1.):
    """Duration for name->Nm endpoints with a cubic smoothstep slew bound.

    For u(s)=3*s**2-2*s**3, max du/ds=1.5. This returns a duration
    guaranteeing max |d tau_ff/dt| <= rate for every supplied joint. The
    caller must actually use this smoothstep and include any simultaneous
    changing feedforward component when checking its total slew.
    """
    minimum = _number(minseconds, 'minseconds')
    rate = _number(rate, 'rate')
    if minimum < 0 or rate <= 0:
        raise ValueError('minseconds must be nonnegative and rate positive')
    if (not isinstance(start_ff, Mapping) or not start_ff or not isinstance(end_ff, Mapping)
            or set(start_ff) != set(end_ff)):
        raise ValueError('feedforward endpoints must have matching nonempty joint keys')
    delta = max(abs(_number(end_ff[n], 'end feedforward for '+n)
                    -_number(start_ff[n], 'start feedforward for '+n)) for n in start_ff)
    return max(minimum, 1.5*delta/rate)
