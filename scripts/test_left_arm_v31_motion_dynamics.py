import copy
import importlib.util
import io
import json
import math
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import left_arm_v3_1_motion_dynamics as motion
from left_arm_v3_1_model_control import JOINTS, empty_model


def fixture():
    model = empty_model()
    model['gain_units_verified'] = True
    pose = dict.fromkeys(JOINTS, 0.0)
    for joint in JOINTS:
        data = model['joints'][joint]
        data['tau_limits'] = [-2.0, 2.0]
        for coordinate, torque, inertia in ((0.0, 0.0, 0.1), (0.1, 0.4, 0.15), (0.2, 0.8, 0.2)):
            sample_pose = dict(pose, elbow=coordinate)
            data['gravity_samples'].append({'pose_rad': sample_pose, 'tau': torque})
            data['inertia_samples'].append({'pose_rad': sample_pose, 'effective_inertia': inertia})
    return model, pose


class MotionDynamicsTests(unittest.TestCase):
    def test_offline_preview_is_read_only_and_only_commands_requested_joints(self):
        model, pose = fixture()
        plan = {'start_pose_rad': pose, 'targets_rad': {'elbow': 0.2},
                'commands': {'elbow': {'kp': 40, 'kd': 1.5, 'tau': 4.2}}, 'seconds': 2}
        before_model, before_plan = copy.deepcopy(model), copy.deepcopy(plan)
        with redirect_stdout(io.StringIO()):
            report = motion.preview_reference_plan(model, plan)
        self.assertFalse(report['motion_issued'])
        self.assertEqual(report['commanded_joints'], ['elbow'])
        self.assertTrue(report['terminal_correction_zero'])
        self.assertGreater(report['peak_correction']['elbow']['tau'], 0.1)
        self.assertEqual(report['samples'][-1]['commands']['elbow'], {'kp': 40, 'kd': 1.5, 'tau': 4.2})
        self.assertEqual((model, plan), (before_model, before_plan))

    def test_entry_and_terminal_preserve_learned_baseline_with_slew(self):
        model, pose = fixture()
        original = copy.deepcopy(model)
        with redirect_stdout(io.StringIO()):
            session = motion.MotionSession(model, pose, ['elbow'])
            session.prepare(pose, 0, 0)
            self.assertEqual(session.transform('elbow', 40, 1.5, 4.2), (40, 1.5, 4.2))
            previous = (40, 1.5, 4.2)
            peak = 0
            for frame in range(1, 251):
                progress = min(frame / 200, 1)
                session.prepare(dict(pose, elbow=0.2 * progress), progress, frame * 0.01)
                applied = session.transform('elbow', 40, 1.5, 4.2)
                for old, new, rate in zip(previous, applied, (20, 5, 1.5)):
                    self.assertLessEqual(abs(new - old), rate * 0.01 + 1e-9)
                peak = max(peak, abs(applied[2] - 4.2))
                previous = applied
            self.assertGreater(peak, 0.1)
            self.assertEqual(previous, (40, 1.5, 4.2))
            session.finish()
        self.assertEqual(model, original)

    def test_coverage_loss_slews_to_legacy_without_relaxing_limits(self):
        model, pose = fixture()
        with redirect_stdout(io.StringIO()):
            session = motion.MotionSession(model, pose, ['elbow'])
            for frame in range(101):
                target = 0.1 if frame < 50 else 2.0
                session.prepare(dict(pose, elbow=target), 0.5, frame * 0.01)
                output = session.transform('elbow', 40, 1.5, 4.2)
            self.assertEqual(output, (40, 1.5, 4.2))
            self.assertGreater(session.fallback['elbow'], 0)
        self.assertEqual(model['settings']['max_gravity_distance_deg'], 25)

    def test_missing_entry_coverage_cannot_start_unanchored_compensation(self):
        model, pose = fixture()
        with redirect_stdout(io.StringIO()):
            session = motion.MotionSession(model, dict(pose, elbow=2.0), ['elbow'])
            session.prepare(pose, 0.5, 0)
            self.assertEqual(session.transform('elbow', 40, 1.5, 4.2), (40, 1.5, 4.2))

    def test_minimum_gain_fallback_does_not_invent_inertia_ratio(self):
        model, pose = fixture()
        joint = model['joints']['shoulder_rotate']
        joint['inertia_samples'] = []
        joint['scheduler']['inertia_fallback'] = 'minimum-gains'
        with redirect_stdout(io.StringIO()):
            session = motion.MotionSession(model, pose, ['shoulder_rotate'])
            for frame in range(80):
                session.prepare(dict(pose, elbow=0.1), 0.5, frame * 0.01)
                kp, kd, tau = session.transform('shoulder_rotate', 24, 4.5, -2.43)
            self.assertEqual((kp, kd), (24, 4.5))
            self.assertGreater(tau, -2.43)
            self.assertLess(tau, -1)

    def test_invalid_model_fails_before_hardware_and_no_env_is_noop(self):
        model, _ = fixture()
        with mock.patch.dict('os.environ', {}, clear=True):
            self.assertIsNone(motion.configured_model())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.json'
            model['gain_units_verified'] = False
            path.write_text(json.dumps(model), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'gain_units_verified'):
                motion.load_motion_model(path)
            model['gain_units_verified'] = True
            model['settings']['tau_slew_per_second'] = float('nan')
            path.write_text(json.dumps(model), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'non-finite'):
                motion.load_motion_model(path)

    def test_low_slew_settings_still_release_within_existing_terminal_hold(self):
        model, pose = fixture()
        model['settings'].update(kp_slew_per_second=0.1, kd_slew_per_second=0.1,
                                 tau_slew_per_second=0.1)
        with redirect_stdout(io.StringIO()):
            session = motion.MotionSession(model, pose, ['elbow'])
            for frame in range(151):
                session.prepare(dict(pose, elbow=0.1), 0.5 if frame < 100 else 1,
                                frame * 0.01)
                applied = session.transform('elbow', 40, 1.5, 4.2)
            session.finish()
        self.assertEqual(applied, (40, 1.5, 4.2))


def load_legacy_without_hardware():
    name = '_v26_motion_integration_test'
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name('left_arm_v2_6.py'))
    module = importlib.util.module_from_spec(spec)
    can = types.ModuleType('DM_CAN')
    can.DM_Motor_Type = types.SimpleNamespace(DM4340=1, DM4310=2)
    can.Motor = object
    can.MotorControl = object
    with mock.patch.dict(sys.modules, {name: module, 'serial': types.ModuleType('serial'), 'DM_CAN': can}):
        spec.loader.exec_module(module)
    return module


class TrajectoryIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.legacy = load_legacy_without_hardware()

    def run_trajectory(self, kind, enabled):
        model, pose = fixture()
        arm = self.legacy.LeftArmV2.__new__(self.legacy.LeftArmV2)
        arm.motion_dynamics_model = model if enabled else None
        arm.specs = dict.fromkeys(list(JOINTS) + ['claw'])
        arm.motors = {name: name for name in arm.specs}
        arm.positions = lambda names: {name: pose.get(name, 1.0) for name in names}
        output = []
        arm.ctrl = types.SimpleNamespace(controlMIT=lambda *args: output.append(args))
        clock = types.SimpleNamespace(now=0.0)

        def sleep(seconds):
            clock.now += seconds
        with (mock.patch.object(self.legacy.time, 'time', lambda: clock.now),
              mock.patch.object(self.legacy.time, 'monotonic', lambda: clock.now),
              mock.patch.object(self.legacy.time, 'sleep', sleep),
              redirect_stdout(io.StringIO())):
            if kind == 'single':
                arm.move_target_with_holds('elbow', 0.2, 2, 40, 1.5,
                    {'wrist': 0.0, 'claw': 1.0}, {}, 3, 0.3,
                    hold_tau={'wrist': 0.2}, active_tau=4.2, control_dt=0.01)
            elif kind == 'group':
                arm.move_targets_with_holds({'elbow': 0.2}, 2,
                    {'elbow': {'kp': 40, 'kd': 1.5}}, {'wrist': 0.0, 'claw': 1.0},
                    {}, 3, 0.3, hold_tau={'wrist': 0.2},
                    move_tau_ff={'elbow': {'start_tau': 4.2, 'end_tau': 3.8,
                                          'ramp': 'smoothstep'}},
                    control_dt=0.01, velocity_ff_joints={'elbow'})
            elif kind == 'auto':
                arm.move_targets_auto_gains({'elbow': 0.2}, 2,
                                           {'elbow': {'kp': 40, 'kd': 1.5}}, 3, 0.3)
            else:
                arm.move_targets({'elbow': 0.2}, 2, 40, 1.5)
        return output

    def test_real_trajectory_loops_preserve_targets_velocity_claw_and_terminal_law(self):
        for kind in ('single', 'group', 'auto', 'basic'):
            with self.subTest(kind=kind):
                legacy = self.run_trajectory(kind, False)
                modeled = self.run_trajectory(kind, True)
                self.assertEqual(len(legacy), len(modeled))
                changed = False
                for old, new in zip(legacy, modeled):
                    self.assertEqual((old[0], old[3], old[4]), (new[0], new[3], new[4]))
                    self.assertNotEqual(new[0], 'shoulder_rotate')
                    if old[0] == 'claw':
                        self.assertEqual(old, new)
                    changed |= old != new
                self.assertTrue(changed)
                # Last controller law is exactly the original, so subsequent
                # terminal validation, adaptive updates and Model Hold see it.
                self.assertEqual(legacy[-10:], modeled[-10:])


if __name__ == '__main__':
    unittest.main()
