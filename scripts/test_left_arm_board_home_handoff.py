import io
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import left_arm_board_home_handoff as home
from left_arm_fault_support import FaultSupport
import chessboard_vision_v3_0_web_control as web


class Arm:
    opened = 0
    def __init__(self):
        type(self).opened += 1
        self.closed = 0
        self.board_gravity_feedforward = True
    def close(self):self.closed += 1


class HomeHandoffTests(unittest.TestCase):
    def test_home_handoff_monitor_is_record_only(self):
        arm=mock.Mock();arm.allow_home_handoff=False
        support=types.SimpleNamespace(names=['elbow'])
        home.install_home_handoff(support,arm,types.SimpleNamespace())
        self.assertTrue(support.home_guard.record_only)

    def test_real_v28_wrapper_reuses_serial_across_both_legacy_passes(self):
        from test_left_arm_v31_motion_dynamics import load_legacy_without_hardware
        import left_arm_v2_8 as v28
        legacy = load_legacy_without_hardware()
        cls = legacy.LeftArmV2
        arm = object.__new__(cls)
        arm.board_gravity_feedforward = True
        pose = dict.fromkeys(legacy.DEFAULT_JOINTS, 0.)
        arm.read_status = lambda names: {n:dict(pos=pose[n],vel=0.,tau=1.) for n in names}
        arm.close = mock.Mock()
        calls = []
        def legacy_main():
            current = cls()
            self.assertIs(current, arm)
            calls.append(list(sys.argv))
            self.assertEqual(current.positions(list(pose)), pose)
            current.close()
        with (mock.patch.dict(sys.modules, {'left_arm_v2_6':legacy}),
              mock.patch.object(legacy,'main',legacy_main),
              mock.patch.object(legacy,'load_home',lambda path:pose),
              mock.patch.object(legacy,'load_pose',lambda path:pose),
              redirect_stdout(io.StringIO())):
            result = home.run_home_on_existing_arm(arm,legacy,v28.main)
        self.assertTrue(result['ok'])
        self.assertEqual(len(calls),2)
        self.assertIn('--prehome-clearance',calls[0])
        self.assertNotIn('--prehome-clearance',calls[1])
        for command in calls:
            self.assertEqual(float(command[command.index('--max-delta-deg')+1]),180.)
        arm.close.assert_not_called()

    def test_web_home_explicitly_uses_180_degree_staged_entry(self):
        with tempfile.TemporaryDirectory() as directory:
            state=web.OperationalRunState(str(Path(directory)/'log'),web.build_parser().parse_args([]),auto_hold_enabled=False)
            for action,expected in (('home','180'),('table-clearance','120')):
                command=['python3','left_arm_v2_8.py','home' if action=='home' else 'clearance',
                         '--max-delta-deg','120','--prehome-clearance','--execute']
                with mock.patch.object(web.v28.FullRunState,'start',return_value={'ok':True,'running':False}) as start:
                    state.start(action,command,{})
                    sent=start.call_args.args[1]
                self.assertEqual(sent[sent.index('--max-delta-deg')+1],expected)
                self.assertEqual(command[command.index('--max-delta-deg')+1],'120')

    def test_successful_home_exits_support_only_after_explicit_request(self):
        arm=mock.Mock();arm.motors={'elbow':object()}
        status={'elbow':dict(pos=0.,vel=0.,tau=3.)}
        arm.read_status.return_value=status
        support=FaultSupport(arm,['elbow'],{'elbow':dict(kp=70.,kd=3.)},emit=lambda x:None)
        support.activate(status)
        support.home_requested=True
        expected=dict(ok=True,recovered_action='home')
        support.home_callback=lambda:expected
        with redirect_stdout(io.StringIO()):
            self.assertEqual(support.run(RuntimeError('stop')),expected)
        arm.disable.assert_not_called();arm.close.assert_not_called()

    def test_multiple_home_passes_reuse_one_connection_and_restore_factory(self):
        arm=Arm();opens=Arm.opened
        legacy=types.SimpleNamespace(LeftArmV2=Arm,HOME_JOINT_DEADBANDS_DEG={'wrist':1.})
        def main():
            for _ in range(3):
                active=legacy.LeftArmV2()
                self.assertIs(active,arm)
                self.assertFalse(active.board_gravity_feedforward)
                active.close()
                self.assertEqual(arm.closed,0)
            legacy.HOME_JOINT_DEADBANDS_DEG={}
        self.assertTrue(home.run_home_on_existing_arm(arm,legacy,main)['same_serial_session'])
        self.assertEqual(Arm.opened,opens)
        self.assertTrue(arm.board_gravity_feedforward)
        self.assertEqual(legacy.HOME_JOINT_DEADBANDS_DEG,{'wrist':1.})
        arm.close();self.assertEqual(arm.closed,1)
        self.assertIsNot(Arm(),arm)

    def test_home_exception_rearms_at_latest_pose_and_does_not_close(self):
        arm=mock.Mock();arm.motors={'elbow':object()}
        status={'elbow':{'pos':0.,'vel':0.,'tau':3.}}
        arm.read_status.side_effect=lambda names:{n:dict(status[n]) for n in names}
        support=FaultSupport(arm,['elbow'],{'elbow':{'kp':70.,'kd':3.}},emit=lambda x:None)
        support.activate(status)
        with redirect_stdout(io.StringIO()):support.latch(RuntimeError('first stop'))
        def fail():
            status['elbow']['pos']=.4;status['elbow']['tau']=4.
            raise RuntimeError('home failed')
        support.home_callback=fail;support.home_requested=True
        with redirect_stdout(io.StringIO()):self.assertIsNone(home.process_home_request(support))
        self.assertTrue(support.latched)
        self.assertEqual(support.targets['elbow'],.4)
        self.assertEqual(support.support_tau['elbow'],4.)
        arm.close.assert_not_called();arm.disable.assert_not_called()

    def test_web_home_requests_signal_only_when_new_support_advertises_it(self):
        import json
        with tempfile.TemporaryDirectory() as directory:
            state=web.OperationalRunState(str(Path(directory)/'log'),web.build_parser().parse_args([]))
            state.proc=types.SimpleNamespace(pid=123,poll=lambda:None)
            state.current={'action':'dynamics:board-grasp','running':True,
                           'stdout':'board protective hold= '+json.dumps({'home_handoff_available':True})+'\n'}
            with mock.patch.object(web.signal,'SIGUSR1',10,create=True),mock.patch.object(web.os,'kill') as kill:
                before=state.snapshot();self.assertFalse(before['running']);self.assertTrue(before['support_running'])
                result=state.start('home',['python3','left_arm_v2_8.py','home','--execute'],{})
                self.assertTrue(result['ok']);self.assertTrue(result['same_serial_session'])
                kill.assert_called_once_with(123,10)
            state.current['stdout']='board protective hold= '+json.dumps({'home_handoff_available':False})+'\n'
            with mock.patch.object(web.os,'kill') as kill:
                self.assertFalse(state.start('home',[],{})['ok']);kill.assert_not_called()


if __name__=='__main__':unittest.main()
