import io
import json
import tempfile
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import numpy as np

import left_arm_dynamics_worker as backend
import left_arm_general_motion as general
from left_arm_rigid_body import RigidBodyModel

MODEL=Path(__file__).resolve().parents[1]/'outputs/left_arm_general_model/left_arm_general_dynamics.json'


def _delayed_worker_target(*args):
    original=backend.PreparedDynamics.inverse_dynamics
    def delayed(self,*a,**kw):
        time.sleep(.15)
        return original(self,*a,**kw)
    with mock.patch.object(backend.PreparedDynamics,'inverse_dynamics',delayed):
        backend._run(*args)


class WorkerTests(unittest.TestCase):
    def test_prepared_kernel_matches_general_implementation_across_states(self):
        model=RigidBodyModel.load(MODEL)
        prepared=backend.PreparedDynamics(model)
        rng=np.random.default_rng(120)
        for _ in range(60):
            q=rng.uniform(-1.2,1.2,7);dq=rng.uniform(-.7,.7,7);ddq=rng.uniform(-2,2,7)
            np.testing.assert_allclose(prepared.inverse_dynamics(q,dq,ddq),model.inverse_dynamics(q,dq,ddq),atol=3e-12)
            np.testing.assert_allclose(prepared.friction(dq),model.friction(dq),atol=1e-14)
        # No physical tensor validation/eigendecomposition or generic FK is
        # allowed in the warmed worker's per-reference path.
        with (mock.patch.object(model,'_parameters',side_effect=AssertionError('revalidated physical tensors')),
              mock.patch.object(model,'fk',side_effect=AssertionError('used generic FK'))):
            self.assertTrue(np.isfinite(prepared.inverse_dynamics(q,dq,ddq)).all())

    def test_real_process_matches_eager_dynamics_and_trylocks_do_not_wait(self):
        model=RigidBodyModel.load(MODEL)
        worker=backend.DynamicsWorker(model.data)
        try:
            session=worker.new_session()
            q=np.linspace(-.2,.1,7);dq=np.linspace(-.05,.1,7);ddq=np.linspace(.2,-.1,7)
            entry=np.zeros(7)
            worker.request_lock.acquire()
            try:self.assertFalse(worker.submit(session,time.monotonic(),entry,q,dq,ddq))
            finally:worker.request_lock.release()
            worker.response_lock.acquire()
            try:self.assertIsNone(worker.poll())
            finally:worker.response_lock.release()
            submitted=time.monotonic()
            self.assertTrue(worker.submit(session,submitted,entry,q,dq,ddq))
            deadline=time.monotonic()+5
            result=None
            while time.monotonic()<deadline:
                result=worker.poll()
                if result is not None:break
                time.sleep(.005)
            self.assertIsNotNone(result)
            self.assertTrue(result['ok'])
            self.assertEqual(result['session'],session)
            self.assertEqual(result['submitted'],submitted)
            expected=model.signs*(model.inverse_dynamics(q,dq,ddq)+model.friction(dq)-model.gravity_torques(entry))
            np.testing.assert_allclose(result['prediction'],expected,atol=1e-12)
        finally:worker.close()
        self.assertFalse(worker.is_alive())

    def test_slow_worker_cannot_block_trajectory_and_stale_output_is_unused(self):
        with tempfile.TemporaryDirectory() as directory,redirect_stdout(io.StringIO()):
            config=general.load_configuration(MODEL,'apply')
            config['log_path']=str(Path(directory)/'trace.jsonl')
            with mock.patch.object(backend,'_run',_delayed_worker_target):
                worker=backend.DynamicsWorker(config['model'].data)
            config['worker']=worker
            try:
                model=config['model'];pose=dict(zip(model.names,model.reference));zeros=dict.fromkeys(pose,0.)
                session=general.GeneralMotionSession(config,pose,['elbow'])
                target=dict(pose,elbow=pose['elbow']+.1)
                deadline=time.monotonic()+.65
                count=0
                while time.monotonic()<deadline:
                    session.prepare(target,.5,time.monotonic(),zeros,zeros)
                    self.assertEqual(session.transform('elbow',40,1,1.),(40,1,1.))
                    count+=1
                    time.sleep(.005)
                # Every worker evaluation takes >=150ms; the parent must still
                # execute many frames while it is busy and discard stale data.
                self.assertGreater(count,25)
                self.assertGreater(session.stale_frames,0)
                self.assertEqual(session.accepted_results,0)
                self.assertLess(session.frame_peak_ms,100)
                self.assertGreater(session.compute_peak_ms,150)
                session.prepare(target,1.,time.monotonic(),zeros,zeros)
                session.transform('elbow',40,1,1.)
                session.finish()
                report=json.loads(Path(config['log_path']).read_text(encoding='utf8'))
                self.assertEqual(report['prediction_status'],'no-fresh-worker-result')
                self.assertTrue(report['terminal_correction_zero'])
            finally:worker.close()

    def test_board_worker_separates_gravity_from_dynamic_residual(self):
        model=RigidBodyModel.load(MODEL);worker=backend.DynamicsWorker(model.data)
        try:
            session=worker.new_session(include_gravity=True)
            q=np.linspace(-.2,.1,7);zero=np.zeros(7)
            self.assertTrue(worker.submit(session,time.monotonic(),zero,q,zero,zero))
            deadline=time.monotonic()+5
            result=None
            while time.monotonic()<deadline:
                result=worker.poll()
                if result is not None:break
                time.sleep(.005)
            self.assertIsNotNone(result);self.assertTrue(result['ok'])
            np.testing.assert_allclose(result['gravity_delta'],result['prediction'],atol=1e-12)
        finally:worker.close()


if __name__=='__main__':unittest.main()
