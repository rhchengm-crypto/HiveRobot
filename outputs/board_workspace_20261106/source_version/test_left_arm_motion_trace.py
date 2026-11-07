import json
import queue
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock
from left_arm_motion_trace import MotionTrace,capture_frame


class MotionTraceTests(unittest.TestCase):
    def test_complete_frame_and_termination_survive_async_writer(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'trace.jsonl';trace=MotionTrace(path)
            motor=types.SimpleNamespace(getPosition=lambda:.1,getVelocity=lambda:.02,getTorque=lambda:3.)
            command={'elbow':dict(kp=70.,kd=3.,q=.2,dq=0.,tau_ff=2.)}
            row=capture_frame({'elbow':motor},command,{'stage':'contact','waypoint_index':7},{'elbow':1.})
            trace.submit(row);command['elbow']['q']=99
            trace.close()
            lines=[json.loads(s) for s in path.read_text().splitlines()]
            self.assertEqual(lines[0]['commands']['elbow']['q'],.2)
            self.assertEqual(lines[0]['feedback']['elbow']['tau'],3.)
            self.assertFalse(lines[0]['individual_feedback_freshness_verified'])
            self.assertEqual(lines[-1]['frames_written'],1)
            self.assertIsNone(trace.error)

    def test_full_queue_drops_without_blocking_motion(self):
        trace=MotionTrace.__new__(MotionTrace);trace.queue=queue.Queue(maxsize=1)
        trace.closed=False;trace.dropped=0
        trace.submit({'n':1});trace.submit({'n':2})
        self.assertEqual(trace.dropped,1)
        self.assertEqual(trace.queue.get_nowait(),{'n':1})

    def test_nonfinite_feedback_is_explicit_without_stopping_writer(self):
        motor=types.SimpleNamespace(getPosition=lambda:float('nan'),getVelocity=lambda:0.,getTorque=lambda:1.)
        row=capture_frame({'elbow':motor},{'elbow':{}},{},{})
        self.assertIsNone(row['feedback']['elbow']['pos'])
        self.assertEqual(row['invalid_feedback_fields'],['elbow.pos'])
        json.dumps(row,allow_nan=False)
