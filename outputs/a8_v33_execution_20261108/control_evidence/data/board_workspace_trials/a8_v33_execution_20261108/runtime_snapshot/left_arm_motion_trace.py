"""Bounded, asynchronous per-frame motor-command evidence (no CAN reads)."""
import json
import math
import queue
import threading
import time
from pathlib import Path


class MotionTrace:
    def __init__(self, path, capacity=2048):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.path.touch(exist_ok=True)  # verify storage before entering motion
        self.queue=queue.Queue(maxsize=capacity)
        self.dropped=0;self.written=0;self.error=None;self.closed=False;self.capture_errors=0
        self.thread=threading.Thread(target=self._write,daemon=True)
        self.thread.start()

    def submit(self, row):
        if self.closed:return
        try:self.queue.put_nowait(row)
        except queue.Full:self.dropped+=1

    def _write(self):
        try:
            with self.path.open('a',encoding='utf-8') as f:
                last=time.monotonic()
                while True:
                    try:row=self.queue.get(timeout=.1)
                    except queue.Empty:
                        f.flush()
                        continue
                    if row is None:break
                    f.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n')
                    self.written+=1
                    if time.monotonic()-last>=.25:
                        f.flush();last=time.monotonic()
                f.write(json.dumps({'event':'trace_closed','frames_written':self.written,
                                    'queue_dropped_frames':self.dropped})+'\n')
        except Exception as exc:self.error=str(exc)

    def close(self):
        if self.closed:return
        self.closed=True
        # Only called after motion/support ends, never from the command loop.
        if self.thread.is_alive():
            try:self.queue.put(None,timeout=1.)
            except queue.Full:return
            self.thread.join(timeout=2.)


def capture_frame(motors, commands, context, completed, previous_time=None):
    start=time.monotonic()
    invalid=[]
    def number(name,key,value):
        value=float(value)
        if math.isfinite(value):return value
        invalid.append(name+'.'+key)
        return None
    row={'event':'motor_frame','wall_s':time.time(),'monotonic_s':start,
         'frame_interval_ms':None if previous_time is None else (start-previous_time)*1000,
         'context':dict(context),'commands':{n:dict(c) for n,c in commands.items()},
         'control_call_completed_monotonic_s':dict(completed),
         'feedback_source':'MIT-reply-cache','individual_feedback_freshness_verified':False,
         'feedback':{n:{'pos':number(n,'pos',m.getPosition()),'vel':number(n,'vel',m.getVelocity()),
                        'tau':number(n,'tau',m.getTorque())} for n,m in motors.items()},
         'invalid_feedback_fields':invalid}
    row['capture_ms']=(time.monotonic()-start)*1000
    return row
