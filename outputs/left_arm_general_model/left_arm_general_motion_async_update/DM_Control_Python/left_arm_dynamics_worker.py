#!/usr/bin/env python3
"""Isolated numerical process. No serial, CAN, motion or learning imports.

The parent uses try-lock mailboxes and never waits for a prediction. Each
mailbox has one slot: a newer reference replaces an older pending reference.
The worker is started and warmed BEFORE serial opens. Session IDs prevent
results from a previous trajectory being used with a new entry-gravity anchor.
"""
from __future__ import annotations

import atexit
import multiprocessing as mp
import os
import time

import numpy as np

from left_arm_rigid_body import RigidBodyModel


def _run(data, request, request_lock, response, response_lock, stop, ready):
    try:
        # Prefer the command process under CPU contention on Linux/Jetson.
        if hasattr(os, 'nice'):
            try:
                os.nice(5)
            except OSError:
                pass
        model = RigidBodyModel(data)
        n = model.n
        zero = np.zeros(n)
        # Cold NumPy/validation work is outside the motor-control process.
        model.inverse_dynamics(zero, zero, zero)
        model.friction(zero)
        last_sequence = -1
        last_session = None
        entry_gravity = None
        ready.set()
        while not stop.is_set():
            with request_lock:
                values = list(request[:])
            session, sequence, submitted = values[:3]
            if sequence <= 0 or sequence == last_sequence:
                stop.wait(.002)
                continue
            last_sequence = sequence
            start = time.perf_counter()
            try:
                vectors = np.asarray(values[3:]).reshape(4, n)
                if not np.isfinite(vectors).all():
                    raise ValueError('nonfinite dynamics request')
                entry, q, dq, ddq = vectors
                if session != last_session:
                    entry_gravity = model.gravity_torques(entry)
                    last_session = session
                prediction = model.signs*(model.inverse_dynamics(q, dq, ddq)
                                          + model.friction(dq)-entry_gravity)
                if not np.isfinite(prediction).all():
                    raise ValueError('nonfinite dynamics result')
                status = 1.
            except Exception:
                status = -1.
                prediction = np.zeros(n)
            elapsed_ms = (time.perf_counter()-start)*1000
            with response_lock:
                response[:] = [session, sequence, submitted, elapsed_ms, status, *prediction]
    except Exception as exc:
        print('general dynamics worker startup error=', str(exc), flush=True)
        with response_lock:
            response[4] = -2.
        ready.set()


class DynamicsWorker:
    def __init__(self, model_data, startup_timeout=10.):
        ctx = mp.get_context('spawn')
        n = len(model_data['joint_order'])
        self.request = ctx.RawArray('d', 3+4*n)
        self.response = ctx.RawArray('d', 5+n)
        self.request_lock = ctx.Lock()
        self.response_lock = ctx.Lock()
        self.stop = ctx.Event()
        self.ready = ctx.Event()
        self.sequence = 0
        self.session = 0
        self.closed = False
        self.process = ctx.Process(target=_run, args=(model_data, self.request,
            self.request_lock, self.response, self.response_lock, self.stop, self.ready),
            name='left-arm-numerical-dynamics', daemon=True)
        self.process.start()
        if not self.ready.wait(startup_timeout) or self.response[4] == -2 or not self.process.is_alive():
            self.close()
            raise RuntimeError('numerical dynamics worker failed startup before serial access')
        atexit.register(self.close)

    def new_session(self):
        self.session += 1
        return self.session

    def submit(self, session, submitted, entry, q, dq, ddq):
        if self.closed or not self.request_lock.acquire(False):
            return False
        try:
            self.sequence += 1
            self.request[:] = [session, self.sequence, submitted, *entry, *q, *dq, *ddq]
        finally:
            self.request_lock.release()
        return True

    def poll(self):
        if self.closed or not self.response_lock.acquire(False):
            return None
        try:
            values = list(self.response[:])
        finally:
            self.response_lock.release()
        if values[1] <= 0:
            return None
        return {'session': int(values[0]), 'sequence': int(values[1]),
                'submitted': values[2], 'compute_ms': values[3],
                'ok': values[4] == 1., 'prediction': np.asarray(values[5:])}

    def is_alive(self):
        return not self.closed and self.process.is_alive()

    def close(self, wait=True):
        if self.closed:
            return
        self.closed = True
        # Called at configuration failure / arm close, never in prepare/transform.
        # The child cannot send motor commands and owns no evidence/log writes.
        self.stop.set()
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=1. if wait else 0.)
        atexit.unregister(self.close)
