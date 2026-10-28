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

from left_arm_rigid_body import RigidBodyModel, vector


def _cross(a,b):
    """Fixed 3-vector cross product; no broadcast/axis dispatch per call."""
    return np.array((a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]))


class PreparedDynamics:
    """Same world-frame RNEA as RigidBodyModel, with immutable data prepared.

    Validate physical tensors once at worker startup, not for every reference.
    Joint-local axes in the calibrated serial model are all +Z; this permits
    direct 3x3 rotation updates instead of generic 4x4 axis-angle construction.
    The general implementation remains the independent numerical oracle.
    """
    def __init__(self,model):
        self.n=model.n
        self.parameters=model._parameters(None).copy()
        self.fixed_r=np.array([p[:3,:3] for p in model.parents])
        self.fixed_p=np.array([p[:3,3] for p in model.parents])
        for name in model.names:
            if not np.array_equal(model.rows[name]['axis_in_joint_frame'],[0.,0.,1.]):
                raise ValueError('prepared serial dynamics requires calibrated local +Z axes')
        self.gravity=model.configured_gravity()
        self.rotors=model.rotor_inertias().copy()
        self.signs=model.signs.copy()
        self.tensor=np.empty((self.n,3,3))
        for i,par in enumerate(self.parameters):
            xx,yy,zz,xy,xz,yz=par[4:]
            self.tensor[i]=[[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]]
        model.friction(np.zeros(self.n))
        self.coulomb=np.array([model.rows[n]['friction_prior']['coulomb'] for n in model.names])
        self.viscous=np.array([model.rows[n]['friction_prior']['viscous'] for n in model.names])

    def friction(self,dq):
        return self.coulomb*np.tanh(dq/.05)+self.viscous*dq

    def inverse_dynamics(self,q,dq,ddq):
        q=vector(q,self.n);dq=vector(dq,self.n);ddq=vector(ddq,self.n)
        positions=np.empty((self.n,3));axes=np.empty((self.n,3))
        forces=np.empty((self.n,3));moments=np.empty((self.n,3))
        previous_p=np.zeros(3);previous_r=np.eye(3)
        omega=np.zeros(3);alpha=np.zeros(3);acc=-self.gravity
        for i in range(self.n):
            r=previous_r@self.fixed_p[i]
            p=previous_p+r
            joint_r=previous_r@self.fixed_r[i]
            axis=joint_r[:,2]
            c,s=np.cos(q[i]),np.sin(q[i])
            rotation=np.column_stack((c*joint_r[:,0]+s*joint_r[:,1],
                                      -s*joint_r[:,0]+c*joint_r[:,1],axis))
            acc=acc+_cross(alpha,r)+_cross(omega,_cross(omega,r))
            next_alpha=alpha+axis*ddq[i]+_cross(omega,axis*dq[i])
            omega=omega+axis*dq[i];alpha=next_alpha
            mass=self.parameters[i,0];h=rotation@self.parameters[i,1:4]
            tensor=rotation@self.tensor[i]@rotation.T
            forces[i]=mass*acc+_cross(alpha,h)+_cross(omega,_cross(omega,h))
            moments[i]=tensor@alpha+_cross(omega,tensor@omega)+_cross(h,acc)
            positions[i]=p;axes[i]=axis
            previous_p=p;previous_r=rotation
        tau=np.empty(self.n)
        for i in range(self.n-1,-1,-1):
            if i+1<self.n:
                moments[i]+=moments[i+1]+_cross(positions[i+1]-positions[i],forces[i+1])
                forces[i]+=forces[i+1]
            tau[i]=axes[i]@moments[i]
        return tau+self.rotors*ddq


def _run(data, request, request_lock, response, response_lock, stop, ready):
    try:
        # Prefer the command process under CPU contention on Linux/Jetson.
        if hasattr(os, 'nice'):
            try:
                os.nice(5)
            except OSError:
                pass
        model = RigidBodyModel(data)
        prepared=PreparedDynamics(model)
        n = model.n
        zero = np.zeros(n)
        # Cold NumPy/validation work is outside the motor-control process.
        prepared.inverse_dynamics(zero, zero, zero)
        prepared.friction(zero)
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
            cpu_start = time.process_time()
            try:
                vectors = np.asarray(values[3:3+4*n]).reshape(4, n)
                if not np.isfinite(vectors).all():
                    raise ValueError('nonfinite dynamics request')
                entry, q, dq, ddq = vectors
                if session != last_session:
                    entry_gravity = prepared.inverse_dynamics(entry,zero,zero)
                    last_session = session
                prediction = prepared.signs*(prepared.inverse_dynamics(q, dq, ddq)
                                            + prepared.friction(dq)-entry_gravity)
                gravity_delta = (prepared.signs*(prepared.inverse_dynamics(q,zero,zero)-entry_gravity)
                                 if values[-1] else np.zeros(n))
                if not np.isfinite(prediction).all():
                    raise ValueError('nonfinite dynamics result')
                status = 1.
            except Exception:
                status = -1.
                prediction = np.zeros(n)
                gravity_delta = np.zeros(n)
            elapsed_ms = (time.perf_counter()-start)*1000
            cpu_ms = (time.process_time()-cpu_start)*1000
            with response_lock:
                response[:] = [session, sequence, submitted, elapsed_ms, status, cpu_ms, *prediction, *gravity_delta]
    except Exception as exc:
        print('general dynamics worker startup error=', str(exc), flush=True)
        with response_lock:
            response[4] = -2.
        ready.set()


class DynamicsWorker:
    def __init__(self, model_data, startup_timeout=10.):
        ctx = mp.get_context('spawn')
        n = len(model_data['joint_order'])
        self.n = n
        self.include_gravity = False
        self.request = ctx.RawArray('d', 4+4*n)
        self.response = ctx.RawArray('d', 6+2*n)
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

    def new_session(self, include_gravity=False):
        self.session += 1
        self.include_gravity = bool(include_gravity)
        return self.session

    def submit(self, session, submitted, entry, q, dq, ddq):
        if self.closed or not self.request_lock.acquire(False):
            return False
        try:
            self.sequence += 1
            self.request[:] = [session, self.sequence, submitted, *entry, *q, *dq, *ddq, float(self.include_gravity)]
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
                'ok': values[4] == 1., 'cpu_ms': values[5], 'prediction': np.asarray(values[6:6+self.n]),
                'gravity_delta': np.asarray(values[6+self.n:6+2*self.n])}

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
