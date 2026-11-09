"""Reproduce numerical fits from the sealed audit's self-contained dataset.

The original audit was executed as two stdin scripts. This retained numerical
reproducer uses the recorded FK, wrist rotations, nominal XY labels and CAD
clearances as its immutable inputs; it does not rerun historical extraction or
modify the audit/model/configuration. NumPy is the sole external dependency.
Run: python reproduce_common_geometry_fit_audit.py [--audit PATH]
"""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[key] = '1'
import argparse
import json
import math
from pathlib import Path
import numpy as np


def rotation(v):
    angle = np.linalg.norm(v)
    if angle < 1e-15:
        return np.eye(3)
    x, y, z = v / angle
    k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle)*k + (1-np.cos(angle))*(k@k)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path, default=Path(__file__).parents[1]/'common_geometry_fit_audit.json')
    args = parser.parse_args()
    audit = json.loads(args.audit.read_text(encoding='utf8'))
    rows = audit['dataset']
    b = np.array([r['baseline_tcp_board_mm'] for r in rows])
    a = np.array([r['wrist_rotation_in_board'] for r in rows])
    y = np.array([r['nominal_physical_xy_mm'] for r in rows])
    count = len(rows)

    def fit_at(ids, kind, w):
        c = rotation(-np.asarray(w))
        bs = b@c.T
        aa = np.einsum('ij,njk->nik', c, a)
        target = (y[ids]-bs[ids, :2]).reshape(-1)
        if kind == 'translation_only':
            design = np.tile(-np.eye(2), (len(ids), 1))
        elif kind == 'tcp_only':
            design = aa[ids, :2, :].reshape(-1, 3)
        else:
            design = np.column_stack((aa[ids, :2, :].reshape(-1, 3), np.tile(-np.eye(2), (len(ids), 1))))
        p = np.linalg.lstsq(design, target, rcond=1e-10)[0]
        dp, dt = np.zeros(3), np.zeros(2)
        if kind == 'translation_only':
            dt = p
        elif kind == 'tcp_only':
            dp = p
        else:
            dp, dt = p[:3], p[3:]
        pred = (b+np.einsum('nij,j->ni', a, dp))@c.T
        pred[:, :2] -= dt
        residual = (pred[ids, :2]-y[ids]).reshape(-1)
        return residual, dp, dt, pred

    def fit(ids, kind):
        if kind not in ('tcp_translation_yaw', 'tcp_translation_full_board_rotation_gauge_fixed'):
            return fit_at(ids, kind, np.zeros(3))
        if kind == 'tcp_translation_yaw':
            def score(theta):
                e = fit_at(ids, kind, [0., 0., theta])[0]
                return e@e
            grid = np.linspace(-np.pi/12, np.pi/12, 121)
            index = int(np.argmin([score(t) for t in grid]))
            low, high = grid[max(0, index-1)], grid[min(len(grid)-1, index+1)]
            phi = (math.sqrt(5)-1)/2
            for _ in range(32):
                first, second = high-phi*(high-low), low+phi*(high-low)
                if score(first) < score(second):
                    high = second
                else:
                    low = first
            return fit_at(ids, kind, [0., 0., (low+high)/2])
        seeds = [np.zeros(3)]+[np.eye(3)[i]*sign*np.radians(7) for i in range(3) for sign in (-1, 1)]
        best = None
        for seed in seeds:
            w, damping = seed.copy(), .01
            for _ in range(120):
                e = fit_at(ids, kind, w)[0]
                epsilon = 1e-5
                jac = np.column_stack([(fit_at(ids, kind, w+np.eye(3)[k]*epsilon)[0]-fit_at(ids, kind, w-np.eye(3)[k]*epsilon)[0])/(2*epsilon) for k in range(3)])
                step = np.linalg.solve(jac.T@jac+damping*np.eye(3), -jac.T@e)
                if np.linalg.norm(step) < 1e-10:
                    break
                trial = w+step
                trial *= min(1., np.radians(15)/max(np.linalg.norm(trial), 1e-30))
                en = fit_at(ids, kind, trial)[0]
                if en@en < e@e-1e-12:
                    w, damping = trial, max(1e-8, damping/3)
                else:
                    damping = min(1e12, damping*10)
                if damping >= 1e12:
                    break
            item = fit_at(ids, kind, w)
            if best is None or item[0]@item[0] < best[0]@best[0]:
                best = item
        return best

    results = []
    for recorded in audit['fit_models']:
        kind = recorded['model']
        full = fit(np.arange(count), kind)
        loo = []
        for k in range(count):
            fitted = fit(np.array([j for j in range(count) if j != k]), kind)
            loo.append(float(np.linalg.norm(fitted[3][k, :2]-y[k])))
        actual = {'training_xy_rms_norm_mm': float(np.sqrt(np.mean(full[0]**2)*2)),
                  'LOO_xy_rms_norm_mm': float(np.sqrt(np.mean(np.square(loo)))),
                  'LOO_xy_max_norm_mm': max(loo), 'A8_heldout_error_mm': loo[-1]}
        expected = {key: recorded[key] for key in actual if key != 'A8_heldout_error_mm'}
        expected['A8_heldout_error_mm'] = recorded['A8_heldout']['error_norm_mm']
        # Nonlinear optimizers can take a different last floating-point step.
        for key, value in actual.items():
            if not math.isclose(value, expected[key], rel_tol=1e-5, abs_tol=1e-4):
                raise AssertionError((kind, key, value, expected[key]))
        results.append(dict(model=kind, **actual))
    a8 = rows[-1]
    assert math.isclose(a8['baseline_tcp_board_mm'][2]-a8['minimum_CAD_gripper_vertex_height_mm'], 26.480184823797757, abs_tol=1e-10)
    print(json.dumps({'all_recorded_fit_metrics_reproduced': True, 'dataset_rows': count,
                      'models': results, 'runtime_or_audit_written': False}, indent=2))


if __name__ == '__main__':
    main()
