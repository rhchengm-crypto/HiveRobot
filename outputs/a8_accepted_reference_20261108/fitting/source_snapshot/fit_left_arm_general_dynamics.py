#!/usr/bin/env python3
"""Fit a nominal coupled model to existing quasi-static/step identification.

The 29 scalar CLOSED-LOOP effective inertias are soft M_ii constraints, not
measurements of a free-body tensor. CAD segment cylinders are shape priors,
not measured mass properties. Nonnegative cylinder mixtures keep inertias
physical. Per-joint static offset/approach terms are nuisance variables and
are NOT additional deployed feedforward. Historical files are never changed.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from left_arm_rigid_body import RigidBodyModel, inertial_parameters

ROOT = Path(__file__).resolve().parents[1]


def nonnegative_fit(a, b, lower, ridge=1e-3):
    """Scaled active-set NNLS; deterministic and NumPy-only on Jetson."""
    scales = np.maximum(np.linalg.norm(a, axis=0), 1e-12)
    x = a/scales
    h = x.T@x + ridge*np.eye(x.shape[1])
    rhs = x.T@b
    floor = np.asarray(lower)*scales
    c = rhs-h@floor
    z = np.zeros(len(c))
    passive = np.zeros(len(c), dtype=bool)
    for iteration in range(2000):
        gradient = c-h@z
        candidates = np.where(~passive, gradient, -np.inf)
        if np.max(candidates) <= 1e-9:
            return (z+floor)/scales, iteration+1
        passive[int(np.argmax(candidates))] = True
        for _ in range(2000):
            trial = np.zeros(len(c))
            trial[passive] = np.linalg.solve(h[np.ix_(passive,passive)], c[passive])
            if np.all(trial[passive] > 0):
                z = trial
                break
            blocking = passive & (trial <= 0)
            alpha = np.min(z[blocking]/np.maximum(z[blocking]-trial[blocking],1e-30))
            z += alpha*(trial-z)
            passive[passive & (z <= 1e-12)] = False
            z[~passive] = 0
        else:
            raise ValueError('nominal fit inner active set did not converge')
    raise ValueError('nominal fit did not converge')


def shape_basis(model, claw_center):
    templates = []
    for i in range(model.n):
        frame = model.home[i]
        end = model.points[i+1] if i+1 < model.n else claw_center
        segment = frame[:3,:3].T@(end-frame[:3,3])
        length = float(np.linalg.norm(segment))
        if length <= 1e-6:
            raise ValueError('CAD segment length is degenerate')
        axis = segment/length
        # r28.5 is a visible CAD motor-face circle; used as a NOMINAL shape
        # envelope, not a density or proof of physical link ownership.
        radius = .0285
        inertia = ((length*length/12+radius*radius/4)*(np.eye(3)-np.outer(axis,axis))
                   + radius*radius/2*np.outer(axis,axis))
        for fraction in (.2, .5, .8):
            body = {'mass_kg': 1.0, 'com_m': (fraction*segment).tolist(),
                    'inertia_com_kg_m2': inertia.tolist()}
            templates.append((i, body, inertial_parameters([body])[0]))
    return templates


def fit(geometry, history, inspection):
    model = RigidBodyModel(geometry)
    if geometry.get('sources',{}).get('approved_step_sha256') != inspection.get('source_sha256'):
        raise ValueError('CAD inspection and calibrated model refer to different STEP revisions')
    picker = next(row for row in inspection['named_geometric_parts'] if row['name'] == 'OmniPicker.step')
    claw_center = np.mean(picker['vertex_bounds_mm'], axis=0)*.001
    basis = shape_basis(model, claw_center)
    parameter_map = np.zeros((10*model.n, len(basis)))
    for k, (i, _, parameters) in enumerate(basis):
        parameter_map[10*i:10*i+10,k] = parameters
    gravity_map = np.concatenate([parameter_map[10*i:10*i+4] for i in range(model.n)])
    gravity_a, gravity_b, nuisance, inertia_a, inertia_b, provenance = [], [], [], [], [], []
    for j, name in enumerate(model.names):
        for sample in history['joints'][name]['gravity_samples']:
            q = model.encoder_to_model([sample['pose_rad'][n] for n in model.names])
            gravity_a.append(model.gravity_regressor(q)[j]@gravity_map)
            gravity_b.append(model.signs[j]*float(sample['tau']))
            row = np.zeros(2*model.n)
            row[j] = 1
            row[model.n+j] = model.signs[j]*{'positive': 1., 'negative': -1.}.get(sample.get('direction'), 0.)
            nuisance.append(row)
        seen = set()
        for sample in history['joints'][name]['inertia_samples']:
            key = (sample.get('captured_at'), tuple(sample['pose_rad'][n] for n in model.names), sample['effective_inertia'])
            if key in seen:
                continue
            seen.add(key)
            q = model.encoder_to_model([sample['pose_rad'][n] for n in model.names])
            frames = model.fk(q)
            row = np.zeros(len(basis)+model.n)
            for k, (i, body, _) in enumerate(basis):
                jv, jw = model.point_jacobian(q, i, body['com_m'])
                rotation = frames[i,:3,:3]
                tensor = rotation@np.asarray(body['inertia_com_kg_m2'])@rotation.T
                row[k] = jv[:,j]@jv[:,j] + jw[:,j]@tensor@jw[:,j]
            row[len(basis)+j] = 1
            inertia_a.append(row)
            value = float(sample['effective_inertia'])
            if not np.isfinite(value) or value <= 0:
                raise ValueError('invalid effective inertia')
            inertia_b.append(value)
            provenance.append({'joint': name, 'captured_at': sample.get('captured_at'),
                               'label': sample.get('label'), 'effective_inertia': value})
    ga, gb, nuisance = np.asarray(gravity_a), np.asarray(gravity_b), np.asarray(nuisance)
    ia, ib = np.asarray(inertia_a), np.asarray(inertia_b)
    if len(ib) == 0 or len(gb) == 0:
        raise ValueError('historical gravity and inertia observations are required')
    # Eliminate static torque and approach-history offsets before mass fitting.
    pg = ga-nuisance@np.linalg.lstsq(nuisance, ga, rcond=1e-9)[0]
    pb = gb-nuisance@np.linalg.lstsq(nuisance, gb, rcond=1e-9)[0]
    weights = 1/np.maximum(.5*ib, .01)
    a = np.vstack((np.pad(pg/.5, ((0,0),(0,model.n))), ia*weights[:,None]))
    b = np.concatenate((pb/.5, ib*weights))
    lower = np.r_[np.full(len(basis), 1e-6), np.zeros(model.n)]
    coefficients, iterations = nonnegative_fit(a, b, lower)
    if not np.isfinite(coefficients).all():
        raise ValueError('nonfinite nominal fit')
    result = copy.deepcopy(geometry)
    for i, name in enumerate(model.names):
        pars = parameter_map[10*i:10*i+10]@coefficients[:len(basis)]
        mass = pars[0]
        com = pars[1:4]/mass
        xx, yy, zz, xy, xz, yz = pars[4:]
        origin = np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
        tensor = origin-mass*((com@com)*np.eye(3)-np.outer(com,com))
        result['links'][name+'_link']['inertial'] = {'mass_kg': float(mass), 'com_m': com.tolist(),
                                                   'inertia_com_kg_m2': tensor.tolist()}
    result['reflected_rotor_inertias_kg_m2'] = coefficients[len(basis):].tolist()
    result['parameter_status'] = 'nominal-estimate-from-existing-tests; not independently measured link mass properties'
    result['runtime_apply_requires_explicit_opt_in'] = True
    result['missing_for_physical_prediction'] = []
    nuisance_fit = np.linalg.lstsq(nuisance, gb-ga@coefficients[:len(basis)], rcond=1e-9)[0]
    residual_g = ga@coefficients[:len(basis)]+nuisance@nuisance_fit-gb
    residual_i = ia@coefficients-ib
    report = {'gravity_records_used': len(gb), 'unique_inertia_records_used': len(ib),
              'inertia_records': provenance, 'fit_columns': a.shape[1],
              'fit_design_rank': int(np.linalg.matrix_rank(a)), 'iterations': iterations,
              'gravity_training_rmse_with_nuisance_Nm': float(np.sqrt(np.mean(residual_g**2))),
              'effective_inertia_training_rmse_kg_m2': float(np.sqrt(np.mean(residual_i**2))),
              'effective_inertia_median_relative_error': float(np.median(abs(residual_i)/ib)),
              'total_nominal_mass_kg': sum(row['inertial']['mass_kg'] for row in result['links'].values()),
              'nominal_shape_radius_m': .0285, 'claw_CAD_bbox_center_m': claw_center.tolist(),
              'assumptions': ['scalar closed-loop effective inertia approximates M_ii as a soft constraint',
                              'CAD segments use nonnegative cylinder mixtures at 20/50/80 percent',
                              'claw bounding-box centre defines only a nominal mass distribution, not a measured COM or TCP',
                              'diagonal residual inertias are fitted nuisance drive/shape priors, not measured rotor inertias',
                              'static offset and approach nuisance terms are not deployed feedforward',
                              'rank deficiency is regularized; the inferred masses are not uniquely identified'],
              'physical_parameters_independently_verified': False,
              'link_mass_interpretation': 'effective lumped bodies; near-zero fitted mass does not mean the physical link is absent',
              'fit_is_not_hardware_validation': True}
    result['identification'] = report
    fitted = RigidBodyModel(result)
    inertial_parameters(fitted.inertials)
    for _ in range(30):
        # A deterministic algebraic check, never a hardware motion plan.
        q = np.random.default_rng(_).uniform(-1, 1, fitted.n)
        if np.linalg.eigvalsh(fitted.mass_matrix(q))[0] <= 0:
            raise ValueError('nominal mass matrix is not positive definite')
    return result, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, default=ROOT/'outputs/left_arm_general_model/left_arm_cad_exact.json')
    parser.add_argument('--history', type=Path, default=ROOT/'scripts/left_arm_v3_1_dynamics.json')
    parser.add_argument('--inspection', type=Path, default=ROOT/'outputs/left_arm_step_analysis/revised_2/step_inspection.json')
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/left_arm_general_model/left_arm_general_dynamics.json')
    args = parser.parse_args()
    data = [json.loads(p.read_text(encoding='utf8')) for p in (args.geometry,args.history,args.inspection)]
    result, report = fit(*data)
    result['identification_sources'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                       for p in (args.geometry,args.history,args.inspection)}
    if args.output.resolve() in (args.geometry.resolve(),args.history.resolve(),args.inspection.resolve()):
        raise ValueError('output must not overwrite source evidence')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps({'output': str(args.output), **{k:v for k,v in report.items() if k != 'inertia_records'}}, indent=2))


if __name__ == '__main__':
    main()
