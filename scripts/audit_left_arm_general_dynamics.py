#!/usr/bin/env python3
"""Read-only offline audit of dynamics samples and candidate URDF descriptions.

This is not a fitted controller and never imports the CAN/motion modules.
The harmonic design is a coverage diagnostic, NOT a rigid-body regressor.
"""
import argparse
import collections
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


JOINTS = ('shoulder_front', 'shoulder_side', 'shoulder_rotate', 'elbow',
          'arm_roll', 'wrist_side', 'wrist')
ROOT = Path(__file__).resolve().parent.parent
URDF_NAMES = ('left_arm_pitch_joint', 'left_arm_roll_joint', 'left_arm_yaw_joint',
              'left_elbow_pitch_joint', 'left_elbow_yaw_joint')


def distance_deg(a, b):
    return math.degrees(math.sqrt(sum(
        math.atan2(math.sin(a[n] - b[n]), math.cos(a[n] - b[n])) ** 2
        for n in JOINTS) / len(JOINTS)))


def audit_joint(data):
    rows = data['gravity_samples']
    inertia = data['inertia_samples']
    angles = np.array([[row['pose_rad'][n] for n in JOINTS] for row in rows])
    design = np.column_stack((np.ones(len(rows)), np.sin(angles), np.cos(angles)))
    singular = np.linalg.svd(design, compute_uv=False)
    # Overdetermination is required even when an arbitrary design has full row
    # rank. A low residual on eight rows cannot validate fifteen coefficients.
    rank = int(np.linalg.matrix_rank(design))
    close_pairs = []
    for index, left in enumerate(rows):
        for right in rows[index + 1:]:
            distance = distance_deg(left['pose_rad'], right['pose_rad'])
            if distance <= 0.5:
                close_pairs.append({
                    'distance_rms_deg': distance,
                    'absolute_tau_difference_nm': abs(left['tau'] - right['tau']),
                    'left_captured_at': left.get('captured_at'),
                    'right_captured_at': right.get('captured_at'),
                    'left_direction': left.get('direction'),
                    'right_direction': right.get('direction'),
                })
    identities = collections.Counter((row.get('captured_at'),
                                      tuple(row['pose_rad'][n] for n in JOINTS))
                                     for row in inertia)
    return {
        'gravity_count': len(rows), 'inertia_count': len(inertia),
        'gravity_directions': dict(collections.Counter(row.get('direction') for row in rows)),
        'gravity_source_counts': dict(collections.Counter(row.get('source') for row in rows)),
        'zero_tau_mad_count': sum(row.get('tau_mad') == 0 for row in rows),
        'sampled_pose_span_deg': dict(zip(JOINTS, np.degrees(np.ptp(angles, axis=0)).tolist())),
        'measured_holding_tau_range_nm': [min(row['tau'] for row in rows), max(row['tau'] for row in rows)],
        'inertia_range': [min(row['effective_inertia'] for row in inertia),
                          max(row['effective_inertia'] for row in inertia)],
        'inertia_methods': dict(collections.Counter(row.get('method') for row in inertia)),
        'inertia_duplicate_capture_and_pose_count': sum(n - 1 for n in identities.values()),
        'inertia_peak_acceleration_missing': sum(row.get('quality', {}).get('peak_acceleration') is None for row in inertia),
        'nearby_pose_largest_tau_difference': max(close_pairs, key=lambda p: p['absolute_tau_difference_nm'], default=None),
        'illustrative_15_column_harmonic_design': {
            'rows': len(rows), 'columns': 15, 'rank': rank,
            'full_column_rank': rank == 15,
            'singular_values': singular.tolist(),
            'condition_if_full_column_rank': float(singular[0] / singular[-1]) if rank == 15 else None,
            'warning': 'Coverage diagnostic only. Actual rigid-body parameter identifiability requires verified seven-axis geometry and a shared regressor.',
        },
        'configured_friction': data['friction'],
    }


def urdf_audit(path):
    root = ET.parse(path).getroot()
    links = {node.get('name'): node for node in root.findall('link')}
    joints = {node.get('name'): node for node in root.findall('joint')}
    result = []
    for name in URDF_NAMES:
        joint = joints[name]
        child = joint.find('child').get('link')
        inertial = links[child].find('inertial')
        mass = float(inertial.find('mass').get('value'))
        tensor = inertial.find('inertia')
        xx, yy, zz, xy, xz, yz = (float(tensor.get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'))
        matrix = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        principal = np.linalg.eigvalsh(matrix)
        result.append({
            'joint': name, 'parent': joint.find('parent').get('link'), 'child': child,
            'origin': dict(joint.find('origin').attrib),
            'axis': dict(joint.find('axis').attrib),
            'limit': dict(joint.find('limit').attrib),
            'mass_kg': mass, 'com_origin': dict(inertial.find('origin').attrib),
            'inertia_tensor': dict(tensor.attrib), 'principal_inertias': principal.tolist(),
            'positive_mass_and_inertia': mass > 0 and bool(np.all(principal > 0)),
            'principal_triangle_inequality': bool(principal[-1] <= sum(principal[:2]) + 1e-10),
        })
    return {
        'path': str(path.relative_to(ROOT)), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'left_arm_revolute_joint_count': len(result), 'joints': result,
        'five_descendant_link_mass_kg': sum(item['mass_kg'] for item in result),
        'missing_current_joints': ['wrist_side', 'wrist'],
        'missing_tool_description': 'No wrist/claw links or attached payload description in this arm chain.',
        'deployment_approved': False,
    }


def run(model_path):
    raw = model_path.read_bytes()
    model = json.loads(raw)
    source = (ROOT / 'scripts' / 'left_arm_v2_6.py').read_text(encoding='utf-8')
    motors = [{'joint': name, 'motor_type': motor, 'slave_id': sid, 'master_id': mid}
              for name, motor, sid, mid in re.findall(
                  r'JointSpec\("([^\"]+)", DM_Motor_Type\.(\w+), (0x\w+), (0x\w+)\)', source)]
    joint_reports = {name: audit_joint(model['joints'][name]) for name in JOINTS}
    return {
        'scope': 'offline audit only; no controller, model or learning data modified',
        'model_path': str(model_path.relative_to(ROOT)), 'model_sha256': hashlib.sha256(raw).hexdigest(),
        'model_updated_at': model.get('updated_at'), 'gain_units_verified': model.get('gain_units_verified'),
        'gravity_total': sum(row['gravity_count'] for row in joint_reports.values()),
        'inertia_total': sum(row['inertia_count'] for row in joint_reports.values()),
        'joints': joint_reports, 'current_controller_motor_specs': motors,
        'urdf_candidates': [urdf_audit(path) for path in sorted((ROOT / 'roboparty_rpo_hardware').rglob('*.urdf'))],
        'missing_local_raw_logs': [name for name in ('left_arm_v3_1_inertia_runs.jsonl', 'left_arm_v3_1_model_runs.jsonl')
                                   if not any((ROOT / folder / name).is_file() for folder in ('scripts', 'scripts/data'))],
        'known_structure_difference_evidence': {
            'wrist_side_and_wrist': 'Present in current controller, absent from both candidate URDF arm chains.',
            'historical_tip_shortening_mm': 40,
            'historical_tip_shortening_date': '2026-07-08',
            'historical_source': '左臂装机测试记录_20260703.md:451',
            'current_geometry_confirmation': 'User confirms no updated CAD/URDF/dimension drawing is available; current transforms remain unverified.',
        },
        'not_identified_by_current_json': [
            'encoder-to-geometric zero/sign mapping',
            'current seven-axis link transforms and tool/payload geometry',
            'complete physical link mass/first moments/inertia tensors',
            'off-diagonal joint coupling terms',
            'velocity-dependent friction from sustained motion',
            'joint-side torque scale/bias and per-feedback timing uncertainty',
            'structural compliance and usable closed-loop bandwidth',
        ],
        'preserved_runtime_behavior': [
            'existing Model Hold takeover', 'existing adaptive learning',
            'ordinary Clearance shoulder_rotate active-target skip',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT / 'scripts' / 'left_arm_v3_1_dynamics.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs' / 'left_arm_general_dynamics_audit' / 'audit.json')
    args = parser.parse_args()
    result = run(args.model.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'gravity_samples': result['gravity_total'], 'inertia_samples': result['inertia_total'],
                      'candidate_urdfs': len(result['urdf_candidates']), 'output': str(args.output)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
