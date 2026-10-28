#!/usr/bin/env python3
"""Offline AP214 assembly/cylindrical-feature inspection; no robot imports.

Axis candidates are geometric lines, not verified revolute joints. In
particular their normalized signs have no encoder-direction meaning.
"""
import argparse
import collections
import hashlib
import json
import re
from pathlib import Path

import numpy as np


STRINGS = re.compile(r"'(?:''|[^'])*'")
ENTITIES = re.compile(r"#(\d+)\s*=\s*((?:'(?:''|[^'])*'|[^';])*);", re.S)


def decode_name(text):
    for tag, encoding in (('X2', 'utf-16-be'), ('X4', 'utf-32-be')):
        text = re.sub(r'\\' + tag + r'\\([0-9A-Fa-f]+)\\X0\\',
                      lambda m: bytes.fromhex(m[1]).decode(encoding), text)
    return text.replace("''", "'")


def references(text):
    return [int(value) for value in re.findall(r'#(\d+)', STRINGS.sub('', text))]


class StepInspection:
    def __init__(self, path, confirmed_current_assembly=False):
        self.path = Path(path)
        self.confirmed_current_assembly = bool(confirmed_current_assembly)
        self.raw = self.path.read_bytes()
        self.text = self.raw.decode('utf-8')
        self.entities = {int(m[1]): m[2] for m in ENTITIES.finditer(self.text)}
        units = {re.search(r'SI_UNIT\(([^()]*)\)', value)[1].replace(' ', '')
                 for value in self.entities.values() if 'LENGTH_UNIT()' in value and 'SI_UNIT(' in value}
        if units != {'.MILLI.,.METRE.'}:
            raise ValueError('this inspection requires consistently declared millimetre units: ' + str(units))
        self.graph = {n: references(value) for n, value in self.entities.items()}
        self.brep_links = collections.defaultdict(list)
        self.parents = {}
        for value in self.entities.values():
            if value.startswith('SHAPE_REPRESENTATION_RELATIONSHIP('):
                a, b = references(value)
                self.brep_links[a].append(b)
            if value.startswith('(REPRESENTATION_RELATIONSHIP(') and 'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION' in value:
                child, parent, transformation = references(value)
                if child in self.parents:
                    raise ValueError('shared representation needs per-occurrence traversal')
                self.parents[child] = parent, transformation

    def xyz(self, entity):
        values = re.findall(r'\(([-+0-9.Ee, ]+)\)', self.entities[entity])
        return np.array([float(v) for v in values[-1].split(',')])

    def frame(self, entity):
        point, direction, reference = self.graph[entity]
        z = self.xyz(direction)
        z /= np.linalg.norm(z)
        x = self.xyz(reference)
        x -= z * np.dot(z, x)
        x /= np.linalg.norm(x)
        matrix = np.eye(4)
        matrix[:3, :3] = np.column_stack((x, np.cross(z, x), z))
        matrix[:3, 3] = self.xyz(point)
        return matrix

    def placement(self, rep, seen=None):
        seen = set() if seen is None else set(seen)
        if rep in seen:
            raise ValueError('cyclic assembly placement')
        seen.add(rep)
        if rep not in self.parents:
            return np.eye(4)
        parent, transformation = self.parents[rep]
        source, target = self.graph[transformation]
        local = self.frame(target) @ np.linalg.inv(self.frame(source))
        return self.placement(parent, seen) @ local

    def reachable(self, root):
        seen, pending = set(), [root] + self.brep_links[root]
        while pending:
            entity = pending.pop()
            if entity in seen:
                continue
            seen.add(entity)
            pending.extend(n for n in self.graph[entity] if n not in seen)
        return seen

    def part(self, rep, name):
        matrix = self.placement(rep)
        nodes = self.reachable(rep)
        vertices = []
        groups = {}
        for entity in sorted(nodes):
            value = self.entities[entity]
            if value.startswith('VERTEX_POINT('):
                point = self.xyz(self.graph[entity][0])
                vertices.append(matrix[:3, :3] @ point + matrix[:3, 3])
            if not value.startswith('CYLINDRICAL_SURFACE('):
                continue
            radius = float(value.rsplit(',', 1)[1][:-1])
            axis = matrix @ self.frame(self.graph[entity][0])
            direction = axis[:3, 2]
            if next((v for v in direction if abs(v) > 1e-8), 1) < 0:
                direction = -direction
            closest = axis[:3, 3] - np.dot(axis[:3, 3], direction) * direction
            key = tuple(np.round(np.concatenate((direction, closest)), 4))
            row = groups.setdefault(key, {
                'direction': direction.tolist(),
                'point_nearest_cad_origin_mm': closest.tolist(),
                'radius_features_mm': [], 'surface_entities': [],
            })
            row['radius_features_mm'].append(radius)
            row['surface_entities'].append(entity)
        ranked = sorted(groups.values(), key=lambda row: sum(r * r for r in row['radius_features_mm']), reverse=True)
        points = np.array(vertices)
        result = {'name': name, 'representation_entity': rep,
                  'placement_in_cad_frame': matrix.tolist(), 'vertex_count': len(vertices)}
        if len(vertices):
            result['vertex_bounds_mm'] = [points.min(axis=0).tolist(), points.max(axis=0).tolist()]
        if name == 'Wrist motor.step' and ranked:
            candidate = ranked[0]
            d = np.array(candidate['direction'])
            p = np.array(candidate['point_nearest_cad_origin_mm'])
            # Any point along the line is geometrically valid. This display
            # point is NOT claimed to be the output-flange center or mass COM.
            candidate['display_point_on_axis_mm'] = (p + np.dot(points.mean(axis=0), d) * d).tolist()
            candidate['candidate_score_ratio_to_next_axis'] = (
                sum(r*r for r in candidate['radius_features_mm']) /
                sum(r*r for r in ranked[1]['radius_features_mm']) if len(ranked) > 1 else None)
            candidate['status'] = 'geometric-cylinder-axis-candidate; joint identity and sign unverified'
            result['motor_axis_candidate'] = candidate
        return result, vertices

    def inspect(self):
        parts, clouds, motors = [], [], []
        for entity, value in self.entities.items():
            if not value.startswith('SHAPE_REPRESENTATION(') or entity not in self.brep_links:
                continue
            name = decode_name(STRINGS.findall(value)[0][1:-1])
            part, vertices = self.part(entity, name)
            index = len(parts)
            if 'motor_axis_candidate' in part:
                part['motor_label'] = f'M{len(motors) + 1}'
                motors.append(part)
            if vertices:
                clouds.append({'part_index': index, 'vertices_mm': [p.tolist() for p in vertices]})
            parts.append(part)
        products = [decode_name(STRINGS.findall(value)[0][1:-1])
                    for value in self.entities.values() if value.startswith('PRODUCT(')]
        result = {
            'source_path': str(self.path.resolve()), 'source_sha256': hashlib.sha256(self.raw).hexdigest(),
            'source_bytes': len(self.raw), 'declared_length_unit': 'mm',
            'unit_evidence': 'SI_UNIT(.MILLI.,.METRE.)',
            'user_confirmed_matches_current_robot': self.confirmed_current_assembly,
            'entity_count': len(self.entities), 'products': products,
            'assembly_occurrences': sum(value.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE(') for value in self.entities.values()),
            'named_geometric_parts': parts, 'motor_axis_candidates': motors,
            'kinematic_or_density_definitions_found': re.findall(
                r'\b(?:REVOLUTE_PAIR|KINEMATIC_LINK|MASS_DENSITY|MASS_MEASURE|MATERIAL_DESIGNATION)\b', self.text),
            'limitations': [
                'Axis candidates are recovered from repeated coaxial cylindrical surfaces, not revolute-joint definitions.',
                'Axis signs are normalized only for grouping; they are not encoder-positive directions.',
                'Vertex bounds and display points are not exact BRep bounding boxes, flange centers or physical centers of mass.',
                'Geometric volume and uniform-density centroids cannot establish real motor/gripper mass properties.',
                'STEP assembly component groups do not by themselves identify which parts move as one rigid link.',
            ],
            'deployment_ready': False,
        }
        return result, clouds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--confirmed-current-assembly', action='store_true')
    args = parser.parse_args()
    inspection = StepInspection(args.step, args.confirmed_current_assembly)
    result, clouds = inspection.inspect()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / 'step_inspection.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    (args.output_dir / 'step_vertex_preview.json').write_text(json.dumps(clouds), encoding='utf-8')
    print(json.dumps({'products': len(result['products']), 'parts': len(result['named_geometric_parts']),
                      'motor_candidates': len(result['motor_axis_candidates']),
                      'units': result['declared_length_unit'], 'deployment_ready': False}))


if __name__ == '__main__':
    main()
