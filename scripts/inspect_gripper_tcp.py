"""Inspect the gripper's STEP face evidence without loading a motor driver."""
import json
from pathlib import Path
import numpy as np
from inspect_left_arm_step import StepInspection


def inspect(step):
    reader = StepInspection(step, True)
    rep = 63844  # OmniPicker occurrence in the approved revised STEP.
    matrix = reader.placement(rep)
    nodes = reader.reachable(rep)
    faces = []
    for node in sorted(nodes):
        value = reader.entities[node]
        if not value.startswith('ADVANCED_FACE('):
            continue
        refs = reader.graph[node]
        surface = next((n for n in refs if reader.entities[n].startswith('PLANE(')), None)
        if surface is None:
            continue
        points = [reader.xyz(reader.graph[n][0]) for n in reader.reachable(node)
                  if reader.entities[n].startswith('VERTEX_POINT(')]
        if len(points) < 3:
            continue
        points = np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]
        frame = matrix @ reader.frame(reader.graph[surface][0])
        faces.append({'face': node, 'plane': surface, 'origin_mm': frame[:3, 3].tolist(),
                      'normal': frame[:3, 2].tolist(), 'vertices_mm': points.tolist(),
                      'center_mm': points.mean(0).tolist(),
                      'span_mm': np.ptp(points @ frame[:3, :2], axis=0).tolist()})
    return faces


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--step', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    result = inspect(a.step)
    Path(a.output).write_text(json.dumps(result, indent=2), encoding='utf8')
    print(json.dumps([{k: v for k, v in r.items() if k != 'vertices_mm'}
                      for r in result if r['center_mm'][2] < -455 and max(r['span_mm']) > 8], indent=2))
