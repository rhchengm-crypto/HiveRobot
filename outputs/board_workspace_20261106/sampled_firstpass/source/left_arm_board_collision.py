"""NumPy-only, frame-explicit body obstacles for the common board planner.

Capsules use a conservative half-space clearance to oriented boxes. This
overestimates collisions near box corners; it never certifies a real chassis
from an unregistered mesh. No serial imports or controller state changes.
"""
from __future__ import annotations

import numpy as np

from left_arm_rigid_body import vector


def segment_box_clearance(start, end, center, axes, half_extents):
    """Lower bound on segment-to-box signed distance, in metres.

    Minimize the maximum of the six normalized box-plane distances along the
    complete segment. Endpoints and pairwise line crossings contain the exact
    minimum of this convex piecewise-linear function (no point sampling).
    """
    a = (np.asarray(start)-center) @ axes
    b = (np.asarray(end)-center) @ axes
    intercept = np.r_[a-half_extents, -a-half_extents]
    slope = np.r_[b-a, a-b]
    i, j = np.triu_indices(6, 1)
    denominator = slope[i]-slope[j]
    valid = np.abs(denominator) > 1e-14
    crossings = (intercept[j[valid]]-intercept[i[valid]])/denominator[valid]
    ts = np.r_[0., 1., crossings[(crossings > 0) & (crossings < 1)]]
    values = np.max(intercept[:, None]+slope[:, None]*ts, axis=0)
    best = int(np.argmin(values))
    return float(values[best]), float(ts[best])


class BodyCollisionScene:
    def __init__(self, model, cfg):
        spec = cfg['body_collision']
        if spec.get('schema') != 1 or spec.get('frame') != 'cad_base':
            raise ValueError('body collision requires schema 1 and explicit cad_base frame')
        self.model = model
        self.spec = spec
        self.tcp = vector(cfg['tcp_in_wrist_frame_m'], 3, 'TCP')
        self.obstacles = []
        names = set()
        for row in spec['obstacles']:
            if not row.get('name') or row['name'] in names:
                raise ValueError('unique body obstacle names required')
            names.add(row['name'])
            center = vector(row['center_m'], 3)
            axes = np.asarray(row['axes_in_cad'], dtype=float)
            half = vector(row['half_extents_m'], 3)
            if (axes.shape != (3, 3) or not np.isfinite(axes).all()
                    or not np.allclose(axes.T@axes, np.eye(3), atol=1e-8)
                    or not np.isclose(np.linalg.det(axes), 1, atol=1e-8)
                    or np.any(half <= 0)):
                raise ValueError('body obstacle needs a proper rotation and positive extents')
            self.obstacles.append((row['name'], center, axes, half))
        self.segments = []
        for row in spec['capsules']:
            # Index n is the TCP, indices 0..n-1 are the FK joint origins.
            start, end = int(row['start_index']), int(row['end_index'])
            radius, margin = float(row['radius_m']), float(row['margin_m'])
            if (not 0 <= start < end <= model.n or not np.isfinite([radius, margin]).all()
                    or radius <= 0 or margin < 0):
                raise ValueError('invalid body-check capsule')
            self.segments.append((str(row['name']), start, end, radius, margin))
        if not self.obstacles or not self.segments:
            raise ValueError('body scene must contain obstacles and capsules')

    def evaluate(self, encoder, details=False):
        frames = self.model.fk(self.model.encoder_to_model(encoder))
        tcp = frames[-1, :3, 3]+frames[-1, :3, :3]@self.tcp
        points = np.vstack([frames[:, :3, 3], tcp])
        rows, values = [], []
        for name, start, end, radius, margin in self.segments:
            for obstacle, center, axes, half in self.obstacles:
                gap, fraction = segment_box_clearance(points[start], points[end], center, axes, half)
                clearance = gap-radius-margin
                values.append(clearance)
                if details:
                    rows.append(dict(capsule=name, obstacle=obstacle,
                                     clearance_lower_bound_mm=clearance*1000,
                                     centerline_fraction=fraction, radius_mm=radius*1000,
                                     margin_mm=margin*1000))
        return rows if details else np.asarray(values)

    def __call__(self, encoder):
        return self.evaluate(encoder)

    def jacobian(self, encoder):
        """Envelope subgradient using the active witness box planes.

        All point Jacobians share one FK evaluation. This avoids fourteen FK
        evaluations per seven-axis optimization step on the Jetson.
        """
        frames = self.model.fk(self.model.encoder_to_model(encoder))
        origins = frames[:, :3, 3]
        points = np.vstack([origins, origins[-1]+frames[-1, :3, :3]@self.tcp])
        axes = frames[:, :3, 2]
        jpoints = np.cross(axes[None, :, :], points[:, None, :]-origins[None, :, :])
        jpoints *= self.model.signs[None, :, None]
        jpoints[np.arange(self.model.n)[None, :] > np.arange(self.model.n+1)[:, None]] = 0.
        gradients = []
        for _, start, end, _, _ in self.segments:
            for _, center, rotation, half in self.obstacles:
                gap, t = segment_box_clearance(points[start], points[end], center, rotation, half)
                a, b = (points[start]-center)@rotation, (points[end]-center)@rotation
                normals = np.vstack([rotation.T, -rotation.T])
                slopes = np.r_[b-a, a-b]
                values = np.r_[a-half, -a-half]+t*slopes
                active = np.flatnonzero(values >= gap-1e-9)
                weights = np.zeros(6)
                negative = [i for i in active if slopes[i] < -1e-12]
                positive = [i for i in active if slopes[i] > 1e-12]
                if 1e-9 < t < 1-1e-9 and negative and positive:
                    i, j = negative[0], positive[0]
                    weights[i] = slopes[j]/(slopes[j]-slopes[i])
                    weights[j] = -slopes[i]/(slopes[j]-slopes[i])
                else:
                    weights[active] = 1/len(active)
                normal = weights@normals
                gradients.append(((1-t)*jpoints[start]+t*jpoints[end])@normal)
        return np.asarray(gradients)


def collision_scene(model, cfg):
    """Cache derived numeric objects outside the serializable configuration."""
    spec = cfg.get('body_collision')
    if spec is None or not spec.get('enabled', False):
        return None
    # Configs are immutable during a plan; avoid modifying/hash-invalidating cfg.
    cached = getattr(model, '_board_body_scene_cache', None)
    if cached is not None and cached[0] is spec:
        return cached[1]
    scene = BodyCollisionScene(model, cfg)
    model._board_body_scene_cache = (spec, scene)
    return scene
