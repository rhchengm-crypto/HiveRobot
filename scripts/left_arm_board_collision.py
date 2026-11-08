"""NumPy-only, frame-explicit body obstacles for the common board planner.

Capsules use exact Euclidean separation from oriented boxes outside the box,
and a signed half-space lower bound for intersecting centrelines. Geometry
registration remains independent of the distance calculation. No serial imports or controller state changes.
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


def segment_box_distance(start, end, center, axes, half_extents):
    """Exact segment/solid-box separation outside; signed bound inside.

    Box-plane crossings partition the segment into intervals with constant
    active coordinates. Squared Euclidean distance is quadratic on each;
    its endpoints and stationary point contain the global minimum.
    """
    a = (np.asarray(start)-center) @ axes
    delta = (np.asarray(end)-np.asarray(start)) @ axes
    half = np.asarray(half_extents)
    cuts = [0., 1.]
    for j in range(3):
        if abs(delta[j]) > 1e-14:
            for side in (-1., 1.):
                crossing = (side*half[j]-a[j])/delta[j]
                if 0. < crossing < 1.:
                    cuts.append(float(crossing))
    cuts = np.unique(cuts)
    candidates = list(cuts)
    for lo, hi in zip(cuts[:-1], cuts[1:]):
        middle = a+delta*((lo+hi)/2)
        active = np.abs(middle) > half
        velocity = delta[active]
        denominator = velocity@velocity
        if denominator > 1e-28:
            offset = a[active]-np.sign(middle[active])*half[active]
            candidates.append(float(np.clip(-(offset@velocity)/denominator, lo, hi)))
    candidates = np.asarray(candidates)
    excess = np.maximum(np.abs(a[None, :]+candidates[:, None]*delta)-half, 0.)
    squared = np.einsum('ij,ij->i', excess, excess)
    best = int(np.argmin(squared))
    if squared[best] <= 1e-24:
        return segment_box_clearance(start, end, center, axes, half_extents)
    return float(np.sqrt(squared[best])), float(candidates[best])


class BodyCollisionScene:
    def __init__(self, model, cfg):
        spec = cfg['body_collision']
        if spec.get('schema') != 1 or spec.get('frame') != 'cad_base':
            raise ValueError('body collision requires schema 1 and explicit cad_base frame')
        self.model = model
        self.spec = spec
        self.tcp = vector(cfg['tcp_in_wrist_frame_m'], 3, 'TCP')
        self.surface = cfg.get('support_surface', {})
        self.tool_envelope = cfg.get('gripper_vertex_envelope')
        self.tool_vertices = None
        if self.tool_envelope is not None:
            envelope = self.tool_envelope
            vertices = np.asarray(envelope.get('vertices_in_wrist_frame_m'), dtype=float)
            height = float(cfg.get('_gripper_min_height_mm', envelope.get('minimum_height_mm', 5.)))
            if (envelope.get('schema') != 1 or envelope.get('frame') != 'wrist'
                    or vertices.ndim != 2 or vertices.shape[1] != 3 or not 4 <= len(vertices) <= 2048
                    or not np.isfinite(vertices).all() or not np.isfinite(height) or height < 0.):
                raise ValueError('finite wrist-frame gripper envelope and nonnegative clearance required')
            if not self.surface.get('enabled'):
                raise ValueError('gripper envelope requires the registered support surface')
            self.tool_vertices = vertices
            self.tool_min_height = height/1000.
        if self.surface.get('enabled'):
            if self.surface.get('policy') != 'global-board-height-halfspace':
                raise ValueError('unsupported support surface policy')
            self.surface_normal = np.asarray(cfg['board_to_cad_rotation'])[:, 2]
            self.surface_origin = vector(cfg['board_to_cad_translation_m'], 3)
            self.surface_margins = np.r_[np.full(model.n-1, .035), .005]
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
                gap, fraction = segment_box_distance(points[start], points[end], center, axes, half)
                clearance = gap-radius-margin
                values.append(clearance)
                if details:
                    rows.append(dict(capsule=name, obstacle=obstacle,
                                     clearance_lower_bound_mm=clearance*1000,
                                     centerline_fraction=fraction, radius_mm=radius*1000,
                                     margin_mm=margin*1000))
        if self.surface.get('enabled'):
            heights = (points-self.surface_origin)@self.surface_normal
            for i, margin in enumerate(self.surface_margins):
                witness = i if heights[i] <= heights[i+1] else i+1
                gap = float(heights[witness]-margin)
                values.append(gap)
                if details:
                    rows.append(dict(code='tool_table_intersection' if i==self.model.n-1 else 'link_table_intersection',
                        capsule=self.model.names[i]+'-to-'+('tcp' if i==self.model.n-1 else self.model.names[i+1]),
                        obstacle='global-board-height-halfspace', clearance_lower_bound_mm=gap*1000,
                        minimum_height_mm=margin*1000, endpoint_index=witness))
        if self.tool_vertices is not None:
            vertices = self.tool_vertices@frames[-1, :3, :3].T+frames[-1, :3, 3]
            heights = (vertices-self.surface_origin)@self.surface_normal
            witness = int(np.argmin(heights))
            gap = float(heights[witness]-self.tool_min_height)
            values.append(gap)
            if details:
                rows.append(dict(code='gripper_surface_clearance', capsule='complete-CAD-gripper-envelope',
                    obstacle='global-board-height-halfspace', clearance_lower_bound_mm=gap*1000,
                    lowest_vertex_height_mm=float(heights[witness])*1000,
                    minimum_height_mm=self.tool_min_height*1000, vertex_index=witness))
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
                gap, t = segment_box_distance(points[start], points[end], center, rotation, half)
                a, b = (points[start]-center)@rotation, (points[end]-center)@rotation
                witness = (1-t)*a+t*b
                outside = witness-np.clip(witness, -half, half)
                distance = np.linalg.norm(outside)
                if distance > 1e-12:
                    normal = rotation@(outside/distance)
                    gradients.append(((1-t)*jpoints[start]+t*jpoints[end])@normal)
                    continue
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
        if self.surface.get('enabled'):
            heights = (points-self.surface_origin)@self.surface_normal
            for i in range(self.model.n):
                witness = i if heights[i] <= heights[i+1] else i+1
                gradients.append(jpoints[witness]@self.surface_normal)
        if self.tool_vertices is not None:
            vertices = self.tool_vertices@frames[-1, :3, :3].T+frames[-1, :3, 3]
            witness = vertices[np.argmin((vertices-self.surface_origin)@self.surface_normal)]
            jacobian = np.cross(axes, witness-origins)*self.model.signs[:, None]
            gradients.append(jacobian@self.surface_normal)
        return np.asarray(gradients)


def collision_scene(model, cfg):
    """Cache derived numeric objects outside the serializable configuration."""
    spec = cfg.get('body_collision')
    if spec is None or not spec.get('enabled', False):
        if cfg.get('gripper_vertex_envelope') is not None:
            raise ValueError('gripper envelope requires an enabled collision scene')
        return None
    # Configs are immutable during a plan; avoid modifying/hash-invalidating cfg.
    cached = getattr(model, '_board_body_scene_cache', None)
    surface = cfg.get('support_surface')
    envelope = cfg.get('gripper_vertex_envelope')
    height = cfg.get('_gripper_min_height_mm')
    if (cached is not None and len(cached)==5 and cached[0] is spec and cached[1] is surface
            and cached[2] is envelope and cached[3] == height):
        return cached[4]
    scene = BodyCollisionScene(model, cfg)
    model._board_body_scene_cache = (spec, surface, envelope, height, scene)
    return scene


def board_prism_clearances(model, cfg, encoder, details=False):
    """Whole link/tool segments vs the declared finite-board prisms.

    The -10m bottom is below every point of this sub-metre serial arm. Thus
    it represents the existing forbidden space below the playing surface,
    rather than pretending the arm may pass underneath the board.
    """
    frames = model.fk(model.encoder_to_model(encoder))
    r, t = np.asarray(cfg['board_to_cad_rotation']), np.asarray(cfg['board_to_cad_translation_m'])
    points = (frames[:, :3, 3]-t)@r
    tcp = (frames[-1, :3, 3]+frames[-1, :3, :3]@np.asarray(cfg['tcp_in_wrist_frame_m'])-t)@r
    values, rows = [], []
    for i in range(model.n):
        tool = i == model.n-1
        a, b = points[i], tcp if tool else points[i+1]
        lateral, height = (.02, .005) if tool else (.03, .03)
        center = np.array([.22, .22, (height-10.)/2])
        half = np.array([.22+lateral, .22+lateral, (height+10.)/2])
        gap, fraction = segment_box_distance(a, b, center, np.eye(3), half)
        values.append(gap)
        if details:
            rows.append(dict(code='tool_board_intersection' if tool else 'link_board_intersection',
                             link_from=model.names[i], link_to='tcp' if tool else model.names[i+1],
                             link_fraction=fraction, board_point_mm=((1-fraction)*a+fraction*b).tolist(),
                             clearance_lower_bound_mm=gap*1000, minimum_height_mm=height*1000))
            rows[-1]['board_point_mm'] = (np.asarray(rows[-1]['board_point_mm'])*1000).tolist()
    return rows if details else np.asarray(values)
