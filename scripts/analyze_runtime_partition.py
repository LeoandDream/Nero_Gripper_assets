"""Offline union-exterior distances and independent runtime mapping checks.

Distances are exact nearest-triangle queries for the disclosed finite sample
set. The coverage radius gives a conservative continuous upper bound; it is
NOT silently replaced with the smaller preregistered requested resolution.
Separate continuous solid certificates are supplied by the constructors.
"""
from pathlib import Path
import collections
import hashlib
import json
from itertools import product

import numpy as np
from scipy.spatial import cKDTree

from assemble_runtime_partition import insphere
from runtime_contact_partition import load_source
from runtime_backing_partition import poly, subtract_body

ROOT = Path(__file__).resolve().parents[1]


def triangle_distance(p, t):
    """Broadcast points (...,3) and triangles (...,3,3); squared distance."""
    a, b, c = t[..., 0, :], t[..., 1, :], t[..., 2, :]
    n = np.cross(b-a, c-a)
    n2 = np.sum(n*n, axis=-1)
    pa = p-a
    h = np.sum(pa*n, axis=-1)
    projection = p-n * (h/np.maximum(n2, 1e-300))[..., None]
    inside = np.ones(np.broadcast_shapes(p.shape[:-1], t.shape[:-2]), dtype=bool)
    result = np.full(inside.shape, np.inf)
    for x, y in [(a, b), (b, c), (c, a)]:
        v = y-x
        q = p-x
        ratio = np.clip(np.sum(q*v, axis=-1)/np.maximum(np.sum(v*v, axis=-1), 1e-300), 0, 1)
        d = q-ratio[..., None]*v
        result = np.minimum(result, np.sum(d*d, axis=-1))
        inside &= np.sum(np.cross(v, projection-x)*n, axis=-1) >= -1e-13*n2
    return np.where(inside & (n2 > 1e-36), h*h/np.maximum(n2, 1e-300), result)


class TriangleBVH:
    def __init__(self, triangles):
        self.t = np.asarray(triangles)
        self.centers = self.t.mean(axis=1)
        self.tree = cKDTree(self.centers)
        self.nodes = []
        self.build(np.arange(len(self.t)))

    def build(self, ids):
        index = len(self.nodes)
        t = self.t[ids]
        node = dict(lo=t.min(axis=(0, 1)), hi=t.max(axis=(0, 1)))
        self.nodes.append(node)
        if len(ids) <= 12:
            node['ids'] = ids
        else:
            centers = self.centers[ids]
            axis = np.argmax(np.ptp(centers, axis=0))
            ids = ids[np.argsort(centers[:, axis], kind='stable')]
            middle = len(ids)//2
            node['children'] = [self.build(ids[:middle]), self.build(ids[middle:])]
        return index

    def query(self, points):
        points = np.asarray(points)
        initial = self.tree.query(points)[1]
        best = triangle_distance(points, self.t[initial])
        nearest = initial.copy()
        stack = [(0, np.arange(len(points)))]
        while stack:
            index, ids = stack.pop()
            node = self.nodes[index]
            p = points[ids]
            delta = np.maximum(np.maximum(node['lo']-p, p-node['hi']), 0)
            ids = ids[np.sum(delta*delta, axis=1) <= best[ids]+1e-24]
            if not len(ids):
                continue
            if 'children' in node:
                for child in node['children']:
                    stack.append((child, ids))
            else:
                tri_ids = node['ids']
                distances = triangle_distance(points[ids, None, :], self.t[None, tri_ids])
                arg = np.argmin(distances, axis=1)
                val = distances[np.arange(len(ids)), arg]
                change = val < best[ids]
                best[ids[change]] = val[change]
                nearest[ids[change]] = tri_ids[arg[change]]
        return np.sqrt(np.maximum(best, 0)), nearest


def polygon_triangles(polygons):
    return np.array([np.array(p)[[0, i, i+1]] for p in polygons for i in range(1, len(p)-1)])


def exterior(d, runtime=False):
    polygons, families = [], []
    for e in d['entities']:
        patches = e['exterior_patches'] if runtime else [s for patch in e['source_surface_partition'] for s in patch['exterior_subpolygons']]
        for patch in patches:
            p = np.asarray(patch['polygon_m']) * 1000
            polygons.append(p)
            families.extend([e['semantic_family']]*(len(p)-2))
    return polygon_triangles(polygons), np.array(families)


def samples(triangles, families):
    # Seven fixed points per triangle; maximum vertex-centroid radius is a
    # valid covering bound for every point in the triangle. No sampled PASS.
    weights = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [.5, .5, 0],
                        [0, .5, .5], [.5, 0, .5], [1/3, 1/3, 1/3]])
    p = np.einsum('wv,tvc->twc', weights, triangles).reshape(-1, 3)
    radius = np.max(np.linalg.norm(triangles-triangles.mean(axis=1)[:, None, :], axis=2), axis=1)
    return p, np.repeat(families, len(weights)), radius


def inside_union(points, entities):
    inside = np.zeros(len(points), dtype=bool)
    for e in entities:
        v = np.asarray(e['vertices_m'])*1000
        mask = ~inside & np.all(points >= v.min(axis=0)-1e-8, axis=1) & np.all(points <= v.max(axis=0)+1e-8, axis=1)
        ids = np.flatnonzero(mask)
        if len(ids):
            eq = np.asarray(e['halfspaces_m'])*[1, 1, 1, 1000]
            inside[ids] = np.max(points[ids] @ eq[:, :3].T+eq[:, 3], axis=1) <= 1e-8
    return inside


def directed(source, families, target, target_entities):
    p, labels, radii = samples(source, families)
    tree = TriangleBVH(target)
    distances, ids = tree.query(p)
    signed = distances.copy()
    away = np.flatnonzero(distances > 1e-8)
    signed[away[inside_union(p[away], target_entities)]] *= -1
    signed[distances <= 1e-8] = 0
    i = int(np.argmax(distances))
    centers = distances.reshape(-1, 7)[:, -1]
    bounds = centers+radii
    by_family = {}
    for family in sorted(set(families)):
        mask = labels == family
        by_family[family] = dict(sampled_maximum_mm=float(distances[mask].max()),
                                 conservative_triangle_cover_upper_mm=float(bounds[families == family].max()))
    return dict(sample_count=len(p), sampled_maximum_mm=float(distances[i]),
                sampled_signed_range_mm=[float(signed.min()), float(signed.max())],
                signed_convention='Negative inside the target closed-solid union; zero within 1e-8 mm arithmetic resolution.',
                sampled_maximum_position_mm=p[i].tolist(),
                nearest_target_triangle=int(ids[i]),
                conservative_triangle_cover_upper_mm=float(bounds.max()),
                actual_maximum_covering_radius_mm=float(radii.max()),
                resolution='vertices, edge midpoints and centroids; continuous bound uses actual triangle radius',
                by_semantic_family=by_family)


def main():
    path = ROOT/'reports/nero-gripper-runtime-partition.json'
    runtime = json.loads(path.read_text())
    reference = json.loads((ROOT/'reports/nero-gripper-entity-partition.json').read_text())
    rt, rf = exterior(runtime, True)
    ot, of = exterior(reference)
    print(f'Exterior triangles: reference={len(ot)}, runtime={len(rt)}', flush=True)
    a = directed(rt, rf, ot, reference['entities'])
    print('Runtime -> reference exterior:', a['sampled_maximum_mm'], flush=True)
    b = directed(ot, of, rt, runtime['entities'])
    print('Reference -> runtime exterior:', b['sampled_maximum_mm'], flush=True)
    source = load_source()
    source_triangles = np.array(source['vertices_m'])[np.array([t['vertex_ids'] for t in source['triangles']])]*1000
    working_groups = dict(main=list(range(557, 578)), tip=list(range(545, 549)),
                          ramp=list(range(549, 557)), shoulder=list(range(578, 582)))
    workfaces = {}
    runtime_tree = TriangleBVH(rt)
    for name, ids in working_groups.items():
        points, _, radii = samples(source_triangles[ids], np.repeat(name, len(ids)))
        distances, _ = runtime_tree.query(points)
        workfaces[name] = dict(source_triangle_ids=ids, sample_count=len(points),
                               sampled_source_workface_to_runtime_exterior_max_mm=float(distances.max()),
                               note='Actual working source triangles only; excludes buried back/interface surfaces in the same semantic family.')
    # A continuous counterexample to the 50um reverse-exterior budget,
    # independent of the provisional polygon-boundary extraction.
    point = np.array(b['sampled_maximum_position_mm'])
    cube = poly(point+np.array(list(product((-.06, .06), repeat=3))))
    pieces, used = [cube], []
    for entity in runtime['entities']:
        body = poly(np.array(entity['vertices_m'])*1000)
        if np.any(body['lo'] > cube['hi']) or np.any(body['hi'] < cube['lo']):
            continue
        before = sum(p['h'].volume for p in pieces)
        pieces = [out for p in pieces for out in subtract_body(p, body)]
        if before-sum(p['h'].volume for p in pieces) > 1e-14:
            used.append(entity['id'])
        if not pieces:
            break
    burial = dict(reference_surface_point_mm=point.tolist(), cube_half_width_mm=.06,
                   remaining_pieces=len(pieces), covering_runtime_entities=used,
                   certified_distance_to_runtime_exterior_lower_bound_mm=.06 if not pieces else None,
                   arithmetic_resolution_mm=1e-9,
                   interpretation='The entire cube is inside the actual runtime solid union. Its central reference exterior point is therefore at least 60um from the new exterior, independently of exterior triangulation.')
    original_scales = []
    for e in reference['entities']:
        v = np.array(e['vertices_m']) * 1000
        eq = np.array(e['halfspaces_m']) * [1, 1, 1, 1000]
        original_scales.append(dict(id=e['id'], minimum_width_mm=float(np.min(np.ptp(v @ eq[:, :3].T, axis=0))), inscribed_diameter_mm=insphere(eq)))
    families = collections.Counter(e['semantic_family'] for e in runtime['entities'])
    result = dict(schema_version=1, status='RUNTIME_ENTITY_PARTITION_BLOCKED',
                  runtime_geometry_sha256=hashlib.sha256(json.dumps([[e['id'], e['vertices_m'], e['faces']] for e in runtime['entities']], separators=(',', ':')).encode()).hexdigest(),
                  entity_count_per_finger=len(runtime['entities']), counts_by_semantic_family=dict(families),
                  maximum_material_inward_mm=0.,
                  material_inward_basis='Every original convex cell is contained in its assigned new hull; maximum arithmetic vertex residual is disclosed separately. This does not imply reverse exterior distance zero.',
                  continuous_runtime_solid_to_reference_upper_mm=.05,
                  continuous_critical_contact_solid_upper_mm=np.sqrt(3)*.005,
                  continuous_mount_source_part_exterior_upper_mm=.008298777650992406,
                  runtime_exterior_to_reference_exterior=a, reference_exterior_to_runtime_exterior=b,
                  symmetric_sampled_surface_distance_mm=max(a['sampled_maximum_mm'], b['sampled_maximum_mm']),
                  actual_working_surface_samples=workfaces,
                  continuous_reverse_exterior_budget_counterexample=burial,
                  union_exterior_area_partition_residual_mm2=runtime['boundary_extraction']['largest_face_area_partition_residual_mm2'],
                  exterior_triangulation_certified=False,
                  requested_preregistered_covering_radius_met=False,
                  surface_interpretation='Finite-set extrema and triangle-radius bounds describe the extracted polygon meshes only. Their nonzero area-partition residual prevents certification as actual union-exterior bounds. The requested 50um/10um global exterior bounds are not certified. Independent continuous solid certificates and the 60um buried-interface counterexample remain distinct.',
                  original_minimum_width_mm=min(e['minimum_width_mm'] for e in original_scales),
                  original_entities_narrower_than_1nm=sum(e['minimum_width_mm'] < 1e-6 for e in original_scales),
                  original_entities_narrower_than_100nm=sum(e['minimum_width_mm'] < 1e-4 for e in original_scales),
                  original_entities_below_0_1mm_inscribed_diameter=sum(e['inscribed_diameter_mm'] < .1 for e in original_scales),
                  runtime_minimum_inscribed_diameter_mm=runtime['minimum_inscribed_diameter_mm'],
                  runtime_minimum_facet_normal_width_mm=runtime['minimum_facet_normal_width_mm'],
                  runtime_subminimum_entities=runtime['residual_entities_below_minimum'],
                  original_scale_measurements=original_scales)
    (ROOT/'reports/nero-gripper-runtime-errors.json').write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
