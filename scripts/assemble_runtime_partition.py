"""Assemble the single Issue #25 runtime candidate; pure offline geometry only.

Run with Python 3.11, numpy 1.26.4, scipy 1.15.3. The frozen reference
JSON remains an oracle, never an active simulation asset. This script extracts
the *new* union exterior; inherited source patches are provenance, not assumed
to be the new exterior. No simulator or MJCF writer is imported.
"""
from __future__ import annotations

import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
EPS = 1e-10  # mm, arithmetic classification only, never a geometry offset
AREA_EPS = 1e-17  # mm^2, disclosed arithmetic residue
FAMILIES = ['distal_tip_contact', 'inner_main_contact_plate',
            'inner_ramp_contact', 'inner_transition_contact',
            'sidewall_negative_x', 'sidewall_positive_x',
            'outer_chassis', 'mount_root']


def read(p):
    return json.loads(Path(p).read_text())


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def area(p):
    p = np.asarray(p)
    if len(p) < 3:
        return 0.0
    return float(np.linalg.norm(np.cross(p, np.roll(p, -1, axis=0)).sum(axis=0)) / 2)


def clip(p, plane, inside=True):
    """Sutherland-Hodgman on a convex planar polygon, coordinates in mm."""
    p = np.asarray(p, dtype=float)
    if len(p) < 3:
        return np.empty((0, 3))
    d = p @ plane[:3] + plane[3]
    if np.max(d) <= EPS:
        return p.copy() if inside else np.empty((0, 3))
    if np.min(d) > EPS:
        return np.empty((0, 3)) if inside else p.copy()
    if not inside and np.max(d) <= EPS:
        # A coplanar polygon belongs to the closed inside halfspace only;
        # returning it on both sides would duplicate positive area.
        return np.empty((0, 3))
    if not inside:
        d = -d
    d[np.abs(d) <= EPS] = 0.0
    out = []
    for i in range(len(p)):
        j = (i + 1) % len(p)
        if d[i] <= 0:
            out.append(p[i])
        if (d[i] < 0 < d[j]) or (d[j] < 0 < d[i]):
            out.append(p[i] + (p[j] - p[i]) * d[i] / (d[i] - d[j]))
    if len(out) < 3:
        return np.empty((0, 3))
    out = np.asarray(out)
    keep = np.linalg.norm(out - np.roll(out, 1, axis=0), axis=1) > EPS * .01
    out = out[keep]
    return out if area(out) > AREA_EPS else np.empty((0, 3))


def subtract(p, planes):
    """Disjoint outside pieces and the covered inside patch of p."""
    remainder = np.asarray(p)
    outside = []
    for plane in planes:
        if len(remainder) < 3:
            break
        q = clip(remainder, plane, False)
        if len(q) >= 3:
            outside.append(q)
        remainder = clip(remainder, plane, True)
    return outside, remainder


def insphere(equations):
    result = linprog([0., 0., 0., -1.],
                     A_ub=np.column_stack((equations[:, :3], np.ones(len(equations)))),
                     b_ub=-equations[:, 3],
                     bounds=[(None, None)] * 3 + [(0, None)], method='highs')
    if not result.success:
        raise ValueError('Insphere LP failed: ' + result.message)
    return float(2 * result.x[3])


def orient_hull(v):
    hull = ConvexHull(v)
    faces = hull.simplices.copy()
    for i, ids in enumerate(faces):
        a, b, c = v[ids]
        if np.dot(np.cross(b-a, c-a), hull.equations[i, :3]) < 0:
            faces[i] = ids[[0, 2, 1]]
    return hull, faces


def extract_exterior(entities):
    """Actual union boundary, retaining explicit convex-neighbour coverage.

    Coplanar, same-facing exterior overlap has one deterministic owner. An
    opposite-facing shared face is a closure for both cells. Positive-volume
    overlaps are not confused with source-independent exterior surfaces.
    """
    vs = [np.asarray(e['vertices_m']) * 1000 for e in entities]
    planes = [np.asarray(e['halfspaces_m']) * [1, 1, 1, 1000] for e in entities]
    lo = np.array([v.min(axis=0) for v in vs])
    hi = np.array([v.max(axis=0) for v in vs])
    summary = dict(exterior_polygon_count=0, covered_polygon_count=0,
                   largest_face_area_partition_residual_mm2=0.,
                   arithmetic_epsilon_mm=EPS, area_prune_mm2=AREA_EPS)
    for i, e in enumerate(entities):
        neighbours = np.flatnonzero(np.all(hi >= lo[i]-EPS, axis=1) &
                                    np.all(lo <= hi[i]+EPS, axis=1))
        external, covered, adj = [], [], set()
        for fi, ids in enumerate(e['faces']):
            face = vs[i][ids]
            normal = np.cross(face[1]-face[0], face[2]-face[0])
            norm = np.linalg.norm(normal)
            if not norm:
                raise ValueError('Zero-area runtime hull face')
            normal /= norm
            offset = -float(normal @ face[0])
            pieces = [face]
            own_covered = []
            for j in neighbours:
                if j == i or not pieces:
                    continue
                if np.any(face.max(axis=0) < lo[j]-EPS) or np.any(face.min(axis=0) > hi[j]+EPS):
                    continue
                if np.any(np.min(face @ planes[j][:, :3].T + planes[j][:, 3], axis=0) > EPS):
                    continue
                # A same-oriented shared exterior is owned by the lower ID.
                coplanar = (np.linalg.norm(planes[j][:, :3] - normal, axis=1) < 1e-10) & (np.abs(planes[j][:, 3] - offset) < EPS)
                same_exterior = bool(coplanar.any())
                if same_exterior and i < j:
                    continue
                new = []
                for piece in pieces:
                    outside, inside = subtract(piece, planes[j])
                    new.extend(outside)
                    if len(inside) >= 3 and area(inside) > AREA_EPS:
                        adj.add(entities[j]['id'])
                        record = dict(entity_face=fi, neighbor=entities[j]['id'],
                                      kind='coplanar_exterior_ownership' if same_exterior else 'internal_union_coverage',
                                      area_mm2=area(inside), polygon_m=(inside / 1000).tolist())
                        own_covered.append(record)
                pieces = new
                if len(pieces) > 10000:
                    raise ValueError('Boundary clipping complexity limit; candidate is not certified')
            ext_area = sum(area(p) for p in pieces)
            residual = abs(area(face)-ext_area-sum(x['area_mm2'] for x in own_covered))
            summary['largest_face_area_partition_residual_mm2'] = max(residual, summary['largest_face_area_partition_residual_mm2'])
            for p in pieces:
                external.append(dict(entity_face=fi, kind='runtime_union_exterior',
                                     polygon_m=(p/1000).tolist(), area_mm2=area(p)))
            covered.extend(own_covered)
        e['exterior_patches'] = external
        e['closure_and_ownership_patches'] = covered
        e['neighbors'] = sorted(adj)
        summary['exterior_polygon_count'] += len(external)
        summary['covered_polygon_count'] += len(covered)
        if (i + 1) % 25 == 0:
            print(f'Extracted runtime union boundary {i+1}/{len(entities)}', flush=True)
    # Coverage may be directed due to exterior ownership; neighbour graph is symmetric.
    by_id = {e['id']: e for e in entities}
    for e in entities:
        for other in e['neighbors'][:]:
            by_id[other]['neighbors'] = sorted(set(by_id[other]['neighbors']) | {e['id']})
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parts', nargs=3)
    parser.add_argument('--finalize', action='store_true', help='Attach the existing single-candidate error evidence; do not reconstruct geometry.')
    args = parser.parse_args()
    if args.finalize:
        target = ROOT/'reports/nero-gripper-runtime-partition.json'
        d = read(target)
        errors_path = ROOT/'reports/nero-gripper-runtime-errors.json'
        errors = read(errors_path)
        geometry_sha = hashlib.sha256(json.dumps([[e['id'], e['vertices_m'], e['faces']] for e in d['entities']], separators=(',', ':')).encode()).hexdigest()
        assert errors['runtime_geometry_sha256'] == geometry_sha
        d['geometry_sha256'] = geometry_sha
        replaced = {'global_exterior_distance_and_protected_features', 'reverse_exterior_budget_exceeded', 'union_exterior_area_partition_not_certified', 'preregistered_surface_cover_resolution_not_verified'}
        d['blocking_findings'] = [x for x in d['blocking_findings'] if x['check'] not in replaced]
        d['blocking_findings'].extend([
            dict(check='reverse_exterior_budget_exceeded', sampled_mm=errors['reference_exterior_to_runtime_exterior']['sampled_maximum_mm'], independent_continuous_lower_bound_mm=errors['continuous_reverse_exterior_budget_counterexample']['certified_distance_to_runtime_exterior_lower_bound_mm'], budget_mm=.05),
            dict(check='union_exterior_area_partition_not_certified', maximum_residual_mm2=d['boundary_extraction']['largest_face_area_partition_residual_mm2']),
            dict(check='preregistered_surface_cover_resolution_not_verified', required_mm=dict(ordinary=.005, critical=.001))])
        d['global_error_report'] = dict(path=str(errors_path.relative_to(ROOT)), sha256=sha(errors_path))
        d['status'] = 'RUNTIME_ENTITY_PARTITION_BLOCKED'
        d['boundary_extraction']['certified'] = False
        d['boundary_extraction']['interpretation'] = 'Provisional union polygon ownership. Nonzero area residual is unresolved; do not treat these polygons as certified engine input. Solid certificates and the independent buried-interface cube witness do not depend on this extraction.'
        d['simulation_asset_ready'] = False
        target.write_text(json.dumps(d, separators=(',', ':'), ensure_ascii=False)+'\n')
        print(d['status'], geometry_sha)
        return
    if not args.parts:
        parser.error('--parts is required unless --finalize is supplied')
    reference_path = ROOT/'reports/nero-gripper-entity-partition.json'
    protocol_path = ROOT/'reports/nero-gripper-runtime-preregistration.json'
    reference, protocol = read(reference_path), read(protocol_path)
    assert sha(reference_path) == protocol['reference_sha256']
    old = {e['id']: e for e in reference['entities']}
    parts = [read(ROOT / p) for p in args.parts]
    entities, local_verification = [], []
    for path, part in zip(args.parts, parts):
        local_verification.append(dict(path=path, sha256=sha(ROOT/path),
                                       verification=part.get('verification'),
                                       construction=part.get('construction')))
        entities.extend(part['entities'])
    entities.sort(key=lambda e: min(int(x[1:]) for x in e['reference_entity_ids']))
    seen = []
    for i, e in enumerate(entities, 1):
        e['id'] = f'R{i:04d}'
        refs = e['reference_entity_ids']
        seen.extend(refs)
        originals = [old[x] for x in refs]
        e['name'] = f"{e['semantic_family']}_{i:04d}"
        e['source_part'] = originals[0]['source_part']
        assert all(x['source_part'] == e['source_part'] for x in originals)
        e['source_triangle_ids'] = sorted({x for r in originals for x in r['source_triangle_ids']})
        e['source_regions'] = sorted({x for r in originals for x in r['source_regions']})
        e['semantic_feature'] = e.get('semantic_feature', e.get('feature', e['semantic_family']))
        e['reference_boundary_provenance'] = [f'nero-gripper-entity-partition.json#/entities/{int(x[1:])-1}/source_surface_partition' for x in refs]
        v = np.asarray(e['vertices_m']) * 1000
        hull = ConvexHull(v)
        faces = np.asarray([f['vertex_indices'] if isinstance(f, dict) else f for f in e['faces']], dtype=int)
        e['faces'] = faces.tolist()
        equations = np.asarray(e['halfspaces_m']) * [1, 1, 1, 1000]
        e['primitive_type'] = 'closed_convex_polyhedron'
        e['volume_mm3'] = float(hull.volume)
        e['inscribed_diameter_mm'] = insphere(equations)
        e['minimum_facet_normal_width_mm'] = float(np.min(np.ptp(v @ equations[:, :3].T, axis=0)))
        e['reference_containment_max_violation_mm'] = max(float(np.max(np.asarray(r['vertices_m'])*1000 @ equations[:, :3].T + equations[:, 3])) for r in originals)
        edges = collections.Counter(tuple(sorted((int(a), int(b)))) for f in faces for a, b in zip(f, np.roll(f, -1)))
        e['closed_and_convex'] = bool(all(n == 2 for n in edges.values()))
        e['microcell_policy'] = 'Merged material is retained inside this hull; no reference entity is deleted.'
        for inherited in ['source_boundary_patches', 'source_surface_partition',
                          'internal_closure_patches', 'absorbed_reference_closure_patches',
                          'approximated_source_boundary_patches', 'boundary_facets']:
            e.pop(inherited, None)
    assert collections.Counter(seen) == collections.Counter(old.keys()), 'Reference ownership is not a one-to-one partition'
    boundary = extract_exterior(entities)
    counts = collections.Counter(e['semantic_family'] for e in entities)
    assert set(counts) <= set(FAMILIES)
    failures = []
    thin = [e['id'] for e in entities if e['inscribed_diameter_mm'] < protocol['budget']['minimum_entity_inscribed_diameter_mm']]
    if thin:
        failures.append(dict(check='minimum_inscribed_diameter', entities=thin))
    invalid = [e['id'] for e in entities if not e['closed_and_convex'] or e['reference_containment_max_violation_mm'] > 1e-8]
    if invalid:
        failures.append(dict(check='closed_convex_and_containment', entities=invalid))
    for local in local_verification:
        if not local['verification'] or local['verification'].get('status') not in ('PASS', 'RUNTIME_ENTITY_PARTITION_FROZEN'):
            failures.append(dict(check='local_geometry_verification', path=local['path'], status=(local['verification'] or {}).get('status')))
    failures.append(dict(check='global_exterior_distance_and_protected_features', status='PENDING_SEPARATE_OFFLINE_VERIFICATION'))
    result = dict(schema_version=1, status='RUNTIME_ENTITY_PARTITION_BLOCKED',
                  task_comment=protocol['task_comment'], reference_file=str(reference_path.relative_to(ROOT)),
                  reference_sha256=sha(reference_path), preregistration_sha256=sha(protocol_path),
                  units='m', measurements='mm unless explicitly suffixed otherwise',
                  frame=reference['frame'], geometry_template=reference['geometry_template'],
                  entity_count_per_finger=len(entities), total_finger_entities=2*len(entities),
                  geometry_sha256=hashlib.sha256(json.dumps([[e['id'], e['vertices_m'], e['faces']] for e in entities], separators=(',', ':')).encode()).hexdigest(),
                  reduction_from_reference=1253-len(entities),
                  counts_by_source_part=dict(collections.Counter(e['source_part'] for e in entities)),
                  counts_by_semantic_family={f: counts[f] for f in FAMILIES}, entities=entities,
                  reference_entities_deleted=0, reference_entities_in_merged_groups=sum(len(e['reference_entity_ids']) for e in entities if len(e['reference_entity_ids']) > 1),
                  minimum_inscribed_diameter_mm=min(e['inscribed_diameter_mm'] for e in entities),
                  minimum_facet_normal_width_mm=min(e['minimum_facet_normal_width_mm'] for e in entities),
                  residual_entities_below_minimum=len(thin), boundary_extraction=boundary,
                  local_evidence=local_verification, blocking_findings=failures,
                  actual_model_change=False, simulation_or_compilation_run=False,
                  scope='One offline geometry candidate. No active MJCF, collision engine, physics, control or grasp operation.',
                  dependency_versions=dict(python='3.11', numpy='1.26.4', scipy='1.15.3'))
    out = ROOT/'reports/nero-gripper-runtime-partition.json'
    out.write_text(json.dumps(result, separators=(',', ':'), ensure_ascii=False)+'\n')
    cols = ['runtime_entity_id', 'semantic_family', 'semantic_feature', 'source_part', 'reference_entity_ids', 'source_regions', 'source_triangle_ids', 'primitive_type', 'exterior_patch_count', 'closure_and_ownership_patch_count', 'neighbors', 'inscribed_diameter_mm', 'minimum_facet_normal_width_mm', 'json_pointer']
    with (ROOT/'reports/nero-gripper-runtime-partition.csv').open('w', newline='') as f:
        w = csv.writer(f, lineterminator='\n'); w.writerow(cols)
        for i, e in enumerate(entities):
            w.writerow([e['id'], e['semantic_family'], e['semantic_feature'], e['source_part'], ';'.join(e['reference_entity_ids']), ';'.join(e['source_regions']), ';'.join(map(str, e['source_triangle_ids'])), e['primitive_type'], len(e['exterior_patches']), len(e['closure_and_ownership_patches']), ';'.join(e['neighbors']), e['inscribed_diameter_mm'], e['minimum_facet_normal_width_mm'], f'nero-gripper-runtime-partition.json#/entities/{i}'])
    print(json.dumps({k: v for k, v in result.items() if k not in ('entities', 'local_evidence')}, indent=2))


if __name__ == '__main__':
    main()
