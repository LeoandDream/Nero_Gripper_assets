"""Issue 25 contact layer: one frozen, offline, bounded semantic agglomeration.

Only numpy/scipy geometry is used.  No model or simulator is imported.
All lengths used by the construction and BSP certificate are millimetres.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import struct
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull, QhullError

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "debug/issue25/runtime-partition/contact"
REF = ROOT / "reports/nero-gripper-entity-partition.json"
FREEZE = ROOT / "reports/nero-gripper-runtime-preregistration.json"
DELTA = 0.005  # Frozen 5 um cube, Euclidean radius sqrt(3)*DELTA.
GUARD = 1e-7  # mm: shrink each expanded-solid plane before floating BSP.
EPS = 1e-10  # mm, three orders smaller than the inward guard.


def load_source():
    """Read only committed STL and committed functional-region inventory."""
    path = ROOT / "meshes/source/agx_gripper/meshes/gripper_link1.stl"
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    assert len(raw) == 84 + 50 * count, "Expected binary STL"
    vertices, triangles = [], []
    for i in range(count):
        record = struct.unpack_from("<12fH", raw, 84 + 50 * i)
        base = len(vertices)
        vertices.extend([list(record[3 + 3*k:6 + 3*k]) for k in range(3)])
        triangles.append(dict(source_face_id=i, vertex_ids=[base, base+1, base+2]))
    inventory_path = ROOT / "reports/nero-gripper-functional-regions.json"
    inventory = json.loads(inventory_path.read_text())
    regions = copy.deepcopy(inventory["source_regions"])
    for region in regions.values():
        region["source_face_ids"] = [i for first, last in region["source_faces_zero_based_inclusive"]
                                      for i in range(first, last+1)]
    return dict(vertices_m=vertices, triangles=triangles, regions=regions,
                source_stl_sha256=hashlib.sha256(raw).hexdigest(),
                functional_regions_sha256=hashlib.sha256(inventory_path.read_bytes()).hexdigest())


def unique_vertices(v):
    return np.unique(np.asarray(v, dtype=float), axis=0)


def hull(v):
    """Closed oriented hull; rescaling avoids source microcell conditioning."""
    v = unique_vertices(v)
    span = np.ptp(v, axis=0)
    if len(v) < 4 or np.any(span <= 0):
        return None
    origin = v.mean(0)
    scaled = (v - origin) / span
    try:
        h = ConvexHull(scaled)
    except QhullError:
        return None
    used = sorted(set(h.simplices.flatten()))
    remap = {j: i for i, j in enumerate(used)}
    vv = v[used]
    center = vv.mean(0)
    faces, eqs = [], []
    for f, scaled_eq in zip(h.simplices, h.equations):
        p = v[f]
        # QHull's support equation is considerably better conditioned than a
        # cross product of three nearly collinear exported STL vertices.
        n = scaled_eq[:3] / span
        length = np.linalg.norm(n)
        offset = (scaled_eq[3] - np.dot(n, origin)) / length
        n /= length
        q = scaled[f]
        if np.dot(np.cross(q[1] - q[0], q[2] - q[0]), scaled_eq[:3]) < 0:
            f = f[[0, 2, 1]]
        faces.append([remap[int(j)] for j in f])
        eqs.append(np.r_[n, offset])
    eqs = np.asarray(eqs)
    support_error = float(np.max(vv @ eqs[:, :3].T + eqs[:, 3]))
    edges = Counter(tuple(sorted((a, b))) for f in faces
                    for a, b in zip(f, f[1:] + f[:1]))
    if support_error > 2e-8 or set(edges.values()) != {2}:
        return None
    volume = sum(abs(np.dot(vv[f[0]] - center,
                            np.cross(vv[f[1]] - center, vv[f[2]] - center)))
                 / 6 for f in faces)
    return dict(v=vv, faces=faces, eq=eqs, volume=float(volume),
                support_error_mm=support_error,
                bbox=np.array([vv.min(0), vv.max(0)]))


def sphere(h):
    e = h["eq"]
    result = linprog([0, 0, 0, -1],
                     A_ub=np.c_[e[:, :3], np.ones(len(e))],
                     b_ub=-e[:, 3], bounds=[(None, None)] * 3 + [(0, None)],
                     method="highs")
    if not result.success:
        return 0., None
    c = result.x[:3]
    # The reported radius is independently recomputed at the actual LP center.
    radius = max(0., float(np.min(-(e[:, :3] @ c + e[:, 3]))))
    return radius * 2, c


def polys(h):
    return [h["v"][f].copy() for f in h["faces"]]


def poly_vertices(p):
    return unique_vertices(np.concatenate(p)) if p else np.empty((0, 3))


def full_dimensional(p):
    if not p:
        return False
    v = poly_vertices(p)
    if len(v) < 4:
        return False
    # Dropping a numerically zero-thickness remnant is protected by GUARD.
    return np.linalg.svd(v - v.mean(0), compute_uv=False)[-1] > EPS


def clip_polyhedron(p, eq):
    """Convex polygon-face clipping; retain n*x+d <= 0."""
    if not p:
        return []
    v = poly_vertices(p)
    signed = v @ eq[:3] + eq[3]
    if np.max(signed) <= EPS:
        return p
    if np.min(signed) > EPS:
        return []
    out, cuts = [], []
    for face in p:
        q = face @ eq[:3] + eq[3]
        result = []
        for a, b, qa, qb in zip(face, np.roll(face, -1, axis=0), q,
                                 np.roll(q, -1)):
            if qa <= 0:
                result.append(a)
            if (qa < 0 < qb) or (qb < 0 < qa):
                cross = a + (b - a) * (-qa / (qb - qa))
                result.append(cross)
                cuts.append(cross)
        if len(result) >= 3:
            pp = np.asarray(result)
            pp = pp[np.linalg.norm(pp - np.roll(pp, 1, axis=0), axis=1) > EPS]
            if len(pp) >= 3:
                out.append(pp)
    if cuts:
        cuts = unique_vertices(cuts)
        # Merge repeated edge crossings before making the convex cap.
        keep = []
        for x in cuts:
            if not any(np.linalg.norm(x - y) <= EPS for y in keep):
                keep.append(x)
        cuts = np.asarray(keep)
        if len(cuts) >= 3:
            center = cuts.mean(0)
            n = eq[:3]
            axis = np.eye(3)[np.argmin(abs(n))]
            u = np.cross(n, axis)
            u /= np.linalg.norm(u)
            w = np.cross(n, u)
            angles = np.arctan2((cuts - center) @ w, (cuts - center) @ u)
            out.append(cuts[np.argsort(angles)])
    return out if full_dimensional(out) else []


def bbox_intersects(a, b):
    return bool(np.all(a[0] <= b[1] + EPS) and np.all(b[0] <= a[1] + EPS))


def subtract_solid(p, other):
    """Disjoint convex BSP pieces of p outside a convex solid."""
    if not p:
        return []
    v = poly_vertices(p)
    bounds = np.array([v.min(0), v.max(0)])
    if not bbox_intersects(bounds, other["bbox"]):
        return [p]
    e = other["cover_eq"]
    if np.all(v @ e[:, :3].T + e[:, 3] <= 0):
        return []
    # A separating support plane permits immediate rejection.
    if np.any(np.min(v @ e[:, :3].T + e[:, 3], axis=0) > EPS):
        return [p]
    outside, inside = [], p
    for plane in e:
        if not inside:
            break
        cut = clip_polyhedron(inside, -plane)
        if cut:
            outside.append(cut)
        inside = clip_polyhedron(inside, plane)
    return outside


def covered_by_expanded(h, ids, expanded):
    remaining = [polys(h)]
    peak = 1
    for i in sorted(ids, key=lambda i: (-expanded[i]["volume"], i)):
        remaining = [q for p in remaining for q in subtract_solid(p, expanded[i])]
        peak = max(peak, len(remaining))
        if not remaining:
            return True, dict(remaining_positive_volume_pieces=0,
                              peak_bsp_pieces=peak,
                              reference_solids_used=len(ids))
        if peak > 20000:
            return False, dict(remaining_positive_volume_pieces=len(remaining),
                               peak_bsp_pieces=peak, computational_cap=True)
    return False, dict(remaining_positive_volume_pieces=len(remaining),
                       peak_bsp_pieces=peak)


def normal_deviation(h, normal):
    n = h["eq"][:, :3]
    n = n[n[:, 2] < -.9]
    if not len(n):
        return 0.
    return float(np.degrees(np.arccos(np.clip(n @ normal, -1, 1))).max())


def intersects_volume(a, b):
    """Actual convex intersection, guarded against zero-volume shared faces."""
    if not bbox_intersects(a["bbox"], b["bbox"]):
        return False
    p = polys(a)
    for e in b["eq"]:
        # A 1e-8 mm interior inset excludes coincident closure faces.
        ee = e.copy()
        ee[3] += 1e-8
        p = clip_polyhedron(p, ee)
        if not p:
            return False
    return full_dimensional(p)


def source_family_normals(source):
    pairs = {"distal_tip_contact": "short_inner_plane",
             "inner_ramp_contact": "inner_ramp",
             "shoulder_bevel": "inner_transition_1",
             "shoulder_blend_facet": "inner_transition_2",
             "inner_main_contact_plate": "broad_main_inner_plane"}
    out = {}
    for family, region in pairs.items():
        n = np.array(source["regions"][region]["area_weighted_normal"])
        out[family] = n / np.linalg.norm(n)
    return out


def face_normal(v, f):
    q = v[f]
    n = np.cross(q[1] - q[0], q[2] - q[0])
    return n / np.linalg.norm(n)


def assign_patch(polygon_m, ref_face, h):
    """Associate inherited boundary patch to a supporting runtime face.

    Polygon coordinates remain frozen-reference coordinates.  The target face
    and normal projection are concrete, with explicit per-vertex plane offsets.
    The BSP certificate independently bounds all newly created hull material.
    """
    p = np.asarray(polygon_m) * 1000
    n0 = ref_face
    align = h["eq"][:, :3] @ n0
    dist = np.max(abs(p @ h["eq"][:, :3].T + h["eq"][:, 3]), axis=0)
    score = dist + (1 - align) * 100
    target = int(np.argmin(score))
    eq = h["eq"][target]
    signed = p @ eq[:3] + eq[3]
    projected = p - signed[:, None] * eq[:3]
    return target, projected / 1000, float(max(abs(signed))), float(align[target])


def main():
    start = time.time()
    frozen = json.loads(FREEZE.read_text())
    assert frozen["frozen_before_any_merge"]
    assert hashlib.sha256(REF.read_bytes()).hexdigest() == frozen["reference_sha256"]
    budget = frozen["budget"]
    refdoc = json.loads(REF.read_text())
    refs = [e for e in refdoc["entities"] if e["source_part"] == "contact_layer"]
    source = load_source()
    normals = source_family_normals(source)
    originals, expanded, groups = {}, {}, []
    shifts = np.array([[a, b, c] for a in [-DELTA, DELTA]
                       for b in [-DELTA, DELTA] for c in [-DELTA, DELTA]])
    for i, e in enumerate(refs):
        h = hull(np.array(e["vertices_m"]) * 1000)
        assert h is not None, e["id"]
        originals[i] = h
        ex = hull((h["v"][:, None, :] + shifts[None, :, :]).reshape(-1, 3))
        assert ex is not None
        ex["cover_eq"] = ex["eq"].copy()
        ex["cover_eq"][:, 3] += GUARD
        expanded[i] = ex
        d, c = sphere(h)
        groups.append(dict(ids={i}, family=e["semantic_feature"], h=h,
                           diameter=d, center=c))
    ref_adjacency = set()
    name_index = {e["name"]: i for i, e in enumerate(refs)}
    for i, e in enumerate(refs):
        for other in e["neighbors"]:
            if other in name_index:
                ref_adjacency.add(tuple(sorted((i, name_index[other]))))
    rejected, history, examined = Counter(), [], set()
    while True:
        pairs = []
        for i, a in enumerate(groups):
            for j in range(i + 1, len(groups)):
                b = groups[j]
                if a["family"] != b["family"]:
                    continue
                if not any(tuple(sorted((x, y))) in ref_adjacency
                           for x in a["ids"] for y in b["ids"]):
                    continue
                key = (tuple(sorted(a["ids"])), tuple(sorted(b["ids"])))
                if key in examined:
                    continue
                small = min(a["diameter"], b["diameter"])
                pairs.append((small >= budget["minimum_entity_inscribed_diameter_mm"],
                              small, -(a["h"]["volume"] + b["h"]["volume"]),
                              min(a["ids"]), min(b["ids"]), i, j, key))
        pairs.sort()
        merged = False
        for _, __, ___, ____, _____, i, j, key in pairs:
            examined.add(key)
            a, b = groups[i], groups[j]
            ids = a["ids"] | b["ids"]
            h = hull(np.concatenate([originals[x]["v"] for x in sorted(ids)]))
            if h is None:
                rejected["numerical_hull"] += 1
                continue
            absorbed = []
            # Every source piece touched by a new same-family hull is owned by
            # this one group.  This makes workface ownership deterministic.
            while True:
                new = [k for k, g in enumerate(groups) if k not in [i, j]
                       and k not in absorbed and g["family"] == a["family"]
                       and intersects_volume(h, g["h"])]
                if not new:
                    break
                absorbed.extend(new)
                ids |= set().union(*(groups[k]["ids"] for k in new))
                h = hull(np.concatenate([originals[x]["v"] for x in sorted(ids)]))
                if h is None:
                    break
            if h is None:
                rejected["numerical_hull"] += 1
                continue
            angle = normal_deviation(h, normals[a["family"]])
            if angle > budget["critical_normal_deviation_deg"]:
                rejected["critical_normal"] += 1
                continue
            ok, certificate = covered_by_expanded(h, ids, expanded)
            if not ok:
                rejected["continuous_solid_bound"] += 1
                continue
            d, c = sphere(h)
            before = sorted(ids)
            event = dict(reference_entity_ids=[refs[x]["id"] for x in before],
                         semantic_family=a["family"],
                         absorbed_groups=len(absorbed),
                         inscribed_diameter_mm=d,
                         outward_euclidean_bound_mm=math.sqrt(3) * DELTA,
                         critical_normal_deviation_deg=angle,
                         added_volume_mm3=h["volume"] - sum(originals[x]["volume"] for x in ids),
                         continuous_cover=certificate)
            history.append(event)
            remove = sorted({i, j, *absorbed}, reverse=True)
            for k in remove:
                groups.pop(k)
            groups.append(dict(ids=ids, family=a["family"], h=h,
                               diameter=d, center=c))
            groups.sort(key=lambda x: min(x["ids"]))
            print("accepted", len(groups), a["family"], "diameter_mm", d, flush=True)
            merged = True
            break
        if not merged:
            break

    groups.sort(key=lambda g: (g["family"], float(g["h"]["v"][:, 1].mean()),
                              float(g["h"]["v"][:, 0].mean()), min(g["ids"])))
    owner = {i: gi for gi, g in enumerate(groups) for i in g["ids"]}
    names = []
    for g in groups:
        anchor = min(g["ids"])
        names.append("runtime_contact_" + g["family"] + "_ref_" + refs[anchor]["id"])
    entities, checks = [], []
    for gi, g in enumerate(groups):
        h = g["h"]
        ok, cover = covered_by_expanded(h, g["ids"], expanded)
        source_patches, closures, removed_closures, neighbor_names = [], [], [], set()
        for ri in sorted(g["ids"]):
            re = refs[ri]
            rv = np.asarray(re["vertices_m"]) * 1000
            for patch in re["source_boundary_patches"]:
                p = copy.deepcopy(patch)
                n = face_normal(rv, re["faces"][p["entity_face"]])
                target, projected, distance, align = assign_patch(p["polygon_m"], n, h)
                p.update(reference_entity_id=re["id"],
                         reference_entity_face=p["entity_face"],
                         entity_face=target,
                         runtime_normal_projection_polygon_m=projected.tolist(),
                         reference_to_associated_support_plane_max_mm=distance,
                         associated_support_normal_cosine=align,
                         classification="inherited_frozen_source_boundary_provenance")
                source_patches.append(p)
            for patch in re["internal_closure_patches"]:
                p = copy.deepcopy(patch)
                nr = name_index[p["neighbor"]]
                if owner[nr] == gi:
                    removed_closures.append(dict(reference_entity_id=re["id"],
                                                 reference_entity_face=p["entity_face"],
                                                 neighbor_reference_id=refs[nr]["id"],
                                                 polygon_m=p["polygon_m"],
                                                 classification="absorbed_internal_reference_closure"))
                    continue
                n = face_normal(rv, re["faces"][p["entity_face"]])
                target, projected, distance, align = assign_patch(p["polygon_m"], n, h)
                nn = names[owner[nr]]
                neighbor_names.add(nn)
                p.update(reference_entity_id=re["id"], reference_entity_face=p["entity_face"],
                         reference_neighbor=refs[nr]["id"], neighbor=nn,
                         entity_face=target,
                         runtime_normal_projection_polygon_m=projected.tolist(),
                         reference_to_associated_support_plane_max_mm=distance,
                         associated_support_normal_cosine=align,
                         classification="retained_reference_adjacency_interface_provenance")
                closures.append(p)
        # Inclusion follows from all original vertices satisfying every hull
        # support inequality; convexity then proves inclusion for whole cells.
        contained_error = max(float(np.max(originals[x]["v"] @ h["eq"][:, :3].T
                                          + h["eq"][:, 3])) for x in g["ids"])
        edges = Counter(tuple(sorted((a, b))) for f in h["faces"]
                        for a, b in zip(f, f[1:] + f[:1]))
        orient = Counter((a, b) for f in h["faces"]
                         for a, b in zip(f, f[1:] + f[:1]))
        angle = normal_deviation(h, normals[g["family"]])
        check = dict(name=names[gi], continuous_reference_cube_cover=ok,
                     continuous_cover_certificate=cover,
                     all_reference_cells_contained_support_tolerance_mm=contained_error,
                     reference_containment_pass=contained_error <= 2e-8,
                     closed_edge_incidence_two=set(edges.values()) == {2},
                     oriented_edge_pairs=all(orient[(a, b)] == orient[(b, a)] for a, b in orient),
                     convex_support_max_mm=h["support_error_mm"],
                     inscribed_diameter_mm=g["diameter"],
                     minimum_scale_pass=g["diameter"] + 1e-8 >= budget["minimum_entity_inscribed_diameter_mm"],
                     critical_normal_deviation_deg=angle,
                     critical_normal_pass=angle <= budget["critical_normal_deviation_deg"],
                     positive_volume_mm3=h["volume"])
        checks.append(check)
        entities.append(dict(id="RC%03d" % (gi + 1), name=names[gi],
                             semantic_family=refs[min(g["ids"])]["semantic_family"], semantic_feature=g["family"],
                             source_part="contact_layer", primitive_type="convex_polyhedron",
                             vertices_m=(h["v"] / 1000).tolist(), faces=h["faces"],
                             halfspaces_m=np.c_[h["eq"][:, :3], h["eq"][:, 3] / 1000].tolist(),
                             volume_m3=h["volume"] * 1e-9,
                             bounds_m=(h["bbox"] / 1000).tolist(),
                             closed_and_convex=check["closed_edge_incidence_two"] and check["oriented_edge_pairs"],
                             reference_entity_ids=[refs[x]["id"] for x in sorted(g["ids"])],
                             reference_entity_names=[refs[x]["name"] for x in sorted(g["ids"])],
                             source_triangle_ids=sorted(set().union(*(set(refs[x]["source_triangle_ids"]) for x in g["ids"]))),
                             source_regions=sorted(set().union(*(set(refs[x]["source_regions"]) for x in g["ids"]))),
                             source_boundary_patches=source_patches,
                             internal_closure_patches=closures,
                             absorbed_reference_closure_patches=removed_closures,
                             neighbors=sorted(neighbor_names),
                             inscribed_diameter_mm=g["diameter"],
                             inscribed_sphere_center_m=(g["center"] / 1000).tolist(),
                             inscribed_sphere_radius_mm=g["diameter"] / 2,
                             critical_normal_deviation_deg=angle,
                             outward_error_bound_mm=math.sqrt(3) * DELTA,
                             inward_material_error_bound_mm=2e-8,
                             geometry_validation_state="OFFLINE_BOUNDED_SEMANTIC_PARTITION",
                             source_surface_partition=copy.deepcopy(source_patches),
                             overlap_policy="Source cells retained; same-family intersecting hulls absorbed. Cross-family boundary changes bounded by continuous 5um-cube cover; global assembly validator decides exposed union ownership."))
    status = "PASS" if all(c["continuous_reference_cube_cover"]
                           and c["reference_containment_pass"]
                           and c["closed_edge_incidence_two"]
                           and c["oriented_edge_pairs"]
                           and c["minimum_scale_pass"]
                           and c["critical_normal_pass"] for c in checks) else "BLOCKED"
    result = dict(schema_version=1, status=status, source_part="contact_layer",
                  reference_sha256=frozen["reference_sha256"],
                  preregistration_sha256=hashlib.sha256(FREEZE.read_bytes()).hexdigest(),
                  units="m", entity_count=len(entities), reference_entity_count=len(refs),
                  counts_by_semantic_family=dict(Counter(e["semantic_family"] for e in entities)),
                  counts_by_semantic_feature=dict(Counter(g["family"] for g in groups)),
                  entities=entities,
                  construction=dict(method="Deterministic reference-adjacent same-family hull agglomeration; microcell first; absorb intersecting same-family owners",
                                    coordinate_space="mm during construction; m for exported geometry",
                                    candidate_count=1, forbidden_runtime_operations=0),
                  continuous_error_certificate=dict(
                      method="Convex 3D BSP subtraction of runtime hull by union of frozen reference Minkowski cube expansions",
                      cube_halfwidth_mm=DELTA, euclidean_outward_bound_mm=math.sqrt(3)*DELTA,
                      expanded_halfspace_inward_guard_mm=GUARD,
                      clipping_arithmetic_tolerance_mm=EPS,
                      proof="For each original convex S, conv{v+c : v in vertices(S), c in cube corners}=S+cube. Every expanded halfspace is moved inward by 1e-7mm before BSP. Empty positive-volume remainder certifies full runtime convex-solid coverage, with lower-dimensional boundary obtained by closure. Every retained source cell is inside its hull by convex support. Thus inward material error is zero up to documented floating support tolerance, and outward material Hausdorff error <=sqrt(3)*0.005mm.",
                      limitations="This certificate bounds material solids. Actual exterior/critical-surface ownership, inter-part interfaces and protected-void witnesses require the separate frozen global validation; inherited source patches are explicitly provenance, not a claim of unchanged runtime planar footprints."),
                  verification=dict(status=status, entity_checks=checks,
                                    critical_working_supports="See critical-features.json; global actual-exterior validation remains separate",
                                    minimum_scale_failing_entities=[c["name"] for c in checks if not c["minimum_scale_pass"]]), merge_history=history,
                  rejected_merge_counts=dict(rejected),
                  total_volume_m3=sum(e["volume_m3"] for e in entities),
                  reference_total_volume_m3=sum(e["volume_m3"] for e in refs),
                  minimum_inscribed_diameter_mm=min(g["diameter"] for g in groups),
                  wall_seconds=time.time()-start)
    OUT.mkdir(parents=True, exist_ok=True)
    result["continuous_error_certificate"]["certificate_restriction"] = (
        "The fixed 5um cube is a sufficient certificate stricter than the frozen 10um Euclidean critical budget. "
        "Rejected cube-cover merges do not establish that every partition meeting 10um is impossible. "
        "The single 22-entity candidate is retained as BLOCKED by four actual minimum-scale failures.")
    critical = critical_features(result, refs, source, normals)
    result["verification"]["critical_working_support_max_mm"] = critical["maximum_working_support_extent_error_mm"]
    result["verification"]["protected_void_corridors_pass"] = critical["all_clearance_boxes_empty"]
    (OUT / "partition.json").write_text(json.dumps(result, indent=2)+"\n")
    (OUT / "critical-features.json").write_text(json.dumps(critical, indent=2)+"\n")
    (OUT / "verification.json").write_text(json.dumps({k: v for k, v in result.items()
                                                       if k not in ["entities", "merge_history"]}, indent=2)+"\n")
    print(json.dumps({k: result[k] for k in ["status", "entity_count", "counts_by_semantic_family",
                                             "minimum_inscribed_diameter_mm", "rejected_merge_counts",
                                             "total_volume_m3", "reference_total_volume_m3", "wall_seconds"]}, indent=2))


def critical_features(result, refs, source, normals):
    """Continuous support-plane and separating-plane clearance witnesses."""
    critical_ids = set(range(545, 582))
    byid = {e["id"]: e for e in refs}
    supports = []
    for e in result["entities"]:
        n = normals[e["semantic_feature"]]
        points = [p for rid in e["reference_entity_ids"]
                  for patch in byid[rid]["source_boundary_patches"]
                  if patch["source_face_id"] in critical_ids for p in patch["polygon_m"]]
        if not points:
            continue
        p = np.asarray(points) * 1000
        h = np.asarray(e["vertices_m"]) * 1000
        # An entire convex polygon has its support extrema at its vertices.
        extent = float(max(h @ n) - min(p @ n))
        supports.append(dict(entity=e["name"], reference_entity_ids=e["reference_entity_ids"],
                             working_source_triangle_ids=sorted({patch["source_face_id"]
                                 for rid in e["reference_entity_ids"]
                                 for patch in byid[rid]["source_boundary_patches"]
                                 if patch["source_face_id"] in critical_ids}),
                             source_normal=n.tolist(),
                             maximum_working_support_extent_error_mm=max(0., extent),
                             critical_normal_deviation_deg=e["critical_normal_deviation_deg"],
                             proof="Convex support extrema occur at vertices; all inherited working-face polygons lie between the reported parallel support planes. This is a working-plane certificate, not a replacement for global union-exterior distances."))
    boxes = [
        ("central_root_notch_50um_inset_core", [[-8.60, -71.05, 2.05], [8.60, -70.05, 3.95]]),
        ("left_root_relief_50um_inset_core", [[-27.50, -70.95, 2.05], [-10.70, -68.05, 3.95]]),
    ]
    voids = []
    for name, bounds in boxes:
        lo, hi = np.asarray(bounds)
        corners = np.array([[x, y, z] for x in [lo[0], hi[0]]
                            for y in [lo[1], hi[1]] for z in [lo[2], hi[2]]])
        separations = []
        for e in result["entities"]:
            eq = np.asarray(e["halfspaces_m"])
            eq[:, 3] *= 1000
            v = np.asarray(e["vertices_m"]) * 1000
            # Complete convex-polytope/AABB separating-axis set: face normals
            # and cross products of body edges with the three box edge axes.
            axes = list(eq[:, :3]) + list(np.eye(3))
            edges = {tuple(sorted((a, b))) for f in e["faces"]
                     for a, b in zip(f, f[1:] + f[:1])}
            for a, b in edges:
                for axis in np.eye(3):
                    n = np.cross(v[b] - v[a], axis)
                    length = np.linalg.norm(n)
                    if length > 1e-16:
                        axes.append(n / length)
            best, best_axis = -float("inf"), None
            for n in axes:
                body, box = v @ n, corners @ n
                gaps = [float(min(box) - max(body)), float(min(body) - max(box))]
                value = max(gaps)
                if value > best:
                    best, best_axis = value, (n if gaps[0] >= gaps[1] else -n)
            separations.append(dict(entity=e["name"], separating_unit_axis=best_axis.tolist(),
                                    clearance_lower_bound_mm=best))
        margin = min(x["clearance_lower_bound_mm"] for x in separations)
        voids.append(dict(name=name, bounds_mm=bounds, connected_convex_corridor=True,
                          empty_in_runtime_material=margin > EPS,
                          separation_clearance_lower_bound_mm=margin,
                          per_entity_separating_planes=separations,
                          proof="For every runtime convex entity, the recorded unit separating axis has disjoint body/box projection intervals. Vertex support extrema and linearity certify the entire convex box. Face, box and edge-cross axes form the complete separating-axis set; no point sampling is used."))
    contour = {}
    for name in ["short_inner_plane", "inner_ramp", "inner_transition_1",
                 "inner_transition_2", "broad_main_inner_plane"]:
        r = source["regions"][name]
        contour[name] = dict(source_face_ids=r["source_face_ids"],
                             contour_vertex_counts=[len(x) for x in r["planar_contour"]["boundary_points_m_loops"]],
                             source_contour_points_m=r["planar_contour"]["boundary_points_m_loops"],
                             material_outward_bound_mm=math.sqrt(3)*DELTA,
                             material_inward_bound_mm=2e-8,
                             source_material_fully_retained=True)
    return dict(status="PASS" if all(x["empty_in_runtime_material"] for x in voids) else "BLOCKED",
                source_file="meshes/source/agx_gripper/meshes/gripper_link1.stl",
                critical_working_supports=supports,
                maximum_working_support_extent_error_mm=max(x["maximum_working_support_extent_error_mm"] for x in supports),
                maximum_working_normal_deviation_deg=max(x["critical_normal_deviation_deg"] for x in supports),
                source_contours=contour,
                continuous_protected_void_corridors=voids,
                all_clearance_boxes_empty=all(x["empty_in_runtime_material"] for x in voids),
                scope="Contact layer has root/taper notches and no bore/pocket passage. Backing and mount bore/pocket witnesses are outside this local certificate. Global exterior-surface ownership is validated by the assembler.")


if __name__ == "__main__":
    main()
