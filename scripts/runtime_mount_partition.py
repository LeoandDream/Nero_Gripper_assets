#!/usr/bin/env python3
"""Issue 25 mount runtime candidate: pure source computational geometry.

No simulator, collision library, or mesh repair is used.  The frozen mount
reference is a disjoint xz terrain partition.  A merge must preserve its xz
footprint, the y=-70 mm assembly section, and the terrain height on its outer
perimeter.  Only its interior upper terrain may advance, within the frozen
source budget.  Every error is measured against the original source seeds.
"""
from __future__ import annotations

import collections
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "debug/issue25/runtime-partition/mount"
REF = ROOT / "reports/nero-gripper-entity-partition.json"
PRE = ROOT / "reports/nero-gripper-runtime-preregistration.json"
SEED_FILE = ROOT / "reports/nero-gripper-runtime-mount-seeds.json"
EPS = 3e-7  # mm, numerical classification only
AREA_EPS = 2e-8  # mm2, numerical classification only


def area(p):
    p = np.asarray(p)
    if len(p) < 3:
        return 0.0
    return float(np.linalg.norm(np.sum(np.cross(p, np.roll(p, -1, axis=0)), axis=0)) / 2)


def clean(p):
    out = []
    for q in p:
        if not out or np.linalg.norm(q - out[-1]) > 1e-9:
            out.append(np.asarray(q))
    if len(out) > 1 and np.linalg.norm(out[0] - out[-1]) < 1e-9:
        out.pop()
    return np.array(out)


def clip(p, eq, inside=True):
    if len(p) < 3:
        return np.empty((0, 3))
    out = []
    for a, b in zip(p, np.roll(p, -1, axis=0)):
        da, db = np.dot(eq[:3], a) + eq[3], np.dot(eq[:3], b) + eq[3]
        if not inside:
            da, db = -da, -db
        ia, ib = da <= EPS, db <= EPS
        if ia:
            out.append(a)
        if ia != ib:
            out.append(a + (b-a)*da/(da-db))
    return clean(out)


def clip_all(p, eqs):
    for e in eqs:
        p = clip(p, e)
        if len(p) < 3:
            break
    return p


def distinct(eqs):
    out = []
    for e in eqs:
        if not any(np.linalg.norm(e[:3]-a[:3]) < 2e-8 and abs(e[3]-a[3]) < EPS for a in out):
            out.append(e)
    return np.array(out)


def hull(points):
    v = np.unique(np.round(np.array(points), 10), axis=0)
    h = ConvexHull(v)
    v = v[h.vertices]
    h = ConvexHull(v)
    return v, h, distinct(h.equations)


def projected(v):
    h = ConvexHull(v[:, [0, 2]])
    return v[h.vertices][:, [0, 2]], float(h.volume)


def projected_area(p):
    p = np.asarray(p)
    return float(abs(np.dot(p[:, 0], np.roll(p[:, 2], -1)) - np.dot(p[:, 2], np.roll(p[:, 0], -1))) / 2)


def top_section_area(v, top_y):
    p = v[abs(v[:, 1]-top_y) < EPS][:, [0, 2]]
    if len(p) < 3 or np.linalg.matrix_rank(p[1:]-p[0], tol=1e-8) < 2:
        return 0.0
    return float(ConvexHull(p).volume)


def upper_planes(eqs):
    e = eqs[eqs[:, 1] > 1e-7]
    return np.c_[-e[:, 0]/e[:, 1], -e[:, 2]/e[:, 1], -e[:, 3]/e[:, 1]]


def terrain_y(p, planes):
    return np.min(p[:, [0, 2]] @ planes[:, :2].T + planes[:, 2], axis=1)


def exact_seed_error(seed, planes):
    """Exact PL maximum of min(candidate upper planes)-source top plane.

    Each candidate top face is projected onto the frozen source seed polygon.
    Clipping by every competing upper plane enumerates all envelope breakpoints;
    the linear difference reaches its maximum at one of those polygon vertices.
    This is the planar subdivision form of the equivalent halfspace LP.
    """
    p = seed["top"]
    n = np.cross(p[1]-p[0], p[2]-p[0])
    source = np.array([-n[0]/n[1], -n[2]/n[1], np.dot(n, p[0])/n[1]])
    best = 0.0
    witness = p[0].copy()
    for a in planes:
        q = p.copy()
        for b in planes:
            # a(x,z) <= b(x,z) selects the active upper envelope facet.
            q = clip(q, np.array([a[0]-b[0], 0., a[1]-b[1], a[2]-b[2]]))
            if len(q) < 3:
                break
        if len(q) < 3 or projected_area(q) < 1e-12:
            continue
        delta = q[:, [0, 2]] @ (a[:2]-source[:2]) + a[2]-source[2]
        i = int(np.argmax(delta))
        if delta[i] > best:
            best = float(delta[i])
            witness = q[i].copy()
    return best, witness


def perimeter_error(seed_ids, footprint, planes, seeds):
    """Maximum candidate height advance on the complete original perimeter."""
    ph = ConvexHull(footprint)
    pe = ph.equations
    maximum = 0.0
    for si in seed_ids:
        p = seeds[si]["top"]
        for a, b in zip(p, np.roll(p, -1, axis=0)):
            aa, bb = a[[0, 2]], b[[0, 2]]
            if not any(abs(np.dot(e[:2], aa)+e[2]) < EPS and abs(np.dot(e[:2], bb)+e[2]) < EPS for e in pe):
                continue
            y0 = planes[:, :2] @ aa + planes[:, 2] - a[1]
            dy = planes[:, :2] @ (bb-aa) - (b[1]-a[1])
            times = [0., 1.]
            for i, j in itertools.combinations(range(len(planes)), 2):
                den = dy[i]-dy[j]
                if abs(den) > 1e-12:
                    t = float((y0[j]-y0[i])/den)
                    if 0 < t < 1:
                        times.append(t)
            for t in times:
                maximum = max(maximum, float(np.min(y0+t*dy)))
    return maximum


def ball(eqs):
    r = linprog([0, 0, 0, -1], A_ub=np.c_[eqs[:, :3], np.ones(len(eqs))],
                b_ub=-eqs[:, 3], bounds=[(None, None)]*3+[(0, None)], method="highs")
    if not r.success:
        raise RuntimeError("Insphere LP failed")
    return float(r.x[3]), r.x[:3]


def plane_polygon(v, e):
    p = v[abs(v @ e[:3]+e[3]) < 2e-6]
    u = np.cross(e[:3], np.eye(3)[np.argmin(abs(e[:3]))])
    u /= np.linalg.norm(u)
    w = np.cross(e[:3], u)
    p = p[ConvexHull(np.c_[p@u, p@w]).vertices]
    if np.dot(np.sum(np.cross(p, np.roll(p, -1, axis=0)), axis=0), e[:3]) < 0:
        p = p[::-1]
    return p


def subtract(p, other, normal):
    remain, outside = p, []
    for a, b in zip(other, np.roll(other, -1, axis=0)):
        n = np.cross(b-a, normal)
        n /= np.linalg.norm(n)
        e = np.r_[n, -np.dot(n, a)]
        q = clip(remain, e, False)
        if area(q) > AREA_EPS:
            outside.append(q)
        remain = clip(remain, e)
        if len(remain) < 3:
            break
    return outside


def width(v, h):
    edges = set(tuple(sorted((int(t[i]), int(t[(i+1) % 3])))) for t in h.simplices for i in range(3))
    vectors = [v[b]-v[a] for a, b in edges]
    normals = list(h.equations[:, :3])
    for a, b in itertools.combinations(vectors, 2):
        n = np.cross(a, b)
        l = np.linalg.norm(n)
        if l > 1e-12:
            normals.append(n/l)
    return float(np.ptp(v @ np.array(normals).T, axis=0).min())


def main():
    pre = json.loads(PRE.read_text())
    budget = pre["budget"]
    ref_bytes = REF.read_bytes()
    if hashlib.sha256(ref_bytes).hexdigest() != pre["reference_sha256"]:
        raise RuntimeError("Frozen reference hash differs from preregistration")
    reference = json.loads(ref_bytes)
    refs = [e for e in reference["entities"] if e["semantic_family"] == "mount_root"]
    seeds = json.loads(SEED_FILE.read_text())["seeds"]
    for s in seeds:
        s["top"] = np.array(s["top_polygon_m"])*1000
        # Blind recess floors/lips use the tighter frozen positional allocation.
        s["budget_mm"] = budget["critical_working_surface_mm"] if 109 <= s["top_source_face_id"] <= 448 else budget["maximum_outward_mm"]
    cells = {}
    top_y = max(v[1]*1000 for e in refs for v in e["vertices_m"])
    ref_name_to_index = {e["name"]: i for i, e in enumerate(refs)}
    for i, e in enumerate(refs):
        v, h, eq = hull(np.array(e["vertices_m"])*1000)
        fp, ar = projected(v)
        r, center = ball(eq)
        seed_ids = e["construction_cell_ids"]
        zones = {seeds[s]["zone"] for s in seed_ids}
        if len(zones) != 1:
            raise RuntimeError(("Reference mount cell crosses frozen slot zones", e["id"], zones))
        cells[i] = {"members": [i], "seed_ids": seed_ids,
                    "zone": next(iter(zones)), "v": v, "h": h, "eq": eq,
                    "area": ar, "top_area": top_section_area(v, top_y),
                    "radius": r, "center": center, "reference_volume": e["volume_m3"]*1e9,
                    "neighbors": {ref_name_to_index[n] for n in e["neighbors"] if n in ref_name_to_index},
                    "max_error": 0., "perimeter_error": 0.}
    initial_min = min(2*c["radius"] for c in cells.values())
    attempts = collections.Counter()
    history = []
    # Rejected fixed member sets are never retried.  There is one deterministic
    # candidate, one fixed budget, and no feedback from any physical execution.
    cache = set()
    rounds = 0
    while True:
        rounds += 1
        choices = []
        for i in sorted(cells):
            a = cells[i]
            for j in sorted(n for n in a["neighbors"] if n > i and n in cells):
                b = cells[j]
                key = tuple(sorted(a["members"]+b["members"]))
                if key in cache:
                    continue
                cache.add(key)
                attempts["considered"] += 1
                if a["zone"] != b["zone"]:
                    attempts["different_slot_zone"] += 1
                    continue
                if any(seeds[s]["rear_source_face_id"] == 451 for s in a["seed_ids"]+b["seed_ids"]):
                    # Face451 is the one warped through-bore rear wedge, not
                    # a constant-y terrain base.  Its complete original owner
                    # stays fixed rather than applying an upper-only proof.
                    attempts["protected_nonprismatic_bore_wedge"] += 1
                    continue
                v, h, eq = hull(np.concatenate((a["v"], b["v"])))
                fp, ar = projected(v)
                if abs(ar-a["area"]-b["area"]) > AREA_EPS:
                    attempts["nonconvex_projected_union"] += 1
                    continue
                top_area = top_section_area(v, top_y)
                if abs(top_area-a["top_area"]-b["top_area"]) > AREA_EPS:
                    attempts["changes_assembly_section_or_mouth"] += 1
                    continue
                seed_ids = sorted(a["seed_ids"]+b["seed_ids"])
                planes = upper_planes(eq)
                quick = max(float(np.max(terrain_y(seeds[s]["top"], planes)-seeds[s]["top"][:, 1]))-seeds[s]["budget_mm"] for s in seed_ids)
                if quick > EPS:
                    attempts["source_vertex_budget"] += 1
                    continue
                perim = perimeter_error(seed_ids, fp, planes, seeds)
                if perim > EPS:
                    attempts["would_expose_partition_cliff"] += 1
                    continue
                worst = 0.
                rejected = False
                for s in seed_ids:
                    err, _ = exact_seed_error(seeds[s], planes)
                    worst = max(worst, err)
                    if err > seeds[s]["budget_mm"]+EPS:
                        attempts["full_source_envelope_budget"] += 1
                        rejected = True
                        break
                if rejected:
                    continue
                radius, center = ball(eq)
                proxy = min(a["radius"], b["radius"])
                score = (0 if 2*proxy < budget["minimum_entity_inscribed_diameter_mm"] else 1,
                         proxy, -radius, worst, -h.volume, i, j)
                c = {"members": list(key), "seed_ids": seed_ids, "zone": a["zone"],
                     "v": v, "h": h, "eq": eq, "area": ar, "top_area": top_area,
                     "radius": radius, "center": center,
                     "reference_volume": a["reference_volume"]+b["reference_volume"],
                     "neighbors": (a["neighbors"] | b["neighbors"])-{i, j},
                     "max_error": worst, "perimeter_error": perim}
                choices.append((score, i, j, c))
        if not choices:
            break
        used = set()
        accepted = 0
        for score, i, j, c in sorted(choices, key=lambda t: t[0]):
            if i in used or j in used:
                # The group may become eligible in a later neighborhood.
                cache.discard(tuple(c["members"]))
                continue
            a, b = cells.pop(i), cells.pop(j)
            c["neighbors"] = (a["neighbors"] | b["neighbors"])-{i, j}
            cells[i] = c
            for k in list(c["neighbors"]):
                if k in cells:
                    cells[k]["neighbors"].difference_update({i, j})
                    cells[k]["neighbors"].add(i)
            used.update((i, j))
            accepted += 1
            history.append({"round": rounds, "reference_entity_ids": [refs[k]["id"] for k in c["members"]],
                            "source_envelope_max_outward_mm": c["max_error"],
                            "outer_perimeter_height_max_change_mm": c["perimeter_error"],
                            "inscribed_diameter_mm": 2*c["radius"],
                            "added_volume_mm3": float(c["h"].volume-c["reference_volume"])})
        print(f"mount round {rounds}: {len(cells)} entities, {accepted} accepted", flush=True)

    ordered = sorted(cells.values(), key=lambda c: (c["zone"], tuple(np.round(c["v"].mean(0), 8)), c["members"]))
    owner = {}
    for i, c in enumerate(ordered, 1):
        c["name"] = f"mount_root_{c['zone']}_runtime_{i:03d}"
        c["id"] = f"RM{i:04d}"
        for m in c["members"]:
            owner[m] = c

    # Full facet-level classification: unchanged source, neighboring entity,
    # or changed source-backed terrain.  An unclassified closure is a failure.
    entities = []
    closure_failures = []
    source_certificates = []
    all_source_map = collections.defaultdict(set)
    max_boundary_residual = 0.
    for ci, c in enumerate(ordered):
        v, h, eq = c["v"], c["h"], c["eq"]
        supporting = collections.defaultdict(list)
        source_patches = []
        for m in c["members"]:
            for p in refs[m]["source_boundary_patches"]:
                q = np.array(p["polygon_m"])*1000
                n = np.cross(q[1]-q[0], q[2]-q[0])
                if np.linalg.norm(n) < 1e-12:
                    continue
                n /= np.linalg.norm(n)
                if np.dot(n, q.mean(0)-v.mean(0)) < 0:
                    n = -n
                matched = [pi for pi, e in enumerate(eq) if np.linalg.norm(e[:3]-n) < 3e-6 and np.max(abs(q@e[:3]+e[3])) < 3e-6]
                if matched:
                    pi = matched[0]
                    supporting[pi].append(q)
                    source_patches.append({**p, "support_plane_id": pi, "reference_entity_id": refs[m]["id"]})
        neighbors = {owner[n]["name"]: owner[n] for m in c["members"] for n in (ref_name_to_index[x] for x in refs[m]["neighbors"] if x in ref_name_to_index) if owner[n] is not c}
        facets, closures, exposed = [], [], []
        for pi, e in enumerate(eq):
            poly = plane_polygon(v, e)
            remaining = [poly]
            for q in supporting[pi]:
                remaining = [r for p in remaining for r in subtract(p, q, e[:3])]
            shared = []
            for name, neighbor in sorted(neighbors.items()):
                if not any(np.linalg.norm(e[:3]+ee[:3]) < 3e-6 and abs(e[3]+ee[3]) < 3e-6 for ee in neighbor["eq"]):
                    continue
                q = clip_all(poly, neighbor["eq"])
                if area(q) > AREA_EPS:
                    shared.append({"entity": name, "polygon_m": (q/1000).tolist(), "area_m2": area(q)/1e6})
                    remaining = [r for p in remaining for r in subtract(p, q, e[:3])]
            for p in remaining:
                if area(p) <= AREA_EPS:
                    continue
                record = {"support_plane_id": pi, "polygon_m": (p/1000).tolist(), "area_m2": area(p)/1e6}
                if e[1] > 1e-7:
                    record["role"] = "approximated_source_terrain"
                    exposed.append(record)
                else:
                    closure_failures.append({"entity": c["name"], **record})
            if shared:
                closures.append({"support_plane_id": pi, "covered_by": shared,
                                 "area_m2": sum(p["area_m2"] for p in shared),
                                 "external_exposed_area_m2": 0.})
            facets.append({"support_plane_id": pi, "polygon_m": (poly/1000).tolist(),
                           "area_m2": area(poly)/1e6,
                           "unchanged_source_area_m2": sum(area(q) for q in supporting[pi])/1e6,
                           "interior_closure_area_m2": sum(p["area_m2"] for p in shared),
                           "approximated_source_area_m2": sum(area(p) for p in remaining)/1e6 if e[1] > 1e-7 else 0.})
        seed_certs = []
        for s in c["seed_ids"]:
            err, wit = exact_seed_error(seeds[s], upper_planes(eq))
            rec = {"source_seed_id": s, "source_face_id": seeds[s]["top_source_face_id"],
                   "max_vertical_outward_mm": err, "budget_mm": seeds[s]["budget_mm"],
                   "source_witness_m": (wit/1000).tolist()}
            seed_certs.append(rec)
            source_certificates.append({"entity": c["name"], **rec})
        triangles = []
        for tri in h.simplices:
            t = v[tri]
            if np.dot(np.cross(t[1]-t[0], t[2]-t[0]), t.mean(0)-v.mean(0)) < 0:
                tri = tri[::-1]
            triangles.append({"vertex_indices": tri.tolist()})
        edges = collections.Counter(tuple(sorted((t["vertex_indices"][k], t["vertex_indices"][(k+1) % 3]))) for t in triangles for k in range(3))
        source_ids = sorted({sid for m in c["members"] for sid in refs[m]["source_triangle_ids"]})
        for sid in source_ids:
            all_source_map[sid].add(c["id"])
        entities.append({"id": c["id"], "name": c["name"], "semantic_family": "mount_root",
                         "semantic_feature": c["zone"]+"_source_aware_terrain", "source_part": "mount_block",
                         "reference_entity_ids": [refs[m]["id"] for m in c["members"]],
                         "reference_entity_names": [refs[m]["name"] for m in c["members"]],
                         "source_triangle_ids": source_ids, "construction_cell_ids": c["seed_ids"],
                         "source_regions": sorted({r for m in c["members"] for r in refs[m]["source_regions"]}),
                         "vertices_m": (v/1000).tolist(), "faces": triangles,
                         "halfspaces_m": [e[:3].tolist()+[float(e[3])/1000] for e in eq],
                         "bounds_m": [(v.min(0)/1000).tolist(), (v.max(0)/1000).tolist()],
                         "volume_m3": float(h.volume)/1e9,
                         "reference_material_volume_m3": c["reference_volume"]/1e9,
                         "added_volume_m3": float(h.volume-c["reference_volume"])/1e9,
                         "inscribed_diameter_mm": 2*c["radius"],
                         "insphere_center_m": (c["center"]/1000).tolist(),
                         "minimum_width_mm": width(v, h),
                         "closed_and_convex": all(n == 2 for n in edges.values()) and len(v)-len(edges)+len(triangles) == 2,
                         "hull_edge_incidence_two": all(n == 2 for n in edges.values()),
                         "hull_euler_characteristic": len(v)-len(edges)+len(triangles),
                         "source_boundary_patches": source_patches,
                         "approximated_source_boundary_patches": exposed,
                         "boundary_facets": facets, "internal_closure_patches": closures,
                         "neighbors": sorted(neighbors),
                         "source_error_certificate": seed_certs,
                         "maximum_source_outward_mm": max(x["max_vertical_outward_mm"] for x in seed_certs),
                         "maximum_source_inward_mm": 0.,
                         "original_assembly_section_preserved": True,
                         "projected_voids_preserved": True,
                         "outer_perimeter_height_max_change_mm": c["perimeter_error"]})
        print(f"mount finalize {ci+1}/{len(ordered)}", flush=True) if ci % 30 == 0 else None

    thin = [{"entity": e["name"], "inscribed_diameter_mm": e["inscribed_diameter_mm"],
             "reference_entity_ids": e["reference_entity_ids"]} for e in entities if e["inscribed_diameter_mm"]+EPS < budget["minimum_entity_inscribed_diameter_mm"]]
    error_failures = [r for r in source_certificates if r["max_vertical_outward_mm"] > r["budget_mm"]+EPS]
    failures = []
    if thin:
        failures.append("MINIMUM_INSCRIBED_DIAMETER")
    if closure_failures:
        failures.append("EXPOSED_UNCLASSIFIED_CLOSURE")
    if error_failures:
        failures.append("SOURCE_ERROR_BUDGET")
    if set(all_source_map) != set(range(504)):
        failures.append("SOURCE_FACE_MAPPING_INCOMPLETE")
    summary = {"schema_version": 1, "status": "BLOCKED" if failures else "GEOMETRY_CANDIDATE_VALIDATED",
               "source_part": "mount_block", "units": "m", "algorithm_working_units": "mm",
               "reference_file": str(REF.relative_to(ROOT)), "reference_sha256": hashlib.sha256(ref_bytes).hexdigest(),
               "preregistration_sha256": hashlib.sha256(PRE.read_bytes()).hexdigest(),
               "reference_entity_count": len(refs), "entity_count": len(entities),
               "initial_minimum_inscribed_diameter_mm": initial_min,
               "minimum_inscribed_diameter_mm": min(e["inscribed_diameter_mm"] for e in entities),
               "minimum_width_mm": min(e["minimum_width_mm"] for e in entities),
               "source_volume_mm3": sum(c["reference_volume"] for c in ordered),
               "runtime_volume_mm3": sum(float(c["h"].volume) for c in ordered),
               "maximum_source_outward_mm": max(r["max_vertical_outward_mm"] for r in source_certificates),
               "maximum_source_inward_mm": 0., "source_error_failures": error_failures,
               "source_part_bidirectional_exterior_surface_distance_bound_mm": max(r["max_vertical_outward_mm"] for r in source_certificates),
               "zero_inward_value_meaning": "No original material is removed. This is a solid-membership statement; exterior surfaces can move outward by the nonzero terrain bound and no zero surface-distance claim is made.",
               "minimum_scale_failures": thin, "exposed_unclassified_closures": closure_failures,
               "failure_codes": failures, "attempt_counts": dict(attempts), "merge_count": len(history),
               "rounds": rounds, "all_reference_entities_conservatively_contained": True,
               "same_source_part_positive_volume_overlap": "Excluded by exact projected-area conservation and unchanged terrain perimeter; numerical area tolerance 2e-8mm2 is disclosed.",
               "source_error_method": "Complete upper-envelope/source-triangle planar subdivision, equivalent to the seed halfspace LP. Maximum vertical source advance bounds bidirectional Euclidean exterior Hausdorff distance and directed added-material distance. Inward material error is zero because every reference cell is contained in its replacement hull.",
               "source_error_limitations": "Bounds cover this terrain source part. Whole-assembly exterior and cross-part ownership are independently evaluated by the global assembler. No physics/engine validation is performed.",
               "protected_features": {
                   "central_through_bore": "Exact xz footprint including face451 wedge retained; no footprint can bridge its void.",
                   "four_blind_recesses": "All original source material retained; upper-envelope source advance <=0.01mm. The exact y=-70mm mouth section and source floor/axis depth are checked in feature-witnesses.json.",
                   "root_slot": "No merge across x=+-3.7mm ear/bridge boundaries; y=-76.5mm ears and y=-73mm bridge rear surfaces unchanged.",
                   "assembly_side_channels": "Exact assembly y=-70mm section and ear outer side coordinates retain original mount/backing annuli and side channels; global cross-part verification remains required."},
               "numerical_tolerances": {"length_mm": EPS, "projected_area_mm2": AREA_EPS},
               "forbidden_operations_used": []}
    OUT.mkdir(parents=True, exist_ok=True)
    def write(name, data):
        (OUT/name).write_text(json.dumps(data, indent=2, default=lambda v: v.item())+"\n")
    # Independent fixed source clearance witnesses: vertical source axes and
    # radial mouth/shaft cores; no witness is used to change the candidate.
    witnesses = []
    def contains(p):
        return [c["name"] for c in ordered if np.max(c["eq"][:, :3]@p+c["eq"][:, 3]) < -EPS]
    for x, z in [(-6., 11.25), (-6., 19.25), (6., 11.25), (6., 19.25)]:
        for y in [-72.8, -72.5, -71.5, -70.225, -70.05]:
            p = np.array([x, y, z]); hits = contains(p)
            witnesses.append({"feature": f"blind_recess_{x:g}_{z:g}_axis", "point_m": (p/1000).tolist(), "expected": "void", "occupants": hits, "pass": not hits})
    for y in [-72.95, -71.5, -70.05]:
        for theta in np.linspace(0, 2*np.pi, 32, endpoint=False):
            p = np.array([1.35*np.cos(theta), y, 12.25+1.35*np.sin(theta)])
            hits = contains(p)
            witnesses.append({"feature": "central_bore_eroded_core", "point_m": (p/1000).tolist(), "expected": "void", "occupants": hits, "pass": not hits})
    for x in [-3.6, 0., 3.6]:
        for z in [4.1, 8., 16., 22., 26.4]:
            p = np.array([x, -74., z]); hits = contains(p)
            witnesses.append({"feature": "root_slot", "point_m": (p/1000).tolist(), "expected": "void", "occupants": hits, "pass": not hits})
    def axis_top(x, z, groups):
        maximum = -1e100
        for eqs in groups:
            lo, hi = -1e100, 1e100
            possible = True
            for e in eqs:
                d = e[0]*x+e[2]*z+e[3]
                if e[1] > 1e-9:
                    hi = min(hi, -d/e[1])
                elif e[1] < -1e-9:
                    lo = max(lo, -d/e[1])
                elif d > EPS:
                    possible = False
                    break
            if possible and hi >= lo-EPS:
                maximum = max(maximum, hi)
        return maximum
    axes = []
    original_eqs = [np.array(e["halfspaces_m"])*[1, 1, 1, 1000] for e in refs]
    for x, z in [(-6., 11.25), (-6., 19.25), (6., 11.25), (6., 19.25)]:
        before = axis_top(x, z, original_eqs)
        after = axis_top(x, z, [c["eq"] for c in ordered])
        axes.append({"axis_xz_mm": [x, z], "reference_floor_y_mm": before,
                     "runtime_floor_y_mm": after, "floor_advance_mm": after-before,
                     "pass": -EPS <= after-before <= budget["critical_working_surface_mm"]+EPS})
    feature_ok = all(w["pass"] for w in witnesses) and all(a["pass"] for a in axes)
    if not feature_ok:
        summary["failure_codes"].append("PROTECTED_FEATURE_WITNESS")
        summary["status"] = "BLOCKED"
    summary["protected_feature_witnesses_pass"] = feature_ok
    summary["four_blind_recess_axis_floor_advance_mm"] = [a["floor_advance_mm"] for a in axes]
    summary["assembly_y_minus70_section_area_residual_mm2"] = sum(c["top_area"] for c in ordered)-sum(top_section_area(np.array(e["vertices_m"])*1000, top_y) for e in refs)
    summary["construction"] = {
        "algorithm": "Deterministic adjacent reference-cell hull agglomeration with convex projected-footprint and exact assembly-section preservation",
        "source_seed_file": str(SEED_FILE.relative_to(ROOT)),
        "source_seed_sha256": hashlib.sha256(SEED_FILE.read_bytes()).hexdigest(),
        "fixed_face451_rule": "The owner of every nonprismatic face451 rear wedge is immutable; upper-terrain-only merge certificates cannot authorize its convexification.",
        "reference_cell_membership": "Every original reference cell appears in exactly one output reference_entity_ids list.",
        "insphere_method": "Maximum common slack of unit-normal hull halfspaces, solved as a four-variable linear program",
        "budget_selection": "Frozen preregistration 50um general, 10um recess terrain, zero deliberate movement of protected mouths, bore projection, slot and assembly section; no budget scan"
    }
    summary["verification"] = {
        "status": summary["status"], "failure_codes": summary["failure_codes"],
        "closed_convex_entities": all(e["closed_and_convex"] for e in entities),
        "all_reference_material_retained": True,
        "complete_source_face_mapping": set(all_source_map) == set(range(504)),
        "source_envelope_bound_mm": summary["maximum_source_outward_mm"],
        "unclassified_exposed_closure_count": len(closure_failures),
        "minimum_scale_failure_count": len(thin),
        "protected_feature_witnesses_pass": feature_ok,
        "engine_static_validation": "NOT_RUN"
    }
    write("partition.json", {**summary, "entities": entities})
    write("summary.json", summary)
    write("merge-history.json", {"merges": history})
    write("source-reference-mapping.json", {"source_face_ids": [{"source_face_id": sid, "runtime_entity_ids": sorted(all_source_map[sid])} for sid in range(504)],
          "reference_entities": [{"reference_entity_id": e["id"], "runtime_entity_id": owner[i]["id"]} for i, e in enumerate(refs)]})
    write("feature-witnesses.json", {"method": "Fixed frozen-source void core and axis points, plus exact one-dimensional halfspace intervals on all four recess axes. Complementary to complete terrain error and exact section certificates.", "all_pass": feature_ok, "blind_recess_axis_floors": axes, "witnesses": witnesses})
    print(json.dumps({k: v for k, v in summary.items() if k not in ["minimum_scale_failures", "exposed_unclassified_closures"]}, indent=2, default=lambda v: v.item()))


if __name__ == "__main__":
    main()
