#!/usr/bin/env python3
"""Build one offline runtime backing candidate from the frozen reference cells.

No simulator is imported.  A merge retains the hull of every original vertex.
The continuous outward-solid certificate covers the new hull by the union of
the ORIGINAL cells Minkowski-summed with a cube of Euclidean radius < budget.
This solid certificate is distinct from bidirectional UNION EXTERIOR distance.
"""
from __future__ import annotations

import argparse
from collections import Counter
from hashlib import sha256
from itertools import product
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull, QhullError

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "debug/issue25/runtime-partition/backing"
ARITH_MM = 1e-9
CERTIFICATE_RESERVE_MM = 1e-5
MAX_COVER_PIECES = 1800


def write_json(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")


def poly(vertices):
    v = np.unique(np.asarray(vertices, dtype=float), axis=0)
    if len(v) < 4:
        return None
    try:
        h = ConvexHull(v)
    except QhullError:
        return None
    if h.volume <= 1e-16:
        return None
    v = v[h.vertices]
    h = ConvexHull(v)
    eq = h.equations
    # Qhull triangulates polygonal facets; duplicate supporting planes are not
    # separate clipping constraints.  Retaining one does not alter the body.
    _, ix = np.unique(np.round(eq, 10), axis=0, return_index=True)
    eq = eq[np.sort(ix)]
    edge_set = set()
    for f in h.simplices:
        for a, b in zip(f, np.roll(f, -1)):
            edge_set.add(tuple(sorted((int(a), int(b)))))
    return {"v": v, "h": h, "eq": eq,
            "edges": np.array(sorted(edge_set), dtype=int),
            "lo": v.min(0), "hi": v.max(0)}


def clip_body(p, eq):
    """Convex clipping.  The numerical reserve is included in the budget."""
    v = p["v"]
    q = v @ eq[:3] + eq[3]
    if q.max() <= ARITH_MM:
        return p
    if q.min() > ARITH_MM:
        return None
    inside = q <= 0
    ev = p["edges"]
    crossing = ev[inside[ev[:, 0]] != inside[ev[:, 1]]]
    points = list(v[inside])
    for a, b in crossing:
        points.append(v[a] + (v[b] - v[a]) * (-q[a] / (q[b] - q[a])))
    return poly(points) if len(points) >= 4 else None


def subtract_body(p, q):
    """Disjoint convex pieces of p minus convex q (common faces allowed)."""
    if (p["hi"] < q["lo"] - ARITH_MM).any() or (p["lo"] > q["hi"] + ARITH_MM).any():
        return [p]
    distances = p["v"] @ q["eq"][:, :3].T + q["eq"][:, 3]
    if (distances.min(0) > ARITH_MM).any():
        return [p]
    if distances.max() <= ARITH_MM:
        return []
    remainder = p
    outside = []
    # Most separating planes first reduces unnecessary fragmentation.
    order = np.argsort(-distances.max(0), kind="stable")
    for j in order:
        eq = q["eq"][j]
        out = clip_body(remainder, -eq)
        if out is not None:
            outside.append(out)
        remainder = clip_body(remainder, eq)
        if remainder is None:
            break
    return outside


def inradius(p):
    eq = p["eq"]
    lp = linprog([0., 0., 0., -1.],
                 A_ub=np.column_stack((eq[:, :3], np.ones(len(eq)))),
                 b_ub=-eq[:, 3], bounds=[(None, None)] * 3 + [(0, None)],
                 method="highs")
    if not lp.success:
        raise RuntimeError("Chebyshev-centre LP failed: " + lp.message)
    return float(lp.x[3]), lp.x[:3]


def make_expanded(p, budget_mm):
    a = (budget_mm - CERTIFICATE_RESERVE_MM) / np.sqrt(3.)
    corners = np.array(list(product((-a, a), repeat=3)))
    return poly((p["v"][:, None, :] + corners[None, :, :]).reshape(-1, 3))


def certify(hull, ids, expanded, original):
    """All-dimensional continuous cover, not a vertex-only approximation."""
    ids = sorted(ids, key=lambda k: (-original[k]["h"].volume, k))
    # Fixed deterministic quick rejection.  Coverage itself is checked below.
    v = hull["v"]
    f = hull["h"].simplices
    samples = np.vstack((v, v[f].mean(1), v.mean(0)[None, :]))
    covered = np.zeros(len(samples), bool)
    for k in ids:
        q = expanded[k]
        covered |= (samples @ q["eq"][:, :3].T + q["eq"][:, 3]).max(1) <= ARITH_MM
    if not covered.all():
        return False, "fixed_point_prefilter", 0
    pieces = [hull]
    peak = 1
    for k in ids:
        pieces = [out for p in pieces for out in subtract_body(p, expanded[k])]
        peak = max(peak, len(pieces))
        if not pieces:
            return True, "continuous_original_cell_minkowski_cube_cover", peak
        if len(pieces) > MAX_COVER_PIECES:
            return False, "continuous_cover_piece_limit", peak
    return False, "continuous_cover_left_material", peak


def outward_faces(p):
    v = p["v"]
    h = p["h"]
    # Near-collinear source-overlay vertices can make the default merged
    # Qhull facet triangulation have an unstable cross-product normal.  Q0
    # re-triangulates the SAME coordinates without the problematic facet merge.
    if triangle_support_residual(v, h) > 1e-7:
        h = ConvexHull(v, qhull_options="Q0")
        if triangle_support_residual(v, h) > 1e-7:
            raise RuntimeError("No stable outward hull triangulation")
    faces = []
    for f, eq in zip(h.simplices, h.equations):
        f = f.copy()
        t = v[f]
        if np.dot(np.cross(t[1] - t[0], t[2] - t[0]), eq[:3]) < 0:
            f = f[[0, 2, 1]]
        faces.append(f.tolist())
    return faces


def triangle_support_residual(v, h):
    worst = -np.inf
    for f, eq in zip(h.simplices, h.equations):
        t = v[f]
        n = np.cross(t[1] - t[0], t[2] - t[0])
        norm = np.linalg.norm(n)
        if norm == 0:
            return np.inf
        n /= norm
        if n @ eq[:3] < 0:
            n = -n
        worst = max(worst, float(((v - t[0]) @ n).max()))
    return worst


def closed_check(faces):
    c = Counter(tuple(sorted((a, b))) for f in faces for a, b in zip(f, f[1:] + f[:1]))
    return all(v == 2 for v in c.values())


def point_inside_any(points, groups):
    points = np.asarray(points)
    inside = np.zeros(len(points), bool)
    for g in groups.values():
        p = g["poly"]
        eligible = ((points >= p["lo"] - ARITH_MM) & (points <= p["hi"] + ARITH_MM)).all(1)
        ix = np.flatnonzero(eligible & ~inside)
        if len(ix):
            inside[ix] = (points[ix] @ p["eq"][:, :3].T + p["eq"][:, 3]).max(1) <= ARITH_MM
    return inside


def segment_clearance(a, b, groups):
    """Exact LP intersection of each closed entity and a finite segment."""
    a, b = np.asarray(a), np.asarray(b)
    hits = []
    clearance = np.inf
    for gid, g in groups.items():
        p = g["poly"]
        eq = p["eq"]
        q0 = a @ eq[:, :3].T + eq[:, 3]
        slope = (b - a) @ eq[:, :3].T
        lo, hi = 0., 1.
        for d, s in zip(q0, slope):
            if abs(s) < 1e-15:
                if d > ARITH_MM:
                    hi = -1.
                    break
            elif s > 0:
                hi = min(hi, (ARITH_MM - d) / s)
            else:
                lo = max(lo, (ARITH_MM - d) / s)
        if lo <= hi and hi >= 0 and lo <= 1:
            hits.append(int(gid))
        # A supporting plane separating the complete segment provides a lower
        # bound on clearance.  This is conservative and independently useful.
        lower = float(np.maximum(np.minimum(q0, q0 + slope), 0).max())
        clearance = min(clearance, lower)
    return {"clear": not hits, "intersecting_group_ids": hits,
            "conservative_separating_plane_clearance_mm": float(clearance)}


def void_witnesses(groups):
    records = []
    for name, x, z, y0, y1 in [
        ("positive_x_lower_y_bore", 6., 11.25, -70., -64.),
        ("positive_x_upper_y_bore", 6., 19.25, -70., -64.),
        ("negative_x_lower_y_bore", -6., 11.25, -70., -64.),
        ("negative_x_upper_y_bore", -6., 19.25, -70., -64.),
        ("positive_x_side_y_bore", 14.65, 15.25, -70.5, -65.),
    ]:
        # Axis and four parallel corridors 0.8 mm from the axis remain inside
        # the source polygonal bore after the declared 0.05 mm erosion.
        probes = []
        for dx, dz in [(0, 0), (.8, 0), (-.8, 0), (0, .8), (0, -.8)]:
            a, b = [x + dx, y0, z + dz], [x + dx, y1, z + dz]
            probes.append({"a_mm": a, "b_mm": b, **segment_clearance(a, b, groups)})
        records.append({"feature": name, "type": "continuous_finite_segments",
                        "all_clear": all(p["clear"] for p in probes), "probes": probes})
    # Pocket witnesses are inside the actual central recess and join its mouth.
    probes = []
    for x in (-8., 0., 8.):
        for y in (-55., -40., -25.):
            a, b = [x, y, 10.], [x, y, 30.]
            probes.append({"a_mm": a, "b_mm": b, **segment_clearance(a, b, groups)})
    records.append({"feature": "large_rear_pocket_opening", "type": "continuous_finite_segments",
                    "all_clear": all(p["clear"] for p in probes), "probes": probes})
    return records


def build(out):
    started = time.time()
    out.mkdir(parents=True, exist_ok=True)
    refpath = ROOT / "reports/nero-gripper-entity-partition.json"
    preregpath = ROOT / "reports/nero-gripper-runtime-preregistration.json"
    prereg = json.loads(preregpath.read_text())
    ref = json.loads(refpath.read_text())
    if sha256(refpath.read_bytes()).hexdigest() != prereg["reference_sha256"]:
        raise RuntimeError("Reference SHA256 does not match frozen preregistration")
    budget = prereg["budget"]
    es = [e for e in ref["entities"] if e["source_part"] == "backing"]
    original = [poly(np.array(e["vertices_m"]) * 1000.) for e in es]
    if any(p is None for p in original):
        raise RuntimeError("Reference cell failed convex construction")
    name_ix = {e["name"]: i for i, e in enumerate(es)}
    expanded = {}
    for critical in (False, True):
        delta = budget["critical_working_surface_mm"] if critical else budget["maximum_outward_mm"]
        expanded[critical] = [make_expanded(p, delta) for p in original]
    groups = {}
    for i, (e, p) in enumerate(zip(es, original)):
        rad, centre = inradius(p)
        groups[i] = {"ids": {i}, "poly": p, "radius": rad, "centre": centre,
                     "family": e["semantic_family"], "features": {e["feature"]},
                     "neighbors": {name_ix[n] for n in e["neighbors"] if n in name_ix},
                     "version": 0, "critical": e["semantic_family"] == "distal_tip_contact"}
    original_volume = sum(p["h"].volume for p in original)
    logs, rejects = [], Counter()
    tried = set()
    for rnd in range(30):
        accepted = 0
        ordered = sorted(groups, key=lambda k: (2 * groups[k]["radius"] >= budget["minimum_entity_inscribed_diameter_mm"],
                                                groups[k]["poly"]["h"].volume, k))
        for i in ordered:
            if i not in groups:
                continue
            a = groups[i]
            candidates = [j for j in a["neighbors"] if j in groups and j != i
                          and groups[j]["family"] == a["family"]]
            candidates.sort(key=lambda j: (not bool(a["features"] & groups[j]["features"]),
                                           -groups[j]["poly"]["h"].volume, j))
            for j in candidates:
                b = groups[j]
                key = (min(i, j), max(i, j), groups[min(i, j)]["version"], groups[max(i, j)]["version"])
                if key in tried:
                    continue
                tried.add(key)
                p = poly(np.vstack((a["poly"]["v"], b["poly"]["v"])))
                ids = a["ids"] | b["ids"]
                critical = a["critical"] or b["critical"]
                ok, reason, peak = certify(p, ids, expanded[critical], original)
                if not ok:
                    rejects[reason] += 1
                    continue
                rad, centre = inradius(p)
                # IDs remain stable; surviving group j owns the full material.
                merged_neighbors = (a["neighbors"] | b["neighbors"]) - {i, j}
                for n in merged_neighbors:
                    if n in groups:
                        groups[n]["neighbors"].discard(i)
                        groups[n]["neighbors"].add(j)
                groups[j] = {"ids": ids, "poly": p, "radius": rad, "centre": centre,
                             "family": a["family"], "features": a["features"] | b["features"],
                             "neighbors": merged_neighbors, "version": b["version"] + 1,
                             "critical": critical}
                del groups[i]
                delta = budget["critical_working_surface_mm"] if critical else budget["maximum_outward_mm"]
                logs.append({"round": rnd, "consumed_group": i, "surviving_group": j,
                             "reference_cell_count": len(ids), "certificate": reason,
                             "outward_solid_distance_bound_mm": delta,
                             "continuous_cover_peak_pieces": peak,
                             "new_inscribed_diameter_mm": 2 * rad})
                accepted += 1
                break
        print(json.dumps({"round": rnd, "accepted_merges": accepted, "remaining": len(groups),
                          "thin_remaining": sum(2 * g["radius"] < .1 - 1e-8 for g in groups.values()),
                          "elapsed_s": time.time() - started}), flush=True)
        if not accepted:
            break
    entities = []
    final_certificates = []
    for k, g in sorted(groups.items(), key=lambda kv: (kv[1]["family"], tuple(kv[1]["poly"]["v"].mean(0)))):
        p = g["poly"]
        ids = sorted(g["ids"])
        critical = g["critical"]
        delta = budget["critical_working_surface_mm"] if critical else budget["maximum_outward_mm"]
        if len(ids) > 1:
            ok, why, peak = certify(p, ids, expanded[critical], original)
        else:
            ok, why, peak = True, "unchanged_reference_convex_cell", 0
            delta = 0.
        if not ok:
            raise RuntimeError("Final continuous certificate changed for " + str(k))
        centre = p["v"].mean(0)
        dominant = max(ids, key=lambda i: original[i]["h"].volume)
        feature = es[dominant]["feature"]
        name = "runtime_backing_" + g["family"] + "_" + feature + "_" + "_".join(f"{x:+.6f}" for x in centre)
        faces = outward_faces(p)
        own_refs = [es[i]["id"] for i in ids]
        source_ids = sorted({s for i in ids for s in es[i]["source_triangle_ids"]})
        regions = sorted({s for i in ids for s in es[i]["source_regions"]})
        local_reference_volume = sum(original[i]["h"].volume for i in ids)
        containment_max = max(float((original[i]["v"] @ p["eq"][:, :3].T + p["eq"][:, 3]).max()) for i in ids)
        record = {"name": name, "source_part": "backing", "semantic_family": g["family"],
                  "feature": feature, "merged_features": sorted(g["features"]),
                  "vertices_m": (p["v"] / 1000.).tolist(), "faces": faces,
                  "halfspaces_m": np.column_stack((p["eq"][:, :3], p["eq"][:, 3] / 1000.)).tolist(),
                  "reference_entity_ids": own_refs, "source_triangle_ids": source_ids,
                  "source_regions": regions, "inscribed_diameter_mm": 2 * g["radius"],
                  "inscribed_sphere_center_m": (g["centre"] / 1000.).tolist(),
                  "volume_m3": float(p["h"].volume * 1e-9), "closed_and_convex": closed_check(faces),
                  "min_facet_normal_width_mm": float(np.ptp(p["v"] @ p["eq"][:, :3].T, axis=0).min()),
                  "error_certificate": {"method": why, "reference": "frozen original cells, never progressive hulls",
                    "outward_solid_distance_bound_mm": delta,
                    "reference_vertex_containment_max_halfspace_residual_mm": containment_max,
                    "additional_hull_material_vs_disjoint_reference_sum_mm3": float(p["h"].volume - local_reference_volume),
                    "continuous_cover_peak_pieces": peak,
                    "exterior_surface_distance": "requires independent global union-exterior extraction"}}
        entities.append(record)
        final_certificates.append({"runtime_entity": name, **record["error_certificate"]})
    witnesses = void_witnesses(groups)
    reference_witnesses = void_witnesses({i: {"poly": p} for i, p in enumerate(original)})
    thin = [{"name": e["name"], "inscribed_diameter_mm": e["inscribed_diameter_mm"],
             "reference_entity_ids": e["reference_entity_ids"]}
            for e in entities if e["inscribed_diameter_mm"] < budget["minimum_entity_inscribed_diameter_mm"] - 1e-8]
    containment = max(e["error_certificate"]["reference_vertex_containment_max_halfspace_residual_mm"] for e in entities)
    verification = {
        "status": "BLOCKED" if thin or not all(w["all_clear"] for w in witnesses) else "LOCAL_SOLID_CHECKS_PASS_GLOBAL_EXTERIOR_PENDING",
        "reference_entity_count": len(es), "runtime_entity_count": len(entities),
        "reduction_percent": 100 * (1 - len(entities) / len(es)),
        "families": dict(Counter(e["semantic_family"] for e in entities)),
        "all_reference_ids_mapped_exactly_once": sorted(x for e in entities for x in e["reference_entity_ids"]) == sorted(e["id"] for e in es),
        "all_source_face_ids_retained": sorted({x for e in entities for x in e["source_triangle_ids"]}) == list(range(660, 2086)),
        "all_closed_convex": all(e["closed_and_convex"] for e in entities),
        "reference_material_containment_max_residual_mm": containment,
        "minimum_entity_inscribed_diameter_mm": min(e["inscribed_diameter_mm"] for e in entities),
        "subminimum_entities": thin,
        "reference_disjoint_cell_volume_sum_mm3": original_volume,
        "runtime_cell_volume_sum_mm3": sum(e["volume_m3"] for e in entities) * 1e9,
        "volume_note": "Runtime hulls can overlap. Cell-volume sums are not the runtime union volume.",
        "solid_directed_distance_bounds_mm": {"reference_material_to_runtime_material": 0., "runtime_material_to_reference_material": budget["maximum_outward_mm"]},
        "exterior_surface_distance_status": "PENDING_INDEPENDENT_GLOBAL_UNION_EXTERIOR",
        "exterior_surface_note": "Material inclusion does not imply reference exterior to runtime exterior distance is zero. No inward exterior claim is made here.",
        "protected_void_witnesses": witnesses,
        "same_witnesses_on_frozen_reference": reference_witnesses,
        "witness_scope": "Five complete bore axes with four offset continuous segments; nine pocket-to-mouth segments. Global eroded void-core and complete exterior bounds remain root-integration checks.",
        "rejected_merge_counts": dict(rejects), "elapsed_s": time.time() - started,
    }
    construction = {
        "method": "Deterministic same-family reference-adjacency agglomeration; subminimum inscribed diameter first; original-cell Minkowski cube continuous 3D coverage for every accepted hull.",
        "preregistration_sha256": sha256(preregpath.read_bytes()).hexdigest(),
        "reference_sha256": sha256(refpath.read_bytes()).hexdigest(),
        "source_sha256": sha256((ROOT / "meshes/source/agx_gripper/meshes/gripper_link1.stl").read_bytes()).hexdigest(),
        "coordinate_frame": "unchanged frozen gripper_link1 source frame", "units_geometry": "m", "units_calculation": "mm",
        "minkowski_cube": "Each original cell + cube [-a,a]^3 with a=(budget-1e-5mm)/sqrt(3). Every point in the expanded cell is within budget-1e-5mm Euclidean distance of that original cell.",
        "continuous_cover": "Subtract each expanded ORIGINAL convex cell from candidate hull as convex halfspace pieces. Accept only if no 3D remainder survives; no vertex-only or volume-only acceptance.",
        "arithmetic_halfspace_resolution_mm": ARITH_MM,
        "certificate_budget_reserve_mm": CERTIFICATE_RESERVE_MM,
        "cover_piece_limit": MAX_COVER_PIECES,
        "unchanged_thin_material_policy": "Keep and report BLOCKED; no small cell is deleted.",
        "critical_family": "distal_tip_contact uses 10um solid-distance bound; root integration checks original critical normals/exterior.",
        "one_candidate_no_physical_feedback": True,
    }
    write_json(out / "backing-runtime-entities.json", {"schema_version": 1, "source_part": "backing", "entities": entities,
               "verification": verification, "construction": construction})
    write_json(out / "merge-history.json", {"accepted_merges": logs, "rejections": dict(rejects)})
    write_json(out / "continuous-error-certificates.json", {"construction": construction, "entities": final_certificates})
    write_json(out / "verification.json", verification)
    print(json.dumps({k: v for k, v in verification.items() if k not in {"subminimum_entities", "protected_void_witnesses", "same_witnesses_on_frozen_reference"}}, indent=2), flush=True)


def verify_existing(out):
    """Reload emitted geometry and audit it without regenerating a candidate."""
    path = out / "backing-runtime-entities.json"
    data = json.loads(path.read_text())
    reference = json.loads((ROOT / "reports/nero-gripper-entity-partition.json").read_text())
    ref = {e["id"]: e for e in reference["entities"] if e["source_part"] == "backing"}
    checks = Counter()
    failures = []
    burial = []
    radius_differences = []
    containment_max = -np.inf
    all_refs = []
    for e in data["entities"]:
        v = np.array(e["vertices_m"]) * 1000
        faces = e["faces"]
        h = ConvexHull(v)
        eq = h.equations
        finite = np.isfinite(v).all()
        closed = closed_check(faces)
        outward = True
        for f in faces:
            t = v[f]
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n /= np.linalg.norm(n)
            outward &= bool(((v - t[0]) @ n).max() < 1e-7)
        checks["finite_entities"] += int(finite)
        checks["closed_entities"] += int(closed)
        checks["outward_convex_entities"] += int(outward)
        if not (finite and closed and outward):
            failures.append(e["name"])
        lp = linprog([0, 0, 0, -1], A_ub=np.column_stack((eq[:, :3], np.ones(len(eq)))),
                     b_ub=-eq[:, 3], bounds=[(None, None)] * 3 + [(0, None)], method="highs")
        if not lp.success:
            failures.append(e["name"] + ": inscribed sphere LP failed")
        else:
            radius_differences.append(abs(2 * lp.x[3] - e["inscribed_diameter_mm"]))
        all_refs.extend(e["reference_entity_ids"])
        for rid in e["reference_entity_ids"]:
            r = ref[rid]
            residual = float((np.array(r["vertices_m"]) * 1000 @ eq[:, :3].T + eq[:, 3]).max())
            containment_max = max(containment_max, residual)
            for sp in r["source_surface_partition"]:
                for patch in sp["exterior_subpolygons"]:
                    points = np.array(patch["polygon_m"]) * 1000
                    samples = np.vstack((points, points.mean(0)[None, :]))
                    # Distance to this CONVEX owning hull boundary for interior
                    # points is the least normalized supporting-plane slack.
                    # It is a lower bound on distance to the whole union's
                    # exterior; report this distinction explicitly.
                    depth = -(samples @ eq[:, :3].T + eq[:, 3]).max(1)
                    if depth.max() > 1e-7:
                        burial.append({"reference_entity_id": rid, "runtime_entity": e["name"],
                            "source_face_id": sp["source_face_id"],
                            "reference_exterior_patch_area_mm2": patch["area_m2"] * 1e6,
                            "max_vertex_or_centroid_depth_in_own_hull_mm": float(depth.max()),
                            "min_vertex_or_centroid_depth_in_own_hull_mm": float(depth.min()),
                            "polygon_m": patch["polygon_m"],
                            "interpretation": "Original exterior partly buried by added hull material; own-hull interior depth is not a global union exterior-distance upper bound."})
    result = {"status": "PASS" if not failures and sorted(all_refs) == sorted(ref) and containment_max < 1e-7 else "FAIL",
        "artifact_sha256": sha256(path.read_bytes()).hexdigest(),
        "entity_count": len(data["entities"]), "checks": dict(checks), "failures": failures,
        "reference_ids_mapped_exactly_once": sorted(all_refs) == sorted(ref),
        "independently_recomputed_reference_vertex_containment_max_residual_mm": containment_max,
        "max_recomputed_inscribed_diameter_difference_mm": max(radius_differences),
        "partly_buried_reference_exterior_patch_count": len(burial),
        "max_sampled_own_hull_burial_depth_mm": max((p["max_vertex_or_centroid_depth_in_own_hull_mm"] for p in burial), default=0.),
        "minimum_scale_requirement_status": data["verification"]["status"],
        "surface_distance_scope": "Burial samples expose changed old exterior and cannot establish the required global bidirectional exterior bound.",
        "microseam_policy": "Reference contact504..541/backing660..697 interface wedge up to 9.005483698629191um has no independent named mechanical meaning; frozen preregistration permits absorption. Do not claim preservation of all original microseams.",
        "protected_void_core_implication": "Continuous coverage by original material plus a Euclidean <0.05mm offset excludes runtime material from EVERY reference void core eroded by 0.05mm. Explicit continuous axis witnesses independently check named passage connectivity; global assembly unions remain a separate validation.",
    }
    write_json(out / "independent-geometry-verification.json", result)
    write_json(out / "buried-reference-exterior-patches.json", {"metric_scope": result["surface_distance_scope"], "patches": burial})
    print(json.dumps(result, indent=2), flush=True)


def repair_triangulation(out):
    """Representation-only fix; no group, vertex, or accepted merge changes."""
    path = out / "backing-runtime-entities.json"
    before = path.read_bytes()
    data = json.loads(before)
    repairs = []
    for e in data["entities"]:
        v = np.array(e["vertices_m"]) * 1000
        old_h = ConvexHull(v)
        old_residual = triangle_support_residual(v, old_h)
        if old_residual <= 1e-7:
            continue
        h = ConvexHull(v, qhull_options="Q0")
        residual = triangle_support_residual(v, h)
        if residual > 1e-7:
            raise RuntimeError("Triangulation-only repair failed")
        fixes = {"name": e["name"], "vertices_unchanged": True,
                 "reference_groups_unchanged": True,
                 "old_face_count": len(e["faces"]), "new_face_count": len(h.simplices),
                 "old_triangle_support_residual_mm": old_residual,
                 "new_triangle_support_residual_mm": residual,
                 "hull_volume_change_mm3": float(h.volume - old_h.volume)}
        e["faces"] = outward_faces({"v": v, "h": h})
        e["halfspaces_m"] = np.column_stack((h.equations[:, :3], h.equations[:, 3] / 1000.)).tolist()
        e["triangulation_note"] = "Q0 triangulation of unchanged candidate vertices; default facet triangulation had near-collinear triangles."
        e["closed_and_convex"] = closed_check(e["faces"])
        if not e["closed_and_convex"]:
            raise RuntimeError("Repair created an open surface")
        repairs.append(fixes)
    if repairs:
        prior_verify = out / "independent-geometry-verification.json"
        if prior_verify.exists():
            (out / "independent-geometry-verification-before-triangulation-repair.json").write_bytes(prior_verify.read_bytes())
        data["construction"]["triangulation_representation_repair"] = repairs
        write_json(path, data)
    record = {"representation_only": True, "candidate_selection_and_vertices_unchanged": True,
              "before_artifact_sha256": sha256(before).hexdigest(),
              "after_artifact_sha256": sha256(path.read_bytes()).hexdigest(),
              "repairs": repairs}
    write_json(out / "triangulation-repair.json", record)
    print(json.dumps(record, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--verify-existing", action="store_true")
    parser.add_argument("--repair-triangulation", action="store_true")
    args = parser.parse_args()
    if args.repair_triangulation:
        repair_triangulation(args.output)
    elif args.verify_existing:
        verify_existing(args.output)
    else:
        build(args.output)


if __name__ == "__main__":
    main()
