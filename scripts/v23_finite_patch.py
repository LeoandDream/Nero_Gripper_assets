"""One finite-source-patch candidate from the ORIGINAL 346-cell baseline.

The source plane is applied only to the cell/prism intersection. Every
outside-prism branch is retained. A split tree records this set identity;
no whole-cell source-plane replacement or global reconstruction is used.
"""
from pathlib import Path
import argparse
import collections
import copy
import hashlib
import json
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import ConvexHull, QhullError

from v22_local_repartition import patches, geosha
from v21_boundary_atlas import support_cells, basis, NUM
from runtime_backing_partition import inradius

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'debug/issue25/v2-runtime/vendor'))
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient
from shapely import constrained_delaunay_triangles, set_precision

TASK = ROOT / 'debug/issue25/v23-finite'
PRE = ROOT / 'reports/nero-gripper-v23-preregistration.json'
OUT = ROOT / 'reports/nero-gripper-v23-finite-partition.json'
TRACE = ROOT / 'reports/nero-gripper-v23-split-trace.json'
BASE = ROOT / 'reports/nero-gripper-v2-runtime-partition.json'
ALLOWED = set('V20140 V20143 V20154 V20157 V20160 V20166 V20175 V20205 V20207 V20245 V20259 V20263 V20267 V20273 V20274 V20280 V20284 V20291 V20294 V20297 V20301 V20304 V20317 V20322 V20324 V20329 V20330 V20333 V20334 V20337 V20339 V20344'.split())
EPS = NUM / 1024
FOOT_RADIUS = NUM / 64
GRID = NUM / 4096


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n')


def body(vertices):
    """Scaled hull of identical input coordinates; no joggle or inflation."""
    v = np.unique(np.asarray(vertices, dtype=float), axis=0)
    if len(v) < 4:
        return None
    origin = v.mean(0)
    scale = np.ptp(v, axis=0)
    if np.any(scale <= 0):
        return None
    try:
        h = ConvexHull((v - origin) / scale)
    except QhullError:
        return None
    used = h.vertices
    remap = {int(j): i for i, j in enumerate(used)}
    faces, rows = [], []
    for f, row in zip(h.simplices, h.equations):
        n = row[:3] / scale
        length = np.linalg.norm(n)
        d = (row[3] - n @ origin) / length
        n /= length
        t = v[f]
        if np.cross(t[1] - t[0], t[2] - t[0]) @ n < 0:
            f = f[[0, 2, 1]]
        faces.append([remap[int(j)] for j in f])
        rows.append(np.r_[n, d])
    vv = v[used]
    eq = np.array(rows)
    _, indices = np.unique(np.round(eq, 12), axis=0, return_index=True)
    eq = eq[np.sort(indices)]
    edges = sorted({tuple(sorted((a, b))) for f in faces for a, b in zip(f, f[1:] + f[:1])})
    return {'v': vv, 'faces': faces, 'eq': eq, 'edges': edges,
            'volume_mm3': float(h.volume * np.prod(scale))}


def cut(p, row):
    values = p['v'] @ row[:3] + row[3]
    if values.max() <= EPS:
        return p, 'UNCHANGED_WITHIN_FIXED_RESERVE'
    if values.min() > 0:
        return None, 'EMPTY_AFFINE_VERTEX_MINIMUM'
    inside = values <= 0
    points = list(p['v'][inside])
    for a, b in p['edges']:
        if inside[a] == inside[b]:
            continue
        va, vb = p['v'][a].astype(np.longdouble), p['v'][b].astype(np.longdouble)
        rr = row.astype(np.longdouble)
        da, db = va @ rr[:3] + rr[3], vb @ rr[:3] + rr[3]
        points.append(np.asarray(va + (vb - va) * (da / (da - db)), dtype=float))
    result = body(points)
    return result, 'NONDEGENERATE_CLIP' if result is not None else 'NUMERIC_OR_DEGENERATE_REQUIRES_CERTIFICATION'


def footprint_parts(q, n, d):
    u, w = basis(n)
    shape = set_precision(Polygon(np.c_[q @ u, q @ w]), GRID)
    if not shape.is_valid or shape.is_empty:
        raise ValueError('Invalid frozen source footprint; do not guess its topology')
    parts = [shape] if shape.symmetric_difference(shape.convex_hull).is_empty else list(constrained_delaunay_triangles(shape).geoms)
    result = []
    delta = FOOT_RADIUS / np.sqrt(2)
    corners = np.array([[-delta, -delta], [-delta, delta], [delta, -delta], [delta, delta]])
    for part in parts:
        xy = np.array(part.exterior.coords)[:-1]
        # Minkowski sum of this convex footprint with a fixed square. Its
        # Euclidean radius is FOOT_RADIUS even at acute corners.
        points = (xy[:, None, :] + corners[None, :, :]).reshape(-1, 2)
        expanded = orient(Polygon(points[ConvexHull(points).vertices]), sign=1)
        xy = np.array(expanded.exterior.coords)[:-1]
        xyz = xy[:, 0, None] * u + xy[:, 1, None] * w - d * n
        side = []
        for a, b in zip(xyz, np.roll(xyz, -1, axis=0)):
            out = np.cross(b - a, n)
            out /= np.linalg.norm(out)
            side.append(np.r_[out, -out @ a])
        result.append({'source_convex_piece_xy_mm':np.array(part.exterior.coords)[:-1].tolist(),
                       'expanded_polygon_mm':xyz.tolist(), 'side_halfspaces_mm':np.array(side).tolist()})
    return result


def preflight():
    if OUT.exists():
        raise SystemExit('Candidate already exists; no new construction or preregistration allowed')
    TASK.mkdir(parents=True, exist_ok=True)
    raw = subprocess.check_output(['git','show','6eba4f3:reports/nero-gripper-v2-runtime-partition.json'], cwd=ROOT)
    assert raw == BASE.read_bytes()
    cells = json.loads(raw)['entities']
    assert geosha(cells) == '79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e'
    pp, _ = patches()
    assert len(pp) == 777 and len({p['source_face'] for p in pp}) == 326
    bodies, _ = support_cells(cells)
    prisms, forbidden, tiny = [], [], []
    from v21_boundary_atlas import clip
    for patch in pp:
        q = np.array(patch['polygon_mm'])
        plane = np.array(patch['plane']); n, d = plane[:3], plane[3]
        for b in bodies:
            probe = q + 2 * NUM * n
            if np.any(b['hi'] < probe.min(0)) or np.any(b['lo'] > probe.max(0)):
                continue
            section = probe.copy()
            for row in b['equations']:
                section = clip(section, row)
                if len(section) < 3:
                    break
            if len(section) < 3:
                continue
            area = float(np.linalg.norm(np.cross(section[1:] - section[0], np.roll(section, -1, axis=0)[1:] - section[0]).sum(0)) / 2)
            if area <= 1e-16:
                tiny.append({'patch':patch['id'],'entity':b['id'],'area_mm2':area,'status':'UNKNOWN_NUMERICAL_PREFILTER_NOT_ACCEPTANCE'})
                continue
            if b['id'] not in ALLOWED:
                forbidden.append({'patch':patch['id'],'entity':b['id'],'source_face':patch['source_face'],'action':'UNCHANGED'})
                continue
            height = b['vertices'] @ n + d
            for part in footprint_parts(q, n, d):
                low, high = float(height.min() - NUM / 16), float(height.max() + NUM / 16)
                # The cut volume is only the outward cap inside a finite
                # polygon prism. Top/bottom are finite and root-bounded.
                upper = np.r_[n, d - high]
                lower = np.r_[-n, -d + low]
                sides = part['side_halfspaces_mm']
                prisms.append({'id':f"P{len(prisms)+1:04d}",'baseline_entity':b['id'],'patch':patch['id'],
                               'source_face':patch['source_face'],'source_region':patch['region'],
                               'source_plane_mm':plane.tolist(),'source_polygon_mm':patch['polygon_mm'],
                               **part,'axial_range_mm':[low,high],
                               'influence_halfspaces_mm':sides+[upper.tolist(),lower.tolist()],
                               'carve_halfspaces_mm':sides+[upper.tolist(),lower.tolist(),(-plane).tolist()]})
    v22 = json.loads((ROOT/'reports/nero-gripper-v22-local-verification.json').read_text())
    pre = {'task':'https://github.com/LeoandDream/Nero_Gripper_assets/issues/25#issuecomment-6274041658',
           'baseline_geometry_commit':'6eba4f34c5ff5029eb849804e3d2e9c535f58756',
           'baseline_file_sha256':sha(BASE),'baseline_geometry_sha256':geosha(cells),
           'failed_v22_geometry_is_not_input':True,'candidate_limit':1,
           'allowed_baseline_entities':sorted(ALLOWED),'extra_authorized_neighbors':[],
           'patch_count':len(pp),'source_face_count':326,'influence_prisms':prisms,
           'forbidden_potential_coverers_kept_unchanged':forbidden,'tiny_intersections_unknown':tiny,
           'numerical_support_mm':NUM,'clip_arithmetic_mm':EPS,'footprint_grid_mm':GRID,
           'footprint_expansion_Euclidean_radius_mm':FOOT_RADIUS,
           'source_plane_movement_mm':0,
           'numeric_branch_rule':'A component disjoint from a cap by one affine vertex minimum>=-EPS is kept whole. Every positive-dimensional retained fragment is kept even if too thin; degenerate/empty branches remain explicit trace obligations, not silently discarded. Fixed arithmetic EPS is not a new mechanical budget.',
           'construction_rule':'Start each allowed root from its original346-cell vertices/faces. For each registered finite cap, split each current convex component along each prism side/top plane and then the source plane. Retain EVERY outside branch. Discard only the final leaf inside the registered finite outward cap. Keep the below-source local leaf; never substitute a whole-cell source-plane clip. No leaf is removed for failing the0.1mm scale rule.',
           'local_coalescing_rule':'Only within one baseline root, greedily coalesce two retained pieces if a shared opposite support plane exists and every other support halfspace contains all vertices of the other piece. This is an algebraic convex-union identity modulo fixedNUM; no global merge, volume-only shortcut or geometry trial. Log each proof. If residual fragmentation or thin pieces remain, deliver BLOCKED rather than erase them.',
           'outside_preservation_rule':'A complete binary split tree partitions the original root. Each discarded positive-dimensional leaf is inside one registered finite prism; every other terminal region (including numeric/empty branches) is an explicit coverage obligation. Independently outer-bound the root using its original vertices, and verify terminal halfspace regions against emitted material Minkowski-expanded by a cube of Euclidean radius NUM/4 using LP-dual bounds checked with exact rational arithmetic. Prove every emitted vertex is within NUM/4 of a positive convex combination of original root vertices. The ideal construction has exact outside-prism set identity; saved vertices/faces need the continuous NUM-scale implementation certificate, never a point-sampling shortcut. Boolean set-difference is supplementary.',
           'mandatory_regressions':{'original11_sha256':sha(ROOT/'reports/nero-gripper-v21-normal-witnesses.json'),
                                    'v22_result_sha256':sha(ROOT/'reports/nero-gripper-v22-local-verification.json'),
                                    'introduced_normal_count':sum(q['classification']=='INTRODUCED_REAL_NORMAL_FAILURE'for q in v22['baseline_normal_comparisons']),
                                    'introduced_surface_count':len(v22['independent_all_tier_point_scan']['A']['certified_source_exterior_runtime_void_failures']),
                                    'explicit_source2030_point_mm':[19.434445381,-46.729538687,6.169381551]},
           'policy_sha256':{str(p.relative_to(ROOT)):sha(p)for p in [ROOT/'reports/nero-gripper-v21-fidelity-protocol.json',ROOT/'reports/nero-gripper-v21-fidelity-protocol.md',ROOT/'reports/nero-gripper-v2-fidelity-protocol.json']},
           'acceptance':{'A_surface_contour_mm':.01,'A_normal_deg':.05,'minimum_inscribed_diameter_mm':.1,'B_surface_contour_mm':.05,'C_and_void':'Unchanged frozen v2.1; no new true regression; unresolved stays UNKNOWN'},
           'scope':'Only the32 named baseline roots. All other full entity objects and source assets unchanged. No policy/MJCF/MuJoCo/grasp/physics/main/newIssue changes.',
           'builder_sha256':sha(__file__)}
    save(PRE, pre)
    print(json.dumps({'source_patches':len(pp),'source_faces':326,'finite_prisms':len(prisms),'affected_roots':len({p['baseline_entity']for p in prisms}),'candidate_generated':False},indent=2))


def convex_union(a, b):
    """Sufficient shared-facet convex-union identity, no volume heuristic."""
    ea, eb = a['eq'], b['eq']
    span = float(np.linalg.norm(np.ptp(np.vstack([a['v'],b['v']]),axis=0)))
    for i, row in enumerate(ea):
        near = np.flatnonzero(np.linalg.norm(eb[:,:3] + row[:3],axis=1)*max(span,1) + abs(eb[:,3] + row[3]) <= NUM/128)
        for j in near:
            other_a = np.delete(ea,i,axis=0)
            other_b = np.delete(eb,j,axis=0)
            ra = float(np.max(b['v'] @ other_a[:,:3].T + other_a[:,3])) if len(other_a) else -1
            rb = float(np.max(a['v'] @ other_b[:,:3].T + other_b[:,3])) if len(other_b) else -1
            if max(ra, rb) > EPS:
                continue
            result = body(np.vstack([a['v'], b['v']]))
            if result is not None:
                return result, {'shared_support_planes':[int(i),int(j)],'other_support_vertex_residual_mm':[ra,rb],
                                'opposite_plane_gap_bound_mm':float(np.linalg.norm(eb[j,:3]+row[:3])*max(span,1)+abs(eb[j,3]+row[3])),
                                'proof':'Each half of the intersection of all nonshared supports is exactly one old convex piece; the internal split plane partitions their convex union.'}
    return None, None


def build():
    if OUT.exists() or (TASK/'generation-started.json').exists():
        raise SystemExit('One construction already started; no second candidate/retry authorized')
    pre = json.loads(PRE.read_text())
    assert pre['builder_sha256'] == sha(__file__)
    assert sha(BASE) == pre['baseline_file_sha256']
    for p,h in pre['policy_sha256'].items():
        assert sha(ROOT/p) == h
    save(TASK/'generation-started.json',{'preregistration_sha256':sha(PRE),'builder_sha256':sha(__file__),'timestamp_ns':time.time_ns()})
    cells = json.loads(BASE.read_text())['entities']; result=[]; mapping=[]; trees=[]
    byroot = collections.defaultdict(list)
    for prism in pre['influence_prisms']:
        byroot[prism['baseline_entity']].append(prism)
    for old in cells:
        root_id=old['id']
        if root_id not in byroot:
            result.append(copy.deepcopy(old));mapping.append({'old':root_id,'new':[root_id],'changed':False});continue
        p=body(np.array(old['vertices_m'])*1000)
        root={'id':0,'parent':None,'role':'ROOT','vertices_mm':p['v'].tolist(),'region_halfspaces_mm':p['eq'].tolist()}
        nodes=[root];active=[(p,0,p['eq'].tolist())];degenerated=[]
        for prism in sorted(byroot[root_id],key=lambda q:(q['source_face'],q['patch'],q['id'])):
            next_active=[]
            for piece,node_id,region in active:
                lower_bounds=[float(np.min(piece['v']@np.array(row[:3])+row[3]))for row in prism['carve_halfspaces_mm']]
                if max(lower_bounds)>=-EPS:
                    # No positive-dimensional intersection beyond the fixed
                    # arithmetic band. Preserve this whole component; do not
                    # manufacture coplanar sliver branches.
                    nodes[node_id].setdefault('skipped_caps',[]).append({'prism':prism['id'],'separating_plane':int(np.argmax(lower_bounds)),'affine_vertex_minimum_mm':max(lower_bounds),'reserve_mm':EPS})
                    next_active.append((piece,node_id,region))
                    continue
                current,current_id,current_region=piece,node_id,region
                for plane_index,plane in enumerate(prism['carve_halfspaces_mm']):
                    row=np.array(plane)
                    if current is None:
                        break
                    inside,ik=cut(current,row);outside,ok=cut(current,-row)
                    children=[]
                    for child,kind,sign,role in [(inside,ik,1,'INSIDE_CAP_PREFIX'),(outside,ok,-1,'RETAIN_OUTSIDE_CAP')]:
                        child_id=len(nodes);child_region=current_region+[(sign*row).tolist()]
                        entry={'id':child_id,'parent':current_id,'role':role,'prism':prism['id'],'plane_index':plane_index,'branch_sign':sign,'clip_status':kind,'region_halfspaces_mm':child_region}
                        if child is not None:
                            entry['vertices_mm']=child['v'].tolist()
                        elif kind!='EMPTY_AFFINE_VERTEX_MINIMUM':
                            degenerated.append(child_id)
                        nodes.append(entry);children.append(child_id)
                        if role=='RETAIN_OUTSIDE_CAP' and child is not None:
                            next_active.append((child,child_id,child_region))
                    nodes[current_id]['children']=children;nodes[current_id]['cut_plane_mm']=row.tolist();nodes[current_id]['prism']=prism['id']
                    current,current_id,current_region=inside,children[0],nodes[children[0]]['region_halfspaces_mm']
                if current is not None:
                    nodes[current_id]['role']='DISCARD_FINITE_OUTWARD_CAP'
            active=next_active
        leaves=[{'node':node,'region_halfspaces_mm':region,'vertices_mm':p['v'].tolist()}for p,node,region in active]
        components=[{'poly':p,'nodes':[node]}for p,node,region in active]
        merges=[];changed=True
        while changed:
            changed=False
            for i in range(len(components)):
                for j in range(i+1,len(components)):
                    merged,proof=convex_union(components[i]['poly'],components[j]['poly'])
                    if merged is None:
                        continue
                    ids=components[i]['nodes']+components[j]['nodes']
                    merges.append({'raw_leaf_nodes':ids,**proof})
                    components[i]={'poly':merged,'nodes':ids};components.pop(j);changed=True;break
                if changed:break
        successors=[]
        for number,component in enumerate(components,1):
            p=component['poly'];scale_error=None
            try:
                diameter,centre=inradius({'eq':p['eq']})
            except RuntimeError as error:
                diameter,centre=0.,p['v'].mean(0);scale_error=str(error)
            eid=f'{root_id}_FP{number:03d}';successors.append(eid)
            result.append({'id':eid,'name':old['name']+'_finite_patch_'+str(number),'semantic_family':old['semantic_family'],
                           'primitive_type':'convex_polyhedron','vertices_m':(p['v']/1000).tolist(),'faces':p['faces'],
                           'halfspaces_m':(p['eq']*[1,1,1,.001]).tolist(),'inscribed_diameter_mm':2*diameter,
                           'construction_scale_lp_error':scale_error,
                           'inscribed_centre_m':(centre/1000).tolist(),'volume_mm3':p['volume_mm3'],
                           'baseline_entity':root_id,'retained_split_leaf_nodes':component['nodes'],
                           'registered_influence_prisms':[q['id']for q in byroot[root_id]],
                           'source_provenance_only':{'source_regions':old['source_regions'],'reference_entity_ids':old['reference_entity_ids']},
                           'exterior_certification_state':'UNVERIFIED_FINITE_PATCH_CANDIDATE'})
        mapping.append({'old':root_id,'new':successors,'changed':True,'split_count':len(successors),'raw_retained_leaf_count':len(leaves),'prisms':[q['id']for q in byroot[root_id]]})
        trees.append({'baseline_entity':root_id,'root_support_halfspaces_mm':root['region_halfspaces_mm'],'nodes':nodes,'retained_leaves':leaves,'coalescences':merges,'unresolved_degenerate_nodes':degenerated})
        print('root',root_id,'finite caps',len(byroot[root_id]),'raw leaves',len(leaves),'closed convex outputs',len(successors),'degenerate branches',len(degenerated),flush=True)
    output={'schema_version':'2.3-finite-patch','status':'UNVERIFIED_SINGLE_FINITE_PATCH_CANDIDATE',
            'baseline_geometry_commit':pre['baseline_geometry_commit'],'baseline_geometry_sha256':pre['baseline_geometry_sha256'],
            'geometry_sha256':geosha(result),'entity_count_per_finger':len(result),'entities':result,
            'old_to_new':mapping,'modified_baseline_entities':sorted(byroot),'candidate_generation_count':1,
            'preregistration_sha256':sha(PRE),'builder_sha256':sha(__file__),
            'scope':'Only material inside the registered finite prisms can be removed; every outside branch retained. Baseline is original346, never failedv22.'}
    save(TRACE,{'roots':trees,'preregistration_sha256':sha(PRE),'builder_sha256':sha(__file__)})
    save(OUT,output);save(ROOT/'reports/nero-gripper-v23-mapping.json',mapping)
    print(json.dumps({'entity_count':len(result),'geometry_sha256':output['geometry_sha256'],'candidate_generation_count':1},indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['preflight','build']);args=parser.parse_args()
    preflight() if args.stage=='preflight' else build()
