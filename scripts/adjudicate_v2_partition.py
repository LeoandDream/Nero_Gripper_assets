"""Issue25: immutable halfspace-union adjudication; no Boolean or simulator oracle.

All membership uses saved convex coefficients, in metres. Exact rational
fallback interprets their binary64 values literally; it does not snap them.
Original STL bore paths use exact projected-triangle intersections separately.
"""
from pathlib import Path
from fractions import Fraction as F
from collections import Counter
import csv
import hashlib
import json
import sys
import time
import platform

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
from runtime_contact_partition import load_source
from analyze_runtime_partition import TriangleBVH


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def rational(x):
    return F.from_float(float(x))


class Union:
    def __init__(self, cells, tolerance_mm):
        self.cells = cells
        self.eq = [np.asarray(c['halfspaces_m']) for c in cells]
        self.flat = np.concatenate(self.eq)
        self.starts = np.r_[0, np.cumsum([len(a) for a in self.eq])[:-1]]
        self.tau = tolerance_mm / 1000
        self.exact_rows = {}
        self.refined_points = 0
        self.refined_cells = Counter()
        self.rational_planes = 0

    def rows(self, i):
        if i not in self.exact_rows:
            self.exact_rows[i] = [[rational(x) for x in row] for row in self.eq[i]]
        return self.exact_rows[i]

    def exact_cell(self, i, point):
        """Longdouble with local dot-product bounds; exact near-active signs."""
        e = self.eq[i].astype(np.longdouble)
        p = np.asarray(point, dtype=np.longdouble)
        terms = e[:, :3] * p
        val = terms.sum(1) + e[:, 3]
        err = 32 * np.finfo(np.longdouble).eps * (abs(terms).sum(1) + abs(e[:, 3]))
        if np.any(val > err):
            return False, 'longdouble'
        active = np.flatnonzero(abs(val) <= err)
        if not len(active):
            return True, 'longdouble'
        qp = [rational(x) for x in point]
        for j in active:
            a = self.rows(i)[j]
            self.rational_planes += 1
            if sum(x*y for x, y in zip(a[:3], qp)) + a[3] > 0:
                return False, 'exact_rational'
        return True, 'exact_rational'

    def membership(self, points):
        points = np.atleast_2d(points)
        result = []
        for start in range(0, len(points), 256):
            pp = points[start:start+256]
            residual = pp @ self.flat[:, :3].T + self.flat[:, 3]
            margin = np.maximum.reduceat(residual, self.starts, axis=1)
            for p, m in zip(pp, margin):
                solid = np.flatnonzero(m < -self.tau)
                near = np.flatnonzero(abs(m) <= self.tau)
                refined = []
                owners = list(map(int, solid))
                precision = 'binary64_outside_fixed_tolerance'
                if not len(solid) and len(near):
                    self.refined_points += 1
                    precision = 'local_longdouble_or_exact'
                    for i in near:
                        self.refined_cells[int(i)] += 1
                        inside, method = self.exact_cell(int(i), p)
                        refined.append({'entity': self.cells[i]['id'], 'inside': inside, 'method': method})
                        if inside:
                            owners.append(int(i))
                result.append({'state': 'SOLID' if owners else 'VOID',
                               'entity_ids': [self.cells[i]['id'] for i in owners],
                               'minimum_cell_max_residual_mm': float(m.min()*1000),
                               'precision': precision, 'refined': refined})
        return result

    def line_intervals(self, point, direction, window_m):
        """Exact intersections only for the few cells meeting a local ray.

        Float clipping is merely a conservative broad phase with the frozen
        tolerance as reserve. Every accepted endpoint is a Fraction expression.
        """
        intervals = []
        p = [rational(x) for x in point]
        n = [rational(x) for x in direction]
        local = []
        for i, e in enumerate(self.eq):
            a = e[:, :3] @ direction
            b = e[:, :3] @ point + e[:, 3]
            low, high = -window_m-self.tau, window_m+self.tau
            possible = True
            for aa, bb in zip(a, b):
                if abs(aa) < 1e-14:
                    if bb > self.tau:
                        possible = False
                        break
                elif aa > 0:
                    high = min(high, (-bb+self.tau)/aa)
                else:
                    low = max(low, (-bb+self.tau)/aa)
            if not possible or low > high:
                continue
            local.append(i)
            lo, hi = rational(-window_m), rational(window_m)
            for row in self.rows(i):
                aa = sum(x*y for x, y in zip(row[:3], n))
                bb = sum(x*y for x, y in zip(row[:3], p)) + row[3]
                if aa == 0:
                    if bb > 0:
                        lo, hi = F(1), F(0)
                        break
                elif aa > 0:
                    hi = min(hi, -bb/aa)
                else:
                    lo = max(lo, -bb/aa)
            if lo <= hi:
                intervals.append((lo, hi, i))
        intervals.sort()
        merged = []
        for lo, hi, i in intervals:
            if merged and lo <= merged[-1][1]:
                merged[-1][1] = max(merged[-1][1], hi)
                merged[-1][2].append(i)
            else:
                merged.append([lo, hi, [i]])
        gaps = []
        for left, right in zip(merged, merged[1:]):
            if right[0] > left[1]:
                gaps.append({'start_m': left[1], 'end_m': right[0],
                             'width_mm': float((right[0]-left[1])*1000),
                             'left_entities': left[2], 'right_entities': right[2]})
        return intervals, merged, gaps, local

    def strict_cover(self, points):
        """A convex patch is interior if all its vertices are strict interior
        to one frozen cell. This is a continuous proof by affine convexity.
        """
        points=np.asarray(points)
        if not len(points):return None
        residual=points@self.flat[:,:3].T+self.flat[:,3]
        m=np.maximum.reduceat(residual.max(0),self.starts)
        candidates=np.flatnonzero(m<=self.tau)
        for i in candidates[np.argsort(m[candidates])]:
            if m[i]<-self.tau:
                return {'entity':self.cells[i]['id'],'proof':'Every polygon vertex strictly inside one cell; affine convexity covers the entire polygon.', 'maximum_vertex_plane_residual_mm':float(m[i]*1000)}
            vals=[sum(a*b for a,b in zip(row[:3],[rational(x) for x in p]))+row[3] for row in self.rows(int(i)) for p in points]
            if max(vals)<0:
                return {'entity':self.cells[i]['id'],'proof':'Exact rational strict inequalities at every polygon vertex; affine convexity covers the entire polygon.', 'maximum_vertex_plane_residual_mm':float(max(vals)*1000),'maximum_residual_exact_m':str(max(vals))}
        return None


def normal(t):
    n = np.cross(t[1]-t[0], t[2]-t[0])
    return n/np.linalg.norm(n)


def probe_class(center, pairs):
    sides = [(a['state'], b['state']) for a, b in pairs]
    if len(set(sides)) == 1 and sides[0] in [('SOLID', 'VOID'), ('VOID', 'SOLID')]:
        return 'TRUE_BOUNDARY'
    if all(a == b == 'SOLID' for a, b in sides) and center['state'] == 'SOLID':
        return 'BOOLEAN_PSEUDOFACE'
    return 'NUMERICALLY_UNRESOLVED'


def adjudicate(record, t, oracle, protocol, extra_points=()):
    n = normal(t)
    points = [np.array(w) @ t for w in protocol['barycentric_probes']]
    points.extend(np.asarray(p) for p in extra_points)
    centers = oracle.membership(points)
    probes = []
    for p, center in zip(points, centers):
        offsets = np.array(protocol['offsets_mm'])/1000
        shifted = np.array([p+s*e*n for e in offsets for s in [-1, 1]])
        states = oracle.membership(shifted)
        pairs = list(zip(states[::2], states[1::2]))
        probes.append({'point_mm': (p*1000).tolist(), 'center': center,
                       'class': probe_class(center, pairs),
                       'offset_pairs': [{'offset_mm': e, 'minus': a, 'plus': b}
                                        for e, (a, b) in zip(protocol['offsets_mm'], pairs)]})
    classes = {p['class'] for p in probes}
    return {**record, 'classification': classes.pop() if len(classes) == 1 else 'NUMERICALLY_UNRESOLVED',
            'normal': n.tolist(), 'probe_class_counts': dict(Counter(p['class'] for p in probes)),
            'probes': probes, 'scope': 'Fixed interior barycentric probes, not a continuous whole-triangle proof.'}


def legacy_missing_locators(cells, output, records):
    """Reproduce ONLY the old failed patch positions, never their truth.

    Its NUM snapping/buffer is intentionally identical to the old certifier.
    Every located point is subsequently adjudicated by the independent union.
    """
    sys.path.insert(0,str(ROOT/'debug/issue25/v2-runtime/vendor'))
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from shapely import set_precision
    num=1e-7  # mm: old locator, not the membership tolerance
    out=output*1000
    ns=np.array([normal(t) for t in out])
    vertices=[np.array(e['vertices_m'])*1000 for e in cells]
    eqs=[np.array(e['halfspaces_m'])*[1,1,1,1000] for e in cells]
    bounds=np.array([[v.min(0),v.max(0)] for v in vertices])
    ids={e['id']:i for i,e in enumerate(cells)}
    first={}
    for i,e in enumerate(cells):
        for face in e['faces']:
            t=vertices[i][face];n=normal(t);d=-n@t[0]
            key=tuple(np.round(np.r_[n,d],10))
            if key not in first:
                u=np.eye(3)[np.argmin(abs(n))];u-=n*(u@n);u/=np.linalg.norm(u)
                first[key]=(n,d,u,np.cross(n,u))
    def projected(t,u,w):
        return set_precision(Polygon(np.c_[t@u,t@w]).convex_hull,num/16)
    def clip(p,eq):
        if len(p)<3:return np.empty((0,3))
        dd=p@eq[:3]+eq[3];dd[abs(dd)<=num]=0;result=[]
        for j,(a,b) in enumerate(zip(p,np.roll(p,-1,axis=0))):
            da,db=dd[j],dd[(j+1)%len(p)]
            if da<=0:result.append(a)
            if (da<=0)!=(db<=0):result.append(a+(b-a)*da/(da-db))
        return np.array(result)
    def pieces(g):
        if g.is_empty:return []
        if g.geom_type=='Polygon':return [g]
        return [p for q in g.geoms for p in pieces(q)] if hasattr(g,'geoms') else []
    cache={};result={}
    for f in records:
        if f.get('check')!='missing_true_external_face':continue
        i=ids[f['entity']];t=vertices[i][cells[i]['faces'][f['face']]];n=normal(t);d=-n@t[0]
        key=tuple(np.round(np.r_[n,d],10));nn,dd,u,w=first[key]
        if key not in cache:
            ix=np.flatnonzero((ns@nn>1-1e-6)&(np.max(abs(out@nn+dd),axis=1)<=num))
            cache[key]=unary_union([projected(q,u,w) for q in out[ix]])
        base=projected(t,u,w);cover=[]
        near=np.flatnonzero(np.all(bounds[:,1]>=bounds[i,0]-num,axis=1)&np.all(bounds[:,0]<=bounds[i,1]+num,axis=1))
        for j in near:
            if i==j or np.max(vertices[j]@n+d)<=num:continue
            q=t
            for plane in eqs[j]:
                q=clip(q,plane)
                if len(q)<3:break
            if len(q)>=3:
                pg=projected(q,u,w)
                if pg.area>1e-16:cover.append(pg)
        ext=base.difference(unary_union(cover)) if cover else base
        missing=ext.difference(cache[key].buffer(num,quad_segs=32))
        polys=sorted(pieces(missing),key=lambda q:(-q.area,q.bounds))
        points=[];patch_vertices=[]
        for pg in polys:
            if pg.area<=0:continue
            p=pg.representative_point();points.append((u*p.x+w*p.y-n*d)/1000)
            patch_vertices.append([(u*x+w*y-n*d)/1000 for x,y in list(pg.exterior.coords)[:-1]])
        result[(f['entity'],f['face'])]={'points':points,'patch_vertices':patch_vertices,'reproduced_missing_area_mm2':float(missing.area),
                                        'old_missing_area_mm2':f['uncovered_area_mm2'],'components':len(points)}
    # Same process for the specifically alleged buried subpatch, rather than
    # testing a large triangle's exposed centroid and missing its tiny core.
    for f in records:
        if 'covering_entity' not in f:continue
        t=out[f['triangle']];q=t.copy();i=ids[f['covering_entity']]
        for plane in eqs[i]:
            shrunk=plane.copy();shrunk[3]+=2*num;q=clip(q,shrunk)
            if len(q)<3:break
        n=normal(t);d=-n@t[0];u=np.eye(3)[np.argmin(abs(n))];u-=n*(u@n);u/=np.linalg.norm(u);w=np.cross(n,u)
        pg=projected(q,u,w) if len(q)>=3 else Polygon()
        points=[];patch_vertices=[]
        for poly in pieces(pg):
            if poly.area<=0:continue
            p=poly.representative_point();points.append((u*p.x+w*p.y-n*d)/1000)
            patch_vertices.append([(u*x+w*y-n*d)/1000 for x,y in list(poly.exterior.coords)[:-1]])
        result[(f['triangle'],f['covering_entity'])]={'points':points,'patch_vertices':patch_vertices,'components':len(points),'old_core_area_mm2':f['core_overlap_area_mm2'],'reproduced_core_area_mm2':float(pg.area)}
    return result


def exact_gap_certificate(oracle, point, direction, gap):
    """A continuous exact open-interval exclusion proof against ALL346 cells."""
    a,b=gap['start_m'],gap['end_m']
    p=[rational(v) for v in point];n=[rational(v) for v in direction]
    left=[v+a*w for v,w in zip(p,n)];right=[v+b*w for v,w in zip(p,n)]
    mid=np.array([float((x+y)/2) for x,y in zip(left,right)])
    excluded=[]
    boundary_owners={'left':[],'right':[]}
    for i,e in enumerate(oracle.eq):
        row_order=np.argsort(-(e[:,:3]@mid+e[:,3]))
        proof=None
        for j in row_order:
            row=oracle.rows(i)[int(j)]
            va=sum(x*y for x,y in zip(row[:3],left))+row[3]
            vb=sum(x*y for x,y in zip(row[:3],right))+row[3]
            if va>=0 and vb>=0 and (va>0 or vb>0):
                proof={'entity':oracle.cells[i]['id'],'plane':int(j),'left_residual_exact_m':str(va),'right_residual_exact_m':str(vb)}
                break
        if proof is None:
            return {'status':'UNKNOWN','reason':'Could not prove exclusion on the whole interval','entity':oracle.cells[i]['id']}
        excluded.append(proof)
        if i in gap['left_entities']:
            vals=[sum(x*y for x,y in zip(row[:3],left))+row[3] for row in oracle.rows(i)]
            if max(vals)<=0:boundary_owners['left'].append(oracle.cells[i]['id'])
        if i in gap['right_entities']:
            vals=[sum(x*y for x,y in zip(row[:3],right))+row[3] for row in oracle.rows(i)]
            if max(vals)<=0:boundary_owners['right'].append(oracle.cells[i]['id'])
    return {'status':'EXACT_OPEN_VOID_INTERVAL_AND_CLOSED_BOUNDARIES_PROVEN' if all(boundary_owners.values()) else 'UNKNOWN',
            'cell_exclusion_proofs':excluded,'endpoint_solid_owners':boundary_owners,
            'interpretation':'Literal stored halfspaces have a positive gap. Its width is far below the frozen numerical resolution; this is not a macroscopic gap measurement.'}


def source_line_intervals(triangles, x, z):
    """Exact original-STL line crossings, without reference-cell assumptions."""
    x, z = rational(x), rational(z)
    hits = {}
    ambiguous = []
    for i, tri in enumerate(triangles):
        a, b, c = [[rational(v) for v in p] for p in tri]
        ux, uz = b[0]-a[0], b[2]-a[2]
        vx, vz = c[0]-a[0], c[2]-a[2]
        det = ux*vz-uz*vx
        if det == 0:
            # All five paths avoid coplanar vertical triangles. Exact bbox
            # overlap is conservatively reported if this assumption fails.
            if min(p[0] for p in [a,b,c]) <= x <= max(p[0] for p in [a,b,c]) and min(p[2] for p in [a,b,c]) <= z <= max(p[2] for p in [a,b,c]):
                ambiguous.append(i)
            continue
        u = ((x-a[0])*vz-(z-a[2])*vx)/det
        v = (ux*(z-a[2])-uz*(x-a[0]))/det
        if u < 0 or v < 0 or u+v > 1:
            continue
        y = a[1]+u*(b[1]-a[1])+v*(c[1]-a[1])
        hits.setdefault(y, []).append((i, -1 if det > 0 else 1))  # n_y = -det
    winding = 0
    start = None
    intervals = []
    crossings = []
    for y, faces in sorted(hits.items()):
        signs = {s for _,s in faces}
        change = -next(iter(signs)) if len(signs) == 1 else 0
        before = winding
        winding += change
        if before == 0 and winding != 0:
            start = y
        if before != 0 and winding == 0:
            intervals.append((start, y))
        crossings.append({'y_mm': float(y*1000), 'source_local_faces': [i for i,s in faces], 'winding_after': winding})
    return intervals, crossings, ambiguous, winding


def main():
    started = time.time()
    pr = ROOT/'reports/nero-gripper-v2-adjudication-preregistration.json'
    protocol = json.loads(pr.read_text())
    freeze = json.loads((ROOT/'debug/issue25/v2-adjudication/freeze-before-query.json').read_text())
    assert sha(pr) == freeze['preregistration_sha256']
    assert all(sha(ROOT/p) == s for p,s in freeze['inputs_sha256'].items())
    data = json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text())
    cells = data['entities']
    geometry = hashlib.sha256(json.dumps([[e['id'],e['vertices_m'],e['faces']] for e in cells],separators=(',',':')).encode()).hexdigest()
    assert geometry == protocol['geometry_sha256']
    cert = json.loads((ROOT/'reports/nero-gripper-v2-runtime-verification.json').read_text())['union_exterior_certificate']
    z = np.load(ROOT/'reports/nero-gripper-v2-union-exterior.npz')
    output = z['vertices_mm'][z['faces']]/1000
    lookup = {e['id']: e for e in cells}
    oracle = Union(cells, protocol['primary_membership_tolerance_mm'])
    special_p = np.array(json.loads((ROOT/'reports/nero-gripper-v2-runtime-counterexample.json').read_text())['point_mm'])/1000
    special = adjudicate({'triangle':2940}, output[2940], oracle, protocol, [special_p])
    locators=legacy_missing_locators(cells,output,cert['failures']+cert['occluded_output_triangles'])
    def localized(f,t,loc):
        record=adjudicate(f,t,oracle,protocol,loc['points'])
        record['original_face_probe_class_counts']=record['probe_class_counts']
        record['original_face_probes']=record['probes'][:len(protocol['barycentric_probes'])]
        record['probes']=record['probes'][len(protocol['barycentric_probes']):]
        record['probe_class_counts']=dict(Counter(p['class'] for p in record['probes']))
        cc={p['class'] for p in record['probes']}
        record['classification']=cc.pop() if len(cc)==1 else 'NUMERICALLY_UNRESOLVED'
        record['legacy_patch_locator']={k:v for k,v in loc.items() if k not in ['points','patch_vertices']}
        record['continuous_patch_interior_proofs']=[oracle.strict_cover(poly) for poly in loc['patch_vertices']]
        if record['continuous_patch_interior_proofs'] and all(record['continuous_patch_interior_proofs']):
            record['classification']='BOOLEAN_PSEUDOFACE'
        record['scope']='Each alleged failed subpatch, not the large parent triangle. Fixed-scale representative probes plus continuous interior proofs where available; original face probes retained separately.'
        if not record['probes']:record['classification']='NUMERICALLY_UNRESOLVED'
        return record
    failures = []
    for i, f in enumerate(cert['failures']):
        if f['check'] == 'unattributed_output_triangle':
            t = output[f['triangle']]
        else:
            e = lookup[f['entity']]
            t = np.array(e['vertices_m'])[e['faces'][f['face']]]
        if f['check']=='missing_true_external_face':
            loc=locators[(f['entity'],f['face'])]
            record=localized(f,t,loc)
        else:
            record=adjudicate(f,t,oracle,protocol)
        failures.append(record)
        if i%50 == 0:
            print('Adjudicated certificate records',i+1,len(cert['failures']),flush=True)
    occlusion = [localized(f,output[f['triangle']],locators[(f['triangle'],f['covering_entity'])]) for f in cert['occluded_output_triangles']]
    source = load_source()
    st = np.asarray(source['vertices_m'])[np.array([t['vertex_ids'] for t in source['triangles']])]
    assert sha(ROOT/'meshes/source/agx_gripper/meshes/gripper_link1.stl') == freeze['source_stl_sha256']
    bore = []
    lo, hi = [rational(v/1000) for v in protocol['central_bore']['path_y_mm']]
    for x,z0 in protocol['central_bore']['axes_xz_mm']:
        parts = {}
        for name, a, b in [('mount',0,504),('contact',504,660),('backing',660,2086)]:
            intervals, crossings, amb, winding = source_line_intervals(st[a:b],x/1000,z0/1000)
            cuts = [(max(a0,lo),min(b0,hi)) for a0,b0 in intervals if max(a0,lo)<min(b0,hi)]
            parts[name] = {'solid_intervals_y_mm': [[float(a0*1000),float(b0*1000)] for a0,b0 in intervals],
                           'path_solid_intervals_y_mm': [[float(a0*1000),float(b0*1000)] for a0,b0 in cuts],
                           'crossings':crossings,'ambiguous_parallel_faces':amb,'final_winding':winding,
                           'full_line_clear': not intervals and not amb and winding==0,
                           'specified_path_clear': not cuts and not amb and winding==0}
        midpoint = np.array([x/1000,float((lo+hi)/2),z0/1000])
        ints, merged, gaps, local = oracle.line_intervals(midpoint,np.array([0.,1.,0.]),float((hi-lo)/2))
        bore.append({'axis_xz_mm':[x,z0],'source_parts':parts,'v2_solid_intervals_y_mm':[[float((rational(midpoint[1])+a)*1000),float((rational(midpoint[1])+b)*1000)] for a,b,ids in merged], 'v2_intersecting_entities':sorted({cells[i]['id'] for a,b,i in ints})})
    tree = TriangleBVH(st*1000)
    intervals, merged, gaps, local = oracle.line_intervals(special_p,normal(output[2940]),max(protocol['offsets_mm'])/1000)
    exact = {'local_entities':[cells[i]['id'] for i in local], 'merged_intervals_t_mm':[[float(a*1000),float(b*1000)] for a,b,ids in merged], 'positive_exact_gaps':[]}
    for gap in gaps:
        a,b=gap['start_m'],gap['end_m'];p=[rational(v)+a*rational(n) for v,n in zip(special_p,normal(output[2940]))]
        q=np.array([float(v*1000) for v in p]);dist,near=tree.query(q[None])
        exact['positive_exact_gaps'].append({'width_mm':gap['width_mm'],'t_start_mm':float(a*1000),'t_end_mm':float(b*1000),
                                          'start_exact_m':str(a),'end_exact_m':str(b),'boundary_point_exact_m':[str(v) for v in p],
                                          'boundary_point_mm':q.tolist(),'distance_to_all_source_triangles_mm':float(dist[0]),'nearest_source_triangle':int(near[0]),
                                          'left_entities':[cells[i]['id'] for i in gap['left_entities']],'right_entities':[cells[i]['id'] for i in gap['right_entities']],
                                          'all_cell_interval_proof':exact_gap_certificate(oracle,special_p,normal(output[2940]),gap)})
    true_probes=[{'group':group,'record':i,'probe':j,'point_mm':p['point_mm']} for group,records in [('certificate',failures),('occlusion',occlusion)] for i,f in enumerate(records) for j,p in enumerate(f['probes']) if p['class']=='TRUE_BOUNDARY']
    if true_probes:
        distances,nearest=tree.query(np.array([p['point_mm'] for p in true_probes]))
        for r,distance,face in zip(true_probes,distances,nearest):
            r.update(distance_to_all_source_triangles_mm=float(distance),nearest_source_triangle=int(face))
    fp=json.loads((ROOT/'reports/nero-gripper-v2-fidelity-protocol.json').read_text())
    source_tiers={}
    for name,rule in fp['source_region_policy'].items():
        for a0,b0 in rule['source_triangle_ranges_zero_based_inclusive']:
            for face in range(a0,b0+1):source_tiers[face]=rule['tier']
    assert len(source_tiers)==len(st)
    a_ids=[i for i in source_tiers if source_tiers[i]=='A']
    b_ids=[i for i in source_tiers if source_tiers[i] not in ['A','C']]
    atree=TriangleBVH(st[a_ids]*1000);btree=TriangleBVH(st[b_ids]*1000)
    if true_probes:
        points=np.array([p['point_mm'] for p in true_probes]);ad,ai=atree.query(points);bd,bi=btree.query(points)
        for r,da,ia,db,ib in zip(true_probes,ad,ai,bd,bi):
            r.update(nearest_source_tier=source_tiers[r['nearest_source_triangle']],distance_to_A_triangles_mm=float(da),nearest_A_face=a_ids[ia],distance_to_B_or_mixed_triangles_mm=float(db),nearest_B_or_mixed_face=b_ids[ib])
    # Audit source topology without assuming bitwise seam coordinates match.
    # Original-triangle path intersections never use the diagnostic index weld.
    source_structure={}
    for name,a0,b0 in [('mount',0,504),('contact',504,660),('backing',660,2086)]:
        tt=st[a0:b0];edges=Counter();directions=Counter()
        for tri in tt:
            vv=[tuple(v) for v in tri]
            for x,y in zip(vv,vv[1:]+vv[:1]):edges[tuple(sorted((x,y)))]+=1;directions[(x,y)]+=1
        flat=tt.reshape(-1,3)*1000
        _,ix,inv=np.unique(np.round(flat,12),axis=0,return_index=True,return_inverse=True)
        ff=inv.reshape(-1,3);we=Counter();wd=Counter()
        for face in ff:
            for x,y in zip(face,np.roll(face,-1)):we[tuple(sorted((int(x),int(y))))]+=1;wd[(int(x),int(y))]+=1
        source_structure[name]={'source_face_range':[a0,b0-1],'exact_edge_incidence_two':all(n==2 for n in edges.values()),'exact_oriented_edges':all(n==directions[(y,x)] for (x,y),n in directions.items()),'signed_volume_mm3':float(np.einsum('ij,ij->i',tt[:,0],np.cross(tt[:,1],tt[:,2])).sum()/6*1e9),
                                'analysis_only_index_weld_mm':1e-12,'index_weld_edge_incidence_two':all(n==2 for n in we.values()),'index_weld_oriented_edges':all(n==wd[(y,x)] for (x,y),n in wd.items()),'maximum_index_weld_displacement_mm':float(np.linalg.norm(flat-flat[ix][inv],axis=1).max()),
                                'note':'Raw STL is untouched. Index welding only audits topological agreement at the pre-existing source precision; exact ray crossings always use original coordinates.'}
    geometry_representation_residual=max(float(np.max(np.array(e['vertices_m'])@np.array(e['halfspaces_m'])[:,:3].T+np.array(e['halfspaces_m'])[:,3]))*1000 for e in cells)
    result = {'geometry_sha256':geometry,'candidate_commit':protocol['candidate_commit'],'preregistration_sha256':sha(pr),
              'geometry_and_all_inputs_unchanged':all(sha(ROOT/p)==s for p,s in freeze['inputs_sha256'].items()),
              'special_triangle2940':special,'exact_local_line_at_counterexample':exact,
              'fixed_scale_true_boundary_probe_source_distances':true_probes,
              'certificate_failures':failures,'certificate_counts':dict(Counter(f['classification'] for f in failures)),
              'certificate_counts_by_old_check':{name:dict(Counter(f['classification'] for f in failures if f['check']==name)) for name in ['unattributed_output_triangle','missing_true_external_face']},
              'occlusion_records':occlusion,'occlusion_counts':dict(Counter(f['classification'] for f in occlusion)),
              'central_bore_original_STL_paths':bore,'local_precision':{'refined_points':oracle.refined_points,'nearby_cells_refined':len(oracle.refined_cells),'max_repeated_refinements_one_cell':max(oracle.refined_cells.values(),default=0),'rational_plane_signs':oracle.rational_planes,'longdouble_mantissa_bits':np.finfo(np.longdouble).nmant},
              'original_source_part_structure':source_structure,'maximum_saved_vertex_outside_saved_halfspace_mm':geometry_representation_residual,
              'limitations':['Probe classifications do not certify complete faces or whole union.','Positive exact gaps refer to literal frozen halfspaces; saved mesh/halfspace roundoff agreement must be reported separately.'],
              'source_stl_sha256':freeze['source_stl_sha256'],'script_sha256':sha(__file__),'elapsed_seconds':time.time()-started,
              'versions':{'python':platform.python_version(),'numpy':np.__version__},
              'project_modules_sha256':{str(Path(module.__file__).resolve().relative_to(ROOT)):sha(module.__file__) for module in list(sys.modules.values()) if getattr(module,'__file__',None) and Path(module.__file__).resolve().is_relative_to(ROOT) and Path(module.__file__).suffix=='.py'},
              'scope':{'mujoco_imported':any(k.startswith('mujoco') for k in sys.modules),'boolean_oracle':False,'candidate_changes':False}}
    confirmed_gap=any(g['all_cell_interval_proof']['status']=='EXACT_OPEN_VOID_INTERVAL_AND_CLOSED_BOUNDARIES_PROVEN' and g['distance_to_all_source_triangles_mm']>.05 for g in exact['positive_exact_gaps'])
    confirmed_false_alarm=any(f.get('continuous_patch_interior_proofs') and all(f['continuous_patch_interior_proofs']) for f in failures)
    bore_contradiction=all(b['source_parts']['mount']['full_line_clear'] and not b['source_parts']['backing']['specified_path_clear'] and b['v2_solid_intervals_y_mm'] for b in bore)
    result['conclusions']={
        'exterior':'V2_EXTERIOR_ADJUDICATION_MIXED' if confirmed_gap and confirmed_false_alarm else 'V2_EXTERIOR_ADJUDICATION_UNKNOWN',
        'central_bore':'POLICY_SOURCE_CONTRADICTION_CONFIRMED' if bore_contradiction else 'POLICY_SOURCE_CONSISTENCY_UNKNOWN',
        'triangle2940':special['classification'],
        'literal_halfspace_true_boundary_over_AB_budget_confirmed':confirmed_gap,
        'macro_geometry_defect_proven':False,
        'second_vertex_face_candidate_required':'UNKNOWN_NOT_ESTABLISHED_BY_THIS_AUDIT',
        'certifier_policy_only_sufficient_for_acceptance':f"NOT_PROVEN; literal halfspace micro-gap and {result['certificate_counts'].get('NUMERICALLY_UNRESOLVED',0)} unresolved records remain, and no whole-union fidelity recertification was performed",
        'old_candidate_status_unchanged':'V2_RUNTIME_ENTITY_PARTITION_BLOCKED',
        'policy_correction_proposal_only':'保留mount part中央孔本身；完整装配的connectivity必须与冻结source assembly一致。若backing原本封堵，不要求runtime人为挖通。',
        'policy_correction_applied':False,
        'stop':'Independent ChatGPT review before any repair or reacceptance.'}
    path=ROOT/'reports/nero-gripper-v2-adjudication.json'
    path.write_text(json.dumps(result,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')
    with (ROOT/'reports/nero-gripper-v2-adjudication.csv').open('w') as stream:
        out=csv.writer(stream,lineterminator='\n');out.writerow(['group','index','old_check','triangle','entity','face','classification','probe_counts'])
        for group,records in [('certificate',failures),('occlusion',occlusion)]:
            for i,f in enumerate(records):out.writerow([group,i,f.get('check','occlusion'),f.get('triangle',''),f.get('entity',''),f.get('face',''),f['classification'],json.dumps(f['probe_class_counts'])])
    print(json.dumps({k:v for k,v in result.items() if k in ['certificate_counts','certificate_counts_by_old_check','occlusion_counts','exact_local_line_at_counterexample','local_precision','elapsed_seconds']},indent=2),flush=True)


if __name__=='__main__':
    main()
