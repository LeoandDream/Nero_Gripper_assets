"""Effective boundary atlas of UNCHANGED convex cells, at frozen1e-7mm support.

Derived support planes and planar subdivisions are verifier representations,
not runtime geometry or a new collision candidate. No simulator is imported.
"""
from pathlib import Path
import collections
import hashlib
import json
import sys
import time

import numpy as np
from scipy.spatial import ConvexHull

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'debug/issue25/v2-runtime/vendor'))
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import set_precision,constrained_delaunay_triangles

NUM=1e-7  # mm; v2.1 frozen support, not a fidelity budget
GRID=NUM/128
PROBE=NUM/4
BAND=np.sqrt((NUM-NUM/16)**2-(2*PROBE)**2)-2*GRID


def basis(n):
    u=np.eye(3)[int(np.argmin(abs(n)))];u-=n*(u@n);u/=np.linalg.norm(u)
    return u,np.cross(n,u)


def polygon(q,u,w):
    return set_precision(Polygon(np.c_[q@u,q@w]).convex_hull,GRID)


def pieces(g):
    if g.is_empty:return []
    if g.geom_type=='Polygon':return [g]
    return [p for q in g.geoms for p in pieces(q)] if hasattr(g,'geoms') else []


def area_shape(g):
    pp=pieces(g)
    return unary_union(pp) if pp else Polygon()


def clip(q,eq):
    if len(q)<3:return np.empty((0,3))
    d=q@eq[:3]+eq[3];out=[]
    for i,(a,b) in enumerate(zip(q,np.roll(q,-1,axis=0))):
        da,db=d[i],d[(i+1)%len(q)]
        if da<=0:out.append(a)
        if (da<=0)!=(db<=0):out.append(a+(b-a)*(da/(da-db)))
    return np.asarray(out)


def support_cells(cells):
    result=[];checks=[]
    for e in cells:
        v=np.array(e['vertices_m'])*1000
        h=ConvexHull(v)
        eq=h.equations
        _,ix=np.unique(np.round(eq,12),axis=0,return_index=True)
        eq=eq[np.sort(ix)]
        triangles=v[np.array(e['faces'])]
        # Every frozen face is in the convex hull (all its vertices are) and
        # within the support reserve of at least one hull plane. Thus its
        # entire triangle has distance <=reserve to the convex boundary.
        residual=triangles@eq[:,:3].T+eq[:,3]
        plane_height=np.max(abs(residual),axis=1)
        cover=plane_height.min(1)
        support_error=float(np.max(v@eq[:,:3].T+eq[:,3]))
        checks.append({'entity':e['id'],'every_original_face_to_support_boundary_upper_mm':float(cover.max()),'maximum_vertex_support_residual_mm':support_error,
                       'status':'PASS' if cover.max()<=NUM/32 and support_error<=NUM/32 else 'UNKNOWN'})
        facets=[]
        for plane in eq:
            ids=np.flatnonzero(abs(v@plane[:3]+plane[3])<=NUM/64)
            if len(ids)<3:continue
            n=plane[:3];u,w=basis(n);pg=polygon(v[ids],u,w)
            if pg.area<=GRID**2:continue
            xy=np.array(pg.exterior.coords)[:-1]
            q=xy[:,0,None]*u+xy[:,1,None]*w-plane[3]*n
            facets.append({'plane':plane,'u':u,'w':w,'shape':pg,'vertices':q})
        result.append({'id':e['id'],'vertices':v,'equations':eq,'facets':facets,'lo':v.min(0),'hi':v.max(0)})
    return result,checks


def closure_pair_certificate(triangle,bodies,cell_boundary_trees):
    """Continuous small-gap evidence using unchanged vertex/face cells.

    Distance to a convex set is convex: vertex bounds cover the entire
    triangle. Opposite supporting planes establish the two material sides;
    the full bound includes position, angle-span and arithmetic reserves.
    """
    t=np.asarray(triangle);diameter=float(max(np.linalg.norm(a-b)for a in t for b in t));candidates=[]
    for i,b in enumerate(bodies):
        if np.any(b['hi']<t.min(0)-NUM)or np.any(b['lo']>t.max(0)+NUM):continue
        eq=b['equations'];heights=np.max(abs(t@eq[:,:3].T+eq[:,3]),axis=0)
        planes=np.flatnonzero(heights<=NUM/16)
        if not len(planes):continue
        dd,_=cell_boundary_trees[i].query(t)
        upper=float(dd.max())+NUM/128
        if upper>NUM/16:continue
        for plane in planes:candidates.append((i,eq[plane],upper,float(heights[plane])))
    for a,(i,pa,da,ha)in enumerate(candidates):
        for j,pb,db,hb in candidates[a+1:]:
            if i==j or pa[:3]@pb[:3]>-.999999999:continue
            angle_span=float(np.linalg.norm(pa[:3]+pb[:3])*diameter)
            bound=da+db+ha+hb+angle_span+NUM/8
            if bound<=NUM:
                return {'status':'EQUIVALENT_CLOSURE_WITHIN_FROZEN_SUPPORT','cells':[bodies[i]['id'],bodies[j]['id']],
                        'vertex_to_frozen_face_upper_mm':[da,db],'support_plane_height_upper_mm':[ha,hb],
                        'normal_mismatch_over_triangle_span_mm':angle_span,'combined_gap_upper_mm':bound,
                        'proof':'Vertex distances to each convex material set bound every triangle point; opposite support planes bound the two-sided closure, with explicit numerical reserve. No larger effective void can lie across this patch.'}
    return None


def filter_existing_atlas(cells):
    from analyze_runtime_partition import TriangleBVH
    import inspect
    started=time.time();bodies,_=support_cells(cells)
    trees=[TriangleBVH(np.array(e['vertices_m'])[e['faces']]*1000)for e in cells]
    path=ROOT/'reports/nero-gripper-v21-boundary-atlas.npz';rt=np.load(path)['triangles_mm'];kept=[];removed=[]
    for i,t in enumerate(rt):
        proof=closure_pair_certificate(t,bodies,trees)
        if proof:removed.append({'atlas_triangle':i,**proof})
        else:kept.append(i)
        if(i+1)%2000==0:print('Pair-filter',i+1,len(rt),'proved',len(removed),flush=True)
    np.savez_compressed(ROOT/'reports/nero-gripper-v21-effective-exterior.npz',triangles_mm=rt[kept],original_atlas_indices=np.array(kept))
    report={'numerical_support_mm':NUM,'input_triangles':len(rt),'remaining_triangles':len(kept),'continuous_equivalence_proofs':removed,'atlas_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'filter_function_sha256':hashlib.sha256(inspect.getsource(closure_pair_certificate).encode()).hexdigest(),'seconds':time.time()-started}
    (ROOT/'reports/nero-gripper-v21-effective-exterior.json').write_text(json.dumps(report,separators=(',',':'))+'\n')
    return report


def build_atlas(cells):
    started=time.time();bodies,representation=support_cells(cells)
    bounds=np.array([[c['lo'],c['hi']] for c in bodies])
    groups={};partition=[];mask_proofs=[];numeric_strips=[];unknown=[]
    for i,body in enumerate(bodies):
        near=np.flatnonzero(np.all(bounds[:,1]>=body['lo']-NUM,axis=1)&np.all(bounds[:,0]<=body['hi']+NUM,axis=1))
        for fi,facet in enumerate(body['facets']):
            plane=facet['plane'];n=plane[:3];u,w=facet['u'],facet['w'];base=facet['shape'];q=facet['vertices'];covers=[]
            for j in near:
                if i==j:continue
                # A same-facing exterior neighbour cannot cover either
                # outward probe; opposing/overlapping material can.
                eq=bodies[j]['equations'].copy()
                dot=eq[:,:3]@n
                eq[:,3]+=np.maximum(PROBE*dot,2*PROBE*dot)+NUM/128
                if np.any(np.min(q@eq[:,:3].T+eq[:,3],axis=0)>NUM/128):continue
                cut=q
                for row in eq:
                    cut=clip(cut,row)
                    if len(cut)<3:break
                if len(cut)<3:continue
                pg=area_shape(polygon(cut,u,w).intersection(base))
                if pg.area<=GRID**2:continue
                # Independently certify the WHOLE covering polygon at both
                # outward probe locations by affine vertex maxima.
                maximum=-np.inf
                for part in pieces(pg):
                    xy=np.array(part.exterior.coords)[:-1]
                    xyz=xy[:,0,None]*u+xy[:,1,None]*w-plane[3]*n
                    maximum=max(maximum,float(max(np.max((xyz+step*n)@bodies[j]['equations'][:,:3].T+bodies[j]['equations'][:,3]) for step in [PROBE,2*PROBE])))
                if maximum>NUM/64:
                    unknown.append({'entity':body['id'],'facet':fi,'neighbour':bodies[j]['id'],'reason':'Coverage arithmetic exceeded reserve','residual_mm':maximum});continue
                covers.append(pg)
                mask_proofs.append({'entity':body['id'],'facet':fi,'covering_entity':bodies[j]['id'],'area_mm2':float(pg.area),'maximum_probe_support_residual_mm':maximum,'maximum_normal_bridge_mm':2*PROBE})
            removed=area_shape(unary_union(covers))
            exposed=area_shape(base.difference(removed)) if covers else base
            if covers and not exposed.is_empty:
                # Thin residual strips adjacent to a continuously covered
                # patch have in-plane distance <=NUM/8 and bridge depth
                # <=NUM/2, strictly below the1e-7mm equivalence scale.
                expanded=area_shape(set_precision(removed.buffer(BAND,quad_segs=64),GRID))
                band=area_shape(exposed.intersection(expanded,grid_size=GRID))
                if not band.is_empty:
                    numeric_strips.append({'entity':body['id'],'facet':fi,'area_mm2':float(band.area),'in_plane_distance_upper_mm':BAND+2*GRID,'normal_bridge_upper_mm':2*PROBE,'combined_upper_mm':float(np.hypot(BAND+2*GRID,2*PROBE)+NUM/32)})
                    exposed=area_shape(exposed.difference(band))
            accounted=removed.union(exposed)
            expanded=area_shape(set_precision(accounted.buffer(BAND+2*GRID,quad_segs=64),GRID))
            missing=area_shape(base.difference(expanded,grid_size=GRID))
            if missing.area>GRID**2:
                bounds_missing=missing.bounds
                diameter=float(np.hypot(bounds_missing[2]-bounds_missing[0],bounds_missing[3]-bounds_missing[1]))
                distance=float(missing.representative_point().distance(accounted))
                if distance+diameter+NUM/32<=NUM:
                    numeric_strips.append({'entity':body['id'],'facet':fi,'area_mm2':float(missing.area),'point_distance_plus_bbox_diameter_upper_mm':distance+diameter+NUM/32,'proof':'Continuous Lipschitz bound to accounted set; no inference from area alone.'})
                else:unknown.append({'entity':body['id'],'facet':fi,'reason':'Facet partition not covered by exterior/bridged material/numeric band','area_mm2':float(missing.area),'bbox_diameter_mm':diameter,'distance_to_accounted_mm':distance})
            partition.append({'entity':body['id'],'facet':fi,'input_area_mm2':float(base.area),'covered_area_mm2':float(removed.area),'effective_exterior_area_mm2':float(exposed.area),'unaccounted_area_mm2':float(missing.area)})
            if exposed.area<=GRID**2:continue
            key=tuple(np.round(plane,11))
            if key not in groups:groups[key]={'plane':plane,'u':u,'w':w,'shapes':[],'owners':[]}
            group=groups[key]
            if np.linalg.norm(group['plane']-plane)>NUM/32:
                key=(i,fi);groups[key]={'plane':plane,'u':u,'w':w,'shapes':[],'owners':[]};group=groups[key]
            for pg in pieces(exposed):
                xy=np.array(pg.exterior.coords)[:-1];xyz=xy[:,0,None]*u+xy[:,1,None]*w-plane[3]*n
                height=float(np.max(abs(xyz@group['plane'][:3]+group['plane'][3])))
                if height>NUM/32:unknown.append({'entity':body['id'],'facet':fi,'reason':'Coplanar-group height beyond reserve','height_mm':height})
                # Exposed patches are generally NONCONVEX and may have holes.
                # Reprojection must preserve all rings; convex_hull here would
                # refill removed internal material and invent pseudo-exterior.
                def ring_coordinates(ring):
                    xy=np.array(ring.coords)
                    xyz=xy[:,0,None]*u+xy[:,1,None]*w-plane[3]*n
                    return np.c_[xyz@group['u'],xyz@group['w']]
                reprojected=Polygon(ring_coordinates(pg.exterior),[ring_coordinates(ring)for ring in pg.interiors])
                group['shapes'].append(set_precision(reprojected,GRID));group['owners'].append(i)
        if (i+1)%50==0:print('Boundary atlas cells',i+1,len(bodies),'unknown',len(unknown),flush=True)
    triangles=[];owners=[]
    for g in groups.values():
        shape=unary_union(g['shapes']);n=g['plane'][:3];d=g['plane'][3]
        # Buffer arcs are verification-only uncertainty boundaries, not new
        # robot features. Simplify this derived atlas within the remaining
        # NUM/64 reserve; preserve topology and every frozen candidate vertex.
        shape=shape.simplify(NUM/64,preserve_topology=True)
        for pg in pieces(shape):
            for tri in constrained_delaunay_triangles(pg).geoms:
                xy=np.array(tri.exterior.coords)[:3]
                q=xy[:,0,None]*g['u']+xy[:,1,None]*g['w']-d*n
                if np.cross(q[1]-q[0],q[2]-q[0])@n<0:q=q[[0,2,1]]
                triangles.append(q);owners.append(sorted(set(g['owners'])))
    return np.array(triangles),owners,{'status':'PASS_MODULO_FROZEN_SUPPORT' if not unknown and all(c['status']=='PASS'for c in representation) else 'UNKNOWN',
                                      'representation_checks':representation,'input_facet_partition':partition,'normal_bridge_proofs':mask_proofs,'numeric_strip_proofs':numeric_strips,'unresolved':unknown,'effective_exterior_triangles':len(triangles),'support_mm':NUM,'seconds':time.time()-started,
                                      'derived_atlas_simplification_reserve_mm':NUM/64,
                                      'scope':'Verifier-only boundary atlas. Candidate vertices/faces untouched; internal masks have continuous convex support proofs at fixed outward probes.'}


if __name__=='__main__':
    data=json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text())
    if '--filter-existing'not in sys.argv:
        tri,owners,certificate=build_atlas(data['entities'])
        np.savez_compressed(ROOT/'reports/nero-gripper-v21-boundary-atlas.npz',triangles_mm=tri)
        certificate['triangle_owner_cell_indices']=owners
        (ROOT/'reports/nero-gripper-v21-boundary-atlas.json').write_text(json.dumps(certificate,separators=(',',':'),allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in certificate.items()if k in ['status','effective_exterior_triangles','seconds']}),flush=True)
    print('Effective exterior',filter_existing_atlas(data['entities'])['remaining_triangles'],flush=True)
