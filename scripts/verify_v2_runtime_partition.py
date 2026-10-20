"""Certify the single saved v2 partition; never modifies its geometry.

Mesh64 Boolean exterior + independent convex-face coverage; tier-specific
surface metrics; frozen workplanes, scale, mapping and void checks.
"""
from pathlib import Path
import collections
import csv
import hashlib
import inspect
import json
import math
import sys
import time

import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import linprog,nnls

from build_v2_runtime_partition import ROOT,TASK,POLICY_COMMIT,frozen,mesh_solid,union,sha,insphere
import manifold3d as mf
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import constrained_delaunay_triangles,set_precision
from analyze_runtime_partition import TriangleBVH,triangle_distance,exterior,samples
from runtime_contact_partition import load_source
from runtime_backing_partition import segment_clearance

NUM=1e-7  # mm; explicit independent numerical certificate resolution
BLOCKED_UNION_DIAGNOSTICS=False


def write(name,obj):
    (ROOT/name).write_text(json.dumps(obj,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')


def cached_surface(source,target,limit,gap,label):
    key=hashlib.sha256(source.tobytes()+target.tobytes()+inspect.getsource(SurfaceCover).encode()+repr((limit,gap)).encode()).hexdigest()
    path=TASK/('surface-'+label+'-'+key[:16]+'.json')
    if path.exists():return json.loads(path.read_text())
    if BLOCKED_UNION_DIAGNOSTICS:
        # Once the mandatory independent union certificate failed, do not
        # spend unbounded refinement seeking a global PASS. Still measure ALL
        # triangles and provide a valid continuous upper bound, explicitly
        # UNKNOWN when the frozen gap/budget is not certified.
        tree=TriangleBVH(target);points=np.concatenate([source,source.mean(1)[:,None]],axis=1)
        distances,nearest=tree.query(points.reshape(-1,3));dd=distances.reshape(-1,4);ids=nearest.reshape(-1,4)
        q=target[ids];bounds=np.sqrt(triangle_distance(source[:,None],q[:,:,None]).max(2).min(1))
        lower=float(dd.max());upper=float(bounds.max()+math.sqrt(2)*NUM);i=np.unravel_index(int(dd.argmax()),dd.shape)
        result={'status':'FAIL' if lower>limit+NUM else 'PASS' if upper<=limit and upper-lower<=gap else 'UNKNOWN','observed_maximum_mm':lower,'conservative_upper_mm':upper,'cover_gap_mm':gap,'achieved_global_bound_gap_mm':upper-lower,'evaluated_triangles':len(source),'worst_sample_mm':points[i[0],i[1]].tolist(),'failures':[{'sample_mm':points[i[0],i[1]].tolist(),'distance_mm':lower}] if lower>limit+NUM else [],'method':'Complete finite set plus fixed-target-triangle convexity continuous bound. No PASS inferred from samples; adaptive tightening stopped after mandatory union-certificate failure.'}
    else:result=SurfaceCover(target).certify(source,limit,gap)
    path.write_text(json.dumps(result,separators=(',',':'))+'\n')
    return result


def topology(v,f):
    edges=collections.Counter();directed=collections.Counter()
    for face in f:
        for a,b in zip(face,np.roll(face,-1)):
            edges[tuple(sorted((int(a),int(b))))]+=1;directed[(int(a),int(b))]+=1
    return {'edge_incidence_two':all(x==2 for x in edges.values()),'oriented':all(x==directed[(b,a)] for (a,b),x in directed.items()),'used_vertices':len(set(f.reshape(-1))),'edges':len(edges),'faces':len(f)}


def clip(p,eq):
    if len(p)<3:return np.empty((0,3))
    d=p@eq[:3]+eq[3];d[abs(d)<=NUM]=0.;out=[]
    for i,(a,b) in enumerate(zip(p,np.roll(p,-1,axis=0))):
        da=d[i];db=d[(i+1)%len(p)];ia=da<=0;ib=db<=0
        if ia:out.append(a)
        if ia!=ib:out.append(a+(b-a)*da/(da-db))
    return np.array(out)


def normal(tri):
    n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);length=np.linalg.norm(n,axis=1)
    return n/length[:,None]


def basis(n):
    u=np.eye(3)[int(np.argmin(abs(n)))];u-=n*np.dot(n,u);u/=np.linalg.norm(u)
    return u,np.cross(n,u)


def projected(q,u,w):
    # Every input here is a convex triangle or a convex halfspace clipping
    # result. Reordering its identical projected vertices by their convex hull
    # removes numerical duplicate/collinear ordering. The analysis-only2D
    # precision grid is NUM/16; its displacement is included in NUM. Saved
    # collision vertices/faces and source assets are NEVER snapped or changed.
    return set_precision(Polygon(np.c_[q@u,q@w]).convex_hull,NUM/16)


def exterior_certificate(cells,v,f):
    """Independent set-boundary coverage of every closed input face.
    Coplanar same-facing cells are NEVER subtracted from each other; their
    union is covered once by the Boolean exterior. Internal opposite-facing
    faces and positive-volume burial are subtracted by convex halfspaces.
    """
    triangles=v[f];ns=normal(triangles);d=-np.einsum('ij,ij->i',ns,triangles[:,0])
    bounds=np.array([[np.min(e['vertices_m'],axis=0),np.max(e['vertices_m'],axis=0)] for e in cells])*1000
    vs=[np.array(e['vertices_m'])*1000 for e in cells];eqs=[np.array(e['halfspaces_m'])*[1,1,1,1000] for e in cells]
    cache={};failures=[];covered_faces=0;internal_faces=0;max_residual=0.;duplicate_area=0.;neighbours=collections.defaultdict(set)
    output_owner=[]
    # Every output triangle has one source convex-face provenance, continuously
    # proved by support and convexity at its three vertices.
    for ti,t in enumerate(triangles):
        ids=np.flatnonzero(np.all(bounds[:,1]>=t.min(0)-NUM,axis=1)&np.all(bounds[:,0]<=t.max(0)+NUM,axis=1));owners=[]
        for j in ids:
            eq=eqs[j];q=t@eq[:,:3].T+eq[:,3]
            if q.max()>NUM:continue
            active=np.flatnonzero(np.max(abs(q),axis=0)<=NUM)
            if len(active) and np.any(eq[active,:3]@ns[ti]>1-1e-5):owners.append(int(j))
        if not owners:failures.append({'check':'unattributed_output_triangle','triangle':ti})
        output_owner.append(owners)
    for i,e in enumerate(cells):
        near=np.flatnonzero(np.all(bounds[:,1]>=bounds[i,0]-NUM,axis=1)&np.all(bounds[:,0]<=bounds[i,1]+NUM,axis=1))
        for fi,face in enumerate(e['faces']):
            t=vs[i][face];n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n);offset=-n@t[0];u,w=basis(n)
            key=tuple(np.round(np.r_[n,offset],10))
            if key not in cache:
                ids=np.flatnonzero((ns@n>1-1e-6)&(np.max(abs(triangles@n+offset),axis=1)<=NUM))
                cache[key]=(u,w,unary_union([projected(q,u,w) for q in triangles[ids]]))
            u,w,shape=cache[key]
            base=projected(t,u,w)
            if base.area<1e-16:continue
            covering=[]
            for j in near:
                if j==i or np.max(vs[j]@n+offset)<=NUM:continue
                q=t
                for plane in eqs[j]:
                    q=clip(q,plane)
                    if len(q)<3:break
                if len(q)>=3:
                    pg=projected(q,u,w)
                    if pg.area>1e-16:covering.append(pg);neighbours[e['id']].add(cells[j]['id'])
            ext=base.difference(unary_union(covering)) if covering else base
            if ext.area<=1e-14:internal_faces+=1;continue
            missing=ext.difference(shape.buffer(NUM,quad_segs=32))
            max_residual=max(max_residual,float(missing.area))
            if missing.area>1e-14:failures.append({'check':'missing_true_external_face','entity':e['id'],'face':fi,'uncovered_area_mm2':float(missing.area)})
            else:covered_faces+=1
        if (i+1)%50==0:print('Independent exterior face coverage',i+1,len(cells),'failures',len(failures),flush=True)
    # Check no emitted triangle has positive-area interior buried in another
    # convex body. This is continuous polygon intersection, not centroid-only.
    occluded=[]
    for ti,t in enumerate(triangles):
        ids=np.flatnonzero(np.all(bounds[:,1]>=t.min(0)-NUM,axis=1)&np.all(bounds[:,0]<=t.max(0)+NUM,axis=1))
        for j in ids:
            if j in output_owner[ti] or np.max(vs[j]@ns[ti]+d[ti])<=NUM:continue
            q=t
            for plane in eqs[j]:
                inward=plane.copy();inward[3]+=2*NUM
                q=clip(q,inward)
                if len(q)<3:break
            if len(q)>=3:
                u,w=basis(ns[ti]);a=projected(q,u,w).area
                if a>NUM**2:occluded.append({'triangle':ti,'covering_entity':cells[j]['id'],'core_overlap_area_mm2':float(a)})
    duplicate_records=[]
    for i,owners in enumerate(output_owner):
        if len(owners)<2:continue
        t=triangles[i];area=float(np.linalg.norm(np.cross(t[1]-t[0],t[2]-t[0]))/2);perimeter=float(np.linalg.norm(t-np.roll(t,-1,axis=0),axis=1).sum())
        if area>NUM*perimeter:
            duplicate_records.append({'triangle':i,'entities':[cells[j]['id'] for j in owners],'same_exterior_area_mm2':area})
    for e in cells:e['union_neighbours']=sorted(neighbours[e['id']])
    return {'status':'PASS' if not failures and not occluded else 'FAIL','method':'Mesh64 Boolean union, independent complete input-face convex coverage and output-triangle provenance/interior-occlusion checks','numerical_surface_resolution_mm':NUM,'boundary_distance_numerical_upper_mm':math.sqrt(2)*NUM,'covered_external_input_faces':covered_faces,'internal_input_faces':internal_faces,'maximum_uncovered_area_mm2':max_residual,'failures':failures,'occluded_output_triangles':occluded,'duplicate_physical_exterior_records':duplicate_records,'duplicate_physical_exterior_area_mm2':sum(r['same_exterior_area_mm2'] for r in duplicate_records),'output_triangle_owner_indices':output_owner}


class SurfaceCover:
    def __init__(self,target):
        self.tri=target;self.tree=TriangleBVH(target);self.n=normal(target);self.cache={};self.planar_hits=0

    def planar(self,t,limit):
        n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n);d=-n@t[0];key=tuple(np.round(np.r_[n,d,limit],9))
        if key not in self.cache:
            ids=np.flatnonzero((abs(self.n@n)>1-1e-7)&(np.max(abs(self.tri@n+d),axis=1)<limit-math.sqrt(2)*NUM))
            if not len(ids):self.cache[key]=None
            else:
                u,w=basis(n);shape=unary_union([projected(q,u,w) for q in self.tri[ids]])
                self.cache[key]=(u,w,shape,float(np.max(abs(self.tri[ids]@n+d))))
        item=self.cache[key]
        if item is None:return None
        u,w,shape,h=item;p=projected(t,u,w);lo=0.;hi=math.sqrt(max(0,(limit-math.sqrt(2)*NUM)**2-h*h))
        if not p.difference(shape.buffer(hi,quad_segs=32)).is_empty:return None
        for _ in range(16):
            mid=(lo+hi)/2
            if p.difference(shape.buffer(mid,quad_segs=32)).is_empty:hi=mid
            else:lo=mid
        self.planar_hits+=1
        return float(np.hypot(hi,h)+math.sqrt(2)*NUM)

    def certify(self,source,limit,gap,max_nodes=180000):
        pending=list(source);init=np.concatenate([source.reshape(-1,3),source.mean(1)]);idist,_=self.tree.query(init);ii=int(np.argmax(idist));lower=float(idist[ii]);worst=init[ii].tolist();upper=0.;nodes=0;fails=[];incomplete=False
        while pending:
            t=pending.pop();p=np.r_[t,t.mean(0)[None]];dist,ids=self.tree.query(p);lo=float(dist.max());nodes+=1
            if lo>lower:lower=lo;worst=p[int(dist.argmax())].tolist()
            choices=np.unique(ids);q=self.tri[choices]
            dd=triangle_distance(t[None],q[:,None]);ub=float(np.sqrt(dd.max(1).min()))+math.sqrt(2)*NUM
            if ub>lo+gap:
                planar=self.planar(t,min(limit,lower+gap))
                if planar is not None:ub=min(ub,planar)
            if lo>limit+NUM:
                fails.append({'sample_mm':p[int(dist.argmax())].tolist(),'distance_mm':lo});upper=max(upper,ub)
            elif ub<=limit and (ub-lo<=gap or ub<=lower+gap):upper=max(upper,ub)
            elif nodes>=max_nodes:
                incomplete=True;fails.append({'reason':'unresolved_cover_complexity','remaining':len(pending)+1});break
            else:
                edges=[np.linalg.norm(t[(i+1)%3]-t[i]) for i in range(3)];i=int(np.argmax(edges));a=i;b=(i+1)%3;c=(i+2)%3;m=(t[a]+t[b])/2
                pending.extend([np.array([t[a],m,t[c]]),np.array([m,t[b],t[c]])])
            if nodes%20000==0:print('Surface certificate nodes',nodes,'lower',lower,'upper',upper,flush=True)
        return {'status':'UNKNOWN' if incomplete else 'FAIL' if fails else 'PASS','observed_maximum_mm':lower,'conservative_upper_mm':None if incomplete else upper,'cover_gap_mm':gap,'evaluated_triangles':nodes,'coplanar_continuous_cover_uses':self.planar_hits,'worst_sample_mm':worst,'failures':fails}


def tier_meshes(triangles,source_ids,policy):
    face_region={}
    for name,r in policy['source_region_policy'].items():
        for a,b in r['source_triangle_ranges_zero_based_inclusive']:
            for i in range(a,b+1):face_region[i]=name
    groups=collections.defaultdict(list)
    for t,ident in zip(triangles,source_ids):
        r=policy['source_region_policy'][face_region[int(ident)]];tier=r['tier']
        if tier in ['A','B','C']:groups[tier].append(t);continue
        # This is analysis-domain slicing on an EXISTING surface triangle;
        # it is not a new convex collision entity or candidate revision.
        threshold=r['predicate']['mouth_y_mm']-.1
        for label,plane in [('B',np.array([0.,-1,0,threshold])),('C',np.array([0.,1,0,-threshold]))]:
            q=clip(t,plane)
            for i in range(1,len(q)-1):groups[label].append(q[[0,i,i+1]])
    return {k:np.asarray(v) for k,v in groups.items()}


def normal_cover(source,target,limit=.01,degrees=.05):
    tn=normal(target);failures=[];max_angle=0.;uses=0
    for i,t in enumerate(source):
        n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n);d=-n@t[0]
        dots=tn@n;ids=np.flatnonzero((dots>=np.cos(np.deg2rad(degrees)))&(np.max(abs(target@n+d),axis=1)<=limit))
        u,w=basis(n);pg=projected(t,u,w);covers=[];angles=[]
        for j in ids:
            q=target[j];height=float(np.max(abs(q@n+d)))+math.sqrt(2)*NUM
            if height>=limit:continue
            radius=math.sqrt(limit**2-height**2)
            qp=projected(q,u,w)
            if qp.distance(pg)>radius:continue
            covers.append(qp.buffer(radius,quad_segs=32));angles.append(float(np.degrees(np.arccos(np.clip(dots[j],-1,1)))))
        if not covers or pg.difference(unary_union(covers)).area>1e-13:failures.append({'source_triangle':i,'uncovered_normal_correspondence':True})
        else:max_angle=max(max_angle,max(angles,default=0.));uses+=1
    return {'status':'PASS' if not failures else 'FAIL','maximum_corresponding_normal_deviation_deg':max_angle,'budget_deg':degrees,'continuous_normal_and_distance_covered_triangles':uses,'failures':failures}


def main():
    global BLOCKED_UNION_DIAGNOSTICS
    started=time.time();path=ROOT/'reports/nero-gripper-v2-runtime-partition.json';data=json.loads(path.read_text());cells=data['entities'];geom=data['geometry_sha256'];policy=frozen('reports/nero-gripper-v2-fidelity-protocol.json')
    assert hashlib.sha256(json.dumps([[e['id'],e['vertices_m'],e['faces']] for e in cells],separators=(',',':')).encode()).hexdigest()==geom
    print('Building actual final convex-union exterior',flush=True)
    bodies=[mesh_solid(np.array(e['vertices_m'])*1000,e['faces']) for e in cells]
    assert all(s.status()==mf.Error.NoError for s in bodies)
    body_union=union(bodies);m=body_union.to_mesh64();v=np.array(m.vert_properties[:,:3]);f=np.array(m.tri_verts,dtype=int);rt=v[f]
    topo=topology(v,f);print('Actual exterior triangles',len(f),'topology',topo,flush=True)
    cert_key=hashlib.sha256((geom+inspect.getsource(exterior_certificate)+inspect.getsource(clip)+inspect.getsource(projected)+inspect.getsource(basis)).encode()).hexdigest()
    cert_path=TASK/('exterior-'+cert_key[:16]+'.json')
    if cert_path.exists():cert=json.loads(cert_path.read_text())
    else:
        cert=exterior_certificate(cells,v,f)
        cert_path.write_text(json.dumps(cert,separators=(',',':'))+'\n')
    cert['topology']=topo;cert['kernel_status']=str(body_union.status());cert['kernel_tolerance_mm']=body_union.get_tolerance()
    if not topo['edge_incidence_two'] or not topo['oriented']:cert['status']='FAIL'
    BLOCKED_UNION_DIAGNOSTICS=cert['status']!='PASS'
    # Frozen original 3-part source union, including C microseams. Neither the
    # old provisional runtime exterior nor all closed cell faces is a target.
    source=load_source();sv=np.array(source['vertices_m']);st=sv[np.array([t['vertex_ids'] for t in source['triangles']])]*1000
    source_bodies=[]
    for a,b in [(0,504),(504,660),(660,2086)]:
        flat=st[a:b].reshape(-1,3);_,ix,inv=np.unique(np.round(flat,12),axis=0,return_index=True,return_inverse=True)
        mesh=mf.Mesh64(np.ascontiguousarray(flat[ix]),np.ascontiguousarray(inv.reshape(-1,3),dtype=np.uint64),face_id=np.arange(a,b,dtype=np.uint64))
        s=mf.Manifold(mesh);assert s.status()==mf.Error.NoError;source_bodies.append(s)
    reference=union(source_bodies);rm=reference.to_mesh64();rv=np.array(rm.vert_properties[:,:3]);rf=np.array(rm.tri_verts,dtype=int);src_ids=np.array(rm.face_id,dtype=int);ref_tri=rv[rf]
    ref_targets=tier_meshes(ref_tri,src_ids,policy)
    # Source-space C predicates are fixed by8a5518b. Slice only the derived
    # measurement triangles at its fixed collar; never relabel a whole mixed
    # collision entity. New caps inherit adjacent A/B responsibility.
    ref_tree=TriangleBVH(ref_tri);ab=np.r_[ref_targets['A'],ref_targets['B']];abtree=TriangleBVH(ab)
    centres=rt.mean(1);dist,nearest=ref_tree.query(centres);abdist,abi=abtree.query(centres)
    face_region={}
    for name,r in policy['source_region_policy'].items():
        for a,b in r['source_triangle_ranges_zero_based_inclusive']:
            for i in range(a,b+1):face_region[i]=name
    c_regions={}
    for name,entry in policy['source_region_policy'].items():
        if entry['tier'] not in ['C','B_OR_C_BY_SOURCE_POSITION']:continue
        ids=[i for i,j in enumerate(src_ids) if face_region[int(j)]==name]
        if ids:
            cc=tier_meshes(ref_tri[ids],src_ids[ids],policy).get('C',np.empty((0,3,3)))
            if len(cc):c_regions[name]=(cc,TriangleBVH(cc),normal(cc))
    runtime_labels=[];runtime_groups=collections.defaultdict(list);ambiguous=[];c_provenance=[]
    tier_normals={k:normal(q) for k,q in ref_targets.items()};overlay_cache={};overlay_pieces=0
    def append_frozen_region_overlay(piece,default_tier):
        nonlocal overlay_pieces
        n=normal(piece[None])[0];d=-n@piece[0];key=tuple(np.round(np.r_[n,d],10))
        if key not in overlay_cache:
            u,w=basis(n);shapes={}
            for tier,targets in ref_targets.items():
                cop=np.flatnonzero((tier_normals[tier]@n>1-1e-7)&(np.max(abs(targets@n+d),axis=1)<=NUM))
                shapes[tier]=unary_union([projected(q,u,w) for q in targets[cop]])
            overlay_cache[key]=(u,w,n,d,shapes)
        u,w,nn,dd,shapes=overlay_cache[key];remaining=projected(piece,u,w)
        for label in ['A','B','C',default_tier]:
            hit=remaining if label==default_tier and label not in ['A','B','C'] else remaining.intersection(shapes[label])
            if hit.area>1e-14:
                for tr in constrained_delaunay_triangles(hit).geoms:
                    xy=np.array(tr.exterior.coords)[:3];q=xy[:,0,None]*u+xy[:,1,None]*w-dd*nn
                    if np.dot(np.cross(q[1]-q[0],q[2]-q[0]),nn)<0:q=q[[0,2,1]]
                    runtime_groups[label].append(q);overlay_pieces+=1
            remaining=remaining.difference(shapes[label])
        if remaining.area>1e-14:
            for tr in constrained_delaunay_triangles(remaining).geoms:
                xy=np.array(tr.exterior.coords)[:3];q=xy[:,0,None]*u+xy[:,1,None]*w-dd*nn
                if np.dot(np.cross(q[1]-q[0],q[2]-q[0]),nn)<0:q=q[[0,2,1]]
                runtime_groups[default_tier].append(q)
    for i,(t,j) in enumerate(zip(rt,nearest)):
        region=face_region[int(src_ids[j])];entry=policy['source_region_policy'][region];tier=entry['tier']
        pieces=[]
        if tier=='B_OR_C_BY_SOURCE_POSITION':
            threshold=entry['predicate']['mouth_y_mm']-.1
            for label,plane in [('B',np.array([0.,-1,0,threshold])),('C',np.array([0.,1,0,-threshold]))]:
                q=clip(t,plane)
                pieces.extend((label,q[[0,k,k+1]]) for k in range(1,len(q)-1) if np.linalg.norm(np.cross(q[k]-q[0],q[k+1]-q[0]))>1e-14)
        else:pieces=[(tier,t)]
        labels=[]
        for label,piece in pieces:
            if label=='C':
                cc,ctree,cn=c_regions[region];full_dist,_=ctree.query(np.r_[piece,piece.mean(0)[None]])
                near_c=np.flatnonzero(np.all(cc.max(1)>=piece.min(0)-.05,axis=1)&np.all(cc.min(1)<=piece.max(0)+.05,axis=1))
                pn=normal(piece[None])[0]
                residual=nnls(cn[near_c].T,pn)[1] if len(near_c) else 1.
                if full_dist.max()>.05+NUM or residual>NUM:
                    # A novel cap not supported by its parent C surface cone
                    # cannot receive a C waiver. This is a sufficient provenance
                    # check, not a new C normal-error budget.
                    label='A' if abi[i]<len(ref_targets['A']) else 'B'
                    ambiguous.append({'triangle':i,'frozen_C_region':region,'distance_to_C_samples_mm':float(full_dist.max()),'source_normal_cone_residual':float(residual),'conservative_inherited_tier':label,'status':'UNKNOWN_C_PROVENANCE_OR_NEW_A_B_CAP'})
                else:c_provenance.append({'triangle':i,'frozen_C_region':region,'normal_cone_residual':float(residual)})
            labels.append(label);append_frozen_region_overlay(piece,label)
        runtime_labels.append(labels[0] if len(set(labels))==1 else 'M')
    runtime_targets={k:np.array(q) for k,q in runtime_groups.items()}
    np.savez_compressed(ROOT/'reports/nero-gripper-v2-union-exterior.npz',vertices_mm=v,faces=f,reference_vertices_mm=rv,reference_faces=rf,reference_source_face_ids=src_ids,runtime_face_tier_hints=np.array(runtime_labels),reference_A_triangles_mm=ref_targets['A'],reference_B_triangles_mm=ref_targets['B'],reference_C_triangles_mm=ref_targets['C'],runtime_A_triangles_mm=runtime_targets.get('A',np.empty((0,3,3))),runtime_B_triangles_mm=runtime_targets.get('B',np.empty((0,3,3))),runtime_C_triangles_mm=runtime_targets.get('C',np.empty((0,3,3))))
    metrics={}
    for tier in ['A','B']:
        limit=policy['tiers'][tier]['outward_surface_mm'];gap=policy['tiers'][tier]['symmetric_surface_certificate_gap_mm']
        print('Certifying',tier,'source to actual runtime',len(ref_targets[tier]),len(runtime_targets[tier]),flush=True)
        a=cached_surface(ref_targets[tier],runtime_targets[tier],limit,gap,tier+'-reverse')
        print('Certifying',tier,'actual runtime to frozen source',flush=True)
        b=cached_surface(runtime_targets[tier],ref_targets[tier],limit,gap,tier+'-forward')
        metrics[tier]={'reference_to_runtime':a,'runtime_to_reference':b,'status':'PASS' if a['status']==b['status']=='PASS' else 'FAIL' if 'FAIL' in [a['status'],b['status']] else 'UNKNOWN'}
    print('Certifying A normal correspondence',flush=True)
    metrics['A']['normal_reference_to_runtime']=normal_cover(ref_targets['A'],runtime_targets['A'])
    metrics['A']['normal_runtime_to_reference']=normal_cover(runtime_targets['A'],ref_targets['A'])
    if any(metrics['A'][k]['status']!='PASS' for k in ['normal_reference_to_runtime','normal_runtime_to_reference']):metrics['A']['status']='FAIL'
    # Independent current scale and mapping checks; no source model edits.
    ref=json.loads((ROOT/'reports/nero-gripper-entity-partition.json').read_text());byref={e['id']:e for e in ref['entities']};owner={r:e for e in cells for r in e['reference_entity_ids']}
    assert collections.Counter(r for e in cells for r in e['reference_entity_ids'])==collections.Counter(byref.keys())
    scale=[];containment=0.
    for e in cells:
        vv=np.array(e['vertices_m'])*1000;h=ConvexHull(vv);diameter,centre=insphere(h.equations)
        for r in e['reference_entity_ids']:containment=max(containment,float(np.max(np.array(byref[r]['vertices_m'])*1000@h.equations[:,:3].T+h.equations[:,3])))
        scale.append({'id':e['id'],'diameter_mm':diameter,'status':'PASS' if diameter>=.1-1e-8 else 'FAIL'})
    thin=frozen('reports/nero-gripper-v2-thin-classification.json')['entities'];mapping=[]
    for old in thin:
        targets=sorted({owner[r]['id'] for r in old['reference_entity_ids']});standalone=len(targets)==1 and set(owner[old['reference_entity_ids'][0]]['reference_entity_ids'])==set(old['reference_entity_ids'])
        mapping.append({'v1_id':old['id'],'tier':old['classification_tier'],'v2_ids':targets,'C_identity_retired':not standalone if old['classification_tier']=='C' else None,'all_owner_scales_pass':all(next(s['status'] for s in scale if s['id']==i)=='PASS' for i in targets),'A_B_obligations_unchanged':old['retained_A_B_surface_obligations']})
    # The original historical168 points, no reuse of its provisional exterior
    # as the v2 query target. Saved sample indexing is reconstructed exactly.
    from classify_v2_fidelity import WEIGHTS
    tri=[];meta=[]
    for e in ref['entities']:
        for sp in e['source_surface_partition']:
            for p in sp['exterior_subpolygons']:
                q=np.array(p['polygon_m'])*1000
                for j in range(1,len(q)-1):tri.append(q[[0,j,j+1]]);meta.extend([sp['source_face_id']]*7)
    oldpts=np.einsum('wv,tvc->twc',WEIGHTS,np.array(tri)).reshape(-1,3)
    old1=json.loads((ROOT/'reports/nero-gripper-runtime-partition.json').read_text());oldtri,_=exterior(old1,True);olddist,_=TriangleBVH(oldtri).query(oldpts)
    is_a=np.array([policy['source_region_policy'][face_region[i]]['tier']=='A' for i in meta]);ids=np.flatnonzero(is_a&(olddist>.01+1e-8));assert len(ids)==168
    newdist,_=TriangleBVH(rt).query(oldpts[ids]);history=[{'old_sample_index':int(k),'source_face_id':int(meta[k]),'old_provisional_distance_mm':float(olddist[k]),'v2_actual_union_distance_mm':float(d),'within_10um':bool(d<=.01+NUM)} for k,d in zip(ids,newdist)]
    # Material and protected-void verification is completed separately below.
    report={'status':'V2_RUNTIME_ENTITY_PARTITION_BLOCKED','geometry_sha256':geom,'policy_commit':POLICY_COMMIT,'policy_blob_sha256':data['policy_blob_sha256'],'entity_count_per_finger':len(cells),'counts_by_semantic_family':data['counts_by_semantic_family'],'reduction_from_1253':1253-len(cells),'reduction_from_753':753-len(cells),'minimum_inscribed_diameter_mm':min(x['diameter_mm'] for x in scale),'scale_checks':scale,'maximum_reference_containment_residual_mm':containment,'union_exterior_certificate':cert,'tier_metrics':metrics,'historical_168_A_samples':history,'historical_168_maximum_mm':float(newdist.max()),'historical_168_all_within_10um':bool(np.all(newdist<=.01+NUM)),'thin_129_mapping':mapping,'C_identities_retired':sum(e['C_identity_retired'] is True for e in mapping),'forward_C_provenance':{'proven_sufficient_source_cone_records':len(c_provenance),'unresolved_or_new_caps':ambiguous},'protected_voids':'PENDING','material_and_C_domain':'PENDING','verifier_sha256':sha(Path(__file__)),'elapsed_seconds':time.time()-started,'scope':{'mujoco_imported':any(n.startswith('mujoco') for n in sys.modules),'new_candidate_count':1,'formal_mjcf':0,'policy_changes':0}}
    write('reports/nero-gripper-v2-runtime-verification.json',report)
    print(json.dumps({'exterior_status':cert['status'],'exterior_failures':len(cert['failures']),'occluded_faces':len(cert['occluded_output_triangles']),'tier_status':{k:x['status'] for k,x in metrics.items()},'minimum_scale_mm':report['minimum_inscribed_diameter_mm'],'C_retired':report['C_identities_retired'],'history168_max_mm':report['historical_168_maximum_mm']},indent=2),flush=True)


if __name__=='__main__':main()
