"""Reaccept the same frozen346 cells under separately committed v2.1 rules.

Reports genuine failures separately from incomplete verifier proofs. Derived
surface atlases/halfspaces are analysis artifacts, never a new candidate.
"""
from pathlib import Path
import collections
import copy
import hashlib
import inspect
import json
import math
import subprocess
import sys
import time
from itertools import product

import numpy as np
from scipy.optimize import nnls
from scipy.spatial import ConvexHull

from v21_boundary_atlas import ROOT,NUM,GRID,basis,polygon,pieces,area_shape,clip,support_cells,closure_pair_certificate
from shapely.ops import unary_union
from shapely import constrained_delaunay_triangles
from analyze_runtime_partition import TriangleBVH,triangle_distance
from verify_v2_runtime_partition import SurfaceCover,normal_cover,tier_meshes,normal
from build_v2_runtime_partition import mesh_solid,union,insphere
from runtime_contact_partition import load_source
from runtime_backing_partition import segment_clearance
import manifold3d as mf

TASK=ROOT/'debug/issue25/v21-reacceptance'


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def save(p,obj):Path(p).write_text(json.dumps(obj,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')


def policy():
    revision=json.loads((ROOT/'reports/nero-gripper-v21-fidelity-protocol.json').read_text())
    raw=subprocess.check_output(['git','show',revision['inherits']['commit']+':'+revision['inherits']['path']],cwd=ROOT)
    assert hashlib.sha256(raw).hexdigest()==revision['inherits']['sha256']
    p=json.loads(raw);old=copy.deepcopy(p)
    matches=[f for f in p['protected_void_policy']['features'] if f['name']=='central_through_bore'];assert len(matches)==1
    assert matches[0]['rule']==revision['protected_void_single_override']['old_rule']
    matches[0]['rule']=revision['protected_void_single_override']['new_rule']
    assert old['tiers']==p['tiers'] and old['minimum_runtime_entity_scale']==p['minimum_runtime_entity_scale']
    return p,revision


def label_atlas(rt,old,p):
    ref=old['reference_vertices_mm'][old['reference_faces']];source_ids=old['reference_source_face_ids']
    targets={k:old['reference_'+k+'_triangles_mm'] for k in ['A','B','C']}
    face_region={}
    for name,r in p['source_region_policy'].items():
        for a,b in r['source_triangle_ranges_zero_based_inclusive']:
            for i in range(a,b+1):face_region[i]=name
    dist,nearest=TriangleBVH(ref).query(rt.mean(1));ab=np.r_[targets['A'],targets['B']];_,abi=TriangleBVH(ab).query(rt.mean(1))
    c_regions={}
    for name,entry in p['source_region_policy'].items():
        if entry['tier'] not in ['C','B_OR_C_BY_SOURCE_POSITION']:continue
        ids=[i for i,f in enumerate(source_ids) if face_region[int(f)]==name]
        if ids:
            cc=tier_meshes(ref[ids],source_ids[ids],p).get('C',np.empty((0,3,3)))
            if len(cc):c_regions[name]=(cc,TriangleBVH(cc),normal(cc))
    groups=collections.defaultdict(list);cache={};unresolved=[];proven=[]
    norms={k:normal(v) for k,v in targets.items()}
    for i,(t,j) in enumerate(zip(rt,nearest)):
        region=face_region[int(source_ids[j])];entry=p['source_region_policy'][region];tier=entry['tier'];parts=[]
        if tier=='B_OR_C_BY_SOURCE_POSITION':
            threshold=entry['predicate']['mouth_y_mm']-.1
            for lab,plane in [('B',np.array([0.,-1,0,threshold])),('C',np.array([0.,1,0,-threshold]))]:
                q=clip(t,plane)
                parts.extend((lab,q[[0,k,k+1]]) for k in range(1,len(q)-1))
        else:parts=[(tier,t)]
        for lab,piece in parts:
            if np.linalg.norm(np.cross(piece[1]-piece[0],piece[2]-piece[0]))<=GRID**2:continue
            n=normal(piece[None])[0];d=-n@piece[0]
            if lab=='C':
                cc,tree,cn=c_regions[region];dd,_=tree.query(np.r_[piece,piece.mean(0)[None]])
                near_c=np.flatnonzero(np.all(cc.max(1)>=piece.min(0)-.05,axis=1)&np.all(cc.min(1)<=piece.max(0)+.05,axis=1))
                residual=nnls(cn[near_c].T,n)[1] if len(near_c) else 1.
                if dd.max()>.05+NUM or residual>NUM:
                    lab='A' if abi[i]<len(targets['A']) else 'B'
                    unresolved.append({'atlas_triangle':i,'source_C_region':region,'sample_distance_to_C_mm':float(dd.max()),'source_cone_residual':float(residual),'conservative_tier':lab})
                else:proven.append({'atlas_triangle':i,'source_C_region':region,'sample_distance_to_C_mm':float(dd.max()),'source_cone_residual':float(residual)})
            key=tuple(np.round(np.r_[n,d],11))
            if key not in cache:
                u,w=basis(n);shapes={}
                for label,tri in targets.items():
                    cop=np.flatnonzero((norms[label]@n>1-1e-7)&(np.max(abs(tri@n+d),axis=1)<=NUM))
                    shapes[label]=area_shape(unary_union([polygon(q,u,w)for q in tri[cop]]))
                cache[key]=(u,w,n,d,shapes)
            u,w,nn,dd,shapes=cache[key];remaining=polygon(piece,u,w)
            for label in ['A','B','C']:
                hit=area_shape(remaining.intersection(shapes[label],grid_size=GRID))
                for pg in pieces(hit):
                    for tr in constrained_delaunay_triangles(pg).geoms:
                        xy=np.array(tr.exterior.coords)[:3];q=xy[:,0,None]*u+xy[:,1,None]*w-dd*nn
                        if np.cross(q[1]-q[0],q[2]-q[0])@nn<0:q=q[[0,2,1]]
                        groups[label].append(q)
                remaining=area_shape(remaining.difference(shapes[label],grid_size=GRID))
            for pg in pieces(remaining):
                for tr in constrained_delaunay_triangles(pg).geoms:
                    xy=np.array(tr.exterior.coords)[:3];q=xy[:,0,None]*u+xy[:,1,None]*w-dd*nn
                    if np.cross(q[1]-q[0],q[2]-q[0])@nn<0:q=q[[0,2,1]]
                    groups[lab].append(q)
    return targets,{k:np.array(q)for k,q in groups.items()},{'sufficient_C_provenance_records':proven,'C_provenance_unresolved_or_new_AB_caps':unresolved}


def surface(source,target,limit,gap,label):
    key=hashlib.sha256(source.tobytes()+target.tobytes()+inspect.getsource(SurfaceCover).encode()+str((limit,gap)).encode()).hexdigest()
    path=TASK/('surface-'+label+'-'+key[:16]+'.json')
    if path.exists():return json.loads(path.read_text())
    print('Continuous surface',label,len(source),len(target),flush=True)
    result=SurfaceCover(target).certify(source,limit,gap)
    save(path,result);return result


def contour_edges(tri):
    """Cancel oriented collinear overlapping edges, including T-junctions.
    Rounded keys only locate candidates; each line's perpendicular residual
    is explicitly bounded. Coordinates of candidate geometry never change.
    """
    groups={};unknown=[]
    for t in tri:
        for a,b in zip(t,np.roll(t,-1,axis=0)):
            delta=b-a;length=np.linalg.norm(delta)
            if length<=NUM/32:continue
            direction=delta/length;k=int(np.argmax(abs(direction)))
            if direction[k]<0:direction=-direction
            anchor=a-direction*(a@direction)
            key=tuple(np.round(np.r_[direction,anchor],10))
            if key not in groups:groups[key]={'direction':direction,'anchor':anchor,'edges':[]}
            g=groups[key]
            residual=max(np.linalg.norm((p-g['anchor'])-g['direction']*((p-g['anchor'])@g['direction']))for p in [a,b])
            if residual>NUM/8:
                unknown.append({'line_key':key,'residual_mm':float(residual)})
            x,y=float(a@g['direction']),float(b@g['direction']);g['edges'].append((min(x,y),max(x,y),1 if y>x else -1))
    edges=[]
    for g in groups.values():
        events=collections.defaultdict(int)
        for a,b,s in g['edges']:
            # Analysis endpoint equivalence reserve, not a geometry edit.
            events[round(a/(NUM/32))]+=s;events[round(b/(NUM/32))]-=s
        xs=sorted(events);balance=0
        for i,x in enumerate(xs[:-1]):
            balance+=events[x]
            if balance:
                edges.append([g['anchor']+g['direction']*(x*NUM/32),g['anchor']+g['direction']*(xs[i+1]*NUM/32)])
    return np.array(edges),unknown


def point_segments(points,segments):
    result=[];indices=[]
    a=segments[:,0];v=segments[:,1]-a;den=np.sum(v*v,axis=1)
    for p in points:
        q=p-a;r=np.clip(np.sum(q*v,axis=1)/den,0,1);dd=np.sum((q-r[:,None]*v)**2,axis=1);j=int(dd.argmin());result.append(np.sqrt(dd[j]));indices.append(j)
    return np.array(result),np.array(indices)


def contour_cover(source,target,limit,gap):
    if not len(source)or not len(target):return {'status':'UNKNOWN','reason':'No complete contour domain'}
    pending=list(source);lower=0.;upper=0.;nodes=0;incomplete=False;worst=None
    while pending:
        s=pending.pop();points=np.r_[s,s.mean(0)[None]];d,ids=point_segments(points,target);lo=float(d.max());nodes+=1
        if lo>lower:lower=lo;worst=points[int(d.argmax())].tolist()
        candidates=target[np.unique(ids)];ends=[]
        for q in candidates:
            dd,_=point_segments(s,q[None]);ends.append(dd.max())
        ub=float(min(ends))+NUM/2
        if lo>limit+NUM:upper=max(upper,ub)
        elif ub<=limit and ub<=lower+gap:upper=max(upper,ub)
        elif nodes>=80000:incomplete=True;break
        else:
            mid=s.mean(0);pending.extend([np.array([s[0],mid]),np.array([mid,s[1]])])
    return {'status':'FAIL'if lower>limit+NUM else 'UNKNOWN'if incomplete else 'PASS','observed_maximum_mm':lower,'conservative_upper_mm':None if incomplete else upper,'cover_gap_mm':gap,'evaluated_segments':nodes,'worst_sample_mm':worst,'method':'Endpoint/midpoint lower bound; fixed-target-segment convexity continuous upper; adaptive bisection.'}


def material_voids(cells,p):
    """Recompute material and named void checks; never write old reports."""
    source=load_source();st=np.array(source['vertices_m'])[np.array([t['vertex_ids']for t in source['triangles']])]*1000
    parts=[]
    for a,b in [(0,504),(504,660),(660,2086)]:
        flat=st[a:b].reshape(-1,3);_,ix,inv=np.unique(np.round(flat,12),axis=0,return_index=True,return_inverse=True)
        part=mf.Manifold(mf.Mesh64(np.ascontiguousarray(flat[ix]),np.ascontiguousarray(inv.reshape(-1,3),dtype=np.uint64)));assert part.status()==mf.Error.NoError;parts.append(part)
    source_body=union(parts);runtime_body=union([mesh_solid(np.array(e['vertices_m'])*1000,e['faces'])for e in cells])
    delta=.04999;half=delta/np.sqrt(3);cube=mf.Manifold.cube((2*half,)*3,center=True)
    out=runtime_body-source_body.minkowski_sum(cube);inside=source_body-runtime_body.minkowski_sum(cube)
    small=NUM/2;small_half=small/np.sqrt(3)
    numerical_loss=source_body-runtime_body.minkowski_sum(mf.Manifold.cube((2*small_half,)*3,center=True))
    material={'status':'PASS'if out.status()==inside.status()==mf.Error.NoError and out.is_empty() and inside.is_empty() else 'UNKNOWN',
              'outward_upper_mm':delta,'inward_upper_mm':delta,'source_minus_runtime_numerical_offset_empty':numerical_loss.is_empty(),'source_minus_runtime_numerical_offset_status':str(numerical_loss.status()),'effective_material_loss_radius_upper_mm':small if numerical_loss.is_empty() else None,
              'outward_boolean_difference_empty':out.is_empty(),'inward_boolean_difference_empty':inside.is_empty(),'budget_mm':.05,
              'C_domains':'Unchanged fixed source domains. Material distances alone do not certify C provenance or authorize new C caps.'}
    bodies,support_checks=support_cells(cells)
    gs={i:{'poly':{'v':b['vertices'],'eq':b['equations'],'lo':b['lo'],'hi':b['hi']}}for i,b in enumerate(bodies)}
    evidence=json.loads((ROOT/'reports/nero-gripper-runtime-evidence.json').read_text())['constructors']
    segments=[]
    for f in evidence['backing']['verification.json']['protected_void_witnesses']:
        for q in f['probes']:
            segments.append({'feature':f['feature'],'a_mm':q['a_mm'],'b_mm':q['b_mm'],'runtime':segment_clearance(q['a_mm'],q['b_mm'],gs)})
    mount=evidence['mount']['feature-witnesses.json'];points=[]
    for q in mount['witnesses']:
        point=np.array(q['point_m'])*1000
        occupants=[b['id']for b in bodies if np.max(b['equations'][:,:3]@point+b['equations'][:,3])<=NUM/32]
        expected=q['expected']=='solid'
        points.append({'feature':q['feature'],'point_mm':point.tolist(),'expected':q['expected'],'occupants':occupants,'expected_occupancy_matches':bool(occupants)==expected})
    from verify_v2_material_voids import axis_intervals
    floors=[]
    for q in mount['blind_recess_axis_floors']:
        x,z=q['axis_xz_mm'];intervals=axis_intervals(x,z,gs)
        choices=[b for a,b,i in intervals if -73.5<b<-70.1]
        runtime=max(choices)if choices else None
        source_floor=q.get('source_floor_y_m',q.get('floor_y_m',0))
        # The frozen witness stores the authoritative floor in millimetres.
        original=json.loads((ROOT/'reports/nero-gripper-v2-material-voids.json').read_text())['protected_voids']['blind_recess_floors']
        old=next(f['source_floor_y_mm']for f in original if f['axis_xz_mm']==q['axis_xz_mm'])
        floors.append({'axis_xz_mm':q['axis_xz_mm'],'source_floor_y_mm':old,'runtime_floor_y_mm':runtime,'deviation_mm':None if runtime is None else runtime-old,'status':'PASS'if runtime is not None and abs(runtime-old)<=.05 else'UNKNOWN'})
    from adjudicate_v2_partition import source_line_intervals
    central=[]
    for x,z in [(0.,12.25),(.8,12.25),(-.8,12.25),(0.,13.05),(0.,11.45)]:
        mount_int,_,mount_amb,mw=source_line_intervals(st[:504]/1000,x/1000,z/1000)
        backing_int,_,backing_amb,bw=source_line_intervals(st[660:]/1000,x/1000,z/1000)
        runtime=axis_intervals(x,z,gs)
        cuts=[(max(a,-72.9999),min(b,-69.9999),i)for a,b,i in runtime if max(a,-72.9999)<min(b,-69.9999)]
        source_entry=float(min(a for a,b in backing_int)*1000)if backing_int else None
        runtime_entry=min((a for a,b,i in cuts),default=None)
        central.append({'axis_xz_mm':[x,z],'source_mount_clear':not mount_int and not mount_amb and mw==0,'source_backing_intervals_y_mm':[[float(a*1000),float(b*1000)]for a,b in backing_int],
                        'runtime_path_solid_intervals_y_mm':[[a,b,cells[i]['id']]for a,b,i in cuts],'entry_error_mm':None if runtime_entry is None or source_entry is None else runtime_entry-source_entry,
                        'status':'PASS'if not mount_int and not mount_amb and not backing_amb and runtime_entry is not None and source_entry is not None and abs(runtime_entry-source_entry)<=.05 else 'UNKNOWN'})
    channel=[]
    for ids in [[1514,1515],[2082,2083]]:
        wall=st[ids];x=float(wall[:,:,0].mean());side=1 if x>0 else-1
        mount_ids=[19,20,21]if side>0 else[9,10,11];mx=float(st[mount_ids,:,0].mean());width=abs(x-mx)
        lower=width-2*(delta+NUM)
        channel.append({'backing_wall_source_ids':ids,'backing_wall_x_mm':x,'mount_wall_x_mm':mx,'source_width_mm':width,'continuous_runtime_width_lower_mm':lower,'required_source_minus_point1_mm':width-.1,'status':'PASS'if material['status']=='PASS'and lower>=width-.1 and lower>0 else'UNKNOWN',
                        'proof':'Every source void core eroded50um is continuously excluded by the global material cover. Both frozen source channels have positive connected eroded cores; no claim from entity size.'})
    names={q['name']:{}for q in p['protected_void_policy']['features']}
    for name in names:
        names[name]={'status':'UNKNOWN','reason':'Full opening/contour/loop association still requires the boundary certificate and B contour check.'}
    names['side_channels']={'status':'PASS'if all(q['status']=='PASS'for q in channel)else'UNKNOWN','continuous_width_checks':channel}
    names['central_through_bore']={'status':'PASS'if all(q['status']=='PASS'for q in central)else'UNKNOWN','rule':next(q['rule']for q in p['protected_void_policy']['features']if q['name']=='central_through_bore'),'paths':central}
    names['four_mount_recesses'].update(axis_floor_status='PASS'if all(f['status']=='PASS'for f in floors)else'UNKNOWN',axis_floors=floors)
    return {'material':material,'named_protected_voids':names,'all_continuous_backing_segments':segments,'all_mount_points':points,'all_segment_checks_pass':all(q['runtime']['clear']for q in segments),
            'all_point_checks_pass':all(q['expected_occupancy_matches']for q in points),'continuous_eroded_source_void_core_preserved':material['status']=='PASS','full_connectivity_status':'UNKNOWN_PENDING_BOUNDARY_CONTOUR_ASSOCIATION'}


def recheck_suspects_history(cells,rt):
    from adjudicate_v2_partition import Union,legacy_missing_locators
    old=json.loads((ROOT/'reports/nero-gripper-v2-adjudication.json').read_text());cert=json.loads((ROOT/'reports/nero-gripper-v2-runtime-verification.json').read_text())['union_exterior_certificate']
    z=np.load(ROOT/'reports/nero-gripper-v2-union-exterior.npz');oldtri=z['vertices_mm'][z['faces']]/1000
    lookup={e['id']:e for e in cells};bodies,_=support_cells(cells)
    oracle=Union([{'id':b['id'],'halfspaces_m':(b['equations']*[1,1,1,.001]).tolist()}for b in bodies],NUM/64)
    trees=[TriangleBVH(np.array(e['vertices_m'])[e['faces']]*1000)for e in cells];atlas_tree=TriangleBVH(rt)
    locators=legacy_missing_locators(cells,oldtri,cert['failures']+cert['occluded_output_triangles'])
    records=[]
    for group,fs in [('certificate',old['certificate_failures']),('occlusion',old['occlusion_records'])]:
        for i,f in enumerate(fs):
            if 'entity'in f:
                e=lookup[f['entity']];t=np.array(e['vertices_m'])[e['faces'][f['face']]]*1000;loc=locators[(f['entity'],f['face'])];polys=loc['patch_vertices']
            else:
                t=oldtri[f['triangle']]*1000
                loc=locators.get((f['triangle'],f.get('covering_entity')));polys=loc['patch_vertices']if loc else[t/1000]
            support=[]
            for poly in polys:
                xyz=np.array(poly);inside=oracle.strict_cover(xyz)
                support.append({'strict_interior':inside,'equivalent_closure':None if inside else closure_pair_certificate(xyz*1000,bodies,trees)})
            point_records=[]
            points=np.array([q['point_mm']for q in f['probes']]);distances,_=atlas_tree.query(points)if len(points)else(np.empty(0),None)
            n=np.array(f['normal'])
            for q,d in zip(points,distances):
                shifted=[q+s*eps*n for eps in [1e-6,1e-5,1e-4]for s in[-1,1]];states=oracle.membership(np.array(shifted)/1000);pairs=[(a['state'],b['state'])for a,b in zip(states[::2],states[1::2])]
                if len(set(pairs))==1 and pairs[0]in[('SOLID','VOID'),('VOID','SOLID')]and d<=NUM:label='EFFECTIVE_BOUNDARY_AT_FIXED_PROBES'
                elif all(a==b=='SOLID'for a,b in pairs):label='INTERNAL_AT_FIXED_PROBES'
                else:label='UNRESOLVED'
                point_records.append({'point_mm':q.tolist(),'effective_atlas_distance_mm':float(d),'classification':label,'normal_side_memberships':pairs})
            if support and all(q['strict_interior']for q in support):status='CONTINUOUS_INTERNAL_PATCH'
            elif support and all(q['strict_interior']or q['equivalent_closure']for q in support):status='REPRESENTATION_EQUIVALENT_WITHIN_SUPPORT'
            elif point_records and len({q['classification']for q in point_records})==1:status=point_records[0]['classification']
            else:status='UNRESOLVED'
            records.append({'group':group,'old_record_index':i,'old_check':f.get('check','occlusion'),'triangle':f.get('triangle'),'entity':f.get('entity'),'face':f.get('face'),'old_classification':f['classification'],'new_status':status,'patch_proofs':support,'fixed_probe_records':point_records})
    t=oldtri[2940]*1000;proof=closure_pair_certificate(t,bodies,trees)
    special={'triangle':2940,'status':'REPRESENTATION_EQUIVALENT_WITHIN_SUPPORT'if proof else'UNRESOLVED','continuous_closure_proof':proof,'old_literal_gap_mm':old['exact_local_line_at_counterexample']['positive_exact_gaps'][0]['width_mm']}
    from classify_v2_fidelity import WEIGHTS
    reference=json.loads((ROOT/'reports/nero-gripper-entity-partition.json').read_text());tri=[]
    for e in reference['entities']:
        for sp in e['source_surface_partition']:
            for poly in sp['exterior_subpolygons']:
                q=np.array(poly['polygon_m'])*1000
                tri.extend(q[[0,j,j+1]]for j in range(1,len(q)-1))
    points=np.einsum('wv,tvc->twc',WEIGHTS,np.array(tri)).reshape(-1,3)
    historical=json.loads((ROOT/'reports/nero-gripper-v2-runtime-verification.json').read_text())['historical_168_A_samples']
    ids=[q['old_sample_index']for q in historical];dd,_=atlas_tree.query(points[ids]);history=[{'old_sample_index':i,'point_mm':p.tolist(),'distance_mm':float(d),'within_10um':bool(d<=.01)}for i,p,d in zip(ids,points[ids],dd)]
    old14=[r for r in records if r['group']=='certificate'and r['old_classification']=='NUMERICALLY_UNRESOLVED'];assert len(old14)==14
    result={'records':records,'counts_by_group':{g:dict(collections.Counter(r['new_status']for r in records if r['group']==g))for g in['certificate','occlusion']},'old14_rechecks':old14,'old14_remaining_unresolved':sum(r['new_status']=='UNRESOLVED'for r in old14),'triangle2940':special,'historical_168':history,'historical_168_maximum_mm':float(dd.max()),'historical_168_all_pass':bool(np.all(dd<=.01)),
            'scope':'Continuous patch certificates are distinguished from fixed-probe classifications; no global exterior certificate is inferred from these samples.'}
    save(ROOT/'reports/nero-gripper-v21-rechecks.json',result);return result


def validate_reported_geometry_witnesses(result,cells,rt):
    from adjudicate_v2_partition import Union
    bodies,_=support_cells(cells);oracle=Union([{'id':b['id'],'halfspaces_m':(b['equations']*[1,1,1,.001]).tolist()}for b in bodies],NUM/64)
    source=load_source();st=np.array(source['vertices_m'])[np.array([t['vertex_ids']for t in source['triangles']])]*1000;tree=TriangleBVH(st);runtime_tree=TriangleBVH(rt);checks=[]
    for tier,metrics in result['metrics'].items():
        metric=metrics['runtime_to_source_surface']
        if metric.get('status')!='FAIL' or metric.get('worst_sample_mm')is None:continue
        point=np.array(metric['worst_sample_mm']);dist,sourceid=tree.query(point[None]);_,ri=runtime_tree.query(point[None]);t=rt[ri[0]];n=normal(t[None])[0]
        # Select a stable support normal on the unchanged vertex hull, rather
        # than trusting a near-collinear triangulation's cross product.
        normals=[]
        for b in bodies:
            if np.any(point<b['lo']-NUM)or np.any(point>b['hi']+NUM):continue
            eq=b['equations'];active=np.flatnonzero(abs(eq[:,:3]@point+eq[:,3])<=NUM)
            normals.extend(eq[active,:3])
        if normals:n=max(normals,key=lambda q:q@n)
        states=oracle.membership(np.array([point+s*eps*n for eps in[1e-6,1e-5,1e-4]for s in[-1,1]])/1000);pairs=[(a['state'],b['state'])for a,b in zip(states[::2],states[1::2])]
        true_boundary=len(set(pairs))==1 and pairs[0]in[('SOLID','VOID'),('VOID','SOLID')]
        checks.append({'metric_tier':tier,'point_mm':point.tolist(),'distance_to_all_original_source_mm':float(dist[0]),'nearest_source_face':int(sourceid[0]),'normal_memberships':pairs,'effective_boundary_confirmed_at_fixed_scales':true_boundary,
                       'real_AB_budget_failure_confirmed':bool(true_boundary and dist[0]>.05+NUM),
                       'scope':'Sufficient actual-boundary counterexample when distance to ALL source surfaces exceeds the largest A/B budget. Negative result does not certify all untested regions.'})
    return checks


def direct_historical_certificates(cells,rt,history):
    """Direct boundary existence proofs; the atlas only supplies locators."""
    from adjudicate_v2_partition import Union
    bodies,_=support_cells(cells);oracle=Union([{'id':b['id'],'halfspaces_m':(b['equations']*[1,1,1,.001]).tolist()}for b in bodies],NUM/64)
    old=json.loads((ROOT/'reports/nero-gripper-v2-runtime-verification.json').read_text())['historical_168_A_samples']
    source=load_source();st=np.array(source['vertices_m'])[np.array([t['vertex_ids']for t in source['triangles']])]
    directions=np.array([q for q in product([-1,0,1],repeat=3)if any(q)],dtype=float);directions/=np.linalg.norm(directions,axis=1)[:,None]
    tree=TriangleBVH(rt);records=[]
    def closest(point,t):
        a,b,c=t;n=np.cross(b-a,c-a);n2=n@n;choices=[]
        if n2:
            projection=point-n*((point-a)@n/n2)
            if all(np.dot(np.cross(y-x,projection-x),n)>=-1e-13*n2 for x,y in[(a,b),(b,c),(c,a)]):choices.append(projection)
        for x,y in[(a,b),(b,c),(c,a)]:
            v=y-x;choices.append(x+np.clip((point-x)@v/(v@v),0,1)*v)
        return min(choices,key=lambda q:np.linalg.norm(q-point))
    for a,b in zip(old,history):
        point=np.array(b['point_mm'])/1000;t=st[a['source_face_id']];n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n)
        intervals,merged,gaps,local=oracle.line_intervals(point,n,.02/1000)
        ends=sorted({t0 for lo,hi,ids in merged for t0 in[lo,hi]if abs(float(t0))<.019999/1000},key=abs);proofs=[]
        for end in ends:
            q=point+float(end)*n;states=oracle.membership([q-2*NUM/1000*n,q+2*NUM/1000*n]);pairs=[z['state']for z in states]
            solid=any(z['state']=='SOLID'and z['minimum_cell_max_residual_mm']<-NUM for z in states)
            void=any(z['state']=='VOID'and z['minimum_cell_max_residual_mm']>NUM for z in states)
            if solid and void:proofs.append({'distance_upper_mm':abs(float(end))*1000+2*NUM,'point_mm':(q*1000).tolist(),'memberships_at_2_supports':pairs,'clearance_exceeds_numerical_support':True})
        best=min(proofs,key=lambda q:q['distance_upper_mm'])if proofs else None
        if best is None or best['distance_upper_mm']>.01:
            _,idx=tree.query(np.array(b['point_mm'])[None]);q=closest(np.array(b['point_mm']),rt[idx[0]])
            endpoints=np.r_[np.array(b['point_mm'])[None],q[None],q+2*NUM*directions];states=oracle.membership(endpoints/1000)
            solid=[i for i,z in enumerate(states)if z['state']=='SOLID'and z['minimum_cell_max_residual_mm']<-NUM]
            void=[i for i,z in enumerate(states)if z['state']=='VOID'and z['minimum_cell_max_residual_mm']>NUM]
            if solid and void:
                inside=endpoints[solid[0]];outside=endpoints[void[0]];upper=max(np.linalg.norm(inside-np.array(b['point_mm'])),np.linalg.norm(outside-np.array(b['point_mm'])))+NUM/16
                if best is None or upper<best['distance_upper_mm']:best={'distance_upper_mm':float(upper),'solid_endpoint_mm':inside.tolist(),'void_endpoint_mm':outside.tolist(),'solid_plane_margin_mm':-states[solid[0]]['minimum_cell_max_residual_mm'],'void_all_cells_plane_separation_mm':states[void[0]]['minimum_cell_max_residual_mm'],'proof':'Opposite robust memberships inside the same10um ball force a true boundary in the convex ball. Both endpoint clearances exceed frozen numerical support.'}
        records.append({'old_sample_index':a['old_sample_index'],'source_face_id':a['source_face_id'],'source_point_mm':b['point_mm'],'direct_effective_boundary_proof':best,'within10um_directly_certified':bool(best and best['distance_upper_mm']<=.01)})
    return {'status':'PASS'if all(q['within10um_directly_certified']for q in records)else'UNKNOWN','direct_pass_count':sum(q['within10um_directly_certified']for q in records),'total':len(records),'maximum_certified_upper_mm':max((q['direct_effective_boundary_proof']['distance_upper_mm']for q in records if q['within10um_directly_certified']),default=None),'records':records,'method':'Direct convex-union membership and certified boundary existence; no Boolean/atlas mesh truth.'}


def finalize_existing():
    path=ROOT/'reports/nero-gripper-v21-reacceptance.json';r=json.loads(path.read_text())
    assert not r['stages_pending'] and set(r['metrics'])=={'A','B'} and'minimum_entity_scale'in r
    data=json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text());cells=data['entities']
    geometry=hashlib.sha256(json.dumps([[e['id'],e['vertices_m'],e['faces']]for e in cells],separators=(',',':')).encode()).hexdigest();assert geometry==r['candidate_geometry_sha256']
    witness_path=ROOT/'reports/nero-gripper-v21-normal-witnesses.json';w=json.loads(witness_path.read_text());assert w['candidate_geometry_sha256']==geometry and w['script_sha256']==sha(ROOT/'scripts/verify_v21_normal_witnesses.py')
    confirmed=[q for q in w['confirmed_witnesses']if q.get('local_boundary_normal_cone_certificate',{}).get('failure_even_allowing_edge_normal_cones')]
    r['confirmed_Tier_A_normal_geometry_failures']={'status':'FAIL'if confirmed else'UNKNOWN','witness_count':len(confirmed),'global_maximum_deviation_deg':None,'global_maximum_certified_lower_bound_deg':max((q['local_boundary_normal_cone_certificate']['normal_deviation_lower_bound_deg']for q in confirmed),default=None),'evidence':'nero-gripper-v21-normal-witnesses.json'}
    r['blockers']=[b for b in r['blockers']if b['kind']!='REAL_TIER_A_NORMAL_GEOMETRY_FAILURE']
    if confirmed:r['blockers'].insert(0,{'kind':'REAL_TIER_A_NORMAL_GEOMETRY_FAILURE','count':len(confirmed),'source_faces':[q['source_stl_face']for q in confirmed],'scope':'Independent closed-facet and local normal-cone proofs, not dependent on uncertified exterior.'})
    for name,feature in r['material_and_voids']['named_protected_voids'].items():
        feature.setdefault('component_subcheck_status',feature['status'])
        feature['full_feature_acceptance_status']='UNKNOWN'
        feature['full_feature_acceptance_reason']='Core/path/axis subchecks do not replace the still-uncertified complete wall, mouth/exit contour and loop association.'
        feature['status']=feature['full_feature_acceptance_status']
    r['blockers']=[b for b in r['blockers']if b['kind']!='PROTECTED_VOID_VERIFICATION_UNRESOLVED']
    r['blockers'].extend({'kind':'PROTECTED_VOID_VERIFICATION_UNRESOLVED','item':name,'reason':feature['full_feature_acceptance_reason']}for name,feature in r['material_and_voids']['named_protected_voids'].items())
    r['material_and_voids']['all_named_protected_voids_fully_accepted']=False
    r['certified_real_protected_void_failure']=False
    rt=np.load(ROOT/'reports/nero-gripper-v21-effective-exterior.npz')['triangles_mm']
    history=direct_historical_certificates(cells,rt,r['historical_and_suspect_checks']['historical_168'])
    history['function_sha256']=hashlib.sha256(inspect.getsource(direct_historical_certificates).encode()).hexdigest();save(ROOT/'reports/nero-gripper-v21-historical-certificates.json',history)
    r['direct_historical_validation']={k:v for k,v in history.items()if k!='records'}
    oldthin=json.loads(subprocess.check_output(['git','show','8a5518b:reports/nero-gripper-v2-thin-classification.json'],cwd=ROOT))['entities'];owner={ref:e for e in cells for ref in e['reference_entity_ids']};retired=0
    for q in oldthin:
        if q['classification_tier']!='C':continue
        ids={owner[i]['id']for i in q['reference_entity_ids']};standalone=len(ids)==1 and set(owner[q['reference_entity_ids'][0]]['reference_entity_ids'])==set(q['reference_entity_ids']);retired+=not standalone
    r['C_identity_retirement']={'status':'PASS'if retired==74 else'FAIL','retired_count':retired,'source_domain_provenance_acceptance':'UNKNOWN'}
    r['primary_metric_verifier_sha256']=r['verifier_sha256'];r['normal_witness_verifier_sha256']=w['script_sha256'];r['finalizer_sha256']=sha(__file__)
    r['status']='V2_RUNTIME_ENTITY_PARTITION_BLOCKED'
    r['scope'].update(analyzed_frozen_candidate_count=1,new_candidate_count=0,formal_MJCF_changes=0,physics_integrations=0)
    r['final_blocker_classification']={'real_geometry_failure_confirmed':bool(confirmed),'real_protected_void_failure_confirmed':False,'certifier_still_unresolved':True,'reason':'Tier A normal correspondence fails independently. Exterior coverage, target responsibility and full protected-void/contour certification remain incomplete. Millimetre diagnostic distances are not certified physical defects.'}
    save(path,r);print(json.dumps({'status':r['status'],'real_A_normal_witnesses':len(confirmed),'global_normal_deviation_lower_bound_deg':r['confirmed_Tier_A_normal_geometry_failures']['global_maximum_certified_lower_bound_deg'],'historical_direct_pass':history['direct_pass_count'],'historical_total':history['total']},indent=2),flush=True)


def main():
    started=time.time();p,revision=policy();freeze=json.loads((TASK/'freeze-before-reacceptance.json').read_text());assert sha(ROOT/'reports/nero-gripper-v21-fidelity-protocol.json')==freeze['rules_sha256']
    data=json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text());cells=data['entities'];assert sha(ROOT/'reports/nero-gripper-v2-runtime-partition.json')==freeze['candidate_file_sha256']
    geo=hashlib.sha256(json.dumps([[e['id'],e['vertices_m'],e['faces']]for e in cells],separators=(',',':')).encode()).hexdigest();assert geo==revision['candidate_geometry_sha256']
    rt=np.load(ROOT/'reports/nero-gripper-v21-effective-exterior.npz')['triangles_mm'];atlas=json.loads((ROOT/'reports/nero-gripper-v21-boundary-atlas.json').read_text());old=np.load(ROOT/'reports/nero-gripper-v2-union-exterior.npz')
    target_path=TASK/'tier-targets.npz';target_key=sha(ROOT/'reports/nero-gripper-v21-effective-exterior.npz')+hashlib.sha256(inspect.getsource(label_atlas).encode()).hexdigest()
    cache=TASK/'tier-key.txt'
    if target_path.exists() and cache.exists() and cache.read_text()==target_key:
        z=np.load(target_path);ref={k:z['reference_'+k]for k in ['A','B','C']};runtime={k:z['runtime_'+k]for k in ['A','B','C']};provenance=json.loads((TASK/'tier-provenance.json').read_text())
    else:
        ref,runtime,provenance=label_atlas(rt,old,p)
        np.savez_compressed(target_path,**{'reference_'+k:v for k,v in ref.items()},**{'runtime_'+k:v for k,v in runtime.items()});save(TASK/'tier-provenance.json',provenance);cache.write_text(target_key)
    result={'status':'V2_RUNTIME_ENTITY_PARTITION_BLOCKED','candidate_geometry_sha256':geo,'rules_commit':freeze['rules_commit'],'rules_sha256':freeze['rules_sha256'],'atlas_status':atlas['status'],'metrics':{},'C_surface_provenance':provenance,'stages_pending':['material','voids','suspects','history'],'scope':{'vertices_faces_changed':False,'candidate_count':1,'mujoco_imported':any(k.startswith('mujoco')for k in sys.modules)}}
    material_cache=TASK/'material-voids.json';mh=hashlib.sha256(inspect.getsource(material_voids).encode()).hexdigest()
    if material_cache.exists()and json.loads(material_cache.read_text()).get('function_sha256')==mh:mv=json.loads(material_cache.read_text())
    else:mv=material_voids(cells,p);mv['function_sha256']=mh;save(material_cache,mv)
    result['material_and_voids']=mv
    recheck_key=sha(ROOT/'reports/nero-gripper-v21-effective-exterior.npz')+hashlib.sha256(inspect.getsource(recheck_suspects_history).encode()).hexdigest()
    recheck_path=TASK/'recheck-key.txt';rechecks=ROOT/'reports/nero-gripper-v21-rechecks.json'
    if rechecks.exists()and recheck_path.exists()and recheck_path.read_text()==recheck_key:result['historical_and_suspect_checks']=json.loads(rechecks.read_text())
    else:result['historical_and_suspect_checks']=recheck_suspects_history(cells,rt);recheck_path.write_text(recheck_key)
    result['stages_pending']=[]
    save(ROOT/'reports/nero-gripper-v21-reacceptance.json',result)
    for tier in ['A','B']:
        limit=p['tiers'][tier]['outward_surface_mm'];gap=p['tiers'][tier]['symmetric_surface_certificate_gap_mm']
        metrics={'source_to_runtime_surface':surface(ref[tier],runtime[tier],limit,gap,tier+'-reverse'),'runtime_to_source_surface':surface(runtime[tier],ref[tier],limit,gap,tier+'-forward')}
        se,su=contour_edges(ref[tier]);te,tu=contour_edges(runtime[tier]);print('Contour',tier,len(se),len(te),flush=True)
        metrics['source_to_runtime_contour']=contour_cover(se,te,limit,gap);metrics['runtime_to_source_contour']=contour_cover(te,se,limit,gap)
        metrics['contour_line_representation_unresolved']=su+tu
        if tier=='A':
            metrics['source_to_runtime_normal']=normal_cover(ref[tier],runtime[tier]);metrics['runtime_to_source_normal']=normal_cover(runtime[tier],ref[tier])
        result['metrics'][tier]=metrics;save(ROOT/'reports/nero-gripper-v21-reacceptance.json',result)
    scale=[]
    for e in cells:
        h=ConvexHull(np.array(e['vertices_m'])*1000);diameter,center=insphere(h.equations);scale.append({'entity':e['id'],'diameter_mm':diameter,'status':'PASS'if diameter>=.1 else'FAIL'})
    result['minimum_entity_scale']={'status':'PASS'if all(x['status']=='PASS'for x in scale)else'FAIL','minimum_diameter_mm':min(x['diameter_mm']for x in scale),'entities':scale}
    result['validated_geometry_counterexample_checks']=validate_reported_geometry_witnesses(result,cells,rt)
    blockers=[]
    for witness in result['validated_geometry_counterexample_checks']:
        if witness['real_AB_budget_failure_confirmed']:blockers.append({'kind':'REAL_AB_SURFACE_BUDGET_FAILURE','witness':witness})
    if atlas['status']!='PASS_MODULO_FROZEN_SUPPORT':blockers.append({'kind':'CERTIFIER_UNRESOLVED','item':'union exterior','details':atlas['unresolved']})
    if provenance['C_provenance_unresolved_or_new_AB_caps']:blockers.append({'kind':'CERTIFIER_UNRESOLVED','item':'C-domain/new-cap responsibility','count':len(provenance['C_provenance_unresolved_or_new_AB_caps'])})
    for tier,metrics in result['metrics'].items():
        for name,value in metrics.items():
            if isinstance(value,dict)and value.get('status')!='PASS':blockers.append({'kind':'CERTIFIER_OR_TARGET_CORRESPONDENCE_UNRESOLVED','item':tier+'/'+name,'conditional_metric_status':value.get('status')})
        if metrics['contour_line_representation_unresolved']:blockers.append({'kind':'CERTIFIER_UNRESOLVED','item':tier+'/contour line representation','count':len(metrics['contour_line_representation_unresolved'])})
    for name,value in mv['named_protected_voids'].items():
        if value['status']!='PASS':blockers.append({'kind':'PROTECTED_VOID_VERIFICATION_UNRESOLVED','item':name,'reason':value['reason']})
    rr=result['historical_and_suspect_checks']
    if rr['old14_remaining_unresolved']:blockers.append({'kind':'CERTIFIER_UNRESOLVED','item':'old14','count':rr['old14_remaining_unresolved']})
    if rr['triangle2940']['status']=='UNRESOLVED':blockers.append({'kind':'CERTIFIER_UNRESOLVED','item':'triangle2940'})
    if not rr['historical_168_all_pass']:blockers.append({'kind':'CERTIFIER_UNRESOLVED','item':'old168 on uncertified atlas'})
    result['blockers']=blockers;result['status']='V2_RUNTIME_ENTITY_PARTITION_FROZEN'if not blockers and result['minimum_entity_scale']['status']=='PASS'else'V2_RUNTIME_ENTITY_PARTITION_BLOCKED'
    result['interpretation']='Metrics are conditional on the derived exterior/target correspondence. An uncertified atlas cannot establish a real geometry failure solely from its distances. No budget was relaxed.'
    result['elapsed_seconds']=time.time()-started;result['verifier_sha256']=sha(__file__);save(ROOT/'reports/nero-gripper-v21-reacceptance.json',result)
    print(json.dumps({'metrics':{k:{n:d.get('status')for n,d in v.items()if isinstance(d,dict)}for k,v in result['metrics'].items()},'minimum_diameter_mm':result['minimum_entity_scale']['minimum_diameter_mm'],'seconds':result['elapsed_seconds']},indent=2),flush=True)


if __name__=='__main__':
    if'--finalize-existing'in sys.argv:finalize_existing()
    else:main()
