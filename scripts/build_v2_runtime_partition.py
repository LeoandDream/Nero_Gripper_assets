"""Issue #25: one offline v2 partition under protocol commit 8a5518b.

Constructs convex entities only. A separate verifier owns all acceptance.
No MuJoCo, MJCF, controller or physics code is imported or executed.
"""
from pathlib import Path
import collections
import hashlib
import itertools
import json
import pickle
import subprocess
import sys
import time

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
TASK=ROOT/'debug/issue25/v2-runtime'
sys.path.insert(0,str(TASK/'vendor'))
import manifold3d as mf
from runtime_backing_partition import poly,outward_faces,inradius
from runtime_contact_partition import load_source

POLICY_COMMIT='8a5518b37f15678893ca6f540f5ef2edea199ab8'
ARITH=1e-7  # mm; numerical support reserve, never a fidelity-policy change


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frozen(name):
    return json.loads(subprocess.check_output(['git','show',POLICY_COMMIT+':'+name],cwd=ROOT))


def mesh_solid(v,faces):
    f=np.asarray(faces,dtype=np.uint64);used=np.unique(f);remap=np.zeros(len(v),dtype=np.uint64);remap[used]=np.arange(len(used))
    return mf.Manifold(mf.Mesh64(np.ascontiguousarray(v[used],dtype=np.float64),np.ascontiguousarray(remap[f],dtype=np.uint64)))


def solid(p):
    # The kernel triangulates the SAME convex vertex set robustly. Avoid
    # inheriting a nearly-collinear Qhull triangle-normal artefact; no joggle,
    # vertex inflation or source coordinate edit is used.
    m=mf.Manifold.hull_points(np.ascontiguousarray(p['v'],dtype=np.float64))
    if m.status()!=mf.Error.NoError:raise ValueError('Convex input Boolean status '+str(m.status()))
    return m


def union(items):
    u=mf.Manifold.batch_boolean(items,mf.OpType.Add)
    if u.status()!=mf.Error.NoError:raise ValueError('Boolean union failed '+str(u.status()))
    u.volume()  # evaluate lazily cached Boolean result once
    return u


def insphere(eq):
    result=linprog([0,0,0,-1],A_ub=np.c_[eq[:,:3],np.ones(len(eq))],b_ub=-eq[:,3],bounds=[(None,None)]*3+[(0,None)],method='highs')
    if not result.success:return 0.,np.zeros(3)
    return float(2*result.x[3]),result.x[:3]


def make_group(p,ids,refs):
    d,c=insphere(p['eq'])
    family=collections.Counter()
    for i in ids:family[refs[i]['semantic_family']]+=float(refs[i]['volume_m3'])
    return {'p':p,'ids':set(ids),'diameter':d,'centre':c,'family':family.most_common(1)[0][0],'version':0}


def main():
    target=ROOT/'reports/nero-gripper-v2-runtime-partition.json'
    if target.exists():raise SystemExit('Single candidate already exists; verify it, do not regenerate.')
    start=time.time();policy=frozen('reports/nero-gripper-v2-fidelity-protocol.json')
    ref=json.loads((ROOT/'reports/nero-gripper-entity-partition.json').read_text());refs=ref['entities']
    v1=json.loads((ROOT/'reports/nero-gripper-runtime-partition.json').read_text())
    for name,expected in policy['input_sha256'].items():
        if name.endswith('scripts/analyze_runtime_partition.py') or (ROOT/name).exists():assert sha(ROOT/name)==expected,name
    source=load_source();source_v=np.array(source['vertices_m'])*1000
    source_tri=source_v[np.array([t['vertex_ids'] for t in source['triangles']])]
    normals=np.cross(source_tri[:,1]-source_tri[:,0],source_tri[:,2]-source_tri[:,0]);normals/=np.linalg.norm(normals,axis=1)[:,None]
    source_eq=np.c_[normals,-np.einsum('ij,ij->i',normals,source_tri[:,0])]
    a_faces=set()
    for item in policy['source_region_policy'].values():
        if item['tier']=='A':a_faces.update(i for a,b in item['source_triangle_ranges_zero_based_inclusive'] for i in range(a,b+1))
    owned_a=[];a_points=[]
    for e in refs:
        owned=set()
        for sp in e['source_surface_partition']:
            if sp['source_face_id'] not in a_faces:continue
            for patch in sp['exterior_subpolygons']:
                if patch['area_m2']*1e6<=1e-12:continue
                owned.add(sp['source_face_id']);q=np.array(patch['polygon_m'])*1000;a_points.extend(q);a_points.append(q.mean(0))
        owned_a.append(owned)
    a_points=np.unique(np.array(a_points),axis=0)
    originals=[poly(np.array(e['vertices_m'])*1000) for e in refs]
    assert all(p is not None for p in originals)
    byid={e['id']:i for i,e in enumerate(refs)};byname={e['name']:i for i,e in enumerate(refs)}
    print('Preparing fixed original-material offset oracle',flush=True)
    delta=policy['tiers']['B']['outward_surface_mm']-1e-5
    corners=np.array(list(itertools.product((-delta/np.sqrt(3),delta/np.sqrt(3)),repeat=3)))
    expanded=[]
    for i,p in enumerate(originals):
        expanded.append(solid(poly((p['v'][:,None]+corners[None]).reshape(-1,3))))
        if (i+1)%250==0:print('offset cells',i+1,flush=True)
    oracle=union(expanded)
    om=oracle.to_mesh64()
    np.savez_compressed(TASK/'fixed-expanded-oracle.npz',vertices_mm=om.vert_properties[:,:3],faces=om.tri_verts)
    print('Offset oracle ready',oracle.num_tri(),'triangles',round(time.time()-start,1),'s',flush=True)
    rejection=collections.Counter();accepted=[];cover_count=0

    def check(p,ids):
        nonlocal cover_count
        owned=set().union(*(owned_a[i] for i in ids))
        if owned:
            eq=source_eq[sorted(owned)]
            if np.max(p['v']@eq[:,:3].T+eq[:,3])>ARITH:
                rejection['inherited_A_plane']+=1;return False
        # Monotone rejection of a demonstrably deeply buried A target point.
        mask=np.all(a_points>=p['lo']-ARITH,axis=1)&np.all(a_points<=p['hi']+ARITH,axis=1)
        q=a_points[mask]
        if len(q) and np.any(np.min(-(q@p['eq'][:,:3].T+p['eq'][:,3]),axis=1)>.01):
            rejection['buried_A_witness']+=1;return False
        difference=solid(p)-oracle;cover_count+=1
        if difference.status()!=mf.Error.NoError:
            rejection['boolean_cover_UNKNOWN']+=1;return False
        if not difference.is_empty():rejection['outside_fixed_material_offset']+=1;return False
        return True

    groups={};seed_log=[]
    for e in v1['entities']:
        ids={byid[x] for x in e['reference_entity_ids']};p=poly(np.array(e['vertices_m'])*1000)
        if len(ids)>1 and check(p,ids):
            key=min(ids);groups[key]=make_group(p,ids,refs);seed_log.append({'v1_id':e['id'],'used_group':True})
        else:
            for i in sorted(ids):groups[i]=make_group(originals[i],{i},refs)
            seed_log.append({'v1_id':e['id'],'used_group':False})
    print('Source-safe seed count',len(groups),'thin',sum(g['diameter']<.1-1e-8 for g in groups.values()),flush=True)
    # Reference graph includes physical interfaces, not just within-part cuts.
    ref_adj=[]
    for e in refs:
        ref_adj.append({byname[n] for n in e['neighbors']+e.get('cross_part_neighbors',[]) if n in byname})
    for i,ns in enumerate(ref_adj):
        for j in list(ns):ref_adj[j].add(i)
    thin=frozen('reports/nero-gripper-v2-thin-classification.json')['entities']
    c_refs={byid[r] for e in thin if e['classification_tier']=='C' for r in e['reference_entity_ids']}

    def merge_pass(round_id):
        owner={i:k for k,g in groups.items() for i in g['ids']};total=0
        ordered=sorted(groups,key=lambda k:(not bool(groups[k]['ids']&c_refs),groups[k]['diameter']>=.1-1e-8,groups[k]['p']['h'].volume,k))
        for i in ordered:
            if i not in groups:continue
            a=groups[i];near=set().union(*(ref_adj[x] for x in a['ids']))
            candidates={owner[x] for x in near if owner[x] in groups and owner[x]!=i}
            # Macro semantic family is preferred; source-piece labels are not
            # mechanical boundaries. Workface support checks remain mandatory.
            candidates=sorted(candidates,key=lambda j:(a['family']!=groups[j]['family'],-groups[j]['p']['h'].volume,j))
            for j in candidates:
                b=groups[j];key=(tuple(sorted(a['ids'])),tuple(sorted(b['ids'])))
                if key in tried:continue
                tried.add(key);p=poly(np.r_[a['p']['v'],b['p']['v']]);ids=a['ids']|b['ids']
                if p is None or not check(p,ids):continue
                g=make_group(p,ids,refs);new=min(i,j)
                del groups[i];del groups[j];groups[new]=g
                for x in ids:owner[x]=new
                accepted.append({'round':round_id,'reference_ids':[refs[x]['id'] for x in sorted(ids)],'action':'neighbour_hull_coalescence','diameter_mm':g['diameter']})
                total+=1;break
        return total

    tried=set()
    for rnd in range(12):
        n=merge_pass(rnd)
        print('coalescence',rnd,'merged',n,'remaining',len(groups),'thin',sum(g['diameter']<.1-1e-8 for g in groups.values()),'seconds',round(time.time()-start,1),flush=True)
        with (TASK/'construction-checkpoint.pkl').open('wb') as f:pickle.dump({'groups':groups,'accepted':accepted,'rejection':rejection},f)
        if not n:break
    # A partial interior anchor changes an internal cut without consuming the
    # entire neighbouring cell or moving the owned external support plane.
    centres=np.array([inradius(p)[1] for p in originals]);tree=cKDTree(centres)
    anchor_log=[];half=.1/2;required=half*np.sqrt(3)+ARITH
    cube=np.array(list(itertools.product((-half,half),repeat=3)))
    for key in sorted(groups,key=lambda k:(groups[k]['diameter'],k)):
        g=groups[key]
        if g['diameter']>=.1-1e-8:continue
        owned=set().union(*(owned_a[i] for i in g['ids']));aeq=source_eq[sorted(owned)] if owned else np.empty((0,4))
        near=np.atleast_1d(tree.query(g['p']['v'].mean(0),k=min(96,len(originals)))[1])
        done=False
        for j in near:
            eq=np.r_[originals[int(j)]['eq'],aeq];diameter,centre=insphere(eq)
            if diameter/2<required:continue
            p=poly(np.r_[g['p']['v'],centre+cube]);ids=g['ids']
            if p is None or not check(p,ids):continue
            new=make_group(p,ids,refs)
            if new['diameter']<.1-1e-8:raise ValueError('Inscribed anchor construction inconsistent')
            groups[key]=new;anchor_log.append({'reference_ids':[refs[x]['id'] for x in sorted(ids)],'anchor_source_reference':refs[int(j)]['id'],'cube_centre_mm':centre.tolist(),'cube_half_width_mm':half,'anchor_contained_in_original_reference_and_A_halfspaces':True,'new_diameter_mm':new['diameter']});done=True;break
        if not done:anchor_log.append({'reference_ids':[refs[x]['id'] for x in sorted(g['ids'])],'status':'NO_CERTIFIED_INTERIOR_ANCHOR','diameter_mm':g['diameter']})
    # One more deterministic coalescence of the newly thickened partitions.
    tried=set()
    for rnd in range(4):
        n=merge_pass('after_internal_repartition_'+str(rnd))
        print('post-anchor coalescence',rnd,n,len(groups),flush=True)
        if not n:break
    entities=[]
    for k,g in sorted(groups.items(),key=lambda kv:(kv[1]['family'],tuple(kv[1]['p']['v'].mean(0)))):
        p=g['p'];ids=sorted(g['ids']);original=[refs[i] for i in ids]
        ident='V2%04d'%(len(entities)+1)
        features=sorted({e.get('semantic_feature',e.get('feature','')) for e in original})
        sm=solid(p).to_mesh64();vertices=np.array(sm.vert_properties[:,:3]);faces=np.array(sm.tri_verts,dtype=int)
        entities.append({'id':ident,'name':g['family']+'_'+features[0]+'_'+ident,'semantic_family':g['family'],'semantic_features':features,'source_parts':sorted({e['source_part'] for e in original}),'primitive_type':'convex_polyhedron','vertices_m':(vertices/1000).tolist(),'faces':faces.tolist(),'halfspaces_m':(p['eq']*[1,1,1,.001]).tolist(),'reference_entity_ids':[e['id'] for e in original],'source_regions':sorted({r for e in original for r in e['source_regions']}),'source_triangle_ids':sorted({t for e in original for t in e['source_triangle_ids']}),'inscribed_diameter_mm':g['diameter'],'inscribed_centre_m':(g['centre']/1000).tolist(),'volume_mm3':float(p['h'].volume),'minimum_facet_normal_width_mm':float(np.ptp(p['v']@p['eq'][:,:3].T,axis=0).min()),'A_source_support_face_ids':sorted(set().union(*(owned_a[i] for i in ids))),'material_cover_certificate':'Hull minus fixed original-reference offset union is empty; original reference cells included.','overlap_policy':'Interior overlap may arise from inward repartition. Actual exterior ownership and any coplanar exterior overlap are independently checked; a metadata owner never hides duplicate physical exterior.'})
    result={'schema_version':2,'status':'V2_RUNTIME_ENTITY_PARTITION_BLOCKED','stage':'SINGLE_CANDIDATE_GENERATED_PENDING_VERIFICATION','policy_commit':POLICY_COMMIT,'policy_blob_sha256':sha(TASK/'nero-gripper-v2-fidelity-protocol.json'),'baseline_delivery_commit':'12e0762d86c1a2f42f14090b19ef70e63ce556e3','units':'m','calculation_units':'mm','entity_count_per_finger':len(entities),'total_finger_entities':2*len(entities),'counts_by_semantic_family':dict(collections.Counter(e['semantic_family'] for e in entities)),'minimum_inscribed_diameter_mm':min(e['inscribed_diameter_mm'] for e in entities),'reference_sha256':sha(ROOT/'reports/nero-gripper-entity-partition.json'),'v1_sha256':sha(ROOT/'reports/nero-gripper-runtime-partition.json'),'entities':entities,'construction':{'method':'Source-safe v1 seeds, semantic neighbour coalescence and interior-closure anchor repartition; one candidate only.','seed_log':seed_log,'accepted_coalescences':accepted,'interior_anchor_log':anchor_log,'rejection_counts':dict(rejection),'continuous_cover_boolean_evaluations':cover_count,'original_material_outward_upper_mm':delta,'material_oracle_cube_half_width_mm':delta/np.sqrt(3),'boolean_kernel_tolerance_mm':oracle.get_tolerance(),'arithmetic_support_resolution_mm':ARITH,'seconds':time.time()-start},'generator_sha256':sha(Path(__file__)),'scope':{'new_runtime_candidate_count':1,'new_formal_mjcf':0,'active_model_changes':0,'mujoco_imports':0,'mj_step':0,'grasp':0,'policy_changes':0},'verification':'Pending independent actual union exterior, A/B/C, void, normal, minimum-scale and historical-point certification; BLOCKED until complete.'}
    result['geometry_sha256']=hashlib.sha256(json.dumps([[e['id'],e['vertices_m'],e['faces']] for e in entities],separators=(',',':')).encode()).hexdigest()
    target.write_text(json.dumps(result,separators=(',',':'),allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['entities','construction']},indent=2),flush=True)


if __name__=='__main__':main()
