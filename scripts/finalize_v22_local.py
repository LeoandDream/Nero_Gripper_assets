"""Targeted independent regressions and provenance for the frozen v22 file.

Never generates/changes geometry. Earlier10 regions retain their historical
coordinates, including old cell face6; new array indices are not substituted.
"""
from pathlib import Path
import collections,hashlib,json,subprocess,sys,time
import numpy as np
from scipy.spatial import ConvexHull
from verify_v22_local import (ROOT,TASK,BASE,OUT,REPORT,NUM,GRID,sha,save,
    geosha,support_cells,oracle_for,boundary_point,normal_counterexample,
    TriangleBVH,source_arrays,source_external)
from v21_boundary_atlas import closure_pair_certificate
from adjudicate_v2_partition import legacy_missing_locators

def main():
    started=time.time();r=json.loads(REPORT.read_text());candidate_sha=sha(OUT);data=json.loads(OUT.read_text());cells=data['entities'];old=json.loads(BASE.read_text())['entities'];bodies,_=support_cells(cells);oldb,_=support_cells(old);u=oracle_for(cells,bodies);ou=oracle_for(old,oldb)
    st=source_arrays();tree=TriangleBVH(np.concatenate([np.array(e['vertices_m'])[e['faces']]*1000 for e in cells]));oldtree=TriangleBVH(np.concatenate([np.array(e['vertices_m'])[e['faces']]*1000 for e in old]))
    mandatory=[]
    witnesses=json.loads((ROOT/'reports/nero-gripper-v21-normal-witnesses.json').read_text())['confirmed_witnesses']
    for row,w in zip(r['mandatory_11']['records'],witnesses):
        q=dict(row)
        if q['status']!='PASS':
            proof=normal_counterexample(np.array(w['source_point_mm']),np.array(w['source_normal']),cells,bodies,u)
            if proof and proof['status']=='FAIL':q['status']='FAIL';q['independent_normal_failure_certificate']=proof
        mandatory.append(q)
    r['mandatory_11'].update(records=mandatory,status='FAIL'if any(q['status']=='FAIL'for q in mandatory)else'PASS'if all(q['status']=='PASS'for q in mandatory)else'UNKNOWN')
    comparisons=[]
    for proof in r['all_A_sidewalls']['new_real_normal_counterexamples']:
        point=np.array(proof['source_point_mm']);n=np.array(proof['source_normal']);before=boundary_point(point,n,ou,oldb)
        oldproof=normal_counterexample(point,n,old,oldb,ou)if before['status']!='PASS'else None
        comparisons.append({'source_face':proof['source_face'],'patch':proof['patch'],'source_point_mm':point.tolist(),'before_ray':before,'before_failure_certificate':oldproof,'after_normal_lower_deg':proof['normal_deviation_lower_bound_deg'],'classification':'INTRODUCED_REAL_NORMAL_FAILURE'if before['status']=='PASS'else'PERSISTING_REAL_NORMAL_FAILURE'if oldproof and oldproof['status']=='FAIL'else'BASELINE_COMPARISON_UNKNOWN'})
    # Scan every fixed tier target triangle (7 deterministic barycentric
    # points); exact distances to the closed-facet SUPERSET lower-bound true
    # union-boundary distance. Only source-exterior + runtimeVOID witnesses
    # are called actual inward geometric failures.
    weights=np.array([[1/3]*3,[.6,.2,.2],[.2,.6,.2],[.2,.2,.6],[.8,.1,.1],[.1,.8,.1],[.1,.1,.8]])
    z=np.load(ROOT/'reports/nero-gripper-v2-union-exterior.npz');ss=TriangleBVH(st);scan={}
    for tier in ['A','B','C']:
        ts=z['reference_'+tier+'_triangles_mm'];points=np.einsum('wv,tvc->twc',weights,ts).reshape(-1,3);dd,nearest=tree.query(points);before_dd,_=oldtree.query(points)
        limit=.01 if tier=='A'else .05;ids=np.flatnonzero(dd-NUM>limit);confirmed=[]
        for j in ids[np.argsort(-dd[ids])]:
            point=points[j];sd,si=ss.query(point[None]);face=int(si[0]);t=st[face];n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n);exposure=source_external(point,n,st);state=u.membership([point/1000])[0]
            if sd[0]>NUM or not exposure['confirmed']or state['state']!='VOID':continue
            baseline=boundary_point(point,n,ou,oldb)
            confirmed.append({'source_face':face,'source_point_mm':point.tolist(),'source_normal':n.tolist(),'frozen_target_triangle':int(j//len(weights)),'target_sample':int(j%len(weights)),'source_exposure':exposure,'runtime_membership':state,'distance_to_all_closed_facets_mm':float(dd[j]),'conservative_true_union_boundary_distance_lower_mm':float(dd[j]-NUM),'before_closed_facet_distance_mm':float(before_dd[j]),'before_boundary':baseline,'introduced_vs_baseline_certified':baseline['status']=='PASS'and baseline['distance_upper_mm']<=limit,'nearest_closed_facet_index':int(nearest[j])})
            if len(confirmed)>=12:break
        scan[tier]={'evaluated_triangles':len(ts),'evaluated_points':len(points),'sample_distance_max_mm':float(dd.max()),'sample_above_budget_lower_bound_count':len(ids),'certified_source_exterior_runtime_void_failures':confirmed,'status':'FAIL'if confirmed else'UNKNOWN','scope':'A sufficient true inward failure certificate; absence of sampled failures is not full surface, material or contour PASS.'}
        print('Tier scan',tier,len(points),'confirmed actual inward failures',len(confirmed),flush=True)
    # Historical locators are created using BASELINE cells/meshes. Cell
    # V20333 face6 may have a different index in v22; NEVER index new face6.
    adj=json.loads((ROOT/'reports/nero-gripper-v2-adjudication.json').read_text());cert=json.loads((ROOT/'reports/nero-gripper-v2-runtime-verification.json').read_text())['union_exterior_certificate'];oldtris=z['vertices_mm'][z['faces']]/1000
    locators=legacy_missing_locators(old,oldtris,cert['failures']+cert['occluded_output_triangles']);prior=json.loads((ROOT/'reports/nero-gripper-v21-rechecks.json').read_text());trees=[TriangleBVH(np.array(e['vertices_m'])[e['faces']]*1000)for e in cells];rechecks=[]
    for prev in prior['old14_rechecks']:
        if prev['new_status']!='UNRESOLVED':continue
        f=adj['certificate_failures'][prev['old_record_index']]
        if f.get('entity'):
            loc=locators[(f['entity'],f['face'])];polys=loc['patch_vertices']
        else:
            loc=locators.get((f['triangle'],f.get('covering_entity')));polys=loc['patch_vertices']if loc else[oldtris[f['triangle']]]
        proofs=[]
        for pp in polys:
            pp=np.array(pp);inside=u.strict_cover(pp)
            proofs.append({'strict_interior':inside,'equivalent_closure':None if inside else closure_pair_certificate(pp*1000,bodies,trees),'frozen_patch_vertices_mm':(pp*1000).tolist()})
        points=[];n=np.array(f['normal'])
        for q in f['probes']:
            point=np.array(q['point_mm']);states=u.membership(np.array([point+s*eps*n for eps in [1e-6,1e-5,1e-4]for s in[-1,1]])/1000)
            points.append({'point_mm':point.tolist(),'normal_side_states':[[a['state'],b['state']]for a,b in zip(states[::2],states[1::2])]})
        status='CONTINUOUS_INTERNAL_PATCH'if proofs and all(p['strict_interior']for p in proofs)else'REPRESENTATION_EQUIVALENT_WITHIN_SUPPORT'if proofs and all(p['strict_interior']or p['equivalent_closure']for p in proofs)else'UNRESOLVED'
        rechecks.append({'triangle':f.get('triangle'),'entity':f.get('entity'),'old_face':f.get('face'),'status':status,'patch_proofs':proofs,'fixed_probe_records':points,'historical_geometry_not_reindexed':True})
    assert len(rechecks)==10
    r['historical_unresolved']={'previous_count':10,'rechecked_count':10,'remaining_unresolved':sum(q['status']=='UNRESOLVED'for q in rechecks),'records':rechecks,'status':'UNKNOWN'if any(q['status']=='UNRESOLVED'for q in rechecks)else'RESOLVED'}
    r['baseline_normal_comparisons']=comparisons;r['independent_all_tier_point_scan']=scan
    # A new cell is a subset of its baseline cell. No previously empty void
    # can be newly filled, but wall loss/widening is NOT excluded by this fact.
    r['protected_void_non_degradation']={'new_void_filling_excluded':r['all_new_cells_subsets_of_corresponding_old_cells'],'complete_wall_contour_connectivity_acceptance':'UNKNOWN','all_old_path_and_occupancy_subchecks_pass':r['material_voids']['all_segment_checks_pass']and r['material_voids']['all_point_checks_pass'],'warning':'Shrinking cells cannot newly block a void, but can lose walls or enlarge openings; full non-degradation not inferred.'}
    material_loss=any(q['conservative_true_union_boundary_distance_lower_mm']>.05 for tier in scan.values()for q in tier['certified_source_exterior_runtime_void_failures'])
    r['blocker_classification'].update(real_A_normal_failure=any(q['status']=='FAIL'for q in mandatory)or bool(r['all_A_sidewalls']['new_real_normal_counterexamples']),real_A_surface_failure=bool(scan['A']['certified_source_exterior_runtime_void_failures']),real_B_surface_failure=bool(scan['B']['certified_source_exterior_runtime_void_failures']),real_material_loss_failure=material_loss,protected_void_degradation='UNKNOWN_NOT_EXCLUDED',B_C_non_degradation='FAIL'if scan['B']['status']=='FAIL'or scan['C']['status']=='FAIL'else'UNKNOWN')
    r['global_material_set_status']='FAIL'if material_loss else'UNKNOWN'
    r['status']='V22_LOCAL_REPARTITION_BLOCKED';r['finalizer_sha256']=sha(__file__);r['finalizer_seconds']=time.time()-started
    assert sha(OUT)==candidate_sha==r['candidate_file_sha256'] and geosha(cells)==r['candidate_geometry_sha256'];save(REPORT,r)
    print(json.dumps({'status':r['status'],'mandatory_status':r['mandatory_11']['status'],'mandatory_pass':r['mandatory_11']['pass_count'],'old10_still_unresolved':r['historical_unresolved']['remaining_unresolved'],'blockers':r['blocker_classification']},indent=2))
if __name__=='__main__':main()
