"""Read-only v23 regressions from saved vertices/faces, never a new candidate."""
from pathlib import Path
import collections
import gzip
import hashlib
import json
import sys
import time

import numpy as np
from scipy.optimize import linprog

from v23_finite_patch import ROOT,TASK,BASE,OUT,PRE,TRACE,NUM,body,geosha,sha,save
from verify_v22_local import oracle_for,boundary_point,all_patches,source_arrays,source_external
from analyze_runtime_partition import TriangleBVH
from verify_v21_partition import policy,material_voids

REPORT=ROOT/'reports/nero-gripper-v23-verification.json'


def bodies_from_saved(cells):
    bodies,checks=[],[]
    for e in cells:
        v=np.array(e['vertices_m'])*1000;p=body(v)
        if p is None:
            checks.append({'entity':e['id'],'status':'UNKNOWN_SAVED_CELL_HULL_NUMERICALLY_UNRESOLVED','excluded_from_regularized_solid_queries_only':True,'candidate_not_modified':True,'not_a_proof_of_exact_zero_volume':True})
            continue
        eq=p['eq'];tri=v[np.array(e['faces'])]
        heights=np.max(abs(tri@eq[:,:3].T+eq[:,3]),axis=1).min(1)
        error=float(np.max(v@eq[:,:3].T+eq[:,3]))
        checks.append({'entity':e['id'],'frozen_face_to_support_plane_upper_mm':float(heights.max()),'vertex_support_residual_mm':error,'status':'PASS'if heights.max()<=NUM/32 and error<=NUM/32 else'UNKNOWN'})
        bodies.append({'id':e['id'],'vertices':v,'equations':eq,'lo':v.min(0),'hi':v.max(0)})
    return bodies,checks


def saved_facet_check(record,cells_by_id):
    """An analytical support normal alone is not a saved-facet normal."""
    if record.get('status')!='PASS':return record
    point=np.array(record['point_mm']);matches=[]
    for owner in record['boundary_facets']:
        e=cells_by_id[owner['entity']];tri=np.array(e['vertices_m'])[e['faces']]*1000
        nsource=np.array(record['source_normal']);tree=TriangleBVH(tri);dd,ix=tree.query(point[None]);j=int(ix[0]);t=tri[j]
        n=np.cross((t[1]-t[0]).astype(np.longdouble),(t[2]-t[0]).astype(np.longdouble));length=np.sqrt(np.sum(n*n))
        if length==0:continue
        n=np.array(n/length,dtype=float)
        center=np.array(e['vertices_m']).mean(0)*1000
        if n@(t.mean(0)-center)<0:n=-n
        angle=float(np.degrees(np.arccos(np.clip(n@nsource,-1,1))))
        matches.append({'entity':e['id'],'face':j,'saved_triangle_distance_mm':float(dd[0]),'saved_triangle_normal_deviation_deg':angle,'status':'PASS'if dd[0]<=NUM and angle+1e-5<=.05 else'UNKNOWN'})
    record['saved_facet_confirmation']=matches
    if not any(q['status']=='PASS'for q in matches):record['status']='UNKNOWN'
    return record


def fixed_regressions(cells,bodies,u):
    original=json.loads((ROOT/'reports/nero-gripper-v21-normal-witnesses.json').read_text())['confirmed_witnesses']
    v22=json.loads((ROOT/'reports/nero-gripper-v22-local-verification.json').read_text());lookup={e['id']:e for e in cells};st=source_arrays();tree=TriangleBVH(st)
    inputs=[]
    for w in original:inputs.append(('original11',w['source_stl_face'],w['source_point_mm'],w['source_normal']))
    introduced=[q for q in v22['baseline_normal_comparisons']if q['classification']=='INTRODUCED_REAL_NORMAL_FAILURE'];assert len(introduced)==15
    for q in introduced:
        old=next(w for w in v22['all_A_sidewalls']['new_real_normal_counterexamples']if w['patch']==q['patch'])
        inputs.append(('v22_normal15',q['source_face'],q['source_point_mm'],old['source_normal']))
    surface=v22['independent_all_tier_point_scan']['A']['certified_source_exterior_runtime_void_failures'];assert len(surface)==12
    for q in surface:inputs.append(('v22_surface12',q['source_face'],q['source_point_mm'],q['source_normal']))
    explicit=np.array(json.loads(PRE.read_text())['mandatory_regressions']['explicit_source2030_point_mm']);_,ix=tree.query(explicit[None]);t=st[int(ix[0])];n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n)
    inputs.append(('explicit2030',2030,explicit.tolist(),n.tolist()))
    records=[]
    for group,face,point,n in inputs:
        row=boundary_point(np.array(point),np.array(n),u,bodies);row.update(group=group,source_face=face,source_point_mm=point,source_normal=n)
        records.append(saved_facet_check(row,lookup))
    return {'counts':{g:{'total':sum(q['group']==g for q in records),'PASS':sum(q['group']==g and q['status']=='PASS'for q in records),'UNKNOWN':sum(q['group']==g and q['status']!='PASS'for q in records)}for g in ['original11','v22_normal15','v22_surface12','explicit2030']},'records':records}


def scale_checks(cells,bodies):
    rows=[];lookup={b['id']:b for b in bodies}
    for e in cells:
        v=np.array(e['vertices_m'])*1000;span=np.ptp(v,axis=0);axis=int(np.argmin(span));width=float(span[axis]);b=lookup.get(e['id']);eq=b['equations']if b else None
        lp=linprog([0,0,0,-1],A_ub=np.c_[eq[:,:3],np.ones(len(eq))],b_ub=-eq[:,3],bounds=[(None,None)]*3+[(0,None)],method='highs')if b else None
        estimated=float(2*lp.x[3])if lp is not None and lp.success else None
        center_margin=float(np.min(-(eq[:,:3]@lp.x[:3]+eq[:,3])))if lp is not None and lp.success else None
        rows.append({'entity':e['id'],'baseline_entity':e.get('baseline_entity',e['id']),'lp_success':bool(lp is not None and lp.success),'raw_lp_diameter_mm':estimated,'actual_center_minimum_plane_margin_mm':center_margin,'bbox_axis':axis,'bbox_width_mm':width,'inscribed_diameter_upper_bound_mm':width+NUM,'status':'FAIL'if width+NUM<.1 else'FAIL_LP_ESTIMATE'if estimated is not None and 0<=estimated<.1 else'UNKNOWN'if estimated is None or estimated<0 or center_margin is None or center_margin<0 else'PASS_LP_ESTIMATE'})
    certified=[r for r in rows if r['status']=='FAIL'];physical=[r for r in certified if r['bbox_width_mm']>=100*NUM]
    return {'status':'FAIL'if certified else'UNKNOWN','entity_count':len(rows),'axis_width_certified_fail_count':len(certified),'lp_below_threshold_estimate_count':sum(r['raw_lp_diameter_mm']is not None and 0<=r['raw_lp_diameter_mm']<.1 for r in rows),'invalid_negative_lp_estimate_count':sum(r['raw_lp_diameter_mm']is not None and r['raw_lp_diameter_mm']<0 for r in rows),'minimum_physical_diameter_exact_mm':None,'global_minimum_diameter_interval_mm':[0.,min(r['inscribed_diameter_upper_bound_mm']for r in rows)],'non_numerical_failure_example':min(physical,key=lambda r:r['bbox_width_mm'])if physical else None,'rows':rows,'scope':'A sphere diameter cannot exceed ANY coordinate span of the saved vertices. This is a sufficient real failure independent of ill-conditioned thin-cell LP estimates. Negative LP values are not physical diameters.'}


def main():
    started=time.time();pre=json.loads(PRE.read_text());data=json.loads(OUT.read_text());cells=data['entities'];assert sha(BASE)==pre['baseline_file_sha256']and geosha(cells)==data['geometry_sha256']
    for p,h in pre['policy_sha256'].items():assert sha(ROOT/p)==h
    old=json.loads(BASE.read_text())['entities'];by={e['id']:e for e in cells};unchanged=[]
    for e in old:
        if e['id']in data['modified_baseline_entities']:continue
        assert json.dumps(e,separators=(',',':'))==json.dumps(by[e['id']],separators=(',',':'))
        assert np.array(e['vertices_m']).tobytes()==np.array(by[e['id']]['vertices_m']).tobytes()
        assert np.array(e['faces']).tobytes()==np.array(by[e['id']]['faces']).tobytes();unchanged.append(e['id'])
    assert len(unchanged)==314
    bodies,representation=bodies_from_saved(cells);valid_ids={b['id']for b in bodies};valid_cells=[e for e in cells if e['id']in valid_ids];u=oracle_for(valid_cells,bodies)
    result={'status':'V23_FINITE_PATCH_REPARTITION_BLOCKED','candidate_geometry_sha256':data['geometry_sha256'],'candidate_file_sha256':sha(OUT),'preregistration_sha256':sha(PRE),'verifier_sha256':sha(__file__),'entity_count':len(cells),'baseline_entity_count':346,'candidate_generation_count':1,'modified_baseline_entities':data['modified_baseline_entities'],'unmodified_full_entity_objects_and_array_bytes':unchanged,'representation_checks':representation,'scale':scale_checks(cells,bodies),'fixed_regressions':fixed_regressions(cells,bodies,u),'scope':{'baseline':'original v2.1 /6eba4f3','failed_v22_geometry_used':False,'mujoco_imported':any(k.startswith('mujoco')for k in sys.modules),'formal_model_changes':0,'physics_integrations':0}}
    save(REPORT,result);print('Fixed regressions',result['fixed_regressions']['counts'],flush=True)
    excluded=[e for e in cells if e['id']not in valid_ids]
    for row in result['fixed_regressions']['records']:
        q=np.array(row['source_point_mm']);bounds=[]
        for e in excluded:
            v=np.array(e['vertices_m'])*1000;distance=float(np.linalg.norm(np.maximum(np.maximum(v.min(0)-q,q-v.max(0)),0)))
            bounds.append({'entity':e['id'],'bbox_distance_lower_mm':distance,'outside10um_matching_ball':distance>.01+NUM})
        row['unresolved_cell_bbox_exclusion']=bounds
        if any(not q['outside10um_matching_ball']for q in bounds):row['status']='UNKNOWN'
    for group,counts in result['fixed_regressions']['counts'].items():
        counts['PASS']=sum(q['group']==group and q['status']=='PASS'for q in result['fixed_regressions']['records']);counts['UNKNOWN']=counts['total']-counts['PASS']
    save(REPORT,result)
    try:
        result['all_A_sidewalls']=all_patches(valid_cells,bodies,u)
    except Exception as error:
        result['all_A_sidewalls']={'status':'UNKNOWN','error':str(error),'not_replaced_by_point_PASS':True}
    save(REPORT,result)
    try:
        p,_=policy();result['material_voids']=material_voids(cells,p)
    except Exception as error:
        result['material_voids']={'status':'UNKNOWN','error':str(error),'reason':'Do not infer material-set or void PASS from numerically invalid thin-cell Boolean inputs.'}
    result['elapsed_seconds']=time.time()-started
    result['blockers']={'scale_real_failure':result['scale']['axis_width_certified_fail_count']>0,'saved_cell_hull_unresolved_ids':[q['entity']for q in representation if q['status']=='UNKNOWN_SAVED_CELL_HULL_NUMERICALLY_UNRESOLVED'],'fragmentation_not_a_few_splits':max(len(m['new'])for m in data['old_to_new']if m['changed']), 'all_A_contour_status':'UNKNOWN','outside_prism_material_preservation':'PENDING_INDEPENDENT_CERTIFICATION'}
    save(REPORT,result);print(json.dumps({'status':result['status'],'entity_count':len(cells),'scale_axis_width_certified_failures':result['scale']['axis_width_certified_fail_count'],'regressions':result['fixed_regressions']['counts']},indent=2),flush=True)


if __name__=='__main__':main()
