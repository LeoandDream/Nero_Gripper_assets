"""Fresh protected-void, tier, old-region and baseline no-regression checks."""
import gzip
import json
import time

import numpy as np

from v23_finite_patch import ROOT,TASK,BASE,OUT,PRE,NUM,sha,save,patches
from verify_v23_finite import REPORT,bodies_from_saved
from verify_v22_local import oracle_for,boundary_point,source_arrays,source_external
from v21_boundary_atlas import support_cells,closure_pair_certificate
from runtime_backing_partition import segment_clearance
from analyze_runtime_partition import TriangleBVH
from adjudicate_v2_partition import legacy_missing_locators,source_line_intervals
from verify_v2_material_voids import axis_intervals

FILE=ROOT/'reports/nero-gripper-v23-supplement.json'


def main():
    start=time.time();data=json.loads(OUT.read_text());cells=data['entities'];old=json.loads(BASE.read_text())['entities'];bodies,checks=bodies_from_saved(cells);validids={b['id']for b in bodies};valid=[e for e in cells if e['id']in validids];u=oracle_for(valid,bodies)
    oldb,_=support_cells(old);ou=oracle_for(old,oldb);unknown=[e for e in cells if e['id']not in validids];r=json.loads(REPORT.read_text());pp,_=patches();lookup={p['id']:p for p in pp}
    comparisons=[]
    for current in r['all_A_sidewalls'].get('records',[]):
        p=lookup[current['patch']];before=boundary_point(np.array(current['source_point_mm']),np.array(p['plane'][:3]),ou,oldb)
        after=current['point_regression'];comparisons.append({'patch':current['patch'],'source_face':p['source_face'],'baseline_point_status':before['status'],'candidate_point_status':after['status'],'baseline_pass_candidate_unresolved':before['status']=='PASS'and after['status']!='PASS','true_new_failure_inferred':False})
    result={'candidate_geometry_sha256':data['geometry_sha256'],'candidate_file_sha256':sha(OUT),'verifier_sha256':sha(__file__),'all_patch_point_baseline_comparison':comparisons,'all_patch_contour_status':'UNKNOWN','baseline_pass_candidate_unresolved_count':sum(q['baseline_pass_candidate_unresolved']for q in comparisons)}
    # Every named source tier is scanned; only sufficient true failure
    # witnesses are called FAIL. Internal C faces are not forced external.
    z=np.load(ROOT/'reports/nero-gripper-v2-union-exterior.npz');st=source_arrays();source_tree=TriangleBVH(st);tree=TriangleBVH(np.concatenate([np.array(e['vertices_m'])[e['faces']]*1000 for e in cells]));weights=np.array([[1/3]*3,[.6,.2,.2],[.2,.6,.2],[.2,.2,.6],[.8,.1,.1],[.1,.8,.1],[.1,.1,.8]]);scans={}
    for tier in ['A','B','C']:
        triangles=z['reference_'+tier+'_triangles_mm'];points=np.einsum('wv,tvc->twc',weights,triangles).reshape(-1,3);dd,_=tree.query(points);limit=.01 if tier=='A'else .05;certs=[]
        for i in np.flatnonzero(dd-NUM>limit):
            point=points[i];source_dist,nearest=source_tree.query(point[None]);face=int(nearest[0]);t=st[face];n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n);exposure=source_external(point,n,st);state=u.membership([point/1000])[0]
            if source_dist[0]<=NUM and exposure['confirmed']and state['state']=='VOID':certs.append({'point_mm':point.tolist(),'source_face':face,'saved_closed_facet_distance_lower_mm':float(dd[i]-NUM),'source_exposure':exposure,'regularized_known_solids_state':state})
        scans[tier]={'points_evaluated':len(points),'triangles_evaluated':len(triangles),'maximum_sample_to_any_saved_closed_facet_mm':float(dd.max()),'sufficient_new_or_persisting_true_surface_failure_witnesses':certs,'full_surface_contour_status':'FAIL'if certs else'UNKNOWN','C_domains_not_retiered':True}
        print('tier scan',tier,len(points),'sufficient true fails',len(certs),flush=True)
    result['tier_scans']=scans
    gs={i:{'poly':{'v':b['vertices'],'eq':b['equations'],'lo':b['lo'],'hi':b['hi']}}for i,b in enumerate(bodies)}
    evidence=json.loads((ROOT/'reports/nero-gripper-runtime-evidence.json').read_text())['constructors'];segments=[]
    for feature in evidence['backing']['verification.json']['protected_void_witnesses']:
        for q in feature['probes']:
            segments.append({'feature':feature['feature'],'a_mm':q['a_mm'],'b_mm':q['b_mm'],'new_query':segment_clearance(q['a_mm'],q['b_mm'],gs)})
    mount=evidence['mount']['feature-witnesses.json'];points=[]
    for q in mount['witnesses']:
        point=np.array(q['point_m'])*1000;state=u.membership([point/1000])[0];points.append({'feature':q['feature'],'point_mm':point.tolist(),'expected':q['expected'],'new_state':state,'matches_expected':(state['state']=='SOLID')==(q['expected']=='solid')})
    floors=[];oldfloor=json.loads((ROOT/'reports/nero-gripper-v2-material-voids.json').read_text())['protected_voids']['blind_recess_floors']
    for q in mount['blind_recess_axis_floors']:
        x,z0=q['axis_xz_mm'];intervals=axis_intervals(x,z0,gs);ends=[b for a,b,i in intervals if -73.5<b<-70.1];value=max(ends)if ends else None;target=next(f['source_floor_y_mm']for f in oldfloor if f['axis_xz_mm']==q['axis_xz_mm']);floors.append({'axis_xz_mm':q['axis_xz_mm'],'source_y_mm':target,'candidate_y_mm':value,'status':'PASS'if value is not None and abs(value-target)<=.05 else'UNKNOWN'})
    central=[]
    for x,z0 in [(0.,12.25),(.8,12.25),(-.8,12.25),(0.,13.05),(0.,11.45)]:
        mount_int,_,mount_amb,mw=source_line_intervals(st[:504]/1000,x/1000,z0/1000);back_int,_,back_amb,bw=source_line_intervals(st[660:]/1000,x/1000,z0/1000);runtime=axis_intervals(x,z0,gs);cuts=[(max(a,-72.9999),min(b,-69.9999),i)for a,b,i in runtime if max(a,-72.9999)<min(b,-69.9999)];source_entry=min((float(a*1000)for a,b in back_int),default=None);new_entry=min((a for a,b,i in cuts),default=None)
        central.append({'axis_xz_mm':[x,z0],'source_mount_clear':not mount_int and not mount_amb and mw==0,'source_backing_entry_y_mm':source_entry,'candidate_entry_y_mm':new_entry,'status':'PASS'if not mount_int and not mount_amb and not back_amb and source_entry is not None and new_entry is not None and abs(source_entry-new_entry)<=.05 else'UNKNOWN'})
    result['protected_void_subchecks']={'continuous_segments':segments,'mount_witnesses':points,'blind_recess_axes':floors,'central_v21_paths':central,'all34_segments_clear':all(q['new_query']['clear']for q in segments),'all131_mount_witnesses_match':all(q['matches_expected']for q in points),'all4_floors_pass':all(q['status']=='PASS'for q in floors),'all5_central_paths_pass':all(q['status']=='PASS'for q in central),'full_side_channels_rear_pocket_bores_recesses_central_slot_notches_status':'UNKNOWN','scope':'Queries use the regularized analyzable solid cells; one numerically unresolved saved cell remains explicit. No new blockage or wall-identity PASS is inferred from finite path checks.'}
    # Retain the original historical coordinates and face indices, never new
    # successor indices. Recheck the10 old regions plus the4 old atlas gaps.
    adj=json.loads((ROOT/'reports/nero-gripper-v2-adjudication.json').read_text());cert=json.loads((ROOT/'reports/nero-gripper-v2-runtime-verification.json').read_text())['union_exterior_certificate'];oldtri=z['vertices_mm'][z['faces']]/1000;loc=legacy_missing_locators(old,oldtri,cert['failures']+cert['occluded_output_triangles']);prior=json.loads((ROOT/'reports/nero-gripper-v21-rechecks.json').read_text());trees=[TriangleBVH(np.array(e['vertices_m'])[e['faces']]*1000)for e in valid];rechecks=[]
    for prev in prior['old14_rechecks']:
        if prev['new_status']!='UNRESOLVED':continue
        f=adj['certificate_failures'][prev['old_record_index']]
        locator=loc.get((f['entity'],f['face']))if f.get('entity')else loc.get((f['triangle'],f.get('covering_entity')))
        polygons=locator['patch_vertices']if locator else[oldtri[f['triangle']]];proof=[]
        for polygon in polygons:
            q=np.array(polygon);inside=u.strict_cover(q);proof.append({'old_patch_mm':(q*1000).tolist(),'continuous_interior':inside,'continuous_equivalent_closure':None if inside else closure_pair_certificate(q*1000,bodies,trees)})
        status='CONTINUOUS_INTERNAL_PATCH'if proof and all(q['continuous_interior']for q in proof)else'REPRESENTATION_EQUIVALENT_WITHIN_SUPPORT'if proof and all(q['continuous_interior']or q['continuous_equivalent_closure']for q in proof)else'UNKNOWN'
        rechecks.append({'triangle':f.get('triangle'),'old_entity':f.get('entity'),'old_face':f.get('face'),'status':status,'proofs':proof,'historical_coordinates_not_reindexed':True})
    assert len(rechecks)==10;result['old10_regions']=rechecks
    gaps=[];oldatlas=json.loads((ROOT/'reports/nero-gripper-v21-boundary-atlas.json').read_text());oldby={b['id']:b for b in oldb}
    for gap in oldatlas['unresolved']:
        oldbody=oldby[gap['entity']];q=oldbody['facets'][gap['facet']]['vertices'];inside=u.strict_cover(q/1000);bridge=None if inside else closure_pair_certificate(q,bodies,trees)
        gaps.append({'old_gap':gap,'old_facet_vertices_mm':q.tolist(),'status':'CONTINUOUS_INTERNAL_PATCH'if inside else'REPRESENTATION_EQUIVALENT_WITHIN_SUPPORT'if bridge else'UNKNOWN','continuous_interior':inside,'closure_proof':bridge})
    assert len(gaps)==4;result['old4_atlas_regions']=gaps;result['new_full_union_exterior_and_contour_certificate']='UNKNOWN_NOT_REBUILT_AFTER_HARD_SCALE_FAILURE';result['elapsed_seconds']=time.time()-start
    save(FILE,result)
    preservation=json.loads(gzip.decompress((ROOT/'reports/nero-gripper-v23-preservation.json.gz').read_bytes()))
    primary=json.loads(REPORT.read_text())
    primary['outside_footprint_material_preservation']={'ideal_partition_identity':preservation['ideal_finite_prism_partition_identity'],'saved_implementation_status':preservation['actual_saved_geometry_outside_preservation'],'root_count':len(preservation['roots']),'counterexample_of_actual_outside_material_loss_found':False,'scope':'UNKNOWN is not a proof of absence of outside material loss; detailed continuous certificates and unresolved obligations are retained.'}
    primary['blockers']['outside_prism_material_preservation']=preservation['actual_saved_geometry_outside_preservation']
    primary['blockers']['B_C_and_protected_void_full_non_degradation']='UNKNOWN'
    primary['old_regions_summary']={'old10_rechecked':10,'old10_still_unknown':sum(q['status']=='UNKNOWN'for q in rechecks),'old4_atlas_rechecked':4,'old4_still_unknown':sum(q['status']=='UNKNOWN'for q in gaps)}
    primary['supplementary_evidence']={'path':str(FILE.relative_to(ROOT)),'sha256':sha(FILE),'preservation_path':'reports/nero-gripper-v23-preservation.json.gz','preservation_sha256':sha(ROOT/'reports/nero-gripper-v23-preservation.json.gz')}
    primary['final_supplement_verifier_sha256']=sha(__file__)
    primary['status']='V23_FINITE_PATCH_REPARTITION_BLOCKED'
    save(REPORT,primary)
    print(json.dumps({'old10_statuses':[q['status']for q in rechecks],'old4_statuses':[q['status']for q in gaps],'segments34':result['protected_void_subchecks']['all34_segments_clear'],'points131':result['protected_void_subchecks']['all131_mount_witnesses_match'],'final_status':primary['status']},indent=2),flush=True)


if __name__=='__main__':main()
