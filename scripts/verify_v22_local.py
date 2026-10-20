"""Read-only acceptance of the ONE v22 local candidate; no old report writes.

Continuous source-patch coverage is separate from point regressions. Unknown
target/exterior/contour proofs never turn into PASS from sampling.
"""
from pathlib import Path
import argparse,collections,hashlib,json,sys,time
import numpy as np
from scipy.spatial import ConvexHull
from v22_local_repartition import ROOT,TASK,BASE,OUT,patches,geosha,sha,save,ALLOWED
from v21_boundary_atlas import NUM,GRID,basis,polygon,pieces,area_shape,clip,support_cells,build_atlas
from adjudicate_v2_partition import Union
from runtime_contact_partition import load_source
from runtime_backing_partition import inradius,poly
from analyze_runtime_partition import TriangleBVH
from verify_v21_partition import policy,material_voids
from shapely.ops import unary_union
from shapely import set_precision

REPORT=ROOT/'reports/nero-gripper-v22-local-verification.json'
def oracle_for(cells,bodies):
    return Union([{'id':e['id'],'halfspaces_m':(b['equations']*[1,1,1,.001]).tolist()}for e,b in zip(cells,bodies)],NUM/64)
def boundary_point(p,n,u,bodies):
    point=np.array(p)/1000;n=np.array(n)
    intervals,merged,gaps,local=u.line_intervals(point,n,.03/1000)
    ends=sorted({float(m[1])for m in merged if abs(float(m[1]))<=.03/1000},key=abs)
    records=[]
    for end in ends:
        q=point+end*n;states=u.membership([q-2*NUM/1000*n,q+2*NUM/1000*n])
        if [s['state']for s in states]!=['SOLID','VOID']:continue
        owners=[]
        for lo,hi,i in intervals:
            if abs(float(hi)-end)*1000>NUM/64:continue
            eq=bodies[i]['equations'];delta=abs(eq[:,:3]@(q*1000)+eq[:,3]);rows=np.flatnonzero(delta<=NUM/32)
            for row in rows:
                if eq[row,:3]@n<=0:continue
                owners.append({'entity':bodies[i]['id'],'support_plane':int(row),'normal':eq[row,:3].tolist(),'normal_deviation_deg':float(np.degrees(np.arccos(np.clip(eq[row,:3]@n,-1,1))))})
        accepted=[o for o in owners if o['normal_deviation_deg']+1e-5<=.05]
        records.append({'point_mm':(q*1000).tolist(),'signed_displacement_mm':end*1000,'euclidean_distance_mm':abs(end)*1000,'distance_upper_mm':abs(end)*1000+NUM,'sides':[s['state']for s in states],'boundary_facets':owners,'normal_pass_exists':bool(accepted),'status':'PASS'if accepted and abs(end)*1000+NUM<=.01 else'UNKNOWN'})
        if records[-1]['status']=='PASS':return records[-1]
    return min(records,key=lambda r:r['euclidean_distance_mm'])if records else {'status':'UNKNOWN','reason':'No effective source-normal ray exit certified within30um.'}
def source_arrays():
    s=load_source();return np.array(s['vertices_m'])[np.array([t['vertex_ids']for t in s['triangles']])]*1000
def winding(point,t):
    a,b,c=np.moveaxis(t-point,1,0);la=np.linalg.norm(a,axis=1);lb=np.linalg.norm(b,axis=1);lc=np.linalg.norm(c,axis=1)
    top=np.einsum('ij,ij->i',a,np.cross(b,c));bottom=la*lb*lc+(a*b).sum(1)*lc+(b*c).sum(1)*la+(c*a).sum(1)*lb
    return float(np.sum(2*np.arctan2(top,bottom))/(4*np.pi))
def source_external(point,n,st):
    inside=[winding(point-1e-5*n,st[a:b])for a,b in [(0,504),(504,660),(660,2086)]]
    outside=[winding(point+1e-5*n,st[a:b])for a,b in [(0,504),(504,660),(660,2086)]]
    return {'inside':inside,'outside':outside,'confirmed':any(abs(q-1)<1e-5 for q in inside)and all(abs(q)<1e-5 for q in outside)}
def normal_counterexample(point,n,cells,bodies,u):
    """Continuous ALL-closed-facet superset, including edge normal cones."""
    ray=boundary_point(point,n,u,bodies)
    if not ray.get('boundary_facets')or ray['euclidean_distance_mm']>.01:return None
    exitn=np.array(ray['boundary_facets'][0]['normal']);half=.01+NUM;remaining=[];buried=[]
    for e,b in zip(cells,bodies):
        if np.any(b['hi']<point-half)or np.any(b['lo']>point+half):continue
        v=np.array(e['vertices_m'])*1000
        for j,t in enumerate(v[np.array(e['faces'])]):
            if np.any(t.max(0)<point-half)or np.any(t.min(0)>point+half):continue
            q=t.copy()
            for k in range(3):
                eq=np.zeros(4);eq[k]=1;eq[3]=-(point[k]+half);q=clip(q,eq)
                if len(q)<3:break
                eq=np.zeros(4);eq[k]=-1;eq[3]=point[k]-half;q=clip(q,eq)
                if len(q)<3:break
            if len(q)<3:continue
            proof=u.strict_cover(q/1000)
            if proof and proof['maximum_vertex_plane_residual_mm']<-NUM:
                buried.append({'entity':e['id'],'face':j,'polygon_mm':q.tolist(),'proof':proof});continue
            nn=np.cross(np.array(t[1]-t[0],dtype=np.longdouble),np.array(t[2]-t[0],dtype=np.longdouble));nn=np.array(nn/np.linalg.norm(nn),dtype=float)
            if nn@(t.mean(0)-v.mean(0))<0:nn=-nn
            remaining.append({'entity':e['id'],'face':j,'normal':nn.tolist(),'angle_to_exit_deg':float(np.degrees(np.arccos(np.clip(nn@exitn,-1,1)))),'polygon_mm':q.tolist()})
    radius=max((r['angle_to_exit_deg']for r in remaining),default=180)+1e-5
    deviation=float(np.degrees(np.arccos(np.clip(n@exitn,-1,1))));lower=deviation-radius
    return {'status':'FAIL'if radius<90 and lower>.05 else'UNKNOWN','ray':ray,'source_point_mm':point.tolist(),'source_normal':n.tolist(),'cube_half_width_mm':half,'strictly_buried':buried,'remaining_possible_boundary':remaining,'normal_cone_radius_upper_deg':radius,'normal_deviation_lower_bound_deg':lower,'arithmetic_angle_reserve_deg':1e-5}
def all_patches(cells,bodies,u):
    pp,st=patches();records=[];counter=[]
    for index,p in enumerate(pp):
        q=np.array(p['polygon_mm']);plane=np.array(p['plane']);n=plane[:3];uu,ww=basis(n);shape=polygon(q,uu,ww);covers=[];occluded=[];supporters=[]
        for b in bodies:
            if np.any(b['hi']<q.min(0)-NUM)or np.any(b['lo']>q.max(0)+NUM):continue
            eq=b['equations'];dots=eq[:,:3]@n;heights=np.max(abs(q@eq[:,:3].T+eq[:,3]),axis=0)
            matching=np.flatnonzero((dots>=np.cos(np.deg2rad(.05)))&(heights<=NUM/32))
            if len(matching):
                cut=q.copy()
                for k,row in enumerate(eq):
                    if k in matching:continue
                    relaxed=row.copy();relaxed[3]-=NUM/128;cut=clip(cut,relaxed)
                    if len(cut)<3:break
                if len(cut)>=3:
                    pg=area_shape(polygon(cut,uu,ww).intersection(shape))
                    if not pg.is_empty:covers.append(pg);supporters.append({'entity':b['id'],'planes':matching.tolist(),'angle_deg':float(np.degrees(np.arccos(np.clip(dots[matching].min(),-1,1))))})
            probe=q+2*NUM*n;cut=probe
            for row in eq:
                cut=clip(cut,row)
                if len(cut)<3:break
            if len(cut)>=3:
                projected=cut-2*NUM*n;pg=area_shape(polygon(projected,uu,ww).intersection(shape))
                if not pg.is_empty:occluded.append(pg)
        covered=area_shape(unary_union(covers));hidden=area_shape(unary_union(occluded));exposed=area_shape(covered.difference(hidden))
        # A boundary-Hausdorff/topology proof is NOT inferred from tiny area.
        uncovered=area_shape(shape.difference(exposed.buffer(NUM/32)))
        continuous_pass=bool(uncovered.is_empty and hidden.is_empty and covers)
        center=q.mean(0);source=source_external(center,n,st);ray=boundary_point(center,n,u,bodies)
        rec={'patch':p['id'],'source_face':p['source_face'],'source_region':p['region'],'source_area_mm2':float(shape.area),'source_point_mm':center.tolist(),'continuous_exact_plane_coverage_status':'PASS'if continuous_pass else'UNKNOWN','uncovered_area_mm2':float(uncovered.area),'outward_occluded_area_mm2':float(hidden.area),'supporters':supporters,'point_source_exposure':source,'point_regression':ray}
        if source['confirmed'] and ray['status']!='PASS' and len(counter)<16:
            proof=normal_counterexample(center,n,cells,bodies,u)
            if proof and proof['status']=='FAIL':proof.update(patch=p['id'],source_face=p['source_face'],source_exposure=source);counter.append(proof);rec['real_failure_certificate']=len(counter)-1
        records.append(rec)
        if(index+1)%100==0:print('ALL A patches',index+1,len(pp),'real normal failures',len(counter),flush=True)
    return {'records':records,'source_patch_count':len(pp),'source_face_count':len({p['source_face']for p in pp}),'continuous_exact_plane_PASS_count':sum(r['continuous_exact_plane_coverage_status']=='PASS'for r in records),'point_PASS_count':sum(r['point_regression']['status']=='PASS'for r in records),'new_real_normal_counterexamples':counter,'all_sidewall_surface_normal_status':'FAIL'if counter else'PASS'if all(r['continuous_exact_plane_coverage_status']=='PASS'for r in records)else'UNKNOWN','contour_status':'UNKNOWN','contour_reason':'Exact source patch plane coverage and finite rays do not certify bidirectional union-exterior contour or new closure-edge responsibility.'}
def main():
    started=time.time();pre=json.loads((ROOT/'reports/nero-gripper-v22-local-preregistration.json').read_text());data=json.loads(OUT.read_text());cells=data['entities'];old=json.loads(BASE.read_text())['entities'];assert geosha(cells)==data['geometry_sha256']
    for p,h in pre['v21_policy_sha256'].items():assert sha(ROOT/p)==h
    changed=set(data['modified_entity_ids']);unchanged=[]
    for a,b in zip(old,cells):
        if a['id']not in changed:
            assert json.dumps(a,separators=(',',':'))==json.dumps(b,separators=(',',':'));unchanged.append(a['id'])
        else:assert a['semantic_family']in ALLOWED
    bodies,reprchecks=support_cells(cells);u=oracle_for(cells,bodies);scales=[];contain=[]
    oldb,_=support_cells(old)
    for e,b,ob in zip(cells,bodies,oldb):
        diameter,center=inradius(poly(b['vertices']));scales.append({'entity':e['id'],'diameter_mm':2*diameter,'status':'PASS'if 2*diameter>=.1 else'FAIL'})
        residual=float(np.max(b['vertices']@ob['equations'][:,:3].T+ob['equations'][:,3]));contain.append({'entity':e['id'],'new_in_old_vertex_halfspace_maximum_mm':residual,'continuous_subset_status':'PASS_MODULO_NUM'if residual<=NUM/32 else'UNKNOWN'})
    mandatory=[]
    for w in json.loads((ROOT/'reports/nero-gripper-v21-normal-witnesses.json').read_text())['confirmed_witnesses']:
        p=np.array(w['source_point_mm']);n=np.array(w['source_normal']);r=boundary_point(p,n,u,bodies);r.update(source_face=w['source_stl_face'],source_point_mm=p.tolist(),original_witness_source_sha256=pre['mandatory_regression_sha256']);mandatory.append(r)
    result={'status':'V22_LOCAL_REPARTITION_BLOCKED','candidate_geometry_sha256':data['geometry_sha256'],'candidate_file_sha256':sha(OUT),'preregistration_sha256':sha(ROOT/'reports/nero-gripper-v22-local-preregistration.json'),'verifier_sha256':sha(__file__),'entity_count':len(cells),'modified_entities':sorted(changed),'unmodified_entity_objects_byte_equivalent':unchanged,'all_new_cells_subsets_of_corresponding_old_cells':all(c['continuous_subset_status']=='PASS_MODULO_NUM'for c in contain),'cell_containment_certificates':contain,'minimum_entity_scale':{'status':'PASS'if all(s['status']=='PASS'for s in scales)else'FAIL','minimum_diameter_mm':min(s['diameter_mm']for s in scales),'entities':scales},'representation_checks':reprchecks,'mandatory_11':{'status':'PASS'if all(w['status']=='PASS'for w in mandatory)else'UNKNOWN','pass_count':sum(w['status']=='PASS'for w in mandatory),'records':mandatory},'policy_unchanged':pre['v21_policy_sha256'],'scope':{'new_candidates':1,'changed_formal_models':0,'mujoco_imported':any(k.startswith('mujoco')for k in sys.modules),'physics_integrations':0}}
    save(REPORT,result)
    result['all_A_sidewalls']=all_patches(cells,bodies,u);save(REPORT,result)
    # Reuse the frozen previous read-only material/void checker, without its
    # hardcoded main output paths. Fresh results written only under v22.
    p,_=policy();result['material_voids']=material_voids(cells,p);save(REPORT,result)
    result['protected_unchanged_families']={f:{'status':'PASS_GEOMETRY_IDENTITY','entity_ids':[e['id']for e in old if e['semantic_family']==f],'scope':'Frozen entity objects identical. A global union non-interference claim still needs checking of removed overlapping material.'}for f in ['inner_main_contact_plate','distal_tip_contact','inner_ramp_contact','inner_transition_contact','mount_root']}
    prior=json.loads((ROOT/'reports/nero-gripper-v21-rechecks.json').read_text());result['historical_unresolved']={'previous_count':10,'previous_details':prior['old14_rechecks'],'status':'PENDING_DIRECT_RECHECK'}
    result['previous_global_boundary_atlas_uncertainty']={'count':4,'not_automatically_cleared':True}
    result['elapsed_seconds']=time.time()-started;result['blocker_classification']={'real_A_normal_failure':bool(result['all_A_sidewalls']['new_real_normal_counterexamples']),'scale_failure':result['minimum_entity_scale']['status']=='FAIL','protected_void_degradation':'UNKNOWN','B_C_non_degradation':'UNKNOWN','contour_certification':'UNKNOWN'}
    save(REPORT,result);print(json.dumps({'status':result['status'],'mandatory_pass':result['mandatory_11']['pass_count'],'all_patch_exact_plane_pass':result['all_A_sidewalls']['continuous_exact_plane_PASS_count'],'all_patch_count':result['all_A_sidewalls']['source_patch_count'],'real_normal_counterexamples':len(result['all_A_sidewalls']['new_real_normal_counterexamples']),'minimum_scale':result['minimum_entity_scale']['minimum_diameter_mm']},indent=2),flush=True)
if __name__=='__main__':main()
