"""Issue #25 classification only: read saved geometry; never emit geometry.

Reuses the existing finite nearest-triangle measurement on the unchanged v1
exterior meshes. All reported failures retain source-face/sample provenance.
No hull, clipping, model, simulator, or candidate construction is called.
"""
from pathlib import Path
import collections
import csv
import hashlib
import json
import sys

import numpy as np
from analyze_runtime_partition import TriangleBVH, exterior, inside_union, triangle_distance
from runtime_contact_partition import load_source

ROOT=Path(__file__).resolve().parents[1]
TASK=ROOT/'debug/issue25/v2-policy'
WEIGHTS=np.array([[1,0,0],[0,1,0],[0,0,1],[.5,.5,0],[0,.5,.5],[.5,0,.5],[1/3,1/3,1/3]])


def read(name):
    return json.loads((ROOT/name).read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(name,obj):
    (ROOT/name).write_text(json.dumps(obj,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')


def main():
    policy=read('reports/nero-gripper-v2-fidelity-protocol.json')
    for name,expected in policy['input_sha256'].items():assert sha(ROOT/name)==expected,name
    ref=read('reports/nero-gripper-entity-partition.json')
    run=read('reports/nero-gripper-runtime-partition.json')
    old_errors=read('reports/nero-gripper-runtime-errors.json')
    catalog=policy['source_region_policy'];face_regions={}
    for name,item in catalog.items():
        for a,b in item['source_triangle_ranges_zero_based_inclusive']:
            for i in range(a,b+1):assert i not in face_regions;face_regions[i]=name
    assert len(face_regions)==2086

    def tiers(region,points):
        entry=catalog[region];t=entry['tier']
        if t in ['A','B','C']:return np.full(len(points),t)
        mouth=entry['predicate']['mouth_y_mm']
        return np.where(np.array(points)[:,1]>=mouth-.1,'B','C')

    old_byid={e['id']:e for e in ref['entities']};thin=[]
    for e in run['entities']:
        if e['inscribed_diameter_mm']>=.1-1e-8:continue
        region_areas=collections.Counter();owned_faces=collections.defaultdict(set)
        obligations=set();retired=set()
        for rid in e['reference_entity_ids']:
            for sp in old_byid[rid]['source_surface_partition']:
                region=face_regions[sp['source_face_id']]
                for p in sp['exterior_subpolygons']:
                    ar=p['area_m2']*1e6
                    if ar<=1e-12:continue
                    region_areas[region]+=ar;owned_faces[region].add(sp['source_face_id'])
                    ts=set(tiers(region,np.array(p['polygon_m'])*1000))
                    if ts & {'A','B'}:obligations.add(region)
                    if 'C' in ts:retired.add(region)
        if e['source_part']=='contact_layer':
            tier='A';reason='Carries the true workface or its exposed taper/shoulder perimeter; preserve functionality while repartitioning the thin convex cut.'
        elif e['source_part']=='backing':
            tier='A' if any(catalog[n]['tier']=='A' for n in obligations) else 'B'
            reason='Major manipulation sidewall/tip boundary' if tier=='A' else 'Macroscopic pocket wall, root bulkhead or outer roof; thinness is a partition artefact, not permission to remove the shell.'
        elif 'mount_notch_floor' in obligations or 'mount_notch_left_right_walls' in obligations:
            tier='B';reason='Carries an actual root-slot boundary; preserve that macro void/silhouette while repartitioning.'
        else:
            tier='C';reason='Assembly bore/recess terrain sliver has no independent manipulation identity. Absorb its entity identity, while delegating EVERY incident B cap/mouth/protected-void obligation to the replacement.'
        thin.append({'id':e['id'],'name':e['name'],'source_part':e['source_part'],'semantic_feature':e['semantic_feature'],'classification_tier':tier,'reason':reason,'inscribed_diameter_mm':e['inscribed_diameter_mm'],'minimum_facet_normal_width_mm':e['minimum_facet_normal_width_mm'],'reference_entity_ids':e['reference_entity_ids'],'source_triangle_ids':e['source_triangle_ids'],'source_regions':e['source_regions'],'reference_exterior_area_by_region_mm2':dict(region_areas),'positive_area_source_faces_by_region':{k:sorted(v) for k,v in owned_faces.items()},'retained_A_B_surface_obligations':sorted(obligations),'retired_C_source_surface_domains':sorted(retired),'independent_thin_entity_must_survive':False,'v2_action':'Repartition into >=0.1mm entities without changing the accepted workface' if tier=='A' else 'Repartition macro boundary into >=0.1mm entities' if tier=='B' else 'Absorb/repartition assembly identity; retain its listed A/B surface and void obligations','whole_entity_surface_waiver':False})
    assert len(thin)==129
    thin_result={'status':'CLASSIFIED','scope':'Each of the frozen 129 subminimum entities, not a new partition. Entity identity tier is distinct from surface fidelity obligations.','baseline_commit':policy['baseline_commit'],'counts_by_tier':dict(collections.Counter(e['classification_tier'] for e in thin)),'counts_by_part_and_tier':{part:dict(collections.Counter(e['classification_tier'] for e in thin if e['source_part']==part)) for part in sorted({e['source_part'] for e in thin})},'entities':thin}
    write('reports/nero-gripper-v2-thin-classification.json',thin_result)
    with (ROOT/'reports/nero-gripper-v2-thin-classification.csv').open('w',newline='') as f:
        fields=['id','name','classification_tier','source_part','semantic_feature','inscribed_diameter_mm','reference_entity_ids','retained_A_B_surface_obligations','v2_action','reason']
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for e in thin:w.writerow({k:';'.join(e[k]) if isinstance(e[k],list) else e[k] for k in fields})

    triangles=[];provenance=[];patch_ledger=[]
    for e in ref['entities']:
        for si,sp in enumerate(e['source_surface_partition']):
            region=face_regions[sp['source_face_id']]
            for pi,p in enumerate(sp['exterior_subpolygons']):
                points=np.array(p['polygon_m'])*1000
                key={'reference_entity_id':e['id'],'source_face_id':sp['source_face_id'],'source_region':region,'surface_partition_index':si,'exterior_polygon_index':pi}
                labels=sorted(set(tiers(region,points)))
                patch_ledger.append(dict(key,tiers_present=labels,classification_rule='source_region_policy/'+region,reference_exterior_area_mm2=p['area_m2']*1e6))
                for i in range(1,len(points)-1):
                    triangles.append(points[[0,i,i+1]]);provenance.append(dict(key,fan_triangle_index=i-1))
    triangles=np.array(triangles);points=np.einsum('wv,tvc->twc',WEIGHTS,triangles).reshape(-1,3)
    mesh,_=exterior(run,True)
    print('Reclassifying saved finite samples',len(points),'against unchanged runtime exterior triangles',len(mesh),flush=True)
    distances,_=TriangleBVH(mesh).query(points)
    assert len(points)==old_errors['reference_exterior_to_runtime_exterior']['sample_count']
    assert abs(float(distances.max())-old_errors['reference_exterior_to_runtime_exterior']['sampled_maximum_mm'])<1e-10
    labels=np.empty(len(points),dtype='<U1')
    for i,meta in enumerate(provenance):labels[7*i:7*i+7]=tiers(meta['source_region'],points[7*i:7*i+7])
    fail_ids=np.flatnonzero(distances>.05+1e-8)
    interior=inside_union(points[fail_ids],run['entities'])
    failures=[];tier_stats={}
    for sid,inside in zip(fail_ids,interior):
        tier=str(labels[sid]);record=dict(provenance[int(sid)//7],legacy_sample_index=int(sid),triangle_sample_index=int(sid)%7,point_mm=points[sid].tolist(),distance_to_v1_extracted_exterior_mm=float(distances[sid]),buried_in_runtime_material=bool(inside),surface_tier=tier,disposition='RETIRE_OLD_SURFACE_TARGET_FOR_V2_ONLY' if tier=='C' else 'RETAIN_AS_TRUE_A_B_FAILURE_OBLIGATION',evidence_level='Reproduced original finite sample; v1 union-exterior mesh remains provisional.')
        failures.append(record)
    # An A/B label cannot be changed to C to hide a failure. Independently
    # distinguish an actual buried target from a hole in the provisional
    # extracted exterior, using ONLY the saved source triangle and halfspaces.
    source=load_source();source_vertices=np.array(source['vertices_m'])*1000
    retained_review=[]
    for record in failures:
        if record['surface_tier']=='C':continue
        p=np.array(record['point_mm']);ids=source['triangles'][record['source_face_id']]['vertex_ids'];t=source_vertices[ids]
        n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n)
        offset=1e-4  # mm; fixed read-only witness, not a model offset
        q=p+n*offset
        separation=[]
        for e in run['entities']:
            eq=np.array(e['halfspaces_m'])*[1,1,1,1000]
            separation.append(float(np.max(eq[:,:3]@q+eq[:,3])))
        minimum_separation=min(separation)
        brute=float(np.sqrt(triangle_distance(p,mesh).min()))
        assert abs(brute-record['distance_to_v1_extracted_exterior_mm'])<1e-9
        outside_proved=minimum_separation>1e-7
        record['actual_union_review']='NOT_A_TRUE_BURIED_FAIL_AT_THIS_SAMPLE' if outside_proved else 'RETAIN_TRUE_A_B_FAILURE_OR_UNKNOWN_UNTIL_UNION_BOUNDARY_VERIFIED'
        retained_review.append({'legacy_sample_index':record['legacy_sample_index'],'source_face_id':record['source_face_id'],'surface_tier':record['surface_tier'],'source_plane_residual_mm':float((p-t[0])@n),'brute_force_extracted_mesh_distance_mm':brute,'source_outward_normal':n.tolist(),'outside_witness_point_mm':q.tolist(),'outside_witness_offset_mm':offset,'minimum_separation_from_all_runtime_hulls_mm':minimum_separation,'actual_union_exterior_distance_upper_mm':offset if outside_proved else None,'conclusion':record['actual_union_review'],'proof':'p belongs to the closed runtime union; q is outside every saved convex hull with the reported positive separation. Segment p-q crosses its true exterior within the offset. No new geometry, hull or engine query is constructed.'})
    for tier in ['A','B','C']:
        mask=labels==tier;fs=[f for f in failures if f['surface_tier']==tier]
        tier_stats[tier]={'sample_count':int(mask.sum()),'sampled_maximum_mm':float(distances[mask].max(initial=0)),'legacy_50um_fail_samples':len(fs),'legacy_fail_reference_entities':len({f['reference_entity_id'] for f in fs}),'legacy_fail_source_faces':sorted({f['source_face_id'] for f in fs}),'v2_target_budget_mm':.01 if tier=='A' else .05 if tier=='B' else None,'observed_v2_target_violations':int(np.count_nonzero(mask & (distances>(.01 if tier=='A' else .05)+1e-8))) if tier!='C' else None}
    buried=read('reports/nero-gripper-runtime-buried-interface.json')
    # Independent historical cube witness: source689 is the backing/veneer
    # assembly seam, explicitly C before any v2 geometry exists.
    assert face_regions[689]=='backing_inner_interface'
    certificate={'reference_source':buried['reference_source'],'source_face_id':689,'source_region':face_regions[689],'surface_tier':'C','old_surface_point_mm':buried['reference_surface_point_mm'],'historical_distance_lower_bound_mm':buried['certified_distance_to_runtime_exterior_lower_bound_mm'],'v1_status':'FAIL_UNCHANGED','v2_disposition':'Retire only this C seam target, not neighbouring A/B sidewalls or protected voids.'}
    local_path=ROOT/'debug/issue25/runtime-partition/backing/buried-reference-exterior-patches.json'
    local=[]
    if local_path.exists():
        for i,p in enumerate(json.loads(local_path.read_text())['patches']):
            n=face_regions[p['source_face_id']];ls=sorted(set(tiers(n,np.array(p['polygon_m'])*1000)))
            local.append({'historical_patch_index':i,'reference_entity_id':p['reference_entity_id'],'source_face_id':p['source_face_id'],'source_region':n,'surface_tiers':ls,'maximum_own_hull_burial_depth_mm':p['max_vertex_or_centroid_depth_in_own_hull_mm'],'interpretation':'Partly buried inventory, not automatically a 50um surface-distance failure; never confuse own-hull depth with union-exterior distance.'})
    result={'status':'CLASSIFIED_REPRODUCED_HISTORICAL_SAMPLES','baseline_commit':policy['baseline_commit'],'reference_exterior_patch_count':len(patch_ledger),'legacy_sample_count':len(points),'legacy_maximum_mm':float(distances.max()),'legacy_50um_failure_samples':len(failures),'all_fail_samples_inside_runtime':bool(interior.all()),'by_tier':tier_stats,'retained_A_B_legacy_sample_review':retained_review,'certified_burial_counterexample':certificate,'failures':failures,'complete_reference_exterior_classification_ledger':patch_ledger,'prior_backing_partial_burial_inventory':local,'scope_limit':'Complete classification atlas covers every saved reference exterior patch, including unsampled interiors via the fixed source-space predicates. Failure enumeration reproduces ALL 81060 original finite samples, not all continuous surface violations. The v1 provisional exterior extraction is not recertified. Missing continuous evidence is not called PASS.','input_sha256':policy['input_sha256'],'classifier_sha256':sha(Path(__file__)),'mujoco_imported':any(n.startswith('mujoco') for n in sys.modules),'new_collision_entities_generated':0}
    assert not result['mujoco_imported']
    write('reports/nero-gripper-v2-buried-classification.json',result)
    print(json.dumps({'thin_counts':thin_result['counts_by_tier'],'by_part':thin_result['counts_by_part_and_tier'],'by_surface_tier':tier_stats,'fail_samples':len(failures),'classified_reference_patches':len(patch_ledger),'prior_partial_burial_inventory':len(local)},indent=2),flush=True)


if __name__=='__main__':main()
