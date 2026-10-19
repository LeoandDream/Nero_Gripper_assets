"""Whole-solid offset cover and protected-void checks on the saved v2 only."""
import json
import time
import numpy as np
from build_v2_runtime_partition import ROOT,TASK,frozen,mesh_solid,union,sha
import manifold3d as mf
from runtime_contact_partition import load_source
from runtime_backing_partition import segment_clearance


def critical_workfaces():
    from shapely.ops import unary_union
    from verify_v2_runtime_partition import projected,basis,normal,NUM
    z=np.load(ROOT/'reports/nero-gripper-v2-union-exterior.npz');rt=z['vertices_mm'][z['faces']];rr=z['reference_vertices_mm'][z['reference_faces']];ids=z['reference_source_face_ids'];rn=normal(rt)
    source=load_source();st=np.array(source['vertices_m'])[np.array([t['vertex_ids'] for t in source['triangles']])]*1000
    groups={'main':list(range(557,578)),'tip_short':list(range(545,549)),'tip_front':[592,593,1962,1963,1964,1965],'ramp':list(range(549,557)),'shoulder_bevel':[578,579],'shoulder_blend':[580,581]};results={}
    for name,faces in groups.items():
        t=st[faces[0]];n=np.cross(t[1]-t[0],t[2]-t[0]);n/=np.linalg.norm(n);d=-n@t[0];u,w=basis(n)
        chosen=np.flatnonzero((rn@n>=np.cos(np.deg2rad(.05)))&(np.max(abs(rt@n+d),axis=1)<=.01))
        old=unary_union([projected(q,u,w) for q in rr[np.isin(ids,faces)]]);new=unary_union([projected(q,u,w) for q in rt[chosen]])
        if old.is_empty or new.is_empty:
            results[name]={'status':'UNKNOWN','reason':'No certified coplanar exposed patch'};continue
        # A co-planar island elsewhere in the finger is not the main workface.
        # Keep every connected exposed component intersecting the frozen
        # functional footprint; report unrelated new islands separately, never
        # silently discard them from whole-union validation.
        components=list(new.geoms) if hasattr(new,'geoms') else [new]
        selected=[q for q in components if q.intersection(old).area>1e-12]
        unrelated=[q for q in components if q.intersection(old).area<=1e-12]
        new=unary_union(selected)
        if new.is_empty:
            results[name]={'status':'UNKNOWN','reason':'No source-corresponding connected workface component'};continue
        lo=0.;hi=max(.02,old.boundary.hausdorff_distance(new.boundary)+.01)
        for _ in range(32):
            mid=(lo+hi)/2
            if old.boundary.difference(new.boundary.buffer(mid,quad_segs=64)).is_empty and new.boundary.difference(old.boundary.buffer(mid,quad_segs=64)).is_empty:hi=mid
            else:lo=mid
        angle=float(np.degrees(np.arccos(np.clip(rn[chosen]@n,-1,1))).max(initial=0))
        plane=float(np.max(abs(rt[chosen]@n+d))) if len(chosen) else None
        results[name]={'status':'PASS' if hi+2*NUM<=.01 and angle<=.05 and plane<=.01 else 'FAIL','bidirectional_contour_upper_mm':hi+2*NUM,'maximum_plane_residual_mm':plane,'maximum_exposed_triangle_normal_deviation_deg':angle,'original_area_mm2':old.area,'runtime_corresponding_area_mm2':new.area,'unrelated_coplanar_island_count':len(unrelated),'unrelated_coplanar_island_area_mm2':sum(p.area for p in unrelated),'matching_boundary_triangles':len(chosen),'source_face_ids':faces,'scope':'Actual Boolean-exterior source-corresponding connected workface component. Unrelated new closure islands remain failures/uncertainties of the whole-union certificate, not excluded from it.'}
    return results


def groups(entities):
    return {i:{'poly':{'v':np.array(e['vertices_m'])*1000,'eq':np.array(e['halfspaces_m'])*[1,1,1,1000],'lo':np.min(e['vertices_m'],axis=0)*1000,'hi':np.max(e['vertices_m'],axis=0)*1000}} for i,e in enumerate(entities)}


def point_membership(p,gs):
    return [i for i,g in gs.items() if np.max(g['poly']['eq'][:,:3]@p+g['poly']['eq'][:,3])<=1e-8]


def axis_intervals(x,z,gs):
    result=[]
    for i,g in gs.items():
        eq=g['poly']['eq'];lo=-np.inf;hi=np.inf
        for nx,ny,nz,d in eq:
            b=nx*x+nz*z+d
            if abs(ny)<1e-13:
                if b>1e-8:lo=1;hi=0;break
            elif ny>0:hi=min(hi,-b/ny)
            else:lo=max(lo,-b/ny)
        if lo<=hi:result.append((float(lo),float(hi),i))
    return result


def main():
    start=time.time();candidate=json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text());ref=json.loads((ROOT/'reports/nero-gripper-entity-partition.json').read_text());policy=frozen('reports/nero-gripper-v2-fidelity-protocol.json')
    gs=groups(candidate['entities']);oldgs=groups(ref['entities'])
    body=union([mesh_solid(np.array(e['vertices_m'])*1000,e['faces']) for e in candidate['entities']])
    source=load_source();tri=np.array(source['vertices_m'])[np.array([t['vertex_ids'] for t in source['triangles']])]*1000;parts=[]
    for a,b in [(0,504),(504,660),(660,2086)]:
        flat=tri[a:b].reshape(-1,3);_,ix,inv=np.unique(np.round(flat,12),axis=0,return_index=True,return_inverse=True)
        m=mf.Manifold(mf.Mesh64(np.ascontiguousarray(flat[ix]),np.ascontiguousarray(inv.reshape(-1,3),dtype=np.uint64)));assert m.status()==mf.Error.NoError;parts.append(m)
    source_solid=union(parts)
    inward=source_solid-body
    print('Source material minus v2 status',inward.status(),'empty',inward.is_empty(),flush=True)
    # Exact Minkowski sum with a fixed cube provides a continuous Euclidean
    # offset superset, still strictly INSIDE the frozen50um budget.
    delta=.05-1e-5;a=delta/np.sqrt(3);cube=mf.Manifold.cube((2*a,2*a,2*a),center=True)
    print('Building independent original-STL offset certificate',flush=True)
    expanded=source_solid.minkowski_sum(cube)
    status=expanded.status();outward=body-expanded
    cover_ok=status==mf.Error.NoError and outward.status()==mf.Error.NoError and outward.is_empty()
    # The policy permits <=50um inward material change; an exact-zero result
    # is stronger and is reported separately, never silently assumed from a
    # reference-cell vertex proof in a different Boolean representation.
    body_offset=body.minkowski_sum(cube);reverse=source_solid-body_offset
    inward_ok=body_offset.status()==mf.Error.NoError and reverse.status()==mf.Error.NoError and reverse.is_empty()
    print('Independent STL material cover',cover_ok,'seconds',time.time()-start,flush=True)
    evidence=json.loads((ROOT/'reports/nero-gripper-runtime-evidence.json').read_text())['constructors'];segments=[]
    for f in evidence['backing']['verification.json']['protected_void_witnesses']:
        for p in f['probes']:
            check=segment_clearance(p['a_mm'],p['b_mm'],gs);base=segment_clearance(p['a_mm'],p['b_mm'],oldgs)
            segments.append({'feature':f['feature'],'a_mm':p['a_mm'],'b_mm':p['b_mm'],'reference_clear':base['clear'],'v2':check})
    mount=evidence['mount']['feature-witnesses.json'];points=[]
    for p in mount['witnesses']:
        q=np.array(p['point_m'])*1000;before=point_membership(q,oldgs);after=point_membership(q,gs)
        points.append({'feature':p['feature'],'point_mm':q.tolist(),'expected':p['expected'],'reference_occupants':before,'v2_occupants':after,'unchanged_expected_occupancy':bool(before)==bool(after)})
    floors=[]
    for f in mount['blind_recess_axis_floors']:
        x,z=f['axis_xz_mm'];r=axis_intervals(x,z,oldgs);n=axis_intervals(x,z,gs)
        choose=lambda values:max(b for a,b,i in values if -73.5<b<-70.1)
        old=choose(r);new=choose(n);floors.append({'axis_xz_mm':[x,z],'source_floor_y_mm':old,'v2_floor_y_mm':new,'advance_mm':new-old,'status':'PASS' if abs(new-old)<=.05 else 'FAIL'})
    # Explicitly include the full named central source-part passage, rather
    # than silently reusing a mount-only core as an assembly through-path.
    central=[]
    for x,z in [(0.,12.25),(.8,12.25),(-.8,12.25),(0.,13.05),(0.,11.45)]:
        a0=[x,-72.9999,z];b0=[x,-69.9999,z]
        base=segment_clearance(a0,b0,oldgs);new=segment_clearance(a0,b0,gs)
        central.append({'a_mm':a0,'b_mm':b0,'reference_clear':base['clear'],'v2':new,'new_occlusion':base['clear'] and not new['clear']})
    # All void cores eroded by50um are excluded CONTINUOUSLY by whole-solid
    # cover. The segments and source-axis tests bind this to named topology.
    point_groups={}
    for name in sorted({p['feature'] for p in points}):
        q=[p for p in points if p['feature']==name];point_groups[name]={'points':len(q),'all_source_occupancies_preserved':all(p['unchanged_expected_occupancy'] for p in q)}
    central_source_clear=all(c['reference_clear'] for c in central)
    material={'status':'PASS' if cover_ok and inward_ok else 'FAIL_OR_UNKNOWN','reference_to_v2_material_empty_difference':inward.is_empty(),'unexpanded_source_minus_v2_volume_mm3':inward.volume(),'reference_to_v2_kernel_status':str(inward.status()),'v2_minus_source_offset_empty':outward.is_empty(),'v2_minus_source_offset_kernel_status':str(outward.status()),'source_minus_v2_offset_empty':reverse.is_empty(),'source_minus_v2_offset_status':str(reverse.status()),'source_offset_kernel_status':str(status),'continuous_outward_upper_mm':delta if cover_ok else None,'inward_upper_mm':0. if inward.is_empty() else delta if inward_ok else None,'source_offset_cube_half_width_mm':a,'kernel_tolerance_mm':expanded.get_tolerance(),'budget_mm':.05,'method':'Direct bidirectional Boolean difference against original3-part STL union and fixed cube offsets; no reference-cell progressive-error assumption.'}
    protected={'status':'PASS' if cover_ok and all(p['v2']['clear'] for p in segments if p['reference_clear']) and all(p['unchanged_expected_occupancy'] for p in points) and all(f['status']=='PASS' for f in floors) and central_source_clear and all(c['v2']['clear'] for c in central) else 'BLOCKED_OR_UNKNOWN','continuous_eroded_source_void_core_preservation':cover_ok,'continuous_core_proof':'v2 subset(source plus cube), cube Euclidean radius49.99um <50um; therefore EVERY source void core eroded50um remains empty. If source subsetv2, no new passage can be drilled through an original solid wall.','continuous_segment_checks':segments,'frozen_mount_witness_points':points,'mount_feature_point_summary':point_groups,'blind_recess_floors':floors,'central_bore_assembled_through_checks':central,'central_bore_source_already_occluded':not central_source_clear,'side_channel_rule':'Source150um slit eroded50um retains50um connected core; explicit frozen channel witnesses included above. No inference from small entity size.','full_named_connectivity_status':'PASS via continuous core + source corridor witnesses' if cover_ok and central_source_clear else 'UNKNOWN_OR_FROZEN_SOURCE_THROUGH_PATH_CONFLICT'}
    result={'geometry_sha256':candidate['geometry_sha256'],'material':material,'protected_voids':protected,'critical_workfaces':critical_workfaces(),'scope':'One saved candidate. Original-STL Boolean operations and analytic convex halfspaces only. No physics.','verifier_sha256':sha(__file__),'elapsed_seconds':time.time()-start}
    (ROOT/'reports/nero-gripper-v2-material-voids.json').write_text(json.dumps(result,separators=(',',':'),allow_nan=False)+'\n')
    print(json.dumps({'material':material,'protected_void_status':protected['status'],'central_source_already_occluded':protected['central_bore_source_already_occluded'],'floors':floors,'named_mount_features':point_groups},indent=2),flush=True)


if __name__=='__main__':main()
