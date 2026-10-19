"""Issue25 bounded, one-candidate local clipping of A-sidewall coverers.

Preflight derives constraints from ALL frozen source sidewall patches; build
may execute once. No simulation or policy changes. Units of analysis: mm.
"""
from pathlib import Path
import argparse,collections,copy,hashlib,json,subprocess,sys,time
import numpy as np
from scipy.spatial import ConvexHull
from runtime_contact_partition import load_source
from runtime_backing_partition import poly,clip_body,outward_faces,inradius
from v21_boundary_atlas import support_cells,clip,NUM

ROOT=Path(__file__).resolve().parents[1]
TASK=ROOT/'debug/issue25/v22-local'
BASE=ROOT/'reports/nero-gripper-v2-runtime-partition.json'
OUT=ROOT/'reports/nero-gripper-v22-local-partition.json'
REGIONS={'thin_layer_left_sidewall','thin_layer_right_sidewall','thin_layer_root_edges_and_bevels','backing_outer_left_sidewall','backing_outer_right_sidewall'}
ALLOWED={'sidewall_negative_x','sidewall_positive_x','outer_chassis'}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,o):Path(p).write_text(json.dumps(o,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def geosha(cells):return hashlib.sha256(json.dumps([[e['id'],e['vertices_m'],e['faces']]for e in cells],separators=(',',':')).encode()).hexdigest()
def patches():
    source=load_source();sv=np.array(source['vertices_m'])*1000;st=sv[np.array([t['vertex_ids']for t in source['triangles']])]
    n=np.cross(st[:,1]-st[:,0],st[:,2]-st[:,0]);n/=np.linalg.norm(n,axis=1)[:,None]
    p=json.loads(subprocess.check_output(['git','show','8a5518b:reports/nero-gripper-v2-fidelity-protocol.json'],cwd=ROOT))
    face_region={i:name for name,r in p['source_region_policy'].items()if name in REGIONS for a,b in r['source_triangle_ranges_zero_based_inclusive']for i in range(a,b+1)}
    result=[]
    for e in json.loads((ROOT/'reports/nero-gripper-entity-partition.json').read_text())['entities']:
        for sp in e['source_surface_partition']:
            f=sp['source_face_id']
            if f not in face_region:continue
            for j,q in enumerate(sp['exterior_subpolygons']):
                v=np.array(q['polygon_m'])*1000
                if len(v)<3:continue
                result.append({'id':f"{e['id']}:{sp['source_patch_index']}:{j}",'reference_owner':e['id'],'source_face':f,'region':face_region[f],'polygon_mm':v.tolist(),'plane':np.r_[n[f],-n[f]@st[f,0]].tolist()})
    return result,st
def preflight():
    TASK.mkdir(parents=True,exist_ok=True)
    cells=json.loads(BASE.read_text())['entities'];assert geosha(cells)=='79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e'
    bodies,checks=support_cells(cells);pp,st=patches();constraints=collections.defaultdict(dict);hits=[];forbidden=[]
    for patch in pp:
        q=np.array(patch['polygon_mm']);plane=np.array(patch['plane']);n=plane[:3]
        for i,b in enumerate(bodies):
            probe=q+2*NUM*n
            if np.any(b['hi']<probe.min(0))or np.any(b['lo']>probe.max(0)):continue
            cut=probe.copy()
            for eq in b['equations']:
                cut=clip(cut,eq)
                if len(cut)<3:break
            if len(cut)<3:continue
            area=float(np.linalg.norm(np.cross(cut[1:]-cut[0],np.roll(cut,-1,axis=0)[1:]-cut[0]).sum(0))/2)
            if area<=1e-16:continue
            row={'patch':patch['id'],'source_face':patch['source_face'],'entity':b['id'],'semantic_family':cells[i]['semantic_family'],'outward_probe_overlap_area_mm2':area}
            hits.append(row)
            if cells[i]['semantic_family']not in ALLOWED:
                forbidden.append(row)
            else:
                key=str(patch['source_face']);constraints[b['id']][key]=patch['plane']
    bounds=[]
    for e in cells:
        if e['id']not in constraints:continue
        normals=np.array(list(constraints[e['id']].values()))
        v=np.array(e['vertices_m'])*1000
        bounds.append({'entity':e['id'],'family':e['semantic_family'],'source_plane_count':len(normals),'maximum_old_vertex_outside_new_planes_mm':float(np.max(v@normals[:,:3].T+normals[:,3]))})
    old=json.loads((ROOT/'reports/nero-gripper-v21-rechecks.json').read_text())
    prereg={'task':'https://github.com/LeoandDream/Nero_Gripper_assets/issues/25#issuecomment-6273457243','baseline_commit':'642d3b020a01e800dbad31b32f5e7abf0620550f','candidate_commit':'6eba4f34c5ff5029eb849804e3d2e9c535f58756','baseline_geometry_sha256':geosha(cells),'baseline_file_sha256':sha(BASE),'v21_policy_sha256':{str(p.relative_to(ROOT)):sha(p)for p in [ROOT/'reports/nero-gripper-v21-fidelity-protocol.json',ROOT/'reports/nero-gripper-v21-fidelity-protocol.md',ROOT/'reports/nero-gripper-v2-fidelity-protocol.json']},'candidate_limit':1,'hypothesis':'Intersection of a directly covering sidewall/outer_chassis cell with the frozen A sidewall source support halfspaces restores exact exposed A planes without moving source workfaces. No global merges or compression. If this single bounded clipping changes unrelated source functions or cannot meet scale/fidelity, stop BLOCKED.','construction_rule':'For every frozen reference exterior A-sidewall polygon, intersect its +2NUM normal translate with each baseline convex cell. Positive-area overlap identifies a coverer. Apply those exact source plane halfspaces to that cell in sorted source-face order, retaining the inward halfspace. Do not apply a plane where no source-patch overlap was found. Rebuild only those cells; preserve all other JSON entity objects unchanged. No snapping/joggle/offset of source planes.','numeric_support_mm':NUM,'minimum_overlap_area_numerical_resolution_mm2':1e-16,'allowed_families':sorted(ALLOWED),'prohibited':'main/tip/ramp/shoulder/mount_root/unrelated entities, policy, global redesign, MJCF, MuJoCo, physics parameters','new_hard_rule':'Replacement union exterior must simultaneously meet A surface10um, contour10um and normal0.05deg. Burial depth alone never passes.','acceptance':{'A_surface_contour_mm':.01,'A_normal_deg':.05,'minimum_inscribed_diameter_mm':.1,'B_surface_contour_mm':.05,'C_and_void':'inherit frozen v2.1; no relaxation','mandatory_witness_count':11,'all_A_sidewall_patches':True},'sidewall_patch_count':len(pp),'source_face_count':len({p['source_face']for p in pp}),'patches':pp,'constraints_by_entity':constraints,'preflight_bounds':bounds,'covering_patch_intersections':hits,'forbidden_coverers':forbidden,'old10_unresolved':old['old14_remaining_unresolved'],'old10_details':[r for r in old['old14_rechecks']if 'UNRESOLVED'in str(r)],'mandatory_regression_sha256':sha(ROOT/'reports/nero-gripper-v21-normal-witnesses.json'),'builder_sha256':sha(__file__),'preserve_baseline_reports':True}
    save(ROOT/'reports/nero-gripper-v22-local-preregistration.json',prereg)
    print(json.dumps({'source_patches':len(pp),'source_faces':prereg['source_face_count'],'coverer_entities':len(constraints),'forbidden_coverers':len(forbidden),'bounds':bounds},indent=2))
def build():
    if OUT.exists():raise SystemExit('One v22 candidate already exists; no regeneration authorized.')
    pre=json.loads((ROOT/'reports/nero-gripper-v22-local-preregistration.json').read_text())
    assert pre['builder_sha256']==sha(__file__)
    assert sha(BASE)==pre['baseline_file_sha256']
    for p,h in pre['v21_policy_sha256'].items():assert sha(ROOT/p)==h
    cells=json.loads(BASE.read_text())['entities'];out=[];mapping=[]
    for e in cells:
        constraints=pre['constraints_by_entity'].get(e['id'])
        if not constraints:out.append(copy.deepcopy(e));mapping.append({'old':e['id'],'new':[e['id']],'changed':False});continue
        p=poly(np.array(e['vertices_m'])*1000)
        for f,plane in sorted(constraints.items(),key=lambda kv:int(kv[0])):
            p=clip_body(p,np.array(plane))
            if p is None:raise RuntimeError('Local inward intersection removed entire entity '+e['id'])
        item=copy.deepcopy(e);v=p['v'];f=outward_faces(p);d,centre=inradius(p)
        item.update(vertices_m=(v/1000).tolist(),faces=f,halfspaces_m=(p['eq']*[1,1,1,.001]).tolist(),inscribed_diameter_mm=2*d,inscribed_centre_m=(centre/1000).tolist(),volume_mm3=float(p['h'].volume),v22_clipping_source_faces=sorted(map(int,constraints)),v22_baseline_entity=e['id'],exterior_certification_state='PENDING_V22_REACCEPTANCE')
        # Baseline metric and atlas fields are provenance only, not v22 claims.
        for k in ['actual_union_exterior_triangle_indices','minimum_facet_normal_width_mm','material_cover_certificate']:item.pop(k,None)
        out.append(item);mapping.append({'old':e['id'],'new':[e['id']],'changed':True,'source_planes':sorted(map(int,constraints)),'old_geometry_sha256':geosha([e]),'new_geometry_sha256':geosha([item]),'old_volume_mm3':e['volume_mm3'],'new_volume_mm3':item['volume_mm3'],'new_inscribed_diameter_mm':2*d})
    result={'schema_version':'2.2-local','status':'UNVERIFIED_SINGLE_CANDIDATE','baseline':pre['baseline_commit'],'baseline_geometry_sha256':pre['baseline_geometry_sha256'],'geometry_sha256':geosha(out),'entity_count_per_finger':len(out),'counts_by_semantic_family':dict(collections.Counter(e['semantic_family']for e in out)),'entities':out,'old_to_new':mapping,'modified_entity_ids':[m['old']for m in mapping if m['changed']],'added_entities':[],'deleted_entities':[],'generation_count':1,'preregistration_sha256':sha(ROOT/'reports/nero-gripper-v22-local-preregistration.json'),'builder_sha256':sha(__file__),'scope':'Only local support-halfspace intersections on directly covering allowed cells. No global redesign/merge. Old files unchanged.'}
    save(OUT,result);save(ROOT/'reports/nero-gripper-v22-local-mapping.json',mapping)
    print(json.dumps({k:result[k]for k in ['geometry_sha256','entity_count_per_finger','modified_entity_ids']},indent=2))
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['preflight','build']);args=parser.parse_args()
    preflight()if args.stage=='preflight'else build()
