"""Read-only sufficient normal-correspondence counterexample search.

The0.1degree candidate superset is intentionally WIDER than the frozen0.05
acceptance angle. It is used only to falsify acceptance, never to pass it.
"""
from pathlib import Path
import sys,json,hashlib,subprocess,numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from v21_boundary_atlas import support_cells,clip,NUM
from adjudicate_v2_partition import Union
from analyze_runtime_partition import TriangleBVH
from runtime_contact_partition import load_source
from scipy.spatial import ConvexHull

root=ROOT;data=json.loads((root/'reports/nero-gripper-v21-reacceptance.json').read_text())
failed=data['metrics']['A']['source_to_runtime_normal']['failures']
source_A=np.load(root/'debug/issue25/v21-reacceptance/tier-targets.npz')['reference_A']
cells=json.loads((root/'reports/nero-gripper-v2-runtime-partition.json').read_text())['entities']
geometry=hashlib.sha256(json.dumps([[e['id'],e['vertices_m'],e['faces']]for e in cells],separators=(',',':')).encode()).hexdigest()
assert geometry=='79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e'
rules=json.loads(subprocess.check_output(['git','show','8a5518b:reports/nero-gripper-v2-fidelity-protocol.json'],cwd=root))
assert rules['tiers']['A']['normal_deviation_deg']==.05 and rules['tiers']['A']['outward_surface_mm']==.01
source_tiers={}
for name,rule in rules['source_region_policy'].items():
    for a,b in rule['source_triangle_ranges_zero_based_inclusive']:
        for i in range(a,b+1):source_tiers[i]=rule['tier']
bodies,_=support_cells(cells)
oracle=Union([{'id':b['id'],'halfspaces_m':(b['equations']*[1,1,1,.001]).tolist()}for b in bodies],NUM/64)
src=load_source();st=np.array(src['vertices_m'])[np.array([t['vertex_ids']for t in src['triangles']])]*1000
tree=TriangleBVH(st)
tri=[];norm=[];owners=[]
for e in cells:
    vv=np.array(e['vertices_m'])*1000;h=ConvexHull(vv)
    for faces,ns in [(np.array(e['faces']),None),(h.simplices,h.equations[:,:3])]:
        for j,f in enumerate(faces):
            t=vv[f];n=np.cross(np.array(t[1]-t[0],dtype=np.longdouble),np.array(t[2]-t[0],dtype=np.longdouble));n=np.array(n/np.sqrt(np.sum(n*n)),dtype=float)
            if n@(t.mean(0)-vv.mean(0))<0:n=-n
            tri.append(t);norm.append(n);owners.append(e['id'])
            if ns is not None:tri.append(t);norm.append(ns[j]);owners.append(e['id'])
tri=np.array(tri);norm=np.array(norm)
def winding(point,t):
    a,b,c=np.moveaxis(t-point,1,0);la=np.linalg.norm(a,axis=1);lb=np.linalg.norm(b,axis=1);lc=np.linalg.norm(c,axis=1)
    numerator=np.einsum('ij,ij->i',a,np.cross(b,c))
    denominator=la*lb*lc+np.sum(a*b,axis=1)*lc+np.sum(b*c,axis=1)*la+np.sum(c*a,axis=1)*lb
    return float(np.sum(2*np.arctan2(numerator,denominator))/(4*np.pi))
def union_windings(point):return [winding(point,st[a:b])for a,b in[(0,504),(504,660),(660,2086)]]
weights=np.array([[1/3]*3,[.6,.2,.2],[.2,.6,.2],[.2,.2,.6],[.8,.1,.1],[.1,.8,.1],[.1,.1,.8]])
records=[]
for failure in failed:
    index=failure['source_triangle']
    for point in weights@source_A[index]:
        dd,si=tree.query(point[None]);source_face=int(si[0]);t=st[source_face]
        n=np.cross(np.array(t[1]-t[0],dtype=np.longdouble),np.array(t[2]-t[0],dtype=np.longdouble));n=np.array(n/np.sqrt(np.sum(n*n)),dtype=float)
        inside=union_windings(point-1e-5*n);outside=union_windings(point+1e-5*n)
        source_valid=dd[0]<=NUM and any(abs(q-1)<1e-5 for q in inside)and all(abs(q)<1e-5 for q in outside)
        ref=source_A[index];ref_n=np.cross(ref[1]-ref[0],ref[2]-ref[0]);ref_n/=np.linalg.norm(ref_n)
        source_normal_disagreement=float(np.degrees(np.arccos(np.clip(ref_n@n,-1,1))))
        source_valid=source_valid and source_tiers[source_face]=='A' and source_normal_disagreement+.05<.1
        if not source_valid:continue
        ids=np.flatnonzero(norm@n>=np.cos(np.deg2rad(.1))-1e-12)
        buried=[];unproved=[];half=.01+NUM
        for j in ids:
            q=tri[j]
            if np.any(q.max(0)<point-half)or np.any(q.min(0)>point+half):continue
            for k in range(3):
                plane=np.zeros(4);plane[k]=1;plane[3]=-(point[k]+half);q=clip(q,plane)
                if len(q)<3:break
                plane=np.zeros(4);plane[k]=-1;plane[3]=point[k]-half;q=clip(q,plane)
                if len(q)<3:break
            if len(q)<3:continue
            proof=oracle.strict_cover(q/1000)
            if proof and proof['maximum_vertex_plane_residual_mm']<-NUM:
                buried.append({'candidate_facet':int(j),'owner':owners[j],'polygon_mm':q.tolist(),'cover':proof,'normal':norm[j].tolist()})
            else:unproved.append(int(j))
        result={'source_A_reference_triangle':index,'source_stl_face':source_face,'source_point_mm':point.tolist(),'source_normal':n.tolist(),'source_inside_windings':inside,'source_outside_windings':outside,
                'frozen_angle_deg':.05,'conservative_superset_angle_deg':.1,'source_normal_disagreement_deg':source_normal_disagreement,'cube_half_width_mm':half,'compatible_closed_facets_total':len(ids),'intersecting_facets_proved_interior':buried,'intersecting_facets_unproved':unproved,
                'normal_correspondence_failure_confirmed':not unproved,'scope':'All closed candidate facets are a superset of actual union exterior; every compatible facet portion in a cube containing the10um target ball is strictly interior to another convex cell. Finite source-point counterexample, not a whole-union PASS.'}
        records.append(result)
        if result['normal_correspondence_failure_confirmed']:
            print('CERTIFIED source face',source_face,'reference',index,'buried',len(buried),flush=True)
            break
confirmed=[r for r in records if r['normal_correspondence_failure_confirmed']]
for w in confirmed:
    point=np.array(w['source_point_mm'])/1000;n=np.array(w['source_normal'])
    intervals,merged,gaps,local=oracle.line_intervals(point,n,.03/1000)
    containing=[m for m in merged if m[0]<=0<=m[1]]
    if not containing:continue
    end=containing[0][1];q=point+float(end)*n;exit_owners=[]
    for lo,hi,i in intervals:
        if hi!=end:continue
        eq=bodies[i]['equations'];row=int(np.argmin(abs(eq[:,:3]@(q*1000)+eq[:,3])))
        angle=float(np.degrees(np.arccos(np.clip(eq[row,:3]@n,-1,1))))
        exit_owners.append({'entity':bodies[i]['id'],'support_plane':row,'normal':eq[row,:3].tolist(),'normal_deviation_deg':angle})
    states=oracle.membership([q-2*NUM/1000*n,q+2*NUM/1000*n])
    w['direct_source_normal_ray_exit']={'distance_mm':float(end)*1000,'point_mm':(q*1000).tolist(),'owner_facets':exit_owners,'sides_at_2_supports':[s['state']for s in states],'distance_within_10um':bool(float(end)*1000<=.01)}
    # Stronger check: enumerate ALL closed facets in the matching cube,
    # not only the near-normal subset. Bound the entire cone of every
    # potentially exposed facet, so edge-normal combinations are covered too.
    if not exit_owners:continue
    exit_normal=np.array(exit_owners[0]['normal']);possible=[];all_buried=[];half=w['cube_half_width_mm'];center=np.array(w['source_point_mm'])
    for j,t in enumerate(tri):
        if np.any(t.max(0)<center-half)or np.any(t.min(0)>center+half):continue
        poly=t
        for k in range(3):
            plane=np.zeros(4);plane[k]=1;plane[3]=-(center[k]+half);poly=clip(poly,plane)
            if len(poly)<3:break
            plane=np.zeros(4);plane[k]=-1;plane[3]=center[k]-half;poly=clip(poly,plane)
            if len(poly)<3:break
        if len(poly)<3:continue
        proof=oracle.strict_cover(poly/1000)
        if proof and proof['maximum_vertex_plane_residual_mm']<-NUM:
            all_buried.append({'candidate_facet':j,'owner':owners[j],'polygon_mm':poly.tolist(),'cover':proof})
        else:
            spread=float(np.degrees(np.arccos(np.clip(norm[j]@exit_normal,-1,1))))
            possible.append({'candidate_facet':j,'owner':owners[j],'normal':norm[j].tolist(),'angle_from_exit_normal_deg':spread,'polygon_mm':poly.tolist()})
    angular_reserve=1e-5  # degrees: conservative arithmetic uncertainty only;
    # widening the possible-normal cone makes falsification HARDER and never
    # relaxes the frozen0.05degree acceptance limit.
    maximum_spread=max((f['angle_from_exit_normal_deg']for f in possible),default=180.)+angular_reserve
    exit_angle=exit_owners[0]['normal_deviation_deg'];lower=exit_angle-maximum_spread-w['source_normal_disagreement_deg']
    cone_failure=bool(maximum_spread<90 and lower>.05 and w['direct_source_normal_ray_exit']['distance_within_10um']and w['direct_source_normal_ray_exit']['sides_at_2_supports']==['SOLID','VOID'])
    w['local_boundary_normal_cone_certificate']={'all_clipped_facets_proved_buried':all_buried,'remaining_possible_boundary_facets':possible,'angular_arithmetic_reserve_deg':angular_reserve,'maximum_normal_cone_radius_deg':maximum_spread,'normal_deviation_lower_bound_deg':lower,'failure_even_allowing_edge_normal_cones':cone_failure,
                                               'proof':'All union boundary facets are among this closed-facet superset. Strictly interior portions cannot be boundary. Every remaining normal lies in a circular convex cone with the reported radius; positive edge-normal combinations stay in that cone.'}
output={'status':'REAL_TIER_A_NORMAL_CORRESPONDENCE_FAILURE'if confirmed else'NO_SUFFICIENT_COUNTEREXAMPLE_FOUND','confirmed_witnesses':confirmed,'tested_points':len(records),'all_records':records,'candidate_geometry_sha256':geometry,'candidate_geometry_changed':False,'rule_change':False,
        'method':'All closed candidate facets are a superset of the true union exterior. Every facet within the conservative0.1deg normal set is clipped to a cube containing the10um matching ball and proved strictly interior to another unchanged cell. This falsifies0.05deg acceptance without trusting an exterior atlas.',
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
(root/'reports/nero-gripper-v21-normal-witnesses.json').write_text(json.dumps(output,separators=(',',':'))+'\n')
print('Confirmed',len(output['confirmed_witnesses']),'tested',len(records),flush=True)
