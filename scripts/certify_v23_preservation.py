"""Continuous, independent split-tree/material certificates; no geometry writes.

The ideal finite-prism partition identity and the saved-mesh realization are
different claims. Exact-rational dual bounds are used only when the expanded
cover mesh itself has an exact closed-convex certificate; otherwise UNKNOWN.
"""
from fractions import Fraction as F
import collections
import gzip
import json
import time

import numpy as np
from scipy.optimize import linprog

from v23_finite_patch import ROOT,TASK,BASE,OUT,PRE,NUM,EPS,body,sha,save

FILE=ROOT/'reports/nero-gripper-v23-preservation.json.gz'


def rational_dot(a,b):
    return sum(x*y for x,y in zip(a,b))


def exact_expanded_cover(entity):
    delta=NUM/4/np.sqrt(3)
    corners=np.array([[a,b,c]for a in [-delta,delta]for b in [-delta,delta]for c in [-delta,delta]])
    vertices=(np.array(entity['vertices_m'])*1000)[:,None]+corners[None]
    p=body(vertices.reshape(-1,3))
    if p is None:return None,{'status':'UNKNOWN','reason':'Expanded hull cannot be represented reliably'}
    v=[[F(float(x))for x in q]for q in p['v']];center=[sum(q[k]for q in v)/len(v)for k in range(3)];rows=[];unique=set();edgecount=collections.Counter()
    for face in p['faces']:
        a,b,c=[v[i]for i in face];ab=[y-x for x,y in zip(a,b)];ac=[y-x for x,y in zip(a,c)]
        n=[ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0]]
        if not any(n):return None,{'status':'UNKNOWN','reason':'Exact degenerate expanded-cover triangle'}
        if rational_dot(n,[center[k]-a[k]for k in range(3)])>0:n=[-x for x in n]
        d=-rational_dot(n,a);scale=max(map(abs,n));row=[x/scale for x in n]+[d/scale]
        if any(rational_dot(row[:3],q)+row[3]>0 for q in v):
            return None,{'status':'UNKNOWN','reason':'Floating expanded triangulation is not an exact supporting mesh; no unconditional saved-geometry claim'}
        key=tuple(row)
        if key not in unique:rows.append(row);unique.add(key)
        for x,y in zip(face,face[1:]+face[:1]):edgecount[tuple(sorted((x,y)))]+=1
    if set(edgecount.values())!={2}:return None,{'status':'UNKNOWN','reason':'Expanded-cover topology not closed'}
    return rows,{'status':'PASS_EXACT_CONVEX_COVER','vertices_mm':p['v'].tolist(),'faces':p['faces'],'exact_support_rows':[[str(x)for x in r]for r in rows],'distance_to_saved_convex_material_upper_mm':NUM/4+1e-12,'proof':'A closed mesh with every face an exact supporting face is the convex hull of its vertices; each expanded vertex is one saved cell vertex plus a bounded cube offset (with floating-addition reserve).'}


def bounds_for(root):
    v=np.array(root['vertices_m'])*1000
    return v.min(0)-EPS,v.max(0)+EPS


def outer_root_rows(root,rows):
    v=np.array(root['vertices_m'])*1000;result=[]
    for row in rows:
        a=[F(float(x))for x in row[:3]];maximum=max(rational_dot(a,[F(float(x))for x in p])for p in v)
        d=np.nextafter(float(-maximum-F(float(EPS))),-np.inf)
        assert all(rational_dot(a,[F(float(x))for x in p])+F(float(d))<=0 for p in v)
        result.append([*row[:3],float(d)])
    lo,hi=bounds_for(root)
    for k in range(3):
        a=np.zeros(4);a[k]=1;a[3]=-hi[k];result.append(a.tolist());a=-a;a[3]=lo[k];result.append(a.tolist())
    return result


def maximum_certificate(region,target,lo,hi):
    rows=np.array(region);a,b=rows[:,:3],rows[:,3];c=np.array([float(x)for x in target[:3]])
    center=(lo+hi)/2;scale=np.maximum((hi-lo)/2,1e-3);gain=1e6
    lp=linprog(-c*scale,A_ub=a*scale*gain,b_ub=-(a@center+b)*gain,bounds=[(None,None)]*3,method='highs',options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
    if not lp.success:return None
    dual=np.maximum(-lp.ineqlin.marginals*gain,0);ids=np.flatnonzero(dual>0);y=[F(float(dual[i]))for i in ids]
    aa=[[F(float(x))for x in rows[i,:3]]for i in ids];bb=[F(float(rows[i,3]))for i in ids]
    remainder=[target[k]-sum(w*q[k]for w,q in zip(y,aa))for k in range(3)]
    upper=target[3]-sum(w*d for w,d in zip(y,bb))+sum(max(q*F(float(l)),q*F(float(h)))for q,l,h in zip(remainder,lo,hi))
    if upper>0:return None
    return {'constraint_ids':ids.tolist(),'nonnegative_dual_weights':[str(w)for w in y],'exact_objective_row':[str(q)for q in target],'exact_upper_bound':str(upper),'upper_bound_float':float(upper)}


def infeasible_certificate(region,lo,hi):
    rows=np.array(region);a,b=rows[:,:3],rows[:,3];center=(lo+hi)/2
    lp=linprog(-(a@center+b),A_eq=np.vstack([a.T,np.ones(len(rows))]),b_eq=[0,0,0,1],bounds=(0,None),method='highs')
    if not lp.success:return None
    y0=np.maximum(lp.x,0);ids=np.flatnonzero(y0>0);y=[F(float(y0[i]))for i in ids]
    r=[sum(w*F(float(rows[i,k]))for w,i in zip(y,ids))for k in range(3)]
    constant=sum(w*F(float(rows[i,3]))for w,i in zip(y,ids))
    lower=constant+sum(min(q*F(float(l)),q*F(float(h)))for q,l,h in zip(r,lo,hi))
    if lower<=0:return None
    return {'constraint_ids':ids.tolist(),'nonnegative_dual_weights':[str(w)for w in y],'exact_positive_contradiction_lower_bound':str(lower),'proof':'A nonnegative sum of feasible inequalities cannot be strictly positive anywhere in the enclosing root bbox.'}


def vertex_subset(root,successors):
    old=np.array(root['vertices_m'])*1000;rows=[]
    for e in successors:
        certs=[];bad=[]
        for point in np.array(e['vertices_m'])*1000:
            lp=linprog(np.zeros(len(old)),A_eq=np.vstack([old.T,np.ones(len(old))]),b_eq=np.r_[point,1],bounds=(0,None),method='highs',options={'primal_feasibility_tolerance':1e-9})
            if not lp.success:bad.append({'point_mm':point.tolist(),'reason':'No reliable nonnegative combination found'});continue
            ids=np.flatnonzero(lp.x>0);weights=[F(float(lp.x[i]))for i in ids];total=sum(weights);weights=[w/total for w in weights]
            residual=[F(float(point[k]))-sum(w*F(float(old[i,k]))for w,i in zip(weights,ids))for k in range(3)];upper=float(np.linalg.norm([float(x)for x in residual]))+1e-12
            item={'point_mm':point.tolist(),'old_vertex_indices':ids.tolist(),'exact_positive_weights':[str(w)for w in weights],'distance_upper_mm':upper}
            (certs if upper<=NUM/4 else bad).append(item)
        rows.append({'entity':e['id'],'status':'PASS'if not bad else'UNKNOWN','certificates':certs,'unresolved':bad})
    return rows


def main():
    started=time.time();pre=json.loads(PRE.read_text());data=json.loads(OUT.read_text());before=sha(OUT)
    trace_raw=gzip.decompress((ROOT/'reports/nero-gripper-v23-split-trace.json.gz').read_bytes());trace=json.loads(trace_raw)
    roots={e['id']:e for e in json.loads(BASE.read_text())['entities']};prisms={p['id']:p for p in pre['influence_prisms']};output=[]
    for tree in trace['roots']:
        rid=tree['baseline_entity'];root=roots[rid];nodes=tree['nodes'];m=len(tree['root_support_halfspaces_mm']);outer=outer_root_rows(root,tree['root_support_halfspaces_mm']);lo,hi=bounds_for(root)
        successors=[e for e in data['entities']if e.get('baseline_entity')==rid];targets={};target_proofs={}
        leaf_owner={n:e['id']for e in successors for n in e['retained_split_leaf_nodes']}
        for e in successors:
            targets[e['id']],target_proofs[e['id']]=exact_expanded_cover(e)
        identities=[];obligations=[]
        for node in nodes:
            if 'children'in node:
                assert len(node['children'])==2
                plus,minus=[nodes[i]for i in node['children']];row=node['cut_plane_mm']
                assert plus['parent']==minus['parent']==node['id']
                assert plus['region_halfspaces_mm'][:-1]==minus['region_halfspaces_mm'][:-1]==node['region_halfspaces_mm']
                assert plus['region_halfspaces_mm'][-1]==row and minus['region_halfspaces_mm'][-1]==(-np.array(row)).tolist()
                continue
            region=outer+node['region_halfspaces_mm'][m:]
            if node['role']=='DISCARD_FINITE_OUTWARD_CAP':
                prism=prisms[node['prism']];assert all(q in node['region_halfspaces_mm']for q in prism['influence_halfspaces_mm'])
                identities.append({'node':node['id'],'status':'PASS_DISCARD_SUBSET_OF_REGISTERED_PRISM','prism':prism['id']});continue
            eid=leaf_owner.get(node['id']);choices=[eid]if eid else [e['id']for e in successors]
            feasible=None;cert=None;covered=None
            if eid is None:
                cert=infeasible_certificate(region,lo,hi)
                if cert:
                    obligations.append({'node':node['id'],'status':'PASS_EXACT_INFEASIBLE','certificate':cert});continue
                ar=np.array(region);feasible=linprog([0,0,0],A_ub=ar[:,:3],b_ub=-ar[:,3],bounds=list(zip(lo,hi)),method='highs')
                if feasible.success:
                    choices=sorted(choices,key=lambda k:np.linalg.norm(np.array(next(e for e in successors if e['id']==k)['vertices_m']).mean(0)*1000-feasible.x))[:3]
                else:choices=choices[:3]
            for candidate in choices:
                if targets[candidate]is None:continue
                proofs=[]
                for target in targets[candidate]:
                    proof=maximum_certificate(region,target,lo,hi)
                    if proof is None:break
                    proofs.append(proof)
                if len(proofs)==len(targets[candidate]):covered={'entity':candidate,'facet_dual_certificates':proofs};break
            obligations.append({'node':node['id'],'status':'PASS_CONTINUOUS_SAVED_CONVEX_COVER'if covered else'UNKNOWN','cover':covered,'reason':None if covered else'Could not prove this entire terminal region covered by one exact certified saved-cell expansion; no sampled PASS inferred.'})
        subset=vertex_subset(root,successors);counts=dict(collections.Counter(o['status']for o in obligations))
        output.append({'baseline_entity':rid,'logical_partition_identity':'PASS','discarded_leaf_prism_certificates':identities,'expanded_cover_geometry_certificates':target_proofs,'terminal_material_obligations':obligations,'new_material_subset_certificates':subset,'terminal_counts':counts,'saved_implementation_status':'PASS_MODULO_NUM'if all(o['status']!='UNKNOWN'for o in obligations)and all(s['status']=='PASS'for s in subset)else'UNKNOWN'})
        checkpoint={'status':'PENDING','roots':output,'preregistration_sha256':sha(PRE),'candidate_geometry_sha256':data['geometry_sha256']}
        FILE.write_bytes(gzip.compress(json.dumps(checkpoint,separators=(',',':')).encode(),mtime=0))
        print('outside root',rid,'terminals',len(obligations),'statuses',counts,'exact cover meshes',sum(v is not None for v in targets.values()),'/',len(targets),flush=True)
    result={'status':'PASS_MODULO_NUM'if all(q['saved_implementation_status']=='PASS_MODULO_NUM'for q in output)else'UNKNOWN','ideal_finite_prism_partition_identity':'PASS','actual_saved_geometry_outside_preservation':'PASS_MODULO_NUM'if all(q['saved_implementation_status']=='PASS_MODULO_NUM'for q in output)else'UNKNOWN','candidate_geometry_sha256':data['geometry_sha256'],'candidate_file_sha256':before,'candidate_not_modified':sha(OUT)==before,'preregistration_sha256':sha(PRE),'verifier_sha256':sha(__file__),'roots':output,'method':'Exact binary halfspace partition; discarded leaves explicitly inside registered finite prisms. All retained/empty/numeric terminal regions are obligations. Saved realization is claimed only with exact convex cover mesh and exact-rational LP dual upper bounds. Vertex-positive-combination certificates exclude new material beyond NUM. Unproved cases remain UNKNOWN.','elapsed_seconds':time.time()-started}
    FILE.write_bytes(gzip.compress(json.dumps(result,separators=(',',':')).encode(),mtime=0))
    print(json.dumps({'status':result['status'],'root_status_counts':dict(collections.Counter(q['saved_implementation_status']for q in output)),'seconds':result['elapsed_seconds']},indent=2),flush=True)


if __name__=='__main__':main()
