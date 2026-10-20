"""Issue25 cross-root feasibility; analytical sets only, no runtime geometry.

One-hop neighborhoods come solely from frozen reference-adjacency edges.
Every one of the 104 original finite caps is checked, not only root-owned caps.
Search failure never proves infeasibility. Old root-local proofs are not reused
as cross-root verdicts. No v24 evidence is overwritten.
"""
from fractions import Fraction as F
from collections import Counter
from itertools import product,combinations
from pathlib import Path
import argparse,gzip,json,math,time
import numpy as np
from scipy.optimize import milp,Bounds,LinearConstraint
from analyze_v24_convex_covers import exact_root,sphere,sha,dot
from certify_v23_preservation import maximum_certificate,infeasible_certificate

ROOT=Path(__file__).resolve().parents[1]
NUM=F(float(1e-7));H=0.050001;ROOT_EXPAND=1e-12;CAP_EPS=1e-10
PRE=ROOT/'reports/nero-gripper-v25-cross-root-preregistration.json'
OUT=ROOT/'reports/nero-gripper-v25-cross-root.json.gz'


def solve_square(a,b):
    n=len(b);m=[list(row)+[y]for row,y in zip(a,b)]
    for k in range(n):
        pivot=next((i for i in range(k,n)if m[i][k]),None)
        if pivot is None:return None
        m[k],m[pivot]=m[pivot],m[k];v=m[k][k];m[k]=[x/v for x in m[k]]
        for i in range(n):
            if i!=k:
                v=m[i][k];m[i]=[x-v*y for x,y in zip(m[i],m[k])]
    return [r[-1]for r in m]


def closest_cap(point,rows):
    values=[dot(row[:3],point)+row[3]for row in rows]
    if max(values)<=0:return {'inside':True,'distance_squared_exact':'0','closest_point_exact':[str(x)for x in point],'active_rows':[],'nonnegative_KKT_weights':[]}
    order=sorted(range(len(rows)),key=lambda i:values[i],reverse=True)
    for size in (1,2,3):
        for ids in combinations(order,size):
            aa=[rows[i][:3]for i in ids];gram=[[dot(a,b)for b in aa]for a in aa];lam=solve_square(gram,[values[i]for i in ids])
            if lam is None or min(lam)<0:continue
            q=[point[k]-sum(w*a[k]for w,a in zip(lam,aa))for k in range(3)]
            if any(dot(row[:3],q)+row[3]>0 for row in rows):continue
            d2=sum((a-b)**2 for a,b in zip(q,point))
            return {'inside':False,'distance_squared_exact':str(d2),'closest_point_exact':[str(x)for x in q],'active_rows':list(ids),'nonnegative_KKT_weights':[str(x)for x in lam]}
    raise RuntimeError('No exact closest-cap KKT basis found')


def cap_margins(point,caps):
    lower=[]
    for cap in caps:
        rows=[[F(float(x))for x in r]for r in cap['carve_halfspaces_mm']];vals=[dot(r[:3],point)+r[3]for r in rows]
        options=[(x*x/dot(r[:3],r[:3]),i)for i,(r,x)in enumerate(zip(rows,vals))if x>0]
        lb,i=max(options)if options else(F(0),-1)
        lower.append((lb,cap,rows,i))
    best=None;certs=[]
    for lb,cap,rows,i in sorted(lower,key=lambda x:x[0]):
        if best is not None and lb>=best:continue
        cert=closest_cap(point,rows);certs.append({'cap':cap['id'],**cert});d2=F(cert['distance_squared_exact'])
        best=d2 if best is None else min(best,d2)
    assert best is not None
    return {'minimum_distance_squared_exact':str(best),'minimum_distance_mm':math.sqrt(float(best)),'minimum_distance_NUM':math.sqrt(float(best))/float(NUM),'robust_gt_NUM':best>NUM*NUM,'closest_cap_ids':[q['cap']for q in certs if F(q['distance_squared_exact'])==best],'exact_projection_certificates':certs,'all_cap_lower_bounds':[{'cap':c['id'],'facet':i,'distance_squared_lower_exact':str(lb)}for lb,c,rows,i in lower]}


def ray_exit(point,direction,ids,proofs):
    intervals=[]
    for rid in ids:
        low=F(0);high=None;valid=True
        for q in proofs[rid]['exact_support_rows']:
            row=list(map(F,q));v=dot(row[:3],point)+row[3];s=dot(row[:3],direction)
            if s==0:
                if v>0:valid=False;break
            elif s>0:high=-v/s if high is None else min(high,-v/s)
            else:low=max(low,-v/s)
        if valid and high is not None and high>=low:intervals.append((low,high,rid))
    intervals.sort();end=None
    for low,high,rid in intervals:
        if end is None:
            if low>0:break
            end=high
        elif low<=end:end=max(end,high)
        else:break
    if end is None:return None
    return {'direction_exact':[str(x)for x in direction],'exit_t_exact':str(end),'distance_squared_exact':str(end*end*dot(direction,direction)),'all_root_ray_intervals':[{'root':rid,'lo':str(lo),'hi':str(hi)}for lo,hi,rid in intervals]}


def neighborhood_margins(point,ids,proofs):
    lower=F(0);owner=None;face=None;directions=[]
    for rid in ids:
        rows=[list(map(F,q))for q in proofs[rid]['exact_support_rows']];vals=[dot(r[:3],point)+r[3]for r in rows]
        if max(vals)<=0:
            distance,index=min((v*v/dot(r[:3],r[:3]),i)for i,(r,v)in enumerate(zip(rows,vals)))
            if distance>=lower:lower=distance;owner=rid;face=index
        directions.extend(tuple(r[:3])for r in rows)
    candidates=[]
    for d in set(directions)|{(F(a),F(b),F(c))for a,b,c in product((-1,0,1),repeat=3)if (a,b,c)!=(0,0,0)}:
        q=ray_exit(point,d,ids,proofs)
        if q:candidates.append(q)
    assert candidates
    best=min(candidates,key=lambda x:F(x['distance_squared_exact']));upper=F(best['distance_squared_exact']);assert upper>=lower
    return {'minimum_distance_squared_lower_exact':str(lower),'minimum_distance_squared_upper_exact':str(upper),'minimum_distance_interval_mm':[math.sqrt(float(lower)),math.sqrt(float(upper))],'minimum_distance_interval_NUM':[math.sqrt(float(lower))/float(NUM),math.sqrt(float(upper))/float(NUM)],'robust_gt_NUM':lower>NUM*NUM,'entirely_within_NUM_band':upper<=NUM*NUM,'lower_ball_root':owner,'lower_ball_root_facet':face,'upper_ray_certificate':best,'interpretation':'Certified interval for actual distance to the complement of the root UNION, not a reported internal root face. Lower bound is a whole ball in one member; upper bound is the exact first void on a ray through the entire union. Equal squared endpoints give the exact minimum.'}


def search_center(point,seed,proof,caps):
    p=np.array([float(x)for x in point]);q=np.array(proof['inner']);lo,hi=np.array(proof['bbox_mm']);entries=[]
    for j,cap in enumerate(caps):
        options=[]
        for i,row in enumerate(cap['carve_halfspaces_mm']):
            rr=[F(float(x))for x in row]
            if dot(rr[:3],point)+rr[3]>=0:options.append(i)
        if not options:return None,{'reason':'Required point lies in a frozen cap','cap':cap['id']}
        entries.extend((j,i)for i in options)
    n=6+len(entries);rows=[];lb=[];ub=[]
    for row in q:
        v=np.zeros(n);v[:3]=row[:3];rows.append(v);lb.append(-np.inf);ub.append(-row[3]-H*sum(abs(row[:3]))-1e-10)
    for k,(j,i)in enumerate(entries):
        row=np.array(caps[j]['carve_halfspaces_mm'][i]);a=-row[:3];M=max(1e-3,np.sum(np.maximum(a*lo,a*hi))-row[3]+H*sum(abs(a))+1e-7)
        v=np.zeros(n);v[:3]=a;v[6+k]=M;rows.append(v);lb.append(-np.inf);ub.append(M+row[3]-H*sum(abs(a))-1e-9)
    for j in range(len(caps)):
        v=np.zeros(n)
        for k,(jj,i)in enumerate(entries):
            if jj==j:v[6+k]=1
        rows.append(v);lb.append(1);ub.append(np.inf)
    for k in range(3):
        v=np.zeros(n);v[k]=1;v[3+k]=-1;rows.append(v);lb.append(-np.inf);ub.append(p[k]);v=v.copy();v[k]=-1;rows.append(v);lb.append(-np.inf);ub.append(-p[k])
    obj=np.r_[np.zeros(3),np.ones(3),np.zeros(n-6)]
    res=milp(obj,integrality=[0]*6+[1]*(n-6),bounds=Bounds([*lo,0,0,0]+[0]*(n-6),[*hi,np.inf,np.inf,np.inf]+[1]*(n-6)),constraints=LinearConstraint(rows,lb,ub),options={'time_limit':3,'mip_rel_gap':.01})
    if res.x is None:return None,{'reason':'No cube-center incumbent; not an infeasibility proof','milp_status':int(res.status)}
    return res.x[:3],None


def analytical_hull(point,center):
    vertices=np.array([[float(x)for x in point]]+[list(center+np.array(c)*H)for c in product((-1,1),repeat=3)])
    raw=exact_root({'vertices_m':(vertices/1000).tolist()});rows=np.array(raw['outer']);rows[:,3]-=1e-12
    lo=vertices.min(0)-2e-12;hi=vertices.max(0)+2e-12
    for k in range(3):
        a=np.zeros(4);a[k]=1;a[3]=-hi[k];rows=np.vstack([rows,a]);a=np.zeros(4);a[k]=-1;a[3]=lo[k];rows=np.vstack([rows,a])
    assert all(dot([F(float(x))for x in r[:3]],point)+F(float(r[3]))<=0 for r in rows)
    ball=sphere(rows.tolist())
    if not ball or ball['diameter_lower_mm']<.1:return None
    return {'halfspaces_mm':rows.tolist(),'bbox_mm':[lo.tolist(),hi.tolist()],'sphere':ball,'analytical_only':True,'construction':'Halfspace outer hull of required point and a 0.100002mm cube. Not a runtime mesh.'}


def cap_safety(cover,caps):
    lo,hi=np.array(cover['bbox_mm']);certs=[]
    for cap in caps:
        done=None
        for i,row in enumerate(cap['carve_halfspaces_mm']):
            # Necessary prefilter at the exact required point plus cube hull.
            target=-np.array(row);target[3]-=CAP_EPS
            c=maximum_certificate(cover['halfspaces_mm'],[F(float(x))for x in target],lo,hi)
            if c:
                actual=F(c['exact_upper_bound'])-F(float(row[3]))-F(c['exact_objective_row'][3])
                done={'cap':cap['id'],'facet':i,'certificate':c,'actual_reversed_facet_upper_exact':str(actual)};break
        if done is None:return None
        certs.append(done)
    return {'all_104_caps':certs,'strict_cap_disjoint':all(F(p['actual_reversed_facet_upper_exact'])<0 for p in certs),'worst_permitted_halfspace_intrusion_mm':CAP_EPS,'equivalence':'Any overlap with a cap is confined to this <=0.001NUM support slab; no >NUM solid/void region is filled.'}


def union_certificate(cover,ids,proofs):
    lo,hi=np.array(cover['bbox_mm']);nodes=[];work=0
    # Tiny analysis rounding is permitted only with an explicit continuous
    # distance-to-original-root homothety bound, never as an input mutation.
    rootrows={};rootbounds={}
    for rid in ids:
        rp=proofs[rid];rr=np.array(rp['outer']);rr[:,3]-=ROOT_EXPAND;rootrows[rid]=rr.tolist()
        b=sphere(rp['inner']);assert b
        center=[F(float(x))for x in b['center_mm']];inner=rp['inner'];margin=min(-(dot([F(float(x))for x in row[:3]],center)+F(float(row[3])))for row in inner)
        eps=max(F(float(a[3]))-F(float(c[3]))for a,c in zip(inner,rr));ratio=eps/(margin+eps)
        bound=ratio*sum(max(abs(center[k]-F(float(lo[k]))),abs(center[k]-F(float(hi[k]))))for k in range(3));assert bound<NUM
        rootbounds[rid]={'root_ball':b,'homothety_ratio_exact':str(ratio),'distance_to_original_root_upper_mm':float(bound),'distance_to_original_root_upper_exact':str(bound)}
    def walk(region,remaining):
        nonlocal work
        work+=1
        if work>1200:return None
        empty=infeasible_certificate(region,lo,hi)
        if empty:
            index=len(nodes);nodes.append({'kind':'EMPTY','region':region,'certificate':empty});return index
        if not remaining:return None
        rid=remaining[0];prefix=region;children=[];skip=[]
        for row in rootrows[rid]:
            target=[F(float(x))for x in row];bound=maximum_certificate(prefix,target,lo,hi)
            if bound:skip.append({'row':row,'proof':bound});continue
            # If the whole prefix is outside, no pointless binary chain.
            reverse=(-np.array(row)).tolist();out=prefix+[reverse]
            child=walk(out,remaining[1:])
            if child is None:return None
            children.append({'row':row,'outside_child':child});prefix=prefix+[row]
        index=len(nodes);nodes.append({'kind':'ROOT_SUBTRACTION','region':region,'root':rid,'ordered_rows':rootrows[rid],'outside_branches':children,'inside_prefix':prefix,'implied_row_certificates':skip});return index
    index=walk(cover['halfspaces_mm'],ids)
    if index is None:return None,{'reason':'No complete continuous union-subset certificate','nodes_attempted':work}
    return {'status':'PASS_CONTINUOUS_MODULO_NUM','entry_node':index,'nodes':nodes,'root_halfspaces':rootrows,'root_equivalence_bounds':rootbounds,'node_count':len(nodes),'maximum_distance_to_original_union_upper_mm':max(q['distance_to_original_root_upper_mm']for q in rootbounds.values())},None


def run(only=None):
    started=time.time();pre=json.loads(PRE.read_text());presha=sha(PRE);assert all(sha(ROOT/'reports'/n)==s for n,s in pre['inputs_sha256'].items())
    base={e['id']:e for e in json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text())['entities']};caps=json.loads((ROOT/'reports/nero-gripper-v23-preregistration.json').read_text())['influence_prisms'];assert len(caps)==104
    old=json.loads(gzip.decompress((ROOT/'reports/nero-gripper-v24-obstructions.json.gz').read_bytes()));targets=[x for t in old['roots']for x in t['classifications']if x['status']=='GEOMETRICALLY_INFEASIBLE'];assert len(targets)==12
    targets.sort(key=lambda x:({'V20157_FP028':0,'V20157_FP090':1}.get(x['entity'],2),x['entity']))
    allids={i for v in pre['neighborhoods'].values()for i in v['root_ids']};proofs={i:exact_root(base[i])for i in sorted(allids)};output=[]
    for target in targets:
        if only and target['entity']not in only:continue
        rid=target['baseline_entity'];ids=pre['neighborhoods'][rid]['root_ids'];point=list(map(F,target['obstruction']['required_point_exact']));nm=neighborhood_margins(point,ids,proofs);cm=cap_margins(point,caps);attempts=[];accepted=None
        ordered=sorted(ids,key=lambda i:max(float(dot(list(map(F,row[:3])),point)+F(row[3]))for row in proofs[i]['exact_support_rows']))
        for seed in ordered:
            center,reason=search_center(point,seed,proofs[seed],caps)
            if center is None:attempts.append({'seed':seed,**reason});continue
            cover=analytical_hull(point,center)
            if cover is None:attempts.append({'seed':seed,'reason':'Analytical hull ball not certified'});continue
            safety=cap_safety(cover,caps)
            if safety is None:attempts.append({'seed':seed,'reason':'Not all 104 cap exclusions certified'});continue
            order=[seed]+[i for i in ordered if i!=seed];union,reason=union_certificate(cover,order,proofs)
            if union is None:attempts.append({'seed':seed,**reason});continue
            strict=[]
            for row in proofs[seed]['exact_support_rows']:
                proof=maximum_certificate(cover['halfspaces_mm'],list(map(F,row)),*np.array(cover['bbox_mm']))
                if proof is None:break
                strict.append(proof)
            strict_subset={'root':seed,'exact_row_dual_certificates':strict,'strict_interior':all(F(p['exact_upper_bound'])<0 for p in strict)}if len(strict)==len(proofs[seed]['exact_support_rows'])else None
            accepted={'seed':seed,'cover':cover,'cap_exclusion':safety,'union_subset':union,'strict_single_root_subset':strict_subset};break
        item={'entity':target['entity'],'baseline_entity':rid,'required_point_exact':[str(x)for x in point],'neighborhood_ids':ids,'neighborhood_margin':nm,'cap_margin':cm,'required_point_robust_gt_NUM':nm['robust_gt_NUM']and cm['robust_gt_NUM'],'classification':'CROSS_ROOT_FEASIBLE'if accepted else'CROSS_ROOT_UNKNOWN','feasible_certificate':accepted,'search_attempts':attempts,'old_root_local_verdict_not_inherited':True}
        output.append(item);report={'status':'V25_CROSS_ROOT_FEASIBLE'if len(output)==12 and all(x['classification']=='CROSS_ROOT_FEASIBLE'for x in output)else'V25_CROSS_ROOT_UNKNOWN','preregistration_sha256':presha,'inputs_sha256':pre['inputs_sha256'],'root_proofs':proofs,'targets':output,'counts':dict(Counter(x['classification']for x in output)),'new_runtime_geometry':False,'num_mm':float(NUM),'elapsed_seconds':time.time()-started,'script_sha256':sha(__file__),'scope':'Only the 12 old root-local obstruction targets. No 343-UNKNOWN analysis. No model/policy/cap edits or MuJoCo.'}
        OUT.write_bytes(gzip.compress(json.dumps(report,separators=(',',':')).encode(),mtime=0));print(target['entity'],item['classification'],'N marginNUM',nm['minimum_distance_interval_NUM'],'cap marginNUM',cm['minimum_distance_NUM'],'attempts',len(attempts),'time',round(time.time()-started,1),flush=True)
    assert sha(PRE)==presha and all(sha(ROOT/'reports'/n)==s for n,s in pre['inputs_sha256'].items())

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--only',nargs='*');args=parser.parse_args();run(args.only)
