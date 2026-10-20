"""Read-only bounded witness search across all unabsorbed scale failures."""
from collections import Counter
from fractions import Fraction as F
import gzip,json,time
import numpy as np
from analyze_v24_convex_covers import ROOT,NUM,dot,sha
from certify_v24_point_obstruction import centers,obstruction

OUT=ROOT/'reports/nero-gripper-v24-obstructions.json.gz'

def membership(point,rp,caps,region):
    root_res=[dot(list(map(F,r[:3])),point)+F(r[3])for r in rp['exact_support_rows']]
    if any(x>0 for x in root_res):return None
    leaf_res=[dot([F(float(x))for x in r[:3]],point)+F(float(r[3]))for r in region]
    if any(x>0 for x in leaf_res):return None
    exclusions=[]
    for c in caps:
        vals=[dot([F(float(x))for x in r[:3]],point)+F(float(r[3]))for r in c['carve_halfspaces_mm']];i=max(range(len(vals)),key=vals.__getitem__)
        if vals[i]<0:return None
        exclusions.append({'cap':c['id'],'outside_facet':i,'exact_outside_margin':str(vals[i])})
    return {'exact_root_maximum':str(max(root_res)),'exact_leaf_maximum':str(max(leaf_res)),'all_cap_exclusions':exclusions}


def main():
    started=time.time();audit=json.loads(gzip.decompress((ROOT/'reports/nero-gripper-v24-convex-cover.json.gz').read_bytes()));before={n:sha(ROOT/'reports'/n)for n in audit['inputs_sha256']}
    trace=json.loads(gzip.decompress((ROOT/'reports/nero-gripper-v23-split-trace.json.gz').read_bytes()));trees={t['baseline_entity']:t for t in trace['roots']};pre=json.loads((ROOT/'reports/nero-gripper-v23-preregistration.json').read_text())
    output=[];checks=[]
    for root in audit['roots']:
        rid=root['baseline_entity'];rp=root['root_certificate'];caps=[c for c in pre['influence_prisms']if c['baseline_entity']==rid];tree=trees[rid];leaves={l['node']:l for l in tree['retained_leaves']};cp=centers(rp);rows=[]
        for target in root['classification']:
            if target['status']!='UNKNOWN':continue
            done=None;attempts=0;pointtrials=0
            for node in target['unresolved_oracle_leaves']:
                leaf=leaves[node];v=np.array(leaf['vertices_mm']);mean=v.mean(0);candidates=[]
                if target['entity']=='V20157_FP028':
                    saved=json.loads((ROOT/'reports/nero-gripper-v23-finite-partition.json').read_text());ev=np.array(next(e for e in saved['entities']if e['id']==target['entity'])['vertices_m'])*1000
                    src=np.array(next(c for c in caps if c['id']=='P0011')['source_plane_mm']);p=ev[4]-src[:3]*1e-9;p[2]-=1e-9;candidates.append(p)
                candidates.append(mean)
                # Boundary-near material, but all membership predicates are checked
                # exactly before a point can support any impossibility claim.
                for lam in [1e-6,1e-3]:
                    candidates.extend((1-lam)*q+lam*mean for q in v)
                valid=[]
                for pp in candidates:
                    point=[F(float(x))for x in pp];m=membership(point,rp,caps,leaf['region_halfspaces_mm'])
                    if not m:continue
                    close=[]
                    for cap in caps:
                        vals=np.array(cap['carve_halfspaces_mm']);s=float(np.max(vals[:,:3]@pp+vals[:,3]))
                        if 0<=s<0.02:close.append((s,cap))
                    if close:valid.append((min(x[0]for x in close),point,m,sorted(close,key=lambda x:x[0])))
                for _,point,m,close in sorted(valid,key=lambda x:x[0])[:8]:
                    pointtrials+=1
                    for _,cap in close[:3]:
                        attempts+=1;proof,unknown=obstruction(point,cap,cp)
                        if proof:
                            done={'entity':target['entity'],'baseline_entity':rid,'status':'GEOMETRICALLY_INFEASIBLE','required_leaf':node,'membership':m,'obstruction':proof};break
                    if done:break
                if done:break
            rows.append(done or {'entity':target['entity'],'baseline_entity':rid,'status':'UNKNOWN','reason':'No continuously certified robust cap-shadow obstruction or thick cover. This is not a geometric impossibility claim.','valid_point_trials':pointtrials,'cap_shadow_attempts':attempts})
        output.append({'baseline_entity':rid,'possible_ball_centers':cp,'classifications':rows});checks.extend(rows)
        report={'input_cover_audit_sha256':sha(ROOT/'reports/nero-gripper-v24-convex-cover.json.gz'),'frozen_inputs_sha256':before,'roots':output,'counts':dict(Counter(x['status']for x in checks)),'method':'Exact required-point membership; exact complete center-polytope enumeration; robust convex cap-shadow witnesses at ALL center vertices, extended by convexity. Failure of this bounded search remains UNKNOWN.','elapsed_seconds':time.time()-started,'script_sha256':sha(__file__)}
        OUT.write_bytes(gzip.compress(json.dumps(report,separators=(',',':')).encode(),mtime=0));print(rid,dict(Counter(x['status']for x in rows)),'seconds',round(time.time()-started,1),flush=True)
    assert before=={n:sha(ROOT/'reports'/n)for n in before}
    print('DONE',report['counts'],flush=True)

if __name__=='__main__':main()
