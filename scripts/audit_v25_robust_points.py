"""Bounded search for stable points in the SAME old retained obligation.

Saved leaf coordinates are only seeds. Membership in original-root halfspaces,
leaf inequalities and all 104 frozen caps is checked exactly before use. No
search failure is called a proof that a robust required point cannot exist.
"""
from fractions import Fraction as F
from pathlib import Path
import gzip,json,time
from audit_v25_cross_root import ROOT,NUM,dot,neighborhood_margins,cap_margins

OUT=ROOT/'reports/nero-gripper-v25-robust-points.json'

def main():
    started=time.time();audit=json.loads(gzip.decompress((ROOT/'reports/nero-gripper-v25-cross-root.json.gz').read_bytes()));trace=json.loads(gzip.decompress((ROOT/'reports/nero-gripper-v23-split-trace.json.gz').read_bytes()));trees={t['baseline_entity']:t for t in trace['roots']};saved={e['id']:e for e in json.loads((ROOT/'reports/nero-gripper-v23-finite-partition.json').read_text())['entities']};caps=json.loads((ROOT/'reports/nero-gripper-v23-preregistration.json').read_text())['influence_prisms'];result=[]
    caprows={c['id']:[[F(float(x))for x in row]for row in c['carve_halfspaces_mm']]for c in caps}
    for target in audit['targets']:
        p0=list(map(F,target['required_point_exact']));rid=target['baseline_entity'];tree=trees[rid];nodes=saved[target['entity']]['retained_split_leaf_nodes'];candidates=[]
        for leaf in tree['retained_leaves']:
            if leaf['node']not in nodes:continue
            v=[[F(float(x))for x in q]for q in leaf['vertices_mm']];mean=[sum(q[k]for q in v)/len(v)for k in range(3)];candidates.append((mean,leaf))
            for q in v:
                for t in [F(1,100),F(1,2),F(99,100)]:candidates.append(([(1-t)*a+t*b for a,b in zip(q,mean)],leaf))
        found=None;valid=0;tested=0
        for point,leaf in candidates:
            tested+=1;root=[list(map(F,r))for r in audit['root_proofs'][rid]['exact_support_rows']]
            if any(dot(r[:3],point)+r[3]>0 for r in root):continue
            if any(dot([F(float(x))for x in r[:3]],point)+F(float(r[3]))>0 for r in leaf['region_halfspaces_mm']):continue
            lowercaps=[];outside=True
            for rows in caprows.values():
                vv=[dot(r[:3],point)+r[3]for r in rows]
                if max(vv)<0:outside=False;break
                lowercaps.append(max((value*value/dot(r[:3],r[:3])for value,r in zip(vv,rows)if value>0),default=F(0)))
            if not outside:continue
            valid+=1
            # These are sufficient WHOLE-BALL margins in the real root union,
            # not a requirement that the ball fit in the artificial BSP leaf.
            rootmargin=min((dot(r[:3],point)+r[3])**2/dot(r[:3],r[:3])for r in root)
            if rootmargin<=NUM*NUM or min(lowercaps)<=NUM*NUM:continue
            nm=neighborhood_margins(point,target['neighborhood_ids'],audit['root_proofs']);cm=cap_margins(point,caps)
            assert nm['robust_gt_NUM']and cm['robust_gt_NUM']
            found={'point_exact':[str(x)for x in point],'required_leaf':leaf['node'],'neighborhood_margin':nm,'cap_margin':cm,'same_retained_obligation_membership':'EXACT_ROOT_AND_LEAF_AND_ALL_CAPS','root_minimum_squared_margin_exact':str(rootmargin)};break
        row={'entity':target['entity'],'original_point_robust':target['required_point_robust_gt_NUM'],'robust_representative':found,'status':'ROBUST_POINT_FOUND'if found else'ROBUST_POINT_NOT_FOUND','candidates_tested':tested,'exact_membership_valid_candidates':valid,'interpretation':'NOT_FOUND is bounded-search uncertainty, never a claim that no robust material exists.'};result.append(row)
        print(row['entity'],row['status'],'tested',tested,'valid',valid,flush=True)
    OUT.write_text(json.dumps({'targets':result,'elapsed_seconds':time.time()-started,'scope':'Only 12 old obstruction targets, original assigned raw retained leaf regions; no geometry generation or proof of infeasibility.'},separators=(',',':'))+'\n')

if __name__=='__main__':main()
