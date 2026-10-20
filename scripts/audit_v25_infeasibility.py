"""Bounded cross-root arbitrary-convex-cover obstruction attempt.

The ball-center domain is a certified OUTER hull of the whole fixed
neighborhood. Thus a complete shadow certificate would exclude arbitrary
convex covers, including cross-root covers. Failure remains UNKNOWN.
Only >NUM robust required points may be used. This writes analysis only.
"""
from fractions import Fraction as F
from pathlib import Path
from collections import Counter
import gzip,json,time
import numpy as np
from scipy.spatial import ConvexHull
from audit_v25_cross_root import ROOT,NUM,dot
from certify_v24_point_obstruction import centers,obstruction

OUT=ROOT/'reports/nero-gripper-v25-obstruction-attempts.json.gz'

def outer_domain(ids,base):
    vertices=[[F(float(x))*1000 for x in v]for i in ids for v in base[i]['vertices_m']];a=np.array([[float(x)for x in v]for v in vertices]);h=ConvexHull(a)
    # Rounding normals only enlarges the hull when each offset is recomputed
    # from ALL exact frozen vertices. No Qhull facet completeness is assumed.
    normals=sorted({tuple(np.round(q[:3],10))for q in h.equations});rows=[]
    for n0 in normals:
        n=[F(float(x))for x in n0];d=-max(dot(n,v)for v in vertices);row=n+[d]
        assert all(dot(n,v)+d<=0 for v in vertices);rows.append([str(x)for x in row])
    return {'exact_support_rows':rows,'bbox_mm':[np.nextafter(a.min(0),-np.inf).tolist(),np.nextafter(a.max(0),np.inf).tolist()],'scope':'Conservative analytical outer support hull of ALL vertices of all fixed neighborhood roots. Not runtime geometry.'}

def main():
    start=time.time();a=json.loads(gzip.decompress((ROOT/'reports/nero-gripper-v25-cross-root.json.gz').read_bytes()));rob={t['entity']:t for t in json.loads((ROOT/'reports/nero-gripper-v25-robust-points.json').read_text())['targets']};base={e['id']:e for e in json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text())['entities']};caps={c['id']:c for c in json.loads((ROOT/'reports/nero-gripper-v23-preregistration.json').read_text())['influence_prisms']};domains={};output=[]
    for target in a['targets']:
        if target['classification']=='CROSS_ROOT_FEASIBLE':continue
        ridge=target['baseline_entity'];replacement=rob[target['entity']]['robust_representative']
        if target['required_point_robust_gt_NUM']:
            point=list(map(F,target['required_point_exact']));cm=target['cap_margin'];nm=target['neighborhood_margin'];origin='ORIGINAL'
        elif replacement:
            point=list(map(F,replacement['point_exact']));cm=replacement['cap_margin'];nm=replacement['neighborhood_margin'];origin='SAME_OBLIGATION_ROBUST_REPLACEMENT'
        else:
            output.append({'entity':target['entity'],'classification':'CROSS_ROOT_UNKNOWN','reason':'No robust required point; forbidden to infer infeasibility from NUM-band point.'});continue
        assert nm['robust_gt_NUM']and cm['robust_gt_NUM']
        if ridge not in domains:
            outer=outer_domain(target['neighborhood_ids'],base);domains[ridge]={'bounding_hull':outer,'possible_ball_centers':centers(outer)}
        domain=domains[ridge];near=sorted(cm['all_cap_lower_bounds'],key=lambda q:F(q['distance_squared_lower_exact']))[:4];attempts=[];proof=None
        for item in near:
            cid=item['cap'];p,unknown=obstruction(point,caps[cid],domain['possible_ball_centers']);attempts.append({'cap':cid,'result':'CERTIFIED'if p else'UNKNOWN','detail':unknown})
            if p:proof=p;break
        output.append({'entity':target['entity'],'classification':'CROSS_ROOT_INFEASIBLE'if proof else'CROSS_ROOT_UNKNOWN','point_origin':origin,'required_point_exact':[str(x)for x in point],'neighborhood_margin':nm,'cap_margin':cm,'cap_attempts':attempts,'arbitrary_cover_obstruction':proof,'domain_key':ridge})
        print(target['entity'],output[-1]['classification'],'point',origin,'tested caps',len(attempts),flush=True)
    report={'domains':domains,'targets':output,'counts':dict(Counter(x['classification']for x in output)),'elapsed_seconds':time.time()-start,'method':'The complete center polytope is a conservative overapproximation for balls contained anywhere in the entire fixed union. A single-cap robust shadow must cover EVERY exact center vertex to prove infeasibility. No search failure is promoted.'};OUT.write_bytes(gzip.compress(json.dumps(report,separators=(',',':')).encode(),mtime=0))

if __name__=='__main__':main()
