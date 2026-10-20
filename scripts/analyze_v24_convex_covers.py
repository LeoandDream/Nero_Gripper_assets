"""Issue25 read-only overlapping convex-cover audit, not a geometry candidate.

The retained oracle is the ORIGINAL root minus frozen finite caps. Saved v23
meshes supply names/leaf ownership only. Covers are analytical halfspace sets;
no runtime vertices/faces are generated. Search failure is always UNKNOWN.
"""
from collections import Counter
from fractions import Fraction as F
from itertools import combinations
from pathlib import Path
import argparse
import gzip
import hashlib
import json
import math
import time

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, linprog, milp

from certify_v23_preservation import maximum_certificate, rational_dot

ROOT=Path(__file__).resolve().parents[1]
NUM=1e-7
COVER_EPS=NUM/4096
REPORT=ROOT/'reports/nero-gripper-v24-convex-cover.json.gz'
INPUT_NAMES=['nero-gripper-v2-runtime-partition.json','nero-gripper-v23-preregistration.json','nero-gripper-v23-split-trace.json.gz','nero-gripper-v23-finite-partition.json','nero-gripper-v23-verification.json','nero-gripper-v21-fidelity-protocol.json','nero-gripper-v2-fidelity-protocol.json']

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,d):Path(p).write_text(json.dumps(d,separators=(',',':'),ensure_ascii=False,allow_nan=False)+'\n')
def dot(a,b):return sum(x*y for x,y in zip(a,b))

def exact_root(root):
    """Enumerate ALL supporting vertex triples with integer predicates.

    Vertices are exact frozen binary64 meter coordinates times integer 1000;
    integer cross products avoid
    treating almost-coplanar facets as exactly coplanar. Every support facet of
    a 3D hull contains a vertex triple, so this enumeration is complete.
    """
    v=np.array(root['vertices_m'])*1000
    fv=[[F(float(x))*1000 for x in q]for q in root['vertices_m']]
    den=max(x.denominator for q in fv for x in q)
    iv=[[int(x*den)for x in q]for q in fv]
    supports=set()
    for ia,ib,ic in combinations(range(len(iv)),3):
        a,b,c=[iv[i]for i in (ia,ib,ic)]
        ab=[b[k]-a[k]for k in range(3)];ac=[c[k]-a[k]for k in range(3)]
        n=[ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0]]
        if not any(n):continue
        d=-dot(n,a);values=[dot(n,q)+d for q in iv]
        if max(values)>0 and min(values)<0:continue
        if max(values)>0:n=[-x for x in n];d=-d
        vals=n+[d];g=math.gcd(*vals);vals=[x//g for x in vals];supports.add(tuple(vals))
    exact=[];inner=[];outer=[];rounding=[]
    lo=np.nextafter(v.min(0),-np.inf);hi=np.nextafter(v.max(0),np.inf)
    assert all(F(float(lo[k]))<=q[k]<=F(float(hi[k]))for q in fv for k in range(3))
    for n0,n1,n2,d in sorted(supports):
        scale=max(abs(n0),abs(n1),abs(n2))
        row=[F(n0,scale),F(n1,scale),F(n2,scale),F(d,scale*den)]
        approx=np.array([float(x)for x in row]);fr=[F(float(x))for x in approx]
        # Upper bound on exact(row)-float(row), over the exact root bbox.
        err_hi=(row[3]-fr[3])+sum(max((row[k]-fr[k])*F(float(lo[k])),(row[k]-fr[k])*F(float(hi[k])))for k in range(3))
        err_lo=(row[3]-fr[3])+sum(min((row[k]-fr[k])*F(float(lo[k])),(row[k]-fr[k])*F(float(hi[k])))for k in range(3))
        reserve=F(1,10**13)
        ri=approx.copy();ri[3]=np.nextafter(float(fr[3]+max(err_hi,F(0))+reserve),np.inf)
        ro=approx.copy();ro[3]=np.nextafter(float(fr[3]+min(err_lo,F(0))-reserve),-np.inf)
        assert err_hi+fr[3]-F(float(ri[3]))<=0
        assert F(float(ro[3]))-fr[3]-err_lo<=0
        exact.append([str(x)for x in row]);inner.append(ri.tolist());outer.append(ro.tolist());rounding.append({'exact_minus_inner_upper':str(err_hi+fr[3]-F(float(ri[3]))),'outer_minus_exact_upper':str(F(float(ro[3]))-fr[3]-err_lo)})
    for k in range(3):
        q=np.zeros(4);q[k]=1;q[3]=-hi[k];inner.append(q.tolist());outer.append(q.tolist())
        q=np.zeros(4);q[k]=-1;q[3]=lo[k];inner.append(q.tolist());outer.append(q.tolist())
    assert len(exact)>3
    return {'inner':inner,'outer':outer,'exact_support_rows':exact,'rounding_certificates':rounding,'bbox_mm':[lo.tolist(),hi.tolist()],'integer_coordinate_denominator':str(den),'supporting_triples_complete':True}


def sphere(rows):
    q=np.array(rows);norm=np.linalg.norm(q[:,:3],axis=1)
    lp=linprog([0,0,0,-1],A_ub=np.c_[q[:,:3],norm],b_ub=-q[:,3],bounds=[(None,None)]*3+[(0,None)],method='highs',options={'primal_feasibility_tolerance':1e-9})
    if not lp.success:return None
    center=lp.x[:3];margins=-(q[:,:3]@center+q[:,3]);radius=max(0,min(margins/norm)-1e-11)
    radius=math.floor(radius*1e11)/1e11
    c=[F(float(x))for x in center];r=F(float(radius));proof=[]
    for row in rows:
        rr=[F(float(x))for x in row];m=-(dot(rr[:3],c)+rr[3]);n2=dot(rr[:3],rr[:3])
        if m<0 or m*m<r*r*n2:return None
        proof.append({'margin_exact':str(m),'squared_sphere_reserve':str(m*m-r*r*n2)})
    return {'center_mm':center.tolist(),'radius_lower_mm':radius,'diameter_lower_mm':2*radius,'ball_constraints':proof}


def search(inner,caps,allowed,lo,hi):
    if any(not o for o in allowed):return None,{'reason':'No registered-cap facet contains the entire obligation; arbitrary separators not excluded.'}
    entries=[(j,i)for j,o in enumerate(allowed)for i in o];n=len(entries);rows=[];lb=[];ub=[]
    for row in inner:
        a=np.zeros(4+n);a[:3]=row[:3];a[3]=np.linalg.norm(row[:3]);rows.append(a);lb.append(-np.inf);ub.append(-row[3])
    rmax=float(np.linalg.norm(hi-lo));big=4*rmax+10
    for k,(j,i)in enumerate(entries):
        q=np.array(caps[j]['carve_halfspaces_mm'][i]);a=np.zeros(4+n);a[:3]=-q[:3];a[3]=np.linalg.norm(q[:3]);a[4+k]=big
        rows.append(a);lb.append(-np.inf);ub.append(big+q[3])
    for j in range(len(caps)):
        a=np.zeros(4+n)
        for k,(jj,i)in enumerate(entries):
            if jj==j:a[4+k]=1
        rows.append(a);lb.append(1);ub.append(np.inf)
    obj=np.zeros(4+n);obj[3]=-1
    result=milp(obj,integrality=[0]*4+[1]*n,bounds=Bounds([*lo,0.050000001]+[0]*n,[*hi,rmax]+[1]*n),constraints=LinearConstraint(rows,lb,ub),options={'time_limit':3,'mip_rel_gap':0.01})
    if result.x is None:return None,{'reason':'Facet-separator MILP has no thick incumbent; NOT a general convex-cover infeasibility proof.','milp_status':int(result.status)}
    choice=[]
    for j in range(len(caps)):
        candidates=[(float(-np.array(caps[j]['carve_halfspaces_mm'][i])[:3]@result.x[:3]-caps[j]['carve_halfspaces_mm'][i][3]),i)for k,(jj,i)in enumerate(entries)if jj==j and result.x[4+k]>0.5]
        if not candidates:return None,{'reason':'No reliable selected separator'}
        choice.append(max(candidates)[1])
    selected=[(-np.array(cap['carve_halfspaces_mm'][i])).tolist()for cap,i in zip(caps,choice)]
    ball=sphere(inner+selected)
    if not ball or ball['diameter_lower_mm']<0.1:return None,{'reason':'No exact certified 0.1mm ball in numerical incumbent'}
    return (choice,ball),None


def prove_leaf(tree,leaf,root_proof,caps,cover,cache):
    m=len(tree['root_support_halfspaces_mm']);region=root_proof['outer']+leaf['region_halfspaces_mm'][m:]
    lo,hi=np.array(root_proof['bbox_mm']);certificates=[]
    for j,i in enumerate(cover['cap_facet_indices']):
        key=(leaf['node'],j,i)
        if key not in cache:
            row=-np.array(caps[j]['carve_halfspaces_mm'][i]);row[3]-=COVER_EPS
            cache[key]=maximum_certificate(region,[F(float(x))for x in row],lo,hi)
        proof=cache[key]
        if proof is None:return None
        certificates.append({'cap':caps[j]['id'],'facet_index':i,'certificate':proof})
    # Every inner-root row differs from the corresponding outer row only in d.
    root_error=max(a[3]-b[3]for a,b in zip(root_proof['inner'],root_proof['outer']))
    eps=max([F(float(COVER_EPS))]+[F(float(a[3]))-F(float(b[3]))for a,b in zip(root_proof['inner'],root_proof['outer'])]+[-F(float(caps[j]['carve_halfspaces_mm'][i][3]))-F(p['certificate']['exact_objective_row'][3])for j,(i,p)in enumerate(zip(cover['cap_facet_indices'],certificates))])
    c=[F(float(x))for x in cover['sphere']['center_mm']]
    rows=cover['halfspaces_mm'];margin=min(-(dot([F(float(x))for x in row[:3]],c)+F(float(row[3])))for row in rows)
    if margin<=0:return None
    # Euclidean distance <= L1 distance. Homothety to the certified ball center.
    diameter=sum(max(abs(c[k]-F(float(lo[k]))),abs(c[k]-F(float(hi[k]))))for k in range(3))
    ef=eps;ratio=ef/(margin+ef);distance=ratio*diameter
    if distance>F(float(NUM)):return None
    return {'node':leaf['node'],'cap_subset_dual_bounds':certificates,'homothety_ratio_exact':str(ratio),'material_to_cover_distance_upper_mm':float(distance),'material_to_cover_distance_exact':str(distance),'coverage':'CONTINUOUS_MODULO_NUM','root_row_error_upper_mm':root_error,'certified_inequality_epsilon_exact':str(eps)}


def run(only=None):
    start=time.time();inputs={name:sha(ROOT/'reports'/name)for name in INPUT_NAMES}
    before=json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text());data=json.loads((ROOT/'reports/nero-gripper-v23-finite-partition.json').read_text())
    assert data['geometry_sha256']=='4050c488325b409532360ee9d8dc3311bd3a53bc36b75949af637bfc8b59fd96'
    pre=json.loads((ROOT/'reports/nero-gripper-v23-preregistration.json').read_text());trace=json.loads(gzip.decompress((ROOT/'reports/nero-gripper-v23-split-trace.json.gz').read_bytes()))
    roots={e['id']:e for e in before['entities']};fails=[q for q in json.loads((ROOT/'reports/nero-gripper-v23-verification.json').read_text())['scale']['rows']if q['status']=='FAIL'];assert len(fails)==729
    failids={q['entity']for q in fails};output=[];allrows=[]
    for tree in sorted(trace['roots'],key=lambda t:(t['baseline_entity']!='V20157',t['baseline_entity'])):
        rid=tree['baseline_entity']
        if only and rid!=only:continue
        rp=exact_root(roots[rid]);caps=[c for c in pre['influence_prisms']if c['baseline_entity']==rid];lo,hi=np.array(rp['bbox_mm']);cache={};covers=[];leafproofs={};uncovered=[]
        leaves=tree['retained_leaves'];successors=[e for e in data['entities']if e.get('baseline_entity')==rid];leafowner={n:e['id']for e in successors for n in e['retained_split_leaf_nodes']}
        # Thick existing leaves first; overlapping covers then absorb thin leaves.
        def width(l):return min(np.ptp(np.array(l['vertices_mm']),axis=0))
        leaves_order=sorted(leaves,key=width,reverse=True)
        attempts=[]
        for leaf in leaves_order:
            found=None
            for cover in covers:
                proof=prove_leaf(tree,leaf,rp,caps,cover,cache)
                if proof:
                    found={'cover':cover['id'],'proof':proof,'new_seed':False};break
            if found:leafproofs[leaf['node']]=found;continue
            v=np.array(leaf['vertices_mm']);allowed=[]
            for cap in caps:
                a=np.array(cap['carve_halfspaces_mm']);minimum=np.min(v@a[:,:3].T+a[:,3],axis=0)
                allowed.append([int(i)for i,x in enumerate(minimum)if x>=-1e-8])
            candidate,reason=search(rp['inner'],caps,allowed,lo,hi)
            if candidate:
                choice,ball=candidate;cover={'id':rid+'_ANALYSIS_COVER_'+str(len(covers)+1).zfill(3),'seed_leaf':leaf['node'],'seed_successor':leafowner.get(leaf['node']),'cap_facet_indices':choice,'halfspaces_mm':rp['inner']+[(-np.array(c['carve_halfspaces_mm'][i])).tolist()for c,i in zip(caps,choice)],'sphere':ball,'baseline_subset':'EXACT_ALL_ROOT_SUPPORTING_TRIPLES','cap_disjoint':'EXACT_ONE_REVERSED_FACET_PER_FROZEN_CAP','runtime_asset':False}
                proof=prove_leaf(tree,leaf,rp,caps,cover,cache)
                if proof:
                    covers.append(cover);leafproofs[leaf['node']]={'cover':cover['id'],'proof':proof,'new_seed':True};found=True
                else:reason={'reason':'Candidate ball safe, but entire oracle obligation coverage not continuously certified.'}
            if not found:uncovered.append(leaf['node']);attempts.append({'node':leaf['node'],'successor':leafowner.get(leaf['node']),'status':'UNKNOWN','detail':reason})
        # Later covers can cover earlier failed leaves too.
        for node in list(uncovered):
            leaf=next(l for l in leaves if l['node']==node)
            for cover in covers:
                proof=prove_leaf(tree,leaf,rp,caps,cover,cache)
                if proof:leafproofs[node]={'cover':cover['id'],'proof':proof,'new_seed':False};uncovered.remove(node);break
        classrows=[]
        for e in successors:
            if e['id']not in failids:continue
            obligations=e['retained_split_leaf_nodes'];pp=[leafproofs.get(n)for n in obligations]
            status='UNKNOWN'
            if pp and all(pp):
                status='ABSORBABLE'if any(p['new_seed']and next(c for c in covers if c['id']==p['cover'])['seed_successor']==e['id']for p in pp)else'REPRESENTATION_FRAGMENT'
            classrows.append({'entity':e['id'],'baseline_entity':rid,'status':status,'required_oracle_leaves':obligations,'cover_ids':sorted({p['cover']for p in pp if p}),'unresolved_oracle_leaves':[n for n,p in zip(obligations,pp)if p is None],'coverage_distances_mm':[p['proof']['material_to_cover_distance_upper_mm']for p in pp if p]})
        rootout={'baseline_entity':rid,'finite_caps':len(caps),'raw_retained_leaves':len(leaves),'saved_successors':len(successors),'root_certificate':rp,'analysis_covers':covers,'covered_raw_leaves':len(leafproofs),'unresolved_raw_leaves':uncovered,'leaf_coverage':[{**p,'node':n}for n,p in sorted(leafproofs.items())],'search_unknowns':attempts,'classification':classrows,'classification_counts':dict(Counter(x['status']for x in classrows)),'all_other_non_discard_terminal_obligations':'NOT_CERTIFIED_IN_THIS_RAW_LEAF_AUDIT'}
        output.append(rootout);allrows.extend(classrows)
        report={'status':'V24_CONVEX_COVER_UNKNOWN','inputs_sha256':inputs,'geometry_unchanged':True,'roots':output,'classification_counts':dict(Counter(x['status']for x in allrows)),'classified_count':len(allrows),'analysis_only':True,'numerical_equivalence_mm':NUM,'obligation_row_reserve':COVER_EPS,'cap_normalization':'NONE; original finite caps unchanged','elapsed_seconds':time.time()-start,'script_sha256':sha(__file__)}
        REPORT.write_bytes(gzip.compress(json.dumps(report,separators=(',',':')).encode(),mtime=0))
        print(rid,'covers',len(covers),'leaves',len(leafproofs),'/',len(leaves),'class',rootout['classification_counts'],'seconds',round(time.time()-start,1),flush=True)
    assert all(sha(ROOT/'reports'/n)==s for n,s in inputs.items())
    print(json.dumps({'status':report['status'],'counts':report['classification_counts'],'elapsed_seconds':report['elapsed_seconds']},indent=2),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--only-root');args=parser.parse_args();run(args.only_root)
