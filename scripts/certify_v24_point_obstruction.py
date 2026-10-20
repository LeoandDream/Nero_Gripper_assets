"""Continuous point/ball obstruction certificates, analytical sets only.

If every possible 0.05mm-ball center is in a cap's robust shadow from a
required point, no legal thick convex cover can contain that point. This is
stronger than failing a facet-choice search. All final inequalities are exact
rational, including a >NUM-radius solid overlap, not just boundary contact.
"""
from fractions import Fraction as F
from itertools import combinations
from decimal import Decimal, localcontext
from pathlib import Path
import gzip,json,math
import numpy as np
from scipy.optimize import linprog
from analyze_v24_convex_covers import ROOT,NUM,exact_root,dot,dump


def norm_bound(a,upper):
    with localcontext()as ctx:
        ctx.prec=90
        square=sum(x*x for x in a);z=(Decimal(square.numerator)/Decimal(square.denominator)).sqrt();scale=10**30
        n=int(z*scale)+(1 if upper else 0);q=F(n,scale)
        assert q*q>=square if upper else q*q<=square
        return q

def det(a):
    return a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])-a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])+a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0])

def exact_vertices(rows):
    ints=[]
    for row in rows:
        d=math.lcm(*(x.denominator for x in row));rr=[int(x*d)for x in row];g=math.gcd(*rr);ints.append([x//g for x in rr])
    vertices={};tested=0;singular=0
    for ids in combinations(range(len(rows)),3):
        tested+=1;rr=[ints[i]for i in ids];aa=[x[:3]for x in rr];den=det(aa)
        if not den:singular+=1;continue
        ns=[]
        for k in range(3):
            mat=[r.copy()for r in aa]
            for i in range(3):mat[i][k]=-rr[i][3]
            ns.append(det(mat))
        if den<0:den=-den;ns=[-x for x in ns]
        if any(dot(row[:3],ns)+row[3]*den>0 for row in ints):continue
        p=tuple(F(x,den)for x in ns);vertices[p]=ids
    return [{'point_exact':[str(x)for x in p],'active_triple':list(ids)}for p,ids in sorted(vertices.items())],{'all_triples':tested,'exact_singular_triples':singular,'exact_vertex_count':len(vertices),'completeness':'Every vertex of a bounded 3D halfspace polytope has an independent active triple; all triples enumerated with integer Cramer determinants.'}

def centers(rootproof):
    r=F(1,20);num=F(float(NUM));rows=[]
    for row in rootproof['exact_support_rows']:
        q=list(map(F,row));norm=norm_bound(q[:3],False);q[3]+=(r-num)*norm;rows.append(q)
    lo,hi=rootproof['bbox_mm']
    for k in range(3):
        q=[F(0)]*4;q[k]=F(1);q[3]=-F(float(hi[k]))+r-num;rows.append(q)
        q=[F(0)]*4;q[k]=F(-1);q[3]=F(float(lo[k]))+r-num;rows.append(q)
    v,proof=exact_vertices(rows)
    return {'rows_exact':[[str(x)for x in row]for row in rows],'vertices':v,'certificate':proof,'radius_mm':0.05,'root_equivalence_expansion_mm':NUM}


def obstruction(point,cap,centerproof):
    p=list(point);rows=[[F(float(x))for x in row]for row in cap['carve_halfspaces_mm']];num=F(float(NUM));r=F(1,20);scale=F(10000)
    proof=[]
    # Search has one extra NUM reserve; final checks use 5NUM eroded cap,
    # and 3NUM ball overlap. The p-uncertainty consumes <=NUM, leaving >NUM.
    for vertex in centerproof['vertices']:
        c=list(map(F,vertex['point_exact']));ar=[];br=[]
        for row in rows:
            a=row[:3];ap=dot(a,p);coeff=ap+row[3]+6*num*norm_bound(a,True)
            ar.append([*[float(x)for x in a],float(coeff*scale),0,0,0]);br.append(float(ap))
        for k in range(3):
            q=[0.]*7;q[k]=1;q[4+k]=-1;ar.append(q);br.append(float(c[k]))
            q=[0.]*7;q[k]=-1;q[4+k]=-1;ar.append(q);br.append(-float(c[k]))
        q=[0.,0.,0.,float(4*num*scale),1,1,1];ar.append(q);br.append(float(r))
        lp=linprog([0,0,0,float(4*num*scale),1,1,1],A_ub=ar,b_ub=br,bounds=[(None,None)]*3+[(float(1/scale),None)]+[(0,None)]*3,method='highs',options={'primal_feasibility_tolerance':1e-9})
        if not lp.success:return None,{'reason':'Shadow LP not certified at one possible ball-center vertex; no impossibility inferred.','center_vertex':vertex['point_exact']}
        s=[F(float(x))for x in lp.x[:3]];t=F(float(lp.x[3]))*scale
        if t<1:return None,{'reason':'Shadow perspective t<1'}
        capres=[]
        for row in rows:
            a=row[:3];value=dot(a,s)+(dot(a,p)+row[3]+5*num*norm_bound(a,True))*t-dot(a,p)
            if value>0:return None,{'reason':'Exact cap shadow inequality not proved'}
            capres.append(str(-value))
        l1=sum(abs(s[k]-c[k])for k in range(3));slack=r-l1-3*num*t
        if slack<=0:return None,{'reason':'No robust >NUM solid-overlap radius reserve'}
        proof.append({'center_vertex':vertex['point_exact'],'shadow_point_exact':[str(x)for x in s],'perspective_t_exact':str(t),'cap_erosion_inequality_reserves':capres,'ball_overlap_margin_exact':str(slack),'inscribed_overlap_radius_before_point_uncertainty_mm':3*NUM})
    return {'status':'PASS_ROBUST_GEOMETRIC_OBSTRUCTION','required_point_exact':[str(x)for x in p],'cap':cap['id'],'point_uncertainty_mm':NUM,'cap_inward_clearance_mm':5*NUM,'convex_cover_overlap_radius_mm':3*NUM,'after_point_uncertainty_overlap_radius_mm':2*NUM,'all_ball_center_vertices':proof,'continuous_proof':'Every center lies in the exact enumerated bounded polytope. Convex combinations of its certified shadow (s,t) witnesses preserve all linear perspective inequalities and the convex L1 bound. Thus conv(p,ball(c,0.05)) contains a 3NUM-ball inside a cap eroded by 5NUM. Perturbing p by <=NUM still leaves a 2NUM-radius forbidden solid overlap. This excludes every convex cover, including arbitrary separating planes and overlapping covers.'},None


def main():
    baseline=json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text());root=next(e for e in baseline['entities']if e['id']=='V20157');rp=exact_root(root);cp=centers(rp)
    data=json.loads((ROOT/'reports/nero-gripper-v23-finite-partition.json').read_text());e=next(e for e in data['entities']if e['id']=='V20157_FP028');v=np.array(e['vertices_m'])*1000
    pre=json.loads((ROOT/'reports/nero-gripper-v23-preregistration.json').read_text());caps=[c for c in pre['influence_prisms']if c['baseline_entity']=='V20157'];cap=next(c for c in caps if c['id']=='P0064')
    source=np.array(next(c for c in caps if c['id']=='P0011')['source_plane_mm'])
    p=v[4]-source[:3]*1e-9;p[2]-=1e-9;point=[F(float(x))for x in p]
    # Membership in the immutable root and outside every registered cap.
    rootres=[dot(list(map(F,row[:3])),point)+F(row[3])for row in rp['exact_support_rows']]
    selected=[]
    for c in caps:
        vals=[dot([F(float(x))for x in row[:3]],point)+F(float(row[3]))for row in c['carve_halfspaces_mm']];i=max(range(len(vals)),key=vals.__getitem__);selected.append({'cap':c['id'],'outside_facet':i,'exact_outside_margin':str(vals[i])})
    print('centers',cp['certificate'],'root membership',max(map(float,rootres)),'cap min',min(float(F(x['exact_outside_margin']))for x in selected),flush=True)
    result,unknown=obstruction(point,cap,cp)
    out={'point_mm':p.tolist(),'root_membership':all(x<=0 for x in rootres),'cap_exclusion':selected,'center_polytope':cp,'obstruction':result,'unknown':unknown}
    dump(ROOT/'debug/issue25/v24-cover/fp028-obstruction.json',out)
    print('obstruction',result['status']if result else unknown,flush=True)

if __name__=='__main__':main()
