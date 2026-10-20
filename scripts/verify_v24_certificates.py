"""Independent certificate replay using stdlib exact rational arithmetic.

No optimizer, scientific package, project helper, collision API or model writer
is imported. Recomputes hull-support completeness, LP dual bounds, balls and
robust obstruction witnesses, rather than trusting search status strings.
"""
from collections import Counter
from fractions import Fraction as Q
from itertools import combinations
from pathlib import Path
import gzip,hashlib,json,math,subprocess,time

ROOT=Path(__file__).resolve().parents[1]
NUM=Q(float(1e-7))

def read(name):
    p=ROOT/'reports'/name;b=p.read_bytes();return json.loads(gzip.decompress(b)if p.suffix=='.gz'else b)
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def vec(x):return list(map(Q,x))
def floatvec(x):return [Q(float(y))for y in x]
def scalar(a,b):return sum(x*y for x,y in zip(a,b))
def residual(row,p):return scalar(row[:3],p)+row[3]
def determinant(m):return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])

def hull_rows(entity):
    p=[[Q(float(x))*1000 for x in v]for v in entity['vertices_m']];d=max(q.denominator for v in p for q in v);iv=[[int(q*d)for q in v]for v in p];out=set()
    for ids in combinations(range(len(p)),3):
        a,b,c=[iv[i]for i in ids];u=[b[k]-a[k]for k in range(3)];v=[c[k]-a[k]for k in range(3)];n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
        if n==[0,0,0]:continue
        b0=-scalar(n,a);rr=[scalar(n,q)+b0 for q in iv]
        if min(rr)<0<max(rr):continue
        if max(rr)>0:n=[-x for x in n];b0=-b0
        scale=max(map(abs,n));out.add(tuple([Q(x,scale)for x in n]+[Q(b0,d*scale)]))
    assert len(out)>3;return out

def verify_dual(proof,region,target,lo,hi):
    ids=proof['constraint_ids'];w=vec(proof['nonnegative_dual_weights']);assert len(ids)==len(w)and all(y>=0 for y in w)
    assert vec(proof['exact_objective_row'])==target
    remainder=[target[k]-sum(y*region[i][k]for y,i in zip(w,ids))for k in range(3)]
    upper=target[3]-sum(y*region[i][3]for y,i in zip(w,ids))+sum(max(x*a,x*b)for x,a,b in zip(remainder,lo,hi))
    assert upper==Q(proof['exact_upper_bound'])and upper<=0


def center_vertices(rows):
    ints=[]
    for row in rows:
        d=math.lcm(*(v.denominator for v in row));ints.append([int(v*d)for v in row])
    result=set()
    for ids in combinations(range(len(rows)),3):
        rr=[ints[i]for i in ids];aa=[r[:3]for r in rr];den=determinant(aa)
        if den==0:continue
        numer=[]
        for k in range(3):
            mm=[r.copy()for r in aa]
            for i in range(3):mm[i][k]=-rr[i][3]
            numer.append(determinant(mm))
        if den<0:den=-den;numer=[-x for x in numer]
        if all(scalar(r[:3],numer)+r[3]*den<=0 for r in ints):result.add(tuple(Q(x,den)for x in numer))
    return result


def main():
    start=time.time();r=read('nero-gripper-v24-convex-cover.json.gz');o=read('nero-gripper-v24-obstructions.json.gz');baseline={e['id']:e for e in read('nero-gripper-v2-runtime-partition.json')['entities']};trees={t['baseline_entity']:t for t in read('nero-gripper-v23-split-trace.json.gz')['roots']};pre=read('nero-gripper-v23-preregistration.json');capdict={c['id']:c for c in pre['influence_prisms']};capsbyroot={rid:[c for c in pre['influence_prisms']if c['baseline_entity']==rid]for rid in trees}
    assert all(digest(ROOT/'reports'/n)==s for n,s in r['inputs_sha256'].items());assert o['input_cover_audit_sha256']==digest(ROOT/'reports/nero-gripper-v24-convex-cover.json.gz')
    covercount=dualcount=ballcount=leafcount=shadowcount=pointcount=0;classes={};rootproofs={}
    for root in r['roots']:
        rid=root['baseline_entity'];rp=root['root_certificate'];rootproofs[rid]=rp;expected=hull_rows(baseline[rid]);assert expected=={tuple(vec(x))for x in rp['exact_support_rows']};lo,hi=[floatvec(x)for x in rp['bbox_mm']];ex=[vec(x)for x in rp['exact_support_rows']];inn=[floatvec(x)for x in rp['inner']];out=[floatvec(x)for x in rp['outer']]
        for exact,a,b in zip(ex,inn,out):
            assert (exact[3]-a[3])+sum(max((exact[k]-a[k])*lo[k],(exact[k]-a[k])*hi[k])for k in range(3))<=0
            assert (b[3]-exact[3])+sum(max((b[k]-exact[k])*lo[k],(b[k]-exact[k])*hi[k])for k in range(3))<=0
        assert len(inn)==len(out)==len(ex)+6
        for k in range(3):
            a=[Q(0)]*4;a[k]=Q(1);a[3]=-hi[k];b=[Q(0)]*4;b[k]=Q(-1);b[3]=lo[k]
            assert inn[len(ex)+2*k]==out[len(ex)+2*k]==a and inn[len(ex)+2*k+1]==out[len(ex)+2*k+1]==b
        covers={c['id']:c for c in root['analysis_covers']};caps=capsbyroot[rid]
        for cover in covers.values():
            rows=[floatvec(x)for x in cover['halfspaces_mm']];assert rows[:len(inn)]==inn
            assert rows[len(inn):]==[[-Q(float(x))for x in cap['carve_halfspaces_mm'][i]]for cap,i in zip(caps,cover['cap_facet_indices'])]
            c=floatvec(cover['sphere']['center_mm']);radius=Q(float(cover['sphere']['radius_lower_mm']));assert radius>=Q(1,20)
            for row in rows:
                m=-residual(row,c);assert m>=0 and m*m>=radius*radius*scalar(row[:3],row[:3]);ballcount+=1
            covercount+=1
        tree=trees[rid];leaves={l['node']:l for l in tree['retained_leaves']};m=len(tree['root_support_halfspaces_mm']);proven=set()
        for item in root['leaf_coverage']:
            p=item['proof'];leaf=leaves[item['node']];cover=covers[item['cover']];region=out+[floatvec(q)for q in leaf['region_halfspaces_mm'][m:]]
            assert len(p['cap_subset_dual_bounds'])==len(caps)
            for cap,i,proof in zip(caps,cover['cap_facet_indices'],p['cap_subset_dual_bounds']):
                assert proof['cap']==cap['id']and proof['facet_index']==i
                target=[-Q(float(x))for x in cap['carve_halfspaces_mm'][i]];target[3]=Q(float(-float(cap['carve_halfspaces_mm'][i][3])-float(r['obligation_row_reserve'])));verify_dual(proof['certificate'],region,target,lo,hi);dualcount+=1
            rows=[floatvec(x)for x in cover['halfspaces_mm']];c=floatvec(cover['sphere']['center_mm']);margin=min(-residual(q,c)for q in rows);eps=max([Q(float(r['obligation_row_reserve']))]+[a[3]-b[3]for a,b in zip(inn,out)]+[-Q(float(cap['carve_halfspaces_mm'][i][3]))-Q(cert['certificate']['exact_objective_row'][3])for cap,i,cert in zip(caps,cover['cap_facet_indices'],p['cap_subset_dual_bounds'])]);assert eps==Q(p['certified_inequality_epsilon_exact']);ratio=eps/(margin+eps);distance=ratio*sum(max(abs(c[k]-lo[k]),abs(c[k]-hi[k]))for k in range(3))
            assert ratio==Q(p['homothety_ratio_exact'])and distance==Q(p['material_to_cover_distance_exact'])and distance<=NUM;leafcount+=1;proven.add(item['node'])
        for x in root['classification']:
            if x['status']!='UNKNOWN':assert set(x['required_oracle_leaves'])<=proven
            classes[x['entity']]=dict(x)
    for root in o['roots']:
        rid=root['baseline_entity'];cp=root['possible_ball_centers'];cr=[vec(x)for x in cp['rows_exact']];expected=center_vertices(cr);assert expected=={tuple(vec(v['point_exact']))for v in cp['vertices']}
        rp=rootproofs[rid];exact=[vec(x)for x in rp['exact_support_rows']];assert len(cr)==len(exact)+6
        for row,b in zip(exact,cr):
            assert row[:3]==b[:3];norm=(b[3]-row[3])/(Q(1,20)-NUM);assert norm>=0 and norm*norm<=scalar(row[:3],row[:3])
        lo,hi=[floatvec(x)for x in rp['bbox_mm']]
        for k in range(3):
            a=[Q(0)]*4;a[k]=Q(1);a[3]=-hi[k]+Q(1,20)-NUM;b=[Q(0)]*4;b[k]=Q(-1);b[3]=lo[k]+Q(1,20)-NUM
            assert cr[len(exact)+2*k]==a and cr[len(exact)+2*k+1]==b
        for x in root['classifications']:
            if x['status']=='UNKNOWN':continue
            assert x['status']=='GEOMETRICALLY_INFEASIBLE';proof=x['obstruction'];p=vec(proof['required_point_exact']);leaf=next(l for l in trees[rid]['retained_leaves']if l['node']==x['required_leaf']);assert all(residual(q,p)<=0 for q in exact);assert all(residual(floatvec(q),p)<=0 for q in leaf['region_halfspaces_mm'])
            for cap in capsbyroot[rid]:assert max(residual(floatvec(q),p)for q in cap['carve_halfspaces_mm'])>=0
            cap=[floatvec(q)for q in capdict[proof['cap']]['carve_halfspaces_mm']];assert {tuple(vec(s['center_vertex']))for s in proof['all_ball_center_vertices']}==expected
            for witness in proof['all_ball_center_vertices']:
                c=vec(witness['center_vertex']);s=vec(witness['shadow_point_exact']);t=Q(witness['perspective_t_exact']);assert t>=1
                for row in cap:
                    # Recover the actual cap point k; verify a Euclidean 5NUM
                    # clearance directly via squaring, without trusting sqrt.
                    k=[(s[j]+(t-1)*p[j])/t for j in range(3)];clearance=-residual(row,k);assert clearance>=0 and clearance*clearance>=(5*NUM)**2*scalar(row[:3],row[:3])
                l1=sum(abs(a-b)for a,b in zip(s,c));assert Q(1,20)-l1-3*NUM*t==Q(witness['ball_overlap_margin_exact'])>0;shadowcount+=1
            classes[x['entity']]={**classes[x['entity']],**x};pointcount+=1
    frozen_entities={e['id']:e for e in read('nero-gripper-v23-finite-partition.json')['entities']}
    for entity,item in classes.items():
        assert item['baseline_entity']==frozen_entities[entity]['baseline_entity']
        assert item['required_oracle_leaves']==frozen_entities[entity]['retained_split_leaf_nodes']
        if item['status']=='GEOMETRICALLY_INFEASIBLE':assert item['required_leaf']in item['required_oracle_leaves']
    failures={x['entity']for x in read('nero-gripper-v23-verification.json')['scale']['rows']if x['status']=='FAIL'};assert set(classes)==failures and len(classes)==729
    counts=Counter(x['status']for x in classes.values());assert counts['GEOMETRICALLY_INFEASIBLE']>0 and counts['ABSORBABLE']+counts['REPRESENTATION_FRAGMENT']>0
    old=read('nero-gripper-v23-provenance.json');mechanical=[]
    for item in old['mechanical_source_inputs']:
        assert digest(ROOT/item['path'])==item['sha256'];mechanical.append(item['path'])
    worktrees=[]
    for path,want in old['protected_worktrees_unchanged'].items():
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=path,text=True);status=subprocess.check_output(['git','status','--porcelain','--untracked-files=all'],cwd=path,text=True);assert head==want['head']and status==want['status'];worktrees.append(path)
    summary={'status':'V24_CONVEX_COVER_MIXED','certificate_replay':'PASS','classification_counts':{k:counts[k]for k in ['ABSORBABLE','REPRESENTATION_FRAGMENT','GEOMETRICALLY_INFEASIBLE','UNKNOWN']},'classified_count':729,'certified_analysis_covers':covercount,'exact_ball_inequalities':ballcount,'continuous_leaf_coverage_certificates':leafcount,'exact_dual_bounds_replayed':dualcount,'robust_obstruction_points':pointcount,'robust_shadow_vertex_certificates':shadowcount,'frozen_input_sha256':r['inputs_sha256'],'mechanical_inputs_unchanged':mechanical,'protected_worktrees_unchanged':worktrees,'analysis_report_sha256':digest(ROOT/'reports/nero-gripper-v24-convex-cover.json.gz'),'obstruction_report_sha256':digest(ROOT/'reports/nero-gripper-v24-obstructions.json.gz'),'independent_verifier_sha256':digest(__file__),'elapsed_seconds':time.time()-start,'scope':'Analytical halfspace covers and certificates only. No runtime candidate or model change. Full A-union exterior acceptance is not claimed.'}
    (ROOT/'reports/nero-gripper-v24-classification.json').write_text(json.dumps({'status':summary['status'],'counts':summary['classification_counts'],'successors':sorted(classes.values(),key=lambda x:x['entity'])},separators=(',',':'))+'\n')
    # The per-entity table references the compressed proof archive; do not
    # duplicate all obstruction rational witnesses in an uncompressed table.
    table=json.loads((ROOT/'reports/nero-gripper-v24-classification.json').read_text())
    for x in table['successors']:
        if 'obstruction'in x:x['obstruction']={'archive':'nero-gripper-v24-obstructions.json.gz','entity':x['entity'],'cap':x['obstruction']['cap'],'required_point_exact':x['obstruction']['required_point_exact']}
    (ROOT/'reports/nero-gripper-v24-classification.json').write_text(json.dumps(table,separators=(',',':'))+'\n')
    (ROOT/'reports/nero-gripper-v24-verification.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
