"""Independent standard-library replay of cross-root certificates.

No optimizer or analysis helper is imported. Shared prior replay utilities use
only exact rational arithmetic. Unknown search results are not promoted.
"""
from pathlib import Path
from fractions import Fraction as Q
from collections import Counter
import argparse,gzip,json,hashlib,math,time,subprocess
from verify_v24_certificates import hull_rows,verify_dual,scalar,center_vertices

ROOT=Path(__file__).resolve().parents[1];NUM=Q(float(1e-7))

def read(name):
 p=ROOT/'reports'/name;b=p.read_bytes();return json.loads(gzip.decompress(b)if p.suffix=='.gz'else b)
def vf(x):return [Q(float(y))for y in x]
def vq(x):return list(map(Q,x))
def val(row,p):return scalar(row[:3],p)+row[3]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def check_cap_margin(p,data,caps):
 projections={}
 for item in data['exact_projection_certificates']:
  cap=caps[item['cap']];rows=[vf(r)for r in cap['carve_halfspaces_mm']];x=vq(item['closest_point_exact']);d2=sum((a-b)**2 for a,b in zip(x,p));assert d2==Q(item['distance_squared_exact'])
  assert all(val(r,x)<=0 for r in rows)
  lam=vq(item['nonnegative_KKT_weights']);ids=item['active_rows'];assert len(lam)==len(ids)and all(a>=0 for a in lam)
  assert all(x[k]-p[k]+sum(a*rows[i][k]for a,i in zip(lam,ids))==0 for k in range(3));assert all(a*val(rows[i],x)==0 for a,i in zip(lam,ids));projections[item['cap']]=d2
 minimum=Q(data['minimum_distance_squared_exact']);assert min(projections.values())==minimum
 assert {x['cap']for x in data['all_cap_lower_bounds']}==set(caps)
 for item in data['all_cap_lower_bounds']:
  rows=[vf(r)for r in caps[item['cap']]['carve_halfspaces_mm']];values=[val(r,p)for r in rows];options=[(x*x/scalar(r[:3],r[:3]),i)for i,(r,x)in enumerate(zip(rows,values))if x>0];bound,i=max(options)if options else(Q(0),-1)
  assert item['facet']==i and Q(item['distance_squared_lower_exact'])==bound
  assert bound>=minimum or projections.get(item['cap'],Q(-1))>=minimum
 assert data['robust_gt_NUM']==(minimum>NUM*NUM)


def check_neighborhood_margin(p,data,ids,roots):
 lower=Q(data['minimum_distance_squared_lower_exact']);upper=Q(data['minimum_distance_squared_upper_exact']);root=data['lower_ball_root'];assert root in ids
 rr=[vq(r)for r in roots[root]['exact_support_rows']];values=[val(r,p)for r in rr];assert max(values)<=0;assert min(v*v/scalar(r[:3],r[:3])for v,r in zip(values,rr))==lower
 ray=data['upper_ray_certificate'];d=vq(ray['direction_exact']);intervals=[]
 for rid in ids:
  lo=Q(0);hi=None;valid=True
  for row in roots[rid]['exact_support_rows']:
   r=vq(row);a=val(r,p);b=scalar(r[:3],d)
   if b==0:
    if a>0:valid=False;break
   elif b>0:hi=-a/b if hi is None else min(hi,-a/b)
   else:lo=max(lo,-a/b)
  if valid and hi is not None and hi>=lo:intervals.append((lo,hi,rid))
 intervals.sort();end=None
 for lo,hi,rid in intervals:
  if end is None:
   if lo>0:break
   end=hi
  elif lo<=end:end=max(end,hi)
  else:break
 assert end==Q(ray['exit_t_exact'])and upper==end*end*scalar(d,d)and lower<=upper
 assert [{'root':rid,'lo':str(lo),'hi':str(hi)}for lo,hi,rid in intervals]==ray['all_root_ray_intervals']
 assert data['robust_gt_NUM']==(lower>NUM*NUM)and data['entirely_within_NUM_band']==(upper<=NUM*NUM)


def main(certificates_only=False):
 start=time.time();r=read('nero-gripper-v25-cross-root.json.gz');pre=read('nero-gripper-v25-cross-root-preregistration.json');base={e['id']:e for e in read('nero-gripper-v2-runtime-partition.json')['entities']};caps={e['id']:e for e in read('nero-gripper-v23-preregistration.json')['influence_prisms']};rob=read('nero-gripper-v25-robust-points.json');neg=read('nero-gripper-v25-obstruction-attempts.json.gz');trace={t['baseline_entity']:t for t in read('nero-gripper-v23-split-trace.json.gz')['roots']};saved={e['id']:e for e in read('nero-gripper-v23-finite-partition.json')['entities']}
 assert r['preregistration_sha256']==sha(ROOT/'reports/nero-gripper-v25-cross-root-preregistration.json');assert len(caps)==104
 for name,s in pre['inputs_sha256'].items():assert sha(ROOT/'reports'/name)==s
 old=read('nero-gripper-v24-classification.json');old12={x['entity']for x in old['successors']if x['status']=='GEOMETRICALLY_INFEASIBLE'};oldunknown={x['entity']for x in old['successors']if x['status']=='UNKNOWN'};assert len(oldunknown)==343 and len(old12)==12 and {x['entity']for x in r['targets']}==old12
 assert not oldunknown&{x['entity']for x in r['targets']}
 for rid,d in pre['neighborhoods'].items():
  expected={rid}|set(base[rid]['reference_adjacency_v2_ids'])|{i for i,x in base.items()if rid in x['reference_adjacency_v2_ids']};assert set(d['root_ids'])==expected and d['second_order_expansion']is False
 proofs=r['root_proofs']
 for rid,rp in proofs.items():
  exact=[vq(row)for row in rp['exact_support_rows']];assert set(map(tuple,exact))==hull_rows(base[rid]);inn=[vf(x)for x in rp['inner']];out=[vf(x)for x in rp['outer']];lo,hi=[vf(x)for x in rp['bbox_mm']]
  for vertex in base[rid]['vertices_m']:
   v=[Q(float(x))*1000 for x in vertex];assert all(lo[k]<=v[k]<=hi[k]for k in range(3))
  for a,b,c in zip(exact,inn,out):
   assert (a[3]-b[3])+sum(max((a[k]-b[k])*lo[k],(a[k]-b[k])*hi[k])for k in range(3))<=0
   assert (c[3]-a[3])+sum(max((c[k]-a[k])*lo[k],(c[k]-a[k])*hi[k])for k in range(3))<=0
 covercount=dualcount=ballcount=margin_count=0
 for t in r['targets']:
  p=vq(t['required_point_exact']);ids=t['neighborhood_ids'];assert ids==pre['neighborhoods'][t['baseline_entity']]['root_ids'];check_neighborhood_margin(p,t['neighborhood_margin'],ids,proofs);check_cap_margin(p,t['cap_margin'],caps);margin_count+=1
  assert t['required_point_robust_gt_NUM']==(t['neighborhood_margin']['robust_gt_NUM']and t['cap_margin']['robust_gt_NUM'])
  if t['classification']!='CROSS_ROOT_FEASIBLE':assert t['classification']=='CROSS_ROOT_UNKNOWN';continue
  item=t['feasible_certificate'];cover=item['cover'];rows=[vf(x)for x in cover['halfspaces_mm']];lo,hi=[vf(x)for x in cover['bbox_mm']];assert all(val(q,p)<=0 for q in rows)
  for k in range(3):
   a=[Q(0)]*4;a[k]=1;a[3]=-hi[k];b=[Q(0)]*4;b[k]=-1;b[3]=lo[k];assert a in rows and b in rows
  c=vf(cover['sphere']['center_mm']);radius=Q(float(cover['sphere']['radius_lower_mm']));assert radius>=Q(1,20)
  for row in rows:
   m=-val(row,c);assert m>=0 and m*m>=radius*radius*scalar(row[:3],row[:3]);ballcount+=1
  assert {a['cap']for a in item['cap_exclusion']['all_104_caps']}==set(caps)
  for q in item['cap_exclusion']['all_104_caps']:
   original=vf(caps[q['cap']]['carve_halfspaces_mm'][q['facet']]);target=vq(q['certificate']['exact_objective_row']);assert target[:3]==[-x for x in original[:3]]
   verify_dual(q['certificate'],rows,target,lo,hi);actual=Q(q['certificate']['exact_upper_bound'])-original[3]-target[3];assert actual==Q(q['actual_reversed_facet_upper_exact']);assert actual<=0 or actual*actual<=NUM*NUM*scalar(original[:3],original[:3]);dualcount+=1
  # For the delivered feasible cover we have a stronger STRICT containment in
  # one adjacent original root, so the relaxed BSP certificate is unnecessary.
  strict=item['strict_single_root_subset'];assert strict and strict['root']in ids
  assert len(strict['exact_row_dual_certificates'])==len(proofs[strict['root']]['exact_support_rows'])
  for row,q in zip(proofs[strict['root']]['exact_support_rows'],strict['exact_row_dual_certificates']):verify_dual(q,rows,vq(row),lo,hi);assert Q(q['exact_upper_bound'])<0;dualcount+=1
  assert item['cap_exclusion']['strict_cap_disjoint']and all(Q(q['actual_reversed_facet_upper_exact'])<0 for q in item['cap_exclusion']['all_104_caps'])
  # Strictly interior to the baseline union, strictly separated from every
  # removed cap: H does not participate in ANY true neighborhood exterior.
  covercount+=1
 for t in rob['targets']:
  assert t['entity']in old12;q=t['robust_representative']
  if q is None:continue
  p=vq(q['point_exact']);original=next(x for x in r['targets']if x['entity']==t['entity']);rid=original['baseline_entity'];assert q['required_leaf']in saved[t['entity']]['retained_split_leaf_nodes'];leaf=next(l for l in trace[rid]['retained_leaves']if l['node']==q['required_leaf'])
  assert all(val(vq(row),p)<=0 for row in proofs[rid]['exact_support_rows']);assert all(val(vf(row),p)<=0 for row in leaf['region_halfspaces_mm']);assert all(max(val(vf(row),p)for row in cap['carve_halfspaces_mm'])>=0 for cap in caps.values())
  check_neighborhood_margin(p,q['neighborhood_margin'],original['neighborhood_ids'],proofs);check_cap_margin(p,q['cap_margin'],caps);assert q['neighborhood_margin']['robust_gt_NUM']and q['cap_margin']['robust_gt_NUM'];margin_count+=1
 # Replay the complete OUTER ball-center domains as a check of the negative
 # search's scope. No failed LP is used as an infeasibility certificate.
 domain_vertex_counts={}
 for rid,d in neg['domains'].items():
  ids=pre['neighborhoods'][rid]['root_ids'];outer=[vq(x)for x in d['bounding_hull']['exact_support_rows']]
  for i in ids:
   for v in base[i]['vertices_m']:
    p=[Q(float(x))*1000 for x in v];assert all(val(row,p)<=0 for row in outer)
  cp=d['possible_ball_centers'];rr=[vq(x)for x in cp['rows_exact']];assert len(rr)==len(outer)+6
  for a,b in zip(outer,rr):
   assert a[:3]==b[:3];norm=(b[3]-a[3])/(Q(1,20)-NUM);assert norm>=0 and norm*norm<=scalar(a[:3],a[:3])
  vertices=center_vertices(rr);assert vertices=={tuple(vq(x['point_exact']))for x in cp['vertices']};domain_vertex_counts[rid]=len(vertices)
 for t in neg['targets']:assert t['entity']in old12 and t['classification']=='CROSS_ROOT_UNKNOWN'and t.get('arbitrary_cover_obstruction')is None
 counts=Counter(t['classification']for t in r['targets']);assert counts=={'CROSS_ROOT_FEASIBLE':1,'CROSS_ROOT_UNKNOWN':11};assert r['status']=='V25_CROSS_ROOT_UNKNOWN'
 oldprov=read('nero-gripper-v23-provenance.json')
 for x in oldprov['mechanical_source_inputs']:assert sha(ROOT/x['path'])==x['sha256']
 for path,w in ([] if certificates_only else oldprov['protected_worktrees_unchanged'].items()):
  assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=path,text=True)==w['head'];assert subprocess.check_output(['git','status','--porcelain','--untracked-files=all'],cwd=path,text=True)==w['status']
 summary={'status':'V25_CROSS_ROOT_UNKNOWN','certificate_replay':'PASS','counts':{'CROSS_ROOT_FEASIBLE':1,'CROSS_ROOT_INFEASIBLE':0,'CROSS_ROOT_UNKNOWN':11},'targets_checked':12,'margin_records_replayed':margin_count,'strict_feasible_covers':covercount,'exact_dual_bounds_replayed':dualcount,'exact_ball_inequalities':ballcount,'robust_replacement_points':sum(t['robust_representative']is not None for t in rob['targets']),'original_robust_points':sum(t['required_point_robust_gt_NUM']for t in r['targets']),'original_points_in_NUM_band':sum(not t['required_point_robust_gt_NUM']for t in r['targets']),'outer_center_domain_vertex_counts':domain_vertex_counts,'all_original_geometry_caps_policy_and_343_UNKNOWN_unchanged':True,'mechanical_inputs_unchanged':len(oldprov['mechanical_source_inputs']),'protected_worktrees_unchanged':None if certificates_only else len(oldprov['protected_worktrees_unchanged']),'inputs_sha256':pre['inputs_sha256'],'evidence_sha256':{n:sha(ROOT/'reports'/n)for n in ['nero-gripper-v25-cross-root.json.gz','nero-gripper-v25-robust-points.json','nero-gripper-v25-obstruction-attempts.json.gz']},'independent_verifier_sha256':sha(__file__),'elapsed_seconds':time.time()-start}
 if not certificates_only:(ROOT/'reports/nero-gripper-v25-verification.json').write_text(json.dumps(summary,indent=2)+'\n')
 print(json.dumps(summary,indent=2))

if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--certificates-only',action='store_true',help='Pure read-only replay; skip other local worktrees and do not write a report.');args=parser.parse_args();main(args.certificates_only)
