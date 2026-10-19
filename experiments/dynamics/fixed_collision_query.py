"""Issue18: fixed historical qpos collision queries, never dynamics/constraints.

prepare -> gate0(MPR only) -> preregister -> ab -> reduce. Each query receives a
new MjData. No original model/XML or state is written; native flag changes only
on a copied scratch model. Every stage refuses reuse or a previous STOP.
"""
import argparse
from collections import Counter
import copy
import datetime
import hashlib
import json
import numbers
from pathlib import Path
import subprocess
import sys
import traceback

import numpy as np
from collision_geometry_match import compare_geometry, GATE0_TOLERANCES

BASE='53c99982ffb9d5b8cec51849dba0a33e2f9b751e'
SOURCE=Path('/data_robot/projects/.worktrees/nero-gripper-contact-geometry')
HISTORY=Path('/data_robot/projects/.worktrees/nero-gripper-full-grasp/experiments/dynamics/runs/issue14_full_grasp_002')
AB_SCALES=dict(distance_m=.001,position_m=.01,normal_deg=30.)
KEYS={'R1':[23622,23623,23624],'R0':[33317,33318,33319]}


def now():return datetime.datetime.now(datetime.timezone.utc).isoformat()
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')
def code_seals():return {p.name:sha(p) for p in [Path(__file__),Path(__file__).with_name('collision_geometry_match.py')]}
def angle(a,b):return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b)),np.dot(a,b))))
def geometry(e):return {k:e[k] for k in ['geoms','bodies','distance_m','position_m','normal']}
def robot(e):return 'cube' in e['bodies'] and 'world' not in e['bodies']
def over(events):return any(robot(e) and e['distance_m']<-.001 for e in events)
def unknown(compare):return any(p['correspondence_status']=='UNKNOWN' for p in compare['pairs'])


def output_outside_sources(output,sources):
    output=Path(output).resolve()
    if any(output==Path(s).resolve() or Path(s).resolve() in output.parents for s in sources):
        raise ValueError('Output must be outside archived sources')


def guard_api(mj):
    allowed={'mj_kinematics','mj_collision','mj_id2name','mj_name2id'}
    audit=dict(calls=Counter(),forbidden_attempts=[])
    def wrapper(name,fn):
        def checked(*args,**kwargs):
            if name not in allowed:
                audit['forbidden_attempts'].append(name)
                raise RuntimeError('Unauthorized API: '+name)
            audit['calls'][name]+=1
            return fn(*args,**kwargs)
        return checked
    for name in dir(mj):
        if name.startswith('mj_') and callable(getattr(mj,name)):
            setattr(mj,name,wrapper(name,getattr(mj,name)))
    return audit


def require_gate(gate):
    if gate['status']!='PASS':raise ValueError('STOP: QUERY_REPRODUCTION_UNKNOWN')


def fields(obj):
    result={}
    for name in dir(obj):
        if name.startswith('_'):continue
        value=getattr(obj,name)
        if isinstance(value,np.ndarray):result[name]=value.copy()
        elif isinstance(value,(numbers.Number,str,bytes)):result[name]=value
    return result


def compare_models(a,b,native_bit):
    aa,bb=fields(a),fields(b)
    if aa.keys()!=bb.keys():raise ValueError('Compiled model schema changed')
    for key,value in aa.items():
        if not np.array_equal(value,bb[key]):raise ValueError('Model field changed: '+key)
    oa,ob=fields(a.opt),fields(b.opt)
    if oa.keys()!=ob.keys():raise ValueError('Option schema changed')
    diff=[k for k in oa if not np.array_equal(oa[k],ob[k])]
    if diff!=['enableflags'] or int(oa['enableflags']) & native_bit or int(ob['enableflags'])!=int(oa['enableflags'])|native_bit:
        raise ValueError('Only nativeccd may differ: '+str(diff))
    return dict(compiled_fields_unchanged=len(aa),compiled_field_names=sorted(aa),
                option_differences=diff,enableflags_A=int(oa['enableflags']),enableflags_B=int(ob['enableflags']),
                other_option_fields_unchanged=len(oa)-1)


def verify_inputs(source):
    rel='reports/issue16-evidence/manifest.json';manifest=source/rel
    expected=subprocess.check_output(['git','show',BASE+':'+rel])
    if manifest.read_bytes()!=expected:raise ValueError('BLOCKED: Issue16 manifest differs from frozen Git')
    m=read(manifest);seals={p:e['sha256'] for p,e in m['input_freeze']['core_inputs'].items()}
    # Include the Issue16 published references actually used for MTD/row scope.
    for name in ['manifest.json','summary.json','exceedances.csv']:
        p=source/'reports/issue16-evidence'/name
        frozen=subprocess.check_output(['git','show',BASE+':reports/issue16-evidence/'+name])
        if p.read_bytes()!=frozen:raise ValueError('BLOCKED: frozen Issue16 artifact changed: '+name)
        seals[str(p)]=sha(p)
    for p,digest in seals.items():
        if sha(p)!=digest:raise ValueError('BLOCKED: original SHA mismatch '+p)
    if (HISTORY/'R0/model.xml').read_bytes()!=(HISTORY/'R1/model.xml').read_bytes():
        raise ValueError('BLOCKED: R0/R1 models differ')
    return seals


def load_episode(label):
    p=HISTORY/label
    with np.load(p/'trajectory.npz',allow_pickle=False) as z:
        a={k:z[k] for k in ['time','phase','qpos']}
    rows=[]
    with (p/'contacts.jsonl').open() as stream:
        for i,line in enumerate(stream):
            r=json.loads(line)
            if r['time']!=a['time'][i] or r['phase']!=a['phase'][i]:raise ValueError('Archive row/time mismatch')
            rows.append([geometry(e) for e in r['events']])
    if len(rows)!=len(a['time']):raise ValueError('Archive length mismatch')
    a['events']=rows
    return a


def prepare(campaign):
    output_outside_sources(campaign,[SOURCE,HISTORY]);campaign.mkdir(parents=True,exist_ok=False)
    seals=verify_inputs(SOURCE);episodes={k:load_episode(k) for k in ['R0','R1']}
    selected={};violations={}
    for label,a in episodes.items():
        violations[label]=[i for i,e in enumerate(a['events']) if over(e)]
        selected[label]=set(KEYS[label])|{j for i in violations[label] for j in [i-1,i,i+1] if 0<=j<len(a['time'])}
    if [len(violations[k]) for k in ['R0','R1']]!=[156,93]:raise ValueError('BLOCKED: original exceedance set changed')
    a=episodes['R1'];controls=[];pair={'cube_geom','gripper_link2_collision_000'}
    for i in range(violations['R1'][0]-1,1,-1):
        if i in selected['R1'] or a['phase'][i]!='lift':continue
        contacts=[[e for e in a['events'][j] if set(e['geoms'])==pair] for j in [i-1,i,i+1]]
        if any(len(es)!=1 for es in contacts):continue
        prev,cur,nxt=[es[0] for es in contacts]
        jumps=[angle(prev['normal'],cur['normal']),angle(cur['normal'],nxt['normal'])]
        if max(0.,-cur['distance_m'])>=.0009 or max(jumps)>=30.:continue
        controls.append(dict(label='R1',row=i,phase=str(a['phase'][i]),geometry=cur,
                             adjacent_normal_jumps_deg=jumps,rule='nearest earlier unselected lift rows; saved penetration<0.9mm; adjacent jumps<30deg'))
        if len(controls)==2:break
    if len(controls)!=2:raise ValueError('BLOCKED: no two eligible normal controls')
    selected['R1'].update(c['row'] for c in controls)
    gate={'R1':sorted(set(KEYS['R1']+[25984,25985]+[c['row'] for c in controls])),'R0':KEYS['R0']}
    qpos_seals={label:{str(i):hashlib.sha256(a['qpos'][i].tobytes()).hexdigest() for i in sorted(selected[label])} for label,a in episodes.items()}
    plan=dict(issue=18,source_commit=BASE,created_utc=now(),source_root=str(SOURCE),history=str(HISTORY),
        inputs_sha256=seals,selected_rows={k:sorted(v) for k,v in selected.items()},original_exceedance_rows=violations,
        gate0_rows=gate,normal_controls=controls,gate0_tolerances=GATE0_TOLERANCES,ab_matching_cost_scales=AB_SCALES,
        qpos_sha256=qpos_seals,new_steps=0,new_force_solves=0,query_count=0,code_sha256=code_seals())
    write(campaign/'plan.json',plan)
    (campaign/'plan.sha256').write_text(sha(campaign/'plan.json')+'\n')
    print(json.dumps(dict(status='PREPARED',rows={k:len(v) for k,v in selected.items()},controls=[c['row'] for c in controls],gate0_rows=sum(map(len,gate.values())),queries=0)))


def load_plan(campaign):
    if (campaign/'STOPPED.json').exists():raise ValueError('Campaign STOPPED; no new queries authorized')
    try:
        if sha(campaign/'plan.json')!=(campaign/'plan.sha256').read_text().strip():raise ValueError('Plan seal changed')
        plan=read(campaign/'plan.json')
        if plan['code_sha256']!=code_seals():raise ValueError('Experiment code changed since sealed plan')
        for p,digest in plan['inputs_sha256'].items():
            if sha(p)!=digest:raise ValueError('BLOCKED input SHA '+p)
        return plan
    except BaseException:
        if campaign.is_dir():
            write(campaign/'STOPPED.json',dict(status='BLOCKED_UNKNOWN',mode='preflight',utc=now(),
                error=traceback.format_exc(),new_queries_in_failed_preflight=0,new_steps=0,new_force_solves=0))
        raise


class Queries:
    def __init__(self):
        import mujoco
        self.mj=mujoco;self.audit=guard_api(mujoco)
        if mujoco.__version__!='3.2.3':raise ValueError('BLOCKED: requires MuJoCo3.2.3')
        self.a=mujoco.MjModel.from_xml_path(str(HISTORY/'R1/model.xml'));self.b=None
        if int(self.a.opt.enableflags)!=0:raise ValueError('Unexpected original enable flags')
        if self.a.nflex or self.a.nplugin:raise ValueError('Only frozen rigid model permitted')
        self.geoms=[mujoco.mj_id2name(self.a,mujoco.mjtObj.mjOBJ_GEOM,i) for i in range(self.a.ngeom)]
        self.bodies=[mujoco.mj_id2name(self.a,mujoco.mjtObj.mjOBJ_BODY,i) for i in range(self.a.nbody)]

    def enable_native(self):
        bit=int(self.mj.mjtEnableBit.mjENBL_NATIVECCD)
        self.b=copy.copy(self.a);self.b.opt.enableflags=int(self.a.opt.enableflags)|bit
        return compare_models(self.a,self.b,bit)

    def query(self,label,row,qpos,algorithm,stream):
        m=self.a if algorithm=='A' else self.b
        if m is None:raise ValueError('Native not enabled')
        data=self.mj.MjData(m)  # Fresh independently allocated buffers every query.
        data.qpos[:]=qpos
        started=now();self.mj.mj_kinematics(m,data);self.mj.mj_collision(m,data)
        if data.time!=0 or data.nefc!=0 or np.any(data.qvel) or np.any(data.ctrl) or np.any(data.qfrc_constraint):
            raise ValueError('Unexpected dynamics/constraint state')
        np.testing.assert_array_equal(data.qpos,qpos)
        if np.any(data.warning.number):raise ValueError('Collision API warning')
        events=[]
        for c in data.contact:
            ids=[int(c.geom1),int(c.geom2)]
            events.append(dict(geoms=[self.geoms[g] for g in ids],bodies=[self.bodies[int(m.geom_bodyid[g])] for g in ids],
                distance_m=float(c.dist),position_m=c.pos.tolist(),normal=c.frame[:3].tolist()))
        record=dict(label=label,row=row,algorithm=algorithm,start_utc=started,end_utc=now(),time=0.,nefc=0,
            scratch_qvel_and_ctrl_zero=True,fresh_MjData=True,qpos_sha256=hashlib.sha256(data.qpos.tobytes()).hexdigest(),
            contact_count=len(events),events=events)
        stream.write(json.dumps(record,allow_nan=False)+'\n');stream.flush()
        return record


def stage(campaign,mode):
    plan=load_plan(campaign);path=campaign/mode;path.mkdir(exist_ok=False)
    write(path/'STARTED.json',dict(utc=now(),mode=mode,git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        argv=sys.argv,code_sha256=code_seals(),plan_sha256=sha(campaign/'plan.json'),python=sys.executable,new_steps=0,new_force_solves=0))
    for name in code_seals():(path/name).write_bytes(Path(__file__).with_name(name).read_bytes())
    engine=None
    try:
        if mode=='ab':
            require_gate(read(campaign/'gate0/result.json'))
            pred=read(campaign/'predictions.json')
            if sha(campaign/'predictions.json')!=(campaign/'predictions.sha256').read_text().strip():raise ValueError('Prediction SHA changed')
            if pred['gate0_result_sha256']!=sha(campaign/'gate0/result.json') or pred['plan_sha256']!=sha(campaign/'plan.json') or pred['code_sha256']!=code_seals():
                raise ValueError('Preregistration dependency changed')
        engine=Queries();episodes={k:load_episode(k) for k in ['R0','R1']};results=[];fresh_a={}
        selection=plan['gate0_rows'] if mode=='gate0' else plan['selected_rows']
        with (path/'A.jsonl').open('x') as stream:
            for label,ids in selection.items():
                for i in ids:
                    a=episodes[label];q=a['qpos'][i]
                    if hashlib.sha256(q.tobytes()).hexdigest()!=plan['qpos_sha256'][label][str(i)]:raise ValueError('qpos seal mismatch')
                    fresh=engine.query(label,i,q,'A',stream);fresh_a[(label,i)]=fresh
                    cmp=compare_geometry(a['events'][i],fresh['events'],plan['gate0_tolerances'])
                    rec=dict(label=label,row=i,comparison=cmp);results.append(rec)
                    if cmp['status']!='PASS':
                        write(path/'reproduction-failure.json',dict(**rec,archived=a['events'][i],fresh_A=fresh))
                        raise ValueError('QUERY_REPRODUCTION_UNKNOWN at '+label+':'+str(i))
        write(path/'reproduction.json',results)
        result=dict(status='PASS',query_reproduction='PASS',mode=mode,rows={k:len(v) for k,v in selection.items()},
                    native_queries=0,api_calls=engine.audit,plan_sha256=sha(campaign/'plan.json'),new_steps=0,new_force_solves=0)
        if mode=='ab':
            difference=engine.enable_native();write(path/'model-difference.json',difference)
            with (path/'B.jsonl').open('x') as stream,(path/'comparison.jsonl').open('x') as output:
                for label,ids in selection.items():
                    for i in ids:
                        a=episodes[label];b=engine.query(label,i,a['qpos'][i],'B',stream);aa=fresh_a[(label,i)]
                        cmp=compare_geometry(aa['events'],b['events'],cost_scales=plan['ab_matching_cost_scales'])
                        record=dict(label=label,row=i,phase=str(a['phase'][i]),archive=a['events'][i],A=aa['events'],B=b['events'],A_B_comparison=cmp)
                        output.write(json.dumps(record,allow_nan=False)+'\n');output.flush()
                        if unknown(cmp):raise ValueError('STOP UNKNOWN contact correspondence '+label+':'+str(i))
            compare_models(engine.a,engine.b,int(engine.mj.mjtEnableBit.mjENBL_NATIVECCD))
            result['native_queries']=sum(map(len,selection.values()))
            result['preregistration_sha256']=sha(campaign/'predictions.json')
        for p,digest in plan['inputs_sha256'].items():
            if sha(p)!=digest:raise ValueError('Input changed after queries '+p)
        write(path/'result.json',dict(**result,input_hashes_unchanged=True,completed_utc=now()))
        print(json.dumps(result))
    except BaseException:
        failure=dict(status='BLOCKED_UNKNOWN',error=traceback.format_exc(),utc=now(),mode=mode,
            api_calls={} if engine is None else engine.audit,new_steps=0,new_force_solves=0)
        write(path/'failure.json',failure);write(campaign/'STOPPED.json',failure)
        raise


def preregister(campaign):
    plan=load_plan(campaign);require_gate(read(campaign/'gate0/result.json'))
    started=read(campaign/'gate0/STARTED.json')
    if started['code_sha256']!=plan['code_sha256'] or started['plan_sha256']!=sha(campaign/'plan.json'):
        raise ValueError('Gate0 executed code/plan differs from preregistration')
    source=read(SOURCE/'reports/issue16-evidence/summary.json')
    frame=next(r for r in source['episodes']['R1']['key_frames'] if r['row']==23623)
    target=next(c['sat']['depth_m'] for c in frame['contacts'] if c['mesh']=='gripper_link2_collision_000')
    prediction=dict(issue=18,created_utc=now(),source_commit=BASE,plan_sha256=sha(campaign/'plan.json'),
        gate0_result_sha256=sha(campaign/'gate0/result.json'),code_sha256=code_seals(),first_native_query_not_yet_executed=True,
        H1=dict(row=23623,preceding_row=23622,geom_pair=['cube_geom','gripper_link2_collision_000'],
            independent_MTD_reference_m=target,error_reduction_fraction_at_least=.5,native_incoming_normal_jump_strictly_below_deg=20.,
            conjunction_required=True,interpretation='Fixed-pose collision-query sensitivity only, not grasp/root-cause acceptance'),
        H2=dict(rows=list(range(25984,25992)),predicted_any_native_robot_cube_penetration_over_m=.001,
                expected_any=True,interpretation='Boundary observation; report actual yes/no, not an obligatory success or replacement criterion'),
        physical_grasp='FAIL',model_change_authorization='NONE',new_steps=0,new_force_solves=0)
    write(campaign/'predictions.json',prediction)
    (campaign/'predictions.sha256').write_text(sha(campaign/'predictions.json')+'\n')
    print(json.dumps(dict(status='FROZEN',prediction_sha256=sha(campaign/'predictions.json'),H1=prediction['H1'],H2=prediction['H2'])))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=['prepare','gate0','preregister','ab'])
    parser.add_argument('--campaign',required=True,type=Path);args=parser.parse_args();campaign=args.campaign.resolve()
    if args.mode=='prepare':prepare(campaign)
    elif args.mode=='preregister':preregister(campaign)
    else:stage(campaign,args.mode)
