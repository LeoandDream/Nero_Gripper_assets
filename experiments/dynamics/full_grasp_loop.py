"""Issue14 bounded full-grasp wrapper: frozen runtime, <=5 real episodes.

prepare never integrates. register seals a single R0-relative action hypothesis.
run calls the original execute_grasp/evaluate_grasp, without changing their gates.
"""
import argparse
import copy
from datetime import datetime, timezone
import difflib
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

import run as frozen
from controller import ASSET_ROOT, Controller, build_model
from grasp_support_ab import verify_frozen, digest, integration_state, pose
from grasp_cone_ab import verify_files, npz

FREEZE='1e4d633730f62a398513d33991fa96f9c4977daa'
ACTIONS=('close_width_rate_m_s','hold_compression_target_force_N_each','expert_peak_joint_target_speed_rad_s')
CORE=('run.py','controller.py','config.json','grasp.json','base-filter-approval.json')


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')


def candidate_xml(text):
    root=ET.fromstring(text);opts=root.findall('option')
    if len(opts)!=1 or any(k in opts[0].attrib for k in ('cone','impratio')):
        raise ValueError('Expected the frozen builder default options')
    start=text.index('<option ');stop=text.index('/>',start)
    result=text[:stop]+('' if text[stop-1].isspace() else ' ')+'cone="elliptic" impratio="10" '+text[stop:]
    changed=ET.fromstring(result);option=changed.find('option')
    if option.attrib.pop('cone')!='elliptic' or option.attrib.pop('impratio')!='10' or ET.tostring(root)!=ET.tostring(changed):
        raise ValueError('Unapproved XML difference')
    return result


def compare_models(a,b):
    fields=[]
    for name in dir(a):
        if name.startswith('_'):continue
        x,y=getattr(a,name),getattr(b,name)
        if isinstance(x,(np.ndarray,int,float,str)):
            if not np.array_equal(x,y):raise ValueError('Compiled field differs: '+name)
            fields.append(name)
    changed=[n for n in dir(a.opt) if not n.startswith('_') and not callable(getattr(a.opt,n)) and
             not np.array_equal(getattr(a.opt,n),getattr(b.opt,n))]
    if changed!=['cone','impratio'] or int(a.opt.cone)!=0 or a.opt.impratio!=1 or int(b.opt.cone)!=1 or b.opt.impratio!=10:
        raise ValueError('Expected only default -> elliptic/impratio10: '+str(changed))
    return dict(option_differences=changed,identical_public_model_fields=fields)


def derived_model(path,cfg,*,exclude_base_link1=False,cube=None):
    """Build the original normal-table scene, then change two diagnostic options."""
    if not exclude_base_link1:raise ValueError('Specific approved base/link1 filter required')
    original=build_model(path,cfg,exclude_base_link1=True,cube=cube)
    text=path.read_text();base=path.with_name(path.stem+'-default.xml')
    with base.open('x') as stream:stream.write(text)
    path.write_text(candidate_xml(text))
    candidate=mujoco.MjModel.from_xml_path(str(path))
    comparison=compare_models(original,candidate)
    write(path.with_suffix('.comparison.json'),comparison)
    with path.with_suffix('.diff').open('x') as stream:
        stream.write(''.join(difflib.unified_diff(text.splitlines(True),path.read_text().splitlines(True),fromfile=base.name,tofile=path.name)))
    return candidate


def action_protocol(original, action):
    value=copy.deepcopy(original)
    if action is None:return value
    if set(action)!= {'key','value'} or action['key'] not in ACTIONS:
        raise ValueError('Exactly one authorized action scalar required')
    scalar=action['value'];key=action['key']
    if isinstance(scalar,bool) or not isinstance(scalar,(int,float)) or not np.isfinite(scalar) or scalar<=0 or scalar==original[key]:
        raise ValueError('Action candidate must be one finite positive changed scalar')
    value[key]=scalar
    return value


def sealed_case(frozen_plan,entry,label):
    """Bind a registration to the exact Gate0 task/initial condition/trajectory."""
    strategy=entry['strategy'];condition=entry['condition'];action=entry['hypothesis'].get('action')
    if strategy not in ('R0','R1','R2') or entry['label']!=label:
        raise ValueError('Invalid registered strategy or label')
    expected_label=strategy if condition=='fixed' else strategy+'_'+condition
    if label!=expected_label or condition not in ('fixed','offset_positive','offset_negative'):
        raise ValueError('Invalid registered condition')
    if (strategy=='R0' and action is not None) or (strategy!='R0' and action is None):
        raise ValueError('Strategy/action mismatch')
    case=next(c for c in frozen_plan['conditions_and_plans'] if c['id']==condition)
    if entry['case']!=case or entry['protocol']!=action_protocol(frozen_plan['protocol'],action):
        raise ValueError('Registered task, initial state, plan or protocol differs from Gate0')
    return case


def verify_runtime():
    verify_frozen()
    if platform.python_version()!='3.11.9' or mujoco.__version__!='3.2.3':raise ValueError('Frozen environment mismatch')
    for name in CORE:
        path=Path(__file__).with_name(name)
        expected=subprocess.check_output(['git','show',FREEZE+':experiments/dynamics/'+name],cwd=ASSET_ROOT)
        if path.read_bytes()!=expected:raise ValueError('Frozen source differs: '+name)
    approval=json.loads(Path(__file__).with_name('base-filter-approval.json').read_text())
    if not approval['accepted'] or approval['scope']!=['base_link','link1']:raise ValueError('Invalid base filter approval')


def snapshot(model,data,controller):
    return dict(integration_state=integration_state(model,data),time=np.array([data.time]),
        qpos=data.qpos.copy(),qvel=data.qvel.copy(),ctrl=data.ctrl.copy(),qacc_warmstart=data.qacc_warmstart.copy(),
        controller_requested=controller.requested.copy(),controller_command=controller.command.copy(),**pose(model,data))


def prepare(campaign):
    verify_runtime();campaign.mkdir(parents=True,exist_ok=False)
    gate=campaign/'gate0';gate.mkdir();(campaign/'registrations').mkdir();(campaign/'attempts').mkdir()
    cfg=json.loads(Path(__file__).with_name('config.json').read_text())
    protocol=json.loads(Path(__file__).with_name('grasp.json').read_text())
    cases=[];assets={}
    original_step=mujoco.mj_step
    def deny(*args,**kwargs):raise RuntimeError('Gate0 cannot integrate')
    mujoco.mj_step=deny
    try:
        for condition in protocol['initial_conditions']:
            cube=copy.deepcopy(protocol['cube']);cube['center']=(np.asarray(cube['center'])+np.r_[condition['offset_xy_m'],0]).tolist();cube['yaw_rad']=condition['yaw_rad']
            path=gate/(condition['id']+'.xml');model=derived_model(path,cfg,exclude_base_link1=True,cube=cube)
            controller=Controller(model,cfg);plan=frozen.make_grasp_plan(model,controller,cube,protocol)
            for q in [plan['initial_action']]+[q+[protocol['open_width_m']] for q in plan['approach']+plan['lift']]:controller.validate(q)
            data=frozen.reset(model,controller,plan['initial_action'])
            np.testing.assert_array_equal(data.qpos[9:12],cube['center'])
            table=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'table')
            np.testing.assert_array_equal(model.geom_pos[table],[-.32,0,.065])
            np.testing.assert_array_equal(model.geom_size[table],[.12,.12,.065])
            if abs(cube['center'][2]-cube['half_size'][2]-.13)>1e-14:raise ValueError('Cube must start on normal table')
            state=snapshot(model,data,controller);np.savez_compressed(gate/(condition['id']+'-initial.npz'),**state)
            write(gate/(condition['id']+'-initial-contacts.json'),frozen.contacts(model,data))
            for node in ET.fromstring(path.read_text()).iter():
                if node.get('file'):
                    asset=(path.parent/node.get('file')).resolve();assets[str(asset)]=digest(asset)
            cases.append(dict(id=condition['id'],cube=cube,plan=plan))
            print('Gate0 planned',condition['id'],'zero mj_step',flush=True)
    finally:mujoco.mj_step=original_step
    write(gate/'frozen-plan.json',dict(config=cfg,protocol=protocol,conditions_and_plans=cases))
    sources=campaign/'executed-source';sources.mkdir()
    for name in CORE+('full_grasp_loop.py',):
        (sources/name).write_bytes(Path(__file__).with_name(name).read_bytes())
    files={str(p.relative_to(campaign)):digest(p) for folder in [gate,sources] for p in folder.iterdir() if p.is_file()}
    write(campaign/'prepared.json',dict(issue=14,source_freeze=FREEZE,diagnostic_only=True,artifacts=files,
        asset_hashes=assets,runner_sha256=digest(__file__),maximum_episodes=5,maximum_action_candidates=2,
        actual_new_mj_steps=0,python=sys.version,mujoco=mujoco.__version__,argv=sys.argv))
    print('Gate0 PASS; prepared SHA256',digest(campaign/'prepared.json'),flush=True)


def verify_campaign(campaign):
    verify_runtime();prepared=json.loads((campaign/'prepared.json').read_text())
    verify_files(campaign,prepared['artifacts'])
    if digest(__file__)!=prepared['runner_sha256']:raise ValueError('Runner changed after Gate0 sealing')
    for name,value in prepared['asset_hashes'].items():
        if digest(name)!=value:raise ValueError('Source asset changed: '+name)
    return json.loads((campaign/'gate0/frozen-plan.json').read_text())


def register(campaign,label,condition,strategy,hypothesis_file):
    frozen_plan=verify_campaign(campaign)
    hypothesis=json.loads(hypothesis_file.read_text());action=hypothesis.get('action')
    if not hypothesis.get('prediction') or not hypothesis.get('rationale'):raise ValueError('Predeclare rationale and falsifiable prediction')
    if strategy not in ('R0','R1','R2'):raise ValueError('Only R0 and two independent candidates allowed')
    if condition=='fixed':
        if label!=strategy:raise ValueError('Fixed label must equal strategy')
        if strategy=='R0' and action is not None:raise ValueError('R0 must preserve v2 action')
        if strategy!='R0':
            r0=campaign/'R0/grasp-result.json'
            if not r0.is_file() or json.loads(r0.read_text())['status']=='PASS':raise ValueError('Candidates only after failed R0')
            if not action or not hypothesis.get('evidence'):raise ValueError('Candidate needs saved R0 evidence')
            for old in (campaign/'registrations').glob('R[12].json'):
                other=json.loads(old.read_text())
                if other['hypothesis']['action']['key']==action['key']:raise ValueError('No multi-value scan of one action variable')
    else:
        if condition not in ('offset_positive','offset_negative') or label!=strategy+'_'+condition:raise ValueError('Only the two frozen offsets allowed')
        prior=campaign/strategy/'grasp-result.json'
        if not prior.is_file() or json.loads(prior.read_text())['status']!='PASS':raise ValueError('Offsets require complete fixed PASS')
        registered=json.loads((campaign/'registrations'/(strategy+'.json')).read_text())
        if hypothesis!=registered['hypothesis']:raise ValueError('Offsets must reuse identical passing strategy')
    protocol=action_protocol(frozen_plan['protocol'],action)
    case=next(c for c in frozen_plan['conditions_and_plans'] if c['id']==condition)
    entry=dict(label=label,condition=condition,strategy=strategy,hypothesis=hypothesis,protocol=protocol,case=case,
        runner_sha256=digest(__file__),prepared_sha256=digest(campaign/'prepared.json'),hypothesis_file_sha256=digest(hypothesis_file),
        registered_utc=datetime.now(timezone.utc).isoformat(),actual_mj_steps=0)
    sealed_case(frozen_plan,entry,label)
    write(campaign/'registrations'/(label+'.json'),entry)
    print('REGISTERED',label,'SHA256',digest(campaign/'registrations'/(label+'.json')),flush=True)


def execute(campaign,label):
    plan=verify_campaign(campaign)
    entry=json.loads((campaign/'registrations'/(label+'.json')).read_text())
    if entry['prepared_sha256']!=digest(campaign/'prepared.json') or entry['runner_sha256']!=digest(__file__):raise ValueError('Registration seal changed')
    case=sealed_case(plan,entry,label)
    attempts=campaign/'attempts'
    if len(list(attempts.glob('*.json')))>=5:raise ValueError('Episode budget exhausted')
    if (campaign/label).exists():raise FileExistsError('Episode already exists, never retry or overwrite')
    # Exclusive per-episode reservation; interrupted attempts count, never resume.
    write(attempts/(label+'.json'),dict(label=label,started_utc=datetime.now(timezone.utc).isoformat(),registration_sha256=digest(campaign/'registrations'/(label+'.json'))))
    original_builder,original_recorder=frozen.build_model,frozen.Recorder
    episode=campaign/label
    def builder(path,*args,**kwargs):
        with (path.parent/'STARTED').open('x') as stream:stream.write('One bounded full physical episode; never resume/retry here.\n')
        model=derived_model(path,*args,**kwargs)
        (path.parent/'runner.executed.py').write_bytes(Path(__file__).read_bytes())
        write(path.parent/'registration.executed.json',entry)
        return model
    class Recorded(original_recorder):
        def __init__(self,*args):
            super().__init__(*args)
            self.observations=dict(time=[],**{k:[] for k in ('gripper_position_world_m','gripper_rotation_world','cube_position_in_gripper_m','cube_rotation_in_gripper')})
            self.warnings=[];self.parameters=(self.directory/'contact-parameters.jsonl').open('x');self.initial_saved=False
        def step(self,data,name):
            if not self.initial_saved:
                state=snapshot(self.model,data,self.controller)
                initial=npz(campaign/'gate0'/(entry['condition']+'-initial.npz'))
                for k in initial:np.testing.assert_array_equal(state[k],initial[k])
                np.savez_compressed(self.directory/'initial-state.npz',**state);self.initial_saved=True
            try:return super().step(data,name)
            except BaseException:
                np.savez_compressed(self.directory/'failure-state.npz',time=[data.time],qpos=data.qpos,qvel=data.qvel,ctrl=data.ctrl,
                    actuator_force=data.actuator_force,qfrc_applied=data.qfrc_applied,xfrc_applied=data.xfrc_applied,warning_numbers=data.warning.number)
                write(self.directory/'failure-state.json',dict(phase=name,actual_steps=self.actual_steps,saved_rows=len(self.rows['time']),error=traceback.format_exc()))
                try:write(self.directory/'failure-contacts.json',frozen.contacts(self.model,data))
                except Exception:pass  # Invalid solver state may not expose usable contacts.
                raise
        def record(self,data,name):
            events=super().record(data,name)
            self.observations['time'].append(float(data.time))
            for k,v in pose(self.model,data).items():self.observations[k].append(v)
            self.warnings.append(data.warning.number.copy())
            self.parameters.write(json.dumps(dict(time=float(data.time),contacts=[dict(dim=int(c.dim),friction5=c.friction.tolist()) for c in data.contact]))+'\n')
            if self.actual_steps%10000==0:print(label,'actual steps',self.actual_steps,'phase',name,flush=True)
            return events
        def finish(self,error=None):
            self.parameters.close();result=super().finish(error)
            np.savez_compressed(self.directory/'observations.npz',**{k:np.asarray(v) for k,v in self.observations.items()})
            np.savez_compressed(self.directory/'warnings.npz',time=np.asarray(self.observations['time']),numbers=np.asarray(self.warnings))
            return result
    frozen.build_model=builder;frozen.Recorder=Recorded
    result=None;error=None
    try:result=frozen.execute_grasp(episode,plan['config'],entry['protocol'],case['cube'],case['plan'])
    except BaseException:error=traceback.format_exc();raise
    finally:
        frozen.build_model, frozen.Recorder=original_builder,original_recorder
        if episode.exists():
            health_path=episode/'result.json'
            health=json.loads(health_path.read_text()) if health_path.is_file() else {}
            write(episode/'execution.json',dict(issue=14,label=label,diagnostic_only=True,argv=sys.argv,python=sys.version,mujoco=mujoco.__version__,
                actual_mj_steps=health.get('actual_mj_steps'),recorded_physics_samples=health.get('recorded_physics_samples'),
                wrapper_error=error,wrapper_end_utc=datetime.now(timezone.utc).isoformat(),
                registration_sha256=digest(campaign/'registrations'/(label+'.json')),prepared_sha256=digest(campaign/'prepared.json'),
                runner_sha256=digest(__file__),git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ASSET_ROOT,text=True).strip()))
            write(episode/'raw-manifest.json',{p.name:dict(sha256=digest(p),bytes=p.stat().st_size) for p in episode.iterdir() if p.is_file()})
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=['prepare','register','run'])
    parser.add_argument('--campaign',type=Path,required=True);parser.add_argument('--label');parser.add_argument('--condition',default='fixed')
    parser.add_argument('--strategy',choices=['R0','R1','R2']);parser.add_argument('--hypothesis-file',type=Path);args=parser.parse_args()
    if args.mode=='prepare':prepare(args.campaign.resolve())
    elif args.mode=='register':
        if not args.label or not args.strategy or args.hypothesis_file is None:parser.error('register needs label/strategy/hypothesis-file')
        register(args.campaign.resolve(),args.label,args.condition,args.strategy,args.hypothesis_file)
    else:
        if not args.label:parser.error('run needs label')
        result=execute(args.campaign.resolve(),args.label)
        if result['status']!='PASS':raise SystemExit(1)
