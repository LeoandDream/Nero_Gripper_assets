"""Native MuJoCo control/risk/grasp experiment; all execution uses mj_step."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import mujoco
import numpy as np

from controller import ASSET_ROOT, HERE, JOINTS, Controller, build_model, load_config


def write_json(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def asset_integrity():
    for line in (ASSET_ROOT/'SHA256SUMS').read_text().splitlines():
        digest,relative=line.split('  ',1)
        if hashlib.sha256((ASSET_ROOT/relative).read_bytes()).hexdigest()!=digest:
            raise ValueError('Protected asset changed: '+relative)


def reset(model,controller,initial):
    data=mujoco.MjData(model)
    mujoco.mj_resetData(model,data)
    controller.reset_target(initial)
    # Only legal episode initialization. Never called from the step loop.
    data.qpos[controller.qadr]=controller.physical_target
    mujoco.mj_forward(model,data)
    return data


def contacts(model,data):
    events=[]
    for i,c in enumerate(data.contact):
        force=np.zeros(6);mujoco.mj_contactForce(model,data,i,force)
        events.append({'geoms':[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,int(g)) for g in [c.geom1,c.geom2]],
                       'bodies':[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[g])) for g in [c.geom1,c.geom2]],
                       'distance_m':float(c.dist),'position_m':c.pos.tolist(),'normal':c.frame[:3].tolist(),'contact_frame':c.frame.tolist(),
                       'constraint_address':int(c.efc_address),'force_contact_frame':force.tolist()})
    return events


class Recorder:
    def __init__(self,directory,model,controller):
        self.directory=Path(directory);self.model=model;self.controller=controller
        self.rows={key:[] for key in ['time','phase','qpos','qvel','requested','command','physical_target','ctrl','actuator_force','qfrc_actuator','qfrc_constraint','qfrc_applied','xfrc_applied']}
        self.events=(self.directory/'contacts.jsonl').open('x')
        self.phases=[];self.actual_steps=0
        self.max_arm_velocity=0.;self.max_finger_velocity=0.;self.max_coupling_error=0.
        self.minimum_contact_distance=0.;self.max_base_normal_force=0.;self.base_generalized_force_max=0.
        self.contact_pairs={}

    def record(self,data,phase):
        c=self.controller
        if np.any(data.qfrc_applied) or np.any(data.xfrc_applied):
            raise ValueError('Unexpected external generalized/body forces')
        if not np.isfinite(np.r_[data.qpos,data.qvel,data.ctrl,data.actuator_force]).all():
            raise ValueError('Non-finite physical state/control')
        if np.any(data.warning.number):raise ValueError('MuJoCo numerical warning')
        if not np.all(np.abs(data.actuator_force[c.aids])<=c.force_limits+1e-9):
            raise ValueError('Actuator force limit exceeded')
        q=data.qpos[c.qadr];v=data.qvel[c.vadr]
        self.max_arm_velocity=max(self.max_arm_velocity,float(abs(v[:7]).max()))
        self.max_finger_velocity=max(self.max_finger_velocity,float(abs(v[7:]).max()))
        self.max_coupling_error=max(self.max_coupling_error,float(abs(q[7]+q[8])))
        events=contacts(self.model,data)
        for e in events:
            self.minimum_contact_distance=min(self.minimum_contact_distance,e['distance_m'])
            pair=' / '.join(sorted(e['geoms']))
            self.contact_pairs[pair]=self.contact_pairs.get(pair,0)+1
            if set(e['bodies'])=={'base_link','link1'}:
                self.max_base_normal_force=max(self.max_base_normal_force,abs(e['force_contact_frame'][0]))
                world_force=np.asarray(e['normal'])*e['force_contact_frame'][0]
                jac=[]
                for body in e['bodies']:
                    p=np.zeros((3,self.model.nv));r=np.zeros_like(p)
                    bid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,body)
                    mujoco.mj_jac(self.model,data,p,r,np.asarray(e['position_m']),bid)
                    jac.append(p)
                self.base_generalized_force_max=max(self.base_generalized_force_max,float(abs((jac[1]-jac[0]).T@world_force).max()))
        self.events.write(json.dumps({'time':float(data.time),'phase':phase,'events':events})+'\n')
        values={'time':float(data.time),'phase':phase,'qpos':data.qpos.copy(),'qvel':data.qvel.copy(),
                'requested':c.requested.copy(),'command':c.command.copy(),'physical_target':c.physical_target,
                'ctrl':data.ctrl.copy(),'actuator_force':data.actuator_force.copy(),'qfrc_actuator':data.qfrc_actuator.copy(),
                'qfrc_constraint':data.qfrc_constraint.copy(),'qfrc_applied':data.qfrc_applied.copy(),'xfrc_applied':data.xfrc_applied.copy()}
        for key,value in values.items():self.rows[key].append(value)
        return events

    def step(self,data,name):
        self.controller.apply(data)
        before=float(data.time)
        mujoco.mj_step(self.model,data)
        self.actual_steps+=1
        if not np.isclose(data.time,before+self.model.opt.timestep,rtol=0,atol=1e-10):
            raise ValueError('Physics time reset or discontinuity')
        mujoco.mj_forward(self.model,data)
        return self.record(data,name)

    def phase(self,data,name,target,duration_s,*,loaded_fingers=False,motion_duration_s=None):
        c=self.controller;c.set_target(target)
        goal=c.requested.copy();initial=c.command.copy()
        steps=int(round(duration_s/self.model.opt.timestep))
        period=int(round(c.cfg['command_period_s']/self.model.opt.timestep))
        start=float(data.time);start_row=len(self.rows['time'])
        for k in range(steps):
            if k%period==0:
                if motion_duration_s is not None:
                    progress=min(k*self.model.opt.timestep/motion_duration_s,1.)
                    blend=.5-.5*np.cos(np.pi*progress)
                    c.set_target(initial+blend*(goal-initial))
                c.advance_command()
            self.step(data,name)
        q=np.asarray(self.rows['qpos'][start_row:])[:,c.qadr]
        target_q=np.r_[np.asarray(target[:7]),target[7]/2,-target[7]/2]
        settled=q[-min(len(q),int(.25/self.model.opt.timestep)):]
        error=np.max(abs(settled-target_q),axis=0)
        limits=c.cfg['acceptance']
        result={'phase':name,'start_time_s':start,'end_time_s':float(data.time),'target_8d':list(map(float,target)),
                'settled_max_error_9d':error.tolist(),'loaded_fingers':loaded_fingers,'motion_duration_s':motion_duration_s,
                'status':'PASS' if float(error[:7].max())<=limits['settled_arm_error_rad'] and (loaded_fingers or float(error[7:].max())<=limits['settled_finger_error_m']) else 'FAIL'}
        self.phases.append(result)
        return result

    def finish(self,error=None):
        self.events.close()
        np.savez_compressed(self.directory/'trajectory.npz',**{k:np.asarray(v) for k,v in self.rows.items()})
        a=self.controller.cfg['acceptance']
        health=(self.max_coupling_error<=a['maximum_coupling_error_m'] and self.max_arm_velocity<=a['maximum_arm_velocity_rad_s'] and self.max_finger_velocity<=a['maximum_finger_velocity_m_s'])
        result={'status':'INTERRUPTED' if error else ('PASS' if health and all(p['status']=='PASS' for p in self.phases) else 'FAIL'),
                'error':error,'actual_mj_steps':self.actual_steps,'recorded_physics_samples':len(self.rows['time']),
                'simulation_time_s':float(self.rows['time'][-1]) if self.rows['time'] else 0,
                'max_arm_velocity_rad_s':self.max_arm_velocity,'max_finger_velocity_m_s':self.max_finger_velocity,
                'max_coupling_error_m':self.max_coupling_error,'minimum_contact_distance_m':self.minimum_contact_distance,
                'max_base_link_link1_normal_force_N':self.max_base_normal_force,
                'base_contact_normal_generalized_force_max':self.base_generalized_force_max,
                'contact_pair_physics_step_counts':self.contact_pairs,'phases':self.phases,
                'external_force_shortcuts':False,'qpos_writes_during_execution':0,'hardware_fidelity':'UNKNOWN'}
        write_json(self.directory/'result.json',result)
        return result


def execute(directory,cfg,phases,*,filtered=False,cube=None,diagnostic=False):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    model=build_model(directory/'model.xml',cfg,exclude_base_link1=filtered,cube=cube)
    control=Controller(model,cfg);data=reset(model,control,cfg['initial_action'])
    write_json(directory/'protocol.json',{'config':cfg,'initial_action':cfg['initial_action'],'phases':phases,
                                        'explicit_base_filter':filtered,'diagnostic_candidate_only':diagnostic,
                                        'asset_sha256':hashlib.sha256((ASSET_ROOT/'mjcf/nero_gripper.xml').read_bytes()).hexdigest(),
                                        'mjcf_ctrl_order':JOINTS,'raw_ctrl_units':['Nm']*7+['N']*2,
                                        'mujoco_version':mujoco.__version__,'cube':cube})
    recorder=Recorder(directory,model,control)
    error=None;stop=None
    try:
        for phase in phases:recorder.phase(data,phase['name'],phase['target'],phase['duration_s'])
    except (Exception,KeyboardInterrupt) as exc:
        error=traceback.format_exc()
        if isinstance(exc,KeyboardInterrupt):stop=exc
    finally:
        result=recorder.finish(error)
    print(directory.name,result['status'],'steps',result['actual_mj_steps'],'base_force_N',result['max_base_link_link1_normal_force_N'],flush=True)
    if stop is not None:raise stop
    return result


def risk(directory,cfg):
    initial=cfg['initial_action']
    positive=np.asarray(initial,float);positive[0]=.05
    negative=np.asarray(initial,float);negative[0]=-.05
    phases=[{'name':'hold','target':initial,'duration_s':1.},
            {'name':'joint1_positive','target':positive.tolist(),'duration_s':2.},
            {'name':'joint1_negative','target':negative.tolist(),'duration_s':2.}]
    write_json(directory/'frozen-protocol.json',{'purpose':'unapproved diagnostic comparison, not adopted control/grasp baseline','phases':phases,'config':cfg})
    before=execute(directory/'unfiltered',cfg,phases)
    after=execute(directory/'single-pair-exclusion-candidate',cfg,phases,filtered=True,diagnostic=True)
    physical_fields=['body_mass','body_inertia','body_pos','body_quat','body_ipos','body_iquat','jnt_range','jnt_axis','jnt_pos','dof_damping','dof_armature','dof_frictionloss','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp','actuator_gainprm','actuator_biasprm','actuator_forcerange','eq_data']
    models=[mujoco.MjModel.from_xml_path(str(directory/name/'model.xml')) for name in ['unfiltered','single-pair-exclusion-candidate']]
    same={key:bool(np.array_equal(getattr(models[0],key),getattr(models[1],key))) for key in physical_fields}
    assert all(same.values())
    diff='<contact>\n  <exclude body1="base_link" body2="link1"/>\n</contact>\n'
    (directory/'candidate-diff.xml').write_text(diff)
    write_json(directory/'comparison.json',{'baseline':before,'candidate':after,'compiled_fields_unchanged':same,'only_contact_exclusion_changed':True,'adopted':False,'user_confirmation_required':True})


def control(directory,cfg,filtered=False):
    initial=np.asarray(cfg['initial_action'],float)
    phases=[{'name':'stationary_hold','target':initial.tolist(),'duration_s':2.}]
    for j in range(7):
        target=initial.copy();target[j]+=.05
        phases.extend([{'name':f'joint{j+1}_small_motion','target':target.tolist(),'duration_s':1.5},
                       {'name':f'joint{j+1}_return','target':initial.tolist(),'duration_s':1.}])
    combined=initial.copy();combined[:7]+=np.array([.04,.04,-.04,.04,-.04,.04,-.04])
    phases.extend([{'name':'multi_joint_motion','target':combined.tolist(),'duration_s':2.},
                   {'name':'multi_joint_return','target':initial.tolist(),'duration_s':1.5}])
    for name,width in [('open',.1),('half_open',.05),('closed',0),('reopen',.08)]:
        target=initial.copy();target[7]=width
        phases.append({'name':'gripper_'+name,'target':target.tolist(),'duration_s':3.5})
    return execute(directory/('approved-filter-control' if filtered else 'unfiltered-control'),cfg,phases,filtered=filtered)


def solve_ik(model,controller,position,rotation,seed):
    """Bounded SciPy least_squares on scratch data, never execution data."""
    scratch=mujoco.MjData(model)
    sid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'grasp_center')
    goal=np.asarray(position);desired=Rotation.from_matrix(rotation)
    def residual(q):
        scratch.qpos[controller.qadr[:7]]=q
        scratch.qpos[controller.qadr[7:]]=[.05,-.05]
        # Pure FK avoids contacts/dynamics on iterative scratch configurations.
        mujoco.mj_kinematics(model,scratch)
        actual=Rotation.from_matrix(scratch.site_xmat[sid].reshape(3,3))
        return np.r_[scratch.site_xpos[sid]-goal,.15*(desired.inv()*actual).as_rotvec()]
    lower,upper=controller.limits.T
    result=least_squares(residual,np.clip(seed,lower+1e-7,upper-1e-7),bounds=(lower,upper),
                         xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=5000)
    error=residual(result.x)
    if np.linalg.norm(error[:3])>1e-5 or np.linalg.norm(error[3:])/.15>1e-4:
        raise ValueError(f'IK pose unreachable under command envelope: residual {error.tolist()}')
    return result.x


def evaluate_grasp(rows,contact_rows,cube_qadr,initial_center,criteria,timestep):
    """Acceptance from saved physical states AND actual contact support."""
    phase=np.asarray(rows['phase']);q=np.asarray(rows['qpos'])
    position=q[:,cube_qadr:cube_qadr+3]
    hold=phase=='hold';hold_indices=np.flatnonzero(hold)
    forces=np.zeros((len(phase),2));other=np.zeros(len(phase),bool)
    minimum=0.;minimum_phase=None
    for i,row in enumerate(contact_rows):
        for e in row['events']:
            if 'cube' not in e['bodies']:continue
            body=next(b for b in e['bodies'] if b!='cube')
            force=max(0.,e['force_contact_frame'][0])
            if body in ['gripper_link1','gripper_link2']:
                forces[i,int(body[-1])-1]+=force
            elif force>criteria['minimum_finger_normal_force_N']:other[i]=True
            if body!='world' and e['distance_m']<minimum:
                minimum=e['distance_m'];minimum_phase=str(phase[i])
    bilateral=np.all(forces>=criteria['minimum_finger_normal_force_N'],axis=1)
    height=position[:,2]-initial_center[2]
    if len(hold_indices):
        hp=position[hold]
        drift=float(np.linalg.norm(hp-hp[0],axis=1).max())
        low=float(height[hold].min());fraction=float(bilateral[hold].mean())
    else:drift=None;low=None;fraction=0.
    checks={'complete_hold':bool(len(hold_indices)*timestep>=criteria['hold_duration_s']-timestep/2),
            'lift_height':bool(low is not None and low>=criteria['minimum_lift_m']),
            'hold_position_drift':bool(drift is not None and drift<=criteria['maximum_hold_drift_m']),
            'bilateral_finger_support':fraction>=criteria['minimum_bilateral_contact_fraction'],
            'no_nonfinger_support_during_hold':not bool(other[hold].any()),
            'robot_object_penetration':-minimum<=criteria['maximum_robot_object_penetration_m']}
    return {'status':'PASS' if all(checks.values()) else 'FAIL','criteria':criteria,'checks':checks,
            'failed_criteria':[k for k,v in checks.items() if not v],
            'maximum_lift_m':float(height.max()) if len(height) else None,
            'minimum_lift_during_hold_m':low,'hold_duration_s':len(hold_indices)*timestep,
            'maximum_hold_position_drift_m':drift,'bilateral_contact_fraction_during_hold':fraction,
            'minimum_robot_object_contact_distance_m':minimum,'maximum_penetration_phase':minimum_phase,
            'finger_normal_force_range_during_hold_N':[[float(f.min()),float(f.max())] for f in forces[hold].T] if len(hold_indices) else None,
            'nonfinger_support_samples_during_hold':int(other[hold].sum())}


def make_grasp_plan(model,c,cube,protocol):
    rotation=np.diag([-1.,1.,-1.]);center=np.asarray(cube['center'])
    # Fixed seed selected by earlier scratch FK feasibility, no simulated trial.
    seed=np.array([-.85,.56,1.18,1.84,-.33,.475,1.08])
    grasp_q=solve_ik(model,c,center,rotation,seed)
    n=protocol['vertical_waypoint_count']
    upward=[];q=grasp_q
    for dz in np.linspace(0,protocol['pregrasp_height_m'],n+1)[1:]:
        q=solve_ik(model,c,center+[0,0,dz],rotation,q);upward.append(q.tolist())
    approach=upward[-2::-1]+[grasp_q.tolist()]
    lifted=[];q=grasp_q
    for dz in np.linspace(0,protocol['lift_height_m'],n+1)[1:]:
        q=solve_ik(model,c,center+[0,0,dz],rotation,q);lifted.append(q.tolist())
    # Check actual source collision proxies on scratch path before execution.
    scratch=mujoco.MjData(model);previous=np.array(upward[-1]);bad=[]
    for target in approach+lifted:
        for alpha in np.linspace(0,1,31):
            scratch.qpos[c.qadr]=np.r_[(1-alpha)*previous+alpha*np.array(target),.05,-.05]
            mujoco.mj_forward(model,scratch)
            for e in contacts(model,scratch):
                if 'cube' not in e['bodies'] and e['distance_m']<-.00001:
                    bad.append({'bodies':e['bodies'],'distance_m':e['distance_m']})
        previous=np.array(target)
    if bad:raise ValueError('Scratch approach/lift has non-object collision: '+str(bad[:4]))
    return {'initial_action':upward[-1]+[.08],'approach':approach,'lift':lifted,
            'planning_method':'bounded SciPy least_squares in scratch MjData, source ranges plus conservative Joint2 envelope',
            'nonobject_scratch_collision_count':len(bad)}


def execute_grasp(directory,cfg,protocol,cube,plan):
    directory.mkdir(exist_ok=False)
    model=build_model(directory/'model.xml',cfg,exclude_base_link1=True,cube=cube)
    c=Controller(model,cfg);data=reset(model,c,plan['initial_action'])
    write_json(directory/'protocol.json',{'config':cfg,'grasp_protocol':protocol,'cube':cube,'joint_plan':plan,
        'mujoco_version':mujoco.__version__,'explicit_base_filter':True,'qpos_initialization_only':True,
        'cube_inertia':'homogeneous solid box, I=m*(edge_b_squared+edge_c_squared)/12',
        'eq_type':model.eq_type.tolist(),'nu':model.nu,'nq':model.nq,'nv':model.nv})
    recorder=Recorder(directory,model,c);error=None;trigger=None;stop=None
    def move(name,q,width,loaded=False):
        target=np.r_[q,width]
        expert_slew=np.r_[np.full(7,protocol['expert_peak_joint_target_speed_rad_s']),c.slew[7]]
        motion=max(np.pi/2*float(np.max(np.abs(target-c.command)/expert_slew)),.1)
        return recorder.phase(data,name,target,motion+.8,loaded_fingers=loaded,motion_duration_s=motion)
    try:
        move('open',plan['initial_action'][:7],protocol['open_width_m'])
        for q in plan['approach']:move('approach',q,protocol['open_width_m'])
        arm=plan['approach'][-1];period=int(round(cfg['command_period_s']/model.opt.timestep))
        width=protocol['open_width_m'];start=data.time
        for k in range(round(protocol['maximum_close_duration_s']/model.opt.timestep)):
            if k%period==0:
                width=max(0.,width-protocol['close_width_rate_m_s']*cfg['command_period_s'])
                c.set_target(np.r_[arm,width]);c.advance_command()
            events=recorder.step(data,'close')
            forces=np.zeros(2)
            for e in events:
                if 'cube' in e['bodies']:
                    for j in range(2):
                        if f'gripper_link{j+1}' in e['bodies']:forces[j]+=max(0.,e['force_contact_frame'][0])
            if np.all(forces>=protocol['contact_trigger_force_N_each']):
                measured=data.qpos[c.qadr[-2]]-data.qpos[c.qadr[-1]]
                width=max(0.,measured-2*protocol['hold_compression_target_force_N_each']/cfg['finger_kp_N_m'])
                trigger={'time_s':float(data.time),'measured_source_width_m':float(measured),'hold_target_width_m':float(width),'finger_normal_forces_N':forces.tolist()}
                break
        recorder.phases.append({'phase':'close','start_time_s':float(start),'end_time_s':float(data.time),'loaded_fingers':True,
                                'status':'PASS' if trigger else 'FAIL','bilateral_contact_trigger':trigger})
        recorder.phase(data,'grasp_settle',np.r_[arm,width],1.,loaded_fingers=True)
        for q in plan['lift']:move('lift',q,width,loaded=True)
        recorder.phase(data,'hold',np.r_[plan['lift'][-1],width],protocol['criteria']['hold_duration_s'],loaded_fingers=True)
        move('release',plan['lift'][-1],protocol['open_width_m'])
    except (Exception,KeyboardInterrupt) as exc:
        error=traceback.format_exc()
        if isinstance(exc,KeyboardInterrupt):stop=exc
    finally:
        control_result=recorder.finish(error)
    if recorder.rows['time']:
        jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'cube_free')
        event_rows=[json.loads(line) for line in (directory/'contacts.jsonl').read_text().splitlines()]
        result=evaluate_grasp(recorder.rows,event_rows,int(model.jnt_qposadr[jid]),cube['center'],protocol['criteria'],model.opt.timestep)
    else:result={'status':'INTERRUPTED','failed_criteria':['no_physics_samples']}
    result['control_health_status']=control_result['status'];result['actual_mj_steps']=control_result['actual_mj_steps']
    result['bilateral_contact_trigger']=trigger;result['error']=error
    if error:result['status']='INTERRUPTED'
    elif control_result['status']!='PASS':result['status']='FAIL'
    write_json(directory/'grasp-result.json',result)
    print(directory.name,result['status'],'steps',result['actual_mj_steps'],'criteria failures',result['failed_criteria'],flush=True)
    if stop is not None:raise stop
    return result


def grasp(directory,cfg,plan_from_run=None):
    protocol=json.loads((HERE/'grasp.json').read_text())
    if plan_from_run:
        source=HERE/'runs'/plan_from_run/'frozen-protocol.json'
        frozen=json.loads(source.read_text())
        if frozen['config']!=cfg or any(frozen['protocol'][key]!=protocol[key] for key in ['cube','criteria','initial_conditions','pregrasp_height_m','lift_height_m']):
            raise ValueError('Cached plan physics/task/initial conditions differ')
        cases=frozen['conditions_and_plans']
        write_json(directory/'plan-reuse.json',{'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                                               'purpose':'reuse identical previously bounded IK waypoints; expert timing/compression revision disclosed'})
    else:
        planning_model=build_model(directory/'planning-model.xml',cfg,exclude_base_link1=True,cube=protocol['cube'])
        c=Controller(planning_model,cfg);cases=[]
        for condition in protocol['initial_conditions']:
            cube=json.loads(json.dumps(protocol['cube']))
            cube['center']=(np.asarray(cube['center'])+np.r_[condition['offset_xy_m'],0]).tolist()
            cube['yaw_rad']=condition['yaw_rad']
            cases.append({'id':condition['id'],'cube':cube,'plan':make_grasp_plan(planning_model,c,cube,protocol)})
            print('Scratch plan ready:',condition['id'],flush=True)
    # Freeze every condition and plan before any mj_step of the first attempt.
    write_json(directory/'frozen-protocol.json',{'protocol':protocol,'conditions_and_plans':cases,'config':cfg})
    results=[]
    for case in cases:
        result=execute_grasp(directory/case['id'],cfg,protocol,case['cube'],case['plan'])
        results.append({'id':case['id'],**result})
        if case['id']=='fixed' and result['status']!='PASS':break
    summary={'status':'PASS' if len(results)==len(cases) and all(r['status']=='PASS' for r in results) else 'FAIL',
             'planned_conditions':len(cases),'executed_conditions':len(results),'successful_conditions':sum(r['status']=='PASS' for r in results),
             'results':results,'frozen_conditions_not_executed':[c['id'] for c in cases[len(results):]]}
    write_json(directory/'summary.json',summary)
    return summary


def replay(directory):
    """Offline rendering of the saved integrated states; no simulation execution."""
    from PIL import Image, ImageDraw
    os.environ.setdefault('MUJOCO_GL','egl')
    # EGL selection must be in the shell before importing mujoco on some hosts.
    video=directory/'replay.mp4'
    if video.exists():raise FileExistsError(video)
    model=mujoco.MjModel.from_xml_path(str(directory/'model.xml'))
    saved=np.load(directory/'trajectory.npz',allow_pickle=False)
    data=mujoco.MjData(model)
    model.vis.global_.offwidth=960;model.vis.global_.offheight=640
    camera=mujoco.MjvCamera();camera.lookat[:]=[-.18,0,.36]
    camera.distance=.85;camera.azimuth=135;camera.elevation=-20
    option=mujoco.MjvOption();option.geomgroup[0]=0
    dt=model.opt.timestep;fps=20;indices=np.arange(0,len(saved['time']),round(1/fps/dt))
    frame_map=[];hold_saved=False
    command=['ffmpeg','-hide_banner','-loglevel','error','-n','-f','rawvideo','-pixel_format','rgb24',
             '-video_size','960x640','-framerate',str(fps),'-i','pipe:0','-an','-c:v','libx264','-crf','20','-pix_fmt','yuv420p',str(video)]
    with (directory/'replay-ffmpeg.log').open('x') as log:
        process=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=log)
        try:
            with mujoco.Renderer(model,height=640,width=960) as renderer:
                for i in indices:
                    # Legal offline replay only: this data is never passed to mj_step.
                    data.qpos[:]=saved['qpos'][i];data.qvel[:]=saved['qvel'][i]
                    data.ctrl[:]=saved['ctrl'][i];data.time=float(saved['time'][i])
                    mujoco.mj_forward(model,data)
                    renderer.update_scene(data,camera=camera,scene_option=option)
                    frame=Image.fromarray(renderer.render());draw=ImageDraw.Draw(frame)
                    draw.rectangle([0,0,960,43],fill=(240,242,245))
                    text=f"PHYSICS STATE REPLAY | t={data.time:.2f}s | {saved['phase'][i]} | source width={saved['command'][i,7]:.4f}m"
                    draw.text((12,8),text,fill=(20,20,20))
                    process.stdin.write(np.asarray(frame).tobytes())
                    frame_map.append({'frame':len(frame_map),'physics_row':int(i),'simulation_time_s':data.time})
                    if saved['phase'][i]=='hold' and not hold_saved:
                        frame.save(directory/'hold.png');hold_saved=True
        finally:
            process.stdin.close()
            if process.wait()!=0:raise RuntimeError('ffmpeg replay encoding failed')
    write_json(directory/'replay.json',{'mode':'offline replay of actual recorded mj_step states','actual_mj_steps_in_replay':0,
        'fps':fps,'camera':{'lookat':camera.lookat.tolist(),'distance':camera.distance,'azimuth':camera.azimuth,'elevation':camera.elevation},
        'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [directory/'model.xml',directory/'trajectory.npz']},
        'frames':frame_map,'qpos_writes':'one per displayed frame, offline data only; not a control test'})
    print('Replay saved:',video,flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=['risk','control','grasp','replay'],required=True)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--filtered',action='store_true',help='use the specifically approved base/link1 exclusion')
    parser.add_argument('--episode',default='fixed',help='episode within a saved grasp run, for offline replay')
    parser.add_argument('--plan-from-run',help='reuse frozen IK plans with identical object, criteria and initial conditions')
    args=parser.parse_args()
    if args.stage=='grasp' and not args.filtered:parser.error('grasp requires the approved --filtered baseline')
    if Path(args.run_id).name!=args.run_id:parser.error('run-id must be a single directory name')
    if args.plan_from_run and Path(args.plan_from_run).name!=args.plan_from_run:parser.error('plan-from-run must be a single directory name')
    asset_integrity()
    if args.stage=='replay':
        if Path(args.episode).name!=args.episode:parser.error('episode must be a single directory name')
        replay(HERE/'runs'/args.run_id/args.episode)
        return
    directory=HERE/'runs'/args.run_id;directory.mkdir(parents=True,exist_ok=False)
    cfg=load_config();write_json(directory/'config.executed.json',cfg)
    if args.filtered:
        approval=json.loads((HERE/'base-filter-approval.json').read_text())
        if not approval.get('accepted') or approval['scope']!=['base_link','link1']:
            raise ValueError('Missing specific base/link1 approval')
        write_json(directory/'approval.executed.json',approval)
    source_files=[Path(__file__),HERE/'controller.py',HERE/'config.json',HERE/'grasp.json']
    snapshot=directory/'executed-source';snapshot.mkdir()
    for source in source_files:(snapshot/source.name).write_bytes(source.read_bytes())
    write_json(directory/'execution.json',{'argv':sys.argv,'python':sys.version,'executable':sys.executable,
                 'source_files':{p.name:hashlib.sha256((snapshot/p.name).read_bytes()).hexdigest() for p in source_files},
                 'baseline_asset_commit':cfg['baseline_commit']})
    try:
        if args.stage=='risk':risk(directory,cfg)
        else:
            result=grasp(directory,cfg,args.plan_from_run) if args.stage=='grasp' else control(directory,cfg,args.filtered)
            if result['status']!='PASS':raise SystemExit(1)
    except (Exception,KeyboardInterrupt):
        write_json(directory/'interruption.json',{'error':traceback.format_exc()})
        raise


if __name__=='__main__':main()
