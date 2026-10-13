"""Native MuJoCo control/risk/grasp experiment; all execution uses mj_step."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET

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
                       'distance_m':float(c.dist),'position_m':c.pos.tolist(),'normal':c.frame[:3].tolist(),
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

    def phase(self,data,name,target,duration_s):
        c=self.controller;c.set_target(target)
        steps=int(round(duration_s/self.model.opt.timestep))
        period=int(round(c.cfg['command_period_s']/self.model.opt.timestep))
        start=float(data.time);start_row=len(self.rows['time'])
        for k in range(steps):
            if k%period==0:c.advance_command()
            c.apply(data)
            before=float(data.time)
            mujoco.mj_step(self.model,data)
            self.actual_steps+=1
            if not np.isclose(data.time,before+self.model.opt.timestep,rtol=0,atol=1e-10):
                raise ValueError('Physics time reset or discontinuity')
            mujoco.mj_forward(self.model,data)
            self.record(data,name)
        q=np.asarray(self.rows['qpos'][start_row:])[:,c.qadr]
        target_q=np.r_[np.asarray(target[:7]),target[7]/2,-target[7]/2]
        settled=q[-min(len(q),int(.25/self.model.opt.timestep)):]
        error=np.max(abs(settled-target_q),axis=0)
        limits=c.cfg['acceptance']
        result={'phase':name,'start_time_s':start,'end_time_s':float(data.time),'target_8d':list(map(float,target)),
                'settled_max_error_9d':error.tolist(),'status':'PASS' if float(error[:7].max())<=limits['settled_arm_error_rad'] and float(error[7:].max())<=limits['settled_finger_error_m'] else 'FAIL'}
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


def execute(directory,cfg,phases,*,filtered=False,cube=None):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    model=build_model(directory/'model.xml',cfg,exclude_base_link1=filtered,cube=cube)
    control=Controller(model,cfg);data=reset(model,control,cfg['initial_action'])
    write_json(directory/'protocol.json',{'config':cfg,'initial_action':cfg['initial_action'],'phases':phases,
                                        'explicit_base_filter':filtered,'diagnostic_candidate_only':filtered,
                                        'asset_sha256':hashlib.sha256((ASSET_ROOT/'mjcf/nero_gripper.xml').read_bytes()).hexdigest(),
                                        'mjcf_ctrl_order':JOINTS,'raw_ctrl_units':['Nm']*7+['N']*2,
                                        'mujoco_version':mujoco.__version__,'cube':cube})
    recorder=Recorder(directory,model,control)
    error=None
    try:
        for phase in phases:recorder.phase(data,phase['name'],phase['target'],phase['duration_s'])
    except Exception:
        error=traceback.format_exc()
    result=recorder.finish(error)
    print(directory.name,result['status'],'steps',result['actual_mj_steps'],'base_force_N',result['max_base_link_link1_normal_force_N'],flush=True)
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
    after=execute(directory/'single-pair-exclusion-candidate',cfg,phases,filtered=True)
    physical_fields=['body_mass','body_inertia','body_pos','body_quat','body_ipos','body_iquat','jnt_range','jnt_axis','jnt_pos','dof_damping','dof_armature','dof_frictionloss','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp','actuator_gainprm','actuator_biasprm','actuator_forcerange','eq_data']
    models=[mujoco.MjModel.from_xml_path(str(directory/name/'model.xml')) for name in ['unfiltered','single-pair-exclusion-candidate']]
    same={key:bool(np.array_equal(getattr(models[0],key),getattr(models[1],key))) for key in physical_fields}
    assert all(same.values())
    diff='<contact>\n  <exclude body1="base_link" body2="link1"/>\n</contact>\n'
    (directory/'candidate-diff.xml').write_text(diff)
    write_json(directory/'comparison.json',{'baseline':before,'candidate':after,'compiled_fields_unchanged':same,'only_contact_exclusion_changed':True,'adopted':False,'user_confirmation_required':True})


def control(directory,cfg):
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
    return execute(directory/'unfiltered-control',cfg,phases)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=['risk','control'],required=True)
    parser.add_argument('--run-id',required=True)
    args=parser.parse_args()
    if Path(args.run_id).name!=args.run_id:parser.error('run-id must be a single directory name')
    asset_integrity()
    directory=HERE/'runs'/args.run_id;directory.mkdir(parents=True,exist_ok=False)
    cfg=load_config();write_json(directory/'config.executed.json',cfg)
    try:
        {'risk':risk,'control':control}[args.stage](directory,cfg)
    except Exception:
        write_json(directory/'interruption.json',{'error':traceback.format_exc()})
        raise


if __name__=='__main__':main()
