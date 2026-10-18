"""Issue #10: saved-state reduction, kinematics only; no force solve/integration.

All forces come from immutable post-step logs. Scratch qpos is used only for
mj_kinematics -> mj_comPos -> mj_jac; no model or source Run is written.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform
import sys
import traceback

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_ab import load_ab_episode

PINNED = {'trajectory.npz':'fea51467c8001321dddea0f16eb85bf60f9184f2fdb2d507fd9a1464dfad948d',
          'contacts.jsonl':'89aeb60697d7c6badde163965439b2b2fe37bffc02845b1a41d36a33ec5d43b5',
          'model.xml':'79f5f9116e812687e9c4592f9faf42e9660276f77329297e690be4a0db28d376'}


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path, value): Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def force_parts(event):
    frame=np.asarray(event['contact_frame']).reshape(3,3)
    f=np.asarray(event['force_contact_frame'][:3])
    sign=1 if event['bodies'][1]=='cube' else -1
    normal=sign*frame[0]*f[0]
    tangent=sign*(frame[1:].T@f[1:])
    return normal,tangent,normal+tangent


def rho_elliptic(force, mu):
    return None if force[0]<=1e-12 else float(np.linalg.norm(np.asarray(force[1:3])/mu)/force[0])


def local_point(rotation, origin, point):
    return np.asarray(rotation).T@(np.asarray(point)-np.asarray(origin))


def window_masks(ticks, table_tick):
    cuts=[0,141,1328,2164,int(table_tick)]
    return [(ticks>=lo)&(ticks<hi)&(ticks>0) for lo,hi in zip(cuts[:-1],cuts[1:])]


def stats(values):
    a=np.asarray(values)
    if not a.size:return None
    return dict(min=a.min(axis=0).tolist(),mean=a.mean(axis=0).tolist(),median=np.median(a,axis=0).tolist(),max=a.max(axis=0).tolist())


def analyze(source, output):
    output.mkdir(parents=True,exist_ok=False)
    source=source.resolve(); root=source.parents[3]
    if platform.python_version()!='3.11.9' or mujoco.__version__!='3.2.3':
        raise ValueError('Unexpected analysis Python/MuJoCo version')
    manifest_path=root/'debug/issue8/evidence-manifest.json'
    manifest=json.loads(manifest_path.read_text())
    inputs={}
    for p in list(source.iterdir())+[root/'reports/issue8-friction-cone-ab.md',root/'debug/issue8/analysis_001/comparison.json']:
        if not p.is_file():continue
        actual=digest(p); expected=manifest[str(p.relative_to(root))]['sha256']
        if actual!=expected:raise ValueError('Source manifest mismatch: '+str(p))
        inputs[str(p)]=actual
    for name,value in PINNED.items():
        if digest(source/name)!=value:raise ValueError('Pinned source mismatch: '+name)
    inputs[str(manifest_path)]=digest(manifest_path)
    a,rows,protocol,obs,initial=load_ab_episode(source)
    if len(rows)!=14100 or protocol['config']['timestep_s']!=.001:raise ValueError('Wrong frozen record')
    if np.any(a['qfrc_applied']) or np.any(a['xfrc_applied']):raise ValueError('Nonzero saved external forces')
    table=[i for i,r in enumerate(rows) if any('cube' in e['bodies'] and 'table' in e['geoms'] for e in r['events'])]
    if not table or table[0]!=4415:raise ValueError('Unexpected first table boundary; do not silently repartition')
    stop=table[0];dt=.001;t0=float(initial['time'][0])
    metadata=[json.loads(line) for line in (source/'contact-parameters.jsonl').read_text().splitlines()]
    if len(metadata)!=len(rows):raise ValueError('Missing contact parameters')
    for i,r in enumerate(rows):
        if metadata[i]['time']!=r['time'] or len(metadata[i]['contacts'])!=len(r['events']):raise ValueError('Unaligned parameter row')
    forbidden={}
    def deny(*args,**kwargs):
        forbidden['calls']=forbidden.get('calls',0)+1
        raise RuntimeError('Forbidden dynamics/force-solving call in readonly analysis')
    for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_forwardSkip','mj_inverse','mj_inverseSkip','mj_fwdConstraint'):
        setattr(mujoco,name,deny)
    m=mujoco.MjModel.from_xml_path(str(source/'model.xml'));d=mujoco.MjData(m)
    if int(m.opt.cone)!=1 or m.nq!=16 or m.nv!=15:raise ValueError('Wrong compiled frozen model')
    prepared=json.loads((source/'prepared.json').read_text())
    for path,expected in prepared['referenced_asset_hashes'].items():
        if digest(path)!=expected:raise ValueError('Referenced frozen asset changed: '+path)
        inputs[path]=expected
    ids={name:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,name) for name in ('cube','gripper_base','gripper_link1','gripper_link2')}
    if min(ids.values())<0 or not np.array_equal(m.body_ipos[ids['cube']],np.zeros(3)):raise ValueError('Unexpected COM convention')
    mass=float(m.body_mass[ids['cube']]);mg=-mass*float(m.opt.gravity[2])
    if mass!=.1 or not np.isclose(mg,.981,rtol=0,atol=1e-14):raise ValueError('Wrong frozen weight')
    results=[];contacts=[];max_pose_error=0.;max_force_error=0.;calls=0
    initial_contacts=json.loads((source/'elliptic-initial-contacts.json').read_text())
    initial_cube_R=initial['gripper_rotation_world']@initial['cube_rotation_in_gripper']
    def velocity(body,point,qvel):
        jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
        mujoco.mj_jac(m,d,jp,jr,np.asarray(point),body)
        return jp@qvel,jr@qvel
    for row in [-1]+list(range(stop)):
        q=initial['qpos'] if row<0 else a['qpos'][row]
        v=initial['qvel'] if row<0 else a['qvel'][row]
        elapsed=0. if row<0 else float(a['time'][row]-t0)
        events=initial_contacts if row<0 else rows[row]['events']
        d.qpos[:]=q;mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d);calls+=1
        cube=ids['cube'];gripper=ids['gripper_base']
        saved_g=initial['gripper_position_world_m'] if row<0 else obs['gripper_position_world_m'][row]
        max_pose_error=max(max_pose_error,float(np.abs(d.xpos[gripper]-saved_g).max()),float(np.abs(d.xpos[cube]-q[9:12]).max()))
        if max_pose_error>1e-12:raise ValueError('Saved/kinematic pose mismatch')
        vc,wc=velocity(cube,d.xipos[cube],v);vg,wg=velocity(gripper,d.xpos[gripper],v)
        np.testing.assert_allclose(vc,v[9:12],rtol=0,atol=1e-12)
        rr=d.xmat[gripper].reshape(3,3).T@d.xmat[cube].reshape(3,3)
        relative_R=rr@initial['cube_rotation_in_gripper'].T
        world_R=d.xmat[cube].reshape(3,3)@initial_cube_R.T
        angle=lambda R:float(np.arccos(np.clip((np.trace(R)-1)/2,-1,1)))
        normal=np.zeros((2,3));tangent=np.zeros((2,3));fn=np.zeros(2);counts=np.zeros(2,int)
        torque=np.zeros(3);penetration=0.;capacity=0.;rhos=[];slips=[]
        for j,e in enumerate(events):
            if 'cube' not in e['bodies']:continue
            other=e['bodies'][1-e['bodies'].index('cube')]
            if other not in ('gripper_link1','gripper_link2'):raise ValueError('Unexpected cube support before table')
            finger=int(other[-1])-1;otherid=ids[other]
            gid=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n) for n in e['geoms']]
            if row<0:
                if not np.array_equal(m.geom_friction[gid[0]],m.geom_friction[gid[1]]):raise ValueError('Unexpected initial friction mixing')
                mu=[float(m.geom_friction[gid[0],0])]*2;dim=3
            else:
                meta=metadata[row]['contacts'][j];mu=meta['friction5'][:2];dim=meta['dim']
            if dim!=3 or mu!=[1.,1.]:raise ValueError('Unexpected active contact dimensions/coefficients')
            nforce,tforce,force=force_parts(e);normal[finger]+=nforce;tangent[finger]+=tforce
            f=e['force_contact_frame'];fn[finger]+=max(f[0],0.);counts[finger]+=1
            sign=1 if e['bodies'][1]=='cube' else -1
            frame=np.asarray(e['contact_frame']).reshape(3,3);n=sign*frame[0]
            point=np.asarray(e['position_m']);lever=point-d.xipos[cube]
            tau=np.cross(lever,force);torque+=tau
            vcp=vc+np.cross(wc,lever);vfp,_=velocity(otherid,point,v)
            relative_v=vcp-vfp;slip=relative_v-n*np.dot(n,relative_v)
            rho=rho_elliptic(f,mu)
            if rho is not None:rhos.append(rho)
            slips.append(float(np.linalg.norm(slip)))
            penetration=max(penetration,-float(e['distance_m']))
            upper=float(f[0]*(n[2]+np.linalg.norm(np.asarray(mu)*frame[1:,2])))
            capacity+=upper
            item=dict(row=row,jsonl_line=None if row<0 else row+1,contact_index=j,elapsed_s=elapsed,finger=finger+1,
                Fn_N=f[0],Ft1_N=f[1],Ft2_N=f[2],Fz_normal_N=nforce[2],Fz_tangent_N=tforce[2],Fz_total_N=force[2],
                normal_on_cube_z=n[2],penetration_m=max(0.,-e['distance_m']),rho_elliptic=rho,
                at_geometric_boundary=None if rho is None else abs(rho-1)<=1e-10,
                slip_speed_m_s=float(np.linalg.norm(slip)),vertical_capacity_at_logged_Fn_N=upper)
            for prefix,vector in [('point_cube_local_m',local_point(d.xmat[cube].reshape(3,3),d.xpos[cube],point)),
                    ('point_finger_local_m',local_point(d.xmat[otherid].reshape(3,3),d.xpos[otherid],point)),
                    ('point_world_m',point),('normal_world',n),('relative_point_velocity_m_s',relative_v),
                    ('tangential_slip_m_s',slip),('torque_world_Nm',tau),('lever_world_m',lever)]:
                for axis,value in zip('xyz',vector):item[prefix+'_'+axis]=float(value)
            contacts.append(item)
        force=normal+tangent
        if row>=0:
            max_force_error=max(max_force_error,abs(float(force[:,2].sum()-a['qfrc_constraint'][row,11])))
            if max_force_error>1e-12:raise ValueError('Saved contact projection / generalized force mismatch')
        target=initial['controller_command'] if row<0 else a['command'][row]
        physical=np.r_[target[:7],target[7]/2,-target[7]/2]
        rec=dict(row=row,elapsed_s=elapsed,cube_world_vz_m_s=vc[2],gripper_world_vz_m_s=vg[2],relative_world_vz_m_s=vc[2]-vg[2],
            cube_world_delta_z_m=q[11]-initial['qpos'][11],gripper_world_delta_z_m=d.xpos[gripper,2]-initial['gripper_position_world_m'][2],
            relative_world_z_drop_m=initial['qpos'][11]-initial['gripper_position_world_m'][2]-(q[11]-d.xpos[gripper,2]),
            Fz_N=float(force[:,2].sum()),Fz_minus_mg_N=float(force[:,2].sum()-mg),
            relative_rotation_rad=angle(relative_R),world_rotation_rad=angle(world_R),cube_angular_speed_rad_s=float(np.linalg.norm(wc)),
            relative_angular_speed_rad_s=float(np.linalg.norm(wc-wg)),max_penetration_m=penetration,
            actual_slide_width_m=q[7]-q[8],target_slide_width_m=target[7],finger1_q_m=q[7],finger2_q_m=q[8],
            arm_error_max_rad=float(np.abs(q[:7]-physical[:7]).max()),coupling_m=abs(q[7]+q[8]),
            geometric_vertical_capacity_N=capacity,rho_max=np.nan if not rhos else max(rhos),
            slip_speed_max_m_s=0. if not slips else max(slips),contact_count=int(counts.sum()),
            torque_world_Nm=torque.copy(),cube_angular_velocity_world_rad_s=wc.copy())
        for k in range(2):
            rec.update({f'finger{k+1}_Fn_N':fn[k],f'finger{k+1}_count':counts[k],
                        f'finger{k+1}_Fz_normal_N':normal[k,2],f'finger{k+1}_Fz_tangent_N':tangent[k,2],
                        f'finger{k+1}_Fz_N':force[k,2]})
        results.append(rec)
        if row>0 and row%1000==0:print('read-only rows',row,flush=True)
    series={k:np.asarray([r[k] for r in results]) for k in results[0]}
    np.savez_compressed(output/'series.npz',**series)
    with (output/'contacts.csv').open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(contacts[0]));writer.writeheader();writer.writerows(contacts)
    ticks=np.rint(series['elapsed_s']/dt).astype(int)
    if not np.array_equal(ticks,np.arange(stop+1)):raise ValueError('Missing samples before table')
    masks=window_masks(ticks,4416);windows=[]
    for wi,mask in enumerate(masks):
        ix=np.flatnonzero(mask);start=int(ix[0]);end=int(ix[-1]);previous=start-1
        points=[p for p in contacts if p['row']>=0 and mask[p['row']+1]]
        duration=len(ix)*dt;dv=float(series['cube_world_vz_m_s'][end]-series['cube_world_vz_m_s'][previous])
        right=float(np.sum(series['Fz_minus_mg_N'][ix])*dt/mass)
        left=float(np.sum(series['Fz_minus_mg_N'][ix-1])*dt/mass)
        record=dict(window=wi+1,row_start=start-1,row_end=end-1,samples=len(ix),sum_step_durations_s=duration,
            first_sample_elapsed_s=float(series['elapsed_s'][start]),last_sample_elapsed_s=float(series['elapsed_s'][end]),
            initial_boundary_elapsed_s=float(series['elapsed_s'][previous]),
            relative_drop_start_m=float(series['relative_world_z_drop_m'][previous]),relative_drop_end_m=float(series['relative_world_z_drop_m'][end]),
            relative_vz_start_m_s=float(series['relative_world_vz_m_s'][previous]),relative_vz_end_m_s=float(series['relative_world_vz_m_s'][end]),
            cube_vz_start_m_s=float(series['cube_world_vz_m_s'][previous]),cube_vz_end_m_s=float(series['cube_world_vz_m_s'][end]),
            fraction_samples_Fz_above_mg=float(np.mean(series['Fz_minus_mg_N'][ix]>0)),
            approximate_momentum=dict(measured_world_delta_v_m_s=dv,right_post_force_delta_v_m_s=right,left_shifted_force_delta_v_m_s=left,
                trapezoid_force_delta_v_m_s=(right+left)/2,right_residual_m_s=right-dv,left_residual_m_s=left-dv,
                limitation='Endpoint instantaneous-force quadrature, NOT integration-time impulse or causal closure'),
            row_statistics={k:stats(series[k][ix]) for k in ['Fz_N','Fz_minus_mg_N','relative_world_vz_m_s','cube_world_vz_m_s',
                'max_penetration_m','relative_rotation_rad','world_rotation_rad','cube_angular_speed_rad_s','torque_world_Nm',
                'actual_slide_width_m','target_slide_width_m','coupling_m','arm_error_max_rad','contact_count','geometric_vertical_capacity_N']},
            fingers={})
        for finger in [1,2]:
            cp=[p for p in points if p['finger']==finger and p['Fn_N']>1e-12]
            record['fingers'][str(finger)]=dict(positive_contact_samples=len(cp),geometric_boundary_samples=sum(p['at_geometric_boundary'] for p in cp),
                contact_statistics={k:stats([p[k] for p in cp]) for k in ['point_cube_local_m_x','point_cube_local_m_y','point_cube_local_m_z',
                    'normal_on_cube_z','rho_elliptic','slip_speed_m_s','tangential_slip_m_s_z','lever_world_m_z']},
                row_mean_force={k:float(series[f'finger{finger}_{k}'][ix].mean()) for k in ['Fn_N','Fz_normal_N','Fz_tangent_N','Fz_N']})
        windows.append(record)
    samples={}
    for tick in [0,1,15,68,140,141,751,1327,1328,2163,2164,3000,4000,4415]:
        samples[str(tick)]={k:np.asarray(v[tick]).tolist() for k,v in series.items() if k!='rho_max' or np.isfinite(v[tick])}
    total_dv=float(series['cube_world_vz_m_s'][-1]-series['cube_world_vz_m_s'][0])
    total_right=float(np.sum(series['Fz_minus_mg_N'][1:])*dt/mass)
    summary=dict(status='PASS_READONLY_REDUCTION',diagnostic_only=True,physical_grasp_pass=None,
        source=str(source),source_hashes=inputs,script_sha256=digest(__file__),python=sys.version,mujoco=mujoco.__version__,numpy=np.__version__,
        mass_kg=mass,mg_N=mg,dt_s=dt,t0_s=t0,pretable_rows=stop,first_table_row=stop,first_table_elapsed_s=float(a['time'][stop]-t0),
        table_sample_excluded=True,window_definition='post-step elapsed ticks [0,141),[141,1328),[1328,2164),[2164,4416); t0 kept separately',
        new_mj_steps=0,force_solves=0,kinematic_evaluations=calls,forbidden_calls=forbidden.get('calls',0),
        kinematics_vs_saved_pose_max_error_m=max_pose_error,world_force_projection_error_N=max_force_error,
        missing_fields=['integration-time contact impulses/forces','original per-step efc_state and solver regularization arrays','material-point contact identity'],
        initial=results[0],windows=windows,key_samples=samples,
        total_momentum_approx=dict(measured_world_delta_v_m_s=total_dv,right_post_force_delta_v_m_s=total_right,
            residual_m_s=total_right-total_dv),
        notes=['Local contact coordinates describe solver contact samples, not persistent material-point trajectories.',
               'Kinematic relative slip is not an original solver stick/slip label.',
               'No new physics or contact force solve; no inferred strict impulse balance.'])
    # Convert ndarray-valued initial vectors before JSON serialization.
    summary['initial']={k:np.asarray(v).tolist() for k,v in summary['initial'].items() if k!='rho_max' or np.isfinite(v)}
    for path,expected in inputs.items():
        if digest(path)!=expected:raise ValueError('Input changed during read-only analysis: '+path)
    write(output/'summary.json',summary)
    print(json.dumps({'status':summary['status'],'pretable_rows':stop,'new_mj_steps':0,'force_solves':0,
        'initial_cube_vz_m_s':series['cube_world_vz_m_s'][0],'initial_relative_vz_m_s':series['relative_world_vz_m_s'][0],
        'final_relative_drop_m':series['relative_world_z_drop_m'][-1]}),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    if args.output_dir.exists():raise FileExistsError('Refuse existing output directory, including any original Run')
    try:analyze(args.source,args.output_dir)
    except Exception:
        # Only create a failure record inside this newly requested analysis output.
        if args.output_dir.is_dir() and not (args.output_dir/'summary.json').exists() and not (args.output_dir/'BLOCKED.json').exists():
            write(args.output_dir/'BLOCKED.json',{'status':'BLOCKED_UNKNOWN','error':traceback.format_exc(),'new_mj_steps':0})
        raise
