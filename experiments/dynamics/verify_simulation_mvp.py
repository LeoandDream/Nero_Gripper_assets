"""Recheck a saved MVP episode and export compact evidence; zero integration."""
import argparse
import json
from pathlib import Path

import numpy as np

from simulation_mvp import ROOT, verify_assets, load_case, sha, write, runtime


def verify(output, report_path, evidence_path):
    if report_path.exists() or evidence_path.exists():
        raise FileExistsError('Verification outputs must be new; preserve previous evidence')
    report_path.parent.mkdir(parents=True,exist_ok=True)
    evidence_path.parent.mkdir(parents=True,exist_ok=True)
    verify_assets()
    case=load_case();run=output/'R1_smoke'
    manifest=json.loads((output/'raw-manifest.json').read_text())
    for name,record in manifest.items():
        assert sha(output/name)==record['sha256'],name
    a=np.load(run/'trajectory.npz',allow_pickle=False)
    rows={k:a[k] for k in a.files};n=len(rows['time'])
    np.testing.assert_allclose(rows['time'],np.arange(1,n+1)*.001,rtol=0,atol=1e-8)
    for key in ('qpos','qvel','ctrl','actuator_force'):
        assert np.isfinite(rows[key]).all()
    for key in ('qfrc_applied','xfrc_applied'):
        assert not rows[key].any()
    p=json.loads((run/'protocol.json').read_text())
    assert p['config']==case['config'] and p['grasp_protocol']==case['grasp_protocol']
    assert p['joint_plan']==case['joint_plan'] and p['cube']==case['cube']
    contact_rows=[json.loads(line) for line in (run/'contacts.jsonl').read_text().splitlines()]
    assert len(contact_rows)==n
    expected=runtime.evaluate_grasp(rows,contact_rows,9,case['cube']['center'],case['grasp_protocol']['criteria'],.001)
    saved=json.loads((run/'grasp-result.json').read_text())
    for key,value in expected.items():assert saved[key]==value,key
    before=json.loads((output/'before-run.json').read_text())
    for name,digest in before['project_imports_sha256'].items():
        assert sha(output/'executed-source'/name)==digest,name
        assert sha(ROOT/name)==digest,name
    health=json.loads((run/'result.json').read_text())
    assert health['actual_mj_steps']==health['recorded_physics_samples']==n
    phases=list(dict.fromkeys(rows['phase']))
    assert phases==['open','approach','close','grasp_settle','lift','hold','release']
    limits=np.array(case['config']['arm_force_limits_Nm']+[case['config']['finger_force_limit_N']]*2)
    assert np.all(abs(rows['actuator_force'])<=limits+1e-9)
    force=np.zeros((n,2));other=np.zeros(n,dtype=bool)
    depth=np.zeros(n);table=np.zeros(n);counts=np.zeros(n,dtype=int)
    max_normal_jump=0.;normal_jump_record=None;previous={};ambiguous_pairs=0
    samples=[];last_support=None
    for i,row in enumerate(contact_rows):
        np.testing.assert_allclose(row['time'],rows['time'][i],rtol=0,atol=1e-10)
        counts[i]=len(row['events']);current={}
        for e in row['events']:
            if 'cube' not in e['bodies']:continue
            body=next(b for b in e['bodies'] if b!='cube')
            f=max(0.,e['force_contact_frame'][0])
            if body in ('gripper_link1','gripper_link2'):
                force[i,int(body[-1])-1]+=f;depth[i]=max(depth[i],-e['distance_m'])
                pair=tuple(sorted(e['geoms']));current.setdefault(pair,[]).append(e)
            elif f>case['grasp_protocol']['criteria']['minimum_finger_normal_force_N']:
                other[i]=True
            if 'table' in e['geoms']:table[i]=max(table[i],-e['distance_m'])
        for pair,events in current.items():
            old=previous.get(pair,[])
            if len(old)!=1 or len(events)!=1:
                if old:ambiguous_pairs+=1
                continue
            # Stable pair, same geom ordering. No array-index contact matching.
            if old[0]['geoms']!=events[0]['geoms']:continue
            angle=float(np.degrees(np.arccos(np.clip(np.array(old[0]['normal'])@events[0]['normal'],-1,1))))
            if angle>max_normal_jump:
                max_normal_jump=angle;normal_jump_record=dict(row=i,pair=pair,angle_deg=angle,
                    previous_depth_mm=-old[0]['distance_m']*1000,depth_mm=-events[0]['distance_m']*1000)
        previous=current
        if np.all(force[i]>=.05):last_support=i
    hold=rows['phase']=='hold';release=np.flatnonzero(rows['phase']=='release')
    assert len(release)>0
    for label,index in [('initial',0),('trigger',int(np.searchsorted(rows['time'],saved['bilateral_contact_trigger']['time_s']))),
                        ('max_robot_penetration',int(np.argmax(depth))),('hold_start',int(np.flatnonzero(hold)[0])),
                        ('hold_end',int(np.flatnonzero(hold)[-1])),('release_end',n-1),('max_table_penetration',int(np.argmax(table)))]:
        samples.append(dict(label=label,row=index,qpos=rows['qpos'][index].tolist(),**contact_rows[index]))
    np.savez_compressed(evidence_path,time=rows['time'],phase=rows['phase'],
        cube_xyz=rows['qpos'][:,9:12],finger_force=force,other_support=other,robot_cube_penetration_m=depth,
        table_cube_penetration_m=table,contact_count=counts,
        coupling_error_m=rows['qpos'][:,7]+rows['qpos'][:,8],
        max_arm_velocity_rad_s=np.max(abs(rows['qvel'][:,:7]),axis=1),
        max_finger_velocity_m_s=np.max(abs(rows['qvel'][:,7:9]),axis=1))
    report=dict(status='PASS_SAVED_RECORD_CHECKS',verification_mj_steps=0,actual_episode_count=1,actual_mj_steps=n,
        raw_directory=str(output),raw_manifest_sha256=sha(output/'raw-manifest.json'),
        evidence_npz_sha256=sha(evidence_path),
        source_seal_matches=True,zero_external_forces=True,original_evaluation_recomputed=expected,
        control_health=health,grasp_result=saved,contact_count_max=int(counts.max()),
        max_adjacent_unique_pair_normal_jump_deg=max_normal_jump,normal_jump_record=normal_jump_record,
        unmatched_multi_contact_pair_transitions=ambiguous_pairs,
        cube_table_max_penetration_mm=float(table.max()*1000),table_max_phase=str(rows['phase'][np.argmax(table)]),
        release_diagnostics=dict(last_bilateral_support_row=last_support,
            end_cube_center_z_m=float(rows['qpos'][-1,11]),
            post_hold_cube_drop_mm=float((rows['qpos'][np.flatnonzero(hold)[-1],11]-rows['qpos'][-1,11])*1000),
            end_finger_forces_N=force[-1].tolist()),
        selected_raw_records=samples,verifier_sha256=sha(__file__))
    write(report_path,report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('selected_raw_records','control_health','grasp_result','original_evaluation_recomputed')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True)
    args=p.parse_args()
    verify(args.run.resolve(),args.report.resolve(),args.evidence.resolve())
