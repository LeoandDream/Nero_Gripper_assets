"""Issue14 saved-evidence checks; compile/read only, zero physics integration."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from full_grasp_loop import verify_campaign, sealed_case, candidate_xml, compare_models
from grasp_cone_ab import digest, npz, verify_files


def verify(campaign, analyses):
    plan=verify_campaign(campaign);steps=0;groups={}
    for label in ('R0','R1'):
        path=campaign/label;registration=json.loads((campaign/'registrations'/(label+'.json')).read_text())
        case=sealed_case(plan,registration,label)
        manifest=json.loads((path/'raw-manifest.json').read_text())
        verify_files(path,{name:value['sha256'] for name,value in manifest.items()})
        protocol=json.loads((path/'protocol.json').read_text())
        assert protocol['config']==plan['config'] and protocol['grasp_protocol']==registration['protocol']
        assert protocol['cube']==case['cube'] and protocol['joint_plan']==case['plan']
        assert (path/'model.xml').read_text()==candidate_xml((path/'model-default.xml').read_text())
        model=mujoco.MjModel.from_xml_path(str(path/'model.xml'))
        comparison=compare_models(mujoco.MjModel.from_xml_path(str(path/'model-default.xml')),model)
        a=npz(path/'trajectory.npz');n=len(a['time']);steps+=n
        np.testing.assert_allclose(a['time'],np.arange(1,n+1)*.001,rtol=0,atol=1e-8)
        for key in ('qpos','qvel','ctrl','actuator_force'):assert np.isfinite(a[key]).all()
        for key in ('qfrc_applied','xfrc_applied'):assert not a[key].any()
        assert np.all(np.abs(a['ctrl'])<=model.actuator_ctrlrange[:,1]+1e-9)
        assert np.all(np.abs(a['actuator_force'])<=model.actuator_forcerange[:,1]+1e-9)
        warnings=npz(path/'warnings.npz');np.testing.assert_array_equal(warnings['time'],a['time'])
        assert warnings['numbers'].shape==(n,8) and not warnings['numbers'].any()
        initial=npz(path/'initial-state.npz');expected=npz(campaign/'gate0/fixed-initial.npz')
        for key in expected:np.testing.assert_array_equal(initial[key],expected[key])
        health=json.loads((path/'result.json').read_text());grasp=json.loads((path/'grasp-result.json').read_text())
        execution=json.loads((path/'execution.json').read_text())
        assert health['actual_mj_steps']==health['recorded_physics_samples']==execution['actual_mj_steps']==n
        assert grasp['actual_mj_steps']==n and grasp['error'] is None and health['error'] is None
        assert all(c['status']=='PASS' for c in health['phases']) and health['status']=='PASS'
        assert health['qpos_writes_during_execution']==0 and health['external_force_shortcuts'] is False
        assert list(dict.fromkeys(a['phase']))==['open','approach','close','grasp_settle','lift','hold','release']
        assert np.count_nonzero(a['phase']=='hold')==2000
        assert execution['runner_sha256']==digest(path/'runner.executed.py')==digest(Path(__file__).with_name('full_grasp_loop.py'))
        analysis=json.loads((analyses/(label+'-analysis-final/summary.json')).read_text())
        assert analysis['frozen_evaluation']['agreement']=='PASS'
        assert analysis['status']==analysis['frozen_status']==grasp['status']
        verify_files(path,analysis['source_sha256_after'])
        groups[label]=dict(samples=n,original_grasp_status=grasp['status'],native_control_health=health['status'],
            model_identical_fields=len(comparison['identical_public_model_fields']),input_initial_state_equal=True,
            zero_external_forces=True,warning_numbers_max=[0]*8,actual_mj_step_count=n)
    p0=json.loads((campaign/'R0/protocol.json').read_text());p1=json.loads((campaign/'R1/protocol.json').read_text())
    changes=[k for k in p0['grasp_protocol'] if p0['grasp_protocol'][k]!=p1['grasp_protocol'][k]]
    assert changes==['expert_peak_joint_target_speed_rad_s']
    assert p0['grasp_protocol'][changes[0]]==.05 and p1['grasp_protocol'][changes[0]]==.2
    assert (campaign/'R0/model.xml').read_bytes()==(campaign/'R1/model.xml').read_bytes()
    assert sorted(p.stem for p in (campaign/'attempts').glob('*.json'))==['R0','R1']
    assert steps==83598 and (campaign/'STOPPED.json').is_file()
    return dict(status='PASS_SAVED_RECORD_CHECKS',groups=groups,actual_episodes=2,actual_mj_steps=steps,
        verification_mj_steps=0,verification_force_solves=0,only_action_difference=changes,
        source_assets_unchanged=True,original_criteria_unchanged=True,raw_manifests_match=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--campaign',type=Path,required=True)
    p.add_argument('--analyses',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    result=verify(a.campaign,a.analyses)
    with a.output.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result))
