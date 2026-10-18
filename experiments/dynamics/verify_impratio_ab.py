"""Issue #12 evidence verification; zero integration and zero force solving."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from grasp_impratio_ab import check_models, impratio_xml, state_sha, STATE_SHA
from grasp_cone_ab import digest, npz, verify_files
from grasp_support_ab import verify_frozen
from diagnose_ab import load_ab_episode


def verify(source, run, registration):
    # No dynamic tests: only compile, read saved records and compare exact bytes.
    verify_frozen()
    saved = json.loads(registration.read_text())
    prepared = json.loads((run/'prepared.json').read_text())
    assert saved['prepared_sha256'] == digest(run/'prepared.json')
    assert saved['prediction_sha256'] == digest(run/'prediction.json')
    assert prepared == saved['prepared.json']
    verify_files(run, prepared['artifacts']); verify_files(source, prepared['source_hashes'])
    for group in ('execution_helper_hashes','referenced_asset_hashes'):
        for path, expected in prepared[group].items(): assert digest(path) == expected
    assert (run/'model.xml').read_text() == impratio_xml((source/'model.xml').read_text())
    model = mujoco.MjModel.from_xml_path(str(run/'model.xml'))
    comparison = check_models(mujoco.MjModel.from_xml_path(str(source/'model.xml')), model)
    old, new = load_ab_episode(source), load_ab_episode(run)
    for key in old[4]:
        if key != 'restored_integration_after_forward': np.testing.assert_array_equal(old[4][key],new[4][key])
    for key in ('config','grasp_protocol','cube','joint_plan','snapshot_time_s','constant_command_8d'):
        assert old[2][key] == new[2][key]
    assert state_sha(new[4]['integration_state']) == STATE_SHA
    for key in ('time','requested','command','physical_target','phase'):
        np.testing.assert_array_equal(old[0][key],new[0][key])
    trajectory = new[0]; n = len(trajectory['time']); assert n == 14100
    for key in ('qfrc_applied','xfrc_applied'): assert not np.any(trajectory[key])
    limits = model.actuator_forcerange[:,1]
    assert np.all(np.abs(trajectory['actuator_force']) <= limits+1e-9)
    assert np.all(np.abs(trajectory['ctrl']) <= model.actuator_ctrlrange[:,1]+1e-9)
    warnings = npz(run/'warnings.npz')
    np.testing.assert_array_equal(warnings['time'],trajectory['time'])
    assert warnings['numbers'].shape == (n,8) and not warnings['numbers'].any()
    execution = json.loads((run/'execution.json').read_text()); health = json.loads((run/'result.json').read_text())
    assert execution['actual_mj_steps'] == health['actual_mj_steps'] == health['recorded_physics_samples'] == n
    assert execution['error'] is None and health['error'] is None
    assert execution['prediction_sha256'] == saved['prediction_sha256']
    assert execution['runner_sha256'] == digest(run/'runner.executed.py') == prepared['runner_sha256']
    assert health['qpos_writes_during_execution'] == 0 and not health['external_force_shortcuts']
    assert (run/'STARTED').is_file()
    criteria = new[2]['config']['acceptance']
    coupling = float(np.abs(trajectory['qpos'][:,7]+trajectory['qpos'][:,8]).max())
    arm_velocity = float(np.abs(trajectory['qvel'][:,:7]).max())
    finger_velocity = float(np.abs(trajectory['qvel'][:,7:9]).max())
    assert coupling == health['max_coupling_error_m']
    expected = 'PASS' if (coupling<=criteria['maximum_coupling_error_m'] and
        arm_velocity<=criteria['maximum_arm_velocity_rad_s'] and finger_velocity<=criteria['maximum_finger_velocity_m_s']) else 'FAIL'
    assert health['status'] == execution['control_health'] == expected and not health['phases']
    return dict(status='PASS_SAVED_EVIDENCE_CHECKS',new_mj_steps=0,force_solves=0,
        B_samples=n,full_pre_forward_sha256=STATE_SHA,only_option_difference=comparison['option_differences'],
        identical_model_fields=len(comparison['identical_public_model_fields']),
        saved_control_health=expected,source_hashes_unchanged=True,original_criteria_unchanged=True,
        prediction_sha256=saved['prediction_sha256'],prepared_sha256=saved['prepared_sha256'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--registration',type=Path,default=Path('reports/issue12-preregistration.json'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    result=verify(a.source,a.run,a.registration)
    with a.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result))
