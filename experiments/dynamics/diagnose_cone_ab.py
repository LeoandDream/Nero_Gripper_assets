"""Issue #8 saved-record reduction. No integration or contact force solving."""
import argparse
import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from diagnose_ab import load_ab_episode, _analyze, contact_series
from grasp_cone_ab import digest, verify_files, utilization, OLD_HASHES
import run as frozen


def analyze(old_dir, new_dir, output):
    verify_files(old_dir, OLD_HASHES)
    prepared = json.loads((new_dir / 'prepared.json').read_text())
    verify_files(new_dir, prepared['artifacts'])
    if digest(new_dir / 'runner.executed.py') != prepared['runner_sha256']:
        raise ValueError('Executed source snapshot changed')
    loaded = {label: load_ab_episode(path) for label, path in [('pyramidal', old_dir), ('elliptic', new_dir)]}
    old, new = loaded['pyramidal'], loaded['elliptic']
    for k in ('config', 'grasp_protocol', 'cube', 'joint_plan', 'snapshot_time_s'):
        if old[2][k] != new[2][k]: raise ValueError('Protected protocol differs: ' + k)
    for k in old[4]:
        if k != 'restored_integration_after_forward': np.testing.assert_array_equal(old[4][k], new[4][k])
    for k in ('time', 'requested', 'command', 'physical_target'):
        np.testing.assert_array_equal(old[0][k], new[0][k])
    execution = json.loads((new_dir / 'execution.json').read_text())
    if execution['error'] or execution['actual_mj_steps'] != 14100:
        raise ValueError('Incomplete experiment: report UNKNOWN with partial records')
    with np.load(new_dir / 'warnings.npz') as w:
        np.testing.assert_array_equal(w['time'], new[0]['time'])
        if w['numbers'].shape != (14100, 8) or np.any(w['numbers']): raise ValueError('Numerical warning')
    meta = [json.loads(line) for line in (new_dir / 'contact-parameters.jsonl').read_text().splitlines()]
    if len(meta) != 14100: raise ValueError('Contact parameter record mismatch')
    output.mkdir(parents=True, exist_ok=False)
    complete = {}; report = dict(diagnostic_only=True, official_grasp_success=None,
        new_mj_steps_in_analysis=0, force_solves_in_analysis=0, groups={},
        time_and_requested_command_and_physical_targets_exactly_equal=True,
        raw_ctrl_max_difference=float(np.abs(old[0]['ctrl'] - new[0]['ctrl']).max()),
        raw_ctrl_note='Same controller/position targets; measured-state feedback gives different actuator efforts',
        source_dirs=dict(pyramidal=str(old_dir.resolve()), elliptic=str(new_dir.resolve())))
    fields = ['group','row','jsonl_line','event_index','elapsed_s','contact_group','dim','Fn_N','Ft1_N','Ft2_N','mu1','mu2','utilization','on_geometric_boundary','before_table']
    with (output / 'contact-utilization.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for label, path in [('pyramidal', old_dir), ('elliptic', new_dir)]:
            a, rows, p, obs, initial = loaded[label]
            if len(rows) != 14100: raise ValueError('Unexpected record length')
            diag = _analyze(path, loaded[label]); complete[label] = diag
            series, penetration = contact_series(rows)
            model = mujoco.MjModel.from_xml_path(str(path / 'model.xml'))
            if int(model.opt.cone) != (0 if label == 'pyramidal' else 1): raise ValueError('Wrong cone')
            if model.npair or model.opt.enableflags: raise ValueError('Unexpected pair/override')
            cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cube')
            gripper = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_base')
            scratch = mujoco.MjData(model)
            table_event = diag['events']['first_table_geometry']
            stop = len(rows) if table_event is None else table_event['row']
            drop = initial['qpos'][11] - initial['gripper_position_world_m'][2] - (a['qpos'][:, 11] - obs['gripper_position_world_m'][:, 2])
            def event(name):
                e = diag['events'][name]
                return None if e is None else dict(row=e['row'], elapsed_s=e['relative_time_s'],
                    relative_drop_m=e['cube_relative_z_drop_m'], cube_vz_m_s=e['state']['qvel'][11],
                    finger_normal_force_N=e['finger_normal_force_N'])
            samples = []
            for elapsed in (.068, .751, 1.317, 1.328, 14.1):
                i = round(elapsed / p['config']['timestep_s']) - 1
                if abs(a['time'][i] - p['snapshot_time_s'] - elapsed) > 1e-9: raise ValueError('Metric sample misaligned')
                # Kinematics/Jacobian only, no mj_forward/mj_step/force solve.
                scratch.qpos[:] = a['qpos'][i]; mujoco.mj_fwdPosition(model, scratch)
                jac = np.zeros((3, model.nv)); rot = np.zeros_like(jac)
                mujoco.mj_jacBody(model, scratch, jac, rot, gripper)
                gripper_v = jac @ a['qvel'][i]
                samples.append(dict(row=i, elapsed_s=float(a['time'][i]-p['snapshot_time_s']),
                    relative_world_z_drop_m=float(drop[i]), cube_world_delta_z_m=float(a['qpos'][i,11]-initial['qpos'][11]),
                    gripper_world_delta_z_m=float(obs['gripper_position_world_m'][i,2]-initial['gripper_position_world_m'][2]),
                    cube_world_vz_m_s=float(a['qvel'][i,11]), gripper_origin_world_vz_m_s=float(gripper_v[2]),
                    relative_world_vz_m_s=float(a['qvel'][i,11]-gripper_v[2]),
                    finger_normal_force_N=[float(series[g]['normal'][i]) for g in ('finger1','finger2')],
                    finger_world_Fz_N=[float(series[g]['world'][i,2]) for g in ('finger1','finger2')],
                    table_world_Fz_N=float(series['table']['world'][i,2]),
                    robot_penetration_m=float(penetration[i]), coupling_residual_m=float(abs(a['qpos'][i,7]+a['qpos'][i,8]))))
            rho_before = []; positive_all = 0; boundary_before = 0
            for i, row in enumerate(rows):
                if label == 'elliptic':
                    if meta[i]['time'] != row['time'] or len(meta[i]['contacts']) != len(row['events']): raise ValueError('Contact metadata alignment')
                for j, e in enumerate(row['events']):
                    if 'cube' not in e['bodies']: continue
                    other = e['bodies'][1-e['bodies'].index('cube')]
                    group = {'gripper_link1':'finger1','gripper_link2':'finger2'}.get(other, 'table' if 'table' in e['geoms'] else 'other')
                    gids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, g) for g in e['geoms']]
                    g0, g1 = gids
                    # This frozen model has equal-priority, identical coefficients on each pair.
                    # Reject unsupported mixing rather than pretending original solver metadata exists.
                    if model.geom_priority[g0] != model.geom_priority[g1] or not np.array_equal(model.geom_friction[g0], model.geom_friction[g1]):
                        raise ValueError('Unexpected heterogeneous contact parameters')
                    dim = max(int(model.geom_condim[g]) for g in gids)
                    mu = [float(model.geom_friction[g0,0])] * 2
                    if label == 'elliptic':
                        actual = meta[i]['contacts'][j]
                        if actual['dim'] != dim or actual['friction5'][:2] != mu: raise ValueError('Actual contact parameters differ')
                        mu = actual['friction5'][:2]
                    if dim != 3: raise ValueError('Unexpected contact dimension')
                    f = e['force_contact_frame']; rho = utilization(f, mu, label)
                    boundary = None if rho is None else abs(rho - 1) <= 1e-10
                    positive_all += rho is not None
                    if i < stop and group in ('finger1','finger2') and rho is not None:
                        rho_before.append(rho); boundary_before += boundary
                    writer.writerow(dict(group=label,row=i,jsonl_line=i+1,event_index=j,elapsed_s=row['time']-p['snapshot_time_s'],
                        contact_group=group,dim=dim,Fn_N=f[0],Ft1_N=f[1],Ft2_N=f[2],mu1=mu[0],mu2=mu[1],
                        utilization=rho,on_geometric_boundary=boundary,before_table=i<stop))
            bilateral = np.all(np.column_stack([series[g]['normal'] for g in ('finger1','finger2')]) >= p['grasp_protocol']['criteria']['minimum_finger_normal_force_N'],axis=1)
            geom_bilateral = np.all(np.column_stack([series[g]['count'] for g in ('finger1','finger2')]) > 0, axis=1)
            indices = np.flatnonzero(~geom_bilateral)
            report['groups'][label] = dict(samples=samples,
                first_drop_1mm=event('first_relative_z_drop_at_least_1mm'),
                first_bilateral_force_loss=event('first_bilateral_finger_force_below_frozen_threshold'),
                first_bilateral_geometry_loss=None if not len(indices) else dict(row=int(indices[0]),elapsed_s=float(a['time'][indices[0]]-p['snapshot_time_s'])),
                first_table_geometry=event('first_table_geometry'), first_table_upward_force=event('first_table_upward_support'),
                before_table=diag['intervals'][0],
                before_table_finger_utilization=dict(definition='L1' if label=='pyramidal' else 'L2',
                    positive_contact_samples=len(rho_before),geometric_boundary_samples=boundary_before,
                    min=float(min(rho_before)) if rho_before else None, max=float(max(rho_before)) if rho_before else None,
                    median=float(np.median(rho_before)) if rho_before else None),
                all_cube_positive_contact_samples=positive_all,
                arm_tracking_max_abs_rad=float(np.abs(a['qpos'][:,:7]-a['physical_target'][:,:7]).max()),
                first_coupling_exceedance=event('first_coupling_exceedance'),
                first_penetration_exceedance=event('first_robot_penetration_exceedance'),
                full_control_health=json.loads((path/'result.json').read_text()),
                external_force_abs_max=diag['external_force_abs_max'])
            # Preserve aligned per-step descriptors for independent checks without another model run.
            np.savez_compressed(output/(label+'-series.npz'),time=a['time'],relative_world_z_drop_m=drop,
                cube_world_vz_m_s=a['qvel'][:,11], bilateral=bilateral, robot_penetration_m=penetration,
                coupling_residual_m=np.abs(a['qpos'][:,7]+a['qpos'][:,8]),
                finger1_Fz_N=series['finger1']['world'][:,2],finger2_Fz_N=series['finger2']['world'][:,2],
                table_Fz_N=series['table']['world'][:,2])
    pa = report['groups']['pyramidal']['samples'][0]['relative_world_z_drop_m']
    eb = report['groups']['elliptic']['samples'][0]['relative_world_z_drop_m']
    report['primary'] = dict(threshold_m=.001,strict_less_than=True,pyramidal_drop_m=pa,elliptic_drop_m=eb,
        improvement_m=pa-eb, relative_reduction_fraction=(pa-eb)/pa, prediction_holds=eb<.001)
    stops = [report['groups'][g]['before_table']['samples'] for g in ('pyramidal','elliptic')]
    common = min(stops)
    report['common_no_table'] = dict(start_row=0,stop_row_exclusive=common,samples=common,groups={})
    for label in ('pyramidal','elliptic'):
        s = np.load(output/(label+'-series.npz'))
        report['common_no_table']['groups'][label] = dict(mean_finger_Fz_N=[float(s[k][:common].mean()) for k in ('finger1_Fz_N','finger2_Fz_N')],
            relative_drop_at_end_m=float(s['relative_world_z_drop_m'][common-1]),
            end_world_vz_m_s=float(s['cube_world_vz_m_s'][common-1]), max_robot_penetration_m=float(s['robot_penetration_m'][:common].max()))
    report['limitations'] = ['One frozen state and one elliptic run; no statistical population claim.',
        'Global cone selection also changes soft-contact dynamics and all contact solves; geometry-only causality not isolated.',
        'Forces are saved post-step mj_forward instantaneous values, not integration-time impulses.',
        'Pyramidal friction metadata reconstructed from verified uniform frozen pair coefficients; no original efc_state inference.',
        'Geometric cone utilization is per contact sample, not an original stick/slip solver label.',
        'The fixed-pose hold does not command lifting and is not a grasp acceptance trial.']
    frozen.write_json(output/'comparison.json',report)
    frozen.write_json(output/'episodes.json',complete)
    print(json.dumps(dict(primary=report['primary'],events={g:{k:v for k,v in data.items() if k.startswith('first_')} for g,data in report['groups'].items()})),flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pyramidal',type=Path,required=True)
    parser.add_argument('--elliptic',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args();analyze(args.pyramidal,args.elliptic,args.output_dir)
