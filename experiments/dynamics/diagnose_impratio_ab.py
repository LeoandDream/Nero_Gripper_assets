"""Issue #12 saved-record reduction; scratch kinematics only, no dynamics.

Contact forces are immutable post-step forward samples, not integration impulses.
Each series contains snapshot row -1 and every recorded post-step row. CSV contact
coordinates describe solver samples, not persistent material-point trajectories.
"""
import argparse
from contextlib import contextmanager
import csv
import hashlib
import json
from pathlib import Path
import sys
import traceback

import mujoco
import numpy as np

from diagnose_ab import load_ab_episode, _analyze, _json, contact_series, SOURCE_NAMES

KEY_TIMES = (.068, .141, .751, 1.328, 2.164)
P1_TIME_S = 1.328
P1_THRESHOLD_M = .005761707
FORBIDDEN = ('mj_step', 'mj_step1', 'mj_step2', 'mj_forward', 'mj_forwardSkip',
             'mj_fwdPosition', 'mj_inverse', 'mj_inverseSkip', 'mj_fwdConstraint',
             'mj_fwdAcceleration', 'mj_fwdVelocity', 'mj_contactForce')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


@contextmanager
def kinematics_only():
    """Fail closed if this reducer or an imported helper attempts dynamics."""
    originals = {name: getattr(mujoco, name) for name in FORBIDDEN}
    def deny(*args, **kwargs):
        raise RuntimeError('Dynamics/force-solving call forbidden in saved-record reduction')
    try:
        for name in originals:
            setattr(mujoco, name, deny)
        yield
    finally:
        for name, function in originals.items():
            setattr(mujoco, name, function)


def force_parts(event):
    frame = np.asarray(event['contact_frame']).reshape(3, 3)
    force = np.asarray(event['force_contact_frame'][:3])
    sign = 1 if event['bodies'][1] == 'cube' else -1
    normal = sign * frame[0] * force[0]
    tangent = sign * (frame[1:].T @ force[1:])
    return normal, tangent, normal + tangent


def rho_elliptic(force, friction):
    if force[0] <= 1e-12:
        return None
    return float(np.linalg.norm(np.asarray(force[1:3]) / friction) / force[0])


def prediction(drop_m, table_through_time, valid=True):
    if not valid or table_through_time:
        return 'INVALID'
    return 'PASS' if drop_m < P1_THRESHOLD_M else 'FAIL'


def stats(values):
    values = np.asarray(values)
    if not len(values):
        return None
    return dict(min=values.min(axis=0).tolist(), mean=values.mean(axis=0).tolist(),
                max=values.max(axis=0).tolist())


def _metadata(path, rows):
    metadata = [_json(line) for line in (path / 'contact-parameters.jsonl').read_text().splitlines()]
    if len(metadata) != len(rows):
        raise ValueError('Contact parameter row count mismatch: ' + str(path))
    for i, (meta, row) in enumerate(zip(metadata, rows)):
        if meta.get('time') != row['time'] or len(meta.get('contacts', [])) != len(row['events']):
            raise ValueError(f'Contact parameter alignment mismatch: {path}, row {i}')
        for item in meta['contacts']:
            friction = np.asarray(item.get('friction5'))
            if item.get('dim') != 3 or friction.shape != (5,) or not np.isfinite(friction).all() or np.any(friction <= 0):
                raise ValueError('Unexpected saved contact dimension/friction: ' + str(path))
    return metadata


def _validate_pair(loaded):
    old, new = loaded['A'], loaded['B']
    for key in ('config', 'grasp_protocol', 'cube', 'joint_plan', 'snapshot_time_s', 'constant_command_8d'):
        if old[2][key] != new[2][key]:
            raise ValueError('Protected protocol differs: ' + key)
    if set(old[4]) != set(new[4]):
        raise ValueError('Initial-state schemas differ')
    for key in old[4]:
        if key != 'restored_integration_after_forward':
            np.testing.assert_array_equal(old[4][key], new[4][key], err_msg='Initial state differs: ' + key)
    for key in ('time', 'phase', 'requested', 'command', 'physical_target'):
        np.testing.assert_array_equal(old[0][key], new[0][key], err_msg='Saved target/time differs: ' + key)
    for label, (arrays, rows, protocol, _, _) in loaded.items():
        if len(rows) != 14100 or protocol['config']['timestep_s'] != .001:
            raise ValueError('Expected complete 14,100-row, 1 ms record: ' + label)
        if np.any(arrays['qfrc_applied']) or np.any(arrays['xfrc_applied']):
            raise ValueError('Nonzero saved external force: ' + label)


def _reduce_group(label, path, loaded, initial_events, writer, output):
    arrays, rows, protocol, observations, initial = loaded
    diagnostic = _analyze(path, loaded)
    contacts, penetration = contact_series(rows)
    metadata = _metadata(path, rows)
    with np.load(path / 'warnings.npz', allow_pickle=False) as warnings:
        np.testing.assert_array_equal(warnings['time'], arrays['time'])
        numbers = warnings['numbers']
        if numbers.shape != (len(rows), 8) or numbers.dtype.kind not in 'iu' or np.any(numbers < 0):
            raise ValueError('Malformed warning record: ' + str(path))
        if np.any(numbers):
            raise ValueError('Numerical warnings in saved record; P1 invalid: ' + str(path))
        warning_max = np.max(numbers, axis=0).tolist()
    model = mujoco.MjModel.from_xml_path(str(path / 'model.xml'))
    expected_impratio = 1 if label == 'A' else 10
    if (int(model.opt.cone) != 1 or model.opt.impratio != expected_impratio or
            model.nq != 16 or model.nv != 15 or model.npair or model.opt.enableflags):
        raise ValueError('Unexpected compiled elliptic/impratio model: ' + label)
    names = ('cube', 'gripper_base', 'gripper_link1', 'gripper_link2')
    ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in names}
    if min(ids.values()) < 0 or np.any(model.body_ipos[ids['cube']]):
        raise ValueError('Unexpected body/COM convention')
    data = mujoco.MjData(model)
    cube, gripper = ids['cube'], ids['gripper_base']
    mass = float(model.body_mass[cube]); mg = -mass * float(model.opt.gravity[2])
    if mass != .1 or not np.isclose(mg, .981, rtol=0, atol=1e-14):
        raise ValueError('Unexpected frozen mass/weight')
    initial_cube_rotation = initial['gripper_rotation_world'] @ initial['cube_rotation_in_gripper']
    limits = np.asarray(protocol['config']['arm_force_limits_Nm'] + [protocol['config']['finger_force_limit_N']] * 2)
    minimum_normal = protocol['grasp_protocol']['criteria']['minimum_finger_normal_force_N']
    table_event = diagnostic['events']['first_table_geometry']
    stop = len(rows) if table_event is None else table_event['row']
    initial_table = any('cube' in e['bodies'] and 'table' in e['geoms'] for e in initial_events)
    if initial_table:
        stop = 0
    t0 = protocol['snapshot_time_s']
    records = []; max_pose_error = 0.; max_force_error = 0.; rho_before = []; slip_before = []

    def velocity(body, point, qvel):
        jacobian = np.zeros((3, model.nv)); angular = np.zeros_like(jacobian)
        mujoco.mj_jac(model, data, jacobian, angular, np.asarray(point), body)
        return jacobian @ qvel, angular @ qvel

    def angle(rotation):
        return float(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))

    for row in range(-1, len(rows)):
        is_initial = row < 0
        qpos = initial['qpos'] if is_initial else arrays['qpos'][row]
        qvel = initial['qvel'] if is_initial else arrays['qvel'][row]
        events = initial_events if is_initial else rows[row]['events']
        elapsed = 0. if is_initial else float(arrays['time'][row] - t0)
        data.qpos[:] = qpos
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        saved_position = initial['gripper_position_world_m'] if is_initial else observations['gripper_position_world_m'][row]
        saved_rotation = initial['gripper_rotation_world'] if is_initial else observations['gripper_rotation_world'][row]
        cube_rotation = data.xmat[cube].reshape(3, 3)
        gripper_rotation = data.xmat[gripper].reshape(3, 3)
        relative_rotation = gripper_rotation.T @ cube_rotation
        saved_relative_position = initial['cube_position_in_gripper_m'] if is_initial else observations['cube_position_in_gripper_m'][row]
        saved_relative_rotation = initial['cube_rotation_in_gripper'] if is_initial else observations['cube_rotation_in_gripper'][row]
        max_pose_error = max(max_pose_error, float(np.max(np.abs(data.xpos[gripper] - saved_position))),
            float(np.max(np.abs(gripper_rotation - saved_rotation))), float(np.max(np.abs(data.xpos[cube] - qpos[9:12]))),
            float(np.max(np.abs(gripper_rotation.T @ (data.xpos[cube] - data.xpos[gripper]) - saved_relative_position))),
            float(np.max(np.abs(relative_rotation - saved_relative_rotation))))
        if max_pose_error > 1e-12:
            raise ValueError('Saved/kinematic pose mismatch: ' + label)
        cube_velocity, cube_angular = velocity(cube, data.xipos[cube], qvel)
        gripper_velocity, gripper_angular = velocity(gripper, data.xpos[gripper], qvel)
        np.testing.assert_allclose(cube_velocity, qvel[9:12], rtol=0, atol=1e-12)
        normal = np.zeros((4, 3)); tangent = np.zeros((4, 3)); fn = np.zeros(4); counts = np.zeros(4, int)
        slip_max = 0.; rhos = []; torque = np.zeros(3)
        for event_index, event in enumerate(events):
            if 'cube' not in event['bodies']:
                continue
            cube_side = event['bodies'].index('cube'); other = event['bodies'][1 - cube_side]
            group = {'gripper_link1': 'finger1', 'gripper_link2': 'finger2'}.get(other,
                'table' if event['geoms'][1 - cube_side] == 'table' else 'other')
            group_index = ('finger1', 'finger2', 'table', 'other').index(group)
            other_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, other)
            if other_id < 0:
                raise ValueError('Unknown contact body: ' + other)
            if is_initial:
                geom_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in event['geoms']]
                if min(geom_ids) < 0 or not np.array_equal(model.geom_friction[geom_ids[0]], model.geom_friction[geom_ids[1]]):
                    raise ValueError('Cannot reconstruct initial friction mixing')
                friction = np.repeat(model.geom_friction[geom_ids[0], 0], 2)
                dim = max(int(model.geom_condim[g]) for g in geom_ids)
            else:
                saved = metadata[row]['contacts'][event_index]
                friction = np.asarray(saved['friction5'][:2]); dim = saved['dim']
            if dim != 3 or not np.array_equal(friction, [1., 1.]):
                raise ValueError('Unexpected active contact dimension/friction')
            nforce, tforce, force = force_parts(event)
            normal[group_index] += nforce; tangent[group_index] += tforce
            f = event['force_contact_frame']; fn[group_index] += max(0., f[0]); counts[group_index] += 1
            frame = np.asarray(event['contact_frame']).reshape(3, 3)
            point = np.asarray(event['position_m']); lever = point - data.xipos[cube]
            cube_point_velocity = cube_velocity + np.cross(cube_angular, lever)
            other_point_velocity, _ = velocity(other_id, point, qvel)
            relative_point_velocity = cube_point_velocity - other_point_velocity
            tangential_velocity = frame[1:] @ relative_point_velocity
            slip_speed = float(np.linalg.norm(tangential_velocity))
            rho = rho_elliptic(f, friction)
            if rho is not None:
                rhos.append(rho)
            if group_index < 2:
                slip_max = max(slip_max, slip_speed)
                if 0 <= row < stop:
                    slip_before.append(slip_speed)
                    if rho is not None:
                        rho_before.append(rho)
            contact_torque = np.cross(lever, force); torque += contact_torque
            item = dict(group=label, row=row, jsonl_line=None if is_initial else row + 1, event_index=event_index,
                elapsed_s=elapsed, contact_group=group, before_first_table=is_initial or row < stop,
                dim=dim, mu1=float(friction[0]), mu2=float(friction[1]), Fn_N=f[0], Ft1_N=f[1], Ft2_N=f[2],
                Fz_normal_N=float(nforce[2]), Fz_tangent_N=float(tforce[2]), Fz_total_N=float(force[2]),
                rho_elliptic=rho, on_geometric_boundary=None if rho is None else abs(rho - 1) <= 1e-10,
                tangential_point_speed_m_s=slip_speed, distance_m=event['distance_m'],
                robot_penetration_m=max(0., -event['distance_m']) if other != 'world' else 0.,
                constraint_address=event['constraint_address'])
            vectors = [('point_world_m', point), ('point_cube_local_m', cube_rotation.T @ (point - data.xpos[cube])),
                ('point_other_local_m', data.xmat[other_id].reshape(3, 3).T @ (point - data.xpos[other_id])),
                ('relative_point_velocity_m_s', relative_point_velocity), ('torque_world_Nm', contact_torque)]
            for prefix, values in vectors:
                for axis, value in zip('xyz', values):
                    item[prefix + '_' + axis] = float(value)
            for j, value in enumerate(frame.flat):
                item['contact_frame_' + str(j)] = float(value)
            writer.writerow(item)
        total_force = normal + tangent
        if not is_initial:
            max_force_error = max(max_force_error, abs(float(total_force[:, 2].sum() - arrays['qfrc_constraint'][row, 11])))
            if max_force_error > 1e-11:
                raise ValueError('Saved force projection/generalized-force mismatch: ' + label)
            np.testing.assert_array_equal(counts, [contacts[g]['count'][row] for g in ('finger1', 'finger2', 'table', 'other')])
        target = np.r_[initial['controller_command'][:7], initial['controller_command'][7] / 2, -initial['controller_command'][7] / 2] if is_initial else arrays['physical_target'][row]
        actuator_force = np.zeros(9) if is_initial else arrays['actuator_force'][row]
        record = dict(row=row, elapsed_s=elapsed, time_s=t0 + elapsed,
            cube_world_z_m=float(qpos[11]), cube_world_delta_z_m=float(qpos[11] - initial['qpos'][11]),
            gripper_world_delta_z_m=float(data.xpos[gripper, 2] - initial['gripper_position_world_m'][2]),
            relative_world_z_drop_m=float(initial['qpos'][11] - initial['gripper_position_world_m'][2] - (qpos[11] - data.xpos[gripper, 2])),
            cube_world_vz_m_s=float(cube_velocity[2]), gripper_world_vz_m_s=float(gripper_velocity[2]),
            relative_world_vz_m_s=float(cube_velocity[2] - gripper_velocity[2]),
            cube_rotation_world=cube_rotation.copy(), cube_rotation_in_gripper=relative_rotation.copy(),
            relative_rotation_rad=angle(relative_rotation @ initial['cube_rotation_in_gripper'].T),
            world_rotation_rad=angle(cube_rotation @ initial_cube_rotation.T),
            cube_angular_velocity_world_rad_s=cube_angular.copy(), relative_angular_speed_rad_s=float(np.linalg.norm(cube_angular - gripper_angular)),
            finger_tangential_point_speed_max_m_s=slip_max, cube_contact_rho_max=np.nan if not rhos else max(rhos),
            cube_contact_torque_world_Nm=torque, robot_penetration_m=float(max([0.] + [-e['distance_m'] for e in events if 'cube' in e['bodies'] and 'world' not in e['bodies']])),
            coupling_residual_m=float(abs(qpos[7] + qpos[8])), arm_tracking_error_rad=qpos[:7] - target[:7],
            physical_target_9d=target.copy(), actual_slide_width_m=float(qpos[7] - qpos[8]),
            actuator_force_N_or_Nm=actuator_force.copy(), actuator_force_available=not is_initial,
            actuator_saturated_9d=(np.abs(actuator_force) >= limits - 1e-8),
            actuator_limit_excess_9d=np.maximum(np.abs(actuator_force) - limits, 0),
            numerical_warning_numbers=np.zeros(8, dtype=int) if is_initial else numbers[row].copy(),
            bilateral_geometry=bool(np.all(counts[:2] > 0)), bilateral_force=bool(np.all(fn[:2] >= minimum_normal)),
            total_cube_contact_Fz_N=float(total_force[:, 2].sum()), cube_contact_Fz_minus_mg_N=float(total_force[:, 2].sum() - mg))
        for j, group in enumerate(('finger1', 'finger2', 'table', 'other')):
            record.update({group + '_count': int(counts[j]), group + '_Fn_N': float(fn[j]),
                group + '_Fz_normal_N': float(normal[j, 2]), group + '_Fz_tangent_N': float(tangent[j, 2]),
                group + '_Fz_N': float(total_force[j, 2])})
        records.append(record)
    series = {key: np.asarray([r[key] for r in records]) for key in records[0]}
    np.savez_compressed(output / (label + '-series.npz'), **series)

    def sample(index):
        return {key: np.asarray(values[index]).tolist() for key, values in series.items()
                if key != 'cube_contact_rho_max' or np.isfinite(values[index])}

    def first(mask):
        found = np.flatnonzero(mask)
        return None if not len(found) else sample(int(found[0]))

    samples = {}
    for elapsed in KEY_TIMES:
        index = round(elapsed / protocol['config']['timestep_s'])
        if abs(series['elapsed_s'][index] - elapsed) > 1e-9:
            raise ValueError('Key sample is not aligned')
        samples[str(elapsed)] = sample(index)
        samples[str(elapsed)]['no_table_geometry_through_sample'] = not bool(np.any(series['table_count'][:index + 1]))
    samples['final'] = sample(len(records) - 1)
    summary = dict(source_dir=str(path.resolve()), samples=len(rows), impratio=expected_impratio,
        key_samples=samples, snapshot=sample(0), first_drop_at_least_1mm=first(series['relative_world_z_drop_m'] >= .001),
        first_table_geometry=first(series['table_count'] > 0), first_table_upward_support=first(series['table_Fz_N'] > 1e-12),
        first_bilateral_geometry_loss=first(~series['bilateral_geometry']), first_bilateral_force_loss=first(~series['bilateral_force']),
        first_robot_penetration_exceedance=first(series['robot_penetration_m'] > protocol['grasp_protocol']['criteria']['maximum_robot_object_penetration_m']),
        first_coupling_exceedance=first(series['coupling_residual_m'] > protocol['config']['acceptance']['maximum_coupling_error_m']),
        original_control_health=_json((path / 'result.json').read_text()), original_grasp_criteria=protocol['grasp_protocol']['criteria'],
        formal_grasp_acceptance='UNKNOWN_NOT_EVALUATED_DIAGNOSTIC_FIXED_POSE',
        maximum_robot_penetration_m=float(penetration.max()), maximum_coupling_residual_m=float(series['coupling_residual_m'].max()),
        arm_tracking_max_abs_rad=float(np.abs(series['arm_tracking_error_rad'][1:]).max()),
        actuator_saturation_fraction_9d=series['actuator_saturated_9d'][1:].mean(axis=0).tolist(),
        actuator_limit_excess_max_9d=series['actuator_limit_excess_9d'][1:].max(axis=0).tolist(),
        warning_numbers_max=warning_max, before_first_table=diagnostic['intervals'][0],
        before_first_table_finger_rho=stats(rho_before), before_first_table_finger_tangential_point_speed_m_s=stats(slip_before),
        kinematics_vs_saved_pose_max_error=max_pose_error, saved_contact_force_projection_max_error_N=max_force_error,
        external_force_abs_max=diagnostic['external_force_abs_max'],
        observational_sustained_holding=dict(status='FAIL' if np.any(series['table_count']) or not np.all(series['bilateral_force']) else 'OBSERVED_NO_TABLE_BILATERAL_SUPPORT',
            table_geometry_observed=bool(np.any(series['table_count'])), all_samples_bilateral_force=bool(np.all(series['bilateral_force'])),
            final_table_geometry=bool(series['table_count'][-1]), final_table_upward_force_N=float(series['table_Fz_N'][-1]),
            note='Observation only; fixed-pose diagnostic does not evaluate original scripted lift/hold acceptance.'))
    return summary, series, stop


def analyze(a_dir, b_dir, output):
    paths = {'A': a_dir.resolve(), 'B': b_dir.resolve()}; output = output.resolve()
    if paths['A'] == paths['B'] or any(output == path or path in output.parents for path in paths.values()):
        raise ValueError('Output must be separate from both immutable source Runs')
    output.mkdir(parents=True, exist_ok=False)
    inputs = {str(p): digest(p) for path in paths.values() for p in path.iterdir() if p.is_file()}
    inputs.update({str(Path(__file__).resolve()): digest(__file__),
                   str(Path(__file__).with_name('diagnose_ab.py').resolve()): digest(Path(__file__).with_name('diagnose_ab.py')),
                   str(Path(__file__).with_name('diagnose.py').resolve()): digest(Path(__file__).with_name('diagnose.py'))})
    loaded = {label: load_ab_episode(path) for label, path in paths.items()}
    _validate_pair(loaded)
    execution = _json((paths['B'] / 'execution.json').read_text())
    if execution['error'] or execution['actual_mj_steps'] != 14100:
        raise ValueError('Incomplete B; P1 invalid, preserve partial raw records without rerunning')
    initial_events = {'A': _json((paths['A'] / 'elliptic-initial-contacts.json').read_text()),
                      'B': _json((paths['B'] / 'impratio10-initial-contacts.json').read_text())}
    fields = ['group', 'row', 'jsonl_line', 'event_index', 'elapsed_s', 'contact_group', 'before_first_table',
        'dim', 'mu1', 'mu2', 'Fn_N', 'Ft1_N', 'Ft2_N', 'Fz_normal_N', 'Fz_tangent_N', 'Fz_total_N',
        'rho_elliptic', 'on_geometric_boundary', 'tangential_point_speed_m_s', 'distance_m', 'robot_penetration_m', 'constraint_address']
    for prefix in ('point_world_m', 'point_cube_local_m', 'point_other_local_m', 'relative_point_velocity_m_s', 'torque_world_Nm'):
        fields.extend(prefix + '_' + axis for axis in 'xyz')
    fields.extend('contact_frame_' + str(j) for j in range(9))
    groups = {}; series = {}; stops = {}
    with kinematics_only(), (output / 'contacts.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for label, path in paths.items():
            groups[label], series[label], stops[label] = _reduce_group(label, path, loaded[label], initial_events[label], writer, output)
    common = min(stops.values())
    common_summary = dict(start_row=0, stop_row_exclusive=common, post_step_samples=common,
        snapshot_included_in_statistics=False, groups={})
    for label, values in series.items():
        interval = slice(1, common + 1)
        common_summary['groups'][label] = dict(last_elapsed_s=None if not common else float(values['elapsed_s'][common]),
            last_relative_drop_m=None if not common else float(values['relative_world_z_drop_m'][common]),
            relative_world_vz_m_s=stats(values['relative_world_vz_m_s'][interval]),
            finger_Fz_N=stats(np.column_stack([values['finger1_Fz_N'][interval], values['finger2_Fz_N'][interval]])),
            maximum_robot_penetration_m=None if not common else float(values['robot_penetration_m'][interval].max()))
    old = groups['A']['key_samples'][str(P1_TIME_S)]; new = groups['B']['key_samples'][str(P1_TIME_S)]
    p1 = dict(status=prediction(new['relative_world_z_drop_m'], not new['no_table_geometry_through_sample']),
        elapsed_s=P1_TIME_S, strict_drop_threshold_m=P1_THRESHOLD_M, A_drop_m=old['relative_world_z_drop_m'],
        B_drop_m=new['relative_world_z_drop_m'], improvement_m=old['relative_world_z_drop_m'] - new['relative_world_z_drop_m'],
        reduction_fraction=1 - new['relative_world_z_drop_m'] / old['relative_world_z_drop_m'],
        no_B_table_geometry_through_prediction_time=new['no_table_geometry_through_sample'],
        requirement='Strictly below threshold AND no cube/table geometry in saved snapshot or samples through 1.328 s')
    report = dict(diagnostic_only=True, status='READONLY_REDUCTION_COMPLETE', P1=p1, groups=groups,
        common_no_table=common_summary, actual_new_mj_steps_in_analysis=0, force_solves_in_analysis=0,
        kinematic_evaluations=sum(len(values['row']) for values in series.values()),
        protected_protocol_initial_state_times_and_targets_exactly_equal=True,
        derived_after_forward_state_max_difference=float(np.abs(loaded['A'][4]['restored_integration_after_forward'] - loaded['B'][4]['restored_integration_after_forward']).max()),
        actuator_ctrl_max_difference=float(np.abs(loaded['A'][0]['ctrl'] - loaded['B'][0]['ctrl']).max()),
        source_hashes=inputs, python=sys.version, mujoco=mujoco.__version__, numpy=np.__version__, argv=sys.argv,
        limitations=['One frozen state, one new impratio=10 run; no statistical or hardware-fidelity claim.',
            'Post-step forward force samples are instantaneous, not integration-time impulses or step-average forces.',
            'Local contact coordinates describe solver samples; no persistent material-point identity or solver stick/slip labels.',
            'Common no-table statistics exclude the first table contact in either run. Per-run intervals use their own boundary.',
            'P1 and observational holding do not replace original control-health or scripted grasp criteria.'])
    for name, expected in inputs.items():
        if digest(name) != expected:
            raise ValueError('Saved input changed during read-only reduction: ' + name)
    write_json(output / 'comparison.json', report)
    outputs = {p.name: digest(p) for p in output.iterdir() if p.is_file()}
    write_json(output / 'analysis-manifest.json', dict(inputs=inputs, outputs=outputs, new_mj_steps=0, force_solves=0))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a-dir', type=Path, required=True)
    parser.add_argument('--b-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error('Refuse existing output directory, including any original Run')
    try:
        report = analyze(args.a_dir, args.b_dir, args.output_dir)
    except Exception:
        if args.output_dir.is_dir() and not (args.output_dir / 'BLOCKED.json').exists():
            write_json(args.output_dir / 'BLOCKED.json', dict(status='BLOCKED_UNKNOWN', P1='INVALID',
                error=traceback.format_exc(), new_mj_steps=0, force_solves=0))
        raise
    print(json.dumps(dict(status=report['status'], P1=report['P1'], output_dir=str(args.output_dir))))


if __name__ == '__main__':
    main()
