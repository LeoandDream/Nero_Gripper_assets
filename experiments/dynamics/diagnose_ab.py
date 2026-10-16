"""Read-only frozen-snapshot A/B evidence; NumPy only, never loads a model.

Table intervals split at first recorded geometric contact, even at zero force.
The two fall descriptors are fixed diagnostic observations, not acceptance rules.
"""
import argparse
import copy
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

_spec = importlib.util.spec_from_file_location('_grasp_saved_record', Path(__file__).with_name('diagnose.py'))
_saved = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_saved)
GROUPS, WIDTHS = _saved.GROUPS, _saved.WIDTHS
_numeric, _json, contact_series = _saved._numeric, _saved._json, _saved.contact_series
SOURCE_NAMES = ('trajectory.npz', 'contacts.jsonl', 'protocol.json', 'initial-state.npz', 'observations.npz')
OBS_SHAPES = dict(gripper_position_world_m=(3,), gripper_rotation_world=(3, 3),
                  cube_position_in_gripper_m=(3,), cube_rotation_in_gripper=(3, 3))
INITIAL_SHAPES = dict(qpos=(16,), qvel=(15,), time=(1,), controller_requested=(8,),
                      controller_command=(8,), gripper_position_world_m=(3,), gripper_rotation_world=(3, 3),
                      cube_position_in_gripper_m=(3,), cube_rotation_in_gripper=(3, 3))
TABLE_SUPPORT_EPS_N = 1e-12
FALL_VZ_M_S = -1e-3
RELATIVE_DROP_M = 1e-3


def _npz(path):
    with np.load(path, allow_pickle=False) as saved:
        return {key: saved[key] for key in saved.files}


def _rotation(value, shape, name):
    a = _numeric(value, shape, name)
    if not np.allclose(a @ np.swapaxes(a, -1, -2), np.eye(3), rtol=0, atol=1e-6) or not np.allclose(np.linalg.det(a), 1., rtol=0, atol=1e-6):
        raise ValueError(name + ': expected proper orthonormal rotation')
    return a


def load_ab_episode(directory):
    """Validate snapshot-relative samples without the old t=dt assumption."""
    directory = Path(directory)
    a = _npz(directory / 'trajectory.npz'); obs = _npz(directory / 'observations.npz')
    initial = _npz(directory / 'initial-state.npz'); protocol = _json((directory / 'protocol.json').read_text())
    try:
        t = a['time']; n = len(t)
        if n == 0: raise ValueError('Empty trajectory')
        _numeric(t, (n,), 'time')
        for key, width in WIDTHS.items(): _numeric(a[key], (n, width), key)
        if a['xfrc_applied'].ndim != 3 or a['xfrc_applied'].shape[2] != 6:
            raise ValueError('xfrc_applied must have shape (N, bodies, 6)')
        for key, value in a.items():
            if key == 'phase': continue
            if value.ndim < 1 or value.shape[0] != n: raise ValueError('Trajectory row count: ' + key)
            _numeric(value, value.shape, key)
        if a['phase'].shape != (n,) or a['phase'].dtype.kind != 'U' or any(not str(p) for p in a['phase']):
            raise ValueError('Expected N nonempty phase strings')
        for key, shape in OBS_SHAPES.items():
            _numeric(obs[key], (n,) + shape, key)
            if 'rotation' in key: _rotation(obs[key], (n,) + shape, key)
        _numeric(obs['time'], (n,), 'observations time')
        if not np.allclose(obs['time'], t, rtol=0, atol=1e-10): raise ValueError('Observation/trajectory time mismatch')
        for key, shape in INITIAL_SHAPES.items(): _numeric(initial[key], shape, 'initial ' + key)
        for key, value in initial.items(): _numeric(value, value.shape, 'initial ' + key)
        if initial['integration_state'].ndim != 1 or not initial['integration_state'].size:
            raise ValueError('Expected nonempty integration_state vector')
        _numeric(initial['integration_state'], initial['integration_state'].shape, 'integration_state')
        for key in ('restored_integration_before_forward', 'restored_integration_after_forward'):
            if key in initial: _numeric(initial[key], initial['integration_state'].shape, key)
        if 'restored_integration_before_forward' in initial and not np.array_equal(initial['restored_integration_before_forward'], initial['integration_state']):
            raise ValueError('Restored integration state differs before forward')
        _rotation(initial['cube_rotation_in_gripper'], (3, 3), 'initial cube rotation')
        _rotation(initial['gripper_rotation_world'], (3, 3), 'initial gripper rotation')
        cfg = protocol['config']; criteria = protocol['grasp_protocol']['criteria']
        dt = float(_numeric(cfg['timestep_s'], (), 'timestep_s'))
        snapshot = float(_numeric(protocol['snapshot_time_s'], (), 'snapshot_time_s'))
        for key, value in [('timestep_s', dt), ('minimum finger force', criteria['minimum_finger_normal_force_N']),
                           ('maximum penetration', criteria['maximum_robot_object_penetration_m']),
                           ('maximum coupling', cfg['acceptance']['maximum_coupling_error_m'])]:
            if float(_numeric(value, (), key)) <= 0: raise ValueError('Expected positive ' + key)
        _numeric(cfg['arm_force_limits_Nm'], (7,), 'arm force limits')
        _numeric(cfg['finger_force_limit_N'], (), 'finger force limit')
        limits = np.asarray(cfg['arm_force_limits_Nm'] + [cfg['finger_force_limit_N']] * 2)
        if (limits <= 0).any(): raise ValueError('Expected positive force limits')
        _numeric(protocol['cube']['center'], (3,), 'cube center')
        if not isinstance(protocol['joint_plan'], dict) or protocol['diagnostic_only'] is not True:
            raise ValueError('Expected joint plan and diagnostic_only=true')
        if not np.isclose(initial['time'][0], snapshot, rtol=0, atol=1e-10):
            raise ValueError('Initial state/snapshot time mismatch')
        if not np.isclose(t[0], snapshot + dt, rtol=0, atol=1e-9) or not np.allclose(np.diff(t), dt, rtol=0, atol=1e-9):
            raise ValueError('Trajectory must start snapshot+dt with no gaps/resets')
    except (KeyError, TypeError, IndexError) as exc:
        raise ValueError('Incomplete or malformed A/B record: ' + str(exc)) from exc
    rows = []
    for i, line in enumerate((directory / 'contacts.jsonl').read_text().splitlines()):
        try:
            row = _json(line)
            if i >= n or not isinstance(row, dict) or not isinstance(row.get('events'), list):
                raise ValueError('Expected aligned events list')
            if row.get('phase') != str(a['phase'][i]) or not np.isclose(_numeric(row.get('time'), (), 'JSONL time'), t[i], rtol=0, atol=1e-10):
                raise ValueError('JSONL time/phase mismatch')
            for event in row['events']:
                for key in ('bodies', 'geoms'):
                    value = event.get(key)
                    if not isinstance(value, list) or len(value) != 2 or any(not isinstance(v, str) or not v for v in value):
                        raise ValueError('Expected two named ' + key)
                if event['bodies'].count('cube') > 1: raise ValueError('Ambiguous cube contact')
                for key, shape in [('contact_frame', (9,)), ('force_contact_frame', (6,)), ('normal', (3,)),
                                   ('position_m', (3,)), ('distance_m', ())]: _numeric(event.get(key), shape, key)
                frame = np.asarray(event['contact_frame']).reshape(3, 3)
                _rotation(frame, (3, 3), 'contact frame')
                if not np.allclose(event['normal'], frame[0], rtol=0, atol=1e-9): raise ValueError('Contact normal/frame mismatch')
                if type(event.get('constraint_address')) is not int: raise ValueError('Expected integer constraint address')
            rows.append(row)
        except (ValueError, TypeError, AttributeError, KeyError) as exc:
            raise ValueError(f'contacts.jsonl row {i}: {exc}') from exc
    if len(rows) != n: raise ValueError('JSONL/trajectory row count mismatch')
    return a, rows, protocol, obs, initial


def _stats(a):
    return {key: np.asarray(value).tolist() for key, value in [('min', a.min(axis=0)), ('mean', a.mean(axis=0)), ('max', a.max(axis=0))]}


def _first(mask):
    found = np.flatnonzero(mask)
    return int(found[0]) if len(found) else None


def _analyze(directory, loaded):
    a, rows, protocol, obs, initial = loaded; n = len(rows)
    cfg = protocol['config']; criteria = protocol['grasp_protocol']['criteria']; dt = cfg['timestep_s']
    snapshot = protocol['snapshot_time_s']; series, penetration = contact_series(rows)
    coupling = np.abs(a['qpos'][:, 7] + a['qpos'][:, 8])
    limits = np.asarray(cfg['arm_force_limits_Nm'] + [cfg['finger_force_limit_N']] * 2)
    saturation = np.abs(a['actuator_force']) >= limits - 1e-8
    finger_normals = np.column_stack([series[g]['normal'] for g in GROUPS[:2]])
    bilateral = np.all(finger_normals >= criteria['minimum_finger_normal_force_N'], axis=1)
    relative_delta = obs['cube_position_in_gripper_m'] - initial['cube_position_in_gripper_m']
    # A body frame may point local +Z downward. Use the relative world-Z gap
    # for the fall descriptor while preserving the complete body-frame slip.
    initial_world_z_gap = initial['qpos'][11] - initial['gripper_position_world_m'][2]
    world_z_gap = a['qpos'][:, 11] - obs['gripper_position_world_m'][:, 2]
    relative_drop = initial_world_z_gap - world_z_gap
    rotation_delta = obs['cube_rotation_in_gripper'] @ initial['cube_rotation_in_gripper'].T
    rotation_angle = np.arccos(np.clip((np.trace(rotation_delta, axis1=1, axis2=2) - 1.) / 2., -1., 1.))
    contact_index = _first(series['table']['count'] > 0)
    boundary = 'not_observed' if contact_index is None else ('present_at_first_record' if contact_index == 0 else 'first_recorded_contact_after_absence')
    events = {}

    def event(name, index):
        if index is None: events[name] = None; return
        i = int(index)
        events[name] = dict(row=i, time_s=float(a['time'][i]), relative_time_s=float(a['time'][i] - snapshot),
            phase=str(a['phase'][i]), source=dict(npz_row=i, observations_npz_row=i, jsonl_line=i + 1),
            state={k: a[k][i].tolist() for k in WIDTHS}, observations={k: obs[k][i].tolist() for k in OBS_SHAPES},
            contacts=rows[i]['events'], finger_normal_force_N=finger_normals[i].tolist(),
            contacts_by_group={g: dict(normal_force_N=float(v['normal'][i]), tangent_force_N=float(v['tangent'][i]),
                                      world_force_N=v['world'][i].tolist(), geometric_contacts=int(v['count'][i])) for g, v in series.items()},
            cube_relative_delta_m=relative_delta[i].tolist(), cube_relative_z_drop_m=float(relative_drop[i]),
            coupling_residual_abs_m=float(coupling[i]), robot_penetration_m=float(penetration[i]),
            actuator_saturated_9d=saturation[i].tolist())

    event('first_record', 0); event('last_record', n - 1)
    event('first_table_geometry', contact_index)
    event('first_table_upward_support', _first(series['table']['world'][:, 2] > TABLE_SUPPORT_EPS_N))
    falling = a['qvel'][:, 11] < FALL_VZ_M_S; dropped = relative_drop >= RELATIVE_DROP_M
    event('first_world_vz_below_minus_1mm_s', _first(falling))
    event('first_relative_z_drop_at_least_1mm', _first(dropped))
    event('first_both_fall_descriptors', _first(falling & dropped))
    event('first_bilateral_finger_force_below_frozen_threshold', _first(~bilateral))
    event('first_robot_penetration_exceedance', _first(penetration > criteria['maximum_robot_object_penetration_m']))
    event('first_coupling_exceedance', _first(coupling > cfg['acceptance']['maximum_coupling_error_m']))
    event('maximum_robot_penetration', int(np.argmax(penetration)))
    event('maximum_relative_z_drop', int(np.argmax(relative_drop)))
    event('first_any_actuator_saturation', _first(np.any(saturation, axis=1)))

    def interval(label, start, stop):
        result = dict(interval=label, start_row=start if start < stop else None, end_row=stop - 1 if start < stop else None,
                      samples=stop - start, duration_s=float((stop - start) * dt))
        if start == stop:
            result.update(start_time_s=None, end_time_s=None, contacts=None, cube=None,
                          actuator_saturation_fraction_9d=None, maximum_robot_penetration_m=None,
                          coupling_residual_abs_max_m=None, bilateral_force_fraction=None)
            return result
        s = slice(start, stop)
        result.update(start_time_s=float(a['time'][start]), end_time_s=float(a['time'][stop - 1]),
            start_relative_time_s=float(a['time'][start] - snapshot), end_relative_time_s=float(a['time'][stop - 1] - snapshot),
            contacts={g: dict(normal_force_N=_stats(v['normal'][s]), tangent_force_N=_stats(v['tangent'][s]),
                             world_force_N=_stats(v['world'][s]), geometric_rows=int(np.count_nonzero(v['count'][s])),
                             upward_support_rows=int(np.count_nonzero(v['world'][s, 2] > TABLE_SUPPORT_EPS_N))) for g, v in series.items()},
            cube=dict(world_z_m=_stats(a['qpos'][s, 11]), world_vz_m_s=_stats(a['qvel'][s, 11]),
                world_position_m=_stats(a['qpos'][s, 9:12]), position_in_gripper_m=_stats(obs['cube_position_in_gripper_m'][s]),
                relative_delta_m=_stats(relative_delta[s]), relative_z_drop_m=_stats(relative_drop[s]),
                relative_slip_norm_m=_stats(np.linalg.norm(relative_delta[s], axis=1)), relative_rotation_angle_rad=_stats(rotation_angle[s])),
            bilateral_force_fraction=float(bilateral[s].mean()), maximum_robot_penetration_m=float(penetration[s].max()),
            coupling_residual_abs_max_m=float(coupling[s].max()), actuator_saturation_fraction_9d=saturation[s].mean(axis=0).tolist(),
            actuator_limit_excess_max_9d=np.maximum(np.abs(a['actuator_force'][s]) - limits, 0).max(axis=0).tolist(),
            actuator_force_N_or_Nm=_stats(a['actuator_force'][s]))
        return result

    stop = n if contact_index is None else contact_index
    return dict(episode_dir=str(Path(directory).resolve()), samples=n, snapshot_time_s=float(snapshot),
        duration_s=float(n * dt), official_grasp_success=None, table_boundary=boundary,
        sources={name: dict(path=str((Path(directory) / name).resolve()), sha256=hashlib.sha256((Path(directory) / name).read_bytes()).hexdigest()) for name in SOURCE_NAMES},
        initial_state={key: value.tolist() for key, value in initial.items()},
        thresholds=dict(bilateral_normal_force_N_each=criteria['minimum_finger_normal_force_N'],
                        robot_penetration_m=criteria['maximum_robot_object_penetration_m'],
                        coupling_m=cfg['acceptance']['maximum_coupling_error_m']),
        external_force_abs_max={k: float(np.abs(a[k]).max()) for k in ('qfrc_applied', 'xfrc_applied')},
        events=events, intervals=[interval('before_first_table_geometry', 0, stop), interval('from_first_table_geometry', stop, n)])


def analyze_pair(a_dir, b_dir):
    """Compare unchanged snapshot/protocol and report each group's raw events."""
    a = load_ab_episode(a_dir); b = load_ab_episode(b_dir)
    for key in ('config', 'grasp_protocol', 'joint_plan', 'snapshot_time_s'):
        if a[2][key] != b[2][key]: raise ValueError('A/B protected protocol mismatch: ' + key)
    cube_a, cube_b = copy.deepcopy(a[2]['cube']), copy.deepcopy(b[2]['cube'])
    translation = None
    if 'table' in cube_a and 'table' in cube_b:
        ca = _numeric(cube_a['table'].get('center'), (3,), 'A table center')
        cb = _numeric(cube_b['table'].get('center'), (3,), 'B table center')
        if ca[2] != .065 or cb[2] != .015: raise ValueError('Unexpected A/B table z; expected .065 and .015')
        translation = dict(A=float(ca[2]), B=float(cb[2]))
        cube_b['table']['center'][2] = cube_a['table']['center'][2]
    if cube_a != cube_b: raise ValueError('A/B protected cube/table protocol mismatch beyond approved z')
    if set(a[4]) != set(b[4]): raise ValueError('A/B initial state fields mismatch')
    for key in a[4]:
        if key == 'restored_integration_after_forward': continue
        if not np.array_equal(a[4][key], b[4][key]): raise ValueError('A/B initial state mismatch: ' + key)
    after_delta = None
    if 'restored_integration_after_forward' in a[4]:
        after_delta = float(np.abs(a[4]['restored_integration_after_forward'] - b[4]['restored_integration_after_forward']).max())
    if not np.array_equal(a[0]['time'], b[0]['time']): raise ValueError('A/B time arrays mismatch')
    return dict(schema_version=1, status='UNKNOWN', official_grasp_success=None,
        matched_initial_state_and_protected_protocol='PASS',
        approved_table_translation_m=translation, derived_after_forward_state_max_delta=after_delta,
        diagnostic_descriptors=dict(world_vz_strictly_below_m_s=FALL_VZ_M_S,
                                    relative_z_drop_at_least_m=RELATIVE_DROP_M,
                                    table_upward_force_epsilon_N=TABLE_SUPPORT_EPS_N),
        interpretation=dict(success='No grasp acceptance evaluated; late table-supported steady state does not establish grasp success',
            fall='Fixed descriptive thresholds only; not an official success/failure criterion',
            table='Split at first recorded geometric table contact, including zero solver force; positive upward force is separate',
            initial_table='Snapshot contact is not recorded here; row-zero contact means already present at first observation, not observed return',
            initial_state='Exact equality before mj_forward; only restored_integration_after_forward excluded because contacts update derived warmstart cache',
            force_sampling='Instantaneous post-mj_step mj_forward, not time-integrated or step-average',
            world_force='Force on cube = (+ if cube is bodies[1] else -) * frame.T @ force_contact_frame[:3], including tangents',
            relative_pose='Cube position and orientation in physical gripper_base body frame, referenced to frozen snapshot',
            relative_z_drop='Decrease from snapshot in world-Z gap cube_COM.z - gripper_base_origin.z; independent of body-frame Z orientation',
            equality='Protected qpos[7]+qpos[8] coupling, not solver efc residual',
            expected_measurement_duration_s=14.1),
        groups={'A': _analyze(a_dir, a), 'B': _analyze(b_dir, b)})


def write_report(report, output_dir):
    output = Path(output_dir); output.mkdir(parents=True, exist_ok=False)
    (output / 'diagnosis-ab.json').write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')
    with (output / 'intervals.csv').open('x', newline='') as stream:
        fields = ['group', 'interval', 'start_row', 'end_row', 'start_time_s', 'end_time_s', 'samples', 'duration_s',
                  'finger1_Fn_mean_N', 'finger2_Fn_mean_N', 'finger1_Ft_mean_N', 'finger2_Ft_mean_N',
                  'finger1_worldZ_mean_N', 'finger2_worldZ_mean_N', 'table_worldZ_mean_N', 'cube_world_z_min_m',
                  'cube_world_vz_min_m_s', 'relative_z_drop_max_m', 'robot_penetration_max_m', 'coupling_max_m',
                  'actuator_saturation_fraction_9d']
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for group, data in report['groups'].items():
            for interval in data['intervals']:
                row = {key: interval.get(key) for key in fields[1:8]}; row['group'] = group
                if interval['samples']:
                    contacts = interval['contacts']
                    for finger in ('finger1', 'finger2'):
                        row[finger + '_Fn_mean_N'] = contacts[finger]['normal_force_N']['mean']
                        row[finger + '_Ft_mean_N'] = contacts[finger]['tangent_force_N']['mean']
                        row[finger + '_worldZ_mean_N'] = contacts[finger]['world_force_N']['mean'][2]
                    row.update(table_worldZ_mean_N=contacts['table']['world_force_N']['mean'][2],
                        cube_world_z_min_m=interval['cube']['world_z_m']['min'], cube_world_vz_min_m_s=interval['cube']['world_vz_m_s']['min'],
                        relative_z_drop_max_m=interval['cube']['relative_z_drop_m']['max'], robot_penetration_max_m=interval['maximum_robot_penetration_m'],
                        coupling_max_m=interval['coupling_residual_abs_max_m'],
                        actuator_saturation_fraction_9d=json.dumps(interval['actuator_saturation_fraction_9d']))
                writer.writerow(row)
    with (output / 'events.csv').open('x', newline='') as stream:
        fields = ['group', 'event', 'row', 'jsonl_line', 'time_s', 'relative_time_s']
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for group, data in report['groups'].items():
            for name, event in data['events'].items():
                row = dict(group=group, event=name)
                if event: row.update({key: event[key] for key in ('row', 'time_s', 'relative_time_s')}); row['jsonl_line'] = event['source']['jsonl_line']
                writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a-dir', required=True, type=Path); parser.add_argument('--b-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path); args = parser.parse_args()
    try: report = analyze_pair(args.a_dir, args.b_dir); write_report(report, args.output_dir)
    except (ValueError, OSError) as exc: parser.exit(2, f'diagnose_ab: {exc}\n')
    print(json.dumps(dict(status=report['status'], samples={key: value['samples'] for key, value in report['groups'].items()}, output=str(args.output_dir))))


if __name__ == '__main__': main()
