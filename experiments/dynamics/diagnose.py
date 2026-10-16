"""Read-only diagnosis of frozen grasp records; numpy only, never loads a model.

Rows are zero based. Forces are the recorder's post-mj_step mj_forward samples,
not time-integrated/step-average forces. No success criterion is recomputed.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

GROUPS = ('finger1', 'finger2', 'table', 'other')
WIDTHS = dict(qpos=16, qvel=15, requested=8, command=8, physical_target=9,
              ctrl=9, actuator_force=9, qfrc_actuator=15, qfrc_constraint=15, qfrc_applied=15)


def _reject_constant(value):
    raise ValueError('Non-finite JSON number: ' + value)


def _json(text):
    return json.loads(text, parse_constant=_reject_constant)


def _numeric(value, shape, label):
    a = np.asarray(value)
    if a.shape != shape or a.dtype.kind not in 'fiu' or not np.isfinite(a).all():
        raise ValueError(f'{label}: expected finite numeric shape {shape}, got {a.shape}')
    return a.astype(float, copy=False)


def load_episode(directory):
    """Return validated (NPZ arrays, JSONL rows, protocol); malformed input fails."""
    directory = Path(directory)
    with np.load(directory / 'trajectory.npz', allow_pickle=False) as saved:
        arrays = {key: saved[key] for key in saved.files}
    if 'time' not in arrays or arrays['time'].ndim != 1 or not len(arrays['time']):
        raise ValueError('Missing or empty one-dimensional time')
    n = len(arrays['time'])
    for key, width in WIDTHS.items():
        if key not in arrays:
            raise ValueError('Missing NPZ field: ' + key)
        _numeric(arrays[key], (n, width), key)
    if 'xfrc_applied' not in arrays or arrays['xfrc_applied'].ndim != 3 or arrays['xfrc_applied'].shape[2] != 6:
        raise ValueError('xfrc_applied must have shape (N, bodies, 6)')
    for key, a in arrays.items():
        if key != 'phase':
            if a.ndim < 1 or a.shape[0] != n:
                raise ValueError('NPZ row count mismatch: ' + key)
            _numeric(a, a.shape, key)
    phase = arrays.get('phase')
    if phase is None or phase.shape != (n,) or phase.dtype.kind != 'U' or any(not str(p) for p in phase):
        raise ValueError('phase must contain N nonempty strings')
    protocol = _json((directory / 'protocol.json').read_text())
    try:
        cfg = protocol['config']; gp = protocol['grasp_protocol']; criteria = gp['criteria']
        dt = float(_numeric(cfg['timestep_s'], (), 'timestep_s'))
        values = [dt, gp['contact_trigger_force_N_each'], criteria['minimum_finger_normal_force_N'],
                  criteria['maximum_robot_object_penetration_m'], cfg['acceptance']['maximum_coupling_error_m']]
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError('Protocol thresholds must be finite and positive')
        limits = cfg['arm_force_limits_Nm'] + [cfg['finger_force_limit_N']] * 2
        if (_numeric(limits, (9,), 'force limits') <= 0).any():
            raise ValueError('Force limits must be positive')
        _numeric(protocol['cube']['center'], (3,), 'cube center')
        if not isinstance(protocol['joint_plan'], dict):
            raise ValueError('joint_plan must be an object')
    except (KeyError, TypeError) as exc:
        raise ValueError('Incomplete grasp protocol: ' + str(exc)) from exc
    t = arrays['time']
    if not np.isclose(t[0], dt, rtol=0, atol=1e-9) or not np.allclose(np.diff(t), dt, rtol=0, atol=1e-9):
        raise ValueError('NPZ time does not start at one timestep or has gaps/resets')
    rows = []
    for line_no, line in enumerate((directory / 'contacts.jsonl').read_text().splitlines()):
        try:
            row = _json(line)
            if not isinstance(row, dict) or not isinstance(row.get('events'), list):
                raise ValueError('Expected events list')
            if line_no >= n or row.get('phase') != str(phase[line_no]):
                raise ValueError('JSONL/NPZ phase or row count mismatch')
            rt = _numeric(row.get('time'), (), 'JSONL time')
            if not np.isclose(rt, t[line_no], rtol=0, atol=1e-10):
                raise ValueError('JSONL/NPZ time mismatch')
            for e in row['events']:
                for key in ('bodies', 'geoms'):
                    if not isinstance(e.get(key), list) or len(e[key]) != 2 or any(not isinstance(v, str) or not v for v in e[key]):
                        raise ValueError('Expected two named ' + key)
                for key, shape in [('contact_frame', (9,)), ('force_contact_frame', (6,)),
                                   ('normal', (3,)), ('position_m', (3,)), ('distance_m', ())]:
                    _numeric(e.get(key), shape, key)
                frame = np.asarray(e['contact_frame']).reshape(3, 3)
                if not np.allclose(frame @ frame.T, np.eye(3), rtol=0, atol=1e-6) or not np.allclose(e['normal'], frame[0], rtol=0, atol=1e-9):
                    raise ValueError('Contact frame is not orthonormal or normal disagrees')
                if not isinstance(e.get('constraint_address'), int):
                    raise ValueError('Expected integer constraint_address')
                if e['bodies'].count('cube') > 1:
                    raise ValueError('Ambiguous cube contact')
            rows.append(row)
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            raise ValueError(f'contacts.jsonl row {line_no}: {exc}') from exc
    if len(rows) != n:
        raise ValueError(f'JSONL/NPZ row count mismatch: {len(rows)} != {n}')
    return arrays, rows, protocol


def contact_series(rows):
    """Per-row cube contacts: Fn+, sum |Ft|, signed force on cube in world axes."""
    series = {g: dict(normal=np.zeros(len(rows)), tangent=np.zeros(len(rows)),
                      world=np.zeros((len(rows), 3)), count=np.zeros(len(rows), dtype=int)) for g in GROUPS}
    penetration = np.zeros(len(rows))
    for i, row in enumerate(rows):
        for e in row['events']:
            if 'cube' not in e['bodies']:
                continue
            cube = e['bodies'].index('cube'); other = e['bodies'][1 - cube]
            group = {'gripper_link1': 'finger1', 'gripper_link2': 'finger2'}.get(other)
            if group is None:
                group = 'table' if e['geoms'][1 - cube] == 'table' else 'other'
            f = np.asarray(e['force_contact_frame'][:3]); frame = np.asarray(e['contact_frame']).reshape(3, 3)
            s = series[group]; s['count'][i] += 1
            s['normal'][i] += max(0., f[0]); s['tangent'][i] += np.linalg.norm(f[1:])
            s['world'][i] += (1 if cube == 1 else -1) * (frame.T @ f)
            if other != 'world':
                penetration[i] = max(penetration[i], -e['distance_m'])
    return series, penetration


def _stats(a):
    return {key: np.asarray(value).tolist() for key, value in
            [('min', a.min(axis=0)), ('mean', a.mean(axis=0)), ('max', a.max(axis=0))]}


def _ends(a):
    return {'start': a[0].tolist(), 'end': a[-1].tolist()}


def analyze(directory):
    """Return JSON-serializable evidence and contiguous-phase summaries."""
    directory = Path(directory); a, rows, protocol = load_episode(directory)
    n = len(rows); cfg = protocol['config']; gp = protocol['grasp_protocol']; criteria = gp['criteria']
    series, penetration = contact_series(rows)
    trigger_limit = gp['contact_trigger_force_N_each']; support_limit = criteria['minimum_finger_normal_force_N']
    fingers = np.column_stack([series[g]['normal'] for g in GROUPS[:2]])
    coupling = np.abs(a['qpos'][:, 7] + a['qpos'][:, 8])
    limits = np.asarray(cfg['arm_force_limits_Nm'] + [cfg['finger_force_limit_N']] * 2)
    saturated = np.abs(a['actuator_force']) >= limits - 1e-8
    cube_pos = a['qpos'][:, 9:12]; origin = np.asarray(protocol['cube']['center'])
    events = {}

    def event(name, index):
        if index is None:
            events[name] = None
            return
        i = int(index)
        events[name] = dict(row=i, time_s=float(a['time'][i]), phase=str(a['phase'][i]),
                            state={k: a[k][i].tolist() for k in WIDTHS}, contacts=rows[i]['events'],
                            source=dict(npz_row=i, jsonl_line=i + 1),
                            finger_normal_force_N=fingers[i].tolist(),
                            table_normal_force_N=float(series['table']['normal'][i]),
                            table_world_force_N=series['table']['world'][i].tolist())

    def first(mask):
        found = np.flatnonzero(mask)
        return int(found[0]) if len(found) else None

    for j in range(2):
        event(f'finger{j + 1}_first_positive_normal', first(fingers[:, j] > 0))
    trigger = first(np.all(fingers >= trigger_limit, axis=1))
    event('bilateral_trigger', trigger)
    event('bilateral_support_loss_after_trigger', None if trigger is None else
          first((np.arange(n) > trigger) & ~np.all(fingers >= support_limit, axis=1)))
    event('robot_penetration', first(penetration > criteria['maximum_robot_object_penetration_m']))
    event('coupling_limit', first(coupling > cfg['acceptance']['maximum_coupling_error_m']))
    for kind, present in [('geometry', series['table']['count'] > 0),
                          ('support', series['table']['world'][:, 2] > 1e-12)]:
        initial = first(present)
        leave = None if initial is None else first((np.arange(n) > initial) & ~present)
        returned = None if leave is None else first((np.arange(n) > leave) & present)
        event(f'table_{kind}_leave', leave); event(f'table_{kind}_return', returned)
    phases = []
    edges = np.r_[0, np.flatnonzero(a['phase'][1:] != a['phase'][:-1]) + 1, n]
    for segment, (start, stop) in enumerate(zip(edges[:-1], edges[1:])):
        s = slice(start, stop); phase = str(a['phase'][start]); q = a['qpos'][s, :9]
        event(f'phase_{segment}_start', start); event(f'phase_{segment}_end', stop - 1)
        if phase in ('lift', 'hold') and f'{phase}_start' not in events:
            event(f'{phase}_start', start)
        if phase == 'grasp_settle':
            event('grasp_settle_end', stop - 1)
        contact_summary = {}
        for g, value in series.items():
            contact_summary[g] = dict(normal_force_N=_stats(value['normal'][s]),
                                      tangent_force_N=_stats(value['tangent'][s]), world_force_N=_stats(value['world'][s]),
                                      geometric_rows=int(np.count_nonzero(value['count'][s])),
                                      positive_normal_rows=int(np.count_nonzero(value['normal'][s] > 0)),
                                      upward_support_rows=int(np.count_nonzero(value['world'][s, 2] > 1e-12)))
        phases.append(dict(segment=segment, phase=phase, start_row=int(start), end_row=int(stop - 1),
            start_time_s=float(a['time'][start]), end_time_s=float(a['time'][stop - 1]), samples=int(stop - start),
            duration_s=float((stop - start) * cfg['timestep_s']), contacts=contact_summary,
            bilateral_support_fraction=float(np.all(fingers[s] >= support_limit, axis=1).mean()),
            maximum_robot_penetration_m=float(penetration[s].max()),
            joints=dict(actual=_ends(q), requested=_ends(a['requested'][s]), command=_ends(a['command'][s]),
                physical_target=_ends(a['physical_target'][s]), actual_range=_stats(q),
                target_error_abs_max=np.abs(a['physical_target'][s] - q).max(axis=0).tolist(),
                velocity_abs_max=np.abs(a['qvel'][s, :9]).max(axis=0).tolist(),
                actuator_force_range=_stats(a['actuator_force'][s]), ctrl_range=_stats(a['ctrl'][s]),
                actuator_saturation_fraction=saturated[s].mean(axis=0).tolist(),
                actuator_limit_excess_max=np.maximum(np.abs(a['actuator_force'][s]) - limits, 0).max(axis=0).tolist(),
                coupling_residual_abs_max_m=float(coupling[s].max()),
                constraint_force_abs_max=np.abs(a['qfrc_constraint'][s]).max(axis=0).tolist()),
            cube=dict(position_m=_ends(cube_pos[s]), quaternion_wxyz=_ends(a['qpos'][s, 12:16]),
                displacement_from_initial_end_m=(cube_pos[stop - 1] - origin).tolist(),
                displacement_world_m=(cube_pos[stop - 1] - cube_pos[start]).tolist(),
                linear_velocity_world_m_s=_stats(a['qvel'][s, 9:12]),
                angular_velocity_rad_s=_stats(a['qvel'][s, 12:15]),
                speed_max_m_s=float(np.linalg.norm(a['qvel'][s, 9:12], axis=1).max()))))
    return dict(schema_version=1, episode_dir=str(directory.resolve()), samples=n,
        sources={name: dict(path=str((directory / name).resolve()), sha256=hashlib.sha256((directory / name).read_bytes()).hexdigest())
                 for name in ('trajectory.npz', 'contacts.jsonl', 'protocol.json')},
        thresholds=dict(bilateral_trigger_N_each=trigger_limit, bilateral_support_N_each=support_limit,
            robot_penetration_m=criteria['maximum_robot_object_penetration_m'],
            coupling_m=cfg['acceptance']['maximum_coupling_error_m'], table_upward_force_epsilon_N=1e-12),
        interpretation=dict(force_sampling='instantaneous post-mj_step mj_forward, not integrated or step-average',
            force_world='force on cube = (+ if cube is bodies[1] else -) * frame.T @ force_contact_frame[:3]',
            trigger='one sample only; does not establish stable grasp',
            phases='contiguous phase runs; identical lift waypoint labels cannot be separated',
            angular_velocity='saved cube freejoint rotational qvel, not transformed to world axes',
            equality='protected finger coupling residual qpos[7]+qpos[8]; not a solver efc residual',
            table='geometry and positive world-Z support reported separately; finger thresholds do not define table departure',
            success='diagnostic evidence only; frozen acceptance criteria unchanged'),
        external_force_abs_max={k: float(np.abs(a[k]).max()) for k in ('qfrc_applied', 'xfrc_applied')},
        events=events, phases=phases)


def write_report(report, output_dir):
    """Create a new output directory; never overwrite any existing directory."""
    output = Path(output_dir); output.mkdir(parents=True, exist_ok=False)
    (output / 'diagnosis.json').write_text(json.dumps(report, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n')
    fields = ['segment', 'phase', 'start_row', 'end_row', 'start_time_s', 'end_time_s', 'samples',
              'bilateral_support_fraction', 'maximum_robot_penetration_m', 'coupling_max_m',
              'finger1_Fn_mean_N', 'finger2_Fn_mean_N', 'table_Fn_mean_N', 'table_worldZ_mean_N',
              'table_geometry_rows', 'table_upward_support_rows', 'cube_end_x_m', 'cube_end_y_m', 'cube_end_z_m',
              'cube_speed_max_m_s', 'actuator_saturation_fraction_9d']
    with (output / 'phases.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for p in report['phases']:
            row = {k: p[k] for k in fields[:9]}
            row.update(coupling_max_m=p['joints']['coupling_residual_abs_max_m'],
                       cube_speed_max_m_s=p['cube']['speed_max_m_s'],
                       actuator_saturation_fraction_9d=json.dumps(p['joints']['actuator_saturation_fraction']))
            for g in ('finger1', 'finger2', 'table'):
                row[g + '_Fn_mean_N'] = p['contacts'][g]['normal_force_N']['mean']
            row.update(table_worldZ_mean_N=p['contacts']['table']['world_force_N']['mean'][2],
                       table_geometry_rows=p['contacts']['table']['geometric_rows'],
                       table_upward_support_rows=p['contacts']['table']['upward_support_rows'])
            row.update(dict(zip(('cube_end_x_m', 'cube_end_y_m', 'cube_end_z_m'), p['cube']['position_m']['end'])))
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    try:
        report = analyze(args.episode_dir); write_report(report, args.output_dir)
    except (ValueError, OSError) as exc:
        parser.exit(2, f'diagnose: {exc}\n')
    print(json.dumps(dict(samples=report['samples'], phases=len(report['phases']), output=str(args.output_dir))))


if __name__ == '__main__':
    main()
