"""Reduce one completed full-grasp record without importing MuJoCo or solving forces.

The original evaluator is executed verbatim, separately from stage diagnostics.
Only immutable saved arrays/contacts are read; output must be a new directory.
"""
import argparse
import ast
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('_full_grasp_saved', _HERE / 'diagnose.py')
_saved = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_saved)
OBS_SHAPES = dict(gripper_position_world_m=(3,), gripper_rotation_world=(3, 3),
                  cube_position_in_gripper_m=(3,), cube_rotation_in_gripper=(3, 3))
SOURCE_NAMES = ('trajectory.npz', 'contacts.jsonl', 'protocol.json', 'result.json',
                'grasp-result.json', 'model.xml', 'observations.npz', 'warnings.npz',
                'contact-parameters.jsonl', 'initial-state.npz')
FAILURE_NAMES = ('failure-state.npz', 'failure-state.json', 'failure-contacts.json')
STAGES = ('open', 'approach', 'close', 'grasp_settle', 'lift', 'hold', 'release')
SUPPORT_EPS_N = 1e-12
FALL_VZ_M_S = -1e-3
DROP_DESCRIPTOR_M = 1e-3


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def _npz(path):
    with np.load(path, allow_pickle=False) as saved:
        return {key: saved[key] for key in saved.files}


def _source_hashes(path):
    return {name: digest(path / name) for name in SOURCE_NAMES + FAILURE_NAMES if (path / name).is_file()}


def _failure_json_value(value):
    """Keep invalid failure-state numbers explicit without emitting invalid JSON."""
    a = np.asarray(value)
    if a.dtype.kind not in 'fiu':
        raise ValueError('Failure snapshot must contain numeric arrays')
    if a.dtype.kind != 'f' or np.isfinite(a).all():
        return a.tolist()
    safe = a.astype(object)
    safe[np.isnan(a)] = 'NaN'; safe[np.isposinf(a)] = 'Infinity'; safe[np.isneginf(a)] = '-Infinity'
    return safe.tolist()


def _termination(path, control, original, samples, warnings=None):
    metadata = (_saved._json((path / 'failure-state.json').read_text())
                if (path / 'failure-state.json').is_file() else None)
    state = (_npz(path / 'failure-state.npz') if (path / 'failure-state.npz').is_file() else None)
    contacts = (_saved._json((path / 'failure-contacts.json').read_text())
                if (path / 'failure-contacts.json').is_file() else None)
    if contacts is not None and not isinstance(contacts, list):
        raise ValueError('Failure contacts must be an independent saved event list')
    actual = control.get('actual_mj_steps')
    if not isinstance(actual, int) or actual < samples:
        raise ValueError('Actual physics step count is smaller than saved rows')
    return dict(status='FAIL' if original.get('error') or actual > samples else ('PASS' if samples else 'UNKNOWN'),
        actual_mj_steps=actual, saved_rows=samples, unrecorded_physics_steps=actual - samples,
        original_runner_status=original.get('status'), original_control_status=control.get('status'),
        original_error=original.get('error') or control.get('error'), failure_metadata=metadata,
        failure_state=None if state is None else {key: _failure_json_value(value) for key, value in state.items()},
        failure_contact_events=None if contacts is None else [dict(event_index=i, **event) for i, event in enumerate(contacts)],
        saved_warning_max_8d=None if warnings is None or not len(warnings) else warnings.max(axis=0).tolist(),
        interpretation='Independent terminal snapshot and contacts; never appended to accepted recorder rows or used for a new force solve')


def _zero_row_report(path, before, arrays, control, original):
    if arrays['time'].shape != (0,):
        raise ValueError('Malformed empty trajectory time')
    for key, value in arrays.items():
        if value.ndim < 1 or value.shape[0] != 0:
            raise ValueError('Nonempty array in zero-row trajectory: ' + key)
    for name in ('contacts.jsonl', 'contact-parameters.jsonl'):
        if (path / name).read_text().strip():
            raise ValueError('Contact rows present for zero-row trajectory')
    for name in ('observations.npz', 'warnings.npz'):
        if (path / name).is_file():
            for key, value in _npz(path / name).items():
                if value.ndim < 1 or value.shape[0] != 0:
                    raise ValueError('Nonempty zero-row auxiliary array: ' + name + ':' + key)
    termination = _termination(path, control, original, 0)
    after = _source_hashes(path)
    if before != after:
        raise RuntimeError('Episode inputs changed during read-only analysis')
    return dict(schema_version=2, episode_dir=str(path), status=original['status'], frozen_status=original['status'],
        diagnostic_status=termination['status'], original_grasp_result=original, original_control_result=control,
        frozen_evaluation=dict(output=None, agreement='UNKNOWN', source=frozen_evaluator()[1],
            reason='Original runner does not call evaluate_grasp without saved physics rows; no synthetic evaluation'),
        samples=0, actual_mj_steps=control['actual_mj_steps'], termination_evidence=termination,
        source_sha256_before=before, source_sha256_after=after, input_immutability='PASS',
        stages=[dict(phase=name, status='UNKNOWN', frozen_status='UNKNOWN', diagnostic_status='UNKNOWN',
                     checks={}, diagnostic_checks={}, metrics=None) for name in STAGES],
        recorder_phase_chunks=[dict(chunk=i, original_phase=p, metrics=None) for i, p in enumerate(control.get('phases', []))],
        contiguous_phase_spans=[], rows_without_completed_recorder_phase=0,
        earliest_violation=None, earliest_frozen_violation=None, earliest_diagnostic_event=None,
        violations=[], frozen_violations=[], diagnostic_events={}, events={},
        release=dict(status='UNKNOWN', reason='No accepted physics rows; inspect independent termination evidence'))


def frozen_evaluator():
    """Compile only the unchanged pure function, without importing run.py."""
    path = _HERE / 'run.py'
    source = path.read_text()
    node = next(n for n in ast.parse(source).body
                if isinstance(n, ast.FunctionDef) and n.name == 'evaluate_grasp')
    scope = {'np': np}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), scope)
    return scope['evaluate_grasp'], dict(path=str(path), sha256=digest(path),
        function_sha256=hashlib.sha256(ast.get_source_segment(source, node).encode()).hexdigest(),
        execution='Unchanged evaluate_grasp AST only; runner and MuJoCo are not imported')


def _first(mask):
    found = np.flatnonzero(mask)
    return int(found[0]) if len(found) else None


def _stats(values):
    values = np.asarray(values)
    return dict(min=values.min(axis=0).tolist(), mean=values.mean(axis=0).tolist(),
                max=values.max(axis=0).tolist(), start=values[0].tolist(), end=values[-1].tolist())


def _angle(rotation):
    return np.arccos(np.clip((np.trace(rotation, axis1=-2, axis2=-1) - 1.) / 2., -1., 1.))


def _rotation(value, shape, label):
    value = _saved._numeric(value, shape, label)
    if (not np.allclose(value @ np.swapaxes(value, -1, -2), np.eye(3), rtol=0, atol=1e-6)
            or not np.allclose(np.linalg.det(value), 1., rtol=0, atol=1e-6)):
        raise ValueError(label + ': expected proper orthonormal rotation')


def _load_extra(path, a, rows):
    n = len(rows)
    obs = _npz(path / 'observations.npz')
    for key, shape in OBS_SHAPES.items():
        _saved._numeric(obs[key], (n,) + shape, key)
        if 'rotation' in key:
            _rotation(obs[key], (n,) + shape, key)
    _saved._numeric(obs['time'], (n,), 'observation time')
    if not np.allclose(obs['time'], a['time'], rtol=0, atol=1e-10):
        raise ValueError('Observation/trajectory time mismatch')
    predicted = obs['gripper_position_world_m'] + np.einsum(
        'nij,nj->ni', obs['gripper_rotation_world'], obs['cube_position_in_gripper_m'])
    if not np.allclose(predicted, a['qpos'][:, 9:12], rtol=0, atol=1e-8):
        raise ValueError('Saved relative/world cube positions disagree')
    initial = _npz(path / 'initial-state.npz')
    for key, shape in dict(qpos=(16,), qvel=(15,), time=(1,), **OBS_SHAPES).items():
        _saved._numeric(initial[key], shape, 'initial ' + key)
        if 'rotation' in key:
            _rotation(initial[key], shape, 'initial ' + key)
    if initial['time'][0] != 0:
        raise ValueError('Full episode must have legal initial time zero')
    warnings = _npz(path / 'warnings.npz')
    _saved._numeric(warnings['time'], (n,), 'warning time')
    if not np.allclose(warnings['time'], a['time'], rtol=0, atol=1e-10):
        raise ValueError('Warning/trajectory time mismatch')
    numbers = warnings['numbers']
    if numbers.shape != (n, 8) or numbers.dtype.kind not in 'iu' or np.any(numbers < 0):
        raise ValueError('Expected nonnegative N x 8 integer warning numbers')
    with (path / 'contact-parameters.jsonl').open() as stream:
        count = 0
        for i, line in enumerate(stream):
            meta = _saved._json(line)
            if (i >= n or meta.get('time') != rows[i]['time']
                    or len(meta.get('contacts', [])) != len(rows[i]['events'])):
                raise ValueError(f'Contact parameter alignment mismatch at row {i}')
            count += 1
        if count != n:
            raise ValueError('Contact parameter row count mismatch')
    return obs, initial, numbers


def analyze(episode):
    """Return stage evidence, original acceptance, and immutable-input hashes."""
    path = Path(episode).resolve()
    before = _source_hashes(path)
    control = _saved._json((path / 'result.json').read_text())
    original = _saved._json((path / 'grasp-result.json').read_text())
    with np.load(path / 'trajectory.npz', allow_pickle=False) as saved:
        if saved['time'].size == 0:
            return _zero_row_report(path, before, {key: saved[key] for key in saved.files}, control, original)
    a, rows, protocol = _saved.load_episode(path)
    obs, initial, warnings = _load_extra(path, a, rows)
    termination = _termination(path, control, original, len(rows), warnings)
    cfg = protocol['config']; gp = protocol['grasp_protocol']; criteria = gp['criteria']
    dt = cfg['timestep_s']; n = len(rows); times = a['time']; phase = a['phase']
    acceptance = cfg['acceptance']; indices = np.arange(n)
    series, penetration = _saved.contact_series(rows)
    fingers = np.column_stack([series[g]['normal'] for g in ('finger1', 'finger2')])
    bilateral = np.all(fingers >= criteria['minimum_finger_normal_force_N'], axis=1)
    coupling = np.abs(a['qpos'][:, 7] + a['qpos'][:, 8])
    heights = a['qpos'][:, 11] - protocol['cube']['center'][2]
    world_gap = a['qpos'][:, 11] - obs['gripper_position_world_m'][:, 2]
    width = a['qpos'][:, 7] - a['qpos'][:, 8]
    limits = np.asarray(cfg['arm_force_limits_Nm'] + [cfg['finger_force_limit_N']] * 2)
    saturated = np.abs(a['actuator_force']) >= limits - 1e-8
    nonfinger = np.zeros(n, dtype=bool); any_nonfinger_upward = np.zeros(n, dtype=bool)
    unexpected = np.zeros(n, dtype=bool)
    for i, row in enumerate(rows):
        for e in row['events']:
            if 'cube' in e['bodies']:
                other = e['bodies'][1 - e['bodies'].index('cube')]
                if other not in ('gripper_link1', 'gripper_link2'):
                    nonfinger[i] |= max(0., e['force_contact_frame'][0]) > criteria['minimum_finger_normal_force_N']
            elif (set(e['bodies']) != {'base_link', 'link1'} and e['distance_m'] < -1e-5):
                unexpected[i] = True
        any_nonfinger_upward[i] = any(series[g]['world'][i, 2] > SUPPORT_EPS_N for g in ('table', 'other'))
    evaluator, evaluator_source = frozen_evaluator()
    evaluated = evaluator(a, rows, 9, protocol['cube']['center'], criteria, dt)
    # execute_grasp overrides only status after evaluating the frozen criteria.
    recomposed_status = ('INTERRUPTED' if original.get('error') else
                         ('FAIL' if control['status'] != 'PASS' else evaluated['status']))
    disagreements = {key: dict(saved=original.get(key), recomputed=value)
                     for key, value in evaluated.items() if key != 'status' and original.get(key) != value}
    if original.get('status') != recomposed_status:
        disagreements['status'] = dict(saved=original.get('status'), recomputed=recomposed_status)
    if original.get('control_health_status') != control.get('status'):
        disagreements['control_health_status'] = dict(saved=original.get('control_health_status'), recomputed=control.get('status'))
    if original.get('actual_mj_steps') != control.get('actual_mj_steps'):
        disagreements['actual_mj_steps'] = dict(saved=original.get('actual_mj_steps'), recomputed=control.get('actual_mj_steps'))

    events = {}; violations = []; diagnostic_events = {}

    def event(name, row, *, failure=False, meaning=None, category='frozen'):
        if row is None:
            events[name] = None
            return None
        i = int(row)
        value = dict(row=i, physics_step=i + 1, time_s=float(times[i]), phase=str(phase[i]),
            source=dict(trajectory_row=i, observations_row=i, contacts_jsonl_line=i + 1),
            qpos=a['qpos'][i].tolist(), qvel=a['qvel'][i].tolist(),
            requested=a['requested'][i].tolist(), command=a['command'][i].tolist(),
            physical_target=a['physical_target'][i].tolist(),
            contact_events=[dict(event_index=j, **e) for j, e in enumerate(rows[i]['events'])],
            finger_normal_force_N=fingers[i].tolist(), table_world_force_N=series['table']['world'][i].tolist(),
            cube_world_z_m=float(a['qpos'][i, 11]), lift_m=float(heights[i]),
            coupling_error_m=float(coupling[i]), robot_object_penetration_m=float(penetration[i]),
            warning_numbers=warnings[i].tolist(), failure=failure, meaning=meaning,
            category=category if failure or category == 'diagnostic' else 'observation')
        events[name] = value
        if category == 'diagnostic':
            diagnostic_events[name] = value
        elif failure:
            violations.append(dict(event=name, row=i, time_s=float(times[i]), phase=str(phase[i]), meaning=meaning))
        return value

    event('first_record', 0); event('last_record', n - 1)
    event('first_penetration_violation', _first(penetration > criteria['maximum_robot_object_penetration_m']), failure=True)
    event('first_coupling_violation', _first(coupling > acceptance['maximum_coupling_error_m']), failure=True)
    event('first_arm_velocity_violation', _first(np.max(np.abs(a['qvel'][:, :7]), axis=1) > acceptance['maximum_arm_velocity_rad_s']), failure=True)
    event('first_finger_velocity_violation', _first(np.max(np.abs(a['qvel'][:, 7:9]), axis=1) > acceptance['maximum_finger_velocity_m_s']), failure=True)
    event('first_numerical_warning', _first(np.any(warnings > 0, axis=1)), failure=True)
    event('first_external_force', _first(np.any(a['qfrc_applied'] != 0, axis=1) | np.any(a['xfrc_applied'] != 0, axis=(1, 2))), failure=True)
    event('approach_unexpected_nonobject_penetration', _first((phase == 'approach') & unexpected), failure=True, category='diagnostic',
          meaning='Diagnostic unexpected collision: nonobject contact deeper than frozen planning 10um screen')
    trigger = _first((phase == 'close') & np.all(fingers >= gp['contact_trigger_force_N_each'], axis=1))
    event('close_bilateral_trigger', trigger, meaning='One actual close sample; not proof of stable grasp')
    close = np.flatnonzero(phase == 'close')
    close_completed_failed = any(p.get('phase') == 'close' and p.get('status') == 'FAIL' for p in control.get('phases', []))
    if len(close) and trigger is None and close_completed_failed:
        event('close_ended_without_trigger', close[-1], failure=True)
    hold = phase == 'hold'; hold_rows = np.flatnonzero(hold)
    event('hold_first_bilateral_support_loss', _first(hold & ~bilateral),
          meaning='Descriptive sample loss; 95 percent criterion is evaluated over hold only')
    event('hold_lift_height_violation', _first(hold & (heights < criteria['minimum_lift_m'])), failure=True)
    event('hold_nonfinger_support_violation', _first(hold & nonfinger), failure=True,
          meaning='Frozen evaluator threshold on each nonfinger contact normal force')
    event('hold_any_nonfinger_upward_support', _first(hold & any_nonfinger_upward), category='diagnostic',
          meaning='Independent descriptive positive upward force; no 0.05N cutoff')
    drift = np.zeros(n)
    if len(hold_rows):
        drift = np.linalg.norm(a['qpos'][:, 9:12] - a['qpos'][hold_rows[0], 9:12], axis=1)
        event('hold_position_drift_violation', _first(hold & (drift > criteria['maximum_hold_drift_m'])), failure=True)
        budget = (1. - criteria['minimum_bilateral_contact_fraction']) * len(hold_rows)
        lost = np.cumsum(hold & ~bilateral)
        event('hold_bilateral_loss_exceeds_full_duration_budget', _first(hold & (lost > budget + 1e-8)),
              failure=not evaluated['checks']['bilateral_finger_support'],
              meaning='Earliest loss exceeding the final saved hold-sample denominator budget; retrospective frozen fraction evidence')

    def metrics(mask):
        r = np.flatnonzero(mask)
        if not len(r):
            return None
        first = r[0]; relative = obs['cube_position_in_gripper_m'][r] - obs['cube_position_in_gripper_m'][first]
        rotation = obs['cube_rotation_in_gripper'][r] @ obs['cube_rotation_in_gripper'][first].T
        return dict(start_row=int(first), end_row=int(r[-1]), samples=len(r), duration_s=float(len(r) * dt),
            start_time_s=float(times[first]), end_time_s=float(times[r[-1]]),
            cube_world_position_m=_stats(a['qpos'][r, 9:12]), cube_world_z_m=_stats(a['qpos'][r, 11]),
            cube_lift_m=_stats(heights[r]), cube_world_vz_m_s=_stats(a['qvel'][r, 11]),
            cube_position_in_gripper_m=_stats(obs['cube_position_in_gripper_m'][r]),
            relative_translation_m=_stats(relative), relative_translation_norm_m=_stats(np.linalg.norm(relative, axis=1)),
            relative_world_z_drop_m=_stats(world_gap[first] - world_gap[r]),
            relative_rotation_angle_rad=_stats(_angle(rotation)),
            contacts={g: dict(normal_force_N=_stats(s['normal'][r]), tangent_force_N=_stats(s['tangent'][r]),
                              world_force_N=_stats(s['world'][r]), geometric_rows=int(np.count_nonzero(s['count'][r])),
                              positive_upward_rows=int(np.count_nonzero(s['world'][r, 2] > SUPPORT_EPS_N)))
                      for g, s in series.items()},
            bilateral_support_fraction=float(bilateral[r].mean()),
            maximum_robot_object_penetration_m=float(penetration[r].max()), maximum_coupling_error_m=float(coupling[r].max()),
            actual_width_m=_stats(width[r]), commanded_width_m=_stats(a['command'][r, 7]),
            joint_target_error_abs_max_9d=np.abs(a['physical_target'][r] - a['qpos'][r, :9]).max(axis=0).tolist(),
            maximum_arm_velocity_rad_s=float(np.abs(a['qvel'][r, :7]).max()),
            maximum_finger_velocity_m_s=float(np.abs(a['qvel'][r, 7:9]).max()),
            actuator_saturation_fraction_9d=saturated[r].mean(axis=0).tolist(),
            numerical_warning_max_8d=warnings[r].max(axis=0).tolist())

    chunks = []; coverage = np.zeros(n, dtype=int)
    for chunk, saved_phase in enumerate(control.get('phases', [])):
        mask = ((phase == saved_phase['phase']) & (times > saved_phase['start_time_s'] + 1e-10)
                & (times <= saved_phase['end_time_s'] + 1e-10))
        coverage += mask
        chunks.append(dict(chunk=chunk, original_phase=saved_phase, metrics=metrics(mask)))
        if saved_phase.get('status') == 'FAIL' and np.any(mask):
            event(f'frozen_control_phase_{chunk}_failure', np.flatnonzero(mask)[-1], failure=True,
                  meaning='Frozen phase outcome; tracking checked over its settled final 250ms, not the whole motion')
    if np.any(coverage > 1):
        raise ValueError('Overlapping saved Recorder phase chunks')
    # Keep physical phase spans even if an exception prevented Recorder.phase finishing.
    edges = np.r_[0, np.flatnonzero(phase[1:] != phase[:-1]) + 1, n]
    spans = [dict(phase=str(phase[start]), metrics=metrics((indices >= start) & (indices < stop)))
             for start, stop in zip(edges[:-1], edges[1:])]

    release_rows = np.flatnonzero(phase == 'release')
    release = dict(status='UNKNOWN', reason='Release was not reached')
    if len(release_rows):
        first = release_rows[0]; last = release_rows[-1]
        observed_bilateral_no_nonfinger_support = False
        clearance = None; positive_lift = False
        if len(hold_rows):
            prior = hold_rows[-1]
            cube_rotation_world = obs['gripper_rotation_world'][prior] @ obs['cube_rotation_in_gripper'][prior]
            half_extent_z = np.abs(cube_rotation_world[2]) @ np.asarray(protocol['cube']['half_size'])
            table = protocol['cube']['table']
            clearance = float(a['qpos'][prior, 11] - half_extent_z - table['center'][2] - table['half_size'][2])
            positive_lift = bool(heights[prior] > 0)
            observed_bilateral_no_nonfinger_support = bool(bilateral[prior] and all(
                series[group]['normal'][prior] <= SUPPORT_EPS_N for group in ('table', 'other')))
        release_prerequisite = bool(observed_bilateral_no_nonfinger_support and positive_lift and clearance > 0)
        command_open = bool(np.isclose(a['command'][last, 7], gp['open_width_m'], rtol=0, atol=1e-8))
        actual_open = bool(np.all(np.abs(a['qpos'][last, 7:9] - np.array([1., -1.]) * gp['open_width_m'] / 2)
                                  <= acceptance['settled_finger_error_m']))
        separated_mask = ((indices >= first) & (phase == 'release') &
                          (series['finger1']['count'] == 0) & (series['finger2']['count'] == 0) & ~bilateral)
        separated = _first(separated_mask)
        falling = _first((phase == 'release') & (a['qvel'][:, 11] < FALL_VZ_M_S)
                         & (a['qpos'][first, 11] - a['qpos'][:, 11] >= DROP_DESCRIPTOR_M))
        contact_drop = bool(release_prerequisite and separated is not None)
        release_observed = bool(command_open and actual_open and contact_drop and falling is not None)
        release = dict(status='PASS' if release_observed else 'UNKNOWN',
            observed_bilateral_no_nonfinger_support=observed_bilateral_no_nonfinger_support,
            positive_lift_before_release=positive_lift, cube_bottom_clearance_above_table_m=clearance,
            commanded_opening_reached=command_open, actual_opening_reached=actual_open,
            separated_fingers_observed=separated is not None, contact_loss_after_observed_hold=contact_drop,
            cube_fall_after_release_observed=falling is not None,
            reason=('Bilateral contact without nonfinger force and cube clear of table was not established before release'
                    if not release_prerequisite else
                    ('Physical release observed after bilateral contact without nonfinger force, above table; this does not establish stable grasp or the frozen 80mm lift criterion'
                     if release_observed else 'Opening, contact-loss or fall evidence incomplete after observed bilateral contact above table; physical release UNKNOWN')),
            fall_descriptor=dict(world_vz_strictly_below_m_s=FALL_VZ_M_S, world_z_drop_at_least_m=DROP_DESCRIPTOR_M))
        event('release_start', first); event('release_finger_separation', separated, category='diagnostic')
        event('release_cube_fall', falling, category='diagnostic')
        if release['status'] == 'UNKNOWN':
            event('release_not_established_at_end', last, category='diagnostic',
                  meaning='Release evidence UNKNOWN; descriptive fall thresholds are not frozen acceptance criteria')

    lift_rows = np.flatnonzero(phase == 'lift')
    if len(lift_rows) and heights[lift_rows[-1]] < criteria['minimum_lift_m']:
        event('lift_ended_below_frozen_height', lift_rows[-1], failure=True, category='diagnostic',
              meaning='Independent lift-end observation; frozen lift acceptance is checked during hold')
    if len(lift_rows) and series['table']['world'][lift_rows[-1], 2] > SUPPORT_EPS_N:
        event('lift_ended_with_table_support', lift_rows[-1], failure=True, category='diagnostic',
              meaning='Independent lift-end observation of positive table world-Z force')
    stages = []
    for name in STAGES:
        mask = phase == name; r = np.flatnonzero(mask)
        relevant = [v['event'] for v in violations if v['phase'] == name]
        frozen_chunks = [i for i, c in enumerate(chunks) if c['original_phase']['phase'] == name]
        checks = {}; diagnostic_checks = {}; status = 'UNKNOWN'; diagnostic_status = 'UNKNOWN'
        if len(r):
            status = 'FAIL' if relevant else 'PASS'
            completed = bool(frozen_chunks) and not np.any(mask & (coverage == 0))
            if name == 'close' and completed:
                checks['bilateral_trigger'] = trigger is not None
            elif name == 'lift':
                diagnostic_checks['end_height_at_least_frozen_minimum'] = bool(heights[r[-1]] >= criteria['minimum_lift_m'])
                diagnostic_checks['no_table_support_at_end'] = bool(series['table']['world'][r[-1], 2] <= SUPPORT_EPS_N)
            elif name == 'hold' and completed:
                longest = max((stop - start for start, stop in zip(edges[:-1], edges[1:]) if phase[start] == 'hold'), default=0)
                checks.update({key: value for key, value in evaluated['checks'].items()
                               if key != 'robot_object_penetration'})
                diagnostic_checks['continuous_hold'] = bool(longest * dt >= criteria['hold_duration_s'] - dt / 2)
            elif name == 'release':
                diagnostic_checks = dict(release)
            if any(value is False for value in checks.values()):
                status = 'FAIL'
            if not completed:
                status = 'UNKNOWN'
            diagnostic_status = status
            if name == 'release':
                diagnostic_status = release['status']
            elif any(value is False for value in diagnostic_checks.values()) or any(
                    e['phase'] == name and e['failure'] for e in diagnostic_events.values()):
                diagnostic_status = 'FAIL'
        stages.append(dict(phase=name, status=status, frozen_status=status, diagnostic_status=diagnostic_status,
                           checks=checks, diagnostic_checks=diagnostic_checks, violation_events=relevant,
                           frozen_control_chunks=frozen_chunks, metrics=metrics(mask),
                           force_expectation=('No grasp force required' if name in ('open', 'approach') else
                                              'Hold force fraction evaluated only during hold')))
    if evaluated['failed_criteria'] and not original.get('error'):
        event('frozen_criteria_unmet_at_episode_end', n - 1, failure=True,
              meaning='Whole-episode evaluator outcome, not a release-stage check: ' + ', '.join(evaluated['failed_criteria']))
    violations.sort(key=lambda value: (value['row'], value['event']))
    first_failure = None if not violations else dict(violations[0], evidence=events[violations[0]['event']])
    if first_failure is None and original.get('error'):
        failure_state = termination.get('failure_state') or {}
        metadata = termination.get('failure_metadata') or {}
        first_failure = dict(event='original_runner_interruption', row=None,
            time_s=(failure_state.get('time') or [float(times[-1])])[0], phase=metadata.get('phase', str(phase[-1])),
            meaning=termination['original_error'], evidence=termination)
    first_diagnostic = min(diagnostic_events.values(), key=lambda e: e['row']) if diagnostic_events else None
    hold_continuous = next(s for s in stages if s['phase'] == 'hold')['diagnostic_checks'].get('continuous_hold')
    loopholes = [dict(issue='Release is not part of evaluate_grasp checks', evidence=release),
        dict(issue='evaluate_grasp sums hold samples rather than requiring one continuous interval',
             observed_violation=bool(evaluated['checks']['complete_hold'] and hold_continuous is False)),
        dict(issue='Nonfinger support excludes contacts with individual Fn <= 0.05N',
             observed_positive_upward_rows_below_frozen_cutoff=int(np.count_nonzero(hold & any_nonfinger_upward & ~nonfinger))),
        dict(issue='Close trigger and frozen controller phase results are not evaluate_grasp checks',
             close_trigger_observed=trigger is not None, original_control_status=control['status'])]
    after = _source_hashes(path)
    if before != after:
        raise RuntimeError('Episode inputs changed during read-only analysis')
    diagnostic_status = ('FAIL' if any(s['diagnostic_status'] == 'FAIL' for s in stages)
                         else ('PASS' if all(s['diagnostic_status'] == 'PASS' for s in stages) else 'UNKNOWN'))
    table = protocol['cube']['table']
    return dict(schema_version=2, episode_dir=str(path), status=original['status'], frozen_status=original['status'],
        diagnostic_status=diagnostic_status, termination_evidence=termination,
        original_grasp_result=original, original_control_result=control,
        frozen_evaluation=dict(output=evaluated, reconstructed_runner_status=recomposed_status,
                               agreement='PASS' if not disagreements else 'FAIL', disagreements=disagreements, source=evaluator_source),
        samples=n, actual_mj_steps=control['actual_mj_steps'], source_sha256_before=before, source_sha256_after=after,
        input_immutability='PASS', stages=stages, recorder_phase_chunks=chunks, contiguous_phase_spans=spans,
        rows_without_completed_recorder_phase=int(np.count_nonzero(coverage == 0)),
        earliest_violation=first_failure, earliest_frozen_violation=first_failure,
        earliest_diagnostic_event=first_diagnostic, violations=violations, frozen_violations=violations,
        diagnostic_events=diagnostic_events, events=events, release=release, evaluator_limitations=loopholes,
        initial_scene=dict(time_s=float(initial['time'][0]), cube_center_m=initial['qpos'][9:12].tolist(),
            table_top_world_z_m=table['center'][2] + table['half_size'][2],
            nominal_cube_bottom_world_z_m=protocol['cube']['center'][2] - protocol['cube']['half_size'][2],
            first_recorded_table_geometric_contacts=int(series['table']['count'][0]),
            first_recorded_table_upward_force_N=float(series['table']['world'][0, 2]),
            saved_plan_nonobject_scratch_collision_count=protocol['joint_plan'].get('nonobject_scratch_collision_count')),
        interpretation=dict(acceptance='Frozen evaluator output and controller results preserved; independent diagnostics do not modify criteria',
            stage_status='Only applicable frozen criteria and original Recorder phase checks; incomplete phases UNKNOWN; diagnostic status/checks separate',
            force_sampling='Immutable instantaneous post-step mj_forward samples; no integration or new force solve',
            force_world='Force on cube = (+ if cube is bodies[1] else -) * frame.T @ force_contact_frame[:3]',
            relative_pose='Per phase/chunk relative translation and rotation reference its first recorded sample',
            relative_z_drop='Decrease of world-Z gap cube_COM.z - gripper_base_origin.z; not local body Z',
            tracking='Whole-phase target-error maximum is descriptive; frozen tracking checks use final 250ms',
            violation_timing='Frozen sample/phase outcomes; support timing uses final saved hold denominator retrospectively; interruption state separate',
            release='Independent opening, contact-loss and fall evidence after bilateral contact without nonfinger force and positive table clearance; frozen lift height remains separate',
            row_numbering='Rows and contact event indices zero based; physics_step and JSONL lines one based',
            diagnostics='10um unexpected nonobject penetration and release fall descriptors are diagnostic observations, not rewritten frozen criteria'))


def write_report(report, output_dir):
    output = Path(output_dir).resolve(); episode = Path(report['episode_dir'])
    if output == episode or episode in output.parents:
        raise ValueError('Analysis output must be outside the immutable episode')
    output.mkdir(parents=True, exist_ok=False)
    with (output / 'summary.json').open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False); stream.write('\n')
    with (output / 'events.csv').open('x', newline='') as stream:
        fields = ['event', 'row', 'physics_step', 'time_s', 'phase', 'failure', 'meaning', 'category',
                  'finger1_Fn_N', 'finger2_Fn_N', 'table_worldZ_N', 'cube_world_z_m',
                  'coupling_error_m', 'robot_object_penetration_m', 'contacts_jsonl_line']
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for name, event in sorted(report['events'].items(),
                                  key=lambda item: (float('inf') if item[1] is None else item[1]['row'], item[0])):
            if event is None: continue
            row = {key: event[key] for key in fields[1:8]}
            row.update(event=name, finger1_Fn_N=event['finger_normal_force_N'][0], finger2_Fn_N=event['finger_normal_force_N'][1],
                       table_worldZ_N=event['table_world_force_N'][2], cube_world_z_m=event['cube_world_z_m'],
                       coupling_error_m=event['coupling_error_m'], robot_object_penetration_m=event['robot_object_penetration_m'],
                       contacts_jsonl_line=event['source']['contacts_jsonl_line'])
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    # Fail before expensive reads if output would overwrite an existing path.
    if args.output_dir.exists(): raise FileExistsError(args.output_dir)
    report = analyze(args.episode)
    write_report(report, args.output_dir)
    print(json.dumps(dict(status=report['status'], frozen_status=report['frozen_status'], diagnostic_status=report['diagnostic_status'],
                          evaluator_agreement=report['frozen_evaluation']['agreement'],
                          earliest_violation=None if report['earliest_violation'] is None else
                          {key: report['earliest_violation'][key] for key in ('event', 'row', 'time_s', 'phase')},
                          output_dir=str(args.output_dir.resolve()))))


if __name__ == '__main__':
    main()
