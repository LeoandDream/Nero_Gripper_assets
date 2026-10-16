"""Issue #2 approved diagnostic-only A/B; never evaluates official grasp success.

prepare: replay the frozen prefix once and save/audit identical full initial states.
seal: bind both groups' approved models, protocols, and initial states before run.
run: integrate one 14.1 s A/B from those saved states; only table z differs.
"""
import argparse
import copy
import difflib
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from controller import ASSET_ROOT, Controller, build_model
import run as frozen

FREEZE = '1e4d633730f62a398513d33991fa96f9c4977daa'
TRAJECTORY_SHA = 'a1bc4f26640e1b6575f682de0d2b1f71592865816ea794d3b29b84b9b2c8da49'
STATE_SPEC = mujoco.mjtState.mjSTATE_INTEGRATION
STEPS = 14100


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def model_variants(text, source_dir):
    """Canonical asset paths, then one approved XML attribute component change."""
    root = ET.fromstring(text)
    table = root.find('.//geom[@name="table"]')
    if table is None:
        raise ValueError('Missing approved table')
    pos = table.get('pos', '').split()
    if len(pos) != 3 or float(pos[2]) != .065:
        raise ValueError('A table must start at approved z=.065')
    for node in root.iter():
        if node.get('file'):
            node.set('file', str((Path(source_dir) / node.get('file')).resolve()))
    ET.indent(root)
    a = ET.tostring(root, encoding='unicode') + '\n'
    table.set('pos', ' '.join(pos[:2] + ['0.015']))
    b = ET.tostring(root, encoding='unicode') + '\n'
    return a, b


def integration_state(model, data):
    state = np.empty(mujoco.mj_stateSize(model, STATE_SPEC))
    mujoco.mj_getState(model, data, state, STATE_SPEC)
    return state


def restore_state(model, state):
    """One legal initialization; derived contacts are recomputed by the caller."""
    state = np.asarray(state)
    if state.shape != (mujoco.mj_stateSize(model, STATE_SPEC),) or not np.isfinite(state).all():
        raise ValueError('Invalid complete integration state')
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_setState(model, data, state, STATE_SPEC)
    np.testing.assert_array_equal(integration_state(model, data), state)
    return data


def pose(model, data):
    gripper = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_base')
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cube')
    rotation = data.xmat[gripper].reshape(3, 3)
    return dict(gripper_position_world_m=data.xpos[gripper].copy(),
                gripper_rotation_world=rotation.copy(),
                cube_position_in_gripper_m=rotation.T @ (data.xpos[cube] - data.xpos[gripper]),
                cube_rotation_in_gripper=rotation.T @ data.xmat[cube].reshape(3, 3))


def verify_frozen():
    frozen.asset_integrity()
    for name in ('run.py', 'controller.py', 'config.json', 'grasp.json'):
        rel = 'experiments/dynamics/' + name
        expected = subprocess.check_output(['git', 'show', FREEZE + ':' + rel], cwd=ASSET_ROOT)
        if (ASSET_ROOT / rel).read_bytes() != expected:
            raise ValueError('Frozen executable changed: ' + name)


def model_check(a, b):
    changes = {}
    for name in dir(a):
        va, vb = getattr(a, name), getattr(b, name)
        if isinstance(va, np.ndarray) and not np.array_equal(va, vb):
            indices = np.argwhere(va != vb)
            changes[name] = [dict(index=i.tolist(), A=np.asarray(va[tuple(i)]).item(),
                                 B=np.asarray(vb[tuple(i)]).item()) for i in indices]
    table = mujoco.mj_name2id(a, mujoco.mjtObj.mjOBJ_GEOM, 'table')
    if changes.get('geom_pos') != [dict(index=[table, 2], A=.065, B=.015)]:
        raise ValueError('Unexpected compiled table delta: ' + repr(changes))
    if set(changes) != {'geom_pos', 'bvh_aabb'} or changes['bvh_aabb'] != [dict(index=[0, 2], A=.065, B=.015)]:
        raise ValueError('Unexpected compiled model differences: ' + repr(changes))
    for name in dir(a.opt):
        if name.startswith('_') or callable(getattr(a.opt, name)):
            continue
        if not np.array_equal(getattr(a.opt, name), getattr(b.opt, name)):
            raise ValueError('Solver/physics option differs: ' + name)
    return dict(array_differences=changes, all_other_public_model_arrays_equal=True,
                all_option_fields_equal=True,
                derived_display_statistics=dict(A=dict(extent=a.stat.extent, center=a.stat.center.tolist()),
                                                B=dict(extent=b.stat.extent, center=b.stat.center.tolist())),
                explanation='BVH and display bounds are compiler derivatives of the single table translation')


def prepare(source, output):
    verify_frozen()
    source = source.resolve(); output.mkdir(parents=True, exist_ok=False)
    if digest(source / 'trajectory.npz') != TRAJECTORY_SHA:
        raise ValueError('Wrong frozen baseline trajectory')
    p = json.loads((source / 'protocol.json').read_text()); cfg = p['config']
    with np.load(source / 'trajectory.npz', allow_pickle=False) as saved:
        a = {k: saved[k] for k in saved.files}
    boundary = int(np.flatnonzero(a['phase'] == 'lift')[0])
    if boundary != 29128 or a['phase'][boundary - 1] != 'grasp_settle' or cfg['timestep_s'] != .001:
        raise ValueError('Unexpected frozen boundary/timestep')
    model = build_model(output / 'prefix-model.xml', cfg, exclude_base_link1=True, cube=p['cube'])
    canonical, lowered = model_variants((output / 'prefix-model.xml').read_text(), output)
    baseline_canonical, _ = model_variants((source / 'model.xml').read_text(), source)
    if canonical != baseline_canonical:
        raise ValueError('A model does not reproduce frozen baseline XML')
    models = {}
    for label, xml in [('A', canonical), ('B', lowered)]:
        dest = output / label; dest.mkdir()
        (dest / 'model.xml').write_text(xml)
        models[label] = mujoco.MjModel.from_xml_path(str(dest / 'model.xml'))
    diff = model_check(models['A'], models['B'])
    (output / 'model.diff').write_text(''.join(difflib.unified_diff(canonical.splitlines(True), lowered.splitlines(True), fromfile='A/model.xml', tofile='B/model.xml')))
    frozen.write_json(output / 'model-diff.json', diff)
    c = Controller(model, cfg); data = frozen.reset(model, c, p['joint_plan']['initial_action'])
    maximum = {k: 0. for k in ('qpos', 'qvel', 'ctrl', 'actuator_force', 'qfrc_constraint')}
    for i in range(boundary):
        c.requested = a['requested'][i].copy(); c.command = a['command'][i].copy()
        c.apply(data); mujoco.mj_step(model, data); mujoco.mj_forward(model, data)
        for key in maximum:
            maximum[key] = max(maximum[key], float(np.max(np.abs(getattr(data, key) - a[key][i]))))
        if max(maximum.values()) > 1e-12 or abs(data.time - a['time'][i]) > 1e-12 or np.any(data.warning.number):
            raise ValueError(f'Frozen prefix diverged at row {i}: {maximum}')
    np.testing.assert_array_equal(c.requested, c.command)
    snapshot = dict(integration_state=integration_state(model, data), time=np.array([data.time]),
                    qpos=data.qpos.copy(), qvel=data.qvel.copy(), qacc_warmstart=data.qacc_warmstart.copy(),
                    ctrl=data.ctrl.copy(), controller_requested=c.requested.copy(), controller_command=c.command.copy(),
                    **pose(model, data))
    np.savez_compressed(output / 'common-state.npz', **snapshot)
    checks = {}
    for label in ('A', 'B'):
        m = models[label]; dest = output / label
        controller = Controller(m, cfg)
        controller.requested = snapshot['controller_requested'].copy(); controller.command = snapshot['controller_command'].copy()
        initial = restore_state(m, snapshot['integration_state'])
        before = integration_state(m, initial)
        np.testing.assert_array_equal(before, snapshot['integration_state'])
        mujoco.mj_forward(m, initial)
        np.testing.assert_array_equal(initial.qpos, snapshot['qpos'])
        np.testing.assert_array_equal(initial.qvel, snapshot['qvel'])
        np.savez_compressed(dest / 'initial-state.npz', **snapshot,
                            restored_integration_before_forward=before,
                            restored_integration_after_forward=integration_state(m, initial))
        frozen.write_json(dest / 'initial-contacts.json', frozen.contacts(m, initial))
        protocol = copy.deepcopy(p)
        if label == 'B': protocol['cube']['table']['center'][2] = .015
        protocol.update(diagnostic_only=True, official_grasp_success_evaluated=False, label=label,
                        frozen_commit=FREEZE, source_episode=str(source), source_trajectory_sha256=TRAJECTORY_SHA,
                        snapshot_source_row=boundary - 1, snapshot_time_s=float(data.time),
                        duration_s=14.1, hold_physics_samples=STEPS,
                        state_spec='mjSTATE_INTEGRATION', state_spec_value=int(STATE_SPEC),
                        constant_command_8d=c.command.tolist(), script_sha256=digest(__file__),
                        model_sha256=digest(dest / 'model.xml'))
        frozen.write_json(dest / 'protocol.json', protocol)
        checks[label] = dict(integration_before_forward_identical=True, qpos_qvel_after_forward_identical=True,
                             integration_after_forward_max_delta=float(abs(integration_state(m, initial) - snapshot['integration_state']).max()),
                             initial_state_sha256=digest(dest / 'initial-state.npz'))
    frozen.write_json(output / 'preflight.json', dict(diagnostic_only=True, ready=True, frozen_commit=FREEZE,
        implementation_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ASSET_ROOT, text=True).strip(),
        script_sha256=digest(__file__), actual_prefix_mj_steps=boundary, prefix_max_errors=maximum,
        integration_state_size=len(snapshot['integration_state']), initial_checks=checks,
        common_state_sha256=digest(output / 'common-state.npz'), model_difference=diff,
        duration_s_each=14.1, steps_each=STEPS))
    print('PREPARED: prefix 29128 real steps, full identical state; no A/B hold steps yet', flush=True)


def seal(output):
    """Explicit, zero-step review record; preserves the original preparation."""
    verify_frozen()
    preflight = json.loads((output / 'preflight.json').read_text())
    prepared_source = output / 'prepare-runner.py'
    if not prepared_source.exists(): prepared_source.write_bytes(Path(__file__).read_bytes())
    if digest(prepared_source) != preflight['script_sha256'] or digest(output / 'common-state.npz') != preflight['common_state_sha256']:
        raise ValueError('Original preparation changed')
    source = ASSET_ROOT / 'experiments/dynamics/runs/diag_baseline_001/fixed'
    original_protocol = json.loads((source / 'protocol.json').read_text())
    canonical, lowered = model_variants((source / 'model.xml').read_text(), source)
    artifacts = {'preflight.json': digest(output / 'preflight.json'),
                 'prepare-runner.py': digest(prepared_source), 'common-state.npz': digest(output / 'common-state.npz')}
    models = []
    with np.load(output / 'common-state.npz', allow_pickle=False) as state:
        for label, xml in [('A', canonical), ('B', lowered)]:
            dest = output / label; p = json.loads((dest / 'protocol.json').read_text())
            expected = copy.deepcopy(original_protocol)
            if label == 'B': expected['cube']['table']['center'][2] = .015
            for key, value in expected.items():
                if p[key] != value: raise ValueError('Prepared protocol changed: ' + label + '/' + key)
            if (dest / 'model.xml').read_text() != xml or p['hold_physics_samples'] != STEPS or p['duration_s'] != 14.1:
                raise ValueError('Unapproved model/duration: ' + label)
            with np.load(dest / 'initial-state.npz', allow_pickle=False) as initial:
                for key in state.files: np.testing.assert_array_equal(initial[key], state[key])
                np.testing.assert_array_equal(initial['restored_integration_before_forward'], state['integration_state'])
            models.append(mujoco.MjModel.from_xml_path(str(dest / 'model.xml')))
            for name in ('model.xml', 'protocol.json', 'initial-state.npz', 'initial-contacts.json'):
                artifacts[f'{label}/{name}'] = digest(dest / name)
    diff = model_check(*models)
    (output / 'run-runner.py').write_bytes(Path(__file__).read_bytes())
    frozen.write_json(output / 'reviewed-readiness.json', dict(diagnostic_only=True, artifacts=artifacts,
        run_script_sha256=digest(__file__), preparation_script_sha256=preflight['script_sha256'],
        review_change='Bind both model/protocol/state hashes and validate all groups before the first A step; no physical/state changes',
        compiled_model_difference=diff, original_preflight_preserved=True))
    print('SEALED: approved model/config/state for both groups; zero A/B steps', flush=True)


def validate_readiness(output):
    ready = json.loads((output / 'reviewed-readiness.json').read_text())
    if digest(__file__) != ready['run_script_sha256']:
        raise ValueError('Reviewed execution source changed')
    for name, expected in ready['artifacts'].items():
        if digest(output / name) != expected:
            raise ValueError('Prepared artifact changed: ' + name)
    return ready


def execute(output):
    validate_readiness(output)
    verify_frozen()
    preflight = json.loads((output / 'preflight.json').read_text())
    for label in ('A', 'B'):
        dest = output / label
        if (dest / 'trajectory.npz').exists() or (dest / 'contacts.jsonl').exists():
            raise FileExistsError('A/B run already started; never overwrite: ' + label)
    results = {}
    for label in ('A', 'B'):
        dest = output / label; p = json.loads((dest / 'protocol.json').read_text())
        if digest(dest / 'model.xml') != p['model_sha256'] or digest(dest / 'initial-state.npz') != preflight['initial_checks'][label]['initial_state_sha256']:
            raise ValueError('Prepared model or state changed')
        m = mujoco.MjModel.from_xml_path(str(dest / 'model.xml')); c = Controller(m, p['config'])
        with np.load(dest / 'initial-state.npz', allow_pickle=False) as z:
            c.requested = z['controller_requested'].copy(); c.command = z['controller_command'].copy()
            data = restore_state(m, z['integration_state'])
        mujoco.mj_forward(m, data)
        recorder = frozen.Recorder(dest, m, c); error = None
        observations = dict(time=[], **{key: [] for key in pose(m, data)})
        try:
            for _ in range(STEPS):
                recorder.step(data, 'diagnostic_fixed_pose_hold')
                observations['time'].append(float(data.time))
                for key, value in pose(m, data).items(): observations[key].append(value)
        except BaseException:
            error = traceback.format_exc()
            raise
        finally:
            health = recorder.finish(error)
            np.savez_compressed(dest / 'observations.npz', **{k: np.asarray(v) for k, v in observations.items()})
            results[label] = dict(diagnostic_only=True, official_grasp_success_evaluated=False,
                actual_mj_steps=recorder.actual_steps, elapsed_s=float(data.time - p['snapshot_time_s']),
                control_health=health['status'], error=error)
            frozen.write_json(dest / 'diagnostic-result.json', results[label])
            frozen.write_json(output / 'execution.json', dict(argv=sys.argv, python=sys.version,
                mujoco=mujoco.__version__, diagnostic_only=True, results=results))
        print(label, json.dumps(results[label]), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['prepare', 'seal', 'run'])
    parser.add_argument('--source-dir', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.mode == 'prepare':
        if args.source_dir is None: parser.error('prepare requires --source-dir')
        prepare(args.source_dir, args.output_dir)
    elif args.mode == 'seal':
        if (args.output_dir / 'reviewed-readiness.json').exists(): raise FileExistsError('Review seal already exists')
        seal(args.output_dir)
    else: execute(args.output_dir)


if __name__ == '__main__':
    main()
