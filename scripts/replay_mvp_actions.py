"""Issue29: 20Hz absolute ReplayPolicy -> frozen 100Hz/1kHz Controller.

check never integrates; run creates exactly one new Episode. No expert planner,
contact-triggered control, saved-state replay, parameter overrides or weights.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import traceback

import mujoco
import numpy as np
import pyarrow.parquet as pq
from PIL import Image

BASE = Path('/data_robot/projects')
PACKAGE = BASE / '.deliveries/nero-selfbuilt-mvp-77bd26a'
EPISODE = BASE / '.experiments/nero-mvp-lerobot-issue27-20261020'
ARCHIVE_SHA = '51e8267051648c2dec29441ed7e1e8dd9e408871fbfa166d10a9d34eae5b894d'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''): h.update(block)
    return h.hexdigest()


def read(path): return json.loads(Path(path).read_text())


def write(path, value):
    with Path(path).open('x') as f: json.dump(value, f, ensure_ascii=False, indent=2); f.write('\n')


def position_action(raw, limits):
    """Restore ONLY out-of-range float32 endpoint representations (<=half ULP)."""
    raw = np.asarray(raw)
    if raw.shape != (8,) or raw.dtype.kind not in 'fiu' or not np.isfinite(raw).all():
        raise ValueError('Expected finite real 8D absolute position action')
    out = raw.astype(float, copy=True); changes = []
    for j, (lo, hi) in enumerate(limits):
        if lo <= out[j] <= hi: continue
        bound = lo if out[j] < lo else hi
        b32 = np.float32(bound)
        half_ulp = .5 * max(abs(float(np.nextafter(b32, np.float32(np.inf))) - float(b32)),
                            abs(float(np.nextafter(b32, np.float32(-np.inf))) - float(b32)))
        if (raw.dtype == np.float32 and raw[j] == b32 and abs(out[j] - bound) <= half_ulp):
            changes.append(dict(dimension=j, original=float(out[j]), corrected=float(bound),
                                half_float32_ulp=float(half_ulp)))
            out[j] = bound
        else: raise ValueError(f'Position action outside frozen limit at dimension {j}: {out[j]}')
    return out, changes


class ReplayPolicy:
    """Open-loop index-only policy. Observation is accepted but NOT used."""
    def __init__(self, actions): self.actions = np.asarray(actions)

    def chunk(self, start, observation=None):
        if not 0 <= start < len(self.actions): raise IndexError(start)
        indices = start + np.arange(10)
        return self.actions[np.minimum(indices, len(self.actions) - 1)].copy(), indices >= len(self.actions)


def dispatch(k, prefix, actions):
    """Step k starts at k milliseconds; targets precede slew and feedback."""
    target = None
    if k < 50:
        if k % 10 == 0: target = prefix[k]
    elif (k - 50) % 50 == 0: target = actions[(k - 50) // 50]
    return target, k % 10 == 0


def close_phase_check(forces, start_step, source_end_step, maximum_duration_s, threshold, *, first_lift_step=None):
    """The expert's successful trigger time is NOT its maximum-close deadline.

    Frozen execute_grasp allows maximum_close_duration_s after close starts.
    Source phase names remain diagnostic labels when replay is resampled.
    """
    stop = min(len(forces), start_step + round(maximum_duration_s * 1000))
    # Frozen expert closes before lifting. Source phase end is not a deadline,
    # but replay cannot claim a close gate passed only after lift already began.
    if first_lift_step is not None: stop = min(stop, first_lift_step)
    hits = np.flatnonzero(np.all(np.asarray(forces)[start_step:stop] >= threshold, axis=1))
    step = start_step + int(hits[0]) + 1 if len(hits) else None
    return dict(status='PASS' if step is not None else 'FAIL',
                new_bilateral_contact_step=step,
                new_bilateral_contact_time_s=step*.001 if step is not None else None,
                original_close_timeout_step=start_step + round(maximum_duration_s * 1000),
                first_lift_step=first_lift_step,
                source_close_end_step=source_end_step,
                within_source_close_window=bool(step is not None and step <= source_end_step))


def load_inputs(package, episode):
    spec = importlib.util.spec_from_file_location('portable_mvp', package / 'run.py')
    portable = importlib.util.module_from_spec(spec); spec.loader.exec_module(portable)
    case, model, control, data, _ = portable.load()  # frozen reset, once; no integration
    for module in (portable.controller_module, portable.frozen):
        assert Path(module.__file__).resolve().is_relative_to(package)
    assert model.opt.timestep == .001 and case['config']['command_period_s'] == .01
    protocol = read(episode / 'source/protocol.json'); plan = read(episode / 'plan.json')
    assert sha(episode / 'source/protocol.json') == plan['protocol_sha256']
    source = Path(protocol['source_run']); manifest = read(episode / 'dataset-manifest.json')
    files = {str(p.resolve()): sha(p) for p in package.rglob('*') if p.is_file()}
    archive = package.with_suffix('.tar.gz'); assert sha(archive) == ARCHIVE_SHA
    files[str(archive)] = ARCHIVE_SHA
    for p in (episode / 'source/protocol.json', episode / 'plan.json', episode / 'dataset-manifest.json',
              Path(protocol['source_manifest'])): files[str(p)] = sha(p)
    assert files[str(Path(protocol['source_manifest']))] == protocol['source_manifest_sha256']
    for name, info in plan['source_files'].items():
        p = source.parent / name
        assert sha(p) == info['sha256'], str(p)
        files[str(p)] = info['sha256']
    dataset = Path(manifest['root'])
    for name, info in manifest['data_files'].items():
        p = dataset / name; assert sha(p) == info['sha256'], str(p)
        files[str(p)] = info['sha256']
    saved = dict(np.load(source / 'trajectory.npz', allow_pickle=False))
    assert sha(source / 'trajectory.npz') == protocol['source_trajectory_sha256']
    original = read(source / 'protocol.json')
    assert all(original[k] == case[k] for k in ('config', 'grasp_protocol', 'cube', 'joint_plan'))
    table = pq.read_table(dataset / 'data/chunk-000/episode_000000.parquet')
    raw = np.asarray(table['actions'].to_pylist(), dtype=np.float32)
    state = np.asarray(table['observation.state'].to_pylist(), dtype=np.float32)
    indices = 49 + 50 * np.arange(682)
    assert raw.shape == (682, 8) and state.shape == (682, 9)
    assert len(saved['time']) == 34150
    np.testing.assert_array_equal(raw, saved['requested'][indices + 1].astype(np.float32))
    np.testing.assert_array_equal(state, saved['qpos'][indices][:, control.qadr].astype(np.float32))
    np.testing.assert_array_equal(table['frame_index'].to_numpy(), np.arange(682))
    np.testing.assert_array_equal(table['episode_index'].to_numpy(), np.zeros(682))
    np.testing.assert_allclose(table['timestamp'].to_numpy(), np.arange(682) / 20, atol=2e-6, rtol=0)
    np.testing.assert_allclose(saved['time'], (np.arange(34150) + 1) * .001, atol=2e-10, rtol=0)
    limits = np.vstack([control.limits, [0., .1]])
    corrected = []; corrections = []
    for i, value in enumerate(raw):
        action, changes = position_action(value, limits); control.validate(action)
        corrected.append(action)
        corrections.extend(dict(frame_index=i, **c) for c in changes)
    # Startup requested commands are recorded AFTER applying each source step.
    # They therefore drive that step, not the next saved state.
    prefix = saved['requested'][:50].copy()
    for k in range(50):
        control.validate(prefix[k]); np.testing.assert_array_equal(prefix[k], prefix[(k // 10) * 10])
    return dict(portable=portable, case=case, model=model, control=control, data=data,
                protocol=protocol, source=source, files=files, saved=saved,
                actions=np.asarray(corrected), raw=raw, prefix=prefix, corrections=corrections,
                original_result=read(source / 'result.json'))


def make_renderer(model, camera):
    cam = mujoco.MjvCamera(); cam.lookat[:] = camera['lookat']
    cam.distance = camera['distance']; cam.azimuth = camera['azimuth']; cam.elevation = camera['elevation']
    option = mujoco.MjvOption(); option.geomgroup[0] = 0; option.geomgroup[1:3] = 1
    return mujoco.Renderer(model, height=camera['height'], width=camera['width']), cam, option


def intact(files):
    bad = [p for p, h in files.items() if sha(p) != h]
    if bad: raise ValueError('Frozen input changed: ' + repr(bad))
    return dict(files=len(files), changed=bad)


def freeze(out, inputs, mode):
    out.mkdir(parents=True, exist_ok=False)
    own = Path(__file__).resolve(); (out / 'executed-source').mkdir()
    scripts = [own, own.with_name('test_replay_mvp_actions.py')]
    for p in scripts: shutil.copyfile(p, out / 'executed-source' / p.name)
    plan = dict(issue=29, mode=mode, argv=sys.argv, cwd=os.getcwd(), python=sys.executable,
                python_version=platform.python_version(), mujoco=mujoco.__version__, numpy=np.__version__,
                runner_sha256=sha(own), input_sha256=inputs['files'], camera=inputs['protocol']['camera'],
                action_semantics='8D absolute joint radians + width metres; Parquet requested; no ctrl replay',
                rounding_policy='Only out-of-range exact float32(bound) within half endpoint ULP is restored',
                endpoint_corrections=inputs['corrections'],
                schedule=dict(prefix_physics_steps=50, prefix='source requested[0:50]; update each10 steps',
                              action_count=682, action_hold_steps=50, physics_steps=34150,
                              target_steps='50+50*i, i=0..681', slew_steps='k%10==0', feedback='every step',
                              horizon=10, chunk_starts=list(range(0, 682, 10)), final_chunk_valid=2,
                              padding_actions_consumed=0, stop='after last valid action holds50 steps; no extra hold'),
                evaluation='Original R1 time windows/phases used only for recording and acceptance, never decisions',
                limits='Frozen controller validates every target; no broad clipping or double interpolation',
                shortcut_policy='Only frozen legal reset writes initial qpos; no later qpos writes/external force',
                scope='one real Episode; no automatic retry for grasp failure')
    write(out / 'frozen-plan.json', plan)
    (out / 'frozen-plan.sha256').write_text(sha(out / 'frozen-plan.json') + '\n')
    return plan


def check(args):
    def forbidden(*a, **kw): raise RuntimeError('Zero-integration check forbids physics')
    for name in ('mj_step', 'mj_step1', 'mj_step2'): setattr(mujoco, name, forbidden)
    inputs = load_inputs(args.package, args.episode); freeze(args.output, inputs, 'check')
    c = inputs['control']; policy = ReplayPolicy(inputs['actions']); consumed = []
    for start in range(0, 682, 10):
        chunk, mask = policy.chunk(start); consumed.extend(chunk[~mask])
    np.testing.assert_array_equal(consumed, inputs['actions'])
    targets = 0; advances = 0
    for k in range(34150):
        target, advance = dispatch(k, inputs['prefix'], inputs['actions'])
        if target is not None: c.set_target(target); targets += 1
        if advance:
            before = c.command.copy(); c.advance_command(); advances += 1
            assert np.all(abs(c.command - before) <= c.slew * .01 + 1e-14)
    c.apply(inputs['data']); assert inputs['data'].time == 0
    renderer, cam, opt = make_renderer(inputs['model'], inputs['protocol']['camera'])
    with renderer:
        renderer.update_scene(inputs['data'], camera=cam, scene_option=opt)
        rgb = renderer.render(); assert rgb.shape == (256, 256, 3) and rgb.dtype == np.uint8
    result = dict(status='PASS_ZERO_INTEGRATION', actual_mj_steps=0, new_physics_episodes=0,
                  valid_actions=682, high_level_target_calls=682, startup_target_calls=targets - 682,
                  advance_calls=advances, planned_feedback_steps=34150, tail_padding_consumed=0,
                  first_source_observation_s=.05, last_source_observation_s=34.10,
                  endpoint_corrections=len(inputs['corrections']), renderer_shape=list(rgb.shape),
                  input_integrity=intact(inputs['files']), runner_sha256=sha(__file__))
    write(args.output / 'check.json', result); print(json.dumps(result), flush=True)


class ActionRuntime:
    """One reset on construction, then observe current state / step(action8)."""
    def __init__(self, inputs, out):
        self.inputs = inputs; self.model = inputs['model']; self.data = inputs['data']
        self.c = inputs['control']; self.k = 0; self.target_calls = 0; self.advances = 0
        self.apply_calls = 0; self.consumed = []; self.force = []; self.table_depth = []
        self.recorder = inputs['portable'].frozen.Recorder(out, self.model, self.c)
        self.renderer, self.cam, self.opt = make_renderer(self.model, inputs['protocol']['camera'])

    def observe(self):
        self.renderer.update_scene(self.data, camera=self.cam, scene_option=self.opt)
        return dict(state=self.data.qpos[self.c.qadr].copy(), image=self.renderer.render().copy(),
                    prompt=self.inputs['protocol']['task'], time_s=float(self.data.time), physics_step=self.k)

    def physics_step(self, advance):
        if advance:
            before = self.c.command.copy(); self.c.advance_command(); self.advances += 1
            assert np.all(abs(self.c.command - before) <= self.c.slew * .01 + 1e-14)
        # Recorder.step applies original1kHz feedback, integrates1ms, forwards then records.
        events = self.recorder.step(self.data, str(self.inputs['saved']['phase'][self.k]))
        self.apply_calls += 1; self.k += 1
        assert abs(self.data.time - self.k * .001) < 2e-10
        f = np.zeros(2); depth = 0.
        for e in events:
            if 'cube' in e['bodies']:
                for j in range(2):
                    if f'gripper_link{j+1}' in e['bodies']: f[j] += max(0., e['force_contact_frame'][0])
                if 'table' in e['geoms']: depth = max(depth, -e['distance_m'])
        self.force.append(f); self.table_depth.append(depth)

    def startup(self):
        for k in range(50):
            target, advance = dispatch(k, self.inputs['prefix'], self.inputs['actions'])
            if target is not None: self.c.set_target(target)
            self.physics_step(advance)

    def step(self, action8):
        action, changes = position_action(action8, np.vstack([self.c.limits, [0., .1]]))
        assert not changes  # Dataset restoration is audited before integration.
        self.c.set_target(action); self.target_calls += 1
        self.consumed.append(dict(index=self.target_calls-1, start_step=self.k,
                                  end_step=self.k+50, target=action.tolist()))
        for _ in range(50): self.physics_step(self.k % 10 == 0)


def phase_checks(runtime):
    """Original phase settled errors/thresholds, evaluated on new states."""
    x = runtime.inputs; rows = runtime.recorder.rows; q = np.asarray(rows['qpos'])[:, runtime.c.qadr]
    forces = np.asarray(runtime.force); a = x['case']['config']['acceptance']; result = []
    for original in x['original_result']['phases']:
        start = round(original['start_time_s'] * 1000); end = round(original['end_time_s'] * 1000)
        if end > len(q): continue
        p = {k: original[k] for k in ('phase','start_time_s','end_time_s','loaded_fingers')}
        if 'target_8d' in original:
            target = np.asarray(original['target_8d']); physical = np.r_[target[:7], target[7]/2, -target[7]/2]
            err = np.max(abs(q[max(start, end-250):end] - physical), axis=0)
            ok = err[:7].max() <= a['settled_arm_error_rad'] and (p['loaded_fingers'] or err[7:].max() <= a['settled_finger_error_m'])
            p.update(target_8d=target.tolist(), settled_max_error_9d=err.tolist())
        else:
            protocol = x['case']['grasp_protocol']
            first_lift = min(round(p['start_time_s']*1000) for p in x['original_result']['phases'] if p['phase']=='lift')
            close = close_phase_check(forces, start, end, protocol['maximum_close_duration_s'],
                                      protocol['contact_trigger_force_N_each'], first_lift_step=first_lift)
            p.update(close); ok = close['status'] == 'PASS'
        p['status'] = 'PASS' if ok else 'FAIL'; result.append(p)
    return result


def run(args):
    preflight = read(args.checked / 'check.json')
    assert preflight['status'] == 'PASS_ZERO_INTEGRATION' and preflight['runner_sha256'] == sha(__file__)
    inputs = load_inputs(args.package, args.episode)
    assert inputs['files'] == read(args.checked / 'frozen-plan.json')['input_sha256']
    freeze(args.output, inputs, 'run')
    runtime = ActionRuntime(inputs, args.output); policy = ReplayPolicy(inputs['actions'])
    (args.output / 'rgb').mkdir(); obs = []; error = None; start_wall = time.monotonic()
    try:
        runtime.startup()
        for start in range(0, 682, 10):
            chunk, mask = policy.chunk(start)
            for action in chunk[~mask]:
                current = runtime.observe(); i = len(obs)
                Image.fromarray(current.pop('image')).save(args.output / 'rgb' / f'{i:06d}.png')
                obs.append(current); runtime.step(action)
                if i % 100 == 0: print(f'action={i} physics_steps={runtime.k}', flush=True)
        terminal = runtime.observe()
        Image.fromarray(terminal.pop('image')).save(args.output / 'rgb' / f'{len(obs):06d}.png'); obs.append(terminal)
    except (Exception, KeyboardInterrupt): error = traceback.format_exc()
    finally:
        runtime.renderer.close(); runtime.recorder.phases = phase_checks(runtime)
        control = runtime.recorder.finish(error)
    np.savez_compressed(args.output / 'observations.npz', state=np.array([o['state'] for o in obs]),
                        time_s=np.array([o['time_s'] for o in obs]), physics_step=np.array([o['physics_step'] for o in obs]))
    write(args.output / 'consumed-actions.json', runtime.consumed)
    write(args.output / 'observation-spec.json', dict(prompt=inputs['protocol']['task'], camera=inputs['protocol']['camera'],
          images='rgb/%06d.png from CURRENT data after integration; no source qpos assignments',
          frames=len(obs), terminal_observation_has_no_action=True))
    with (args.output / 'contacts.jsonl').open() as f:
        cube_jid = mujoco.mj_name2id(runtime.model, mujoco.mjtObj.mjOBJ_JOINT, 'cube_free')
        grasp = inputs['portable'].frozen.evaluate_grasp(runtime.recorder.rows, (json.loads(l) for l in f),
                 int(runtime.model.jnt_qposadr[cube_jid]), inputs['case']['cube']['center'],
                 inputs['case']['grasp_protocol']['criteria'], .001)
    grasp.update(control_health_status=control['status'], actual_mj_steps=runtime.k, error=error,
                 maximum_cube_table_penetration_m=max(runtime.table_depth, default=0.))
    if error or control['status'] != 'PASS': grasp['status'] = 'FAIL'
    write(args.output / 'grasp-result.json', grasp)
    observed_steps = [o['physics_step'] for o in obs]
    checks = dict(complete=runtime.k == 34150 and error is None,
                  order=np.array_equal([a['target'] for a in runtime.consumed], inputs['actions']),
                  high_level_calls=runtime.target_calls == 682,
                  timing=[a['start_step'] for a in runtime.consumed] == list(range(50, 34150, 50)),
                  slew_calls=runtime.advances == 3415, feedback_calls=runtime.apply_calls == 34150,
                  observations=observed_steps == list(range(50, 34151, 50)),
                  no_forces=not np.any(runtime.recorder.rows['qfrc_applied']) and not np.any(runtime.recorder.rows['xfrc_applied']))
    result = dict(ACTION_RUNTIME='PASS' if all(checks.values()) else 'FAIL', REPLAY_GRASP=grasp['status'],
                  checks=checks, error=error, actual_mj_steps=runtime.k, high_level_calls=runtime.target_calls,
                  slew_calls=runtime.advances, feedback_calls=runtime.apply_calls, observations=len(obs),
                  padding_consumed=0, wall_s=time.monotonic()-start_wall, input_integrity=intact(inputs['files']))
    command = ['ffmpeg','-nostdin','-y','-loglevel','error','-framerate','20','-i',str(args.output/'rgb/%06d.png'),
               '-c:v','libx264','-pix_fmt','yuv420p',str(args.output/'replay.mp4')]
    video = subprocess.run(command, capture_output=True, text=True)
    write(args.output/'video-command.json', dict(argv=command, exit_code=video.returncode, stderr=video.stderr))
    result['video_exit_code'] = video.returncode
    write(args.output/'runtime-result.json', result)
    write(args.output/'raw-manifest.json', {str(p.relative_to(args.output)): dict(sha256=sha(p), bytes=p.stat().st_size)
          for p in args.output.rglob('*') if p.is_file()})
    print(json.dumps(dict(runtime=result, grasp=grasp)), flush=True)
    return 0 if result['ACTION_RUNTIME'] == 'PASS' and video.returncode == 0 else 1


def evaluate_only(args):
    """Correct reporting only; immutable raw Run, no new physics or state replay."""
    def forbidden(*a, **kw): raise RuntimeError('Offline evaluation cannot integrate')
    for name in ('mj_step', 'mj_step1', 'mj_step2'): setattr(mujoco, name, forbidden)
    inputs = load_inputs(args.package, args.episode)
    old_manifest = read(args.run_directory / 'raw-manifest.json')
    for name, info in old_manifest.items(): assert sha(args.run_directory / name) == info['sha256']
    assert inputs['files'] == read(args.run_directory / 'frozen-plan.json')['input_sha256']
    freeze(args.output, inputs, 'evaluate-only')
    from types import SimpleNamespace
    rows = dict(np.load(args.run_directory / 'trajectory.npz', allow_pickle=False))
    forces = np.zeros((len(rows['time']), 2))
    with (args.run_directory / 'contacts.jsonl').open() as f:
        for i, line in enumerate(f):
            event = json.loads(line)
            for e in event['events']:
                if 'cube' in e['bodies']:
                    for j in range(2):
                        if f'gripper_link{j+1}' in e['bodies']: forces[i,j] += max(0.,e['force_contact_frame'][0])
    old = read(args.run_directory / 'result.json')
    phases = phase_checks(SimpleNamespace(inputs=inputs, recorder=SimpleNamespace(rows=rows),
                                         c=inputs['control'], force=forces))
    a = inputs['case']['config']['acceptance']
    health_checks = dict(coupling=old['max_coupling_error_m'] <= a['maximum_coupling_error_m'],
                        arm_velocity=old['max_arm_velocity_rad_s'] <= a['maximum_arm_velocity_rad_s'],
                        finger_velocity=old['max_finger_velocity_m_s'] <= a['maximum_finger_velocity_m_s'],
                        phase_tracking=all(p['status'] == 'PASS' for p in phases),
                        no_error=old['error'] is None)
    control = dict(old, status='PASS' if all(health_checks.values()) else 'FAIL', phases=phases,
                   health_checks=health_checks)
    grasp = read(args.run_directory / 'grasp-result.json')
    grasp['control_health_status'] = control['status']
    grasp['status'] = 'PASS' if control['status'] == 'PASS' and all(grasp['checks'].values()) else 'FAIL'
    runtime = read(args.run_directory / 'runtime-result.json'); runtime['REPLAY_GRASP'] = grasp['status']
    observations = dict(np.load(args.run_directory / 'observations.npz', allow_pickle=False))
    assert np.array_equal(observations['state'], rows['qpos'][observations['physics_step']-1][:, inputs['control'].qadr])
    timing = dict(prefix_50_steps_byte_equal=all(np.array_equal(rows[k][:50], inputs['saved'][k][:50])
                  for k in ('qpos','qvel','command','ctrl')),
                  all_observations_equal_new_integrated_state=True,
                  diagnostic_threshold_only=1e-6, differences={})
    for key in ('requested','command','qpos','qvel'):
        difference = abs(rows[key] - inputs['saved'][key]); hits = np.argwhere(difference > 1e-6)
        timing['differences'][key] = dict(maximum_per_dimension=difference.max(axis=0).tolist(),
                first_over_diagnostic_threshold_step=int(hits[0,0])+1 if len(hits) else None,
                first_over_diagnostic_threshold_dimension=int(hits[0,1]) if len(hits) else None)
    hits = np.flatnonzero(np.any(rows['requested'].astype(np.float32) != inputs['saved']['requested'].astype(np.float32),axis=1))
    timing['first_requested_difference_after_float32_comparison_step'] = int(hits[0])+1 if len(hits) else None
    close_phase = next(p for p in phases if p['phase']=='close')
    trigger_time = close_phase['new_bilateral_contact_time_s']
    timing['close_trigger_delay_s'] = trigger_time - close_phase['end_time_s'] if trigger_time is not None else None
    write(args.output / 'timing-analysis.json', timing)
    write(args.output / 'control-health.json', control); write(args.output / 'grasp-result.json', grasp)
    write(args.output / 'runtime-result.json', dict(runtime, evaluation_mode='zero-integration', actual_mj_steps_in_evaluation=0,
          physical_source=str(args.run_directory), preserved_original_report_status=old['status'],
          correction='Use unchanged original maximum_close_duration_s, not source successful trigger time as a deadline',
          input_integrity_after_evaluation=intact(inputs['files'])))
    for name, info in old_manifest.items(): assert sha(args.run_directory / name) == info['sha256']
    write(args.output / 'raw-manifest.json', {str(p.relative_to(args.output)):dict(sha256=sha(p),bytes=p.stat().st_size)
          for p in args.output.rglob('*') if p.is_file()})
    print(json.dumps(dict(ACTION_RUNTIME=runtime['ACTION_RUNTIME'], REPLAY_GRASP=grasp['status'],
                         health_checks=health_checks, actual_mj_steps_in_evaluation=0)), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('mode', choices=['check','run','evaluate-only'])
    p.add_argument('--package', type=Path, default=PACKAGE); p.add_argument('--episode', type=Path, default=EPISODE)
    p.add_argument('--output', type=Path, required=True); p.add_argument('--checked', type=Path)
    p.add_argument('--run-directory', type=Path)
    args = p.parse_args(); args.package = args.package.resolve(); args.episode = args.episode.resolve(); args.output = args.output.resolve()
    if args.output.is_relative_to(args.package) or args.output.is_relative_to(args.episode):
        p.error('Output must be outside read-only package and source Episode')
    if args.mode == 'check': check(args)
    elif args.mode == 'run':
        if args.checked is None: p.error('run requires --checked zero-integration result directory')
        sys.exit(run(args))
    else:
        if args.run_directory is None: p.error('evaluate-only requires --run-directory')
        if args.output.is_relative_to(args.run_directory.resolve()): p.error('Do not overwrite raw Run')
        evaluate_only(args)
