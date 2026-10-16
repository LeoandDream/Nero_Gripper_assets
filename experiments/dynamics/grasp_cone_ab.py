"""Issue #8 diagnostic-only: saved pyramidal B versus one frozen-state elliptic run.

prepare and analyze never integrate; run refuses reuse and performs <=14100 steps.
The frozen controller/Recorder.step, physical assets and acceptance are reused.
"""
import argparse
import copy
import difflib
import json
from pathlib import Path
import platform
import sys
import traceback
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from controller import ASSET_ROOT, Controller
import run as frozen
from grasp_support_ab import digest, integration_state, restore_state, pose, verify_frozen

STEPS = 14100
OLD_HASHES = {
    'model.xml': 'e42b051da4e13dca061b691cee1c8f6231adadb9068fd815697c210660f02ee0',
    'trajectory.npz': '7eb0976355caba3bc3d940e7026f36f93e75bb60536d6c74f1f42cf3d792942d',
    'contacts.jsonl': '243c3ed224e5766f0ec17254c5144e51805dd4ff256c1e136d32b7c92529ee1c',
    'initial-state.npz': '2cf072f2292c4af92c0feca60f17013e17fba7832fa5b58024fe9b434af629fe',
    'protocol.json': '0a169167681ff5c84595261284eb2ed9f2fb3673104507e3034810333bfd8e3f',
    'observations.npz': '07ef96f564c1bf28d716bbe646725d140d408ac15961e2e4e30a8eef06d41424',
    'initial-contacts.json': '11b124ed8ad4a9de98ea0c0ef173d1dfc21623645251d4342d59ebdbcf6cf301',
}


def npz(path):
    with np.load(path, allow_pickle=False) as z: return {k: z[k] for k in z.files}


def verify_files(root, hashes):
    for name, value in hashes.items():
        if digest(Path(root) / name) != value: raise ValueError('Input hash mismatch: ' + name)


def elliptic_xml(text):
    root = ET.fromstring(text)
    options = root.findall('option')
    if len(options) != 1 or options[0].get('cone', 'pyramidal') != 'pyramidal':
        raise ValueError('Expected one original pyramidal option')
    # Preserve all source bytes except the single option attribute.
    start = text.index('<option '); stop = text.index('/>', start)
    option_text = text[start:stop]
    if 'cone=' in option_text:
        raise ValueError('This frozen input must use the implicit pyramidal default')
    separator = '' if text[stop - 1].isspace() else ' '
    result = text[:stop] + separator + 'cone="elliptic" ' + text[stop:]
    modified = ET.fromstring(result)
    if modified.find('option').attrib.pop('cone') != 'elliptic' or ET.tostring(root) != ET.tostring(modified):
        raise ValueError('More than the approved XML option changed')
    return result


def check_models(a, b):
    checked = []
    for name in dir(a):
        if name.startswith('_'): continue
        x, y = getattr(a, name), getattr(b, name)
        if isinstance(x, (np.ndarray, int, float, str)):
            if not np.array_equal(x, y): raise ValueError('Compiled model differs: ' + name)
            checked.append(name)
    changed = []
    for name in dir(a.opt):
        x, y = getattr(a.opt, name), getattr(b.opt, name)
        if name.startswith('_') or callable(x): continue
        if not np.array_equal(x, y): changed.append(name)
    if changed != ['cone'] or int(a.opt.cone) != 0 or int(b.opt.cone) != 1:
        raise ValueError('Expected only pyramidal->elliptic option delta: ' + repr(changed))
    return dict(option_differences=changed, identical_public_model_fields=checked,
                all_other_option_fields_equal=True)


def utilization(force, friction, cone):
    if cone not in ('pyramidal', 'elliptic'): raise ValueError('Unknown cone')
    f = np.asarray(force); mu = np.asarray(friction)
    if f[0] <= 1e-12: return None
    scaled = f[1:3] / mu[:2]
    return float((np.abs(scaled).sum() if cone == 'pyramidal' else np.linalg.norm(scaled)) / f[0])


def claim_run(out):
    if any((out / n).exists() for n in ('trajectory.npz', 'contacts.jsonl', 'execution.json')):
        raise FileExistsError('Run outputs already exist; never overwrite')
    with (out / 'STARTED').open('x') as stream: stream.write('One authorized elliptic attempt; never resume or rerun here.\n')


def cache(data):
    return {k: np.array(getattr(data, k), copy=True) for k in
            ('qacc', 'qacc_warmstart', 'qfrc_constraint', 'qfrc_bias', 'efc_force',
             'efc_state', 'efc_R', 'efc_D', 'efc_b', 'efc_aref', 'efc_vel')}


def prepare(source, out):
    verify_frozen()
    if platform.python_version() != '3.11.9' or mujoco.__version__ != '3.2.3':
        raise ValueError('Unexpected frozen Python/MuJoCo version')
    source = source.resolve()
    verify_files(source, OLD_HASHES)
    extra = {'common-state.npz': 'aee269e559c74a1cd3005acd627328b2679c6286ee96e696297b1430be72ccf8',
             'run-runner.py': 'f81a1c9c69a35f5ed1012f0df57186ae1cede8e1af52432372b8a9c924869ce1'}
    verify_files(source.parent, extra)
    original = (source / 'model.xml').read_text()
    # XML still references the original worktree: verify those actual asset bytes.
    assets = {}
    for node in ET.fromstring(original).iter():
        if node.get('file'):
            path = Path(node.get('file')).resolve()
            rel = path.relative_to(source.parents[4])
            if digest(path) != digest(ASSET_ROOT / rel): raise ValueError('Referenced asset differs: ' + str(path))
            assets[str(path)] = digest(path)
    out.mkdir(parents=True, exist_ok=False)
    (out / 'model.xml').write_text(elliptic_xml(original))
    (out / 'model.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True), (out / 'model.xml').read_text().splitlines(True), fromfile='old-pyramidal/model.xml', tofile='elliptic/model.xml')))
    ma = mujoco.MjModel.from_xml_path(str(source / 'model.xml'))
    mb = mujoco.MjModel.from_xml_path(str(out / 'model.xml'))
    comparison = check_models(ma, mb)
    state = npz(source / 'initial-state.npz'); common = npz(source.parent / 'common-state.npz')
    for k in common: np.testing.assert_array_equal(common[k], state[k])
    for key in ('integration_state', 'restored_integration_before_forward'):
        np.testing.assert_array_equal(state[key], common['integration_state'])
    config = json.loads((source / 'protocol.json').read_text())
    np.testing.assert_array_equal(state['controller_requested'], state['controller_command'])
    np.testing.assert_array_equal(state['controller_command'], config['constant_command_8d'])
    old = npz(source / 'trajectory.npz')
    for k in ('requested', 'command'):
        np.testing.assert_array_equal(old[k], np.tile(state['controller_' + k], (STEPS, 1)))
    after = {}; data = {}
    for label, model in [('pyramidal', ma), ('elliptic', mb)]:
        d = restore_state(model, state['integration_state']); data[label] = d
        np.testing.assert_array_equal(d.qpos, state['qpos']); np.testing.assert_array_equal(d.qvel, state['qvel'])
        np.testing.assert_array_equal(d.qacc_warmstart, state['qacc_warmstart'])
        if d.time != state['time'][0]: raise ValueError('Snapshot time mismatch')
        mujoco.mj_forward(model, d)
        after[label] = cache(d)
        for k, value in pose(model, d).items(): np.testing.assert_array_equal(value, state[k])
        np.savez_compressed(out / (label + '-after-forward.npz'), **after[label], integration_state=integration_state(model, d))
        frozen.write_json(out / (label + '-initial-contacts.json'), frozen.contacts(model, d))
    np.testing.assert_array_equal(integration_state(ma, data['pyramidal']), state['restored_integration_after_forward'])
    original_contacts = json.loads((source / 'initial-contacts.json').read_text())
    if frozen.contacts(ma, data['pyramidal']) != original_contacts: raise ValueError('Old B initial contact solve not reproduced')
    new_state = dict(state); new_state['restored_integration_after_forward'] = integration_state(mb, data['elliptic'])
    np.savez_compressed(out / 'initial-state.npz', **new_state)
    differences = {}
    for k in after['pyramidal']:
        a, b = after['pyramidal'][k], after['elliptic'][k]
        differences[k] = dict(pyramidal_shape=list(a.shape), elliptic_shape=list(b.shape),
            equal=bool(np.array_equal(a, b)), max_abs_difference=None if a.shape != b.shape else float(np.max(np.abs(a - b), initial=0)))
    protocol = copy.deepcopy(config)
    protocol.update(label='elliptic', diagnostic_only=True, cone='elliptic', model_sha256=digest(out / 'model.xml'),
                    script_sha256=digest(__file__), comparator_dir=str(source))
    frozen.write_json(out / 'protocol.json', protocol)
    frozen.write_json(out / 'prediction.json', dict(diagnostic_only=True, primary='elapsed 68ms relative world-Z cube/gripper drop <1.000mm',
        elapsed_s=.068, strict_drop_threshold_m=.001, auxiliary_times_s=[.068,.751,1.317,1.328],
        relative_drop_definition='initial(cube_COM.z-gripper_base_origin.z) minus current gap',
        bilateral_threshold_N_each=config['grasp_protocol']['criteria']['minimum_finger_normal_force_N'],
        table_boundary='first geometric cube/table contact; upward table force also separately reported',
        utilization='per contact: pyramidal L1 or elliptic L2 of (Ft1/mu1,Ft2/mu2), divided by positive Fn',
        maximum_steps=STEPS, control='same frozen constant requested/command; ctrl feedback naturally differs after states diverge',
        no_grasp_acceptance_or_success_rate_evaluated=True))
    artifacts = {n: digest(out / n) for n in ('model.xml','initial-state.npz','protocol.json','prediction.json','model.diff')}
    frozen.write_json(out / 'prepared.json', dict(diagnostic_only=True, source=str(source), source_hashes=OLD_HASHES,
        parent_hashes=extra, referenced_asset_hashes=assets, artifacts=artifacts, runner_sha256=digest(__file__),
        model_comparison=comparison, pre_forward_full_state_equal=True, post_forward_cache_differences=differences,
        actual_mj_steps=0, python=sys.version, mujoco=mujoco.__version__, argv=sys.argv))
    print('PREPARED: exact snapshot and cone-only delta verified; zero new integration', flush=True)


def execute(out):
    prepared = json.loads((out / 'prepared.json').read_text())
    verify_frozen(); verify_files(out, prepared['artifacts'])
    verify_files(Path(prepared['source']), prepared['source_hashes'])
    verify_files(Path(prepared['source']).parent, prepared['parent_hashes'])
    for name, value in prepared['referenced_asset_hashes'].items():
        if digest(name) != value: raise ValueError('Referenced asset changed')
    if digest(__file__) != prepared['runner_sha256']: raise ValueError('Runner changed after preparation')
    model = mujoco.MjModel.from_xml_path(str(out / 'model.xml'))
    original = mujoco.MjModel.from_xml_path(str(Path(prepared['source']) / 'model.xml'))
    check_models(original, model)
    protocol = json.loads((out / 'protocol.json').read_text()); z = npz(out / 'initial-state.npz')
    controller = Controller(model, protocol['config'])
    controller.requested = z['controller_requested'].copy(); controller.command = z['controller_command'].copy()
    data = restore_state(model, z['integration_state'])
    mujoco.mj_forward(model, data)
    np.testing.assert_array_equal(integration_state(model, data), z['restored_integration_after_forward'])
    claim_run(out)  # atomic refusal of a second attempt, before the first step
    recorder = frozen.Recorder(out, model, controller)
    observations = dict(time=[], **{key: [] for key in pose(model, data)})
    warnings = []; error = None
    parameters = (out / 'contact-parameters.jsonl').open('x')
    try:
        for _ in range(STEPS):
            recorder.step(data, 'diagnostic_fixed_pose_hold')
            warnings.append(data.warning.number.copy())
            observations['time'].append(float(data.time))
            for key, value in pose(model, data).items(): observations[key].append(value)
            parameters.write(json.dumps(dict(time=float(data.time), contacts=[dict(dim=int(c.dim), friction5=c.friction.tolist()) for c in data.contact])) + '\n')
            if recorder.actual_steps % 2000 == 0: print('elliptic actual steps', recorder.actual_steps, flush=True)
    except BaseException:
        error = traceback.format_exc(); raise
    finally:
        parameters.close(); health = recorder.finish(error)
        np.savez_compressed(out / 'observations.npz', **{k: np.asarray(v) for k, v in observations.items()})
        np.savez_compressed(out / 'warnings.npz', time=np.asarray(observations['time']), numbers=np.asarray(warnings))
        frozen.write_json(out / 'execution.json', dict(diagnostic_only=True, official_grasp_success_evaluated=False,
            actual_mj_steps=recorder.actual_steps, actual_elapsed_s=float(data.time-z['time'][0]), error=error,
            control_health=health['status'], warning_numbers=data.warning.number.tolist(), runner_sha256=digest(__file__),
            argv=sys.argv, python=sys.version, mujoco=mujoco.__version__))
    print(json.dumps(json.loads((out / 'execution.json').read_text())), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['prepare','run'])
    parser.add_argument('--source-dir', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.mode == 'prepare':
        if args.source_dir is None: parser.error('prepare requires --source-dir')
        prepare(args.source_dir, args.output_dir)
    else: execute(args.output_dir)
