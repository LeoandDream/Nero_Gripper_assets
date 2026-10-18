"""Issue #12: derive only impratio=10; prepare at zero steps, run once <=14100.

Reuse the frozen Controller / Recorder.step and the original complete snapshot.
No formal assets are changed. A is verified and never integrated.
"""
import argparse
import copy
import difflib
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from controller import ASSET_ROOT, Controller
import run as frozen
from grasp_support_ab import digest, integration_state, restore_state, pose, verify_frozen
from grasp_cone_ab import npz, verify_files, cache

STEPS = 14100
FREEZE = 'c0e23102d7d2231414b712966b9c246615d14f0a'
STATE_SHA = '38fed316c9733655dcf6f6db655204f69e21d0d4f84db3916808eb1a83cb5d10'
A_HASHES = {
    'model.xml': '79f5f9116e812687e9c4592f9faf42e9660276f77329297e690be4a0db28d376',
    'trajectory.npz': 'fea51467c8001321dddea0f16eb85bf60f9184f2fdb2d507fd9a1464dfad948d',
    'contacts.jsonl': '89aeb60697d7c6badde163965439b2b2fe37bffc02845b1a41d36a33ec5d43b5',
    'initial-state.npz': '79e066fb682af6e69ce44f26f06a9b2165cdc9eb0924124a3a3eb6e254bf4ddc',
    'protocol.json': '12ddd136b4d24b53ed12b571b8a5cd5ad086e4ce36a54245e04af2660b26043f',
    'runner.executed.py': 'b0b9e3e4c087b6c072160f78a511bc3c3184730e22fd5ce37ef752160cd4350e',
    'prepared.json': '8b19cf62673b51b9b76d1fc0972da7602d665850f0dea69c56671371c1048406',
}


def state_sha(state):
    return hashlib.sha256(np.asarray(state, dtype=np.float64).tobytes(order='C')).hexdigest()


def impratio_xml(text):
    root = ET.fromstring(text)
    options = root.findall('option')
    if len(options) != 1 or options[0].get('cone') != 'elliptic' or 'impratio' in options[0].attrib:
        raise ValueError('Expected frozen elliptic option with implicit impratio=1')
    start = text.index('<option '); stop = text.index('/>', start)
    result = text[:stop] + ('' if text[stop-1].isspace() else ' ') + 'impratio="10" ' + text[stop:]
    changed = ET.fromstring(result)
    if changed.find('option').attrib.pop('impratio') != '10' or ET.tostring(root) != ET.tostring(changed):
        raise ValueError('More than the authorized option changed')
    return result


def check_models(a, b):
    checked = []
    for name in dir(a):
        if name.startswith('_'): continue
        x, y = getattr(a, name), getattr(b, name)
        if isinstance(x, (np.ndarray, int, float, str)):
            if not np.array_equal(x, y): raise ValueError('Compiled model differs: ' + name)
            checked.append(name)
    changed = [n for n in dir(a.opt) if not n.startswith('_') and not callable(getattr(a.opt, n))
               and not np.array_equal(getattr(a.opt, n), getattr(b.opt, n))]
    if changed != ['impratio'] or a.opt.impratio != 1 or b.opt.impratio != 10 or int(a.opt.cone) != 1:
        raise ValueError('Expected only elliptic impratio1->10: ' + repr(changed))
    return dict(option_differences=changed, identical_public_model_fields=checked,
                all_other_option_fields_equal=True)


def claim_run(out):
    if any((out / n).exists() for n in ('trajectory.npz', 'contacts.jsonl', 'execution.json')):
        raise FileExistsError('Existing Run output; never overwrite')
    with (out / 'STARTED').open('x') as stream:
        stream.write('Issue12: one authorized impratio10 attempt; never resume or rerun.\n')


def prepare(source, out):
    verify_frozen(); verify_files(source, A_HASHES)
    if platform.python_version() != '3.11.9' or mujoco.__version__ != '3.2.3':
        raise ValueError('Unexpected frozen environment')
    original_prepared = json.loads((source / 'prepared.json').read_text())
    verify_files(source, original_prepared['artifacts'])
    # Anchor every original A Run file to the Issue8 evidence manifest.
    manifest = source.parents[3] / 'debug/issue8/evidence-manifest.json'
    archived = json.loads(manifest.read_text()); source_hashes = {}
    for path in sorted(source.iterdir()):
        if not path.is_file(): raise ValueError('Unexpected source directory')
        key = str(path.relative_to(source.parents[3]))
        if archived[key]['sha256'] != digest(path): raise ValueError('A archive differs: ' + key)
        source_hashes[path.name] = digest(path)
    assets = original_prepared['referenced_asset_hashes']
    for path, expected in assets.items():
        if digest(path) != expected: raise ValueError('Referenced asset differs: ' + path)
    # Protect all imported execution helpers in this worktree against c0e2310.
    helpers = {}
    for name in ('controller.py', 'run.py', 'grasp_support_ab.py', 'grasp_cone_ab.py'):
        path = Path(__file__).with_name(name)
        expected = subprocess.check_output(['git','show',FREEZE+':experiments/dynamics/'+name], cwd=ASSET_ROOT)
        if path.read_bytes() != expected: raise ValueError('Execution dependency differs: ' + name)
        helpers[str(path.resolve())] = digest(path)
    original = (source / 'model.xml').read_text(); derived = impratio_xml(original)
    ma = mujoco.MjModel.from_xml_path(str(source / 'model.xml'))
    out.mkdir(parents=True, exist_ok=False)
    (out / 'model.xml').write_text(derived)
    (out / 'model.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True), derived.splitlines(True), fromfile='A-impratio1/model.xml', tofile='B-impratio10/model.xml')))
    mb = mujoco.MjModel.from_xml_path(str(out / 'model.xml'))
    comparison = check_models(ma, mb)
    state = npz(source / 'initial-state.npz'); old = npz(source / 'trajectory.npz')
    if state['integration_state'].shape != (156,) or state_sha(state['integration_state']) != STATE_SHA:
        raise ValueError('Full initial state hash mismatch')
    np.testing.assert_array_equal(state['integration_state'], state['restored_integration_before_forward'])
    np.testing.assert_array_equal(state['controller_requested'], state['controller_command'])
    config = json.loads((source / 'protocol.json').read_text())
    np.testing.assert_array_equal(state['controller_command'], config['constant_command_8d'])
    for key in ('requested', 'command'):
        np.testing.assert_array_equal(old[key], np.tile(state['controller_'+key], (STEPS, 1)))
    after = {}; restored = {}
    for label, model in [('impratio1', ma), ('impratio10', mb)]:
        data = restore_state(model, state['integration_state'])
        before = integration_state(model, data)
        for key in ('qpos','qvel','qacc_warmstart','ctrl'):
            np.testing.assert_array_equal(getattr(data,key),state[key])
        if data.time != state['time'][0]: raise ValueError('Snapshot time mismatch')
        mujoco.mj_forward(model, data)
        for key, value in pose(model, data).items(): np.testing.assert_array_equal(value, state[key])
        after[label] = cache(data); restored[label] = integration_state(model, data)
        np.savez_compressed(out / (label+'-after-forward.npz'), **after[label],
                            pre_forward_state=before, integration_state=restored[label])
        events = frozen.contacts(model, data)
        if any('cube' in e['bodies'] and 'table' in e['geoms'] for e in events):
            raise ValueError('Initial table contact invalidates independent load')
        frozen.write_json(out / (label+'-initial-contacts.json'), events)
        if label == 'impratio1':
            np.testing.assert_array_equal(restored[label], state['restored_integration_after_forward'])
            if events != json.loads((source / 'elliptic-initial-contacts.json').read_text()):
                raise ValueError('Frozen A initial contact solve not reproduced')
    new_state = dict(state); new_state['restored_integration_after_forward'] = restored['impratio10']
    np.savez_compressed(out / 'initial-state.npz', **new_state)
    differences = {k:dict(A_shape=list(after['impratio1'][k].shape),B_shape=list(after['impratio10'][k].shape),
        equal=bool(np.array_equal(after['impratio1'][k],after['impratio10'][k])),
        max_abs_difference=None if after['impratio1'][k].shape != after['impratio10'][k].shape else
        float(np.max(np.abs(after['impratio1'][k]-after['impratio10'][k]),initial=0))) for k in after['impratio1']}
    protocol = copy.deepcopy(config)
    protocol.update(label='impratio10',impratio=10,comparator_dir=str(source),
                    script_sha256=digest(__file__),model_sha256=digest(out / 'model.xml'))
    frozen.write_json(out / 'protocol.json', protocol)
    frozen.write_json(out / 'prediction.json', dict(issue=12,diagnostic_only=True,P1_elapsed_s=1.328,
        P1_strict_relative_drop_threshold_m=.005761707,A_reference_drop_m=.011523413088786877,
        validity='No cube/table geometric contact throughout elapsed [0,1.328s], including t0; exact source/model/state validation',
        definition='Initial(cube_COM.z-gripper_base_origin.z) minus current gap in world Z',
        statuses='PASS if valid and strict threshold met; FAIL if valid and threshold not met; INVALID/UNKNOWN otherwise',
        maximum_new_mj_steps=STEPS,old_A_new_steps=0,attempts=1,
        auxiliary_times_s=[.068,.141,.751,1.328,2.164],
        acceptance='All original config/grasp criteria preserved; P1 does not establish sustained grasp or hardware fidelity',
        stop='Nonfinite state, numerical warning, protocol inconsistency: retain failure; never retry or tune'))
    (out / 'runner.executed.py').write_bytes(Path(__file__).read_bytes())
    sealed = [p for p in out.iterdir() if p.is_file()]
    frozen.write_json(out / 'prepared.json', dict(issue=12,diagnostic_only=True,source=str(source),source_hashes=source_hashes,
        referenced_asset_hashes=assets,execution_helper_hashes=helpers,
        artifacts={p.name:digest(p) for p in sealed},runner_sha256=digest(__file__),
        original_manifest=dict(path=str(manifest),sha256=digest(manifest)),model_comparison=comparison,
        pre_forward_full_state_equal=True,pre_forward_sha256=STATE_SHA,
        command_raw_float64_sha256=state_sha(state['controller_command']),
        post_forward_cache_differences=differences,actual_mj_steps=0,python=sys.version,mujoco=mujoco.__version__,
        git_base=FREEZE,argv=sys.argv))
    print('PREPARED zero steps; prediction SHA256 '+digest(out / 'prediction.json'),flush=True)


def execute(out):
    prepared = json.loads((out / 'prepared.json').read_text())
    if platform.python_version() != '3.11.9' or mujoco.__version__ != '3.2.3':
        raise ValueError('Unexpected frozen environment')
    verify_frozen(); verify_files(out, prepared['artifacts'])
    verify_files(Path(prepared['source']), prepared['source_hashes'])
    for group in ('referenced_asset_hashes', 'execution_helper_hashes'):
        for path, expected in prepared[group].items():
            if digest(path) != expected: raise ValueError('Dependency changed: ' + path)
    if digest(__file__) != prepared['runner_sha256']: raise ValueError('Runner changed after seal')
    model = mujoco.MjModel.from_xml_path(str(out / 'model.xml'))
    original = mujoco.MjModel.from_xml_path(str(Path(prepared['source']) / 'model.xml'))
    check_models(original, model)
    z = npz(out / 'initial-state.npz'); protocol = json.loads((out / 'protocol.json').read_text())
    if state_sha(z['integration_state']) != STATE_SHA: raise ValueError('Initial state changed')
    controller = Controller(model, protocol['config'])
    controller.requested = z['controller_requested'].copy(); controller.command = z['controller_command'].copy()
    data = restore_state(model, z['integration_state']); mujoco.mj_forward(model, data)
    np.testing.assert_array_equal(integration_state(model, data), z['restored_integration_after_forward'])
    claim_run(out)
    recorder = frozen.Recorder(out, model, controller)
    observations = dict(time=[], **{key:[] for key in pose(model,data)})
    warnings = []; error = None
    parameters = (out / 'contact-parameters.jsonl').open('x')
    try:
        for _ in range(STEPS):
            recorder.step(data, 'diagnostic_fixed_pose_hold')
            warnings.append(data.warning.number.copy()); observations['time'].append(float(data.time))
            for key,value in pose(model,data).items(): observations[key].append(value)
            parameters.write(json.dumps(dict(time=float(data.time),contacts=[dict(dim=int(c.dim),friction5=c.friction.tolist()) for c in data.contact]))+'\n')
            if recorder.actual_steps % 2000 == 0: print('impratio10 actual steps',recorder.actual_steps,flush=True)
    except BaseException:
        error = traceback.format_exc(); raise
    finally:
        parameters.close(); health = recorder.finish(error)
        np.savez_compressed(out / 'observations.npz', **{k:np.asarray(v) for k,v in observations.items()})
        np.savez_compressed(out / 'warnings.npz',time=np.asarray(observations['time']),numbers=np.asarray(warnings))
        frozen.write_json(out / 'execution.json',dict(issue=12,diagnostic_only=True,
            actual_mj_steps=recorder.actual_steps,actual_elapsed_s=float(data.time-z['time'][0]),error=error,
            control_health=health['status'],official_grasp_success_evaluated=False,
            warning_numbers=data.warning.number.tolist(),runner_sha256=digest(__file__),
            prediction_sha256=digest(out/'prediction.json'),prepared_sha256=digest(out/'prepared.json'),
            python=sys.version,mujoco=mujoco.__version__,argv=sys.argv))
    print(json.dumps(json.loads((out/'execution.json').read_text())),flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['prepare','run']);parser.add_argument('--source-dir',type=Path)
    parser.add_argument('--output-dir',type=Path,required=True);args=parser.parse_args()
    if args.mode == 'prepare':
        if args.source_dir is None: parser.error('prepare requires --source-dir')
        prepare(args.source_dir.resolve(),args.output_dir.resolve())
    else: execute(args.output_dir.resolve())
