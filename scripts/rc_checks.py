"""Issue37 RC file/closure/static acceptance. Physical integration is forbidden."""
import argparse
import ast
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class RemoveVersionRecord(ast.NodeTransformer):
    def visit_Expr(self, node):
        value = node.value
        if isinstance(value, ast.Call) and isinstance(value.func, ast.Name) and value.func.id == 'write' and value.args:
            path = value.args[0]
            if isinstance(path, ast.BinOp) and isinstance(path.right, ast.Constant) and path.right.value == 'before-run.json':
                return None
        return self.generic_visit(node)


def protected_ast(path):
    tree = ast.parse(Path(path).read_text())
    result = {}
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name != 'checkout_head':
            node = RemoveVersionRecord().visit(node)
            result[node.name] = hashlib.sha256(ast.dump(node).encode()).hexdigest()
    result['__module_statements__'] = hashlib.sha256(repr([
        ast.dump(node) for node in tree.body if not isinstance(node,(ast.FunctionDef,ast.ClassDef))]).encode()).hexdigest()
    return result


def check_files():
    selection = json.loads((ROOT/'RC_FILE_SELECTION.json').read_text())
    assert selection['stage'] == 'FROZEN'
    expected = {r['path']: r for r in selection['entries']}
    ignored = {'.git','.venv','__pycache__','debug','.pytest_cache'}
    actual = {str(p.relative_to(ROOT)) for p in ROOT.rglob('*') if p.is_file() and
              not any(part in ignored for part in p.relative_to(ROOT).parts)}
    assert actual == set(expected), dict(unlisted=sorted(actual-set(expected)),missing=sorted(set(expected)-actual))
    for name, record in expected.items():
        path = ROOT/name
        assert path.is_file(), name
        if record['target_sha256']:
            assert sha(path) == record['target_sha256'], name
        if record.get('immutable_physics'):
            assert sha(path) == record['source_sha256'], 'Core frozen bytes changed: '+name
    for name, fingerprints in selection['protected_physics_ast'].items():
        assert protected_ast(ROOT/name) == fingerprints, 'Physics/control AST changed: '+name
    sources = json.loads((ROOT/'SOURCE_MANIFEST.json').read_text())
    for item in sources['files']:
        assert sha(ROOT/item['file']) == item['sha256']
    for manifest, field in [('mjcf/simulation_mvp.manifest.json','input_sha256'),
                            ('assets/components/gripper/manifest.json','runtime_files_sha256'),
                            ('assets/components/gripper/manifest.json','source_inputs_sha256')]:
        for name, value in json.loads((ROOT/manifest).read_text())[field].items():
            assert (ROOT/name).is_file() and sha(ROOT/name) == value, (manifest,name)
    frozen = selection['standalone_frozen_control']
    current = json.loads((ROOT/'assets/components/gripper/manifest.json').read_text())
    assert current['control'] == frozen
    redacted = copy.deepcopy(current)
    redacted['runtime_files_sha256'].pop('examples/gripper_control.py')
    assert hashlib.sha256(json.dumps(redacted,sort_keys=True,separators=(',',':')).encode()).hexdigest() == selection['standalone_manifest_source_redacted_sha256']
    references = []
    for path in sorted(ROOT.rglob('*.xml')):
        if any(part in ('.git','.venv','debug','__pycache__') for part in path.relative_to(ROOT).parts):
            continue
        for node in ET.parse(path).getroot().iter():
            if node.get('file'):
                value = Path(node.get('file'))
                assert not value.is_absolute(), (str(path),str(value))
                target = (path.parent/value).resolve()
                assert target.is_relative_to(ROOT) and target.is_file(), (str(path),str(value))
                references.append(str(target.relative_to(ROOT)))
    lines = (ROOT/'SHA256SUMS').read_text().splitlines()
    checked_paths = [line.split('  ',1)[1] for line in lines]
    assert len(checked_paths) == len(set(checked_paths))
    assert set(checked_paths) == set(expected)-{'SHA256SUMS'}
    command = subprocess.run(['sha256sum','-c','SHA256SUMS'],cwd=ROOT,capture_output=True,text=True)
    assert command.returncode == 0, command.stdout+command.stderr
    return dict(selection_entries=len(expected),checksum_files=len(lines),sha256sum_exit_code=command.returncode,
                missing_files=0,source_manifest_files=len(sources['files']),xml_file_references=len(references),
                unique_xml_resources=len(set(references)),missing_xml_resources=0,absolute_xml_paths=0,
                protected_physics_files=sum(bool(v.get('immutable_physics')) for v in expected.values()),
                protected_control_ast='UNCHANGED',standalone_control_config='UNCHANGED',
                sha256sums_sha256=sha(ROOT/'SHA256SUMS'),
                file_selection_sha256=sha(ROOT/'RC_FILE_SELECTION.json'),
                checksum_stdout_sha256=hashlib.sha256(command.stdout.encode()).hexdigest())


def check_static():
    import mujoco
    import numpy as np
    assert mujoco.__version__ == '3.2.3'
    attempts = []
    def forbidden(*args, **kwargs):
        attempts.append('mj_step')
        raise AssertionError('Issue37 forbids physics integration')
    originals = {name:getattr(mujoco,name) for name in ('mj_step','mj_step1','mj_step2')}
    for name in originals: setattr(mujoco,name,forbidden)
    try:
        models = {}
        for path, counts in [('mjcf/nero_gripper.xml',(9,9,0,1)),
                             ('mjcf/simulation_mvp.xml',(16,15,9,1)),
                             ('assets/components/gripper/gripper.xml',(2,2,2,1))]:
            model = mujoco.MjModel.from_xml_path(str(ROOT/path))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model,data)
            assert (model.nq,model.nv,model.nu,model.neq) == counts
            assert data.time == 0 and not data.warning.number.any()
            models[path] = dict(sha256=sha(ROOT/path),**{k:int(getattr(model,k)) for k in ('nq','nv','nu','neq','nbody','ngeom','nmesh')})
        sys.path.insert(0,str(ROOT/'experiments/dynamics'))
        sys.path.insert(0,str(ROOT/'tests'))
        import simulation_mvp
        with tempfile.TemporaryDirectory(prefix='nero-rc-metadata-') as directory:
            destination = Path(directory)/'seal'
            simulation_mvp.freeze_execution(destination)  # hashes + recording only; never calls mj_step
            seal = json.loads((destination/'before-run.json').read_text())
        assert seal['asset_delivery_commit'] == '77bd26a58df4eda99f611de7be1013d714ddaa7f'
        if not (ROOT/'.git').exists(): assert seal['head'] is None
        spec = importlib.util.spec_from_file_location('rc_component',ROOT/'examples/gripper_control.py')
        component = importlib.util.module_from_spec(spec);spec.loader.exec_module(component)
        head = component.checkout_head()
        if not (ROOT/'.git').exists(): assert head is None
        stream = io.StringIO()
        suites = [unittest.defaultTestLoader.loadTestsFromName(name) for name in
                  ('test_standalone_gripper','test_simulation_mvp','test_rc_metadata')]
        outcome = unittest.TextTestRunner(stream=stream,verbosity=2).run(unittest.TestSuite(suites))
        assert outcome.wasSuccessful(), stream.getvalue()
        assert not attempts
        return dict(models=models,static_tests=outcome.testsRun,static_test_output=stream.getvalue(),
                    actual_mj_step_calls=0,forbidden_integration_attempts=0,mujoco=mujoco.__version__,numpy=np.__version__,
                    git_directory_present=(ROOT/'.git').exists(),mvp_recording_head=seal['head'],
                    gripper_recording_head=head,source_zip_metadata_supported=True)
    finally:
        for name,value in originals.items():setattr(mujoco,name,value)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path,required=True,help='New file outside the RC tree')
    args = parser.parse_args()
    assert sys.version_info[:2] == (3,11), 'Use the locked Python3.11 AST/runtime convention'
    output = args.report.resolve()
    if output.is_relative_to(ROOT):parser.error('Evidence must be outside the sealed RC tree')
    result = dict(status='RC_TECHNICAL_READY_LICENSE_PENDING',release='RELEASE_BLOCKED',
                  license_gate='BLOCKED_LICENSE_OWNER_DECISION',root=str(ROOT),
                  files=check_files(),static=check_static(),new_physical_episodes=0)
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('static',)}))
