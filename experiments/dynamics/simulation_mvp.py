"""Issue25 Simulation MVP: frozen346 collision export and unchanged R1 execution.

build: independently loadable MJCF + 346 shared meshes, 692 named finger geoms.
static: no integration. smoke: one complete, unmodified R1 episode per new output.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

import run as runtime
from controller import ASSET_ROOT as ROOT, Controller, build_model

BASE = '6eba4f34c5ff5029eb849804e3d2e9c535f58756'
GEOMETRY_SHA = '79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e'
MODEL = ROOT/'mjcf/simulation_mvp.xml'
MANIFEST = MODEL.with_suffix('.manifest.json')
MESH_DIR = ROOT/'meshes/collision/simulation_mvp'
CASE = Path(__file__).with_suffix('.json')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def load_case():
    return json.loads(CASE.read_text())


def baseline_xml(case):
    """Use the original builder; only the already-approved R1 options differ."""
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'baseline.xml'
        build_model(path, case['config'], exclude_base_link1=True, cube=case['cube'])
        root = ET.parse(path).getroot()
        root.find('option').set('cone', 'elliptic')
        root.find('option').set('impratio', '10')
        for node in root.iter():
            if node.get('file'):
                node.set('file', os.path.relpath((path.parent/node.get('file')).resolve(), MODEL.parent))
        return ET.tostring(root, encoding='unicode')


def frozen_entities():
    raw = subprocess.check_output(['git', 'show', BASE+':reports/nero-gripper-v2-runtime-partition.json'], cwd=ROOT)
    payload = json.loads(raw)
    entities = payload['entities']
    digest = hashlib.sha256(json.dumps([[e['id'], e['vertices_m'], e['faces']] for e in entities], separators=(',', ':')).encode()).hexdigest()
    if digest != GEOMETRY_SHA or len(entities) != 346:
        raise ValueError('Wrong baseline collision geometry')
    return entities, hashlib.sha256(raw).hexdigest()


def build():
    if MODEL.exists() or MANIFEST.exists() or MESH_DIR.exists():
        raise FileExistsError('MVP already exists; never overwrite assets. Use static/smoke.')
    runtime.asset_integrity()
    entities, blob_sha = frozen_entities()
    root = ET.fromstring(baseline_xml(load_case()))
    assets = root.find('asset')
    for node in list(assets):
        if node.tag == 'mesh' and node.get('name', '').startswith(('gripper_link1_collision_', 'gripper_link2_collision_')):
            assets.remove(node)
    MESH_DIR.mkdir()
    mesh_hashes = {}
    for e in entities:
        path = MESH_DIR/(e['id']+'.obj')
        # Exact source vertices/faces, meters, 17 significant digits. No hull
        # merge, repartition, inflation, face culling or geometry repair.
        with path.open('x') as stream:
            stream.write('# Frozen v2.1 '+e['id']+'; coordinates in original finger-local meters\n')
            for vertex in e['vertices_m']:
                stream.write('v '+' '.join(format(x, '.17g') for x in vertex)+'\n')
            for face in e['faces']:
                stream.write('f '+' '.join(str(i+1) for i in face)+'\n')
        ET.SubElement(assets, 'mesh', name='mvp_'+e['id'], file=os.path.relpath(path, MODEL.parent))
        mesh_hashes[str(path.relative_to(ROOT))] = sha(path)
    mapping = []
    for index in (1, 2):
        body_name = 'gripper_link'+str(index)
        body = root.find(".//body[@name='"+body_name+"']")
        old = body.findall('geom')[1:]
        if len(old) != 2 or any(n.get('contype') != '1' for n in old):
            raise ValueError('Unexpected source finger geoms')
        template = old[0].attrib.copy()
        for node in old:
            body.remove(node)
        for e in entities:
            name = body_name+'_mvp_'+e['id']
            attrib = dict(template, name=name, mesh='mvp_'+e['id'])
            ET.SubElement(body, 'geom', **attrib)
            mapping.append(dict(entity=e['id'], semantic_family=e['semantic_family'], body=body_name,
                                mesh='mvp_'+e['id'], geom=name, source_regions=e['source_regions'],
                                source_triangle_ids=e['source_triangle_ids']))
    ET.indent(root)
    ET.ElementTree(root).write(MODEL, encoding='utf-8', xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    dependencies = dict(mesh_hashes)
    for node in root.iter():
        if node.get('file'):
            path = (MODEL.parent/node.get('file')).resolve()
            dependencies[str(path.relative_to(ROOT))] = sha(path)
    dependencies[str(MODEL.relative_to(ROOT))] = sha(MODEL)
    dependencies[str(CASE.relative_to(ROOT))] = sha(CASE)
    manifest = dict(baseline_commit=BASE, geometry_sha256=GEOMETRY_SHA, baseline_partition_blob_sha256=blob_sha,
                    model_sha256=sha(MODEL), mapping=mapping, input_sha256=dependencies,
                    source_frame='Unchanged gripper_link1 STL local meters; both fingers use the original body transforms.',
                    geometry_policy='Original 346 closed convex entities; shared meshes, 692 geoms, internal same-body overlap allowed.',
                    physics='Unchanged R1 elliptic/impratio10, legacy MPR; original masks/exclusion/equality and 9 actuators.',
                    research_state='V2_RUNTIME_ENTITY_PARTITION_BLOCKED', engineering_state='PENDING_ACTUAL_TESTS',
                    generator_sha256=sha(__file__), model_counts={k:int(getattr(model,k)) for k in ('nbody','njnt','nq','nv','nu','neq','ngeom','nmesh')})
    write(MANIFEST, manifest)
    print(json.dumps(dict(compile='PASS', geometry_sha256=GEOMETRY_SHA, **manifest['model_counts'])), flush=True)


def verify_assets():
    runtime.asset_integrity()
    manifest = json.loads(MANIFEST.read_text())
    if manifest['geometry_sha256'] != GEOMETRY_SHA:
        raise ValueError('Wrong MVP baseline')
    for name, expected in manifest['input_sha256'].items():
        if sha(ROOT/name) != expected:
            raise ValueError('Asset modified: '+name)
    return manifest


def freeze_execution(output):
    """Seal real project imports, assets and environment before any integration."""
    manifest = verify_assets()
    output.mkdir(parents=True, exist_ok=False)
    source = output/'executed-source'
    source.mkdir()
    modules = {}
    for module in tuple(sys.modules.values()):
        path = getattr(module, '__file__', None)
        if path:
            path = Path(path).resolve()
            if path.is_relative_to(ROOT) and path.is_file():
                relative = path.relative_to(ROOT)
                target = source/relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(path, target)
                modules[str(relative)] = sha(path)
    shutil.copyfile(CASE, source/CASE.relative_to(ROOT))
    write(output/'before-run.json', dict(argv=sys.argv, head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
          python=sys.version, platform=platform.platform(), mujoco=mujoco.__version__, numpy=np.__version__,
          scipy=__import__('scipy').__version__, project_imports_sha256=modules,
          case_sha256=sha(CASE), manifest_sha256=sha(MANIFEST), assets_sha256=manifest['input_sha256'],
          geometry_sha256=GEOMETRY_SHA, timestamp_utc=__import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat()))


def static(output):
    freeze_execution(output)
    case = load_case()
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    results = []
    for name, arm, width in (
        ('home_open',case['config']['initial_action'][:7],.1),
        ('home_close',case['config']['initial_action'][:7],0.),
        ('R1_initial',case['joint_plan']['initial_action'][:7],.08),
        ('R1_at_grasp_open',case['joint_plan']['approach'][-1],.1),
        ('R1_at_grasp_40mm',case['joint_plan']['approach'][-1],.04)):
        control = Controller(model,case['config'])
        data = runtime.reset(model, control, np.r_[arm,width])
        events = runtime.contacts(model,data)
        results.append(dict(pose=name,contact_count=data.ncon,max_penetration_mm=max([0.]+[-e['distance_m']*1000 for e in events]),events=events))
        np.savez_compressed(output/(name+'.npz'), qpos=data.qpos,qvel=data.qvel,geom_xpos=data.geom_xpos,geom_xmat=data.geom_xmat)
    write(output/'static.json',dict(poses=results,actual_mj_steps=0,compile='PASS',warnings='none',geometry_sha256=GEOMETRY_SHA))
    print(json.dumps([{k:r[k] for k in ('pose','contact_count','max_penetration_mm')} for r in results]),flush=True)


def smoke(output):
    freeze_execution(output)
    case = load_case()
    original_builder, original_recorder = runtime.build_model, runtime.Recorder
    started = time.monotonic()
    def builder(path,*args,**kwargs):
        root = ET.parse(MODEL).getroot()
        for node in root.iter():
            if node.get('file'):
                node.set('file',os.path.relpath((MODEL.parent/node.get('file')).resolve(),path.parent))
        ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
        return mujoco.MjModel.from_xml_path(str(path))
    class Recorded(original_recorder):
        def record(self,data,name):
            events = super().record(data,name)
            if self.actual_steps % 2000 == 0:
                print('actual_steps',self.actual_steps,'phase',name,'contacts',data.ncon,'wall_s',round(time.monotonic()-started,1),flush=True)
            return events
    runtime.build_model, runtime.Recorder = builder, Recorded
    result = None
    try:
        result = runtime.execute_grasp(output/'R1_smoke',case['config'],case['grasp_protocol'],case['cube'],case['joint_plan'])
    finally:
        runtime.build_model, runtime.Recorder = original_builder, original_recorder
        verify_assets()
        write(output/'execution.json',dict(wall_s=time.monotonic()-started,result=result,
              actual_mj_steps=None if result is None else result['actual_mj_steps'],
              original_acceptance_unchanged=True, episode_count=1))
        write(output/'raw-manifest.json',{str(p.relative_to(output)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in output.rglob('*') if p.is_file()})
    # A grasp FAIL is evidence, not a script malfunction or automatic MVP blocker.
    if result['error']:
        raise RuntimeError(result['error'])


def view():
    verify_assets()
    from mujoco import viewer
    case = load_case()
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    control = Controller(model,case['config'])
    data = runtime.reset(model,control,case['joint_plan']['initial_action'])
    with viewer.launch_passive(model,data) as window:
        window.cam.lookat[:] = [-.25,0,.25]
        window.cam.distance = 1.2
        window.opt.geomgroup[0] = 0
        window.opt.geomgroup[1:3] = 1
        while window.is_running():
            start = time.monotonic()
            control.apply(data)
            mujoco.mj_step(model,data)
            window.sync()
            time.sleep(max(0.,model.opt.timestep-(time.monotonic()-start)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('build','static','smoke','view'))
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    if args.mode in ('static','smoke') and args.output is None:
        parser.error('--output must name a new, nonexistent evidence directory')
    if args.mode == 'build': build()
    elif args.mode == 'static': static(args.output.resolve())
    elif args.mode == 'smoke': smoke(args.output.resolve())
    else: view()
