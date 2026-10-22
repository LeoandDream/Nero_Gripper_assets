"""Extract Issue35 component from the immutable 77bd26a MVP; never writes meshes."""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
BASE = '77bd26a58df4eda99f611de7be1013d714ddaa7f'
SOURCE = ROOT / 'mjcf/simulation_mvp.xml'
DEST = ROOT / 'assets/components/gripper/gripper.xml'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    if DEST.exists() or DEST.with_name('manifest.json').exists():
        raise FileExistsError('Component already exists; do not overwrite')
    source = ET.parse(SOURCE).getroot()
    original = json.loads((ROOT/'mjcf/simulation_mvp.manifest.json').read_text())
    for name, expected in original['input_sha256'].items():
        if sha(ROOT/name) != expected:
            raise ValueError('Frozen source changed: ' + name)
    inputs = ['mjcf/simulation_mvp.xml', 'mjcf/simulation_mvp.manifest.json',
              'experiments/dynamics/config.json', 'experiments/dynamics/controller.py']
    for name in inputs:
        data = subprocess.check_output(['git', 'show', BASE+':'+name], cwd=ROOT)
        if hashlib.sha256(data).hexdigest() != sha(ROOT/name):
            raise ValueError('Source is not 77bd26a: ' + name)
    body = copy.deepcopy(source.find(".//body[@name='gripper_base']"))
    refs = {key: {n.get(key) for n in body.iter() if n.get(key)} for key in ('mesh', 'material')}
    assets = source.find('asset')
    textures = {n.get('texture') for n in assets if n.tag == 'material' and n.get('name') in refs['material'] and n.get('texture')}
    chosen = {**refs, 'texture': textures}
    root = ET.Element('mujoco', model='nero_mvp_standalone_gripper')
    root.append(copy.deepcopy(source.find('compiler')))
    root.append(copy.deepcopy(source.find('option')))
    target_assets = ET.SubElement(root, 'asset')
    dependencies = {}
    for node in assets:
        if node.get('name') not in chosen.get(node.tag, set()):
            continue
        node = copy.deepcopy(node)
        if node.get('file'):
            path = (SOURCE.parent/node.get('file')).resolve()
            dependencies[str(path.relative_to(ROOT))] = sha(path)
            node.set('file', os.path.relpath(path, DEST.parent))
        target_assets.append(node)
    mount = ET.SubElement(ET.SubElement(root, 'worldbody'), 'body', name='flange_mount', pos='0 0 0')
    mount.append(body)  # Includes the original +6 mm transform and both real slide bodies.
    equality = ET.SubElement(root, 'equality')
    equality.append(copy.deepcopy(source.find("equality/joint[@name='source_gripper_mimic']")))
    # Source base is welded to moving link7: its child slides are implicitly
    # parent-filtered. World-welded roots are exempt from that MuJoCo rule.
    # Preserve the original eligibility explicitly; do not suppress finger pairs.
    contact = ET.SubElement(root, 'contact')
    for name in ('gripper_link1', 'gripper_link2'):
        ET.SubElement(contact, 'exclude', body1='gripper_base', body2=name)
    actuators = ET.SubElement(root, 'actuator')
    for node in source.find('actuator'):
        if node.get('joint') in ('gripper_joint1', 'gripper_joint2'):
            actuators.append(copy.deepcopy(node))
    DEST.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    with DEST.open('xb') as f:
        ET.ElementTree(root).write(f, encoding='utf-8', xml_declaration=True)
    config = json.loads((ROOT/'experiments/dynamics/config.json').read_text())
    controls = {key: config[key] for key in ('finger_kp_N_m', 'finger_kv_Ns_m', 'finger_force_limit_N',
                'gripper_command_slew_m_s', 'command_period_s', 'timestep_s')}
    controls['initial_width_m'] = config['initial_action'][7]
    controls['acceptance'] = {key: config['acceptance'][key] for key in ('settled_finger_error_m',
        'maximum_coupling_error_m', 'maximum_finger_velocity_m_s', 'require_no_numerical_warnings_or_time_resets')}
    dependencies[str(DEST.relative_to(ROOT))] = sha(DEST)
    dependencies['examples/gripper_control.py'] = sha(ROOT/'examples/gripper_control.py')
    manifest = dict(source_commit=BASE, source_model='mjcf/simulation_mvp.xml',
        source_geometry_sha256=original['geometry_sha256'],
        source_inputs_sha256={name:sha(ROOT/name) for name in inputs},
        runtime_files_sha256=dependencies, generator_sha256=sha(__file__),
        path_relocations={n.get('name'):n.get('file') for n in target_assets if n.get('file')},
        installation=dict(parent='flange_mount', child='gripper_base', xyz_m=[0,0,.006], rpy_rad=[0,0,0],
                          fixed_reference='flange_mount represents the source flange_link frame; no arm/flange geometry'),
        action=dict(dimensions=1, name='width_command_m', range_m=[0,.1],
                    physical_targets=['+w/2','-w/2'], calibrated_fingertip_clearance=False),
        control=controls, expected_counts=dict(nq=2,nv=2,nu=2,neq=1,finger_collision_geoms=692),
        source_name_mapping='All gripper body/joint/geom/mesh/material/equality/actuator names preserved; '
                            'gripper_link{1,2}_mvp_V* maps directly to simulation_mvp.manifest.json mapping.',
        shared_meshes_copied_or_generated=0,
        contact_qualification_adaptation=dict(
            explicit_excludes=[['gripper_base','gripper_link1'],['gripper_base','gripper_link2']],
            reason='Preserve source implicit parent filtering after fixed-root/world weld extraction',
            source_reference='https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_collision_driver.c#L135-L150',
            finger_to_finger_collision_eligible=True, geometry_or_contact_parameters_changed=False),
        excluded_source_content=['NERO arm', 'flange_link geometry', 'cube', 'table', 'experiment skybox/light',
                                 'base_link-link1 exclusion (neither body exists in component)'])
    with DEST.with_name('manifest.json').open('x') as f:
        json.dump(manifest, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(dict(model=str(DEST.relative_to(ROOT)), runtime_files=len(dependencies),
                         finger_collision_geoms=692, generated_meshes=0)))


if __name__ == '__main__':
    main()
