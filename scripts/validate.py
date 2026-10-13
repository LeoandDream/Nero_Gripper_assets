"""Compare compiled static fields and geometry with independent source-URDF FK."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import collada
import mujoco
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]


def vector(text, default='0 0 0'):
    return np.fromstring(text or default, sep=' ')


def rpy_matrix(values):
    r, p, y = values
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def transform(element):
    result = np.eye(4)
    o = element.find('origin')
    if o is not None:
        result[:3, :3] = rpy_matrix(vector(o.get('rpy')))
        result[:3, 3] = vector(o.get('xyz'))
    return result


def motion(joint, value):
    result = np.eye(4)
    axis = vector(joint.find('axis').get('xyz'))
    axis /= np.linalg.norm(axis)
    if joint.get('type') == 'prismatic':
        result[:3, 3] = axis * value
    else:
        x, y, z = axis
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        result[:3, :3] = np.eye(3) + np.sin(value)*skew + (1-np.cos(value))*(skew @ skew)
    return result


def forward_kinematics(source, values):
    result = {'world': np.eye(4)}
    remaining = list(source.findall('joint'))
    while remaining:
        progressed = False
        for joint in remaining[:]:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in result:
                continue
            matrix = result[parent] @ transform(joint)
            if joint.get('type') != 'fixed':
                matrix = matrix @ motion(joint, values[joint.get('name')])
            result[child] = matrix
            remaining.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError('Disconnected/cyclic source joint tree')
    return result


def mesh_vertices(path):
    if path.suffix == '.dae':
        document = collada.Collada(str(path))
        pieces = []
        for geometry in document.scene.objects('geometry'):
            for primitive in geometry.primitives():
                t = primitive.triangleset() if hasattr(primitive, 'triangleset') else primitive
                pieces.append(t.vertex)
        return np.concatenate(pieces).astype(float)
    return trimesh.load(path, process=False).vertices.astype(float)


def compiled_vertices(model, data, geom):
    mesh = int(model.geom_dataid[geom])
    start = int(model.mesh_vertadr[mesh])
    count = int(model.mesh_vertnum[mesh])
    points = model.mesh_vert[start:start + count].astype(float)
    return points @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]


def bounds(points):
    return np.array([points.min(axis=0), points.max(axis=0)])


def audit():
    manifest = json.loads((ROOT / 'SOURCE_MANIFEST.json').read_text())
    for item in manifest['files']:
        data = (ROOT / item['file']).read_bytes()
        if hashlib.sha256(data).hexdigest() != item['sha256'] or hashlib.sha1(f'blob {len(data)}\0'.encode()+data).hexdigest() != item['git_blob']:
            raise ValueError(f'Source snapshot changed: {item["file"]}')
    source = ET.parse(ROOT / 'urdf/nero_gripper.urdf').getroot()
    # Do not let a wrong derived URDF certify a matching wrong MJCF.
    originals = {}
    for item in manifest['files']:
        if Path(item['repo_path']).suffix not in ['.urdf', '.xacro']:
            continue
        for element in ET.parse(ROOT/item['file']).getroot():
            if element.tag not in ['link','joint']:
                continue
            key = (element.tag, element.get('name'))
            if key in originals:
                raise ValueError(f'Duplicate original source description: {key}')
            expected = copy.deepcopy(element)
            for mesh in expected.findall('.//mesh'):
                prefix = 'package://agx_arm_description/agx_arm_urdf/'
                if not mesh.get('filename').startswith(prefix):
                    raise ValueError('Unexpected source mesh URI')
                mesh.set('filename','../meshes/source/'+mesh.get('filename')[len(prefix):])
            originals[key] = ET.canonicalize(ET.tostring(expected,encoding='unicode'),strip_text=True)
    derived = {(e.tag,e.get('name')):ET.canonicalize(ET.tostring(e,encoding='unicode'),strip_text=True) for e in source if e.tag in ['link','joint']}
    if derived != originals:
        raise ValueError('Expanded URDF differs from original source descriptions')
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'mjcf/nero_gripper.xml'))
    collision = model.geom_group == 0
    visual = model.geom_group == 1
    if (not np.all(collision | visual) or not np.all(model.geom_contype[collision] == 1)
            or not np.all(model.geom_conaffinity[collision] == 1)
            or not np.all(model.geom_contype[visual] == 0)
            or not np.all(model.geom_conaffinity[visual] == 0)):
        raise ValueError('Collision masks differ from asset contract')
    links = {e.get('name'): e for e in source.findall('link') if e.find('inertial') is not None}
    joint_sources = {e.get('name'): e for e in source.findall('joint') if e.get('type') != 'fixed'}
    expected_bodies = {'world'} | set(links)
    if {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)} != expected_bodies:
        raise ValueError('Compiled physical body set differs from source')
    errors = dict(body_position_m=0., body_rotation_matrix=0., mass_kg=0., inertia_kg_m2=0., inertial_position_m=0., joint_axis=0., joint_range=0., geometry_bounds_m=0., mimic_m=0.)

    def record(key, difference):
        errors[key] = max(errors[key], float(np.max(np.abs(difference))))

    fields = []
    for name, link in links.items():
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        inertial = link.find('inertial')
        record('mass_kg', model.body_mass[bid] - float(inertial.find('mass').get('value')))
        expected = transform(inertial)
        record('inertial_position_m', model.body_ipos[bid] - expected[:3, 3])
        i = inertial.find('inertia')
        xx, yy, zz, xy, xz, yz = [float(i.get(k)) for k in ['ixx','iyy','izz','ixy','ixz','iyz']]
        tensor = np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
        expected_tensor = expected[:3,:3] @ tensor @ expected[:3,:3].T
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, model.body_iquat[bid])
        rotation = rotation.reshape(3,3)
        actual_tensor = rotation @ np.diag(model.body_inertia[bid]) @ rotation.T
        record('inertia_kg_m2', actual_tensor - expected_tensor)
        fields.append(dict(body=name, source_mass_kg=float(inertial.find('mass').get('value')), compiled_mass_kg=float(model.body_mass[bid])))
    for name, joint in joint_sources.items():
        if name == 'gripper':
            continue
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        axis = vector(joint.find('axis').get('xyz'))
        record('joint_axis', model.jnt_axis[jid] - axis / np.linalg.norm(axis))
        limit = joint.find('limit')
        record('joint_range', model.jnt_range[jid] - [float(limit.get('lower')), float(limit.get('upper'))])
        if model.jnt_type[jid] != (mujoco.mjtJoint.mjJNT_HINGE if joint.get('type') == 'revolute' else mujoco.mjtJoint.mjJNT_SLIDE):
            raise ValueError(f'Joint type mismatch: {name}')
    if not np.array_equal(model.eq_data[0,:5], [0,-1,0,0,0]) or model.nu != 0 or model.nexclude != 0:
        raise ValueError('Unexpected coupling, actuator, or explicit contact filter')
    cache = {}
    poses = []
    rng = np.random.default_rng(20261013)
    configs = [(f'zero_arm_width_{w}', np.zeros(7), w) for w in [0.,.05,.1]]
    for index in range(8):
        q = []
        for j in range(1,8):
            limit = joint_sources[f'joint{j}'].find('limit')
            q.append(rng.uniform(float(limit.get('lower')), float(limit.get('upper'))))
        configs.append((f'random_{index}', q, float(rng.uniform(0,.1))))
    for label, arm, width in configs:
        values = {f'joint{i+1}':float(q) for i,q in enumerate(arm)}
        values.update(gripper=width, gripper_joint1=.5*width, gripper_joint2=-.5*width)
        expected = forward_kinematics(source, values)
        data = mujoco.MjData(model)
        for name, value in values.items():
            if name != 'gripper':
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                data.qpos[model.jnt_qposadr[jid]] = value
        mujoco.mj_forward(model, data)
        if data.time != 0:
            raise ValueError('Static audit advanced physics')
        record('mimic_m', data.efc_pos[0])
        for name, link in links.items():
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            record('body_position_m', data.xpos[bid] - expected[name][:3,3])
            record('body_rotation_matrix', data.xmat[bid].reshape(3,3) - expected[name][:3,:3])
            for kind, group in [('visual',1),('collision',0)]:
                geometries = link.findall(kind)
                if not geometries:
                    continue
                expected_points = []
                for geometry in geometries:
                    path = (ROOT/'urdf'/geometry.find('geometry/mesh').get('filename')).resolve()
                    if path not in cache:
                        cache[path] = mesh_vertices(path)
                    matrix = expected[name] @ transform(geometry)
                    expected_points.append(cache[path] @ matrix[:3,:3].T + matrix[:3,3])
                actual = [compiled_vertices(model,data,g) for g in range(model.ngeom) if model.geom_bodyid[g]==bid and model.geom_group[g]==group]
                if not actual:
                    raise ValueError(f'Missing {kind}: {name}')
                record('geometry_bounds_m', bounds(np.concatenate(actual)) - bounds(np.concatenate(expected_points)))
        pairs = {}
        for contact in data.contact:
            names = tuple(sorted(mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[g])) for g in [contact.geom1,contact.geom2]))
            pairs[names] = min(pairs.get(names, 0.), float(contact.dist))
        poses.append(dict(name=label, arm_rad=list(map(float,arm)), source_gripper_command_m=width, self_contacts=[dict(bodies=list(pair),minimum_distance_m=distance) for pair,distance in sorted(pairs.items())], time=float(data.time)))
    tolerances = {key:1e-10 for key in errors}
    tolerances['inertia_kg_m2'] = 1e-12
    tolerances['geometry_bounds_m'] = 1e-6  # MuJoCo mesh vertices are float32.
    failures = {key:dict(error=errors[key], tolerance=tol) for key,tol in tolerances.items() if errors[key]>tol}
    result = dict(status='FAIL_STATIC_EQUIVALENCE' if failures else 'PASS_STATIC_EQUIVALENCE', source_revision=manifest['revision'], source_files=len(manifest['files']), expanded_urdf_matches_original_source=True, source_description_records=len(originals), physical_bodies=len(links), nq=model.nq, nv=model.nv, nu=model.nu, neq=model.neq, explicit_collision_excludes=model.nexclude, visual_geoms=int(np.count_nonzero(model.geom_group==1)), collision_geoms=int(np.count_nonzero(model.geom_group==0)), total_mass_kg=float(model.body_mass.sum()), max_errors=errors, tolerances=tolerances, failures=failures, body_fields=fields, poses=poses, simulation_steps=0, policy_actions=0, dynamic_physics_validated=False, limitations=['Convex collision approximations preserve bounds, not concave triangle surfaces.', 'Self-contact observations are reported; static FK equivalence does not certify contact realism.', 'No actuators/controller, dynamic mimic tracking, grasping, real-hardware calibration, or policy evaluation.'])
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'reports/static-validation.json')
    args = parser.parse_args()
    result = audit()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(result['status'], 'poses', len(result['poses']), 'max_errors', result['max_errors'])
    raise SystemExit(bool(result['failures']))
