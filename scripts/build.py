"""Rebuild this pinned asset from its byte-preserved source snapshots."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import collada
import numpy as np
from PIL import Image
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh

ROOT = Path(__file__).resolve().parents[1]
REV = 'a68b8e876a2c6fcb0ecee7a7943129e4d48dd864'
ENTRY = 'nero/urdf/nero_with_gripper_description.xacro'
PACKAGE = 'package://agx_arm_description/agx_arm_urdf/'
INCLUDE = '$(find agx_arm_description)/agx_arm_urdf/'
XINCLUDE = '{http://www.ros.org/wiki/xacro}include'


def write_xml(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def numbers(values):
    return ' '.join(format(float(v), '.17g') for v in values)


def origin(element):
    o = element.find('origin')
    if o is None:
        return np.zeros(3), np.eye(3)
    return np.fromstring(o.get('xyz', '0 0 0'), sep=' '), Rotation.from_euler('xyz', np.fromstring(o.get('rpy', '0 0 0'), sep=' ')).as_matrix()


def quaternion(matrix):
    xyzw = Rotation.from_matrix(matrix).as_quat()
    return numbers(xyzw[[3, 0, 1, 2]])


def snapshot(repo):
    revision = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != REV:
        raise ValueError(f'Source must be pinned to {REV}, got {revision}')
    paths = set()

    def scan(relative):
        if relative in paths:
            return
        paths.add(relative)
        tree = ET.parse(repo / relative).getroot()
        for element in tree.iter():
            if element.tag == XINCLUDE:
                scan(element.get('filename').removeprefix(INCLUDE))
            elif element.tag == 'mesh':
                paths.add(element.get('filename').removeprefix(PACKAGE))

    scan(ENTRY)
    paths.update(['LICENSE', 'README.md'])
    records = []
    for relative in sorted(paths):
        if relative == 'LICENSE':
            target = ROOT / 'licenses/AGX_MIT.txt'
        elif relative == 'README.md':
            target = ROOT / 'licenses/UPSTREAM_README.md'
        elif '/urdf/' in relative:
            target = ROOT / 'urdf/source' / relative
        else:
            target = ROOT / 'meshes/source' / relative
        path = repo / relative
        data = path.read_bytes() if path.is_file() else subprocess.check_output(['git', '-C', str(repo), 'show', f'HEAD:{relative}'])
        blob = hashlib.sha1(f'blob {len(data)}\0'.encode() + data).hexdigest()
        expected = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', f'HEAD:{relative}'], text=True).strip()
        if blob != expected:
            raise ValueError(f'Source working copy differs from commit: {relative}')
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        records.append(dict(repo_path=relative, file=str(target.relative_to(ROOT)), git_blob=blob, sha256=hashlib.sha256(data).hexdigest(), bytes=len(data)))
    write_json(ROOT / 'SOURCE_MANIFEST.json', dict(repository='https://github.com/agilexrobotics/agx_arm_urdf', revision=REV, entrypoint=ENTRY, files=records))


def expand(relative):
    root = ET.parse(ROOT / 'urdf/source' / relative).getroot()
    result = ET.Element('robot', name='nero_gripper')
    for element in root:
        if element.tag == XINCLUDE:
            result.extend(expand(element.get('filename').removeprefix(INCLUDE)))
        else:
            result.append(copy.deepcopy(element))
    return result


def write_obj(path, vertices, faces, uv=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as out:
        for v in vertices:
            out.write('v ' + numbers(v) + '\n')
        if uv is not None:
            for t in uv:
                out.write('vt ' + numbers(t) + '\n')
        for f in faces:
            if uv is None:
                out.write('f ' + ' '.join(str(int(i) + 1) for i in f) + '\n')
            else:
                out.write('f ' + ' '.join(f'{int(i)+1}/{int(i)+1}' for i in f) + '\n')


def visual_mesh(source, name, asset):
    document = collada.Collada(str(source))
    if document.assetInfo.unitmeter != 1 or document.assetInfo.upaxis != 'Z_UP':
        raise ValueError(f'Unimplemented COLLADA units/up axis: {source}')
    pieces = []
    unbound_material_primitives = 0
    for geometry in document.scene.objects('geometry'):
        for primitive in geometry.primitives():
            t = primitive.triangleset() if hasattr(primitive, 'triangleset') else primitive
            if t.material is None:
                diffuse = (.5, .5, .5, 1.)
                unbound_material_primitives += 1
            else:
                diffuse = t.material.effect.diffuse
            if not isinstance(diffuse, tuple) or len(diffuse) != 4 or diffuse[3] != 1:
                raise ValueError(f'Unsupported diffuse material: {source}, {diffuse}')
            pieces.append((np.asarray(t.vertex, dtype=float), np.asarray(t.vertex_index, dtype=int), diffuse))
    if not pieces:
        raise ValueError(f'No visual triangles: {source}')
    vertices, faces, uv, colors = [], [], [], []
    offset = 0
    for i, (v, f, rgba) in enumerate(pieces):
        vertices.extend(v)
        faces.extend(f + offset)
        uv.extend([[float(i + .5) / len(pieces), .5]] * len(v))
        colors.append(rgba)
        offset += len(v)
    target = ROOT / 'meshes/visual' / f'{name}.obj'
    write_obj(target, vertices, faces, uv)
    palette = np.zeros((16, 16 * len(pieces), 3), dtype=np.uint8)
    for i, color in enumerate(colors):
        palette[:, i * 16:(i + 1) * 16] = np.rint(np.array(color[:3]) * 255).astype(np.uint8)
    texture = ROOT / 'meshes/visual' / f'{name}.png'
    Image.fromarray(palette).save(texture)
    ET.SubElement(asset, 'mesh', name=name + '_visual_mesh', file='../' + str(target.relative_to(ROOT)))
    ET.SubElement(asset, 'texture', name=name + '_colors', type='2d', file='../' + str(texture.relative_to(ROOT)))
    ET.SubElement(asset, 'material', name=name + '_visual_material', texture=name + '_colors', rgba='1 1 1 1', texuniform='false', specular='0.1', shininess='0.1')
    return dict(file=str(target.relative_to(ROOT)), source=str(source.relative_to(ROOT)), triangles=len(faces), diffuse_colors=colors, unbound_material_primitives=unbound_material_primitives, unbound_material_fallback_rgba=[.5,.5,.5,1.] if unbound_material_primitives else None, bounds_m=[np.min(vertices, axis=0).tolist(), np.max(vertices, axis=0).tolist()])


def collision_mesh(source, name, asset):
    mesh = trimesh.load(source, process=False)
    vertices, inverse = np.unique(mesh.vertices, axis=0, return_inverse=True)
    faces = inverse[mesh.faces]
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(len(vertices), len(vertices)))
    count, labels = connected_components(graph, directed=False)
    groups, planar = [], []
    for i in range(count):
        f = faces[labels[faces[:, 0]] == i]
        ids = np.unique(f)
        rank = np.linalg.matrix_rank(vertices[ids] - vertices[ids].mean(axis=0))
        record = dict(source_components=[i], source_face_count=len(f), vertex_ids=ids, faces=f)
        if rank == 3:
            groups.append(record)
        else:
            planar.append(record)
    # Preserve planar source decorations without manufacturing collision thickness.
    planar_merges = []
    for part in planar:
        possible = []
        for i, group in enumerate(groups):
            hull = ConvexHull(vertices[group['vertex_ids']])
            distance = vertices[part['vertex_ids']] @ hull.equations[:, :3].T + hull.equations[:, 3]
            if float(distance.max()) <= 1e-10:
                possible.append((hull.volume, i))
        if not possible:
            raise ValueError(f'{name}: planar component not contained in a source solid hull')
        _, index = min(possible)
        group = groups[index]
        group['vertex_ids'] = np.union1d(group['vertex_ids'], part['vertex_ids'])
        group['faces'] = np.concatenate([group['faces'], part['faces']])
        group['source_face_count'] += part['source_face_count']
        group['source_components'].extend(part['source_components'])
        planar_merges.append(dict(source_components=part['source_components'], target_collision_part=index, source_face_count=part['source_face_count'], reason='all planar source vertices contained in this existing convex hull', containment_tolerance_m=1e-10))
    parts = []
    for i, group in enumerate(groups):
        points = vertices[group['vertex_ids']]
        hull = ConvexHull(points)
        oriented = []
        for face, equation in zip(hull.simplices, hull.equations):
            a, b, c = points[face]
            oriented.append(face if np.dot(np.cross(b - a, c - a), equation[:3]) > 0 else face[[0, 2, 1]])
        target = ROOT / 'meshes/collision' / f'{name}_{i:03d}.obj'
        write_obj(target, points, oriented)
        key = f'{name}_collision_{i:03d}'
        ET.SubElement(asset, 'mesh', name=key, file='../' + str(target.relative_to(ROOT)))
        _, remap = np.unique(group['faces'], return_inverse=True)
        source_part = trimesh.Trimesh(vertices=points, faces=remap.reshape(-1, 3), process=False)
        volume = abs(float(source_part.volume)) if source_part.is_watertight else None
        parts.append(dict(mesh=key, file=str(target.relative_to(ROOT)), source_components=group['source_components'], source_faces=group['source_face_count'], vertices=len(points), hull_volume_m3=float(hull.volume), source_closed_volume_m3=volume, volume_ratio=float(hull.volume / volume) if volume and volume > 1e-20 else None))
    if sum(p['source_faces'] for p in parts) != len(faces):
        raise ValueError(f'Incomplete source-face coverage: {name}')
    return dict(source=str(source.relative_to(ROOT)), source_faces=len(faces), source_components=count, exact_coordinate_partition=True, source_vertices_moved=False, source_topology_repaired=False, parts=parts, planar_merges=planar_merges)


def build():
    source = expand(ENTRY)
    portable = copy.deepcopy(source)
    for mesh in portable.findall('.//mesh'):
        relative = mesh.get('filename').removeprefix(PACKAGE)
        mesh.set('filename', '../meshes/source/' + relative)
    write_xml(portable, ROOT / 'urdf/nero_gripper.urdf')
    links = {e.get('name'): e for e in source.findall('link')}
    joints = {e.find('child').get('link'): e for e in source.findall('joint')}
    # This zero-mass leaf is a ROS command coordinate, not a mechanical part.
    virtual = links['gripper_link']
    if len(virtual) or any(j.find('parent').get('link') == 'gripper_link' for j in source.findall('joint')):
        raise ValueError('Virtual driver is not an empty leaf; refusing to omit physical content')
    driver = joints['gripper_link']
    if driver.get('name') != 'gripper':
        raise ValueError('Unexpected virtual command joint')
    for name, coefficient in [('gripper_link1', .5), ('gripper_link2', -.5)]:
        mimic = joints[name].find('mimic')
        if mimic.get('joint') != 'gripper' or float(mimic.get('multiplier')) != coefficient or float(mimic.get('offset')) != 0:
            raise ValueError('Unexpected source mimic equation')
    root = ET.Element('mujoco', model='nero_official_gripper_assets')
    ET.SubElement(root, 'compiler', angle='radian', fusestatic='false', inertiafromgeom='false', autolimits='true')
    asset = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    visual_records, collision_records = {}, {}

    def body(name, parent):
        if name == 'gripper_link':
            return
        link = links[name]
        joint = joints.get(name)
        position, rotation = origin(joint) if joint is not None else (np.zeros(3), np.eye(3))
        element = ET.SubElement(parent, 'body', name=name, pos=numbers(position), quat=quaternion(rotation))
        inertial = link.find('inertial')
        if inertial is not None:
            pos, r = origin(inertial)
            inertia = inertial.find('inertia')
            xx, yy, zz, xy, xz, yz = [float(inertia.get(k)) for k in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']]
            tensor = r @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ r.T
            moments, axes = np.linalg.eigh(tensor)
            if np.min(moments) <= 0:
                raise ValueError(f'Non-positive source inertia: {name}')
            if np.linalg.det(axes) < 0:
                axes[:, -1] *= -1
            ET.SubElement(element, 'inertial', pos=numbers(pos), mass=inertial.find('mass').get('value'), diaginertia=numbers(moments), quat=quaternion(axes))
        if joint is not None and joint.get('type') != 'fixed':
            limit = joint.find('limit')
            ET.SubElement(element, 'joint', name=joint.get('name'), type='hinge' if joint.get('type') == 'revolute' else 'slide', pos='0 0 0', axis=joint.find('axis').get('xyz'), range=limit.get('lower') + ' ' + limit.get('upper'))
        for i, geometry in enumerate(link.findall('visual')):
            source_mesh = ROOT / 'meshes/source' / geometry.find('geometry/mesh').get('filename').removeprefix(PACKAGE)
            key = name if i == 0 else f'{name}_{i}'
            visual_records[key] = visual_mesh(source_mesh, key, asset)
            pos, r = origin(geometry)
            ET.SubElement(element, 'geom', name=key + '_visual', type='mesh', mesh=key + '_visual_mesh', pos=numbers(pos), quat=quaternion(r), material=key + '_visual_material', contype='0', conaffinity='0', group='1')
        for i, geometry in enumerate(link.findall('collision')):
            source_mesh = ROOT / 'meshes/source' / geometry.find('geometry/mesh').get('filename').removeprefix(PACKAGE)
            key = name if i == 0 else f'{name}_{i}'
            collision_records[key] = collision_mesh(source_mesh, key, asset)
            pos, r = origin(geometry)
            for part in collision_records[key]['parts']:
                ET.SubElement(element, 'geom', name=part['mesh'], type='mesh', mesh=part['mesh'], pos=numbers(pos), quat=quaternion(r), contype='1', conaffinity='1', group='0', rgba='0.15 0.65 0.35 0.25')
        for child, child_joint in joints.items():
            if child_joint.find('parent').get('link') == name:
                body(child, element)

    for child, joint in joints.items():
        if joint.find('parent').get('link') == 'world':
            body(child, world)
    equalities = ET.SubElement(root, 'equality')
    ET.SubElement(equalities, 'joint', name='source_gripper_mimic', joint1='gripper_joint2', joint2='gripper_joint1', polycoef='0 -1 0 0 0')
    ET.SubElement(ET.SubElement(root, 'keyframe'), 'key', name='static_reference', qpos='0 -0.3 0 0.5 0 0 0 0.04 -0.04')
    write_xml(root, ROOT / 'mjcf/nero_gripper.xml')
    write_json(ROOT / 'reports/mesh-conversion.json', dict(visuals=visual_records, collisions=collision_records, source_surface_faces_covered=True, explicit_collision_excludes=0, collision_approximation='convex hull per exact source component; contained planar decorations retained in containing hull'))
    joint_order = [f'joint{i}' for i in range(1, 8)] + ['gripper_joint1', 'gripper_joint2']
    write_json(ROOT / 'docs/robot-contract.json', dict(physical_state_dim=9, physical_joint_order=joint_order, units=['rad'] * 7 + ['m'] * 2, virtual_source_joint='gripper', virtual_source_range_m=[0,.1], source_mimic={'gripper_joint1':'.5 * gripper', 'gripper_joint2':'-.5 * gripper'}, mjcf_constraint='gripper_joint2 = -gripper_joint1', virtual_joint_recovery='gripper = 2 * gripper_joint1', actuators=0, controller_implemented=False, policy_adapter_implemented=False, proposed_policy_state='eef XYZ(3), axis-angle(3), finger positions(2): 8D', proposed_policy_action='eef translation(3), rotation(3), gripper open/close(1): 7D; requires controller and unit/frame verification', limits=[dict(name=j.get('name'), type=j.get('type'), **j.find('limit').attrib) for j in source.findall('joint') if j.find('limit') is not None]))
    print('Built source-preserving URDF and static MJCF')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-repo', type=Path, help='initial import from the exact pinned Git commit')
    args = parser.parse_args()
    if args.source_repo:
        snapshot(args.source_repo)
    build()
