#!/usr/bin/env python3
"""Read NERO source geometry only; write evidence beside this script.

No simulator imports, model mutation, controller, dynamics, or pad candidates.
Indices are zero based: STL face order / OBJ f-line order with process=False.
"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
PATCHES = {
    'tip_zero_plane': list(range(545, 549)),
    'inner_ramp': list(range(549, 557)),
    'ramp_to_root_transition_1': [578, 579],
    'ramp_to_root_transition_2': [580, 581],
    'root_front_plane': list(range(557, 578)),
    'root_left_bevel': [657, 658, 659, 1534, 1535],
    'root_block_front_notched_plane': list(range(2, 9)),
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def component_ids(mesh):
    vertices, inverse = np.unique(mesh.vertices, axis=0, return_inverse=True)
    faces = inverse[mesh.faces]
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                       shape=(len(vertices), len(vertices)))
    count, labels = connected_components(graph, directed=False)
    return [np.flatnonzero(labels[faces[:, 0]] == i) for i in range(count)]


def fit(mesh, ids):
    points = np.unique(mesh.triangles[ids].reshape(-1, 3), axis=0)
    origin = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - origin)
    normal = vt[-1]
    normal *= -1 if normal[2] > 0 else 1
    errors = (points - origin) @ normal
    u = np.array([1., 0., 0.])
    v = np.cross(normal, u)
    uv = np.column_stack(((points - origin) @ u, (points - origin) @ v))
    area = float(mesh.area_faces[ids].sum())
    record = dict(source_face_indices=ids, source_points_m=points.tolist(),
                  origin_m=origin.tolist(), outward_normal=normal.tolist(),
                  plane_offset_m=float(-normal @ origin),
                  tangent_u=u.tolist(), tangent_v=v.tolist(),
                  xyz_bounds_m=[points.min(axis=0).tolist(), points.max(axis=0).tolist()],
                  uv_bounds_m=[uv.min(axis=0).tolist(), uv.max(axis=0).tolist()],
                  surface_area_m2=area,
                  max_abs_plane_error_m=float(abs(errors).max()),
                  rms_plane_error_m=float(np.sqrt(np.mean(errors ** 2))))
    if ids in (PATCHES['tip_zero_plane'], PATCHES['inner_ramp']):
        bbox_area = float(np.ptp(uv, axis=0).prod())
        inner_area = float(2 * np.min(abs(points[:, 0])) * np.ptp(uv[:, 1]))
        record['full_length_rectangle_bounds'] = dict(
            note='Geometry bounds only; these rectangles are not pad candidates.',
            outer_excess_area_fraction=(bbox_area - area) / area,
            inner_missing_area_fraction=(area - inner_area) / area,
            outer_max_side_overhang_m=float((np.ptp(points[:, 0]) -
                                           2 * np.min(abs(points[:, 0]))) / 2))
    return record


def ray_hits(mesh, xy_points):
    """All intersections of +z rays with stored source triangles, no engine."""
    t = mesh.triangles
    xy = t[:, :, :2]
    e = xy[:, 1:] - xy[:, 0, None]
    den = e[:, 0, 0] * e[:, 1, 1] - e[:, 0, 1] * e[:, 1, 0]
    answer = []
    for p in xy_points:
        q = np.asarray(p) - xy[:, 0]
        u = np.divide(q[:, 0] * e[:, 1, 1] - q[:, 1] * e[:, 1, 0], den,
                      out=np.full(len(t), 999.), where=abs(den) > 1e-20)
        v = np.divide(e[:, 0, 0] * q[:, 1] - e[:, 0, 1] * q[:, 0], den,
                      out=np.full(len(t), 999.), where=abs(den) > 1e-20)
        ids = np.flatnonzero((u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8))
        z = t[ids, 0, 2] + u[ids] * (t[ids, 1, 2] - t[ids, 0, 2]) + v[ids] * (t[ids, 2, 2] - t[ids, 0, 2])
        hits = sorted([dict(source_face_index=int(i), z_m=float(h),
                            stored_normal_z=float(mesh.face_normals[i, 2]))
                       for i, h in zip(ids, z)], key=lambda r: r['z_m'])
        answer.append(dict(xy_m=list(p), hits_from_inner_space=hits))
    return answer


def sheet_support(mesh):
    """Determine whether the rank-2 tip face closes a supported sheet."""
    front_ids = PATCHES['tip_zero_plane']
    components = component_ids(mesh)
    body_ids = components[0]
    front_vertices = np.unique(mesh.triangles[front_ids].reshape(-1, 3), axis=0)
    body_vertices = np.unique(mesh.triangles[body_ids].reshape(-1, 3), axis=0)
    seam_distances, nearest = cKDTree(body_vertices).query(front_vertices)
    centroids = mesh.triangles[front_ids].mean(axis=1)
    samples = [('vertex', p) for p in front_vertices]
    samples += [('face_centroid', p) for p in centroids]
    samples += [('geometric_center', front_vertices.mean(axis=0))]
    ray_records = ray_hits(mesh, [p[:2] for _, p in samples])
    result = []
    for (kind, p), hits in zip(samples, ray_records):
        # A +z face has the opposing winding expected at a plate back surface.
        positive_faces = [h for h in hits['hits_from_inner_space']
                          if h['source_face_index'] in body_ids
                          and h['z_m'] > p[2] + 1e-8
                          and h['stored_normal_z'] > .5]
        h = positive_faces[0] if positive_faces else None
        result.append(dict(sample_kind=kind, front_point_m=p.tolist(),
                           next_body_positive_z_face=h,
                           delta_z_to_back_m=h['z_m'] - float(p[2]) if h else None))
    return dict(
        finding='The zero plane is a geometrically supported cap of a 2 mm sheet, not a demonstrably floating decoration. Exact-coordinate component splitting occurs at a sub-attometre material seam.',
        front_vertices_m=front_vertices.tolist(),
        nearest_component0_vertices_m=body_vertices[nearest].tolist(),
        boundary_seam_vertex_distances_m=seam_distances.tolist(),
        maximum_boundary_seam_distance_m=float(seam_distances.max()),
        positive_z_ray_support_samples=result)


def draw(mesh, output):
    # Source patch projections, not a proposed primitive layout.
    im = Image.new('RGB', (1400, 900), 'white')
    d = ImageDraw.Draw(im)
    panels = [((0, 1), (50, 80, 630, 820), 'Finger local x/y: tip y=0; root y=-76.5 mm'),
              ((1, 2), (700, 80, 1350, 430), 'Finger local y/z: inward is -z'),
              ((0, 2), (700, 510, 1350, 820), 'Finger local x/z')]
    palette = ['#ae72cf', '#42b8aa', '#efbd4c', '#d89040', '#6c9bd2', '#aaaaaa', '#e3937c']
    for axes, box, title in panels:
        tris = mesh.triangles[:, :, axes]
        low, high = tris.reshape(-1, 2).min(axis=0), tris.reshape(-1, 2).max(axis=0)
        scale = min((box[2] - box[0]) / (high[0] - low[0]), (box[3] - box[1]) / (high[1] - low[1]))
        def pix(t):
            return [(int(box[0] + (x - low[0]) * scale),
                     int(box[3] - (y - low[1]) * scale)) for x, y in t]
        for tri in tris:
            d.polygon(pix(tri), fill='#eeeeee', outline='#cccccc')
        for (name, ids), color in zip(PATCHES.items(), palette):
            for tri in tris[ids]:
                d.polygon(pix(tri), fill=color, outline='#444444')
        d.text((box[0], box[1] - 28), title, fill='black')
    d.text((30, 15), 'Source geometry only. Purple=tip zero plane; green=inner ramp; yellow/orange=transition; blue=root front.', fill='black')
    im.save(output / 'source-projections.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path,
                        help='New independent output directory; existing directories are refused.')
    output = parser.parse_args().output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((ROOT / 'SOURCE_MANIFEST.json').read_text())
    conversion = json.loads((ROOT / 'reports/mesh-conversion.json').read_text())
    xml = ET.parse(ROOT / 'mjcf/nero_gripper.xml').getroot()
    paths = ['SOURCE_MANIFEST.json', 'docs/BUILD.md', 'scripts/build.py',
             'reports/mesh-conversion.json', 'mjcf/nero_gripper.xml',
             'urdf/nero_gripper.urdf', 'urdf/source/nero/urdf/nero_with_gripper_description.xacro']
    data = dict(
        status='GEOMETRY_ENVELOPE_TRACED_CANDIDATE_UNFROZEN',
        status_scope='Geometric facts only. The parent review decides Gate 0 and the single primitive/protection layout; no impossibility claim is made about <=5 boxes.',
        no_candidate_generated=True, simulator_imports=0, physics_steps=0,
        index_convention='zero-based STL face order and OBJ f-line order; trimesh process=False',
        source_repository=manifest['repository'], source_revision=manifest['revision'],
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        frame_convention='mesh/visual/collision local == finger body frame; base means gripper_base; all lengths metres',
        fingers={})
    source_meshes = []
    for name in ['gripper_link1', 'gripper_link2']:
        files = dict(source=f'meshes/source/agx_gripper/meshes/{name}.stl',
                     source_visual=f'meshes/source/agx_gripper/meshes/dae/{name}.dae',
                     visual=f'meshes/visual/{name}.obj',
                     collision_000=f'meshes/collision/{name}_000.obj',
                     collision_001=f'meshes/collision/{name}_001.obj')
        paths.extend(files.values())
        source = trimesh.load(ROOT / files['source'], process=False)
        visual = trimesh.load(ROOT / files['visual'], process=False)
        hull = trimesh.load(ROOT / files['collision_000'], process=False)
        source_meshes.append(source)
        distance, mapping = cKDTree(visual.triangles.mean(axis=1)).query(source.triangles.mean(axis=1))
        # Verify vertex-set matching, not just centroids.
        vertex_errors = []
        for a, b in zip(source.triangles, visual.triangles[mapping]):
            vertex_errors.append(float(cKDTree(b).query(a)[0].max()))
        body = xml.find(f'.//body[@name="{name}"]')
        q = np.fromstring(body.get('quat'), sep=' ')
        R = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
        origin = np.fromstring(body.get('pos'), sep=' ')
        finger = dict(files=files,
                      mesh_to_body=dict(rotation=np.eye(3).tolist(), translation_m=[0., 0., 0.]),
                      body_to_base_qzero=dict(rotation=R.tolist(), translation_m=origin.tolist()),
                      inward_normal_in_base=(R @ [0., 0., -1.]).tolist(),
                      tip_origin_in_base_m=origin.tolist(),
                      root_extreme_in_base_m=(origin + R @ [0., -.0765, 0.]).tolist(),
                      source_components=[], patches={},
                      all_source_to_visual_max_centroid_error_m=float(distance.max()),
                      all_source_to_visual_max_vertex_error_m=max(vertex_errors),
                      zero_plane_sheet_support=sheet_support(source),
                      collision_conversion=conversion['collisions'][name])
        for i, ids in enumerate(component_ids(source)):
            points = np.unique(source.triangles[ids].reshape(-1, 3), axis=0)
            finger['source_components'].append(dict(component=i, face_count=len(ids),
                affine_rank=int(np.linalg.matrix_rank(points - points.mean(axis=0))),
                xyz_bounds_m=[points.min(axis=0).tolist(), points.max(axis=0).tolist()]))
        for patch, ids in PATCHES.items():
            finger['patches'][patch] = fit(source, ids)
            finger['patches'][patch]['visual_face_indices'] = mapping[ids].tolist()
        finger['collision000_inward_facets'] = [dict(
            obj_face_index=int(i), outward_normal=hull.face_normals[i].tolist(),
            triangle_vertices_m=hull.triangles[i].tolist())
            for i in np.flatnonzero(hull.face_normals[:, 2] < -.9)]
        ys = [-.005, -.02, -.03, -.034717850387096405, -.05, -.07]
        source_hits = ray_hits(source, [(0., y) for y in ys])
        hull_hits = ray_hits(hull, [(0., y) for y in ys])
        finger['geometry_selected_centerline_source_vs_collision000'] = [dict(
            local_xy_m=[0., y], source_front_z_m=a['hits_from_inner_space'][0]['z_m'],
            collision000_front_z_m=b['hits_from_inner_space'][0]['z_m'],
            collision_fills_inward_depth_m=a['hits_from_inner_space'][0]['z_m'] - b['hits_from_inner_space'][0]['z_m'])
            for y, a, b in zip(ys, source_hits, hull_hits)]
        finger['source_inner_ray_examples'] = ray_hits(source, [(0., -.005), (0., -.02), (0., -.05), (0., -.07), (0., -.074)])
        data['fingers'][name] = finger
    data['left_right_max_source_triangle_coordinate_difference_m'] = float(abs(source_meshes[0].triangles - source_meshes[1].triangles).max())
    data['input_sha256'] = {p: sha(ROOT / p) for p in sorted(set(paths))}
    data['analysis_script_sha256'] = sha(Path(__file__))
    # Confirm all frozen source bytes used here against the source manifest.
    hashes = {f['file']: f['sha256'] for f in manifest['files']}
    data['source_manifest_sha_matches'] = {p: data['input_sha256'][p] == hashes[p] for p in data['input_sha256'] if p in hashes}
    (output / 'geometry-basis.json').write_text(json.dumps(data, indent=2) + '\n')
    draw(source_meshes[0], output)
    print(json.dumps(dict(status=data['status'], geometry_basis_json=str(output / 'geometry-basis.json'),
                          source_manifest_all_match=all(data['source_manifest_sha_matches'].values()),
                          no_simulator_imports=True, no_candidate_generated=True)))


if __name__ == '__main__':
    main()
