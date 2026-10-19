"""Issue16: immutable contacts + kinematics/static polytope geometry; no dynamics.

SAT and nearest triangles are independent geometric diagnostics, NOT replacements
for the recorded MuJoCo contact distances or the frozen 1 mm acceptance test.
"""
import argparse
from collections import Counter
import hashlib
from itertools import product
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial.transform import Rotation
import trimesh


def local_point(p, origin, R):
    return (np.asarray(p) - origin) @ R


def world_point(p, origin, R):
    return np.asarray(p) @ R.T + origin


def angle_deg(a, b):
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b) /
                         (np.linalg.norm(a)*np.linalg.norm(b)), -1., 1.))))


def nearest_surface(triangles, p):
    pts = trimesh.triangles.closest_point(triangles, np.tile(p, (len(triangles), 1)))
    distances = np.linalg.norm(pts-p, axis=1)
    ids = np.flatnonzero(distances <= distances.min()+1e-10)
    return dict(distance_m=float(distances.min()), triangle_indices=ids.tolist(),
                nearest_points_m=pts[ids].tolist())


def sat_box_hull(vertices, hull, box_rotation, half_size, box_center):
    """Minimum translation depth for convex polytope and oriented box.

    Inputs share one frame. All face normals and edge cross products are used.
    Positive = intersecting; negative = separating-axis gap (not Euclidean gap).
    """
    edges = np.unique(np.sort(np.concatenate([hull.simplices[:, [0,1]],
                          hull.simplices[:, [1,2]], hull.simplices[:, [2,0]]]), axis=1), axis=0)
    directions = vertices[edges[:,1]] - vertices[edges[:,0]]
    axes = np.concatenate([hull.equations[:,:3], box_rotation.T,
                          np.cross(directions[:,None,:], box_rotation.T[None,:,:]).reshape(-1,3)])
    length = np.linalg.norm(axes, axis=1)
    axes = axes[length > 1e-12] / length[length > 1e-12,None]
    projection = vertices @ axes.T
    center = box_center @ axes.T
    radius = np.abs(axes @ box_rotation) @ half_size
    positive = projection.max(axis=0) - (center-radius)
    negative = center+radius - projection.min(axis=0)
    depths = np.minimum(positive, negative)
    idx = int(np.argmin(depths))
    axis = axes[idx] * (1. if positive[idx] <= negative[idx] else -1.)
    return dict(depth_m=float(depths[idx]), separating_translation_axis=axis.tolist(),
                axes_evaluated=len(axes), method='all face normals and edge cross box-axis SAT')


def read_obj(path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith('v '): vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            ids = [int(x.split('/')[0])-1 for x in line.split()[1:]]
            if min(ids) < 0: raise ValueError('Expected frozen positive OBJ indices')
            faces.extend([[ids[0], ids[k], ids[k+1]] for k in range(1, len(ids)-1)])
    return np.array(vertices), np.array(faces, dtype=int)


def rot(quat):
    return Rotation.from_quat(np.asarray(quat)[[1,2,3,0]]).as_matrix()


def seal_inputs(source, campaign):
    """Verify original seals before computation, then rehash the same paths after."""
    manifest_path = source/'reports/issue14-evidence-manifest.json'
    manifest = json.loads(manifest_path.read_text())
    prepared_path = campaign/'prepared.json'
    expected = {str(prepared_path): manifest['prepared_sha256']}
    for label in ['R0', 'R1']:
        expected.update({p: entry['sha256'] for p, entry in manifest['raw_files'][label].items()})
    prepared = json.loads(prepared_path.read_text())
    expected.update(prepared['asset_hashes'])
    expected.update({str(campaign/p): h for p,h in prepared['artifacts'].items()})
    expected[str(manifest_path)] = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    for p, digest in expected.items():
        if hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest:
            raise ValueError('BLOCKED frozen SHA mismatch: '+p)
    return expected


def xml_body_fk(root, qpos):
    """Frozen MJCF, one saved qpos -> {body_name: (world_position, world_R)}.

    Pure NumPy rigid transforms; XML angle=radian, body quat=wxyz.
    Supports this model's hinge/slide/free joints, joint ref and anchor.
    Mesh vertices, contacts and dynamics are never evaluated.
    """
    import numpy as np

    def vec(node, key, default):
        return np.fromstring(node.get(key, default), sep=' ')

    def rotation(quat):
        w, x, y, z = quat / np.linalg.norm(quat)
        return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

    q = np.asarray(qpos, dtype=float)
    if q.ndim != 1 or not np.isfinite(q).all():
        raise ValueError('Expected one finite saved qpos vector')
    if root.find('compiler').get('angle', 'degree') != 'radian':
        raise ValueError('This frozen-model FK expects angle=radian')
    result = {'world': (np.zeros(3), np.eye(3))}
    qadr = 0

    def visit(body, parent_p, parent_R, depth):
        nonlocal qadr
        p = parent_p + parent_R @ vec(body, 'pos', '0 0 0')
        R = parent_R @ rotation(vec(body, 'quat', '1 0 0 0'))
        for joint in (j for j in body if j.tag in ('joint', 'freejoint')):
            kind = joint.get('type', 'free' if joint.tag == 'freejoint' else 'hinge')
            width = 7 if kind == 'free' else 1
            state = q[qadr:qadr+width]; qadr += width
            if kind == 'free':
                if depth != 0: raise ValueError('Free joint must be a world child')
                p, R = state[:3].copy(), rotation(state[3:7])
                continue
            axis = vec(joint, 'axis', '0 0 1'); axis /= np.linalg.norm(axis)
            displacement = state[0] - float(joint.get('ref', '0'))
            if kind == 'slide':
                p = p + R @ (axis * displacement)
            elif kind == 'hinge':
                J = rotation(np.r_[np.cos(displacement/2), axis*np.sin(displacement/2)])
                anchor = vec(joint, 'pos', '0 0 0')
                p, R = p + R @ (anchor - J @ anchor), R @ J
            else:
                raise ValueError('Joint type outside frozen-model scope: ' + kind)
        if body.get('name'): result[body.get('name')] = (p.copy(), R.copy())
        for child in body.findall('body'): visit(child, p, R, depth+1)

    for body in root.find('worldbody').findall('body'):
        visit(body, *result['world'], 0)
    if qadr != len(q): raise ValueError('XML joints / qpos size disagree')
    return result


def guarded_mujoco():
    import mujoco
    allowed = {'mj_kinematics', 'mj_comPos', 'mj_jac', 'mj_name2id', 'mj_id2name'}
    calls = Counter()
    def wrap(name, f):
        def guarded(*args, **kwargs):
            if name not in allowed:
                raise RuntimeError('Forbidden in offline geometry: '+name)
            calls[name] += 1
            return f(*args, **kwargs)
        return guarded
    for name in dir(mujoco):
        if name.startswith('mj_') and callable(getattr(mujoco, name)):
            setattr(mujoco, name, wrap(name, getattr(mujoco, name)))
    return mujoco, calls


def analyze(source, campaign, output):
    if output == source or source in output.parents or output == campaign or campaign in output.parents:
        raise ValueError('Output must be outside the immutable source/Run')
    output.mkdir(parents=True, exist_ok=False)
    seals = seal_inputs(source, campaign)
    if (campaign/'R0/model.xml').read_bytes() != (campaign/'R1/model.xml').read_bytes():
        raise ValueError('BLOCKED: R0/R1 static models differ')
    mj, calls = guarded_mujoco()
    cases, static = {}, {}
    # Compilation reads the immutable XML and its existing assets, without data
    # propagation or a contact/constraint solve. Data receives archived qpos/qvel.
    model = mj.MjModel.from_xml_path(str(campaign/'R1/model.xml'))
    data = mj.MjData(model)
    xml_root = ET.parse(campaign/'R1/model.xml').getroot()
    gid = lambda s: mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, s)
    bid = lambda s: mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, s)
    gripper, cube = bid('gripper_base'), bid('cube')
    cube_gid = gid('cube_geom')
    assets = {x.get('name'): x for x in ET.parse(campaign/'R1/model.xml').findall('./asset/mesh')}
    for name in ['gripper_link1_collision_000', 'gripper_link2_collision_000']:
        g = gid(name); mid = model.geom_dataid[g]
        path = (campaign/'R1'/assets[name].get('file')).resolve()
        v, faces = read_obj(path); h = ConvexHull(v)
        off, num = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        compiled_body = world_point(model.mesh_vert[off:off+num], model.geom_pos[g], rot(model.geom_quat[g]))
        source_body = v  # Frozen geom pos=0, quat=identity, mesh scale=1/refpos=0.
        xml_geom = ET.parse(campaign/'R1/model.xml').find(".//geom[@name='"+name+"']")
        assert xml_geom.get('pos') == '0 0 0' and xml_geom.get('quat') == '1 0 0 0'
        assert assets[name].get('scale', '1 1 1') == '1 1 1'
        assert assets[name].get('refpos', '0 0 0') == '0 0 0'
        assert assets[name].get('refquat', '1 0 0 0') == '1 0 0 0'
        deviation = max(cKDTree(compiled_body).query(source_body)[0].max(),
                        cKDTree(source_body).query(compiled_body)[0].max())
        if deviation > 3e-8: raise ValueError('Source/compiled mesh transform mismatch')
        static[name] = dict(geom_id=int(g), body_id=int(model.geom_bodyid[g]),
            geom_type='mesh', mesh_name=name, source_path=str(path), sha256=seals[str(path)],
            source_vertices=len(v), source_triangles=len(faces), convex_hull_triangles=len(h.simplices),
            source_compiled_max_deviation_m=float(deviation),
            source_bounds_m=[v.min(axis=0).tolist(),v.max(axis=0).tolist()],
            compiled_mesh_pos=model.mesh_pos[mid].tolist(), compiled_mesh_quat=model.mesh_quat[mid].tolist(),
            _vertices=v, _faces=faces, _hull=h)
    all_sanity = Counter()
    for label, key_rows in [('R1', [23622,23623,23624]), ('R0', [33317,33318,33319])]:
        episode = campaign/label
        with np.load(episode/'trajectory.npz', allow_pickle=False) as z:
            a = {k:z[k] for k in ['time','phase','qpos','qvel']}
        with np.load(episode/'observations.npz', allow_pickle=False) as z:
            obs = {k:z[k] for k in z.files}
        rows = [json.loads(line) for line in (episode/'contacts.jsonl').open()]
        over = [i for i,r in enumerate(rows) if any('cube' in e['bodies'] and 'world' not in e['bodies']
                    and e['distance_m'] < -.001 for e in r['events'])]
        selected = sorted(set(key_rows) | {j for i in over for j in [i-1,i,i+1] if 0<=j<len(rows)})
        detailed = []
        for i in selected:
            data.qpos[:] = a['qpos'][i]; data.qvel[:] = a['qvel'][i]
            mj.mj_kinematics(model,data); mj.mj_comPos(model,data)
            gp, gR = data.xpos[gripper].copy(), data.xmat[gripper].reshape(3,3).copy()
            cp, cR = data.xpos[cube].copy(), data.xmat[cube].reshape(3,3).copy()
            independent_fk = xml_body_fk(xml_root,a['qpos'][i])
            for body in ['gripper_base','cube','gripper_link1','gripper_link2']:
                body_id=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,body)
                p_fk,R_fk=independent_fk[body]
                np.testing.assert_allclose(p_fk,data.xpos[body_id],rtol=0,atol=1e-12)
                np.testing.assert_allclose(R_fk,data.xmat[body_id].reshape(3,3),rtol=0,atol=1e-12)
                all_sanity['independent_xml_body_poses_checked']+=1
            checks = dict(gripper_position=float(np.max(abs(gp-obs['gripper_position_world_m'][i]))),
                gripper_rotation=float(np.max(abs(gR-obs['gripper_rotation_world'][i]))),
                cube_relative_position=float(np.max(abs(gR.T@(cp-gp)-obs['cube_position_in_gripper_m'][i]))),
                cube_relative_rotation=float(np.max(abs(gR.T@cR-obs['cube_rotation_in_gripper'][i]))))
            if max(checks.values()) > 1e-12: raise ValueError('Archived pose does not match kinematics')
            contact_details = []
            for j, e in enumerate(rows[i]['events']):
                names = [s for s in e['geoms'] if s in static]
                if 'cube' not in e['bodies'] or not names: continue
                name = names[0]; s = static[name]; v,f,h = s['_vertices'],s['_faces'],s['_hull']
                finger = s['body_id']; fp = data.xpos[finger].copy(); fR = data.xmat[finger].reshape(3,3).copy()
                p, n = np.array(e['position_m']), np.array(e['normal']); F=np.array(e['contact_frame']).reshape(3,3)
                np.testing.assert_allclose(F@F.T, np.eye(3), rtol=0, atol=1e-12)
                np.testing.assert_allclose(F[0], n, rtol=0, atol=0)
                if np.linalg.det(F) < .999999999: raise ValueError('Improper contact frame')
                cube_index=e['bodies'].index('cube'); on_cube=n*(1 if cube_index==1 else -1)
                pf,pc=local_point(p,fp,fR), local_point(p,cp,cR)
                np.testing.assert_allclose(world_point(pf,fp,fR), p, rtol=0,atol=1e-14)
                qjac=[]
                for body in [cube,finger]:
                    jacp=np.zeros((3,model.nv)); jacr=np.zeros_like(jacp)
                    mj.mj_jac(model,data,jacp,jacr,p,body); qjac.append(jacp@data.qvel)
                rel=qjac[0]-qjac[1]; tangent=rel-n*np.dot(rel,n)
                near=nearest_surface(v[f],pf)
                hull_normals=h.equations[:,:3]
                expected_finger_normal=fR.T@on_cube
                normal_angles=np.degrees(np.arccos(np.clip(hull_normals@expected_finger_normal,-1,1)))
                closest_plane=int(np.argmin(normal_angles))
                plane_dist=hull_normals@pf+h.equations[:,3]
                cr=fR.T@cR; cc=fR.T@(cp-fp)
                sat=sat_box_hull(v,h,cr,model.geom_size[cube_gid],cc)
                near['candidate_outward_normals_finger'] = []
                for tri in near['triangle_indices']:
                    xyz=v[f[tri]]; normal=np.cross(xyz[1]-xyz[0],xyz[2]-xyz[0]);normal/=np.linalg.norm(normal)
                    if np.dot(normal,xyz.mean(axis=0)-v[h.vertices].mean(axis=0))<0:normal=-normal
                    near['candidate_outward_normals_finger'].append(normal.tolist())
                contact_details.append(dict(event_index=j, archived=e,
                    geom_pair=e['geoms'],body_pair=e['bodies'], geom_types=['box' if x=='cube_geom' else 'mesh' for x in e['geoms']],
                    mesh=name, contact_point_cube_local_m=pc.tolist(), contact_point_finger_local_m=pf.tolist(),
                    normal_on_cube_world=on_cube.tolist(), normal_on_cube_cube_local=(cR.T@on_cube).tolist(),
                    normal_on_cube_finger_local=expected_finger_normal.tolist(),
                    force_on_cube_world_N=((1 if cube_index==1 else -1)*F.T@np.array(e['force_contact_frame'][:3])).tolist(),
                    material_velocity_cube_world_m_s=qjac[0].tolist(),material_velocity_finger_world_m_s=qjac[1].tolist(),
                    relative_velocity_cube_minus_finger_world_m_s=rel.tolist(),
                    relative_tangent_speed_m_s=float(np.linalg.norm(tangent)),
                    relative_velocity_dot_archived_normal_m_s=float(np.dot(rel,n)),
                    relative_velocity_dot_normal_on_cube_m_s=float(np.dot(rel,on_cube)),
                    finger_position_world_m=fp.tolist(),finger_rotation_world=fR.tolist(),
                    nearest_source_triangles=near,
                    nearest_hull_face_normal_angle_deg=float(normal_angles.min()),
                    nearest_normal_hull_triangle=int(closest_plane),
                    nearest_normal_hull_triangle_vertices_m=v[h.simplices[closest_plane]].tolist(),
                    nearest_normal_hull_plane_contact_signed_distance_m=float(plane_dist[closest_plane]),
                    maximum_hull_plane_contact_signed_distance_m=float(plane_dist.max()),
                    sat=sat,actual_collision_feature_id='UNKNOWN: not archived'))
                all_sanity['contact_frames_checked']+=1
            detailed.append(dict(row=i,contacts_jsonl_line=i+1,physics_step=i+1,time_s=float(a['time'][i]),
                phase=str(a['phase'][i]),recorded_total_contacts=len(rows[i]['events']),key_frame=i in key_rows,
                cube_position_world_m=cp.tolist(),cube_rotation_world=cR.tolist(),
                gripper_position_world_m=gp.tolist(),gripper_rotation_world=gR.tolist(),
                pose_sanity_max_abs_errors=checks,contacts=contact_details))
        # Sampled contact locations are not material trajectories. Give both their
        # finite differences and the Jacobian-based material relative velocity.
        keys=[d for d in detailed if d['key_frame']]
        deltas=[]
        for prev,now in zip(keys,keys[1:]):
            by_mesh={e['mesh']:e for e in prev['contacts']}
            for e in now['contacts']:
                if e['mesh'] not in by_mesh:continue
                old=by_mesh[e['mesh']];R0=np.array(prev['cube_rotation_world']);R1=np.array(now['cube_rotation_world'])
                F0=np.array(old['finger_rotation_world']);F1=np.array(e['finger_rotation_world'])
                deltas.append(dict(rows=[prev['row'],now['row']],mesh=e['mesh'],
                    normal_angle_deg=angle_deg(old['normal_on_cube_world'],e['normal_on_cube_world']),
                    contact_point_world_delta_m=(np.array(e['archived']['position_m'])-old['archived']['position_m']).tolist(),
                    cube_translation_m=float(np.linalg.norm(np.array(now['cube_position_world_m'])-prev['cube_position_world_m'])),
                    cube_rotation_deg=float(np.degrees(Rotation.from_matrix(R1@R0.T).magnitude())),
                    finger_translation_m=float(np.linalg.norm(np.array(e['finger_position_world_m'])-old['finger_position_world_m'])),
                    finger_rotation_deg=float(np.degrees(Rotation.from_matrix(F1@F0.T).magnitude()))))
        cases[label]=dict(total_rows=len(rows),exceedance_rows=over,geometric_rows_analyzed=len(selected),
                          key_frame_deltas=deltas,records=detailed)
        np.testing.assert_allclose(obs['time'],a['time'],rtol=0,atol=0)
    for path,digest in seals.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:raise ValueError('Input changed: '+path)
    clean_static={name:{k:v for k,v in s.items() if not k.startswith('_')} for name,s in static.items()}
    result=dict(analysis_status='PASS',physical_grasp='FAIL',root_cause='UNKNOWN',
        mujoco_version=mj.__version__,new_mj_steps=0,new_force_solves=0,
        allowed_mujoco_calls=dict(calls),sanity_checks=dict(all_sanity),
        original_source='1bd91cc4c3fc7723eab94881ceb7c4cc3e655316',
        frozen_physics='1e4d633730f62a398513d33991fa96f9c4977daa',
        input_sha256=seals,input_hashes_unchanged_after=True,static_meshes=clean_static,
        collision_configuration=dict(enableflags=int(model.opt.enableflags),disableflags=int(model.opt.disableflags),
             ccd_tolerance=float(model.opt.ccd_tolerance),ccd_iterations=int(model.opt.ccd_iterations),
             nativeccd=bool(model.opt.enableflags & int(mj.mjtEnableBit.mjENBL_NATIVECCD)),
             multiccd=bool(model.opt.enableflags & int(mj.mjtEnableBit.mjENBL_MULTICCD))),episodes=cases,
        analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        read_operations=['read source manifest/prepared and sealed raw/assets; SHA before/after',
            'compile immutable R1/model.xml and its referenced existing assets once',
            'load archived trajectory/observations/contacts for R0 and R1',
            'copy archived qpos/qvel into separate scratch MjData; kinematics/comPos/Jacobian only',
            'read compiled body/geom/mesh arrays; independent OBJ triangles/Qhull/SAT; no contact regeneration'],
        caveats=['No collision face/edge/vertex IDs archived; geometric candidates are not actual selected IDs',
                 'Saved forces are post-step forward samples, not integration impulses',
                 'SAT does not replace saved distances or the original 1mm criterion',
                 'Only neighboring/over-limit rows plus specified keys receive geometric analysis'])
    (output/'analysis.executed.py').write_bytes(Path(__file__).read_bytes())
    (output/'geometry.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(status='PASS',new_mj_steps=0,new_force_solves=0,sha_count=len(seals),
        frames={k:v['geometric_rows_analyzed'] for k,v in cases.items()},allowed_calls=dict(calls))))


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source-root',type=Path,required=True)
    ap.add_argument('--campaign',type=Path,required=True)
    ap.add_argument('--output-dir',type=Path,required=True)
    args=ap.parse_args();analyze(args.source_root.resolve(),args.campaign.resolve(),args.output_dir.resolve())
