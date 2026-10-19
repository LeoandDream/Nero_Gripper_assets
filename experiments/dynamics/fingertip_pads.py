"""Issue23: one source-inscribed two-pad candidate, then bounded static checks.

No integration/controller/force entry point. A critical static unknown stops the
campaign permanently; this module never runs an Episode or changes its pads.
"""
import argparse
import copy
from datetime import datetime, timezone
import difflib
import hashlib
import importlib.metadata
import importlib.util
from itertools import product
import json
import shutil
from pathlib import Path
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh

ROOT = Path(__file__).resolve().parents[2]
BASE = 'd4a37dafb9a7b9cb3392a8937590dd091a5e4a7a'
HISTORY = Path('/data_robot/projects/.worktrees/nero-gripper-full-grasp/experiments/dynamics/runs/issue14_full_grasp_002')
PATCHES = ('tip_zero_plane', 'inner_ramp')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): h.update(b)
    return h.hexdigest()


def write(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False); f.write('\n')


def hull_points(points):
    points = np.unique(np.asarray(points).reshape(-1, 3), axis=0)
    if len(points) < 4 or np.linalg.matrix_rank(points-points.mean(axis=0), tol=1e-13) < 3:
        return np.empty((0, 3))
    return points[ConvexHull(points).vertices]


def clip(vertices, a, b):
    """Intersect one convex solid with a.x<=b; no offsets added to the plane."""
    if len(vertices) == 0: return vertices
    d = vertices @ a - b
    if d.max() <= 1e-13: return vertices.copy()
    if d.min() >= -1e-13: return np.empty((0, 3))
    h = ConvexHull(vertices)
    edges = np.unique(np.sort(np.concatenate([h.simplices[:, [0,1]],
                    h.simplices[:, [1,2]], h.simplices[:, [2,0]]]), axis=1), axis=0)
    points = list(vertices[d <= 0])
    for i, j in edges:
        if d[i]*d[j] < 0: points.append(vertices[i]+d[i]/(d[i]-d[j])*(vertices[j]-vertices[i]))
    return hull_points(points)


def subtract_prism(vertices, planes):
    """Disjoint convex pieces of H minus intersection(a.x<=b for planes)."""
    pieces = []; remaining = vertices
    for a, b in planes:
        outside = clip(remaining, -a, -b)
        if len(outside): pieces.append(outside)
        remaining = clip(remaining, a, b)
        if not len(remaining): break
    return pieces


def pad_from_patch(patch, thickness, name):
    origin, u, v, n = [np.array(patch[k]) for k in
        ('origin_m', 'tangent_u', 'tangent_v', 'outward_normal')]
    points = np.array(patch['source_points_m']); xy = (points-origin) @ np.column_stack((u,v))
    half_width = float(np.min(np.abs(xy[:,0])))
    lo, hi = float(xy[:,1].min()), float(xy[:,1].max())
    face_center = origin + v*(lo+hi)/2
    center = face_center - n*thickness/2
    R = np.column_stack((u,v,n)); q = Rotation.from_matrix(R).as_quat()
    planes = [(u, u@origin+half_width), (-u, -u@origin+half_width),
              (v, v@origin+hi), (-v, -v@origin-lo), (-n, -n@origin+thickness)]
    return dict(name=name, size_m=[half_width, (hi-lo)/2, thickness/2],
        pos_m=center.tolist(), quat_wxyz=np.r_[q[3],q[:3]].tolist(), rotation=R.tolist(),
        front_center_m=face_center.tolist(), outward_normal=n.tolist(),
        source_face_indices=patch.get('source_face_indices', []),
        prism_planes=[dict(a=a.tolist(), b=float(b)) for a,b in planes])


def box_vertices(pad):
    return np.array(list(product([-1.,1.], repeat=3)))*pad['size_m'] @ np.array(pad['rotation']).T + pad['pos_m']


def export_hull(path, vertices):
    h = ConvexHull(vertices); faces = h.simplices.copy()
    for i, face in enumerate(faces):
        a,b,c = vertices[face]
        if np.dot(np.cross(b-a,c-a), h.equations[i,:3]) < 0: faces[i] = face[[0,2,1]]
    with path.open('x') as f:
        for v in vertices: f.write('v '+' '.join(format(float(x),'.17g') for x in v)+'\n')
        for face in faces: f.write('f '+' '.join(str(int(i)+1) for i in face)+'\n')


def prepare(campaign, basis_path):
    campaign = campaign.resolve(); basis_path = basis_path.resolve()
    if campaign == ROOT or ROOT/'meshes' in campaign.parents or HISTORY in campaign.parents:
        raise ValueError('Output must be an independent experiment directory')
    basis = json.loads(basis_path.read_text())
    for name, expected in basis['input_sha256'].items():
        if sha(ROOT/name) != expected: raise ValueError('Source geometry changed: '+name)
    if basis['status'] != 'GEOMETRY_ENVELOPE_TRACED_CANDIDATE_UNFROZEN': raise ValueError('Missing source basis')
    campaign.mkdir(parents=True, exist_ok=False); (campaign/'guards').mkdir()
    a = ET.parse(HISTORY/'R1/model.xml').getroot()
    inputs = {str(basis_path): sha(basis_path), str(Path(__file__).resolve()): sha(__file__)}
    for filename in ('run.py','controller.py','config.json','grasp.json','base-filter-approval.json'):
        path = ROOT/'experiments/dynamics'/filename
        frozen = subprocess.check_output(['git','show','1e4d633730f62a398513d33991fa96f9c4977daa:experiments/dynamics/'+filename],cwd=ROOT)
        if path.read_bytes()!=frozen: raise ValueError('Frozen runtime changed: '+filename)
        inputs[str(path)] = sha(path)
    for p in ('protocol.json','initial-state.npz','model.xml'):
        inputs[str(HISTORY/'R1'/p)] = sha(HISTORY/'R1'/p)
    historical = json.loads((ROOT/'debug/issue23/historical-input-seals.json').read_text())
    for path,digest in historical['input_sha256'].items():
        if sha(path)!=digest: raise ValueError('Historical input changed: '+path)
        inputs[path] = digest
    for filename in ('collision_geometry_match.py','fixed_collision_query.py'):
        path = Path(__file__).with_name(filename)
        if path.read_bytes()!=subprocess.check_output(['git','show',BASE+':experiments/dynamics/'+filename],cwd=ROOT):
            raise ValueError('Query helper changed: '+filename)
        inputs[str(path.resolve())] = sha(path)
    for node in a.iter():
        if node.get('file'):
            path = (HISTORY/'R1'/node.get('file')).resolve()
            inputs[str(path)] = sha(path); node.set('file', str(path))
    ET.indent(a); ET.ElementTree(a).write(campaign/'A.xml', encoding='utf-8', xml_declaration=True)
    b = copy.deepcopy(a); fingers = {}
    for finger, info in basis['fingers'].items():
        pads = []; patches = info['patches']
        thickness = float(patches['root_front_plane']['xyz_bounds_m'][0][2]-patches['tip_zero_plane']['xyz_bounds_m'][0][2])
        body = b.find(f".//body[@name='{finger}']")
        original = body.find(f"geom[@name='{finger}_collision_000']"); body.remove(original)
        mesh_path = ROOT/info['files']['collision_000']; inputs[str(mesh_path)] = sha(mesh_path)
        old_vertices = hull_points(trimesh.load(mesh_path, force='mesh', process=False).vertices)
        pieces = [old_vertices]
        for key in PATCHES:
            pad = pad_from_patch(patches[key], thickness, finger+'_pad_'+key); pads.append(pad)
            planes = [(np.array(p['a']),p['b']) for p in pad['prism_planes']]
            pieces = [v for old in pieces for v in subtract_prism(old, planes)]
            geom = copy.deepcopy(original); geom.attrib.pop('mesh')
            geom.set('name',pad['name']); geom.set('type','box')
            for attr, value in [('size',pad['size_m']),('pos',pad['pos_m']),('quat',pad['quat_wxyz'])]:
                geom.set(attr,' '.join(format(x,'.17g') for x in value))
            body.append(geom)
        guards = []
        for i, vertices in enumerate(pieces):
            name = finger+'_guard_'+str(i); path = campaign/'guards'/(name+'.obj')
            export_hull(path, vertices)
            ET.SubElement(b.find('asset'),'mesh',name=name,file=str(path))
            geom = copy.deepcopy(original); geom.set('name',name); geom.set('mesh',name); body.append(geom)
            guards.append(dict(name=name, file=str(path), vertices=len(vertices),
                               volume_m3=float(ConvexHull(vertices).volume), sha256=sha(path)))
        fingers[finger] = dict(pads=pads, guards=guards, thickness_m=thickness,
            original_convex_volume_m3=float(ConvexHull(old_vertices).volume),
            guards_volume_m3=sum(g['volume_m3'] for g in guards))
    ET.indent(b); ET.ElementTree(b).write(campaign/'B.xml', encoding='utf-8', xml_declaration=True)
    diff = ''.join(difflib.unified_diff((campaign/'A.xml').read_text().splitlines(True),
        (campaign/'B.xml').read_text().splitlines(True), fromfile='A.xml',tofile='B.xml'))
    (campaign/'candidate.diff').write_text(diff)
    plan_path = ROOT/'reports/issue18-evidence/preregistration.json'
    inputs[str(plan_path)] = sha(plan_path)
    inputs.update({str(ROOT/k):v for k,v in basis['input_sha256'].items()})
    candidate = dict(issue=23, base=BASE, created_utc=datetime.now(timezone.utc).isoformat(),
        geometry_rule='One full-length inscribed source rectangle per tip plane and ramp; thickness is measured 2mm plate; no outcome fitting',
        guard_rule='Disjoint convex halfspace partition of old000 minus the two outward-open rectangular prisms, retaining rear/side/root protection; old001 unchanged',
        fingers=fingers, new_steps=0, new_queries=0, inputs_sha256=inputs,
        parameters='All non-target mechanical/control/contact/solver fields unchanged; same original finger bodies',
        stop_rule='On candidate+retained000 guard contact with same cube/finger, stop as WORKFACE_ISOLATION_UNKNOWN; no physical episodes until complete Gate0 PASS',
        files_sha256={str(p.relative_to(campaign)):sha(p) for p in campaign.rglob('*') if p.is_file()})
    write(campaign/'candidate.json',candidate); (campaign/'candidate.sha256').write_text(sha(campaign/'candidate.json')+'\n')
    print(json.dumps(dict(status='CANDIDATE_FROZEN_ZERO_QUERY',sha256=sha(campaign/'candidate.json'),
          pads={k:len(v['pads']) for k,v in fingers.items()}, guards={k:len(v['guards']) for k,v in fingers.items()})))


def query(campaign):
    """Static-only API, exclusive execution and persistent stop. No retry."""
    campaign = campaign.resolve(); stage = campaign/'static'
    if (campaign/'STOPPED.json').exists(): raise ValueError('Campaign stopped; no retry')
    candidate = json.loads((campaign/'candidate.json').read_text())
    if sha(campaign/'candidate.json') != (campaign/'candidate.sha256').read_text().strip(): raise ValueError('Candidate seal changed')
    for path, digest in candidate['inputs_sha256'].items():
        if sha(path) != digest: raise ValueError('Input seal changed: '+path)
    for path, digest in candidate['files_sha256'].items():
        if sha(campaign/path) != digest: raise ValueError('Candidate file changed: '+path)
    for name in ('collision_geometry_match','fixed_collision_query'):
        expected = Path(__file__).with_name(name+'.py').resolve()
        spec = importlib.util.find_spec(name)
        if spec is None or Path(spec.origin).resolve()!=expected: raise ValueError('Wrong helper resolution: '+name)
    head = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if subprocess.check_output(['git','merge-base',head,BASE],cwd=ROOT,text=True).strip()!=BASE:
        raise ValueError('Wrong branch ancestry')
    for relative, actual in [('experiments/dynamics/fingertip_pads.py',Path(__file__)),
                             ('reports/issue23-candidate.json',campaign/'candidate.json')]:
        if subprocess.check_output(['git','show',head+':'+relative],cwd=ROOT)!=actual.read_bytes():
            raise ValueError('Candidate and query code must be committed before B: '+relative)
    from collision_geometry_match import compare_geometry
    from fixed_collision_query import guard_api, load_episode
    import mujoco
    if mujoco.__version__!='3.2.3' or sys.version_info[:3]!=(3,11,9): raise ValueError('Frozen environment mismatch')
    for fn in (compare_geometry,guard_api,load_episode):
        expected = Path(__file__).with_name(fn.__module__+'.py').resolve()
        if Path(sys.modules[fn.__module__].__file__).resolve()!=expected or Path(fn.__code__.co_filename).resolve()!=expected:
            raise ValueError('Wrong runtime helper identity: '+fn.__name__)
    stage.mkdir(exist_ok=False); archive = stage/'executed-source'; archive.mkdir()
    modules = {str(Path(__file__).resolve()):sha(__file__)}
    for module in list(sys.modules.values()):
        path = getattr(module,'__file__',None)
        if path and Path(path).resolve().is_relative_to(ROOT): modules[str(Path(path).resolve())] = sha(path)
    for i,path in enumerate(modules): (archive/(str(i)+'_'+Path(path).name)).write_bytes(Path(path).read_bytes())
    write(stage/'runtime-before.json',dict(project_modules_sha256=modules, actual_modules=True,
          git_head=head,
          git_status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=ROOT,text=True),
          python=sys.version, executable=sys.executable,
          versions={n:importlib.metadata.version(n) for n in ('mujoco','numpy','scipy','trimesh')},
          candidate_sha256=sha(campaign/'candidate.json'), new_steps=0, new_force_solves=0))
    counts={'A':0,'B':0}; status='UNKNOWN'; reason=None
    audit = guard_api(mujoco)
    try:
        sealed = stage/'sealed-inputs'; sealed.mkdir()
        archive_paths = {str(campaign/'candidate.json'), *modules}
        archive_paths.update(str(campaign/p) for p in candidate['files_sha256'])
        archive_paths.update(path for path in candidate['inputs_sha256'] if Path(path).suffix not in ('.npz','.jsonl'))
        receipts=[]
        for i,path in enumerate(sorted(archive_paths)):
            target=sealed/(str(i)+'_'+Path(path).name); shutil.copyfile(path,target)
            receipts.append(dict(source=path,archive=str(target.relative_to(stage)),sha256=sha(target)))
        write(stage/'archive-receipt.json',receipts)
        models = {k:mujoco.MjModel.from_xml_path(str(campaign/(k+'.xml'))) for k in ('A','B')}
        a,b = models['A'],models['B']
        unchanged = []
        for name in dir(a):
            if name.startswith(('actuator_','dof_','jnt_','eq_')) or name in ('nq','nv','nu','neq','nbody','body_mass','body_inertia','body_iquat','body_ipos','body_pos','body_quat','body_parentid'):
                if not np.array_equal(getattr(a,name),getattr(b,name)): raise ValueError('Unauthorized compiled change: '+name)
                unchanged.append(name)
        for name in dir(a.opt):
            if not name.startswith('_') and not callable(getattr(a.opt,name)):
                if not np.array_equal(getattr(a.opt,name),getattr(b.opt,name)): raise ValueError('Solver option changed: '+name)
        if int(a.opt.cone)!=1 or a.opt.impratio!=10 or int(a.opt.enableflags)!=0: raise ValueError('Wrong frozen solver')
        names={k:[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) for i in range(m.ngeom)] for k,m in models.items()}
        material_fields = ('geom_friction','geom_solref','geom_solimp','geom_condim','geom_margin','geom_gap','geom_contype','geom_conaffinity','geom_priority','geom_solmix')
        shape_fields = ('geom_type','geom_bodyid','geom_size','geom_pos','geom_quat','geom_dataid')
        for old,name in enumerate(names['A']):
            if name in {f+'_collision_000' for f in candidate['fingers']}: continue
            new = names['B'].index(name)
            for field in material_fields+shape_fields:
                if not np.array_equal(getattr(a,field)[old],getattr(b,field)[new]):
                    raise ValueError('Non-target geometry changed: '+name+' '+field)
        # Compare effective material for every new geometry against its old000 source.
        for finger, f in candidate['fingers'].items():
            old=names['A'].index(finger+'_collision_000')
            for item in f['pads']+f['guards']:
                new=names['B'].index(item['name'])
                for field in material_fields:
                    if not np.array_equal(getattr(a,field)[old],getattr(b,field)[new]): raise ValueError('Material changed: '+field)
        write(stage/'compiled-invariants.json',dict(unchanged=unchanged, all_options_identical=True, effective_new_geom_materials_identical=True))
        episodes={label:load_episode(label) for label in ('R0','R1')}
        frozen=json.loads((ROOT/'reports/issue18-evidence/preregistration.json').read_text())
        selection=frozen['plan']['selected_rows']
        ordered=[('R1',i) for i in (23622,23623,23624)]
        ordered += [(label,i) for label in ('R1','R0') for i in selection[label] if (label,i) not in ordered]
        with (stage/'queries.jsonl').open('x') as stream:
            for label,row in ordered:
                output={}
                for algorithm,m in models.items():
                    current = {str(Path(module.__file__).resolve()) for module in list(sys.modules.values())
                        if getattr(module,'__file__',None) and Path(module.__file__).resolve().is_relative_to(ROOT)}
                    if current-set(modules): raise ValueError('Unsealed late project module: '+str(current-set(modules)))
                    for path,digest in modules.items():
                        if sha(path)!=digest: raise ValueError('Runtime module bytes changed: '+path)
                    qpos = episodes[label]['qpos'][row]
                    if hashlib.sha256(qpos.tobytes()).hexdigest()!=frozen['plan']['qpos_sha256'][label][str(row)]:
                        raise ValueError('Historical qpos differs from Issue18 seal')
                    if not np.all(np.isfinite(qpos)): raise ValueError('Nonfinite historical qpos')
                    data=mujoco.MjData(m);data.qpos[:]=qpos
                    mujoco.mj_kinematics(m,data);mujoco.mj_collision(m,data);counts[algorithm]+=1
                    if not np.array_equal(data.qpos,qpos) or np.any(data.warning.number): raise ValueError('Static query changed state or warned')
                    if data.time!=0 or data.nefc!=0 or np.any(data.qvel) or np.any(data.ctrl) or np.any(data.qfrc_constraint): raise ValueError('Forbidden dynamics state')
                    events=[]
                    for c in data.contact:
                        if not np.all(np.isfinite(np.r_[c.dist,c.pos,c.frame[:3]])): raise ValueError('Nonfinite contact geometry')
                        events.append(dict(geoms=[names[algorithm][c.geom1],names[algorithm][c.geom2]],
                            bodies=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[g])) for g in (c.geom1,c.geom2)],
                            distance_m=float(c.dist),position_m=c.pos.tolist(),normal=c.frame[:3].tolist()))
                    output[algorithm]=events
                    if algorithm=='A':
                        reproduction=compare_geometry(episodes[label]['events'][row],events)
                        if reproduction['status']!='PASS': break
                coexist={}
                for finger,f in candidate['fingers'].items():
                    touch=[e for e in output.get('B',[]) if 'cube_geom' in e['geoms'] and finger in e['bodies']]
                    pads={p['name'] for p in f['pads']};guards={g['name'] for g in f['guards']}
                    coexist[finger]=dict(pad_contacts=[e for e in touch if pads.intersection(e['geoms'])],
                                        guard_contacts=[e for e in touch if guards.intersection(e['geoms'])])
                record=dict(label=label,row=row,geometry=output,A_reproduction=reproduction,coexist=coexist,
                    qpos_sha256=hashlib.sha256(episodes[label]['qpos'][row].tobytes()).hexdigest(),new_steps=0,new_force_solves=0)
                stream.write(json.dumps(record,allow_nan=False)+'\n');stream.flush()
                if reproduction['status']!='PASS': reason='QUERY_REPRODUCTION_UNKNOWN';break
                if any(v['pad_contacts'] and v['guard_contacts'] for v in coexist.values()):
                    reason='WORKFACE_ISOLATION_UNKNOWN';break
            if reason is None: reason='COVERAGE_AND_EXTREME_POSES_NOT_YET_VERIFIED'
    except BaseException:
        reason='BLOCKED_API_OR_ANALYSIS_ERROR';write(stage/'error.json',dict(traceback=traceback.format_exc()));raise
    finally:
        result=dict(issue=23,gate0=status,stop_reason=reason,counts=counts,api_calls=audit,
                    new_steps=0,new_force_solves=0,new_episodes=0,A_episode='NOT_RUN',B_episode='NOT_RUN',
                    completed_utc=datetime.now(timezone.utc).isoformat(),candidate_sha256=sha(campaign/'candidate.json'))
        write(stage/'result.json',result);write(campaign/'STOPPED.json',result)
    print(json.dumps(result));return 2


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=('prepare','query'));p.add_argument('--campaign',type=Path,required=True);p.add_argument('--basis',type=Path)
    args=p.parse_args()
    if args.mode=='prepare':prepare(args.campaign,args.basis);return 0
    return query(args.campaign)


if __name__=='__main__': raise SystemExit(main())
