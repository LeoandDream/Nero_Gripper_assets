"""Finite static contact sanity; no new physics integration and no asset edits.

Checks saved R1 poses, a 40mm cube against source-selected work faces, and a
0.5mm sphere against those faces and all eleven known sidewall witnesses.
The sphere is a separate query-only probe, never an episode/model change.
"""
import argparse
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from simulation_mvp import ROOT, MODEL, Controller, freeze_execution, load_case, runtime, write, sha


def sat_check(model,data,event,entities):
    """Independent mesh/box separating-axis sanity, not a solver replacement."""
    name=next(g for g in event['geoms'] if '_mvp_' in g)
    body=next(b for b in event['bodies'] if b!='cube')
    bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,body)
    e=entities[name.rsplit('_',1)[1]]
    a=np.array(e['vertices_m'])@data.xmat[bid].reshape(3,3).T+data.xpos[bid]
    faces=np.array(e['faces']);t=a[faces]
    n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);n=n[np.linalg.norm(n,axis=1)>1e-18]
    cid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'cube_geom')
    rot=data.geom_xmat[cid].reshape(3,3)
    corners=np.array([[x,y,z] for x in (-.02,.02) for y in (-.02,.02) for z in (-.02,.02)])
    b=corners@rot.T+data.geom_xpos[cid]
    edges=np.concatenate([t[:,1]-t[:,0],t[:,2]-t[:,1],t[:,0]-t[:,2]])
    axes=np.concatenate([n,rot.T,np.cross(edges[:,None],rot.T[None]).reshape(-1,3)])
    axes=axes[np.linalg.norm(axes,axis=1)>1e-14];axes/=np.linalg.norm(axes,axis=1)[:,None]
    x=a@axes.T;y=b@axes.T
    depth=np.minimum(x.max(0)-y.min(0),y.max(0)-x.min(0));i=int(np.argmin(depth))
    normal=axes[i]*(1 if (b.mean(0)-a.mean(0))@axes[i]>=0 else -1)
    mpr=np.array(event['normal'])*(1 if event['bodies'][1]=='cube' else -1)
    return dict(sat_penetration_mm=float(depth[i]*1000),sat_normal=normal.tolist(),
                mpr_normal_vs_sat_deg=float(np.degrees(np.arccos(np.clip(normal@mpr,-1,1)))))


def cube_events(model,data):
    return [e for e in runtime.contacts(model,data) if 'cube' in e['bodies'] and
            any(b in ('gripper_link1','gripper_link2') for b in e['bodies'])]


def summaries(events,normal=None):
    result = dict(contacts=len(events),max_penetration_mm=max([0.]+[-e['distance_m']*1000 for e in events]),events=events)
    if normal is not None:
        oriented = [np.array(e['normal'])*(1 if e['bodies'][1]=='cube' else -1) for e in events]
        angles = [float(np.degrees(np.arccos(np.clip(n@normal,-1,1)))) for n in oriented]
        result['normal_deviation_deg'] = angles
        result['closest_normal_deviation_deg'] = min(angles) if angles else None
    return result


def diagnose(output):
    freeze_execution(output)
    case=load_case()
    entities={e['id']:e for e in json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text())['entities']}
    model=mujoco.MjModel.from_xml_path(str(MODEL))
    fixture=Path(__file__).with_name('simulation_mvp_reference.npz')
    ref=np.load(fixture,allow_pickle=False)
    historical=[]
    for row,qpos,phase in zip(ref['rows'],ref['qpos'],ref['phase']):
        data=mujoco.MjData(model);data.qpos[:]=qpos;mujoco.mj_forward(model,data)
        historical.append(dict(row=int(row),phase=str(phase),**summaries(cube_events(model,data))))
    raw=(ROOT/'meshes/source/agx_gripper/meshes/gripper_link1.stl').read_bytes()
    triangles=np.array([[struct.unpack_from('<12fH',raw,84+50*i)[3+3*j:6+3*j] for j in range(3)] for i in range(struct.unpack_from('<I',raw,80)[0])])
    regions=json.loads((ROOT/'reports/nero-gripper-functional-regions.json').read_text())['source_regions']
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    area=np.linalg.norm(normals,axis=1);normals/=area[:,None]
    samples=[]
    for region in ('broad_main_inner_plane','short_inner_plane','inner_ramp','inner_transition_1','inner_transition_2','tip_front_edge'):
        ids=[i for a,b in regions[region]['source_faces_zero_based_inclusive'] for i in range(a,b+1)]
        fid=max(ids,key=lambda i:area[i])
        samples.append(dict(label=region,point=triangles[fid].mean(0),normal=normals[fid],source_face=fid))
    witnesses=json.loads((ROOT/'reports/nero-gripper-v21-normal-witnesses.json').read_text())['confirmed_witnesses']
    for i,w in enumerate(witnesses):
        samples.append(dict(label='sidewall_witness_'+str(i),point=np.array(w['source_point_mm'])/1000,
                            normal=np.array(w['source_normal']),source_face=w['source_stl_face']))
    # Independent query-only copy. Only the target cube geom becomes a small
    # sphere for local normal inspection; original MVP and smoke cube untouched.
    root=ET.parse(MODEL).getroot()
    for node in root.iter():
        if node.get('file'):node.set('file',str((MODEL.parent/node.get('file')).resolve()))
    root.find(".//geom[@name='cube_geom']").set('type','sphere')
    root.find(".//geom[@name='cube_geom']").set('size','.0005')
    sphere=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    output_samples=[]
    for side in (1,2):
        bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'gripper_link'+str(side))
        data=runtime.reset(model,Controller(model,case['config']),np.r_[case['joint_plan']['initial_action'][:7],.1])
        rotation=data.xmat[bid].reshape(3,3).copy();origin=data.xpos[bid].copy()
        for s in samples:
            n=s['normal'];tangent=np.cross(n,[1,0,0] if abs(n[0])<.9 else [0,1,0]);tangent/=np.linalg.norm(tangent)
            bitangent=np.cross(n,tangent);world_normal=rotation@n
            for kind,engine,half,push in (('cube40mm',model,.02,.0001),('local_sphere',sphere,.0005,.00001)):
                if kind=='cube40mm' and s['label'].startswith('sidewall_'):continue
                series=[]
                for offset in (-1e-5,0.,1e-5):
                    d=runtime.reset(engine,Controller(engine,case['config']),np.r_[case['joint_plan']['initial_action'][:7],.1])
                    d.qpos[9:12]=origin+rotation@(s['point']+(half-push)*n+offset*tangent)
                    # Cube inward face is parallel to the selected source face.
                    r=rotation@np.column_stack((tangent,bitangent,n))
                    xyzw=Rotation.from_matrix(r).as_quat();d.qpos[12:16]=np.r_[xyzw[3],xyzw[:3]]
                    mujoco.mj_forward(engine,d)
                    events=[e for e in cube_events(engine,d) if 'gripper_link'+str(side) in e['bodies']]
                    if kind=='cube40mm':
                        for event in events:event['independent_sat']=sat_check(engine,d,event,entities)
                    series.append(dict(tangent_offset_mm=offset*1000,**summaries(events,world_normal)))
                output_samples.append(dict(finger=side,region=s['label'],source_face=s['source_face'],probe=kind,series=series))
    report=dict(actual_mj_steps=0,historical_trajectory_sha256=str(ref['source_sha256']),fixture_sha256=sha(fixture),
                historical=historical,surface_queries=output_samples,
                note='Finite engineering observations; source-selected triangle centroids, not whole-surface fidelity certification. Cube probes may contact neighboring real geometry; small sphere isolates local normals.')
    write(output/'contacts.json',report)
    print(json.dumps(dict(historical_frames=len(historical),historical_max_penetration_mm=max(s['max_penetration_mm'] for s in historical),
          cube_probe_count=sum(s['probe']=='cube40mm' for s in output_samples),local_probe_count=sum(s['probe']=='local_sphere' for s in output_samples),
          local_missing=sum(not r['contacts'] for s in output_samples if s['probe']=='local_sphere' for r in s['series']),
          local_max_penetration_mm=max(r['max_penetration_mm'] for s in output_samples if s['probe']=='local_sphere' for r in s['series']))),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    diagnose(p.parse_args().output.resolve())
