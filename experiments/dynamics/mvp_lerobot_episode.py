"""Issue27: one saved R1 -> clean RGB -> LeRobot v2.1. Never integrates.

prepare/render use the existing MuJoCo3.2.3 environment; export uses the
locked OpenPI/LeRobot environment. No package installation or source writes.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
PROTOCOL=ROOT/'reports/issue27-protocol.json'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def selected_rows(times):
    times=np.asarray(times)
    np.testing.assert_allclose(times,np.arange(1,len(times)+1)*.001,rtol=0,atol=1e-8)
    return np.arange(49,len(times)-1,50)


def frame_values(a,row):
    return np.asarray(a['qpos'][row,:9],dtype=np.float32),np.asarray(a['requested'][row+1],dtype=np.float32)


def context(output):
    p=json.loads(PROTOCOL.read_text());plan=json.loads((output/'plan.json').read_text())
    assert sha(PROTOCOL)==plan['protocol_sha256'],'Frozen protocol changed'
    assert sha(Path(p['source_run'])/'trajectory.npz')==p['source_trajectory_sha256']
    return p,plan


def prepare(output):
    p=json.loads(PROTOCOL.read_text());run=Path(p['source_run']);parent=run.parent
    assert sha(p['source_manifest'])==p['source_manifest_sha256']
    m=json.loads(Path(p['source_manifest']).read_text())
    for name,item in m.items():assert sha(parent/name)==item['sha256'],name
    a=np.load(run/'trajectory.npz',allow_pickle=False);rows=selected_rows(a['time'])
    assert len(a['time'])==34150 and len(rows)==682
    output.mkdir(parents=True,exist_ok=False)
    (output/'rgb').mkdir();(output/'source').mkdir()
    (output/'source/protocol.json').write_bytes(PROTOCOL.read_bytes())
    (output/'source/exporter.py').write_bytes(Path(__file__).read_bytes())
    table=[];variation=[]
    for i,k in enumerate(rows):
        stop=min(k+51,len(a['time']))
        variation.append(np.max(abs(a['requested'][k+1:stop]-a['requested'][k+1]),axis=0))
        table.append(dict(frame_index=i,timestamp=i/20,source_observation_row=int(k),source_time_s=float(a['time'][k]),
                          source_action_row=int(k+1),action_interval_start_s=float(a['time'][k]),
                          action_interval_end_s=float(a['time'][k+1]),phase=str(a['phase'][k])))
    with (output/'alignment.csv').open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(table[0]));writer.writeheader();writer.writerows(table)
    np.savez_compressed(output/'sidecar.npz',source_rows=rows,source_action_rows=rows+1,
        state=a['qpos'][rows,:9].astype(np.float32),actions=a['requested'][rows+1].astype(np.float32),
        observation_command=a['command'][rows],next_command=a['command'][rows+1],
        next_physical_target=a['physical_target'][rows+1],next_ctrl_force_torque=a['ctrl'][rows+1],
        qvel=a['qvel'][rows],cube_qpos=a['qpos'][rows,9:],phase=a['phase'][rows],
        within_50ms_requested_max_change=np.array(variation))
    assets={}
    for node in ET.parse(run/'model.xml').getroot().iter():
        if node.get('file'):
            file=(run/node.get('file')).resolve();assets[str(file)]=sha(file)
    write(output/'plan.json',dict(protocol_sha256=sha(PROTOCOL),exporter_sha256=sha(__file__),source_files=m,
        frames=len(rows),fps=20,physical_start_s=float(a['time'][rows[0]]),physical_last_s=float(a['time'][rows[-1]]),
        last_source_time_s=float(a['time'][-1]),alignment_sha256=sha(output/'alignment.csv'),sidecar_sha256=sha(output/'sidecar.npz'),
        model_sha256=sha(run/'model.xml'),asset_sha256=assets,actual_mj_steps=0,
        variation_max_per_dimension=np.max(variation,axis=0).tolist(),variation_median_per_dimension=np.median(variation,axis=0).tolist(),
        note='20Hz first-next-command subsampling is not lossless 1kHz control replay. Positional units/ranges are unchanged.'))
    print('PREPARED',len(rows),'frames; protocol',sha(PROTOCOL),flush=True)


def render(output):
    import mujoco
    from PIL import Image
    if mujoco.__version__!='3.2.3':raise ValueError('Render in the existing MuJoCo3.2.3 environment')
    p,plan=context(output)
    if list((output/'rgb').glob('*.png')):raise FileExistsError('Never overwrite rendered frames')
    def forbidden(*args,**kwargs):raise RuntimeError('Issue27 forbids mj_step')
    mujoco.mj_step=forbidden
    run=Path(p['source_run']);a=np.load(run/'trajectory.npz',allow_pickle=False);rows=selected_rows(a['time'])
    model=mujoco.MjModel.from_xml_path(str(run/'model.xml'));data=mujoco.MjData(model)
    cam=mujoco.MjvCamera();spec=p['camera']
    for key in ('lookat','distance','azimuth','elevation'):
        if key=='lookat':cam.lookat[:]=spec[key]
        else:setattr(cam,key,spec[key])
    opt=mujoco.MjvOption();opt.geomgroup[0]=0;opt.geomgroup[1:3]=1
    image_hashes={};scene_camera=None
    with mujoco.Renderer(model,height=256,width=256) as renderer:
        for i,k in enumerate(rows):
            data.qpos[:]=a['qpos'][k];data.qvel[:]=a['qvel'][k];data.time=float(a['time'][k])
            mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera=cam,scene_option=opt)
            rgb=renderer.render();assert rgb.shape==(256,256,3) and rgb.dtype==np.uint8
            path=output/'rgb'/f'frame_{i:06d}.png';Image.fromarray(rgb).save(path);image_hashes[path.name]=sha(path)
            if scene_camera is None:
                scene_camera=[]
                for c in renderer.scene.camera:
                    scene_camera.append({key:np.asarray(getattr(c,key)).tolist() for key in ('pos','forward','up','frustum_bottom','frustum_top','frustum_center','frustum_near','frustum_far')})
            if i%100==0:print('rendered',i,'source row',int(k),flush=True)
    for name,expected in plan['asset_sha256'].items():assert sha(name)==expected,name
    write(output/'render.json',dict(mujoco=mujoco.__version__,frames=len(rows),camera=spec,scene_camera=scene_camera,
        model_fovy_deg=float(model.vis.global_.fovy),source_model_sha256=plan['model_sha256'],image_sha256=image_hashes,
        actual_mj_steps=0,mj_step_guard=True,clean_rgb=True,overlays=False,only_source_group1_visuals=True,argv=sys.argv))
    print('RENDER_PASS',len(rows),'RGB frames; mj_step=0',flush=True)


def export(output):
    from PIL import Image
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    p,plan=context(output);rendered=json.loads((output/'render.json').read_text())
    a=np.load(output/'sidecar.npz',allow_pickle=False)
    root=output/'lerobot'/p['repo_id']
    if root.exists():raise FileExistsError(root)
    dataset=LeRobotDataset.create(repo_id=p['repo_id'],root=root,robot_type='nero_gripper',fps=20,
        features={'observation.images.base':{'dtype':'video','shape':(256,256,3),'names':['height','width','channels']},
                  'observation.state':{'dtype':'float32','shape':(9,),'names':p['state_names']},
                  'actions':{'dtype':'float32','shape':(8,),'names':p['action_names']}},
        use_videos=True,image_writer_threads=2,video_backend='pyav')
    for i in range(plan['frames']):
        path=output/'rgb'/f'frame_{i:06d}.png';assert sha(path)==rendered['image_sha256'][path.name]
        dataset.add_frame({'observation.images.base':np.array(Image.open(path).convert('RGB')),
            'observation.state':a['state'][i],'actions':a['actions'][i],
            'task':p['task']})  # pinned API generates timestamp=frame_index/fps
    dataset.save_episode()
    assert dataset.meta.total_episodes==1 and dataset.meta.total_frames==plan['frames']
    write(output/'dataset-manifest.json',dict(repo_id=p['repo_id'],root=str(root),frames=plan['frames'],episodes=1,fps=20,
        task=p['task'],posthoc_task=True,source_plan_sha256=sha(output/'plan.json'),render_sha256=sha(output/'render.json'),
        data_files={str(path.relative_to(root)):dict(sha256=sha(path),bytes=path.stat().st_size) for path in root.rglob('*') if path.is_file()},
        exporter_sha256=sha(__file__),argv=sys.argv,actual_mj_steps=0,hub_upload=False))
    print('LEROBOT_EXPORT_PASS',root,plan['frames'],'frames,1 episode',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=('prepare','render','export'))
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    {'prepare':prepare,'render':render,'export':export}[args.mode](args.output.resolve())
