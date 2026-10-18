"""Plot immutable Issue14 full episodes; NumPy/Matplotlib only, no MuJoCo."""
import argparse
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--campaign',type=Path,required=True)
p.add_argument('--analyses',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
fig,axes=plt.subplots(3,2,figsize=(12,10),layout='constrained');inputs={}
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
for label,color in [('R0','tab:blue'),('R1','tab:orange')]:
    path=a.campaign/label
    for name in ('trajectory.npz','contacts.jsonl','observations.npz'):
        inputs[str(path/name)]=sha(path/name)
    with np.load(path/'trajectory.npz') as z:raw={k:z[k] for k in z.files}
    with np.load(path/'observations.npz') as z:gp=z['gripper_position_world_m']
    summary=json.loads((a.analyses/(label+'-analysis-final/summary.json')).read_text())
    lift=np.flatnonzero(raw['phase']=='lift');settle=np.flatnonzero(raw['phase']=='grasp_settle')[-1]
    elapsed=raw['time']-raw['time'][lift[0]]+.001
    selected=elapsed>=0;support=np.zeros(len(elapsed));penetration=np.zeros(len(elapsed));normals={}
    first=summary['earliest_frozen_violation']['row']
    with (path/'contacts.jsonl').open() as stream:
        for i,line in enumerate(stream):
            forces=np.zeros(2)
            for e in json.loads(line)['events']:
                if 'cube' not in e['bodies']:continue
                other=e['bodies'][1-e['bodies'].index('cube')]
                if other!='world':penetration[i]=max(penetration[i],-e['distance_m'])
                if other in ('gripper_link1','gripper_link2'):forces[int(other[-1])-1]+=max(0.,e['force_contact_frame'][0])
                if other=='gripper_link2' and abs(i-first)<=1:
                    normals[i-first]=(1 if e['bodies'][1]=='cube' else -1)*e['normal'][2]
            support[i]=min(forces)
    gap=raw['qpos'][:,11]-gp[:,2];loaded=np.isin(raw['phase'],['lift','hold'])
    axes[0,0].plot(elapsed[selected],(raw['qpos'][selected,11]-.15)*1000,color=color,label=label)
    axes[0,1].plot(elapsed[selected],penetration[selected]*1000,color=color)
    axes[1,0].plot(elapsed[loaded],(gap[settle]-gap[loaded])*1000,color=color)
    axes[1,1].plot(elapsed[selected],support[selected],color=color)
    axes[2,0].plot(sorted(normals),[normals[k] for k in sorted(normals)],'o-',color=color,label=label)
    axes[2,1].plot(elapsed[selected],abs(raw['qpos'][selected,7]+raw['qpos'][selected,8])*1000,color=color)
    hold=np.flatnonzero(raw['phase']=='hold')
    for ax in [axes[0,0],axes[0,1],axes[1,0],axes[1,1],axes[2,1]]:
        ax.axvspan(elapsed[hold[0]],elapsed[hold[-1]],color=color,alpha=.08)
axes[0,0].axhline(80,color='black',ls='--',lw=1)
axes[0,1].axhline(1,color='black',ls='--',lw=1)
axes[1,1].axhline(.05,color='black',ls='--',lw=1)
axes[2,1].axhline(.2,color='black',ls='--',lw=1)
titles=['Cube lift above original center (mm)','Robot-cube penetration (mm)',
        'Relative world-Z drop since settle (mm), release excluded','Minimum of the two finger normal forces (N)',
        'Finger2 normal Z on cube: first violation and adjacent rows','Finger coupling residual (mm)']
for ax,title in zip(axes.flat,titles):ax.set(title=title,xlabel='Time since lift start (s)');ax.grid(alpha=.2)
axes[2,0].set_xlabel('Recorded row offset from first penetration violation');axes[2,0].set_xticks([-1,0,1])
axes[0,0].legend();axes[2,0].legend()
fig.suptitle('Issue14 normal-table full grasp: R0 frozen v2; R1 peak target speed 0.05 -> 0.20 rad/s\nShaded windows: each run\'s original 2s hold. Both original task outcomes FAIL.')
a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=160)
for path,expected in inputs.items():assert sha(Path(path))==expected
with a.output.with_suffix('.inputs.json').open('x') as stream:json.dump(dict(inputs=inputs,new_mj_steps=0,force_solves=0),stream,indent=2)
