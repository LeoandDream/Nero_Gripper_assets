"""Plot saved Issue10 reductions; quaternion tilt is a pure saved-data formula.

Uses an existing plotting interpreter, independent of the frozen MuJoCo env.
No MuJoCo import, no simulation. Refuses to overwrite the figure directory.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--analysis',type=Path,required=True)
parser.add_argument('--output-dir',type=Path,required=True)
args=parser.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=False)
summary=json.loads((args.analysis/'summary.json').read_text())
with np.load(args.analysis/'series.npz',allow_pickle=False) as z:a={k:z[k] for k in z.files}
source=Path(summary['source'])/'trajectory.npz'
if hashlib.sha256(source.read_bytes()).hexdigest()!=summary['source_hashes'][str(source)]:raise ValueError('Source changed')
with np.load(source,allow_pickle=False) as z:q=z['qpos'][:summary['pretable_rows']]
initial_path=Path(summary['source'])/'initial-state.npz'
if hashlib.sha256(initial_path.read_bytes()).hexdigest()!=summary['source_hashes'][str(initial_path)]:raise ValueError('Initial changed')
with np.load(initial_path,allow_pickle=False) as z:initial_q=z['qpos']
q=np.vstack([initial_q,q]);quat=q[:,12:16]
np.testing.assert_allclose(np.linalg.norm(quat,axis=1),1,rtol=0,atol=1e-12)
# Free-joint quaternion is w,x,y,z. This measures tilt of cube +Z from world +Z.
tilt=np.arccos(np.clip(1-2*(quat[:,1]**2+quat[:,2]**2),-1,1))
contacts=list(csv.DictReader((args.analysis/'contacts.csv').open()))
t=a['elapsed_s'];colors=['#1565c0','#c62828']

def decorate(axes):
    for ax in axes:
        for x in [.141,1.328,2.164]:ax.axvline(x,color='.6',lw=.7,ls=':')
        ax.set_xlim(0,4.416);ax.grid(alpha=.2)
    axes[-1].set_xlabel('Elapsed time from frozen snapshot (s); table contact at 4.416 s excluded')

fig,axes=plt.subplots(6,1,figsize=(11,14),sharex=True,layout='constrained')
axes[0].plot(t,a['relative_world_z_drop_m']*1000,label='cube relative to gripper origin')
axes[0].plot(t,-a['cube_world_delta_z_m']*1000,label='cube world descent',alpha=.6)
axes[0].set_ylabel('Descent (mm)');axes[0].legend(loc='upper left')
axes[1].plot(t,a['relative_world_vz_m_s']*1000,label='relative world Z velocity')
axes[1].plot(t,a['cube_world_vz_m_s']*1000,label='cube world Z velocity',alpha=.65)
axes[1].axhline(0,color='k',lw=.6);axes[1].set_ylabel('Velocity (mm/s)');axes[1].legend(loc='lower left')
axes[2].plot(t,a['Fz_minus_mg_N'],color='.65',lw=.6,label='raw post-forward force minus mg')
axes[2].plot(t[19:],np.convolve(a['Fz_minus_mg_N'],np.ones(20)/20,mode='valid'),color='k',lw=1,label='trailing 20-ms visual average')
axes[2].axhline(0,color='tab:red',ls='--',lw=.7);axes[2].set_ylabel('Fz - mg (N)');axes[2].legend(loc='lower left')
axes[3].plot(t,a['actual_slide_width_m']*1000,label='actual joint slide separation')
axes[3].plot(t,a['target_slide_width_m']*1000,ls='--',label='constant command')
axes[3].set_ylabel('Slide width (mm)');axes[3].legend(loc='lower left')
for k,c in zip([1,2],colors):axes[4].step(t,a[f'finger{k}_count'],where='post',color=c,label=f'finger {k} geometry count',alpha=.75)
axes[4].set_ylim(-.1,2.2);axes[4].set_ylabel('Contact count');axes[4].legend(loc='upper left')
axes[5].plot(t,a['max_penetration_m']*1000,label='maximum robot/cube penetration')
axes[5].axhline(1,color='tab:red',ls='--',label='unchanged 1-mm criterion')
axes[5].set_ylabel('Penetration (mm)');axes[5].legend(loc='upper left')
decorate(axes);fig.suptitle('Issue #10 | saved elliptic trajectory | diagnostic-only | no new integration')
fig.savefig(out/'timing.png',dpi=150);plt.close(fig)

fig,axes=plt.subplots(5,1,figsize=(11,12),sharex=True,layout='constrained')
for k,c in zip([1,2],colors):
    cp=[p for p in contacts if int(p['finger'])==k and float(p['Fn_N'])>1e-12]
    ts=np.array([float(p['elapsed_s']) for p in cp])
    for ax,key,scale in [(axes[0],'point_cube_local_m_z',1000),(axes[1],'normal_on_cube_z',1),(axes[2],'rho_elliptic',1)]:
        ax.scatter(ts,np.array([float(p[key]) for p in cp])*scale,s=2,color=c,alpha=.4,label=f'finger {k}')
axes[0].axhline(20,color='k',ls='--',lw=.7,label='cube local upper face')
axes[0].set_ylabel('Contact cube-local Z (mm)');axes[0].legend(loc='upper left')
axes[1].axhline(0,color='k',lw=.6);axes[1].set_ylabel('Normal on cube: Z');axes[1].legend(loc='lower left')
axes[2].axhline(1,color='k',ls='--',lw=.7);axes[2].set_ylabel('Elliptic utilization (L2)')
axes[3].plot(t,np.degrees(a['relative_rotation_rad']),label='relative orientation change')
axes[3].plot(t,np.degrees(tilt),label='cube +Z tilt from world +Z')
axes[3].set_ylabel('Angle (deg)');axes[3].legend(loc='upper left')
axes[4].plot(t,a['finger1_Fz_normal_N']+a['finger2_Fz_normal_N'],label='normal force contribution')
axes[4].plot(t,a['finger1_Fz_tangent_N']+a['finger2_Fz_tangent_N'],label='tangential contribution',alpha=.7)
axes[4].axhline(summary['mg_N'],color='k',ls='--',lw=.7,label='mg')
axes[4].set_ylabel('Vertical support (N)');axes[4].legend(loc='lower left')
decorate(axes);fig.suptitle('Contact migration and orientation | samples are not persistent material points')
fig.savefig(out/'geometry.png',dpi=150);plt.close(fig)

ticks=np.rint(t/.001).astype(int);bounds=[0,141,1328,2164,4416];details=[]
for lo,hi in zip(bounds[:-1],bounds[1:]):
    mask=(ticks>=lo)&(ticks<hi)&(ticks>0)
    cp=[p for p in contacts if lo<=round(float(p['elapsed_s'])/.001)<hi and int(p['row'])>=0 and float(p['Fn_N'])>1e-12]
    interior=[p for p in cp if float(p['rho_elliptic'])<1-1e-10]
    details.append(dict(window_ticks=[lo,hi],positive_contact_samples=len(cp),interior_contact_samples=len(interior),
        interior_with_tangent_speed_gt_1mm_s=sum(float(p['slip_speed_m_s'])>.001 for p in interior),
        cube_tilt_max_deg=float(np.degrees(tilt[mask]).max()),cube_tilt_end_deg=float(np.degrees(tilt[mask][-1]))))
extremes={}
for key in ['Fz_minus_mg_N','relative_world_vz_m_s','max_penetration_m','relative_rotation_rad']:
    i=int(np.argmin(a[key]) if key in ['Fz_minus_mg_N','relative_world_vz_m_s'] else np.argmax(a[key]))
    extremes[key]={'row':int(a['row'][i]),'elapsed_s':float(t[i]),'value':float(a[key][i])}
metadata=dict(python=sys.version,matplotlib=matplotlib.__version__,numpy=np.__version__,
    input_summary_sha256=hashlib.sha256((args.analysis/'summary.json').read_bytes()).hexdigest(),
    input_series_sha256=hashlib.sha256((args.analysis/'series.npz').read_bytes()).hexdigest(),
    input_contacts_sha256=hashlib.sha256((args.analysis/'contacts.csv').read_bytes()).hexdigest(),
    quaternion_source_sha256=summary['source_hashes'][str(source)],
    new_mj_steps=0,force_solves=0,windows=details,extremes=extremes,
    plot_average='20 samples, trailing, display only; statistics use unsmoothed records')
(out/'plot-and-supplement.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps({'figures':str(out),'windows':details,'extremes':extremes}),flush=True)
