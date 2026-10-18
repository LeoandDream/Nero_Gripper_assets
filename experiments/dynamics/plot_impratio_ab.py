"""Plot Issue12 saved reductions only; no MuJoCo dependency."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--analysis',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
report=json.loads((a.analysis/'comparison.json').read_text())
fig,axes=plt.subplots(3,2,figsize=(12,10),layout='constrained')
for group,color in [('A','tab:blue'),('B','tab:orange')]:
    with np.load(a.analysis/(group+'-series.npz')) as s:
        t=s['elapsed_s'];label=group+' impratio='+str(report['groups'][group]['impratio'])
        axes[0,0].plot(t,s['relative_world_z_drop_m']*1000,color=color,label=label)
        early=t<=1.328+1e-9
        axes[0,1].plot(t[early],s['relative_world_z_drop_m'][early]*1000,color=color)
        axes[1,0].plot(t,s['relative_world_vz_m_s']*1000,color=color)
        axes[1,1].plot(t,s['robot_penetration_m']*1000,color=color)
        axes[2,0].plot(t,np.degrees(s['relative_rotation_rad']),color=color)
        axes[2,1].plot(t,s['coupling_residual_m']*1000,color=color)
    event=report['groups'][group]['first_table_geometry']
    if event:
        for ax in axes.flat:
            if ax is not axes[0,1]:ax.axvline(event['elapsed_s'],color=color,ls=':',alpha=.6)
axes[0,1].axhline(5.761707,color='black',ls='--',lw=1,label='P1 threshold at 1.328s')
axes[1,1].axhline(1,color='black',ls='--',lw=1)
axes[2,1].axhline(.2,color='black',ls='--',lw=1)
for ax,title in zip(axes.flat,['Relative world-Z drop (mm)','Early relative drop (mm)',
    'Relative world-Z velocity (mm/s)','Robot-cube penetration (mm)',
    'Cube rotation relative to gripper (deg)','Finger coupling residual (mm)']):
    ax.set(title=title,xlabel='Elapsed time (s)');ax.grid(alpha=.2)
axes[0,0].legend();axes[0,1].legend()
fig.suptitle('Issue12 diagnostic only: A saved; B one new run\nDotted blue line: A first table contact. No B table contact in 14.1s.')
a.output.parent.mkdir(parents=True,exist_ok=True)
fig.savefig(a.output,dpi=160)
