"""Offline reference/v1/v2 comparison, no simulator or geometry generation."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT=Path(__file__).resolve().parents[1]
COLORS={'distal_tip_contact':'#e9aa20','inner_main_contact_plate':'#d56b10',
        'inner_ramp_contact':'#c776a8','inner_transition_contact':'#a94f6d',
        'sidewall_negative_x':'#34a6d3','sidewall_positive_x':'#79b4d7',
        'outer_chassis':'#28ad8e','mount_root':'#9980b7'}


def draw(ax,data,elev,azim):
    triangles=[];colors=[]
    for e in data['entities']:
        t=(np.array(e['vertices_m'])*1000)[e['faces']]
        triangles.extend(t);colors.extend([COLORS[e['semantic_family']]]*len(t))
    ax.add_collection3d(Poly3DCollection(triangles,facecolors=colors,edgecolors='#354452',linewidths=.13,alpha=.96,rasterized=True))
    ax.set(xlim=(-30,30),ylim=(-78,2),zlim=(-2,29),xlabel='x (mm)',ylabel='y (mm)',zlabel='z (mm)')
    ax.set_box_aspect((60,80,31));ax.view_init(elev=elev,azim=azim);ax.tick_params(labelsize=6)


def main():
    data=[json.loads((ROOT/'reports'/name).read_text()) for name in ['nero-gripper-entity-partition.json','nero-gripper-runtime-partition.json','nero-gripper-v2-runtime-partition.json']]
    titles=['Reference: 1253 / finger','V1 runtime: 753 / finger','V2 single candidate: 346 / finger']
    fig=plt.figure(figsize=(17,11),layout='constrained')
    for col,(item,title) in enumerate(zip(data,titles)):
        for row,(elev,azim) in enumerate([(-84,-90),(28,52)]):
            ax=fig.add_subplot(2,3,1+col+3*row,projection='3d');draw(ax,item,elev,azim)
            ax.set_title(title+('\nWorkface / contour' if row==0 else '\nRear pocket / root'),fontsize=11,pad=20)
            if row==0:ax.set_xlabel('')
    fig.suptitle('Issue #25 — one v2 candidate, geometry locked for review; no MJCF or physics\n346 convex entities / finger; minimum inscribed diameter 0.111618 mm; acceptance BLOCKED',fontsize=14)
    fig.savefig(ROOT/'reports/nero-gripper-v2-runtime-comparison.png',dpi=160)


if __name__=='__main__':main()
