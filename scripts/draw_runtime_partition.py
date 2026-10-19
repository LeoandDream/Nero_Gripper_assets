"""Standalone offline comparison figure; numpy + matplotlib, no simulation."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parents[1]
COLORS = dict(distal_tip_contact='#e69f00', inner_main_contact_plate='#d55e00',
              inner_ramp_contact='#cc79a7', inner_transition_contact='#b45071',
              sidewall_negative_x='#0072b2', sidewall_positive_x='#56b4e9',
              outer_chassis='#009e73', mount_root='#8d78ab')


def main():
    old = json.loads((ROOT/'reports/nero-gripper-entity-partition.json').read_text())
    new = json.loads((ROOT/'reports/nero-gripper-runtime-partition.json').read_text())
    fig = plt.figure(figsize=(16, 12), layout='constrained')
    for row, view in enumerate([(-87, -90, 'Inner workfaces / contours'), (22, 47, 'Back pocket / mount recesses')]):
        for col, data in enumerate([old, new]):
            ax = fig.add_subplot(2, 2, row*2+col+1, projection='3d')
            for i, e in enumerate(data['entities']):
                v = np.array(e['vertices_m'])*1000
                faces = [v[f] for f in e['faces']]
                base = np.array(matplotlib.colors.to_rgb(COLORS[e['semantic_family']]))
                color = np.clip(base*(.83+.28*((i*37)%101)/100), 0, 1)
                ax.add_collection3d(Poly3DCollection(faces, facecolors=[color], edgecolors='#1d3236', linewidths=.15 if col else .08))
            ax.set_xlim(-30, 30); ax.set_ylim(-79, 2); ax.set_zlim(-1, 28)
            ax.set_box_aspect((60, 81, 29)); ax.view_init(elev=view[0], azim=view[1])
            ax.set_xlabel('local x (mm)'); ax.set_ylabel('local y (mm)'); ax.set_zlabel('local z (mm)')
            title = 'Reference oracle' if col == 0 else 'Single runtime-oriented candidate'
            ax.set_title(f'{title}: {len(data["entities"])} / finger\n{view[2]}')
    fig.suptitle(f'Issue #25: {len(new["entities"])} entities / finger | {new["reduction_from_reference"]} fewer ({100*new["reduction_from_reference"]/1253:.2f}%)\nRUNTIME_ENTITY_PARTITION_BLOCKED: {new["residual_entities_below_minimum"]} entities below 0.1 mm inscribed diameter; no active model change', fontsize=13)
    fig.legend(handles=[Patch(color=c, label=f.replace('_', ' ')) for f, c in COLORS.items()], loc='outside lower center', ncol=4, fontsize=9)
    fig.savefig(ROOT/'reports/nero-gripper-runtime-partition.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
