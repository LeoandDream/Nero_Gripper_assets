"""Targeted integration checks for the separately generated Simulation MVP."""
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


class SimulationMVP(unittest.TestCase):
    def test_mapping_and_compiled_coordinates(self):
        manifest = json.loads((ROOT/'mjcf/simulation_mvp.manifest.json').read_text())
        model = mujoco.MjModel.from_xml_path(str(ROOT/'mjcf/simulation_mvp.xml'))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertEqual(len(manifest['mapping']), 692)
        self.assertEqual((model.njnt, model.nu, model.neq), (10, 9, 1))
        source = json.loads((ROOT/'reports/nero-gripper-v2-runtime-partition.json').read_text())
        entities = {e['id']: e for e in source['entities']}
        for item in manifest['mapping']:
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, item['geom'])
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, item['body'])
            self.assertGreaterEqual(gid, 0)
            self.assertEqual(model.geom_bodyid[gid], bid)
            mid = model.geom_dataid[gid]
            vertices = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid]+model.mesh_vertnum[mid]]
            actual = vertices @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]
            expected = np.array(entities[item['entity']]['vertices_m']) @ data.xmat[bid].reshape(3, 3).T + data.xpos[bid]
            # MuJoCo stores compiled mesh vertices as float32 after recentering.
            distance = np.linalg.norm(expected[:, None]-actual[None], axis=-1)
            self.assertLess(float(distance.min(axis=1).max()), 2e-8)
            self.assertLess(float(distance.min(axis=0).max()), 2e-8)

    def test_only_finger_collision_representation_changed(self):
        from simulation_mvp import baseline_xml, load_case
        expected = ET.fromstring(baseline_xml(load_case()))
        actual = ET.parse(ROOT/'mjcf/simulation_mvp.xml').getroot()
        for tree in (expected, actual):
            asset = tree.find('asset')
            for node in list(asset):
                if node.tag == 'mesh' and (node.get('name', '').startswith('mvp_') or
                        node.get('name', '').startswith(('gripper_link1_collision_', 'gripper_link2_collision_'))):
                    asset.remove(node)
                elif node.get('file'):
                    node.attrib.pop('file')  # file paths are rebased; bytes checked by manifest
            for body in tree.findall('.//body'):
                if body.get('name') in ('gripper_link1', 'gripper_link2'):
                    for node in list(body):
                        if node.tag == 'geom' and node.get('contype') == '1':
                            body.remove(node)
            for node in tree.iter():
                node.text = None
                node.tail = None
        self.assertEqual(ET.tostring(actual), ET.tostring(expected))


if __name__ == '__main__':
    unittest.main()
