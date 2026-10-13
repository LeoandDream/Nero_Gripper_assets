"""Asset delivery checks; no physics integration or policy execution."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


class AssetTests(unittest.TestCase):
    def test_source_snapshots_are_exact(self):
        manifest = json.loads((ROOT / 'SOURCE_MANIFEST.json').read_text())
        self.assertEqual(manifest['revision'], 'a68b8e876a2c6fcb0ecee7a7943129e4d48dd864')
        for item in manifest['files']:
            data = (ROOT / item['file']).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), item['sha256'])
            git_blob = hashlib.sha1(f'blob {len(data)}\0'.encode() + data).hexdigest()
            self.assertEqual(git_blob, item['git_blob'])

    def test_virtual_driver_elimination_preserves_real_mass_and_mimic(self):
        model = mujoco.MjModel.from_xml_path(str(ROOT / 'mjcf/nero_gripper.xml'))
        source = ET.parse(ROOT / 'urdf/nero_gripper.urdf').getroot()
        driver = source.find("joint[@name='gripper']")
        empty = source.find("link[@name='gripper_link']")
        self.assertIsNotNone(driver)
        self.assertEqual(len(empty), 0)
        self.assertEqual((model.nq, model.nv, model.nu, model.neq), (9, 9, 0, 1))
        self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper_link'), -1)
        total_mass = sum(float(i.find('mass').get('value')) for i in source.findall('link/inertial'))
        self.assertAlmostEqual(float(model.body_mass.sum()), total_mass, places=12)
        np.testing.assert_array_equal(model.eq_data[0, :5], [0, -1, 0, 0, 0])
        for width in [0, .05, .1]:
            data = mujoco.MjData(model)
            for name, value in [('gripper_joint1', width / 2), ('gripper_joint2', -width / 2)]:
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                data.qpos[model.jnt_qposadr[jid]] = value
            mujoco.mj_forward(model, data)
            self.assertEqual(data.time, 0)
            self.assertAlmostEqual(float(data.efc_pos[0]), 0, places=12)

    def test_independent_urdf_validation(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'audit.json'
            subprocess.run([sys.executable, str(ROOT / 'scripts/validate.py'), '--output', str(output)], check=True)
            audit = json.loads(output.read_text())
            self.assertEqual(audit['status'], 'PASS_STATIC_EQUIVALENCE')
            self.assertGreaterEqual(len(audit['poses']), 11)
            self.assertLess(audit['max_errors']['body_position_m'], 1e-10)
            self.assertLess(audit['max_errors']['inertia_kg_m2'], 1e-12)
            self.assertEqual(audit['simulation_steps'], 0)

    def test_portable_model_compiles_after_copy(self):
        with tempfile.TemporaryDirectory() as folder:
            copy = Path(folder) / 'assets'
            shutil.copytree(ROOT, copy, ignore=shutil.ignore_patterns('.venv', '.upstream', '.git', 'debug', '__pycache__'))
            model = mujoco.MjModel.from_xml_path(str(copy / 'examples/scene.xml'))
            self.assertEqual(model.nq, 9)
            for element in ET.parse(copy / 'mjcf/nero_gripper.xml').iter():
                if element.get('file'):
                    self.assertFalse(Path(element.get('file')).is_absolute())

    def test_rejects_correlated_reference_corruption(self):
        # A wrong portable URDF and a matching wrong MJCF must not validate each other.
        with tempfile.TemporaryDirectory() as folder:
            copy = Path(folder)/'assets'
            shutil.copytree(ROOT, copy, ignore=shutil.ignore_patterns('.venv','.upstream','.git','debug','__pycache__'))
            urdf_path = copy/'urdf/nero_gripper.urdf'
            urdf = ET.parse(urdf_path)
            mass = urdf.find("link[@name='base_link']/inertial/mass")
            wrong = float(mass.get('value')) + .1
            mass.set('value', str(wrong))
            urdf.write(urdf_path)
            mjcf_path = copy/'mjcf/nero_gripper.xml'
            mjcf = ET.parse(mjcf_path)
            mjcf.find(".//body[@name='base_link']/inertial").set('mass', str(wrong))
            mjcf.write(mjcf_path)
            result = subprocess.run([sys.executable,str(copy/'scripts/validate.py'),'--output',str(copy/'audit.json')],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0,'corrupted derived model validated against a corrupted reference')
            self.assertIn('Expanded URDF differs from original source',result.stderr)

    def test_rejects_disabled_collision_masks(self):
        with tempfile.TemporaryDirectory() as folder:
            copy = Path(folder)/'assets'
            shutil.copytree(ROOT, copy, ignore=shutil.ignore_patterns('.venv','.upstream','.git','debug','__pycache__'))
            path = copy/'mjcf/nero_gripper.xml'
            tree = ET.parse(path)
            for geom in tree.findall('.//geom'):
                if geom.get('group')=='0':
                    geom.set('contype','0')
                    geom.set('conaffinity','0')
            tree.write(path)
            result = subprocess.run([sys.executable,str(copy/'scripts/validate.py'),'--output',str(copy/'audit.json')],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0,'disabled physical collision geometry passed static validation')
            self.assertIn('Collision masks differ from asset contract',result.stderr)

    def test_static_view_does_not_change_physics(self):
        spec = importlib.util.spec_from_file_location('view_static', ROOT / 'examples/view_static.py')
        viewer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(viewer)
        model = mujoco.MjModel.from_xml_path(str(ROOT / 'examples/scene.xml'))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        fields = ['body_mass', 'body_inertia', 'jnt_range', 'geom_contype', 'geom_conaffinity', 'geom_friction', 'eq_data']
        before = {name: getattr(model, name).copy() for name in fields}
        option, camera = viewer.setup_view(model, data)
        self.assertFalse(option.geomgroup[0])
        collision, _ = viewer.setup_view(model, data, show_collision=True)
        self.assertTrue(collision.geomgroup[0])
        for name in fields:
            np.testing.assert_array_equal(before[name], getattr(model, name))
        self.assertEqual(data.time, 0)
        self.assertGreater(camera.distance, 0)

    def test_headless_static_export(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'preview.png'
            env = dict(os.environ, MUJOCO_GL='egl')
            subprocess.run([sys.executable, str(ROOT/'examples/view_static.py'), '--image', str(output)], env=env, check=True)
            from PIL import Image
            with Image.open(output) as image:
                self.assertEqual(image.size, (640,480))


if __name__ == '__main__':
    unittest.main()
