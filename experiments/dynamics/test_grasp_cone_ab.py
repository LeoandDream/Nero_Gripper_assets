"""Zero-integration checks for Issue #8's explicitly required boundaries."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco

try:
    import grasp_cone_ab as subject
except ModuleNotFoundError:
    subject = None


class ConeDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(subject, 'Issue #8 diagnostic implementation is absent')

    def test_xml_changes_only_cone_and_rejects_wrong_baseline(self):
        text = '<mujoco><option timestep="0.001"/><worldbody/></mujoco>'
        derived = subject.elliptic_xml(text)
        a, b = ET.fromstring(text), ET.fromstring(derived)
        self.assertEqual(b.find('option').attrib.pop('cone'), 'elliptic')
        self.assertEqual(ET.tostring(a), ET.tostring(b))
        with self.assertRaises(ValueError):
            subject.elliptic_xml(text.replace('timestep="0.001"', 'cone="elliptic"'))

    def test_compiled_comparison_rejects_other_options_and_arrays(self):
        text = '<mujoco><option timestep="0.001"/><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
        a = mujoco.MjModel.from_xml_string(text)
        b = mujoco.MjModel.from_xml_string(subject.elliptic_xml(text))
        self.assertEqual(subject.check_models(a, b)['option_differences'], ['cone'])
        b.opt.impratio = 2
        with self.assertRaises(ValueError): subject.check_models(a, b)
        b.opt.impratio = 1
        b.geom_friction[0, 0] = 2
        with self.assertRaises(ValueError): subject.check_models(a, b)

    def test_utilization_uses_current_cone_and_handles_zero_normal(self):
        self.assertAlmostEqual(subject.utilization([2, 1.2, 1.6], [1, 1], 'pyramidal'), 1.4)
        self.assertAlmostEqual(subject.utilization([2, 1.2, 1.6], [1, 1], 'elliptic'), 1.0)
        self.assertIsNone(subject.utilization([0, 0, 0], [1, 1], 'elliptic'))
        with self.assertRaises(ValueError): subject.utilization([1, 0, 0], [1, 1], 'invalid')

    def test_run_claim_rejects_reuse_and_existing_trajectory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            subject.claim_run(root)
            with self.assertRaises(FileExistsError): subject.claim_run(root)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'trajectory.npz').write_bytes(b'preserve')
            with self.assertRaises(FileExistsError): subject.claim_run(root)
            self.assertEqual((root / 'trajectory.npz').read_bytes(), b'preserve')

    def test_input_hash_check_rejects_tampering(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'state.npz'
            p.write_bytes(b'original snapshot')
            hashes = {p.name: subject.digest(p)}
            subject.verify_files(Path(tmp), hashes)
            p.write_bytes(b'changed snapshot')
            with self.assertRaises(ValueError): subject.verify_files(Path(tmp), hashes)


if __name__ == '__main__': unittest.main()
