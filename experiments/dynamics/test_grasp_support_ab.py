"""Guard against unauthorized model changes and incomplete snapshot restoration."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

try:
    import grasp_support_ab as ab
except ModuleNotFoundError:
    ab = None


class SupportABTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(ab, 'A/B diagnostic runner is not implemented')

    def test_variant_changes_only_table_height_and_resolves_assets(self):
        source = '<mujoco><asset><mesh name="m" file="mesh.obj"/></asset><worldbody><geom name="table" pos=".3 .2 .065" size=".2 .2 .065"/><body name="cube" pos=".3 .2 .15"><geom size=".02" mass=".1"/></body></worldbody></mujoco>'
        a, b = ab.model_variants(source, Path('/tmp/source'))
        ar, br = ET.fromstring(a), ET.fromstring(b)
        self.assertEqual(ar.find('.//mesh').get('file'), '/tmp/source/mesh.obj')
        self.assertEqual(br.find('.//geom[@name="table"]').get('pos'), '.3 .2 0.015')
        br.find('.//geom[@name="table"]').set('pos', ar.find('.//geom[@name="table"]').get('pos'))
        self.assertEqual(ET.tostring(ar), ET.tostring(br))

    def test_variant_rejects_unapproved_starting_table(self):
        for source in ['<mujoco><worldbody/></mujoco>', '<mujoco><worldbody><geom name="table" pos="0 0 .064"/></worldbody></mujoco>']:
            with self.subTest(source=source), self.assertRaises(ValueError):
                ab.model_variants(source, Path('/tmp'))

    def test_restore_preserves_warmstart_time_velocity_and_external_inputs(self):
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint name="j"/><geom size=".1"/></body></worldbody><actuator><motor joint="j"/></actuator></mujoco>')
        source = mujoco.MjData(model)
        source.time = 2.3
        source.qpos[:] = .2; source.qvel[:] = -.4
        source.qacc_warmstart[:] = .7; source.ctrl[:] = .8
        source.qfrc_applied[:] = .09; source.xfrc_applied[1, 2] = .06
        state = ab.integration_state(model, source)
        restored = ab.restore_state(model, state)
        np.testing.assert_array_equal(ab.integration_state(model, restored), state)
        self.assertEqual(restored.time, 2.3)
        np.testing.assert_array_equal(restored.qacc_warmstart, [.7])
        np.testing.assert_array_equal(restored.qvel, [-.4])
        self.assertEqual(restored.xfrc_applied[1, 2], .06)
        np.testing.assert_array_equal(source.qpos, [.2])
        with self.assertRaises(ValueError): ab.restore_state(model, state[:-1])
        bad = state.copy(); bad[0] = np.nan
        with self.assertRaises(ValueError): ab.restore_state(model, bad)

    def test_changed_b_protocol_is_rejected_before_a_starts(self):
        self.assertTrue(callable(getattr(ab, 'validate_readiness', None)), 'All artifacts need an execution gate')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); artifacts = {}
            for group in ('A', 'B'):
                (root / group).mkdir()
                for name in ('model.xml', 'protocol.json', 'initial-state.npz'):
                    path = root / group / name; path.write_text('original')
                    artifacts[f'{group}/{name}'] = hashlib.sha256(b'original').hexdigest()
            seal = dict(run_script_sha256=hashlib.sha256(Path(ab.__file__).read_bytes()).hexdigest(), artifacts=artifacts)
            (root / 'reviewed-readiness.json').write_text(json.dumps(seal))
            (root / 'B/protocol.json').write_text('changed gain')
            with self.assertRaisesRegex(ValueError, 'Prepared artifact changed: B/protocol.json'):
                ab.execute(root)
            self.assertFalse((root / 'A/contacts.jsonl').exists())


if __name__ == '__main__':
    unittest.main()
