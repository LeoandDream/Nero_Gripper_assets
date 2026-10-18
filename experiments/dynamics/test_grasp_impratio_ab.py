"""Issue #12 targeted checks: no physical integration."""
import tempfile
import unittest
from pathlib import Path

import mujoco

from grasp_impratio_ab import impratio_xml, check_models, claim_run


class ImpratioChecks(unittest.TestCase):
    def test_single_option_and_compiled_delta(self):
        text = '<mujoco><option cone="elliptic" timestep="0.001" /></mujoco>'
        result = impratio_xml(text)
        self.assertEqual(result.replace('impratio="10" ', ''), text)
        check_models(mujoco.MjModel.from_xml_string(text), mujoco.MjModel.from_xml_string(result))

    def test_reject_other_model_delta(self):
        a = mujoco.MjModel.from_xml_string('<mujoco><option cone="elliptic"/></mujoco>')
        b = mujoco.MjModel.from_xml_string('<mujoco><option cone="elliptic" impratio="10" gravity="0 0 -8"/></mujoco>')
        with self.assertRaises(ValueError): check_models(a, b)

    def test_reject_wrong_source_or_repeat_transformation(self):
        for text in ['<mujoco><option/></mujoco>', '<mujoco><option cone="elliptic" impratio="1"/></mujoco>']:
            with self.assertRaises(ValueError): impratio_xml(text)

    def test_exclusive_attempt_claim(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            claim_run(p)
            with self.assertRaises(FileExistsError): claim_run(p)
            (p / 'trajectory.npz').write_bytes(b'original')
            with self.assertRaises(FileExistsError): claim_run(p)
            self.assertEqual((p / 'trajectory.npz').read_bytes(), b'original')


if __name__ == '__main__': unittest.main()
