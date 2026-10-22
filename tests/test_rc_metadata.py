"""Release metadata only: never run a physical entry point."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'experiments/dynamics'))


def modules():
    import simulation_mvp
    spec = importlib.util.spec_from_file_location('rc_gripper_control', ROOT/'examples/gripper_control.py')
    component = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(component)
    return simulation_mvp, component


class OptionalGitMetadata(unittest.TestCase):
    def test_zip_recording_has_no_fake_git_head(self):
        for module in modules():
            with self.subTest(module=module.__name__):
                self.assertTrue(hasattr(module,'checkout_head'), 'Recording still requires inline git rev-parse')
                with tempfile.TemporaryDirectory() as folder, patch.object(module,'ROOT',Path(folder)), \
                        patch.object(module.subprocess,'check_output',side_effect=AssertionError('ZIP must not call git')):
                    self.assertIsNone(module.checkout_head())

    def test_missing_git_executable_is_not_a_physical_failure(self):
        for module in modules():
            with self.subTest(module=module.__name__):
                self.assertTrue(hasattr(module,'checkout_head'))
                with tempfile.TemporaryDirectory() as folder:
                    (Path(folder)/'.git').touch()
                    with patch.object(module,'ROOT',Path(folder)), \
                            patch.object(module.subprocess,'check_output',side_effect=FileNotFoundError('git absent')):
                        self.assertIsNone(module.checkout_head())


if __name__ == '__main__':
    unittest.main()
