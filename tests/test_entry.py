"""The documented test entry must discover both existing validation layers."""
import importlib.util
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]


class EntryTests(unittest.TestCase):
    def test_unified_entry_discovers_static_and_dynamic_suites(self):
        entry=ROOT/'scripts/test.py'
        self.assertTrue(entry.is_file(),'Missing unified test entry')
        spec=importlib.util.spec_from_file_location('nero_test_entry',entry)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        suites=module.discover_suites('all')
        def ids(suite):
            for test in suite:
                if isinstance(test,unittest.TestSuite):yield from ids(test)
                else:yield test.id()
        self.assertTrue(any(name.startswith('test_assets.AssetTests.') for name in ids(suites['static'])))
        self.assertTrue(any(name.startswith('test_controller.ControllerTests.') for name in ids(suites['dynamic'])))
        self.assertNotIn('dynamic',module.discover_suites('static'))
        self.assertNotIn('static',module.discover_suites('dynamic'))


if __name__=='__main__':unittest.main()
