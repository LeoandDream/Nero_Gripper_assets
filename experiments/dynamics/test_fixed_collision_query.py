"""Zero-query fixtures for the fixed-state experiment boundaries."""
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from fixed_collision_query import guard_api, require_gate, compare_models, output_outside_sources, load_plan, sha


class QueryBoundaryTests(unittest.TestCase):
    def test_gate_cannot_use_a_plan_from_different_executed_code(self):
        import json
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)
            (p/'plan.json').write_text(json.dumps({'code_sha256':{},'inputs_sha256':{}}))
            (p/'plan.sha256').write_text(sha(p/'plan.json')+'\n')
            with self.assertRaisesRegex(ValueError,'code changed'):
                load_plan(p)
            self.assertTrue((p/'STOPPED.json').exists())
            with self.assertRaisesRegex(ValueError,'Campaign STOPPED'):
                load_plan(p)

    def test_only_position_and_collision_functions_can_execute(self):
        calls=[]
        fake=SimpleNamespace(mj_kinematics=lambda *a:calls.append('kinematics'),
            mj_collision=lambda *a:calls.append('collision'),mj_forward=lambda *a:calls.append('forward'),
            mj_step=lambda *a:calls.append('step'),mj_makeConstraint=lambda *a:calls.append('constraint'),
            mj_contactForce=lambda *a:calls.append('force'))
        audit=guard_api(fake)
        fake.mj_kinematics(None,None);fake.mj_collision(None,None)
        for name in ['mj_forward','mj_step','mj_makeConstraint','mj_contactForce']:
            with self.assertRaises(RuntimeError):getattr(fake,name)(None,None)
        self.assertEqual(calls,['kinematics','collision'])
        self.assertEqual(audit['calls']['mj_collision'],1)

    def test_native_stage_requires_a_passed_gate(self):
        for state in ['FAIL','UNKNOWN','QUERY_REPRODUCTION_UNKNOWN']:
            with self.assertRaises(ValueError):require_gate({'status':state})
        require_gate({'status':'PASS'})

    def test_only_native_flag_can_differ_in_scratch_models(self):
        a=SimpleNamespace(qpos0=np.array([1.,2.]),opt=SimpleNamespace(enableflags=0,ccd_tolerance=1e-6))
        b=SimpleNamespace(qpos0=np.array([1.,2.]),opt=SimpleNamespace(enableflags=64,ccd_tolerance=1e-6))
        result=compare_models(a,b,64)
        self.assertEqual(result['option_differences'],['enableflags'])
        b.opt.ccd_tolerance=1e-5
        with self.assertRaises(ValueError):compare_models(a,b,64)

    def test_output_cannot_touch_either_archived_source(self):
        with tempfile.TemporaryDirectory() as t:
            original=Path(t)/'original';analysis=Path(t)/'analysis'
            for bad in [original,original/'new',analysis,analysis/'new']:
                with self.assertRaises(ValueError):output_outside_sources(bad,[original,analysis])
            output_outside_sources(Path(t)/'new',[original,analysis])


if __name__=='__main__':unittest.main()
