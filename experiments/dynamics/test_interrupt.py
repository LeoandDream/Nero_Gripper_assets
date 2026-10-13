"""Actual short integrations must retain their evidence when the user stops."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import run as experiment
from controller import load_config, HERE


class InterruptTests(unittest.TestCase):
    def test_control_and_grasp_save_completed_steps_before_propagating_stop(self):
        original=experiment.Recorder.record
        def stop_after_three_steps(recorder,data,phase):
            result=original(recorder,data,phase)
            if recorder.actual_steps==3:raise KeyboardInterrupt('intentional test stop')
            return result
        cfg=load_config()
        protocol=json.loads((HERE/'grasp.json').read_text())
        for stage in ['control','grasp']:
            with self.subTest(stage=stage),tempfile.TemporaryDirectory() as tmp:
                directory=Path(tmp)/stage
                with patch.object(experiment.Recorder,'record',stop_after_three_steps):
                    with self.assertRaises(KeyboardInterrupt):
                        if stage=='control':
                            experiment.execute(directory,cfg,[{'name':'hold','target':cfg['initial_action'],'duration_s':.01}],filtered=True)
                        else:
                            plan={'initial_action':cfg['initial_action'],'approach':[cfg['initial_action'][:7]],'lift':[cfg['initial_action'][:7]]}
                            experiment.execute_grasp(directory,cfg,protocol,protocol['cube'],plan)
                result=json.loads((directory/'result.json').read_text())
                self.assertEqual(result['status'],'INTERRUPTED')
                self.assertEqual(result['actual_mj_steps'],3)
                self.assertEqual(result['recorded_physics_samples'],3)
                with np.load(directory/'trajectory.npz') as saved:
                    np.testing.assert_allclose(saved['time'],[.001,.002,.003],rtol=0,atol=1e-12)
                    self.assertTrue(np.isfinite(saved['qpos']).all())
                self.assertEqual(len((directory/'contacts.jsonl').read_text().splitlines()),3)
                if stage=='grasp':self.assertEqual(json.loads((directory/'grasp-result.json').read_text())['status'],'INTERRUPTED')


if __name__=='__main__':unittest.main()
