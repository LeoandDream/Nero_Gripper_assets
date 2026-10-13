"""Static reference and executable configuration share immutable mechanics."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import json
import numpy as np
import mujoco
from controller import ASSET_ROOT, Controller, build_model, load_config


class AssetReuseTests(unittest.TestCase):
    def test_static_reference_has_no_actuators_and_derived_robot_fields_match(self):
        static=mujoco.MjModel.from_xml_path(str(ASSET_ROOT/'mjcf/nero_gripper.xml'))
        cube=json.loads((ASSET_ROOT/'experiments/dynamics/grasp.json').read_text())['cube']
        with tempfile.TemporaryDirectory() as tmp:
            derived=build_model(Path(tmp)/'model.xml',load_config(),exclude_base_link1=True,cube=cube)
            self.assertEqual((static.nq,static.nv,static.nu,static.neq),(9,9,0,1))
            self.assertEqual((derived.nq,derived.nv,derived.nu,derived.neq,derived.nexclude),(16,15,9,1,1))
            fields={mujoco.mjtObj.mjOBJ_BODY:(static.nbody,['body_mass','body_inertia','body_pos','body_quat','body_ipos','body_iquat']),
                    mujoco.mjtObj.mjOBJ_JOINT:(static.njnt,['jnt_type','jnt_range','jnt_axis','jnt_pos']),
                    mujoco.mjtObj.mjOBJ_GEOM:(static.ngeom,['geom_type','geom_pos','geom_quat','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp'])}
            for kind,(count,names) in fields.items():
                indices=[mujoco.mj_name2id(derived,kind,mujoco.mj_id2name(static,kind,i)) for i in range(count)]
                for name in names:np.testing.assert_array_equal(getattr(static,name),getattr(derived,name)[indices],err_msg=name)
            for name in ['dof_damping','dof_armature','dof_frictionloss','eq_type','eq_data','eq_solref','eq_solimp']:
                original=getattr(static,name)
                np.testing.assert_array_equal(original,getattr(derived,name)[:len(original)],err_msg=name)
            base=mujoco.mj_name2id(derived,mujoco.mjtObj.mjOBJ_BODY,'base_link')
            link=mujoco.mj_name2id(derived,mujoco.mjtObj.mjOBJ_BODY,'link1')
            self.assertEqual(int(derived.exclude_signature[0]),(min(base,link)<<16)+max(base,link))
            c=Controller(derived,load_config())
            self.assertEqual(c.validate(load_config()['initial_action']).shape,(8,))

    def test_portable_checkout_executes_full_control_without_source_worktree(self):
        with tempfile.TemporaryDirectory() as tmp:
            copied=Path(tmp)/'Nero_Gripper_assets'
            shutil.copytree(ASSET_ROOT,copied,ignore=shutil.ignore_patterns('.git','.venv','.upstream','debug','runs','__pycache__','.worktrees'))
            env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='')
            completed=subprocess.run([sys.executable,str(copied/'experiments/dynamics/run.py'),'--stage','control','--filtered','--run-id','portable-test'],
                                      cwd=tmp,env=env,capture_output=True,text=True)
            self.assertEqual(completed.returncode,0,completed.stdout+completed.stderr)
            output=copied/'experiments/dynamics/runs/portable-test/approved-filter-control'
            result=json.loads((output/'result.json').read_text())
            self.assertEqual(result['status'],'PASS')
            self.assertEqual(result['actual_mj_steps'],37000)
            self.assertEqual(len(result['phases']),21)
            self.assertTrue(all(phase['status']=='PASS' for phase in result['phases']))
            with np.load(output/'trajectory.npz') as state:
                self.assertEqual(state['qpos'].shape,(37000,9))
                self.assertFalse(np.any(state['qfrc_applied']))
                self.assertFalse(np.any(state['xfrc_applied']))


if __name__=='__main__':unittest.main()
