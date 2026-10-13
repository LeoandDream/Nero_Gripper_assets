"""Tests of the actual 8D target contract and derived actuator mapping."""
from pathlib import Path
import tempfile
import unittest
import numpy as np
import mujoco
from controller import Controller, build_model, load_config


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load_config()
        self.tmp=tempfile.TemporaryDirectory()
        self.model=build_model(Path(self.tmp.name)/'model.xml',self.cfg)
        self.data=mujoco.MjData(self.model)
        self.controller=Controller(self.model,self.cfg)

    def tearDown(self):
        self.tmp.cleanup()

    def test_eight_command_dimensions_nine_physical_joints(self):
        self.assertEqual((self.model.nq,self.model.nu,self.model.neq),(9,9,1))
        self.controller.reset_target(np.array(self.cfg['initial_action']))
        self.controller.set_target([0,0,0,.5,0,0,0,.05])
        self.controller.advance_command()
        q=self.controller.physical_target
        self.assertEqual(q.shape,(9,))
        self.assertAlmostEqual(q[-1],-q[-2])
        self.assertEqual(self.model.nexclude,0)

    def test_invalid_target_preserves_last_valid_command(self):
        initial=np.array(self.cfg['initial_action'],float)
        self.controller.reset_target(initial)
        invalid=[np.zeros(9),np.full(8,np.nan),np.full(8,np.inf),initial.astype(complex),[0,-.3,0,.5,0,0,0,.08],[0,0,0,.5,0,0,0,.11],[4,0,0,.5,0,0,0,.08]]
        for command in invalid:
            with self.assertRaises(ValueError):self.controller.set_target(command)
            np.testing.assert_array_equal(self.controller.requested,initial)

    def test_generalized_forces_remain_bounded_and_do_not_write_state(self):
        self.controller.reset_target(np.array(self.cfg['initial_action']))
        mujoco.mj_forward(self.model,self.data)
        before=self.data.qpos.copy()
        self.controller.apply(self.data)
        np.testing.assert_array_equal(before,self.data.qpos)
        limits=np.r_[self.cfg['arm_force_limits_Nm'],[self.cfg['finger_force_limit_N']]*2]
        self.assertTrue(np.all(np.abs(self.data.ctrl)<=limits))
        self.assertTrue(np.all(self.data.qfrc_applied==0))
        self.assertTrue(np.all(self.data.xfrc_applied==0))
        for i,name in enumerate([f'joint{j}' for j in range(1,8)]+['gripper_joint1','gripper_joint2']):
            aid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_ACTUATOR,'servo_'+name)
            jid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,name)
            self.assertEqual(self.model.actuator_trnid[aid,0],jid)


if __name__=='__main__':unittest.main()
