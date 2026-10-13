"""Physics acceptance tests: reject unsupported lifts and excessive penetration."""
import copy
import tempfile
from pathlib import Path
import unittest
import numpy as np
import mujoco
import run as experiment
from controller import build_model, Controller, load_config


class GraspTests(unittest.TestCase):
    def fixture(self):
        rows={'time':np.arange(3000)*.001,'phase':np.array(['lift']*1000+['hold']*2000),
              'qpos':np.tile(np.r_[np.zeros(9),[-.32,0,.25,1,0,0,0]],(3000,1))}
        contacts=[{'events':[{'bodies':['gripper_link1','cube'],'distance_m':-.0002,'force_contact_frame':[1,0,0,0,0,0]},
                            {'bodies':['cube','gripper_link2'],'distance_m':-.0002,'force_contact_frame':[1,0,0,0,0,0]}]} for _ in range(3000)]
        criteria={'minimum_lift_m':.08,'hold_duration_s':2,'maximum_hold_drift_m':.01,
                  'minimum_bilateral_contact_fraction':.95,'minimum_finger_normal_force_N':.05,
                  'maximum_robot_object_penetration_m':.001}
        return rows,contacts,criteria

    def test_raised_object_without_bilateral_support_fails(self):
        self.assertTrue(hasattr(experiment,'evaluate_grasp'),'missing physics acceptance evaluator')
        rows,events,criteria=self.fixture()
        result=experiment.evaluate_grasp(rows,events,9,[-.32,0,.15],criteria,.001)
        self.assertEqual(result['status'],'PASS')
        for e in events:e['events']=e['events'][:1]
        result=experiment.evaluate_grasp(rows,events,9,[-.32,0,.15],criteria,.001)
        self.assertEqual(result['status'],'FAIL')
        self.assertIn('bilateral_finger_support',result['failed_criteria'])

    def test_excess_penetration_or_table_support_fails_even_when_lifted(self):
        self.assertTrue(hasattr(experiment,'evaluate_grasp'),'missing physics acceptance evaluator')
        rows,events,criteria=self.fixture()
        events[1800]['events'][0]['distance_m']=-.002
        result=experiment.evaluate_grasp(rows,events,9,[-.32,0,.15],criteria,.001)
        self.assertIn('robot_object_penetration',result['failed_criteria'])
        events[1800]['events'][0]['distance_m']=-.0002
        events[1800]['events'].append({'bodies':['world','cube'],'distance_m':0,'force_contact_frame':[1,0,0,0,0,0]})
        result=experiment.evaluate_grasp(rows,events,9,[-.32,0,.15],criteria,.001)
        self.assertIn('no_nonfinger_support_during_hold',result['failed_criteria'])

    def test_scratch_ik_rejects_unreachable_pose(self):
        self.assertTrue(hasattr(experiment,'solve_ik'),'missing bounded scratch IK planner')
        with tempfile.TemporaryDirectory() as tmp:
            model=build_model(Path(tmp)/'model.xml',load_config(),exclude_base_link1=True)
            c=Controller(model,load_config())
            with self.assertRaises(ValueError):experiment.solve_ik(model,c,[3,0,3],np.diag([-1,1,-1]),c.command[:7])

    def test_reachable_lift_pose_converges_without_relaxing_pose_tolerance(self):
        with tempfile.TemporaryDirectory() as tmp:
            model=build_model(Path(tmp)/'model.xml',load_config(),exclude_base_link1=True)
            c=Controller(model,load_config())
            q=experiment.solve_ik(model,c,[-.32,0,.25],np.diag([-1,1,-1]),
                [-.856,.322,1.109,1.598,-.243,.278,1.396])
            d=mujoco.MjData(model);d.qpos[c.qadr[:7]]=q;mujoco.mj_forward(model,d)
            sid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'grasp_center')
            self.assertLess(np.linalg.norm(d.site_xpos[sid]-[-.32,0,.25]),1e-5)
            self.assertLess(np.linalg.norm(d.site_xmat[sid].reshape(3,3)-np.diag([-1,1,-1])),.00015)

    def test_free_cube_is_unactuated_and_responds_to_gravity(self):
        cube={'center':[-.32,0,.3],'half_size':[.02]*3,'mass_kg':.1,'friction':[1,.005,.0001],
              'table':{'center':[-.32,0,.065],'half_size':[.12,.12,.065]}}
        with tempfile.TemporaryDirectory() as tmp:
            model=build_model(Path(tmp)/'model.xml',load_config(),exclude_base_link1=True,cube=cube)
            c=Controller(model,load_config());data=experiment.reset(model,c,load_config()['initial_action'])
            jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'cube_free')
            qadr=model.jnt_qposadr[jid]
            self.assertEqual((model.nq,model.nv,model.nu,model.neq),(16,15,9,1))
            self.assertTrue(np.all(model.actuator_trnid[:,0]!=jid))
            self.assertEqual(model.eq_type[0],mujoco.mjtEq.mjEQ_JOINT)
            height=float(data.qpos[qadr+2])
            for _ in range(100):c.apply(data);mujoco.mj_step(model,data)
            self.assertLess(data.qpos[qadr+2],height-.04)
            self.assertTrue(np.all(data.qfrc_applied==0))
            self.assertTrue(np.all(data.xfrc_applied==0))

    def test_expert_motion_ramps_joint_targets_synchronously(self):
        with tempfile.TemporaryDirectory() as tmp:
            model=build_model(Path(tmp)/'model.xml',load_config(),exclude_base_link1=True)
            c=Controller(model,load_config());data=experiment.reset(model,c,load_config()['initial_action'])
            recorder=experiment.Recorder(tmp,model,c);target=c.command.copy();target[:2]=[.002,.004]
            try:
                recorder.phase(data,'expert_ramp',target,.1,motion_duration_s=.1)
                self.assertEqual(recorder.actual_steps,100)
                np.testing.assert_allclose(recorder.rows['requested'][0][:2],[0,0],atol=1e-12)
                np.testing.assert_allclose(recorder.rows['requested'][50][:2],[.001,.002],atol=1e-12)
                self.assertLess(np.max(np.abs(np.diff(np.asarray(recorder.rows['command'])[:,:2],axis=0))),.0021)
            finally:recorder.finish()


if __name__=='__main__':unittest.main()
