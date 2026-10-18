"""Targeted Issue14 wrapper checks; no mj_step or episode execution."""
import copy
import unittest

import mujoco

from full_grasp_loop import candidate_xml, compare_models, action_protocol, sealed_case


class FullGraspChecks(unittest.TestCase):
    def test_only_two_solver_options(self):
        a='<mujoco><option timestep="0.001" /></mujoco>'
        b=candidate_xml(a)
        self.assertEqual(b.replace('cone="elliptic" impratio="10" ',''),a)
        self.assertEqual(compare_models(mujoco.MjModel.from_xml_string(a),mujoco.MjModel.from_xml_string(b))['option_differences'],['cone','impratio'])

    def test_reject_other_physics(self):
        a=mujoco.MjModel.from_xml_string('<mujoco/>')
        b=mujoco.MjModel.from_xml_string('<mujoco><option cone="elliptic" impratio="10" gravity="0 0 -8"/></mujoco>')
        with self.assertRaises(ValueError):compare_models(a,b)

    def test_single_action_scalar_without_mutating_R0(self):
        original={'close_width_rate_m_s':.005,'criteria':{'minimum_lift_m':.08},'cube':{'center':[0,0,.15]}}
        snapshot=copy.deepcopy(original)
        value=action_protocol(original,{'key':'close_width_rate_m_s','value':.0025})
        self.assertEqual(original,snapshot)
        self.assertEqual(value,{**original,'close_width_rate_m_s':.0025})

    def test_reject_unauthorized_or_multiple_changes(self):
        for action in [{'key':'friction','value':2},{'key':'close_width_rate_m_s','value':-.1},
                       {'key':'close_width_rate_m_s','value':.0025,'second':1}]:
            with self.assertRaises(ValueError):action_protocol({'close_width_rate_m_s':.005},action)

    def test_registration_cannot_replace_frozen_case(self):
        case={'id':'fixed','cube':{'center':[-.32,0,.15]},'plan':{'initial_action':[0]*8}}
        plan={'protocol':{},'conditions_and_plans':[case]}
        entry={'strategy':'R0','condition':'fixed','label':'R0','hypothesis':{'action':None},'protocol':{},'case':copy.deepcopy(case)}
        self.assertEqual(sealed_case(plan,entry,'R0'),case)
        entry['case']['cube']['center'][2]=.2
        with self.assertRaises(ValueError):sealed_case(plan,entry,'R0')


if __name__=='__main__':unittest.main()
