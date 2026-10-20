"""Small semantic checks: no rendering or integration."""
import unittest
import numpy as np

from mvp_lerobot_episode import selected_rows, frame_values


class EpisodeSemantics(unittest.TestCase):
    def test_real_grid_has_a_recorded_successor(self):
        times=np.arange(1,151)*.001
        rows=selected_rows(times)
        np.testing.assert_array_equal(rows,[49,99])
        self.assertTrue(np.all(rows+1<len(times)))
        np.testing.assert_allclose(times[rows],[.05,.10])

    def test_next_requested_is_action_not_ctrl_or_measured_state(self):
        a={'qpos':np.arange(4*16).reshape(4,16),
           'requested':np.arange(4*8).reshape(4,8)+1000,
           'command':np.ones((4,8))*2000,'ctrl':np.ones((4,9))*3000}
        state,action=frame_values(a,1)
        np.testing.assert_array_equal(state,a['qpos'][1,:9])
        np.testing.assert_array_equal(action,a['requested'][2])
        self.assertEqual((state.shape,action.shape),((9,),(8,)))

    def test_nero_has_one_real_camera_and_preserves_eight_actions(self):
        from nero_openpi import NeroInputs, NeroOutputs
        actions=np.arange(80,dtype=np.float32).reshape(10,8)
        value=NeroInputs()({'observation/state':np.arange(9,dtype=np.float32),
            'observation/image':np.zeros((3,256,256),dtype=np.float32),
            'actions':actions,'actions_is_pad':np.array([False]*7+[True]*3),'prompt':'pick cube'})
        self.assertEqual(value['state'].shape,(9,))
        self.assertEqual(value['image_mask'],{'base_0_rgb':True,'left_wrist_0_rgb':False,'right_wrist_0_rgb':False})
        np.testing.assert_array_equal(value['actions_is_pad'],[False]*7+[True]*3)
        padded=np.pad(value['actions'],((0,0),(0,24)))
        np.testing.assert_array_equal(NeroOutputs()({'actions':padded})['actions'],actions)


if __name__=='__main__':unittest.main()
