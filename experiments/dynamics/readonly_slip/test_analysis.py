"""Small, zero-integration checks of force signs and analysis boundaries."""
import unittest
import numpy as np
try:
    import analyze as subject
except ModuleNotFoundError:
    subject = None


class ReadonlyMathTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(subject, 'Read-only reducer is not implemented')

    def test_force_sign_decomposition_and_moment_arm(self):
        e={'bodies':['finger','cube'],'contact_frame':[0,1,0,0,0,1,1,0,0],
           'force_contact_frame':[2,-.5,.3,0,0,0]}
        normal,tangent,total=subject.force_parts(e)
        np.testing.assert_allclose(normal,[0,2,0])
        np.testing.assert_allclose(tangent,[.3,0,-.5])
        np.testing.assert_allclose(np.cross([.02,0,.01],total),[-.02,.013,.04])
        e['bodies'].reverse()
        np.testing.assert_allclose(subject.force_parts(e)[2],[-.3,-2,.5])

    def test_half_open_windows_exclude_table_sample(self):
        ticks=np.array([1,140,141,1327,1328,2163,2164,4415,4416])
        got=subject.window_masks(ticks,4416)
        self.assertEqual([list(np.flatnonzero(v)) for v in got],[[0,1],[2,3],[4,5],[6,7]])

    def test_body_local_point_rotation(self):
        R=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        np.testing.assert_allclose(subject.local_point(R,[1,1,0],[1,3,0]),[2,0,0])

    def test_elliptic_ratio_is_scaled_l2_not_l1(self):
        self.assertAlmostEqual(subject.rho_elliptic([2,1.2,1.6],[.6,.8]),np.sqrt(2))
        self.assertIsNone(subject.rho_elliptic([0,0,0],[1,1]))


if __name__=='__main__':unittest.main()
