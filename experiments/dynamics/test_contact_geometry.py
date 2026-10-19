"""Small analytic fixtures for the Issue16 offline geometry calculations."""
import unittest
import numpy as np
from scipy.spatial import ConvexHull
from pathlib import Path
import tempfile
from contact_geometry import local_point, world_point, angle_deg, sat_box_hull, nearest_surface, analyze


class GeometryTests(unittest.TestCase):
    def test_output_cannot_be_inside_archived_source(self):
        with tempfile.TemporaryDirectory() as t:
            source=Path(t)/'source';source.mkdir()
            with self.assertRaisesRegex(ValueError, 'immutable'):
                analyze(source,source/'campaign',source/'new-analysis')
            self.assertFalse((source/'new-analysis').exists())

    def test_nontrivial_rigid_roundtrip_and_normal(self):
        R = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        p, origin = np.array([.3, -.2, .5]), np.array([2., 3., 4.])
        np.testing.assert_allclose(local_point(world_point(p, origin, R), origin, R), p)
        self.assertAlmostEqual(angle_deg(R @ [1., 0., 0.], [0., 1., 0.]), 0.)

    def test_sat_translated_boxes_and_containment(self):
        from itertools import product
        vertices = np.array(list(product([-1., 1.], repeat=3)))
        hull = ConvexHull(vertices)
        common = (vertices, hull, np.eye(3), np.ones(3))
        self.assertAlmostEqual(sat_box_hull(*common, np.array([1.5, 0., 0.]))['depth_m'], .5)
        self.assertAlmostEqual(sat_box_hull(*common, np.array([2.5, 0., 0.]))['depth_m'], -.5)
        self.assertAlmostEqual(sat_box_hull(vertices, hull, np.eye(3), np.ones(3)*.1,
                                           np.zeros(3))['depth_m'], 1.1)

    def test_rotated_boxes_need_the_diagonal_axis(self):
        from itertools import product
        from scipy.spatial.transform import Rotation
        v=np.array(list(product([-1.,1.],repeat=3)))
        R=Rotation.from_euler('z',45,degrees=True).as_matrix()
        got=sat_box_hull(v,ConvexHull(v),R,np.ones(3),np.array([1.9,1.9,0.]))
        self.assertAlmostEqual(got['depth_m'],1.+np.sqrt(2.)-1.9*np.sqrt(2.))

    def test_independent_fk_joint_ref_anchor_slide_and_free(self):
        import xml.etree.ElementTree as ET
        from contact_geometry import xml_body_fk
        root=ET.fromstring('''<mujoco><compiler angle="radian"/><worldbody>
            <body name="arm" pos="1 0 0"><joint axis="0 0 1" ref=".1"/>
            <body name="tip" pos="1 0 0"><joint type="slide" axis="0 1 0"/></body></body>
            <body name="cube"><freejoint/></body></worldbody></mujoco>''')
        body=xml_body_fk(root,np.r_[np.pi/2+.1,.2,3.,4.,5.,1.,0.,0.,0.])
        np.testing.assert_allclose(body['tip'][0],[.8,1.,0.],atol=1e-14)
        np.testing.assert_allclose(body['cube'][0],[3.,4.,5.])
        root.find('worldbody/body/joint').set('pos','1 0 0')
        anchored=xml_body_fk(root,np.r_[np.pi/2+.1,.2,3.,4.,5.,1.,0.,0.,0.])
        np.testing.assert_allclose(anchored['arm'][0],[2.,-1.,0.],atol=1e-14)

    def test_nearest_triangle_is_a_geometric_candidate(self):
        triangles = np.array([[[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],
                              [[0.,0.,2.],[1.,0.,2.],[0.,1.,2.]]])
        got = nearest_surface(triangles, np.array([.2,.2,.3]))
        self.assertAlmostEqual(got['distance_m'], .3)
        self.assertEqual(got['triangle_indices'], [0])
        np.testing.assert_allclose(got['nearest_points_m'][0], [.2,.2,0.])


if __name__ == '__main__':
    unittest.main()
