"""Analytic solid-partition fixtures. No simulator or real query."""
import importlib
from itertools import product
import unittest

import numpy as np
from scipy.spatial import ConvexHull


class PadGeometryTests(unittest.TestCase):
    def setUp(self):
        try:
            self.m = importlib.import_module('fingertip_pads')
        except ImportError:
            self.fail('Missing source-derived pad partition implementation')
        self.cube = np.array(list(product([-1., 1.], repeat=3)))

    def test_halfspace_clip_preserves_the_required_half_solid(self):
        v = self.m.clip(self.cube, np.array([0., 0., 1.]), 0.)
        self.assertAlmostEqual(ConvexHull(v).volume, 4.)
        self.assertLessEqual(v[:, 2].max(), 1e-12)
        np.testing.assert_allclose(v.min(axis=0), [-1, -1, -1])

    def test_front_prism_difference_has_exact_volume_and_membership(self):
        # Width=1, length=1, depth to rear=.25; prism opens to +z.
        planes = [(np.array(a), b) for a, b in
                  [([1.,0,0], .5), ([-1.,0,0], .5),
                   ([0,1.,0], .5), ([0,-1.,0], .5), ([0,0,-1.], .25)]]
        guards = self.m.subtract_prism(self.cube, planes)
        self.assertAlmostEqual(sum(ConvexHull(v).volume for v in guards), 6.75)
        for p in product([-.8, -.3, .2, .7], repeat=3):
            p = np.array(p)
            removed = abs(p[0]) < .5 and abs(p[1]) < .5 and p[2] > -.25
            memberships = [np.all(ConvexHull(v).equations[:,:3]@p +
                                  ConvexHull(v).equations[:,3] < -1e-10) for v in guards]
            self.assertEqual(sum(memberships), 0 if removed else 1)

    def test_pad_outer_face_matches_source_plane_without_outward_offset(self):
        n = np.array([0., -.6, -.8]); u = np.array([1.,0.,0.]); v = np.cross(n,u)
        origin = np.array([.2,.3,.4])
        points = np.array([origin+x*u+y*v for x,y in
                           [(-.01,-.02),(.01,-.02),(-.015,.02),(.015,.02)]])
        patch = dict(source_points_m=points.tolist(), origin_m=origin.tolist(),
                     outward_normal=n.tolist(), tangent_u=u.tolist(), tangent_v=v.tolist())
        pad = self.m.pad_from_patch(patch, .002, 'fixture')
        self.assertAlmostEqual(pad['size_m'][0], .01)
        for corner in self.m.box_vertices(pad):
            self.assertLessEqual(np.dot(n, corner-origin), 1e-12)
        self.assertAlmostEqual(max(np.dot(n, corner-origin) for corner in self.m.box_vertices(pad)), 0.)


if __name__ == '__main__':
    unittest.main()
