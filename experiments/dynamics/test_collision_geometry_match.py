"""Artificial saved-geometry fixtures only; no collision or dynamics calls."""
import json
import math
import unittest

import numpy as np

from collision_geometry_match import compare_geometry


def contact(position=(0., 0., 0.), *, distance=0., geoms=('a', 'b'),
            bodies=('body_a', 'body_b'), normal=(1., 0., 0.)):
    return dict(geoms=list(geoms), bodies=list(bodies), distance_m=distance,
                position_m=list(position), normal=list(normal))


class SavedGeometryTests(unittest.TestCase):
    def test_reversing_geom_order_reverses_oriented_normal(self):
        reference = contact()
        query = contact(geoms=('b', 'a'), bodies=('body_b', 'body_a'), normal=(-1., 0., 0.))
        result = compare_geometry([reference], [query])
        self.assertEqual(result['status'], 'PASS')
        self.assertEqual(result['pairs'][0]['matches'][0]['normal_deg'], 0.)
        json.dumps(result, allow_nan=False)
        query['normal'] = [1., 0., 0.]
        self.assertEqual(compare_geometry([reference], [query])['status'], 'FAIL')

    def test_pair_order_is_not_event_order(self):
        a, b = contact(), contact(geoms=('c', 'd'))
        result = compare_geometry([a, b], [b, a])
        self.assertEqual(result['status'], 'PASS')
        self.assertEqual([(p['matches'][0]['reference_event_index'],
                           p['matches'][0]['query_event_index']) for p in result['pairs']],
                         [(0, 1), (1, 0)])

    def test_multiple_contacts_use_minimum_cost_assignment(self):
        events = [contact((x, 0., 0.)) for x in (0., .01, .02)]
        result = compare_geometry(events, [events[2], events[0], events[1]])
        self.assertEqual(result['status'], 'PASS')
        self.assertEqual([m['query_event_index'] for m in result['pairs'][0]['matches']], [1, 2, 0])
        self.assertGreater(result['pairs'][0]['assignment']['alternative_cost'], 0.)

    def test_equal_and_nearly_equal_assignments_are_unknown(self):
        reference = [contact((-1., 0., 0.)), contact((1., 0., 0.))]
        for shift in (0., 1e-10):
            query = [contact((shift, -1., 0.)), contact((0., 1., 0.))]
            result = compare_geometry(reference, query, cost_scales={'position_m': 1.})
            pair = result['pairs'][0]
            self.assertEqual(result['status'], 'UNKNOWN')
            self.assertEqual(pair['matches'], [])
            self.assertEqual(len(pair['best_candidate_matches']), 2)
            self.assertTrue(pair['assignment']['ambiguous'])

    def test_missing_added_pairs_and_contact_counts_remain_visible(self):
        result = compare_geometry([contact(), contact((.1, 0., 0.)),
                                   contact(geoms=('c', 'd'))],
                                  [contact(), contact(geoms=('e', 'f'))])
        self.assertEqual(result['status'], 'FAIL')
        self.assertEqual([(p['reference_count'], p['query_count']) for p in result['pairs']],
                         [(2, 1), (1, 0), (0, 1)])
        self.assertEqual(result['pairs'][0]['missing_reference_event_indices'], [1])
        self.assertEqual(result['pairs'][1]['missing_reference_event_indices'], [2])
        self.assertEqual(result['pairs'][2]['extra_query_event_indices'], [1])

    def test_gate0_tolerances_are_inclusive_and_use_euclidean_position(self):
        at = contact((1e-8, 0., 0.), distance=1e-9)
        self.assertEqual(compare_geometry([contact()], [at])['status'], 'PASS')
        for query in (contact(distance=np.nextafter(1e-9, math.inf)),
                      contact((np.nextafter(1e-8, math.inf), 0., 0.)),
                      contact((.8e-8, .8e-8, 0.))):
            self.assertEqual(compare_geometry([contact()], [query])['status'], 'FAIL')
        for factor, expected in ((1. - 1e-10, 'PASS'), (1., 'PASS'), (1. + 1e-10, 'FAIL')):
            angle = math.radians(1e-5 * factor)
            query = contact(normal=(math.cos(angle), math.sin(angle), 0.))
            self.assertEqual(compare_geometry([contact()], [query])['status'], expected)

    def test_sub_microdegree_normal_angle_keeps_precision(self):
        angle = math.radians(1e-7)
        result = compare_geometry([contact()], [contact(normal=(math.cos(angle), math.sin(angle), 0.))])
        self.assertEqual(result['status'], 'PASS')
        self.assertAlmostEqual(result['pairs'][0]['matches'][0]['normal_deg'], 1e-7, delta=1e-20)

    def test_unique_contact_large_change_is_measurement_fail_not_unknown(self):
        result = compare_geometry([contact()], [contact((.1, .2, .3), distance=-.01,
                                                        normal=(0., 1., 0.))],
                                  cost_scales={'distance_m': .01, 'position_m': 1., 'normal_deg': 90.})
        pair = result['pairs'][0]
        self.assertEqual(result['status'], 'FAIL')
        self.assertEqual(pair['correspondence_status'], 'PASS')
        self.assertEqual(pair['matches'][0]['normal_deg'], 90.)


if __name__ == '__main__':
    unittest.main()
