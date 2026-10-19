"""Hand-checked saved-geometry fixtures; no simulator import or query."""
import math
from pathlib import Path
import tempfile
import unittest

import diagnose_fixed_collision as reduce
from collision_geometry_match import compare_geometry


PAIR = ['cube_geom', 'gripper_link2_collision_000']


def event(depth=.002, angle=0., *, geoms=None, reverse=False, point=0.):
    normal = [math.cos(math.radians(angle)), math.sin(math.radians(angle)), 0.]
    e = dict(geoms=PAIR[:] if geoms is None else list(geoms), bodies=['cube', 'gripper_link2'],
             distance_m=-depth, position_m=[point, 0., 0.], normal=normal)
    if reverse:
        e['geoms'].reverse(); e['bodies'].reverse(); e['normal'] = [-v for v in normal]
    return e


def record(row, a, b, label='R1'):
    return dict(label=label, row=row, phase='lift', archive=a, A=a, B=b,
                A_B_comparison=compare_geometry(a, b, cost_scales=reduce.AB_SCALES))


def predictions():
    return dict(H1=dict(row=23623, preceding_row=23622, geom_pair=PAIR,
        independent_MTD_reference_m=.00075, error_reduction_fraction_at_least=.5,
        native_incoming_normal_jump_strictly_below_deg=20., conjunction_required=True),
        H2=dict(rows=list(range(25984, 25992)), predicted_any_native_robot_cube_penetration_over_m=.001))


class SavedCollisionReductionTests(unittest.TestCase):
    def test_h1_requires_both_conditions_and_marks_missing_contact_invalid(self):
        # Wrong OR, non-strict normal boundary, or rejected exact 50% improvement fail here.
        pred = predictions()
        before = record(23622, [event(.002)], [event(.001, reverse=True)])
        for depth, angle, expected in [(.001375, 0., 'PASS'), (.001375, 20., 'FAIL'),
                                       (.0015, 0., 'FAIL'), (.001, 30., 'FAIL')]:
            with self.subTest(depth=depth, angle=angle):
                result = reduce.evaluate_h1([before, record(23623, [event(.002)], [event(depth, angle)])], pred)
                self.assertEqual(result['status'], expected)
        result = reduce.evaluate_h1([before, record(23623, [event(.002)], [])], pred)
        self.assertEqual(result['status'], 'INVALID')
        ambiguous = record(23623, [event(.002), event(.002)], [event(.001), event(.001)])
        self.assertEqual(reduce.evaluate_h1([before, ambiguous], pred)['status'], 'INVALID')

    def test_h2_records_each_contact_and_keeps_one_mm_boundary_strict(self):
        rows = [record(i, [event(.002)], [event(.001 if i == 25984 else .001000001)])
                for i in range(25984, 25992)]
        result = reduce.evaluate_h2(rows, predictions())
        self.assertFalse(result['rows'][0]['B_any_robot_cube_over_1mm'])
        self.assertEqual(result['B_rows_with_any_robot_cube_over_1mm'], 7)
        self.assertEqual(len(result['rows'][0]['contacts']), 1)
        self.assertEqual(result['status'], 'OBSERVED')
        self.assertNotIn('PASS', result.values())

    def test_counts_separate_contacts_from_pose_rows_and_preserve_missing_extra(self):
        # Two >1mm contacts at one pose must never become two physical rows.
        old = [event(.002, point=0.), event(.003, point=.02)]
        rows = [record(1, old, [event(.0008, point=0.)]),
                record(2, [], [event(.002, geoms=['cube_geom', 'other_geom'])])]
        result, contacts = reduce.reduce_geometry(rows)
        label = result['R1']
        self.assertEqual(label['A']['robot_cube_over_1mm_contacts'], 2)
        self.assertEqual(label['A']['robot_cube_over_1mm_pose_rows'], 1)
        self.assertEqual(label['B']['robot_cube_over_1mm_contacts'], 1)
        self.assertEqual(label['B']['robot_cube_over_1mm_pose_rows'], 1)
        self.assertEqual(label['reported_over_1mm_pose_changes']['eliminated_rows'], [1])
        self.assertEqual(label['reported_over_1mm_pose_changes']['new_rows'], [2])
        self.assertEqual(label['contact_correspondence_status'], 'UNKNOWN')
        self.assertEqual({r['correspondence_status'] for r in contacts}, {'MATCHED', 'MISSING_B_UNKNOWN', 'EXTRA_B_UNKNOWN'})
        self.assertEqual(len(contacts), 3)
        self.assertTrue(all(r['contact_feature_id'] == 'UNKNOWN' for r in contacts))
        self.assertEqual(label['A_B_measurement_status_counts'], {'FAIL': 2})

    def test_incomplete_campaign_and_existing_output_never_produce_results(self):
        with tempfile.TemporaryDirectory() as t:
            campaign = Path(t) / 'campaign'; campaign.mkdir()
            output = Path(t) / 'output'
            with self.assertRaisesRegex(ValueError, 'completed'):
                reduce.analyze(campaign, output)
            self.assertFalse(output.exists())
            output.mkdir()
            with self.assertRaises(FileExistsError):
                reduce.analyze(campaign, output)
            self.assertEqual(list(output.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
