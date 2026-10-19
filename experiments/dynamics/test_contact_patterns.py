"""Small saved-contact fixtures; no simulator or archived-run execution."""
import unittest
from pathlib import Path

import numpy as np

from contact_patterns import analyze_rows, angle, intervals, run


def event(normal=(0, 1, 0), distance=-0.0005, reverse=False):
    return dict(bodies=['finger', 'cube'] if reverse else ['cube', 'finger'],
                geoms=['finger_geom', 'cube_geom'] if reverse else ['cube_geom', 'finger_geom'],
                normal=(-np.asarray(normal)).tolist() if reverse else list(normal),
                distance_m=distance, position_m=[0, 0, 0])


def analyze(events):
    times = np.arange(1, len(events) + 1) * .001
    rows = [dict(time=float(t), phase='lift', events=e) for t, e in zip(times, events)]
    return analyze_rows(rows, times, np.array(['lift'] * len(rows)),
                        np.repeat(np.eye(3)[None], len(rows), axis=0))


class ContactPatternTests(unittest.TestCase):
    def test_row_contact_count_and_strict_original_limit(self):
        report, over, _, _, _ = analyze([[event(distance=-.0011), event(distance=-.0012)],
                                        [event(distance=-.001)], [event(distance=-.0015)]])
        self.assertEqual(report['over_1mm_contact_records'], 3)
        self.assertEqual(report['over_1mm_physical_rows'], 2)
        self.assertEqual([r['event_index'] for r in over], [0, 1, 0])
        self.assertEqual(report['singleton_intervals'], 2)

    def test_no_index_matching_for_multiple_contacts(self):
        report, _, transitions, _, ambiguous = analyze([[event(), event((0, 0, 1))], [event()]])
        self.assertEqual(transitions, [])
        self.assertEqual(report['ambiguous_adjacent_pair_comparisons'], 1)
        self.assertEqual(ambiguous[0]['previous_contact_count'], 2)

    def test_return_jump_and_sign_reversal(self):
        report, _, transitions, _, _ = analyze([[event()], [event((0, 0, 1), -.0011)], [event(reverse=True)]])
        self.assertAlmostEqual(transitions[0]['world_angle_deg'], 90)
        self.assertAlmostEqual(transitions[1]['world_angle_deg'], 90)
        stats = report['descriptive_angle_thresholds_deg']['60']
        self.assertEqual(stats['bounce_centers_over_1mm'], 1)
        self.assertEqual(stats['either_endpoint_over_1mm'], 2)
        self.assertAlmostEqual(angle([0, 1, 0], [0, 1, 0]), 0)

    def test_no_comparison_across_absent_row(self):
        report, _, transitions, _, _ = analyze([[event()], [], [event((0, 0, 1))]])
        self.assertEqual(report['adjacent_unique_pair_comparisons'], 0)
        self.assertEqual(transitions, [])

    def test_interval_multiplicity_does_not_inflate_physical_length(self):
        rows = [dict(row=i, time_s=(i + 1) * .001, distance_m=-.002, event_index=j)
                for i, j in [(3, 0), (3, 1), (4, 0), (7, 0)]]
        chunks = intervals(rows)
        self.assertEqual([(c['physical_rows'], c['contact_records']) for c in chunks], [(2, 3), (1, 1)])

    def test_refuses_output_inside_archived_campaign(self):
        campaign = Path(__file__).resolve().parent
        with self.assertRaisesRegex(ValueError, 'outside archived campaign'):
            run(campaign, campaign / 'unused.json', campaign / 'forbidden-output')


if __name__ == '__main__':
    unittest.main()
