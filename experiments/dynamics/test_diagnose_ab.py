"""Saved A/B records only: no model loading, integration, or acceptance changes."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np


SOURCE = Path(__file__).with_name('diagnose_ab.py')


def contact(body, force, *, reverse=False, distance=-.0001, frame=None):
    frame = frame or [.6, 0., .8, .8, 0., -.6, 0., 1., 0.]
    bodies = [body, 'cube']; geoms = ['table' if body == 'world' else body + '_geom', 'cube_geom']
    if reverse:
        bodies.reverse(); geoms.reverse()
    return dict(bodies=bodies, geoms=geoms, distance_m=distance, position_m=[0., 0., .15],
                normal=frame[:3], contact_frame=frame, constraint_address=0,
                force_contact_frame=force + [0., 0., 0.])


def fixture(path, *, table_rows=(2, 3), table_zero_row=2):
    path.mkdir(exist_ok=True)
    n = 4; dt = .001; start = 29.128
    widths = dict(qpos=16, qvel=15, requested=8, command=8, physical_target=9, ctrl=9,
                  actuator_force=9, qfrc_actuator=15, qfrc_constraint=15, qfrc_applied=15)
    arrays = {k: np.zeros((n, w)) for k, w in widths.items()}
    arrays.update(time=start + np.arange(1, n + 1) * dt, phase=np.array(['ab_hold'] * n),
                  xfrc_applied=np.zeros((n, 14, 6)))
    arrays['qpos'][:, 11] = [.15, .1495, .148, .147]; arrays['qpos'][:, 12] = 1.
    arrays['qvel'][:, 11] = [0., -.001, -.002, -.003]
    arrays['qpos'][2:, 7] = .0003
    arrays['actuator_force'][2:, 7] = 10.
    observations = dict(time=arrays['time'].copy(), gripper_position_world_m=np.zeros((n, 3)),
                        gripper_rotation_world=np.repeat(np.eye(3)[None], n, axis=0),
                        cube_position_in_gripper_m=np.column_stack([np.zeros(n), np.zeros(n), [.15, .1495, .148, .147]]),
                        cube_rotation_in_gripper=np.repeat(np.eye(3)[None], n, axis=0))
    initial = dict(qpos=arrays['qpos'][0].copy(), qvel=np.zeros(15), time=np.array([start]),
                   integration_state=np.zeros(6), controller_requested=np.zeros(8), controller_command=np.zeros(8),
                   gripper_position_world_m=np.zeros(3), gripper_rotation_world=np.eye(3),
                   cube_position_in_gripper_m=np.array([0., 0., .15]), cube_rotation_in_gripper=np.eye(3))
    rows = []
    for i in range(n):
        events = [contact('gripper_link1', [.3, .4, 0.]),
                  contact('gripper_link2', [.25, 0., -.1], reverse=True)]
        if i in table_rows:
            # At first geometry the solver force is zero; later a tiny positive force is support.
            events.append(contact('world', [0. if i == table_zero_row else 1e-10, 0., 0.],
                                  frame=[0., 0., 1., 1., 0., 0., 0., 1., 0.]))
        if i == 2:
            events.append(contact('link7', [.1, 0., 0.], distance=-.0011))
        rows.append(dict(time=float(arrays['time'][i]), phase='ab_hold', events=events))
    protocol = dict(config=dict(timestep_s=dt, arm_force_limits_Nm=[100.] * 7, finger_force_limit_N=10.,
                               acceptance=dict(maximum_coupling_error_m=.0002)),
                    grasp_protocol=dict(contact_trigger_force_N_each=.2, criteria=dict(
                        minimum_finger_normal_force_N=.05, maximum_robot_object_penetration_m=.001)),
                    cube=dict(center=[0., 0., .15]), joint_plan={}, diagnostic_only=True, snapshot_time_s=start)
    save(path, arrays, rows, protocol, observations, initial)
    return arrays, rows, protocol, observations, initial


def save(path, arrays, rows, protocol, observations, initial):
    np.savez(path / 'trajectory.npz', **arrays)
    np.savez(path / 'observations.npz', **observations)
    np.savez(path / 'initial-state.npz', **initial)
    (path / 'contacts.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
    (path / 'protocol.json').write_text(json.dumps(protocol))


class DiagnoseABTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.diag = None
        if SOURCE.exists():
            spec = importlib.util.spec_from_file_location('diagnose_ab', SOURCE)
            cls.diag = importlib.util.module_from_spec(spec); spec.loader.exec_module(cls.diag)

    def setUp(self):
        self.assertTrue(callable(getattr(self.diag, 'analyze_pair', None)), 'read-only A/B analysis API is missing')
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name); self.a = self.root / 'a'; self.b = self.root / 'b'
        fixture(self.a, table_rows=(0, 1, 2, 3), table_zero_row=-1)
        self.saved = fixture(self.b)

    def test_splits_at_first_table_geometry_not_later_small_positive_support(self):
        report = self.diag.analyze_pair(self.a, self.b); b = report['groups']['B']
        self.assertEqual(b['table_boundary'], 'first_recorded_contact_after_absence')
        self.assertEqual(b['events']['first_table_geometry']['row'], 2)
        self.assertEqual(b['events']['first_table_upward_support']['row'], 3)
        before, after = b['intervals']
        self.assertEqual((before['start_row'], before['end_row'], before['samples']), (0, 1, 2))
        self.assertEqual((after['start_row'], after['end_row'], after['samples']), (2, 3, 2))
        self.assertEqual(before['contacts']['table']['geometric_rows'], 0)
        self.assertEqual(after['contacts']['table']['geometric_rows'], 2)
        self.assertEqual(after['contacts']['table']['upward_support_rows'], 1)
        self.assertAlmostEqual(after['contacts']['table']['world_force_N']['mean'][2], 5e-11)
        a = report['groups']['A']
        self.assertEqual(a['table_boundary'], 'present_at_first_record')
        self.assertEqual(a['intervals'][0]['samples'], 0)
        self.assertIsNone(a['intervals'][0]['contacts'])

    def test_world_force_tangents_body_order_and_slip_are_retained(self):
        b = self.diag.analyze_pair(self.a, self.b)['groups']['B']; before, after = b['intervals']
        np.testing.assert_allclose(before['contacts']['finger1']['world_force_N']['mean'], [.5, 0., 0.], atol=1e-15)
        np.testing.assert_allclose(before['contacts']['finger2']['world_force_N']['mean'], [-.15, .1, -.2])
        self.assertAlmostEqual(before['contacts']['finger1']['tangent_force_N']['mean'], .4)
        self.assertAlmostEqual(after['cube']['relative_z_drop_m']['max'], .003)
        self.assertAlmostEqual(after['cube']['world_z_m']['min'], .147)
        self.assertAlmostEqual(after['cube']['world_vz_m_s']['min'], -.003)
        self.assertAlmostEqual(after['maximum_robot_penetration_m'], .0011)
        self.assertAlmostEqual(after['coupling_residual_abs_max_m'], .0003)
        self.assertEqual(after['actuator_saturation_fraction_9d'], [0.] * 7 + [1., 0.])
        self.assertIsNone(b['official_grasp_success'])

    def test_descriptive_fall_events_include_raw_data_absolute_and_snapshot_relative_times(self):
        b = self.diag.analyze_pair(self.a, self.b)['groups']['B']; events = b['events']
        self.assertEqual(events['first_world_vz_below_minus_1mm_s']['row'], 2)  # strict <, row 1 equals threshold
        self.assertEqual(events['first_relative_z_drop_at_least_1mm']['row'], 2)
        e = events['first_both_fall_descriptors']; self.assertEqual(e['row'], 2)
        self.assertAlmostEqual(e['time_s'], 29.131); self.assertAlmostEqual(e['relative_time_s'], .003)
        self.assertEqual(e['source']['jsonl_line'], 3)
        self.assertEqual(e['contacts'], self.saved[1][2]['events'])
        self.assertEqual(e['state']['qpos'], self.saved[0]['qpos'][2].tolist())
        self.assertEqual(e['observations']['cube_position_in_gripper_m'], [0., 0., .148])
        self.assertEqual(events['first_robot_penetration_exceedance']['row'], 2)
        self.assertEqual(events['first_coupling_exceedance']['row'], 2)
        self.assertEqual(len(b['sources']['trajectory.npz']['sha256']), 64)

    def test_no_recontact_preserves_entire_precontact_interval_and_null_events(self):
        fixture(self.b, table_rows=())
        b = self.diag.analyze_pair(self.a, self.b)['groups']['B']
        self.assertEqual(b['table_boundary'], 'not_observed')
        self.assertIsNone(b['events']['first_table_geometry'])
        self.assertIsNone(b['events']['first_table_upward_support'])
        self.assertEqual(b['intervals'][0]['samples'], 4)
        self.assertEqual(b['intervals'][1]['samples'], 0)

    def test_world_relative_drop_survives_upside_down_gripper_local_z_increase(self):
        flipped = np.diag([-1., 1., -1.])
        for path in (self.a, self.b):
            state = fixture(path)
            arrays, _, _, observations, initial = state
            initial['gripper_position_world_m'] = np.array([0., 0., .2])
            initial['gripper_rotation_world'] = flipped.copy()
            initial['cube_position_in_gripper_m'] = np.array([0., 0., .05])
            observations['gripper_position_world_m'][:] = [0., 0., .2]
            observations['gripper_rotation_world'][:] = flipped
            observations['cube_position_in_gripper_m'][:, 2] = .2 - arrays['qpos'][:, 11]
            save(path, *state)
        b = self.diag.analyze_pair(self.a, self.b)['groups']['B']
        event = b['events']['first_relative_z_drop_at_least_1mm']
        self.assertIsNotNone(event, 'world downward slip must survive flipped local Z')
        self.assertEqual(event['row'], 2)
        self.assertAlmostEqual(event['cube_relative_delta_m'][2], .002)
        self.assertAlmostEqual(event['cube_relative_z_drop_m'], .002)
        self.assertEqual(b['events']['first_both_fall_descriptors']['row'], 2)

    def test_rejects_time_shape_nonfinite_contact_frame_and_threshold_corruption(self):
        mutations = [lambda a, r, p, o, s: a['time'].__setitem__(0, 0.),
                     lambda a, r, p, o, s: a['time'].__setitem__(2, 29.1315),
                     lambda a, r, p, o, s: o['time'].__setitem__(1, 29.1305),
                     lambda a, r, p, o, s: o.update(cube_position_in_gripper_m=np.zeros((4, 2))),
                     lambda a, r, p, o, s: a['qpos'].__setitem__((1, 0), np.nan),
                     lambda a, r, p, o, s: s['time'].__setitem__(0, np.inf),
                     lambda a, r, p, o, s: o['gripper_rotation_world'].__setitem__((0, 0, 0), 2.),
                     lambda a, r, p, o, s: r.pop(),
                     lambda a, r, p, o, s: r[1].update(time=0.),
                     lambda a, r, p, o, s: r[1]['events'][0].update(contact_frame=[0.] * 9),
                     lambda a, r, p, o, s: p['config'].update(timestep_s='0.001'),
                     lambda a, r, p, o, s: p['config'].update(finger_force_limit_N=True),
                     lambda a, r, p, o, s: p['grasp_protocol']['criteria'].update(minimum_finger_normal_force_N=-1.)]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                state = fixture(self.b); mutate(*state); save(self.b, *state)
                with self.assertRaises(ValueError): self.diag.analyze_pair(self.a, self.b)

    def test_rejects_changed_initial_state_controller_or_protocol_between_groups(self):
        mutations = [lambda a, r, p, o, s: s['qpos'].__setitem__(0, .1),
                     lambda a, r, p, o, s: s['integration_state'].__setitem__(0, .1),
                     lambda a, r, p, o, s: s['controller_command'].__setitem__(0, .1),
                     lambda a, r, p, o, s: p['config']['arm_force_limits_Nm'].__setitem__(0, 99.)]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                state = fixture(self.b); mutate(*state); save(self.b, *state)
                with self.assertRaises(ValueError): self.diag.analyze_pair(self.a, self.b)

    def test_allows_only_approved_table_z_and_post_forward_derived_cache_difference(self):
        aa = fixture(self.a, table_rows=(0, 1, 2, 3)); bb = fixture(self.b)
        for state, z in [(aa, .065), (bb, .015)]:
            state[2]['cube']['table'] = dict(center=[.3, .2, z], halfsize=[.2, .2, .065])
            state[4]['restored_integration_before_forward'] = np.zeros(6)
            state[4]['restored_integration_after_forward'] = np.zeros(6)
        bb[4]['restored_integration_after_forward'][2] = 54.8
        save(self.a, *aa); save(self.b, *bb)
        report = self.diag.analyze_pair(self.a, self.b)
        self.assertEqual(report['matched_initial_state_and_protected_protocol'], 'PASS')
        self.assertEqual(report['approved_table_translation_m'], dict(A=.065, B=.015))
        self.assertEqual(report['derived_after_forward_state_max_delta'], 54.8)
        for mutate in [lambda b: b[2]['cube']['table']['center'].__setitem__(0, .31),
                       lambda b: b[2]['cube']['table']['center'].__setitem__(2, .014),
                       lambda b: b[2]['cube']['table']['halfsize'].__setitem__(2, .064),
                       lambda b: b[4]['restored_integration_before_forward'].__setitem__(2, .1)]:
            with self.subTest(mutation=mutate):
                bb = fixture(self.b)
                bb[2]['cube']['table'] = dict(center=[.3, .2, .015], halfsize=[.2, .2, .065])
                bb[4]['restored_integration_before_forward'] = np.zeros(6)
                bb[4]['restored_integration_after_forward'] = np.zeros(6)
                mutate(bb); save(self.b, *bb)
                with self.assertRaises(ValueError): self.diag.analyze_pair(self.a, self.b)

    def test_output_new_directory_refuses_overwrite_and_serializes_empty_intervals(self):
        report = self.diag.analyze_pair(self.a, self.b); output = self.root / 'analysis'
        self.diag.write_report(report, output)
        self.assertEqual(json.loads((output / 'diagnosis-ab.json').read_text())['schema_version'], 1)
        self.assertEqual(len((output / 'intervals.csv').read_text().splitlines()), 5)
        self.assertTrue((output / 'events.csv').exists())
        with self.assertRaises(FileExistsError): self.diag.write_report(report, output)


if __name__ == '__main__': unittest.main()
