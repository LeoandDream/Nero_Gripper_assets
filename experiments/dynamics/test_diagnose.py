"""Pure saved-record fixtures; no MuJoCo import or simulation."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

SPEC = importlib.util.spec_from_file_location('diagnose', Path(__file__).with_name('diagnose.py'))


def contact(body, force, *, reverse=False, distance=-.0001, frame=None):
    bodies = [body, 'cube']; geoms = ['table' if body == 'world' else body + '_geom', 'cube_geom']
    if reverse:
        bodies.reverse(); geoms.reverse()
    return dict(bodies=bodies, geoms=geoms, distance_m=distance, position_m=[0., 0., 0.],
                normal=frame[:3] if frame else [0., 0., 1.], contact_frame=frame or [0., 0., 1., 1., 0., 0., 0., 1., 0.],
                force_contact_frame=force + [0., 0., 0.], constraint_address=0)


def fixture(path):
    phases = np.array(['close', 'close', 'grasp_settle', 'lift', 'lift', 'hold'])
    n = len(phases)
    arrays = {k: np.zeros((n, size)) for k, size in dict(qpos=16, qvel=15, requested=8, command=8,
              physical_target=9, ctrl=9, actuator_force=9, qfrc_constraint=15, qfrc_actuator=15,
              qfrc_applied=15).items()}
    arrays.update(time=np.arange(1, n + 1) * .001, phase=phases, xfrc_applied=np.zeros((n, 14, 6)))
    arrays['qpos'][:, 12] = 1.; arrays['qpos'][:, 11] = .15
    arrays['qpos'][3:, 9] = .01; arrays['qpos'][3, 7] = .0003
    arrays['physical_target'][:, 0] = .1
    arrays['actuator_force'][3:5, 7] = 10.
    rows = []
    for i in range(n):
        events = []
        if i in (0, 1):
            tilted = [.6, 0., .8, .8, 0., -.6, 0., 1., 0.]
            events += [contact('gripper_link1', [.3, .4, 0.], frame=tilted),
                       contact('gripper_link2', [.25, 0., -.1], reverse=True, frame=tilted)]
        if i in (0, 1, 2, 4, 5):
            events += [contact('world', [.01, 0., 0.])]
        if i == 3:
            events += [contact('link7', [.1, 0., 0.], distance=-.0011)]
        rows.append(dict(time=float(arrays['time'][i]), phase=str(phases[i]), events=events))
    protocol = dict(config=dict(timestep_s=.001, arm_force_limits_Nm=[100.] * 7,
                    finger_force_limit_N=10., acceptance=dict(maximum_coupling_error_m=.0002)),
                    grasp_protocol=dict(contact_trigger_force_N_each=.2, criteria=dict(
                        minimum_finger_normal_force_N=.05, maximum_robot_object_penetration_m=.001)),
                    cube=dict(center=[0., 0., .15]), joint_plan={})
    save(path, arrays, rows, protocol)
    return arrays, rows, protocol


def save(path, arrays, rows, protocol):
    np.savez(path / 'trajectory.npz', **arrays)
    (path / 'contacts.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
    (path / 'protocol.json').write_text(json.dumps(protocol))


class DiagnoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.diag = importlib.util.module_from_spec(SPEC)
        SPEC.loader.exec_module(cls.diag)

    def setUp(self):
        self.assertTrue(callable(getattr(self.diag, 'analyze', None)), 'read-only analyze API is missing')
        self.assertTrue(callable(getattr(self.diag, 'write_report', None)), 'exclusive-output API is missing')
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name); self.arrays, self.rows, self.protocol = fixture(self.path)

    def test_world_force_uses_tangents_and_cube_body_order(self):
        # Hand computed tilted normal/tangent: .3*[.6,0,.8]+.4*[.8,0,-.6]=[.5,0,0].
        report = self.diag.analyze(self.path)
        groups = report['phases'][0]['contacts']
        np.testing.assert_allclose(groups['finger1']['world_force_N']['mean'], [.5, 0., 0.], atol=1e-15)
        np.testing.assert_allclose(groups['finger2']['world_force_N']['mean'], [-.15, .1, -.2])
        self.assertAlmostEqual(groups['finger1']['tangent_force_N']['max'], .4)
        self.assertEqual(groups['table']['upward_support_rows'], 2)  # .01N is still support.

    def test_transient_trigger_loss_and_raw_row_references(self):
        report = self.diag.analyze(self.path); events = report['events']
        self.assertEqual(events['bilateral_trigger']['row'], 0)
        self.assertEqual(events['bilateral_support_loss_after_trigger']['row'], 2)
        self.assertEqual(events['robot_penetration']['row'], 3)
        self.assertEqual(events['coupling_limit']['row'], 3)
        self.assertEqual(events['table_geometry_leave']['row'], 3)
        self.assertEqual(events['table_geometry_return']['row'], 4)
        self.assertEqual(events['table_support_leave']['row'], 3)
        self.assertEqual(events['table_support_return']['row'], 4)
        self.assertEqual(events['bilateral_trigger']['contacts'], self.rows[0]['events'])
        self.assertEqual(events['bilateral_trigger']['state']['qpos'], self.arrays['qpos'][0].tolist())
        self.assertEqual(events['grasp_settle_end']['row'], 2)
        self.assertEqual(events['lift_start']['row'], 3)
        self.assertEqual(events['hold_start']['row'], 5)

    def test_contiguous_phases_saturation_and_world_displacement(self):
        report = self.diag.analyze(self.path)
        self.assertEqual([(p['phase'], p['start_row'], p['end_row']) for p in report['phases']],
                         [('close', 0, 1), ('grasp_settle', 2, 2), ('lift', 3, 4), ('hold', 5, 5)])
        lift = report['phases'][2]
        self.assertEqual(lift['joints']['actuator_saturation_fraction'][7], 1.)
        self.assertAlmostEqual(lift['joints']['target_error_abs_max'][0], .1)
        self.assertAlmostEqual(lift['joints']['coupling_residual_abs_max_m'], .0003)
        self.assertEqual(lift['cube']['displacement_from_initial_end_m'], [.01, 0., 0.])

    def test_rejects_short_jsonl_misalignment_gaps_nonfinite_and_bad_frame(self):
        mutations = [lambda a, r: r.pop(), lambda a, r: r[1].update(time=.0015),
                     lambda a, r: r[1].update(phase='hold'),
                     lambda a, r: a['time'].__setitem__(3, .0045),
                     lambda a, r: a['qpos'].__setitem__((1, 0), np.nan),
                     lambda a, r: r[0]['events'][0].update(contact_frame=[1., 0., 0.]),
                     lambda a, r: r[0]['events'][0].update(distance_m=float('nan')),
                     lambda a, r: a.update(ctrl=a['ctrl'][:-1]),
                     lambda a, r: r[0]['events'][0].update(normal=[0., 0., 1.]),
                     lambda a, r: r[0]['events'][0].update(contact_frame=[0.] * 9)]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                arrays, rows, protocol = fixture(self.path); mutate(arrays, rows); save(self.path, arrays, rows, protocol)
                with self.assertRaises(ValueError):
                    self.diag.analyze(self.path)

    def test_rejects_coercible_string_and_boolean_timestep(self):
        for timestep in ('1', True):
            with self.subTest(timestep=timestep):
                arrays, rows, protocol = fixture(self.path)
                protocol['config']['timestep_s'] = timestep
                arrays['time'] *= 1000
                for row in rows:
                    row['time'] *= 1000
                save(self.path, arrays, rows, protocol)
                with self.assertRaises(ValueError):
                    self.diag.analyze(self.path)

    def test_geometry_with_zero_force_is_distinct_from_small_positive_table_support(self):
        self.rows[1]['events'][-1]['force_contact_frame'] = [0.] * 6
        save(self.path, self.arrays, self.rows, self.protocol)
        events = self.diag.analyze(self.path)['events']
        self.assertEqual(events['table_support_leave']['row'], 1)
        self.assertEqual(events['table_support_return']['row'], 2)
        self.assertEqual(events['table_geometry_leave']['row'], 3)

    def test_one_sample_trigger_and_protocol_thresholds(self):
        for e in self.rows[1]['events'][:2]:
            e['force_contact_frame'] = [.01, 0., 0., 0., 0., 0.]
        save(self.path, self.arrays, self.rows, self.protocol)
        events = self.diag.analyze(self.path)['events']
        self.assertEqual(events['bilateral_trigger']['row'], 0)
        self.assertEqual(events['bilateral_support_loss_after_trigger']['row'], 1)
        self.protocol['grasp_protocol']['contact_trigger_force_N_each'] = .4
        save(self.path, self.arrays, self.rows, self.protocol)
        self.assertIsNone(self.diag.analyze(self.path)['events']['bilateral_trigger'])

    def test_output_is_new_directory_with_json_and_stage_csv(self):
        output = self.path / 'analysis'
        self.diag.write_report(self.diag.analyze(self.path), output)
        self.assertEqual(json.loads((output / 'diagnosis.json').read_text())['samples'], 6)
        self.assertEqual(len((output / 'phases.csv').read_text().splitlines()), 5)
        with self.assertRaises(FileExistsError):
            self.diag.write_report(self.diag.analyze(self.path), output)


if __name__ == '__main__':
    unittest.main()
