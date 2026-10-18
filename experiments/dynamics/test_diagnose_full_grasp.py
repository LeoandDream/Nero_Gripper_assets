"""Small saved-record fixtures; no physics model, integration or force solve."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('diagnose_full_grasp', HERE / 'diagnose_full_grasp.py')
diag = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diag)


def contact(body, normal, *, distance=-.0001):
    return dict(bodies=[body, 'cube'], geoms=['table' if body == 'world' else body + '_geom', 'cube_geom'],
                position_m=[0., 0., 0.], normal=[0., 0., 1.],
                contact_frame=[0., 0., 1., 1., 0., 0., 0., 1., 0.],
                force_contact_frame=[normal, 0., 0., 0., 0., 0.],
                distance_m=distance, constraint_address=0)


def fixture(path, *, hold_loss=False, early_penetration=False, never_held=False, missing_release=False, lift_height_m=.1):
    phases = ['open', 'approach', 'approach', 'close', 'close', 'grasp_settle', 'grasp_settle',
              'lift', 'lift', 'hold', 'hold', 'hold', 'release', 'release', 'release']
    if missing_release: phases = phases[:-3]
    n = len(phases)
    cfg = json.loads((HERE / 'config.json').read_text())
    gp = json.loads((HERE / 'grasp.json').read_text())
    gp['criteria']['hold_duration_s'] = .003
    a = {key: np.zeros((n, width)) for key, width in diag._saved.WIDTHS.items()}
    a.update(time=np.arange(1, n + 1) * .001, phase=np.asarray(phases), xfrc_applied=np.zeros((n, 14, 6)))
    a['qpos'][:, 9:12] = [-.32, 0., .15]
    a['qpos'][:, 12] = 1.
    a['qpos'][7:, 11] = .15 + lift_height_m if not never_held else .15
    a['qpos'][:, 7] = .025; a['qpos'][:, 8] = -.025
    a['command'][:, 7] = .05; a['requested'][:, 7] = .05
    if not missing_release:
        a['command'][12:, 7] = [.06, .08, .1]
        a['requested'][12:, 7] = .1
        a['qpos'][12:, 7] = [.03, .04, .05]
        a['qpos'][12:, 8] = [-.03, -.04, -.05]
        if not never_held:
            a['qpos'][12:, 11] = .15 + lift_height_m - np.array([0., .01, .02]); a['qvel'][13:, 11] = -.1
    a['physical_target'][:] = a['qpos'][:, :9]
    observations = dict(time=a['time'], gripper_position_world_m=a['qpos'][:, 9:12] - [0., 0., .12],
                        gripper_rotation_world=np.broadcast_to(np.eye(3), (n, 3, 3)),
                        cube_position_in_gripper_m=np.broadcast_to([0., 0., .12], (n, 3)),
                        cube_rotation_in_gripper=np.broadcast_to(np.eye(3), (n, 3, 3)))
    # A real drop is relative to gripper as well as world, while release arm is fixed.
    if not missing_release and not never_held:
        observations['gripper_position_world_m'][12:] = [-.32, 0., .15 + lift_height_m - .12]
        observations['cube_position_in_gripper_m'] = (a['qpos'][:, 9:12] - observations['gripper_position_world_m'])
    rows = []
    for i, phase in enumerate(phases):
        events = []
        if i <= 6 or never_held: events.append(contact('world', .981))
        if 3 <= i <= 12 and not (hold_loss and phase == 'hold'):
            events.extend([contact('gripper_link1', .3), contact('gripper_link2', .3)])
        if early_penetration and i == 1: events.append(contact('link7', .1, distance=-.0011))
        rows.append(dict(time=float(a['time'][i]), phase=phase, events=events))
    # Two adjacent approach chunks and two lift chunks deliberately share labels.
    bounds = [(0, 1), (1, 2), (2, 3), (3, 5), (5, 7), (7, 8), (8, 9), (9, 12)]
    if not missing_release: bounds += [(12, 15)]
    chunks = [dict(phase=phases[start], start_time_s=float(start * .001), end_time_s=float(stop * .001),
                   target_8d=[0.] * 7 + [.1], status='PASS', loaded_fingers=start >= 3,
                   settled_max_error_9d=[0.] * 9) for start, stop in bounds]
    control = dict(status='PASS', actual_mj_steps=n, recorded_physics_samples=n, error=None, phases=chunks)
    protocol = dict(config=cfg, grasp_protocol=gp, cube=gp['cube'], joint_plan=dict(
        initial_action=[0.] * 7 + [.08], nonobject_scratch_collision_count=0,
        approach=[[0.] * 7] * 2, lift=[[0.] * 7] * 2))
    evaluator, _ = diag.frozen_evaluator()
    original = evaluator(a, rows, 9, protocol['cube']['center'], gp['criteria'], .001)
    original.update(control_health_status=control['status'], actual_mj_steps=n, error=None, bilateral_contact_trigger={})
    for name, value in [('protocol.json', protocol), ('result.json', control), ('grasp-result.json', original)]:
        (path / name).write_text(json.dumps(value))
    (path / 'model.xml').write_text('<mujoco/>')
    np.savez(path / 'trajectory.npz', **a)
    np.savez(path / 'observations.npz', **observations)
    np.savez(path / 'warnings.npz', time=a['time'], numbers=np.zeros((n, 8), dtype=int))
    initial = {key: np.asarray(value[0]) for key, value in observations.items() if key != 'time'}
    initial.update(qpos=a['qpos'][0], qvel=a['qvel'][0], time=np.array([0.]))
    np.savez(path / 'initial-state.npz', **initial)
    (path / 'contacts.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
    (path / 'contact-parameters.jsonl').write_text(''.join(json.dumps(dict(time=row['time'],
        contacts=[dict(index=j) for j in range(len(row['events']))])) + '\n' for row in rows))
    return a, rows


def interrupted_fixture(path, saved_rows):
    a, rows = fixture(path)
    for name in ('trajectory.npz', 'observations.npz', 'warnings.npz'):
        arrays = diag._npz(path / name)
        np.savez(path / name, **{key: value[:saved_rows] for key, value in arrays.items()})
    saved_contacts = rows[:saved_rows]
    if saved_rows == 4:
        saved_contacts[-1]['events'] = [contact('world', .981)]  # Interrupted before any bilateral close trigger.
    (path / 'contacts.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in saved_contacts))
    (path / 'contact-parameters.jsonl').write_text(''.join(json.dumps(dict(time=row['time'],
        contacts=[{}] * len(row['events']))) + '\n' for row in saved_contacts))
    control = json.loads((path / 'result.json').read_text())
    control.update(status='INTERRUPTED', actual_mj_steps=saved_rows + 1, recorded_physics_samples=saved_rows,
                   error='Recorder rejected a numerical warning')
    control['phases'] = [p for p in control['phases'] if p['end_time_s'] <= saved_rows * .001 + 1e-10]
    protocol = json.loads((path / 'protocol.json').read_text())
    if saved_rows:
        original = diag.frozen_evaluator()[0]({key: value[:saved_rows] for key, value in a.items()},
            saved_contacts, 9, protocol['cube']['center'], protocol['grasp_protocol']['criteria'], .001)
    else:
        original = dict(status='INTERRUPTED', failed_criteria=['no_physics_samples'])
        (path / 'initial-state.npz').unlink()  # Early failures can happen before initial snapshot is written.
    original.update(status='INTERRUPTED', error=control['error'], control_health_status='INTERRUPTED',
                    actual_mj_steps=saved_rows + 1, bilateral_contact_trigger=None)
    (path / 'result.json').write_text(json.dumps(control))
    (path / 'grasp-result.json').write_text(json.dumps(original))
    np.savez(path / 'failure-state.npz', time=[(saved_rows + 1) * .001], qpos=a['qpos'][saved_rows],
             qvel=a['qvel'][saved_rows], warning_numbers=[1, 0, 0, 0, 0, 0, 0, 0])
    (path / 'failure-state.json').write_text(json.dumps(dict(phase=str(a['phase'][saved_rows]),
        actual_steps=saved_rows + 1, saved_rows=saved_rows, error=control['error'])))
    (path / 'failure-contacts.json').write_text(json.dumps(rows[saved_rows]['events']))


class FullGraspDiagnosisTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'raw'; self.path.mkdir()

    def test_success_does_not_require_finger_forces_during_approach_and_preserves_chunks(self):
        fixture(self.path)
        report = diag.analyze(self.path)
        self.assertEqual(report['status'], 'PASS')
        self.assertEqual(report['frozen_evaluation']['agreement'], 'PASS')
        self.assertEqual([s['status'] for s in report['stages']], ['PASS'] * 7)
        self.assertEqual(len(report['recorder_phase_chunks']), 9)
        self.assertEqual(report['stages'][1]['frozen_control_chunks'], [1, 2])
        self.assertEqual(report['stages'][4]['frozen_control_chunks'], [5, 6])
        self.assertEqual(report['rows_without_completed_recorder_phase'], 0)
        self.assertEqual(report['source_sha256_before'], report['source_sha256_after'])
        self.assertTrue(report['release']['cube_fall_after_release_observed'])

    def test_first_failure_keeps_actual_qpos_and_contact_event_index(self):
        a, _ = fixture(self.path, early_penetration=True)
        report = diag.analyze(self.path)
        first = report['earliest_violation']
        self.assertEqual(first['event'], 'first_penetration_violation')
        self.assertEqual(first['row'], 1)
        self.assertEqual(first['evidence']['qpos'], a['qpos'][1].tolist())
        self.assertEqual(first['evidence']['contact_events'][1]['event_index'], 1)
        self.assertEqual(report['frozen_evaluation']['agreement'], 'PASS')
        # An approach penetration is not misreported as a hold-stage violation.
        self.assertEqual(report['stages'][5]['status'], 'PASS')

    def test_hold_fraction_first_loss_is_descriptive_and_only_hold_is_counted(self):
        fixture(self.path, hold_loss=True)
        report = diag.analyze(self.path)
        self.assertEqual(report['events']['hold_first_bilateral_support_loss']['row'], 9)
        self.assertFalse(report['events']['hold_first_bilateral_support_loss']['failure'])
        self.assertEqual(report['events']['hold_bilateral_loss_exceeds_full_duration_budget']['row'], 9)
        self.assertEqual(report['stages'][0]['status'], 'PASS')
        self.assertEqual(report['stages'][1]['status'], 'PASS')
        self.assertEqual(report['stages'][5]['status'], 'FAIL')

    def test_opening_empty_fingers_does_not_claim_a_successful_release(self):
        fixture(self.path, never_held=True)
        report = diag.analyze(self.path)
        self.assertEqual(report['release']['status'], 'UNKNOWN')
        self.assertTrue(report['release']['actual_opening_reached'])
        self.assertFalse(report['release']['observed_bilateral_no_nonfinger_support'])
        self.assertFalse(report['release']['contact_loss_after_observed_hold'])
        self.assertEqual(report['status'], 'FAIL')

    def test_physical_release_can_be_observed_below_frozen_80mm_lift(self):
        fixture(self.path, lift_height_m=.067)
        report = diag.analyze(self.path)
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['frozen_status'], 'FAIL')
        self.assertEqual(report['original_grasp_result']['failed_criteria'], ['lift_height'])
        self.assertEqual(report['frozen_evaluation']['agreement'], 'PASS')
        self.assertEqual(report['release']['status'], 'PASS')
        self.assertTrue(report['release']['observed_bilateral_no_nonfinger_support'])
        self.assertAlmostEqual(report['release']['cube_bottom_clearance_above_table_m'], .067)
        self.assertTrue(report['release']['contact_loss_after_observed_hold'])
        self.assertTrue(report['release']['cube_fall_after_release_observed'])

    def test_original_evaluator_can_pass_without_release_while_stage_is_unknown(self):
        fixture(self.path, missing_release=True)
        report = diag.analyze(self.path)
        self.assertEqual(report['original_grasp_result']['status'], 'PASS')
        self.assertEqual(report['frozen_evaluation']['output']['status'], 'PASS')
        self.assertEqual(report['release']['status'], 'UNKNOWN')
        self.assertEqual(report['status'], 'PASS')
        self.assertEqual(report['frozen_status'], 'PASS')
        self.assertEqual(report['diagnostic_status'], 'UNKNOWN')

    def test_new_diagnostic_collision_does_not_change_frozen_acceptance(self):
        fixture(self.path)
        rows = [json.loads(line) for line in (self.path / 'contacts.jsonl').read_text().splitlines()]
        collision = contact('link7', .1, distance=-.00002)
        collision['bodies'] = ['link7', 'world']; collision['geoms'] = ['link7_geom', 'table']
        rows[1]['events'].append(collision)
        (self.path / 'contacts.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
        (self.path / 'contact-parameters.jsonl').write_text(''.join(json.dumps(dict(time=row['time'],
            contacts=[{}] * len(row['events']))) + '\n' for row in rows))
        report = diag.analyze(self.path)
        self.assertEqual(report['status'], 'PASS')
        self.assertEqual(report['frozen_status'], 'PASS')
        self.assertEqual(report['diagnostic_status'], 'FAIL')
        self.assertEqual(report['stages'][1]['status'], 'PASS')
        self.assertIsNone(report['earliest_frozen_violation'])
        self.assertEqual(report['earliest_diagnostic_event']['row'], 1)
        self.assertIn('approach_unexpected_nonobject_penetration', report['diagnostic_events'])

    def test_interrupted_close_and_hold_are_unknown_and_terminal_state_is_separate(self):
        for count, incomplete in ((4, 'close'), (10, 'hold')):
            with self.subTest(phase=incomplete):
                interrupted_fixture(self.path, count)
                report = diag.analyze(self.path)
                self.assertEqual(report['status'], 'INTERRUPTED')
                self.assertEqual(report['frozen_evaluation']['agreement'], 'PASS')
                stage = next(s for s in report['stages'] if s['phase'] == incomplete)
                self.assertEqual(stage['status'], 'UNKNOWN')
                self.assertNotIn('close_ended_without_trigger', report['events'])
                self.assertEqual(report['samples'], count)
                termination = report['termination_evidence']
                self.assertEqual(termination['unrecorded_physics_steps'], 1)
                self.assertEqual(termination['failure_state']['warning_numbers'][0], 1)
                self.assertEqual(report['earliest_frozen_violation']['event'], 'original_runner_interruption')
                for name in diag.FAILURE_NAMES:
                    self.assertIn(name, report['source_sha256_before'])

    def test_zero_row_interruption_keeps_failure_evidence_and_writes_report(self):
        interrupted_fixture(self.path, 0)
        report = diag.analyze(self.path)
        self.assertEqual(report['status'], 'INTERRUPTED')
        self.assertEqual(report['diagnostic_status'], 'FAIL')
        self.assertEqual(report['samples'], 0)
        self.assertEqual(report['frozen_evaluation']['agreement'], 'UNKNOWN')
        self.assertEqual(report['termination_evidence']['failure_metadata']['actual_steps'], 1)
        self.assertEqual(report['termination_evidence']['failure_state']['qpos'][11], .15)
        self.assertTrue(all(s['status'] == 'UNKNOWN' for s in report['stages']))
        self.assertEqual(report['source_sha256_before'], report['source_sha256_after'])
        diag.write_report(report, Path(self.tmp.name) / 'zero-analysis')

    def test_output_is_exclusive_and_outside_raw_inputs(self):
        fixture(self.path)
        report = diag.analyze(self.path)
        with self.assertRaises(ValueError): diag.write_report(report, self.path / 'analysis')
        output = Path(self.tmp.name) / 'analysis'
        diag.write_report(report, output)
        self.assertTrue((output / 'summary.json').is_file())
        self.assertTrue((output / 'events.csv').is_file())
        with self.assertRaises(FileExistsError): diag.write_report(report, output)
        self.assertEqual({name: diag.digest(self.path / name) for name in diag.SOURCE_NAMES}, report['source_sha256_before'])


if __name__ == '__main__':
    unittest.main()
