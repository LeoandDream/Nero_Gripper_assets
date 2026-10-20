"""Targeted zero-integration tests for the policy action boundary and scheduler."""
import unittest
import numpy as np
from replay_mvp_actions import ReplayPolicy, position_action, dispatch, close_phase_check


class ActionContract(unittest.TestCase):
    def setUp(self):
        self.limits = np.array([[-2., 2.]] * 7 + [[0., .1]])

    def test_float32_endpoint_is_restored_and_logged(self):
        value = np.zeros(8, np.float32); value[-1] = .1
        actual, changes = position_action(value, self.limits)
        self.assertEqual(actual[-1], .1)
        self.assertEqual(len(changes), 1)
        self.assertEqual(changes[0]['original'], float(value[-1]))

    def test_invalid_positions_are_rejected(self):
        for value in (np.zeros(9), np.full(8, np.nan), np.full(8, np.inf),
                      np.zeros((1, 8)), np.array([0.] * 7 + [.100001]),
                      np.array([2.000001] + [0.] * 7), np.zeros(8, complex)):
            with self.assertRaises(ValueError): position_action(value, self.limits)

    def test_padding_is_not_consumed(self):
        p = ReplayPolicy(np.arange(682 * 8).reshape(682, 8))
        consumed = []
        for start in range(0, 682, 10):
            chunk, is_pad = p.chunk(start)
            self.assertEqual(chunk.shape, (10, 8))
            consumed.extend(chunk[~is_pad].tolist())
        np.testing.assert_array_equal(consumed, p.actions)
        self.assertEqual(p.chunk(680)[1].sum(), 8)
        self.assertEqual(p.chunk(681)[1].sum(), 9)
        with self.assertRaises(IndexError): p.chunk(682)

    def test_integer_schedule_prefix_and_terminal(self):
        prefix = np.repeat(np.arange(50)[:, None], 8, axis=1)
        actions = np.repeat(np.arange(682)[:, None], 8, axis=1)
        targets = []; advances = []
        for k in range(34150):
            target, advance = dispatch(k, prefix, actions)
            if target is not None: targets.append((k, target[0]))
            if advance: advances.append(k)
        self.assertEqual(targets[:5], [(k, k) for k in range(0, 50, 10)])
        self.assertEqual(targets[5], (50, 0))
        self.assertEqual(targets[-1], (34100, 681))
        self.assertEqual(len(targets), 687)
        self.assertEqual(len(advances), 3415)
        self.assertEqual(advances[-1], 34140)

    def test_source_contact_time_is_not_the_original_close_timeout(self):
        force = np.zeros((34150, 2)); force[20500] = .25
        actual = close_phase_check(force, 8372, 20483, 20., .2)
        self.assertEqual(actual['status'], 'PASS')
        self.assertEqual(actual['new_bilateral_contact_step'], 20501)
        self.assertFalse(actual['within_source_close_window'])
        force[:] = 0; force[28372] = .25
        self.assertEqual(close_phase_check(force, 8372, 20483, 20., .2)['status'], 'FAIL')

    def test_contact_only_after_lift_is_not_successful_close(self):
        force = np.zeros((34150, 2)); force[22000] = .25
        actual = close_phase_check(force, 8372, 20483, 20., .2, first_lift_step=21483)
        self.assertEqual(actual['status'], 'FAIL')
        self.assertIsNone(actual['new_bilateral_contact_time_s'])


if __name__ == '__main__': unittest.main()
