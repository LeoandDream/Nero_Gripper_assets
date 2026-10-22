"""Small boundary tests; no model, optimizer, rendering or physics."""
import unittest
import numpy as np

from nero_training_data import CompleteActionChunks, valid_chunk_starts


class Chunks(unittest.TestCase):
    def test_single_episode_keeps_tail_actions(self):
        starts = valid_chunk_starts([(0, 682)], 682, 10)
        np.testing.assert_array_equal(starts, np.arange(673))
        self.assertEqual((starts[-1], starts[-1]+9), (672, 681))

    def test_multiple_episodes_never_cross_and_short_episodes_have_no_start(self):
        actual = valid_chunk_starts([(0, 9), (9, 19), (19, 31)], 31, 10)
        np.testing.assert_array_equal(actual, [9, 19, 20, 21])
        for bounds, total, horizon in [([(0, 5), (4, 12)], 12, 10),
                                       ([(1, 10)], 10, 10), ([(0, 10)], 10, 0)]:
            with self.assertRaises(ValueError): valid_chunk_starts(bounds, total, horizon)

    def test_view_cannot_access_rejected_start_or_accept_temporal_padding(self):
        class Dataset:
            def __len__(self): return 682
            def __getitem__(self, i):
                return {'actions': np.repeat(np.arange(i, i+10)[:,None], 8, axis=1),
                        'actions_is_pad': np.zeros(10, bool)}
        raw = Dataset(); view = CompleteActionChunks(raw, [(0, 682)], 10)
        self.assertEqual(len(raw), 682); self.assertEqual(len(view), 673)
        np.testing.assert_array_equal(view[672]['actions'][:,0], np.arange(672, 682))
        for i in [-1, 673, 681]:
            with self.assertRaises(IndexError): view[i]
        class Bad(Dataset):
            def __getitem__(self, i):
                item = super().__getitem__(i); item['actions_is_pad'][-1] = True; return item
        with self.assertRaises(ValueError): CompleteActionChunks(Bad(), [(0, 682)], 10)[672]


if __name__ == '__main__': unittest.main()
