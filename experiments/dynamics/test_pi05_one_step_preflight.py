"""Pure boundary tests for one-step input conversion and memory accounting."""
import unittest
import numpy as np
from pi05_one_step_preflight import model_dtype_array, resource_budget


class Preflight(unittest.TestCase):
    def test_dtype_changes_only_real_floating_values(self):
        for original in (np.array([0.,1.000394],np.float64),np.array([1,200],np.int64),np.array([True,False])):
            actual=model_dtype_array(original)
            expected=np.float32 if original.dtype.kind=='f' else original.dtype
            self.assertEqual(actual.dtype,expected)
            np.testing.assert_array_equal(actual,original.astype(expected))
        with self.assertRaises(ValueError): model_dtype_array(np.array([np.nan]))

    def test_persistent_state_can_block_before_any_allocation(self):
        budget=resource_budget(13_413_735_488,26_827_470_984,13_413_735_488,49_087*2**20)
        self.assertFalse(budget['persistent_state_fits'])
        self.assertEqual(budget['persistent_state_bytes'],53_654_941_960)


if __name__=='__main__':unittest.main()
