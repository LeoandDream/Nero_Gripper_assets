import unittest
import numpy as np
from pi05_lora_step import validate_checkpoint_map, compiled_peak, canonical_train_state, state_host, state_hash


class LoraPlan(unittest.TestCase):
    def test_only_new_lora_may_be_absent_from_checkpoint(self):
        from types import SimpleNamespace as S
        ref={('base','w'):S(shape=(2,3),dtype=np.dtype('float32')),
             ('llm','lora_a'):S(shape=(2,1),dtype=np.dtype('float32'))}
        meta={('base','w'):S(shape=(2,3),dtype=np.dtype('float32'))}
        result=validate_checkpoint_map(ref,meta)
        self.assertEqual(result['new_lora_paths'],['llm/lora_a'])
        with self.assertRaises(ValueError):validate_checkpoint_map({('head','w'):ref[('base','w')]},meta)

    def test_memory_analysis_subtracts_donated_aliases(self):
        self.assertEqual(compiled_peak(dict(argument_size_in_bytes=100,output_size_in_bytes=70,
            temp_size_in_bytes=30,alias_size_in_bytes=60)),140)

    def test_static_optimizer_identity_matches_precompiled_state(self):
        from flax import struct
        import jax
        @struct.dataclass
        class State:
            params: object
            model_def: object=struct.field(pytree_node=False)
            tx: object=struct.field(pytree_node=False)
        a=State(np.zeros(2,np.float32),'same_graph',lambda:1)
        b=State(jax.ShapeDtypeStruct((2,),np.float32),'same_graph',lambda:1)
        self.assertNotEqual(jax.tree.structure(a),jax.tree.structure(b))
        result=canonical_train_state(a,b)
        self.assertIs(result.tx,b.tx);self.assertIs(result.params,a.params)
        self.assertEqual(jax.tree.structure(result),jax.tree.structure(b))

    def test_installed_flax_flat_state_mapping_is_supported(self):
        from flax import nnx
        state=nnx.State({'a':nnx.VariableState(nnx.Param,np.zeros(3,np.float32))})
        np.testing.assert_array_equal(state_host(state)['a'],np.zeros(3,np.float32))
        self.assertEqual(len(state_hash(state)['a']),64)


if __name__=='__main__':unittest.main()
