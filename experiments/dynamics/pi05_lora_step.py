"""One fixed official PI05 LoRA candidate: CPU plan, then <=1 GPU update.

No train.main, checkpoint save, weights download, WandB or simulation.
"""
import argparse
import dataclasses
import functools
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import threading
import time

import numpy as np
from pi05_one_step_preflight import CHECKPOINT, STATS, STATS_SHA, EPISODE, sha, write, gpu_snapshot, tree_summary

OLD_MANIFEST=Path('/data_robot/projects/.experiments/nero-one-step-issue32-20261023/preflight_001/checkpoint-sha256.json')
RESERVE=2*2**30  # Per-GPU budget reserve fixed before execution, beyond XLA-estimated buffers.


def validate_checkpoint_map(expected,metadata):
    missing=set(expected)-set(metadata); extra=set(metadata)-set(expected)
    if extra or any('lora' not in '/'.join(k) for k in missing):
        raise ValueError('Only newly initialized official LoRA parameters may be missing')
    bad=[k for k in metadata if tuple(metadata[k].shape)!=tuple(expected[k].shape)]
    if bad:raise ValueError('Pretrained base shape mismatch: '+repr(bad))
    return dict(base_paths=['/'.join(k) for k in sorted(metadata)],
                new_lora_paths=['/'.join(k) for k in sorted(missing)],
                base_shapes_and_dtypes={ '/'.join(k):dict(shape=list(metadata[k].shape),
                    checkpoint_dtype=str(metadata[k].dtype),target_dtype=str(expected[k].dtype)) for k in metadata})


def compiled_peak(info):
    return int(info['argument_size_in_bytes']+info['output_size_in_bytes']+
               info['temp_size_in_bytes']-info['alias_size_in_bytes'])


def canonical_train_state(state,prototype):
    """Use the same Optax static closure identity as the precompiled signature.

    Both states come from official init_train_state with the SAME config.
    Their optimizer numeric state is unchanged; freshly-created tx closures
    otherwise differ as Flax static pytree metadata, despite identical math.
    """
    import jax
    if state.model_def!=prototype.model_def:raise ValueError('Model graph differs from Gate0 prototype')
    state=dataclasses.replace(state,tx=prototype.tx)
    if jax.tree.structure(state)!=jax.tree.structure(prototype):raise ValueError('TrainState static metadata mismatch')
    return state


def context():
    if os.environ.get('HF_HUB_OFFLINE')!='1' or os.environ.get('TRANSFORMERS_OFFLINE')!='1':
        raise ValueError('Use existing offline caches only')
    if Path(os.environ.get('HF_LEROBOT_HOME','')).resolve()!=(EPISODE/'lerobot').resolve():
        raise ValueError('Use the exact frozen NERO dataset cache')
    import jax
    import openpi
    from openpi.models.pi0_config import Pi0Config
    from openpi.training import config,weight_loaders
    import subprocess
    upstream=Path(openpi.__file__).resolve().parents[2]
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=upstream,text=True).strip()=='215abfb217dbac7d5f1273282331b9b1866c0479'
    assert sha(STATS/'norm_stats.json')==STATS_SHA
    model=Pi0Config(pi05=True,action_horizon=10,action_dim=32,
        paligemma_variant='gemma_2b_lora',action_expert_variant='gemma_300m_lora')
    cfg=config.TrainConfig(name='nero_pi05_lora_one_step',exp_name='fixed_smoke',model=model,
        freeze_filter=model.get_freeze_filter(),ema_decay=None,
        weight_loader=weight_loaders.CheckpointWeightLoader(str(CHECKPOINT)),
        batch_size=2,fsdp_devices=2,num_workers=0,num_train_steps=1,wandb_enabled=False,seed=42)
    spec=importlib.util.spec_from_file_location('official_lora_train',upstream/'scripts/train.py')
    official=importlib.util.module_from_spec(spec);spec.loader.exec_module(official)
    return jax,upstream,cfg,official


def seal(output):
    write(output/'manifest.json',{str(p.relative_to(output)):dict(sha256=sha(p),bytes=p.stat().st_size)
        for p in output.rglob('*') if p.is_file() and p.name!='manifest.json'})


def plan(output):
    assert os.environ.get('JAX_PLATFORMS')=='cpu'
    jax,upstream,cfg,official=context()
    from openpi.training import sharding
    from flax import traverse_util
    import orbax.checkpoint as ocp
    output.mkdir(parents=True,exist_ok=False)
    files={str(p):sha(p) for p in [Path(__file__),Path(__file__).with_name('nero_training_data.py'),
        Path(__file__).with_name('nero_openpi.py'),Path(__file__).with_name('pi05_one_step_preflight.py'),
        STATS/'norm_stats.json',upstream/'scripts/train.py',upstream/'src/openpi/models/lora.py',
        upstream/'src/openpi/models/pi0.py',upstream/'src/openpi/models/pi0_config.py',
        upstream/'src/openpi/training/weight_loaders.py',upstream/'src/openpi/training/sharding.py',
        upstream/'src/openpi/training/optimizer.py',upstream/'src/openpi/training/data_loader.py',
        Path(os.environ['OPENPI_DATA_HOME'])/'big_vision/paligemma_tokenizer.model']}
    weights=json.loads(OLD_MANIFEST.read_text())
    for name,record in weights.items():assert sha(CHECKPOINT/name)==record['sha256']
    original=json.loads((EPISODE/'dataset-manifest.json').read_text())
    for name,info in original['data_files'].items():
        p=EPISODE/'lerobot/local/nero_mvp_r1'/name;assert sha(p)==info['sha256'];files[str(p)]=info['sha256']
    train_rng,init_rng=jax.random.split(jax.random.key(cfg.seed))
    shape,_=official.init_train_state(cfg,init_rng,sharding.make_mesh(1),resume=True)
    expected=traverse_util.flatten_dict(shape.params.to_pure_dict())
    with ocp.PyTreeCheckpointer() as ck:
        meta=traverse_util.flatten_dict(ck.metadata(CHECKPOINT)['params'])
    if all(k[-1]=='value' for k in meta):meta={k[:-1]:v for k,v in meta.items()}
    mapping=validate_checkpoint_map(expected,meta)
    trainable=shape.params.filter(cfg.trainable_filter); frozen=shape.params.filter(cfg.freeze_filter)
    tp={ '/'.join(str(s) for s in k):dict(shape=list(v.value.shape),dtype=str(v.value.dtype),
             elements=int(np.prod(v.value.shape))) for k,v in trainable.flat_state().items()}
    data=dict(model=dataclasses.asdict(cfg.model),freeze_filter='official model.get_freeze_filter()',
        optimizer=dataclasses.asdict(cfg.optimizer),lr_schedule=dataclasses.asdict(cfg.lr_schedule),
        ema_decay=cfg.ema_decay,seed=cfg.seed,batch_size=2,fsdp_devices=2,
        checkpoint=str(CHECKPOINT),checkpoint_manifest_sha256=sha(OLD_MANIFEST),
        input_sha256=files,per_gpu_reserve_bytes=RESERVE,
        full=tree_summary(shape.params,jax),frozen=tree_summary(frozen,jax),
        trainable=tree_summary(trainable,jax),optimizer_state=tree_summary(shape.opt_state,jax),
        trainable_paths=tp,loss='Original32D flow matching; physicalNERO8D + featurepadding24D; no new mask',
        gpu_snapshot=gpu_snapshot(),expected_optimizer_steps=1,source_baseline='c24a546 + bec08f9')
    write(output/'plan.json',data);write(output/'checkpoint-map.json',mapping)
    (output/'source').mkdir();shutil.copyfile(__file__,output/'source/pi05_lora_step.py')
    seal(output);print(json.dumps({k:data[k] for k in ['full','frozen','trainable','optimizer_state']}),flush=True)


def array_hash(value):
    a=np.asarray(value)
    if not np.isfinite(a).all():raise ValueError('Nonfinite parameter/optimizer state')
    return hashlib.sha256(a.tobytes()).hexdigest()


def state_host(state):
    import jax
    return { '/'.join(map(str,k)):np.asarray(jax.device_get(v.value)).copy() for k,v in state.flat_state().items()}


def state_hash(state):
    import jax
    return { '/'.join(map(str,k)):array_hash(jax.device_get(v.value)) for k,v in state.flat_state().items()}


def run(plan_directory,output):
    if os.environ.get('CUDA_VISIBLE_DEVICES')!='2,3' or os.environ.get('JAX_PLATFORMS')!='cuda':
        raise ValueError('Only fixed physical GPU2/3, CUDA JAX, no CPU fallback')
    p=json.loads((plan_directory/'plan.json').read_text())
    assert sha(__file__)==p['input_sha256'][str(Path(__file__))]
    assert sha(OLD_MANIFEST)==p['checkpoint_manifest_sha256']
    for name,h in p['input_sha256'].items():assert sha(name)==h
    initial=gpu_snapshot()
    for line in initial['compute_processes_csv'].splitlines():
        uuid,pid,_=[v.strip() for v in line.split(',')]
        if any(g['index'] in [2,3] and g['uuid']==uuid for g in initial['devices']) and int(pid)!=os.getpid():
            raise RuntimeError('Selected GPU has another compute process')
    output.mkdir(parents=True,exist_ok=False);write(output/'gpu-before.json',initial)
    jax,upstream,cfg,official=context()
    from openpi.training import sharding,weight_loaders
    from flax import traverse_util
    from nero_training_data import create_training_loader
    assert jax.device_count()==2 and all(d.platform=='gpu' for d in jax.devices())
    mesh=sharding.make_mesh(2);ds=jax.sharding.NamedSharding(mesh,jax.sharding.PartitionSpec(sharding.DATA_AXIS))
    replicated=jax.sharding.NamedSharding(mesh,jax.sharding.PartitionSpec())
    train_rng,init_rng=jax.random.split(jax.random.key(cfg.seed))
    shape,ss=official.init_train_state(cfg,init_rng,mesh,resume=True)
    per_device={str(d):0 for d in jax.devices()}
    for arr,s in zip(jax.tree.leaves(shape),jax.tree.leaves(ss)):
        n=int(np.prod(s.shard_shape(arr.shape)))*np.dtype(arr.dtype).itemsize
        for d in s.device_set:per_device[str(d)]+=n
    # Official loader input parameters are replicated during initialization.
    expected=traverse_util.flatten_dict(shape.params.to_pure_dict())
    base_paths=set(json.loads((plan_directory/'checkpoint-map.json').read_text())['base_paths'])
    restored_input_bytes=sum(int(np.prod(v.shape))*np.dtype(v.dtype).itemsize for k,v in expected.items() if '/'.join(k) in base_paths)
    free=[g['free_MiB']*2**20 for g in initial['devices'] if g['index'] in [2,3]]
    budgets=[dict(device=str(d),resident_state_bytes=per_device[str(d)],free_bytes=free[i],
        remaining_after_resident_bytes=free[i]-per_device[str(d)],
        conservative_init_bytes=restored_input_bytes+per_device[str(d)]+RESERVE) for i,d in enumerate(jax.devices())]
    write(output/'device-budget.json',dict(devices=budgets,replicated_checkpoint_input_bytes=restored_input_bytes))
    if any(b['conservative_init_bytes']>=b['free_bytes'] for b in budgets):raise RuntimeError('BLOCKED_INIT_MEMORY_BUDGET')
    loader=create_training_loader(STATS,cfg.model,batch_size=2,shuffle=False,num_batches=1,framework='jax',sharding=ds)
    view=loader._data_loader.torch_loader.dataset._dataset
    assert len(view)==673 and np.array_equal(view.starts,np.arange(673))
    obs,act=next(iter(loader));batch=jax.tree.map(lambda x:x.astype(np.float32) if np.issubdtype(x.dtype,np.floating) else x,(obs,act))
    assert batch[1].shape==(2,10,32) and batch[1].dtype==np.float32 and batch[0].state.dtype==np.float32
    assert batch[0].images['base_0_rgb'].shape==(2,224,224,3)
    assert not np.asarray(batch[1])[:,:,8:].any() and not np.asarray(batch[0].state)[:,9:].any()
    for x in jax.tree.leaves(batch):assert np.isfinite(np.asarray(x)).all()
    write(output/'batch.json',dict(valid_starts=673,selected_starts=[0,1],stats_sha256=STATS_SHA,
        state_shape=list(batch[0].state.shape),actions_shape=list(batch[1].shape),state_dtype=str(batch[0].state.dtype),
        actions_dtype=str(batch[1].dtype),image_shape=list(batch[0].images['base_0_rgb'].shape),
        masks={k:np.asarray(v).tolist() for k,v in batch[0].image_masks.items()},
        physical8_normalized_first_actions=np.asarray(batch[1])[:,0,:8].tolist(),feature_tail24_zero=True,
        time_padding=False,discrete_state_input=cfg.model.discrete_state_input))
    monitor_stop=threading.Event();samples=[];phase=['compile']
    def monitor():
        import subprocess
        while not monitor_stop.is_set():
            out=subprocess.check_output(['nvidia-smi','--id=2,3','--query-gpu=index,memory.used,memory.free','--format=csv,noheader,nounits'],text=True)
            samples.append(dict(monotonic_s=time.monotonic(),phase=phase[0],csv=out));monitor_stop.wait(.25)
    thread=threading.Thread(target=monitor,daemon=True);thread.start()
    attempts=0;result=None
    try:
        print('COMPILE_TRAIN_STEP',flush=True)
        with sharding.set_mesh(mesh):
            jitted=jax.jit(functools.partial(official.train_step,cfg),in_shardings=(replicated,ss,ds),
                out_shardings=(ss,replicated),donate_argnums=(1,))
            executable=jitted.lower(train_rng,shape,batch).compile()
        ma=executable.memory_analysis()
        info={k:int(getattr(ma,k)) for k in ['argument_size_in_bytes','output_size_in_bytes','temp_size_in_bytes','alias_size_in_bytes']}
        peak=compiled_peak(info);write(output/'compiled-memory.json',dict(info,estimated_per_device_peak_bytes=peak,reserve_bytes=RESERVE))
        if any(peak+RESERVE>=b['free_bytes'] for b in budgets):raise RuntimeError('BLOCKED_COMPILED_MEMORY_BUDGET')
        # Delegate to exact official weight-loader merging; audit only, no algorithm/data changes.
        class AuditedLoader:
            def load(self,reference):
                loaded=weight_loaders.CheckpointWeightLoader(str(CHECKPOINT)).load(reference)
                flat=traverse_util.flatten_dict(loaded);base={};new=[]
                for k,v in flat.items():
                    key='/'.join(k)
                    if isinstance(v,jax.ShapeDtypeStruct):
                        assert 'lora' in key;new.append(key)
                    else:base[key]=array_hash(v)
                assert set(base)==base_paths
                self.base_hash=base;write(output/'restore-audit.json',dict(base_hash=base,new_random_lora_only=new))
                return loaded
        audit=AuditedLoader();initialize_cfg=dataclasses.replace(cfg,weight_loader=audit)
        # Bind the real restore to the pre-registered source, not just loader->GPU consistency.
        for name,record in json.loads(OLD_MANIFEST.read_text()).items():
            assert sha(CHECKPOINT/name)==record['sha256'],name
        write(output/'checkpoint-source-verified.json',dict(manifest_sha256=sha(OLD_MANIFEST),all_source_files_match=True))
        phase[0]='restore_and_init';print('RESTORE_REAL_LOCAL_WEIGHTS',flush=True)
        state,actual_ss=official.init_train_state(initialize_cfg,init_rng,mesh,resume=False);jax.block_until_ready(state)
        state=canonical_train_state(state,shape)
        assert int(state.step)==0 and state.ema_params is None
        before=state_host(state.params.filter(cfg.trainable_filter));frozen_before=state_hash(state.params.filter(cfg.freeze_filter))
        all_initial={**{k:array_hash(v) for k,v in before.items()},**frozen_before}
        assert all(all_initial[k]==h for k,h in audit.base_hash.items())
        opt_before=[array_hash(jax.device_get(x)) for x in jax.tree.leaves(state.opt_state)]
        write(output/'before-update.json',dict(step=0,trainable_hash={k:array_hash(v) for k,v in before.items()},
            frozen_hash=frozen_before,optimizer_hash=opt_before,base_restore_byte_match=True))
        phase[0]='one_update';print('CALL_OFFICIAL_TRAIN_STEP_ONCE',flush=True)
        write(output/'optimizer-step-attempt.json',dict(allowed_calls=1,attempt=1));attempts=1
        state,metrics=executable(train_rng,state,batch);jax.block_until_ready((state,metrics))
        phase[0]='verification';assert int(state.step)==1
        metrics={k:float(v) for k,v in metrics.items()};assert all(np.isfinite(list(metrics.values()))) and metrics['grad_norm']>0
        after=state_host(state.params.filter(cfg.trainable_filter));frozen_after=state_hash(state.params.filter(cfg.freeze_filter))
        assert frozen_before==frozen_after
        changes={k:dict(changed_elements=int(np.count_nonzero(after[k]!=v)),max_abs_change=float(np.max(abs(after[k]-v))),
            before_sha256=array_hash(v),after_sha256=array_hash(after[k])) for k,v in before.items()}
        changed_lora=[k for k,v in changes.items() if 'lora' in k and v['changed_elements']>0]
        assert changed_lora
        opt_after=[array_hash(jax.device_get(x)) for x in jax.tree.leaves(state.opt_state)];assert opt_after!=opt_before
        write(output/'parameter-changes.json',dict(trainable=changes,frozen_hash_before=frozen_before,frozen_hash_after=frozen_after,
            optimizer_hash_before=opt_before,optimizer_hash_after=opt_after))
        result=dict(status='PI05_NERO_LORA_ONE_STEP_PASS',metrics=metrics,step_before=0,step_after=1,
            optimizer_calls=1,optimizer_state_changed=True,changed_lora_paths=changed_lora,
            trainable_elements=p['trainable']['elements'],frozen_elements=p['frozen']['elements'],
            frozen_parameters_unchanged=True,all_parameters_finite=True,base_checkpoint_restore_exact=True,
            ema_decay=None,new_weights_saved=False,wandb_started=False,full_parameter_blocker_unchanged=True)
    except BaseException:
        import traceback
        result=dict(status='BLOCKED',optimizer_calls=attempts,error=traceback.format_exc(),new_weights_saved=False)
    finally:
        monitor_stop.set();thread.join(timeout=10)
        write(output/'gpu-samples.json',samples)
        try:write(output/'jax-memory-stats.json',{str(d):d.memory_stats() for d in jax.devices()})
        except Exception:pass
        for name,h in p['input_sha256'].items():assert sha(name)==h
        write(output/'result.json',result);seal(output);print(json.dumps(result),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=['plan','run'])
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--plan',type=Path)
    args=parser.parse_args()
    if args.mode=='plan':plan(args.output.resolve())
    else:
        if args.plan is None:parser.error('run requires --plan')
        run(args.plan.resolve(),args.output.resolve())
