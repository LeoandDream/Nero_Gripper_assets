"""Issue32 read-only Gate0. Stops on insufficient resources BEFORE real init/update.

Uses local checkpoint metadata, a real normalized NERO batch and the official
init_train_state(resume=True) ABSTRACT state. No random model is materialized,
no real optimizer state is allocated and train_step/main are never called here.
"""
import argparse
import dataclasses
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

BASE=Path('/data_robot/projects')
CHECKPOINT=BASE/'official_pi05_libero/cache/openpi/openpi-assets/checkpoints/pi05_libero/params'
STATS=Path(__file__).resolve().parent/'nero_norm_stats'
EPISODE=BASE/'.experiments/nero-mvp-lerobot-issue27-20261020'
STATS_SHA='59b12d0aa10aa116018cd8d8c0d1d0615fdb85e7a45ac3bc61d647c33a31ff26'


def model_dtype_array(value):
    value=np.asarray(value)
    if value.dtype.kind not in 'fbiu' or not np.isfinite(value).all():
        raise ValueError('Expected finite model input')
    return value.astype(np.float32) if value.dtype.kind=='f' else value


def resource_budget(params_bytes, optimizer_bytes, ema_bytes, free_bytes):
    persistent=params_bytes+optimizer_bytes+ema_bytes
    return dict(params_bytes=params_bytes,optimizer_bytes=optimizer_bytes,ema_bytes=ema_bytes,
                persistent_state_bytes=persistent,available_free_bytes=free_bytes,
                persistent_state_fits=persistent<=free_bytes,
                additional_gradients_bytes=params_bytes,
                excluded='activations, attention, collectives, compiler/temporary buffers, FSDP replicated small arrays')


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()


def write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def gpu_snapshot():
    command=['nvidia-smi','--query-gpu=index,uuid,name,memory.total,memory.free','--format=csv,noheader,nounits']
    raw=subprocess.check_output(command,text=True)
    result=[]
    for line in raw.splitlines():
        index,uuid,name,total,free=[p.strip() for p in line.split(',')]
        result.append(dict(index=int(index),uuid=uuid,name=name,total_MiB=int(total),free_MiB=int(free)))
    processes=subprocess.check_output(['nvidia-smi','--query-compute-apps=gpu_uuid,pid,used_memory','--format=csv,noheader,nounits'],text=True)
    return dict(devices=result,compute_processes_csv=processes)


def tree_summary(tree,jax):
    leaves=jax.tree.leaves(tree)
    return dict(leaves=len(leaves),elements=sum(int(np.prod(x.shape)) for x in leaves),
        bytes=sum(int(np.prod(x.shape))*np.dtype(x.dtype).itemsize for x in leaves),
        dtypes=sorted(set(str(x.dtype) for x in leaves)))


def preflight(output):
    if os.environ.get('JAX_PLATFORMS')!='cpu':raise ValueError('Gate0 is CPU-only; do not allocate GPU state')
    if os.environ.get('HF_HUB_OFFLINE')!='1':raise ValueError('Offline cache required')
    if Path(os.environ.get('HF_LEROBOT_HOME','')).resolve()!=(EPISODE/'lerobot').resolve():
        raise ValueError('Use frozen NERO dataset cache')
    output.mkdir(parents=True,exist_ok=False)
    gpu=gpu_snapshot();write(output/'gpu-before.json',gpu)
    # Only devices without compute processes are eligible; never release another task's VRAM.
    busy={line.split(',')[0].strip() for line in gpu['compute_processes_csv'].splitlines()}
    eligible=[g for g in gpu['devices'] if g['uuid'] not in busy]
    selected=sorted(eligible,key=lambda g:g['free_MiB'],reverse=True)[:2]
    import jax
    import orbax.checkpoint as ocp
    from flax import traverse_util
    from openpi.models.pi0_config import Pi0Config
    from openpi.training import config,sharding,weight_loaders
    import openpi
    from nero_training_data import create_training_loader
    import nero_training_data,nero_openpi
    upstream=Path(openpi.__file__).resolve().parents[2]
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=upstream,text=True).strip()=='215abfb217dbac7d5f1273282331b9b1866c0479'
    assert CHECKPOINT.is_dir() and sha(STATS/'norm_stats.json')==STATS_SHA
    train_path=upstream/'scripts/train.py'
    spec=importlib.util.spec_from_file_location('locked_train',train_path)
    official=importlib.util.module_from_spec(spec);spec.loader.exec_module(official)
    # Defaults: no freeze/LoRA, AdamW, original LR schedule, EMA0.99.
    cfg=config.TrainConfig(name='nero_issue32_one_step',exp_name='bounded_smoke',
        model=Pi0Config(pi05=True,action_horizon=10,action_dim=32),
        weight_loader=weight_loaders.CheckpointWeightLoader(str(CHECKPOINT)),
        batch_size=2,num_workers=0,num_train_steps=1,wandb_enabled=False,fsdp_devices=2)
    files={str(p):sha(p) for p in (Path(__file__),Path(nero_training_data.__file__),Path(nero_openpi.__file__),
            STATS/'norm_stats.json',train_path,upstream/'src/openpi/models/pi0.py',
            upstream/'src/openpi/training/optimizer.py',upstream/'src/openpi/training/weight_loaders.py')}
    checkpoint_hashes={str(p.relative_to(CHECKPOINT)):dict(sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(CHECKPOINT.rglob('*')) if p.is_file()}
    write(output/'checkpoint-sha256.json',checkpoint_hashes)
    manifest=json.loads((EPISODE/'dataset-manifest.json').read_text())
    for name,item in manifest['data_files'].items():
        p=EPISODE/'lerobot/local/nero_mvp_r1'/name
        assert sha(p)==item['sha256'];files[str(p)]=item['sha256']
    write(output/'frozen-config.json',dict(input_sha256=files,checkpoint=str(CHECKPOINT),
        checkpoint_manifest_sha256=sha(output/'checkpoint-sha256.json'),checkpoint_origin='existing local pi05_libero from official GCS; no new download',
        model=dataclasses.asdict(cfg.model),freeze_filter='nnx.Nothing (all nnx.Param trainable)',
        optimizer=dataclasses.asdict(cfg.optimizer),lr_schedule=dataclasses.asdict(cfg.lr_schedule),
        ema_decay=cfg.ema_decay,seed=cfg.seed,batch_size=2,proposed_fsdp_devices=2,
        eligible_gpu_indices=[g['index'] for g in selected],preflight_devices=[str(d) for d in jax.devices()],
        loss='Original Pi0.compute_loss mean over32 model action dimensions; first8 are physical, last24 padding; no new loss mask',
        stats_sha256=STATS_SHA,original_discrete_state_input=True,argv=sys.argv,
        prohibited_calls=['train.main','wandb.init','checkpoint.save','MuJoCo','actual train_step until Gate0 passes']))
    print('ABSTRACT_STATE_START',flush=True)
    # resume=True returns shapes/shardings ONLY; no checkpoint restore, random parameter materialization or optimizer update.
    abstract,_=official.init_train_state(cfg,jax.random.key(cfg.seed),sharding.make_mesh(1),resume=True)
    summaries={name:tree_summary(value,jax) for name,value in [('params',abstract.params),
        ('trainable',abstract.params.filter(cfg.trainable_filter)),('opt_state',abstract.opt_state),('ema_params',abstract.ema_params)]}
    expected=traverse_util.flatten_dict(abstract.params.to_pure_dict())
    with ocp.PyTreeCheckpointer() as checkpointer:
        meta=checkpointer.metadata(CHECKPOINT)['params']
    metadata=traverse_util.flatten_dict(meta)
    if all(k[-1]=='value' for k in metadata):metadata={k[:-1]:v for k,v in metadata.items()}
    mismatch=[]
    for key in sorted(set(expected)|set(metadata)):
        if key not in metadata or key not in expected or tuple(metadata[key].shape)!=tuple(expected[key].shape):
            mismatch.append('/'.join(key))
    assert not mismatch,f'Checkpoint shape mismatch: {mismatch}'
    compatibility=dict(status='PASS',scope='Orbax metadata keys/global shapes; not a new checkpoint restore/forward',
        expected_leaves=len(expected),checkpoint_leaves=len(metadata),shape_mismatches=mismatch,
        expected_dtypes=sorted(set(str(v.dtype) for v in expected.values())),
        checkpoint_dtypes=sorted(set(str(v.dtype) for v in metadata.values())),actual_weight_restore=False)
    write(output/'checkpoint-compatibility.json',compatibility)
    print('REAL_BATCH_START',flush=True)
    loader=create_training_loader(STATS,cfg.model,batch_size=2,shuffle=False,num_batches=1,framework='jax')
    transformed=loader._data_loader.torch_loader.dataset
    assert len(transformed)==673 and np.array_equal(transformed._dataset.starts,np.arange(673))
    before=transformed[0]
    assert np.asarray(before['state']).dtype==np.float64 and np.asarray(before['actions']).dtype==np.float64
    assert not before['actions_is_pad'].any()
    observation,actions=next(iter(loader))
    original=(observation,actions)
    converted=jax.tree.map(lambda x:jax.device_put(model_dtype_array(x)),original)
    obs,action=converted
    assert obs.state.dtype==np.float32 and action.dtype==np.float32
    assert action.shape==(2,10,32) and obs.state.shape==(2,32)
    assert obs.images['base_0_rgb'].shape==(2,224,224,3)
    assert not np.asarray(action)[:,:,8:].any() and not np.asarray(obs.state)[:,9:].any()
    leaves=[]
    for (path,old),(_,new) in zip(jax.tree_util.tree_flatten_with_path(original)[0],jax.tree_util.tree_flatten_with_path(converted)[0]):
        old=np.asarray(old);new=np.asarray(new)
        if old.dtype.kind in 'biu':np.testing.assert_array_equal(new,old)
        assert np.isfinite(new).all()
        leaves.append(dict(path=jax.tree_util.keystr(path),shape=list(old.shape),
            before_dtype=str(old.dtype),after_dtype=str(new.dtype),min=float(new.min()),max=float(new.max())))
    batch=dict(status='PASS',legal_starts=673,selected_starts=[0,1],time_padding=False,
        transformed_before_dtype=dict(state=str(before['state'].dtype),actions=str(before['actions'].dtype)),
        loader_framework='jax',layout='NHWC; original JAX Observation branch, no PyTorch NCHW mixing',leaves=leaves,
        action_first8=np.asarray(action)[:,0,:8].tolist(),feature_padding_tail24_all_zero=True,
        state_physical_dimensions=9,action_physical_dimensions=8,model_dimensions=32,
        model_discrete_state_input=True,stats_sha256=STATS_SHA,skip_norm_stats=False)
    write(output/'batch-dtype.json',batch)
    free_bytes=sum(g['free_MiB']*2**20 for g in selected)
    budget=resource_budget(summaries['params']['bytes'],summaries['opt_state']['bytes'],summaries['ema_params']['bytes'],free_bytes)
    write(output/'memory-budget.json',dict(budget,abstract_state=summaries,
        selected_gpus=selected,interpretation='Aggregate optimistic lower bound, even before uneven FSDP replication/activations'))
    for p,h in files.items():assert sha(p)==h
    # Do NOT silently continue into model initialization/update after a resource blocker.
    result=dict(status='TRAIN_STEP_BLOCKED' if not budget['persistent_state_fits'] else 'GATE0_ONLY_READY',
        reason='BLOCKED_GPU_MEMORY' if not budget['persistent_state_fits'] else 'Preflight completed; this script never executes optimizer steps',
        CHECKPOINT_MODEL_COMPATIBILITY='PASS',NERO_MODEL_BATCH_DTYPE='PASS',
        REAL_PI05_FORWARD_BACKWARD='FAIL',ONE_OPTIMIZER_STEP='FAIL',
        failed_checks_execution='NOT_RUN_RESOURCE_BLOCKED, not a numerical loss/gradient failure',
        checkpoint_compatibility_scope='metadata-only, actual model restore not performed',
        effective_optimizer_steps=0,real_forward_calls=0,loss=None,grad_norm=None,param_norm=None,
        training_step_before=None,training_step_after=None,trainable_parameter_update=None,
        gpu_training_initialization=False,oom_attempts=0,checkpoint_saved=False,wandb_started=False,
        abstract_state=summaries,memory_budget=budget,input_sha256_unchanged=True,
        checkpoint_manifest_sha256=sha(output/'checkpoint-sha256.json'))
    write(output/'result.json',result)
    (output/'executed-source').mkdir()
    for p in (Path(__file__),Path(__file__).with_name('test_pi05_one_step_preflight.py')):
        shutil.copyfile(p,output/'executed-source'/p.name)
    write(output/'manifest.json',{str(p.relative_to(output)):dict(sha256=sha(p),bytes=p.stat().st_size)
        for p in output.rglob('*') if p.is_file()})
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();preflight(args.output.resolve())
