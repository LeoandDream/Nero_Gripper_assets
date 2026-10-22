"""Issue27 continuation: valid training starts + native NERO stats + real batches.

No optimizer/model creation, weights, MuJoCo, rendering, export or source writes.
Use the existing locked OpenPI environment and offline tokenizer cache.
"""
import argparse
import hashlib
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pyarrow.parquet as pq

from nero_openpi import NeroOutputs, make_data_config
from nero_training_data import (REPO_ID, compute_norm_stats, create_training_loader,
                                create_training_view, load_training_config)

BASE = Path('/data_robot/projects')
EPISODE = BASE / '.experiments/nero-mvp-lerobot-issue27-20261020'
ROOT = Path(__file__).resolve().parents[2]
ROUNDTRIP_TOLERANCE = np.array([1e-6]*7 + [1e-8])  # rad x7, width metres; frozen before execution.


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    with Path(path).open('x') as f: json.dump(value, f, indent=2, allow_nan=False); f.write('\n')


def prepare(episode, output):
    expected_cache = episode / 'lerobot'
    if Path(os.environ.get('HF_LEROBOT_HOME', '')).resolve() != expected_cache.resolve():
        raise ValueError('Set HF_LEROBOT_HOME before importing the real loaders')
    if os.environ.get('HF_HUB_OFFLINE') != '1': raise ValueError('Use offline mode; no downloads')
    from openpi.models.pi0_config import Pi0Config
    from openpi.training import data_loader
    from openpi.shared import normalize
    from openpi import transforms
    import openpi
    import torch
    import nero_training_data
    import nero_openpi
    from lerobot.common.datasets import lerobot_dataset
    protocol = json.loads((ROOT / 'reports/issue27-protocol.json').read_text())
    upstream = Path(openpi.__file__).resolve().parents[2]
    commit = subprocess.check_output(['git','rev-parse','HEAD'], cwd=upstream, text=True).strip()
    assert commit == protocol['openpi_commit']
    direct = json.loads(metadata.distribution('lerobot').read_text('direct_url.json'))
    assert direct['vcs_info']['commit_id'] == protocol['lerobot_commit']
    tokenizer = Path(os.environ['OPENPI_DATA_HOME']) / 'big_vision/paligemma_tokenizer.model'
    assert tokenizer.is_file(), 'Existing offline tokenizer is required'
    source_manifest = episode / 'dataset-manifest.json'
    manifest = json.loads(source_manifest.read_text()); dataset = episode / 'lerobot' / REPO_ID
    assert dataset.resolve() == Path(manifest['root']).resolve()
    hashes = {str(source_manifest):sha(source_manifest), str(tokenizer):sha(tokenizer)}
    for name, info in manifest['data_files'].items():
        path = dataset / name; assert sha(path) == info['sha256'], str(path)
        hashes[str(path)] = info['sha256']
    # Seal actual upstream modules as well; no edits to the shared installation.
    for path in (Path(data_loader.__file__), Path(normalize.__file__), Path(transforms.__file__),
                 Path(lerobot_dataset.__file__), upstream/'scripts/compute_norm_stats.py'):
        hashes[str(path)] = sha(path)
    table = pq.read_table(dataset / 'data/chunk-000/episode_000000.parquet')
    physical_state = np.array(table['observation.state'].to_pylist(), dtype=np.float32)
    physical_actions = np.array(table['actions'].to_pylist(), dtype=np.float32)
    assert physical_state.shape == (682,9) and physical_actions.shape == (682,8)
    model = Pi0Config(pi05=True, action_horizon=10, action_dim=32)
    config = make_data_config(REPO_ID, model)
    view = create_training_view(config, model)
    np.testing.assert_array_equal(view.starts, np.arange(673))
    assert len(view.dataset) == 682 and len(view) == 673
    if output.is_relative_to(episode): raise ValueError('Output must not be inside source episode')
    output.mkdir(parents=True, exist_ok=False); (output / 'executed-source').mkdir()
    sources = [Path(__file__), Path(nero_training_data.__file__), Path(nero_openpi.__file__),
               ROOT/'experiments/dynamics/test_nero_training_data.py']
    for p in sources: shutil.copyfile(p, output/'executed-source'/p.name)
    write(output/'frozen-plan.json', dict(issue=27, task='norm stats + complete chunk training view',
        source_baseline='7f4af8ee730c0ee7913427c12bc2f37dbe2093b7',
        openpi_commit=commit, lerobot_commit=protocol['lerobot_commit'],
        python=sys.executable, argv=sys.argv, input_sha256=hashes,
        source_code_sha256={str(p):sha(p) for p in sources},
        starts=list(map(int,view.starts)), horizon=10, original_frames=682,
        statistics_position='repack + NeroInputs; BEFORE Normalize/tokenization/PadStatesAndActions',
        statistics_batch_size=1, counts=dict(states=673, actions=6730),
        action_weighting='overlapping10-step windows, not independent demonstrations',
        roundtrip_tolerance_rad_and_m=ROUNDTRIP_TOLERANCE.tolist(),
        normalization='official quantile, epsilon1e-6; no clipping/noise',
        scope=dict(training=False, weights=False, new_physics_steps=0, data_export=False)))
    write(output/'valid-starts.json', dict(legal=view.starts.tolist(), rejected=list(range(673,682)),
          last_action_indices=list(range(672,682)), original_frames=682))
    selected = []
    for i in range(len(view)):
        raw = view[i]  # Full real LeRobot read, including current video and actions_is_pad.
        assert not np.asarray(raw['actions_is_pad']).any()
        np.testing.assert_array_equal(raw['actions'], physical_actions[i:i+10])
        np.testing.assert_array_equal(raw['observation.state'], physical_state[i])
        assert int(raw['frame_index']) == i and int(raw['episode_index']) == 0
        if i in (0,336,672):
            selected.append(dict(start=i, action_indices=list(range(i,i+10)),
                                 actions_is_pad=np.asarray(raw['actions_is_pad']).tolist()))
    for i in range(673,682):
        try: view[i]
        except IndexError: pass
        else: raise AssertionError('Illegal start accessible from training view')
    # Demonstrate that original tail frames still exist; this raw sample is NOT a training start.
    assert np.asarray(view.dataset[681]['actions_is_pad']).sum() == 9
    stats, values = compute_norm_stats(view, config, upstream)
    stats_dir = output/'stats'
    normalize.save(stats_dir, stats)
    loaded = normalize.load(stats_dir)
    summaries = {}; counts = {}
    for key, dimension in [('state',9),('actions',8)]:
        measured = np.asarray(values[key]).reshape(-1,dimension)
        expected = physical_state[:673] if key == 'state' else np.array([physical_actions[i:i+10] for i in range(673)]).reshape(-1,8)
        np.testing.assert_array_equal(measured, expected)
        counts[key] = len(measured)
        for field in ('mean','std','q01','q99'):
            before = np.asarray(getattr(stats[key],field)); after = np.asarray(getattr(loaded[key],field))
            np.testing.assert_array_equal(before, after)
            assert after.shape == (dimension,) and np.isfinite(after).all()
        minimum = measured.min(axis=0); maximum = measured.max(axis=0)
        summaries[key] = dict(count=len(measured), dimension=dimension, min=minimum.tolist(), max=maximum.tolist(),
            range=(maximum-minimum).tolist(), units=['rad']*7+(['m']*2 if key=='state' else ['m']),
            constant_dimensions=np.flatnonzero(maximum==minimum).tolist(),
            range_at_most_epsilon_dimensions=np.flatnonzero(maximum-minimum<=1e-6).tolist(),
            quantile_span=(loaded[key].q99-loaded[key].q01).tolist(),
            mean_error_vs_float64_reference=float(abs(loaded[key].mean-measured.astype(float).mean(axis=0)).max()),
            std_error_vs_float64_reference=float(abs(loaded[key].std-measured.astype(float).std(axis=0)).max()))
    assert counts == dict(state=673, actions=6730)
    # This is the SAME public factory that future training must call; its view is not test-only.
    loader = create_training_loader(stats_dir, model, batch_size=1, shuffle=False, num_batches=673)
    torch_dataset = loader._data_loader.torch_loader.dataset
    training_view = torch_dataset._dataset
    np.testing.assert_array_equal(training_view.starts, view.starts)
    assert len(training_view.dataset) == 682 and len(torch_dataset) == 673
    assert loader.data_config().norm_stats is not None and loader.data_config().use_quantile_norm
    assert loaded.keys() == loader.data_config().norm_stats.keys()
    native_normalize = transforms.Normalize(loaded, use_quantiles=True)
    native_unnormalize = transforms.Unnormalize({'actions':loaded['actions']}, use_quantiles=True)
    roundtrip = np.zeros(8); batch_seen = 0; changed_from_physical = False
    normalized_min = np.full(8,np.inf); normalized_max = np.full(8,-np.inf)
    batch_examples = []; expected_masks = {'base_0_rgb':True,'left_wrist_0_rgb':False,'right_wrist_0_rgb':False}
    for i, (obs, actions) in enumerate(loader):
        assert tuple(obs.state.shape) == (1,32) and tuple(actions.shape) == (1,10,32)
        assert tuple(obs.images['base_0_rgb'].shape) == (1,3,224,224)
        assert tuple(obs.tokenized_prompt.shape) == (1,200) and bool(obs.tokenized_prompt_mask.any())
        for key, value in expected_masks.items(): assert bool(obs.image_masks[key][0]) == value
        assert torch.isfinite(obs.state).all() and torch.isfinite(actions).all()
        assert all(torch.isfinite(image).all() for image in obs.images.values())
        expected = native_normalize(dict(state=physical_state[i],actions=physical_actions[i:i+10]))
        current = actions[0].numpy(); current_state = obs.state[0].numpy()
        np.testing.assert_allclose(current[:,:8], expected['actions'], rtol=0, atol=1e-7)
        np.testing.assert_allclose(current_state[:9], expected['state'], rtol=0, atol=1e-7)
        assert not current[:,8:].any() and not current_state[9:].any()
        restored = NeroOutputs()(native_unnormalize({'actions':current}))['actions']
        assert restored.shape == (10,8)
        error = np.max(abs(restored-physical_actions[i:i+10]),axis=0); roundtrip=np.maximum(roundtrip,error)
        assert np.all(error <= ROUNDTRIP_TOLERANCE)
        changed_from_physical |= not np.allclose(current[:,:8],physical_actions[i:i+10])
        normalized_min=np.minimum(normalized_min,current[:,:8].min(axis=0));normalized_max=np.maximum(normalized_max,current[:,:8].max(axis=0))
        if i in (0,336,672):
            transformed = torch_dataset[i]
            assert not transformed['actions_is_pad'].any()
            batch_examples.append(dict(start=i, action_indices=list(range(i,i+10)),
                state_shape=list(obs.state.shape), action_shape=list(actions.shape),
                image_shape=list(obs.images['base_0_rgb'].shape), image_dtype=str(obs.images['base_0_rgb'].dtype),
                state_dtype=str(obs.state.dtype), action_dtype=str(actions.dtype),
                actions_is_pad=transformed['actions_is_pad'].tolist(),
                normalized_first_action=current[0,:8].tolist(),
                restored_first_action=restored[0].tolist(), physical_first_action=physical_actions[i].tolist()))
        batch_seen += 1
        if i % 100 == 0: print(f'NORMALIZED_BATCH start={i}',flush=True)
    assert batch_seen == 673 and changed_from_physical
    # The future shuffle also operates only on the filtered indices.
    shuffled = create_training_loader(stats_dir,model,batch_size=1,shuffle=True,num_batches=1)
    shuffled_view = shuffled._data_loader.torch_loader.dataset._dataset
    sampler_indices = list(shuffled._data_loader.torch_loader.sampler)
    assert sorted(shuffled_view.starts[sampler_indices].tolist()) == list(range(673))
    # Fail closed on absent stats: no normalization bypass in the reusable loader.
    try: load_training_config(output/'missing-stats',model)
    except FileNotFoundError: pass
    else: raise AssertionError('Missing statistics silently accepted')
    for path, digest in hashes.items(): assert sha(path) == digest, path
    weighting = np.bincount(np.concatenate([np.arange(i,i+10) for i in range(673)]),minlength=682)
    write(output/'statistics-audit.json',dict(status='PASS',statistics=summaries,counts=counts,
        action_frame_multiplicity=weighting.tolist(), statistics_batch_size=1, last_included_start=672,
        tools='locked compute_norm_stats.RemoveStrings + official TorchDataLoader/RunningStats/save/load',
        quantiles='official online histogram approximation,5000bins', epsilon=1e-6))
    result = dict(status='NERO_TRAINING_DATA_PREP_READY', VALID_CHUNK_SAMPLING='PASS', NERO_NORM_STATS='PASS',
        NORMALIZED_OPENPI_LOADER='PASS', original_frames=682, valid_starts=673, start_range=[0,672], horizon=10,
        statistics_counts=counts, stats_path=str(stats_dir/'norm_stats.json'), stats_sha256=sha(stats_dir/'norm_stats.json'),
        stats_dimensions=dict(state=9,actions=8), use_quantile_norm=True, skip_norm_stats=False,
        real_apis=['create_torch_dataset','transform_dataset(skip_norm_stats=False)','TorchDataLoader','DataLoaderImpl'],
        reusable_training_entry='nero_training_data.create_training_loader(stats_directory,model_config,...)',
        complete_batches_seen=batch_seen,batch_size=1,tail_padding_count=0,selected_raw_chunks=selected,
        selected_batches=batch_examples, roundtrip_max_per_dimension=roundtrip.tolist(),
        roundtrip_tolerance_per_dimension=ROUNDTRIP_TOLERANCE.tolist(),
        normalized_action_min=normalized_min.tolist(),normalized_action_max=normalized_max.tolist(),
        actual_normalization_applied=changed_from_physical, physical_clipping=False,
        camera_masks=expected_masks, prompt=protocol['task'], input_sha256_unchanged=hashes,
        versions={k:metadata.version(k) for k in ('openpi','lerobot','torch','numpy','jax','pyarrow','torchcodec')},
        openpi_commit=commit,lerobot_commit=protocol['lerobot_commit'],
        code_sha256={str(p):sha(p) for p in sources}, new_physics_steps=0,training=False,weights=False)
    write(output/'result.json', result)
    write(output/'manifest.json',{str(p.relative_to(output)):dict(sha256=sha(p),bytes=p.stat().st_size)
          for p in output.rglob('*') if p.is_file()})
    print(json.dumps(dict(status=result['status'],stats_sha256=result['stats_sha256'],
                         roundtrip_max=roundtrip.max(),batches=batch_seen)),flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode',type=Path,default=EPISODE);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();prepare(args.episode.resolve(),args.output.resolve())
