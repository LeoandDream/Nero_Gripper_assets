"""Reusable NERO training data entry: complete chunks BEFORE batch/shuffle.

Uses the locked OpenPI/LeRobot dataset, transforms and batch implementations.
No upstream patches, weights, optimizer, simulation or source-data writes.
"""
import dataclasses
import operator
from pathlib import Path

import numpy as np

from nero_openpi import make_data_config

REPO_ID = 'local/nero_mvp_r1'


def valid_chunk_starts(bounds, total_frames, horizon):
    if horizon < 1: raise ValueError('Action horizon must be positive')
    starts = []; previous = 0
    for begin, end in bounds:
        begin, end = int(begin), int(end)
        if begin != previous or not begin < end <= total_frames:
            raise ValueError('Episode boundaries must partition the dataset without gaps/overlap')
        starts.extend(range(begin, end-horizon+1)); previous = end
    if previous != total_frames: raise ValueError('Episode boundaries do not cover the dataset')
    return np.asarray(starts, dtype=np.int64)


class CompleteActionChunks:
    """View over immutable frames; only training start indices are filtered."""
    def __init__(self, dataset, bounds, horizon):
        self.dataset = dataset; self.horizon = horizon
        self.starts = valid_chunk_starts(bounds, len(dataset), horizon)
        self.starts.setflags(write=False)

    def __len__(self): return len(self.starts)

    def __getitem__(self, index):
        index = operator.index(index)
        if not 0 <= index < len(self): raise IndexError(index)
        item = self.dataset[int(self.starts[index])]
        pad = np.asarray(item['actions_is_pad'], dtype=bool)
        if pad.shape != (self.horizon,) or pad.any():
            raise ValueError('Training view encountered temporal padding')
        if np.asarray(item['actions']).shape != (self.horizon, 8):
            raise ValueError('Expected complete horizon x 8 absolute action chunk')
        return item


def create_training_view(data_config, model_config):
    from lerobot.common.datasets.lerobot_dataset import LeRobotDatasetMetadata
    from lerobot.common.datasets.utils import get_episode_data_index
    from openpi.training import data_loader
    raw = data_loader.create_torch_dataset(data_config, model_config.action_horizon, model_config)
    meta = LeRobotDatasetMetadata(data_config.repo_id)
    edges = get_episode_data_index(meta.episodes)
    bounds = list(zip(edges['from'].tolist(), edges['to'].tolist()))
    return CompleteActionChunks(raw, bounds, model_config.action_horizon)


def load_training_config(stats_directory, model_config):
    from openpi.shared import normalize
    stats = normalize.load(Path(stats_directory))  # No empty stats or LIBERO fallback.
    if set(stats) != {'state', 'actions'}: raise ValueError('NERO state/action stats required')
    for key, dimension in [('state', 9), ('actions', 8)]:
        for name in ('mean', 'std', 'q01', 'q99'):
            value = np.asarray(getattr(stats[key], name))
            if value.shape != (dimension,) or not np.isfinite(value).all():
                raise ValueError(f'Invalid physical {dimension}D {key}.{name} statistics')
    return dataclasses.replace(make_data_config(REPO_ID, model_config), norm_stats=stats)


def create_training_loader(stats_directory, model_config, *, batch_size=1, shuffle=True,
                           num_batches=None, num_workers=0, framework='pytorch'):
    """Future training code must use this factory, not unfiltered upstream factory.

    Returns the original OpenPI DataLoaderImpl. The underlying dataset ALREADY
    has only valid starts before transforms, batching or shuffle. Default B=1
    avoids the pinned upstream drop_last behavior; other batch sizes retain
    that upstream behavior (no invalid starts can enter any batch).
    """
    from openpi.training import data_loader
    config = load_training_config(stats_directory, model_config)
    view = create_training_view(config, model_config)
    dataset = data_loader.transform_dataset(view, config, skip_norm_stats=False)
    batches = data_loader.TorchDataLoader(dataset, local_batch_size=batch_size,
        shuffle=shuffle, num_batches=num_batches, num_workers=num_workers, framework=framework)
    return data_loader.DataLoaderImpl(config, batches)


def compute_norm_stats(view, data_config, upstream_directory):
    """Same statistics location/tools as pinned scripts/compute_norm_stats.py.

    Its factory cannot receive a filtered view, so only that construction is
    project-side: batch1, no shuffle, len(view) batches. Reuses RemoveStrings,
    TorchDataLoader and RunningStats without changing the upstream install.
    """
    import importlib.util
    from openpi.shared import normalize
    from openpi.training import data_loader
    script = Path(upstream_directory) / 'scripts/compute_norm_stats.py'
    spec = importlib.util.spec_from_file_location('locked_compute_norm_stats', script)
    official = importlib.util.module_from_spec(spec); spec.loader.exec_module(official)
    dataset = data_loader.TransformedDataset(view, [*data_config.repack_transforms.inputs,
        *data_config.data_transforms.inputs, official.RemoveStrings()])
    batches = data_loader.TorchDataLoader(dataset, local_batch_size=1, num_workers=0,
        shuffle=False, num_batches=len(view), framework='pytorch')
    running = {key: normalize.RunningStats() for key in ('state', 'actions')}
    values = {key: [] for key in running}; seen = 0
    for batch in batches:
        for key in running:
            value = np.asarray(batch[key])
            if not np.isfinite(value).all(): raise ValueError(f'Nonfinite {key}')
            expected = (1, 9) if key == 'state' else (1, view.horizon, 8)
            if value.shape != expected: raise ValueError(f'Wrong statistics position: {value.shape}')
            running[key].update(value); values[key].append(value[0])
        seen += 1
    if seen != len(view): raise ValueError('Statistics did not visit all valid starts')
    return {key: value.get_statistics() for key, value in running.items()}, values
