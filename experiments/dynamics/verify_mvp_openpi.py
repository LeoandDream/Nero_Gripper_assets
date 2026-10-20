"""Issue27 real locked LeRobot/OpenPI PI05 reader/transform/batch smoke.

No training, model weights or MuJoCo imports. Outputs require a new path.
"""
import argparse
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image
import pyarrow.parquet as pq

from mvp_lerobot_episode import ROOT, context, sha, write
from nero_openpi import NeroOutputs, make_data_config


def verify(output,report):
    if report.exists():raise FileExistsError(report)
    p,plan=context(output)
    expected_cache=output/'lerobot'
    if Path(os.environ.get('HF_LEROBOT_HOME','')).resolve()!=expected_cache.resolve():
        raise ValueError('Set HF_LEROBOT_HOME to the experiment lerobot directory before importing loaders')
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, HF_LEROBOT_HOME
    import lerobot.common.datasets.lerobot_dataset as lerobot_module
    from openpi.training import data_loader
    from openpi.models.pi0_config import Pi0Config
    import openpi.transforms as transforms
    import torch
    import openpi
    root=output/'lerobot'/p['repo_id']
    manifest=json.loads((output/'dataset-manifest.json').read_text())
    for name,record in manifest['data_files'].items():assert sha(root/name)==record['sha256'],name
    direct=json.loads(metadata.distribution('lerobot').read_text('direct_url.json'))
    assert direct['vcs_info']['commit_id']==p['lerobot_commit']
    openpi_root=Path(openpi.__file__).resolve().parents[2]
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=openpi_root,text=True).strip()==p['openpi_commit']
    tokenizer=Path(os.environ['OPENPI_DATA_HOME'])/'big_vision/paligemma_tokenizer.model'
    assert tokenizer.is_file(),'Use the existing tokenizer cache; do not download weights'
    model=Pi0Config(pi05=True,action_horizon=10,action_dim=32)
    config=make_data_config(p['repo_id'],model)
    # Actual OpenPI creates the real LeRobotDataset and the task->prompt wrapper.
    raw=data_loader.create_torch_dataset(config,10,model)
    transformed=data_loader.transform_dataset(raw,config,skip_norm_stats=True)
    side=np.load(output/'sidecar.npz',allow_pickle=False);n=plan['frames']
    assert len(raw)==len(transformed)==n
    table=pq.read_table(next(root.rglob('*.parquet'))).to_pydict()
    np.testing.assert_array_equal(np.asarray(table['observation.state'],dtype=np.float32),side['state'])
    np.testing.assert_array_equal(np.asarray(table['actions'],dtype=np.float32),side['actions'])
    np.testing.assert_array_equal(table['frame_index'],np.arange(n))
    np.testing.assert_allclose(table['timestamp'],np.arange(n)/20,rtol=0,atol=4e-6)
    assert set(table['episode_index'])=={0} and set(table['task_index'])=={0}
    checks=[]
    for idx in (0,n//2,n-10,n-9,n-1):
        value=raw[idx];indices=np.minimum(np.arange(idx,idx+10),n-1);pad=np.arange(idx,idx+10)>=n
        image=np.asarray(value['observation.images.base'])
        assert image.shape==(3,256,256) and image.dtype==np.float32
        assert 0<=image.min()<=image.max()<=1
        assert value['prompt']==p['task']
        np.testing.assert_array_equal(value['observation.state'],side['state'][idx])
        np.testing.assert_array_equal(value['actions'],side['actions'][indices])
        np.testing.assert_array_equal(value['actions_is_pad'],pad)
        mapped=transformed[idx]
        np.testing.assert_array_equal(mapped['state'][:9],side['state'][idx]);assert not mapped['state'][9:].any()
        np.testing.assert_array_equal(NeroOutputs()(mapped)['actions'],side['actions'][indices])
        assert not mapped['actions'][:,8:].any()
        np.testing.assert_array_equal(mapped['actions_is_pad'],pad)
        assert mapped['image_mask']=={'base_0_rgb':True,'left_wrist_0_rgb':False,'right_wrist_0_rgb':False}
        assert mapped['image']['base_0_rgb'].shape==(224,224,3)
        assert mapped['tokenized_prompt'].shape==(200,) and mapped['tokenized_prompt_mask'].any()
        original=np.asarray(Image.open(output/'rgb'/f'frame_{idx:06d}.png'),dtype=np.float32)
        decoded=image.transpose(1,2,0)*255
        checks.append(dict(frame_index=idx,source_observation_row=int(side['source_rows'][idx]),source_action_row=int(side['source_action_rows'][idx]),
            source_time_s=float((side['source_rows'][idx]+1)*.001),dataset_time_s=idx/20,
            state=np.asarray(value['observation.state']).tolist(),first_action=np.asarray(value['actions'][0]).tolist(),
            raw_image_shape=list(image.shape),raw_image_dtype=str(image.dtype),raw_action_shape=list(value['actions'].shape),
            model_state_shape=list(mapped['state'].shape),model_action_shape=list(mapped['actions'].shape),
            actions_is_pad=pad.tolist(),valid_actions=int((~pad).sum()),prompt=value['prompt'],
            video_decode_mean_absolute_error_255=float(abs(original-decoded).mean()),
            action_roundtrip_8d='PASS'))
    # Use the original OpenPI public loader, not a custom NumPy batching path.
    loader=data_loader.create_torch_data_loader(config,model,10,3,shuffle=False,num_batches=1,
        num_workers=0,skip_norm_stats=True,framework='pytorch')
    obs,actions=next(iter(loader))
    assert tuple(obs.state.shape)==(3,32) and tuple(actions.shape)==(3,10,32)
    # The pinned Observation.from_dict explicitly permutes torch.uint8 NHWC
    # to NCHW while normalizing to [-1,1]; transformed samples remain HWC.
    assert tuple(obs.images['base_0_rgb'].shape)==(3,3,224,224)
    for i in range(3):
        np.testing.assert_array_equal(obs.state[i,:9].numpy(),side['state'][i])
        np.testing.assert_array_equal(actions[i,:,:8].numpy(),side['actions'][i:i+10])
    assert not obs.image_masks['left_wrist_0_rgb'].any() and not obs.image_masks['right_wrist_0_rgb'].any()
    report.parent.mkdir(parents=True,exist_ok=True)
    result=dict(status='LEROBOT_EPISODE_READY',episodes=1,frames=n,fps=20,new_mj_steps=0,training=False,checkpoint_download=False,
        dataset_root=str(root),selected_samples=checks,all_frame_state_action_alignment='PASS',
        batch=dict(real_api='openpi.training.data_loader.create_torch_data_loader',framework='pytorch',
            state_shape=list(obs.state.shape),action_shape=list(actions.shape),image_shape=list(obs.images['base_0_rgb'].shape),
            image_layout='NCHW (pinned OpenPI Observation.from_dict torch branch)',
            image_dtype=str(obs.images['base_0_rgb'].dtype),image_range=[float(obs.images['base_0_rgb'].min()),float(obs.images['base_0_rgb'].max())],
            camera_masks={k:v.tolist() for k,v in obs.image_masks.items()},token_shape=list(obs.tokenized_prompt.shape)),
        transforms=dict(repack=[type(x).__name__ for x in config.repack_transforms.inputs],
            data=[type(x).__name__ for x in config.data_transforms.inputs],model=[type(x).__name__ for x in config.model_transforms.inputs],
            skip_norm_stats=True,no_delta_actions=True,tail_policy=p['tail'],
            training_requirements='Compute suitable NERO norm stats; exclude padded starts or implement a loss mask. Current DataLoaderImpl drops auxiliary actions_is_pad from model output; preserved and checked on raw/transformed samples.'),
        versions={key:metadata.version(key) for key in ('openpi','lerobot','torch','torchvision','datasets','pyarrow','numpy','jax','av','torchcodec')},
        openpi_commit=p['openpi_commit'],lerobot_commit=p['lerobot_commit'],tokenizer_sha256=sha(tokenizer),
        code_sha256={str(path):sha(path) for path in [Path(__file__),ROOT/'experiments/dynamics/nero_openpi.py',
            Path(data_loader.__file__),Path(transforms.__file__),Path(lerobot_module.__file__)]},
        protocol_sha256=sha(ROOT/'reports/issue27-protocol.json'),dataset_manifest_sha256=sha(output/'dataset-manifest.json'),argv=sys.argv)
    write(report,result)
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--report',type=Path,required=True)
    args=p.parse_args();verify(args.output.resolve(),args.report.resolve())
