"""Minimal single-camera, 9D-state/8D-absolute-action OpenPI adapter.

No delta conversion, no finger-width rescaling, no force-command decoding.
Absent wrist slots are zero padding with false masks, not extra observations.
"""
import dataclasses
import numpy as np


@dataclasses.dataclass(frozen=True)
class NeroInputs:
    def __call__(self,data):
        image=np.asarray(data['observation/image'])
        if image.ndim!=3:raise ValueError('Expected one RGB image')
        if image.shape[0]==3:image=image.transpose(1,2,0)
        if np.issubdtype(image.dtype,np.floating):image=np.rint(image*255).clip(0,255).astype(np.uint8)
        if image.dtype!=np.uint8 or image.shape[-1]!=3:raise ValueError('Expected RGB uint8')
        state=np.asarray(data['observation/state'],dtype=np.float32)
        if state.shape!=(9,):raise ValueError('NERO state must contain seven arm + two actual finger positions')
        result={'state':state,'image':{'base_0_rgb':image,'left_wrist_0_rgb':np.zeros_like(image),'right_wrist_0_rgb':np.zeros_like(image)},
                'image_mask':{'base_0_rgb':True,'left_wrist_0_rgb':False,'right_wrist_0_rgb':False}}
        if 'actions' in data:
            action=np.asarray(data['actions'],dtype=np.float32)
            if action.shape[-1]!=8:raise ValueError('Expected 8D absolute position targets')
            result['actions']=action
        if 'actions_is_pad' in data:result['actions_is_pad']=np.asarray(data['actions_is_pad'],dtype=bool)
        if 'prompt' in data:result['prompt']=data['prompt']
        return result


@dataclasses.dataclass(frozen=True)
class NeroOutputs:
    def __call__(self,data):
        action=np.asarray(data['actions'])
        if action.shape[-1]<8:raise ValueError('Eight action dimensions required; do not use LiberoOutputs')
        return {'actions':action[...,:8]}


def make_data_config(repo_id,model_config):
    from openpi import transforms
    from openpi.training import config
    return config.DataConfig(repo_id=repo_id,prompt_from_task=True,action_sequence_keys=('actions',),
        repack_transforms=transforms.Group(inputs=[transforms.RepackTransform({
            'observation/state':'observation.state','observation/image':'observation.images.base',
            'actions':'actions','actions_is_pad':'actions_is_pad','prompt':'prompt'})]),
        data_transforms=transforms.Group(inputs=[NeroInputs()],outputs=[NeroOutputs()]),
        model_transforms=config.ModelTransformFactory()(model_config),use_quantile_norm=True)
