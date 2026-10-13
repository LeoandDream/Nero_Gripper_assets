"""8D position targets, native bounded force servos, and protected-model derivation."""
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent
ASSET_ROOT=HERE.parents[1]
JOINTS=[f'joint{i}' for i in range(1,8)]+['gripper_joint1','gripper_joint2']


def load_config():
    return json.loads((HERE/'config.json').read_text())


def build_model(path,config,*,exclude_base_link1=False,cube=None):
    """Only change independent experiment XML; no source asset writes."""
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    source=ASSET_ROOT/'mjcf/nero_gripper.xml'
    root=ET.parse(source).getroot()
    for node in root.iter():
        if node.get('file'):
            node.set('file',os.path.relpath((source.parent/node.get('file')).resolve(),path.parent))
    for frame in root.findall('keyframe'):root.remove(frame)
    ET.SubElement(root,'option',timestep=str(config['timestep_s']),integrator=config['integrator'],gravity=' '.join(map(str,config['gravity_m_s2'])))
    actuator=ET.SubElement(root,'actuator')
    limits=config['arm_force_limits_Nm']+[config['finger_force_limit_N']]*2
    damping=config['arm_kv_Nms_rad']+[config['finger_kv_Ns_m']]*2
    for name,limit,kv in zip(JOINTS,limits,damping):
        ET.SubElement(actuator,'general',name='servo_'+name,joint=name,gear='1',dyntype='none',gaintype='fixed',gainprm='1',biastype='affine',biasprm=f'0 0 {-kv}',ctrllimited='true',ctrlrange=f'{-limit} {limit}',forcelimited='true',forcerange=f'{-limit} {limit}')
    gripper=root.find(".//body[@name='gripper_base']")
    ET.SubElement(gripper,'site',name='grasp_center',pos='0 0 0.1',size='.002',rgba='0 0 0 0')
    if exclude_base_link1:
        # This explicit diagnostic/adopted candidate requires recorded user approval.
        ET.SubElement(ET.SubElement(root,'contact'),'exclude',body1='base_link',body2='link1')
    if cube is not None:
        world=root.find('worldbody')
        table=cube['table']
        ET.SubElement(world,'geom',name='table',type='box',pos=' '.join(map(str,table['center'])),size=' '.join(map(str,table['half_size'])),rgba='.55 .55 .55 1')
        body=ET.SubElement(world,'body',name='cube',pos=' '.join(map(str,cube['center'])))
        ET.SubElement(body,'freejoint',name='cube_free')
        ET.SubElement(body,'geom',name='cube_geom',type='box',size=' '.join(map(str,cube['half_size'])),mass=str(cube['mass_kg']),friction=' '.join(map(str,cube['friction'])),rgba='.1 .45 .9 1')
    world=root.find('worldbody')
    ET.SubElement(world,'light',name='experiment_key',pos='1 -2 3',dir='-.2 .3 -1',ambient='.3 .3 .3')
    ET.indent(root)
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    model=mujoco.MjModel.from_xml_path(str(path))
    if model.nu!=9 or model.neq!=1 or model.nexclude!=int(exclude_base_link1):
        raise ValueError('Unexpected derived actuator/constraint/filter structure')
    return model


class Controller:
    """Position feedback and model bias feedforward; velocity feedback is native.

    User target is 8D; internal ctrl is 9 bounded Nm/N inputs. No qpos writes,
    qfrc_applied, object attachment, or contact parameter modification.
    """
    def __init__(self,model,config):
        self.model=model;self.cfg=config
        self.jids=np.array([mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in JOINTS])
        self.aids=np.array([mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,'servo_'+n) for n in JOINTS])
        if np.any(self.jids<0) or np.any(self.aids<0):raise ValueError('Missing actuator/joint')
        self.qadr=model.jnt_qposadr[self.jids];self.vadr=model.jnt_dofadr[self.jids]
        self.limits=model.jnt_range[self.jids[:7]].copy()
        self.limits[1]=config['joint2_command_envelope_rad']
        self.kp=np.array(config['arm_kp_Nm_rad']+[config['finger_kp_N_m']]*2)
        self.force_limits=np.array(config['arm_force_limits_Nm']+[config['finger_force_limit_N']]*2)
        self.slew=np.r_[np.full(7,config['arm_target_slew_rad_s']),config['gripper_command_slew_m_s']]
        self.reset_target(config['initial_action'])

    def validate(self,action):
        value=np.asarray(action)
        if value.shape!=(8,) or value.dtype.kind not in 'fiu':
            raise ValueError('Expected finite real 8D absolute position target')
        value=value.astype(float,copy=True)
        if not np.isfinite(value).all():raise ValueError('Non-finite target')
        if np.any(value[:7]<self.limits[:,0]) or np.any(value[:7]>self.limits[:,1]):
            raise ValueError('Joint target outside allowed command envelope')
        if not 0<=value[7]<=.1:raise ValueError('Gripper command outside0..0.1m')
        return value

    def reset_target(self,action):
        self.requested=self.validate(action)
        self.command=self.requested.copy()

    def set_target(self,action):
        self.requested=self.validate(action)

    def advance_command(self):
        step=self.slew*self.cfg['command_period_s']
        self.command+=np.clip(self.requested-self.command,-step,step)

    @property
    def physical_target(self):
        return np.r_[self.command[:7],self.command[7]/2,-self.command[7]/2]

    def apply(self,data):
        effort=self.kp*(self.physical_target-data.qpos[self.qadr])+data.qfrc_bias[self.vadr]
        data.ctrl[self.aids]=np.clip(effort,-self.force_limits,self.force_limits)
        return data.ctrl[self.aids].copy()
