"""One-dimensional MVP gripper feedback; one short empty-load sequence per run."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT/'assets/components/gripper'
MODEL = COMPONENT/'gripper.xml'
MANIFEST = COMPONENT/'manifest.json'
SEQUENCE = [('open', .1, 1.2), ('middle', .04, 2.5), ('close', 0., 1.9), ('reopen', .1, 3.9)]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checkout_head():
    """Optional checkout identity; do not invent a HEAD for a source ZIP."""
    if not (ROOT/'.git').exists():
        return None
    try:
        return subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True,stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def write(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


class GripperController:
    """Same MVP proportional+bias feedforward and native velocity feedback.

    User target is one absolute width command, not 2D force or 9D robot ctrl.
    No qpos writes, external forces, actuator/solver modification or extra DOF.
    """
    def __init__(self, model):
        if (model.nq, model.nv, model.nu, model.neq) != (2, 2, 2, 1):
            raise ValueError('Requires the standalone 2-slide component')
        self.model = model
        self.cfg = json.loads(MANIFEST.read_text())['control']
        names = ('gripper_joint1', 'gripper_joint2')
        jids = np.array([mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in names])
        self.aids = np.array([mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,'servo_'+n) for n in names])
        if np.any(jids < 0) or np.any(self.aids < 0):
            raise ValueError('Missing gripper joint or actuator')
        self.qadr, self.vadr = model.jnt_qposadr[jids], model.jnt_dofadr[jids]
        if model.opt.timestep != self.cfg['timestep_s']:
            raise ValueError('Unexpected model timestep')
        self.requested = self.command = self.cfg['initial_width_m']

    def set_target(self, width):
        value = np.asarray(width)
        if value.shape not in ((), (1,)) or value.dtype.kind not in 'fiu':
            raise ValueError('Expected one real absolute width command in meters')
        value = float(value.reshape(-1)[0])
        if not np.isfinite(value) or not 0 <= value <= .1:
            raise ValueError('Width must be finite and inside [0, 0.1] m')
        self.requested = value

    def advance_command(self):
        step = self.cfg['gripper_command_slew_m_s']*self.cfg['command_period_s']
        self.command += float(np.clip(self.requested-self.command, -step, step))

    @property
    def physical_target(self):
        return np.array([self.command/2, -self.command/2])

    def apply(self, data):
        q, v, bias = data.qpos[self.qadr], data.qvel[self.vadr], data.qfrc_bias[self.vadr]
        if not np.isfinite(np.r_[q,v,bias]).all():
            raise ValueError('Nonfinite gripper state')
        effort = self.cfg['finger_kp_N_m']*(self.physical_target-q)+bias
        limit = self.cfg['finger_force_limit_N']
        data.ctrl[self.aids] = np.clip(effort, -limit, limit)
        return data.ctrl[self.aids].copy()


def plot(trajectory, output):
    """Saved physical samples only; no model load, collision call or integration."""
    from PIL import Image, ImageDraw, ImageFont
    data = np.load(trajectory)
    image = Image.new('RGB',(1100,760),'white')
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=17)
    draw.text((30,12),f"Standalone Gripper: one empty-load sequence ({len(data['time'])} physical samples)",fill='black',font=font)
    t = data['time']; target = data['requested_width']; q = data['qpos']
    panels = [('Width command / measured (mm)', [(target*1000,'gray'),((q[:,0]-q[:,1])*1000,'#1761ad')]),
              ('Per-finger target error (mm)', [(np.max(abs(q-np.c_[target/2,-target/2]),axis=1)*1000,'#ba3535')]),
              ('Coupling |q1 + q2| (micrometers)', [(abs(q.sum(axis=1))*1e6,'#27804c')])]
    for i,(label,curves) in enumerate(panels):
        y0=70+i*225; x0,x1=95,1060; y1=y0+165
        maximum=max(max(float(np.max(v)) for v,_ in curves),.001)
        draw.text((30,y0-26),label,fill='black',font=font)
        draw.line([(x0,y0),(x0,y1),(x1,y1)],fill='black')
        for tick in range(4):
            y=y1-tick*(y1-y0)/3
            draw.text((10,y-7),f'{maximum*tick/3:.3g}',fill='black',font=font)
        for tick in (0,2,4,6,8):
            x=x0+tick/t[-1]*(x1-x0)
            draw.text((x,y1+4),str(tick),fill='black',font=font)
        for values,color in curves:
            draw.line(list(zip(x0+t/t[-1]*(x1-x0),y1-values/maximum*(y1-y0))),fill=color,width=2)
    draw.text((980,735),'Time (s)',fill='black',font=font)
    image.save(output)


def run(output):
    manifest = json.loads(MANIFEST.read_text())
    for name, expected in manifest['runtime_files_sha256'].items():
        if sha(ROOT/name) != expected:
            raise ValueError('Component runtime file changed: '+name)
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    control = GripperController(model)
    data = mujoco.MjData(model)
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(__file__, output/'script.executed.py')
    write(output/'before-run.json',dict(argv=sys.argv,head=checkout_head(),checkout_head_optional=True,
        asset_delivery_commit='77bd26a58df4eda99f611de7be1013d714ddaa7f',
        component_delivery_commit='0d944a2e384c838ea1d72c8655d5efb15e5897d0',
        python=sys.version,platform=platform.platform(),mujoco=mujoco.__version__,numpy=np.__version__,
        script_sha256=sha(__file__),manifest_sha256=sha(MANIFEST),runtime_files_sha256=manifest['runtime_files_sha256']))
    write(output/'protocol.json',dict(initial_width_m=control.command,sequence=SEQUENCE,
        control=control.cfg,settled_window_s=.25,maximum_physical_sequences=1,expected_mj_steps=9500))
    # Initialization only, before the first mj_step. No state resets in the sequence.
    data.qpos[control.qadr] = control.physical_target
    mujoco.mj_forward(model,data)
    period = int(round(control.cfg['command_period_s']/model.opt.timestep))
    rows = {k:[] for k in ('time','phase','qpos','qvel','requested_width','command_width','ctrl','actuator_force',
            'contact_count','minimum_contact_distance','solver_niter','qfrc_applied','xfrc_applied')}
    phases=[]; actual_steps=0; error=None; started=time.monotonic()
    try:
        for name,width,duration in SEQUENCE:
            control.set_target(width)
            start=len(rows['time'])
            for k in range(int(round(duration/model.opt.timestep))):
                if k%period == 0:
                    control.advance_command()
                control.apply(data)
                before=float(data.time)
                mujoco.mj_step(model,data)
                actual_steps+=1
                if not np.isclose(data.time,before+model.opt.timestep,rtol=0,atol=1e-10):
                    raise ValueError('Physics time reset/discontinuity')
                mujoco.mj_forward(model,data)
                if np.any(data.warning.number):
                    raise ValueError('MuJoCo numerical warning')
                if not np.isfinite(np.r_[data.qpos,data.qvel,data.ctrl,data.actuator_force]).all():
                    raise ValueError('Nonfinite physical state')
                if np.any(data.qfrc_applied) or np.any(data.xfrc_applied):
                    raise ValueError('Unexpected external applied force')
                values=dict(time=float(data.time),phase=name,qpos=data.qpos.copy(),qvel=data.qvel.copy(),
                    requested_width=control.requested,command_width=control.command,ctrl=data.ctrl.copy(),
                    actuator_force=data.actuator_force.copy(),contact_count=data.ncon,
                    minimum_contact_distance=min([float(c.dist) for c in data.contact[:data.ncon]] or [0.]),
                    solver_niter=int(np.max(data.solver_niter)),qfrc_applied=data.qfrc_applied.copy(),
                    xfrc_applied=data.xfrc_applied.copy())
                for key,value in values.items(): rows[key].append(value)
            settled=np.asarray(rows['qpos'][start:])[-int(round(.25/model.opt.timestep)):]
            finger_error=np.max(abs(settled-np.array([width/2,-width/2])),axis=0)
            phases.append(dict(name=name,target_width_m=width,duration_s=duration,
                settled_max_finger_error_m=finger_error.tolist(),
                passed=bool(np.max(finger_error)<=control.cfg['acceptance']['settled_finger_error_m'])))
            print(json.dumps(phases[-1]),flush=True)
    except Exception:
        import traceback
        error=traceback.format_exc()
    finally:
        arrays={k:np.asarray(v) for k,v in rows.items()}
        np.savez_compressed(output/'trajectory.npz',**arrays)
        if len(rows['time']):
            q,v=arrays['qpos'],arrays['qvel']
            coupling=float(np.max(abs(q.sum(axis=1))))
            velocity=float(np.max(abs(v)))
            force=float(np.max(abs(arrays['actuator_force'])))
            lower,upper=model.jnt_range.T
            limit_violation=float(np.max(np.maximum(np.maximum(lower-q,q-upper),0)))
        else:coupling=velocity=force=limit_violation=None
        limits=control.cfg['acceptance']
        passed=(error is None and actual_steps==9500 and all(p['passed'] for p in phases) and
                coupling<=limits['maximum_coupling_error_m'] and velocity<=limits['maximum_finger_velocity_m_s'] and
                force<=control.cfg['finger_force_limit_N']+1e-9)
        result=dict(status='PASS' if passed else 'FAIL',actual_mj_steps=actual_steps,simulation_time_s=float(data.time),
            wall_s=time.monotonic()-started,error=error,phases=phases,max_coupling_error_m=coupling,
            max_finger_velocity_m_s=velocity,max_actuator_force_N=force,max_joint_limit_violation_m=limit_violation,
            max_penetration_m=max([max(0.,-x) for x in rows['minimum_contact_distance']] or [0.]),
            max_contacts=max(rows['contact_count'] or [0]),max_solver_niter=max(rows['solver_niter'] or [0]),
            warning_counts=data.warning.number.tolist(),qpos_writes_during_execution=0,external_force_shortcuts=False,
            verification_scope='Standalone fixed mount, empty-load control only; no grasp/dual-arm/hardware certification')
        write(output/'result.json',result)
        for path,expected in manifest['runtime_files_sha256'].items():
            if sha(ROOT/path)!=expected:raise ValueError('Runtime input modified: '+path)
        if len(rows['time']):plot(output/'trajectory.npz',output/'opening.png')
        write(output/'files.json',{p.name:dict(sha256=sha(p),bytes=p.stat().st_size) for p in output.iterdir() if p.is_file()})
        print(json.dumps(result),flush=True)
    return passed


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='New directory; never overwrite an earlier run')
    args=parser.parse_args()
    raise SystemExit(0 if run(args.output.resolve()) else 1)
