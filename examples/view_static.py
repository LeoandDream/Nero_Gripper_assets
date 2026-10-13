"""View a fixed source-consistent pose; never integrate physics or apply actions."""
import argparse
from pathlib import Path
import time

import mujoco
import numpy as np


def setup_view(model, data, show_collision=False):
    option = mujoco.MjvOption()
    option.geomgroup[0] = int(show_collision)
    option.geomgroup[1] = 1
    visible = np.flatnonzero(model.geom_group == 1)
    centers = data.geom_xpos[visible]
    radii = model.geom_rbound[visible, None]
    low, high = (centers - radii).min(axis=0), (centers + radii).max(axis=0)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (low + high) / 2
    camera.distance = max(.5, float(np.linalg.norm(high - low)) * 1.2)
    camera.azimuth, camera.elevation = 135, -20
    return option, camera


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, help='export a static image without opening a window')
    parser.add_argument('--collision', action='store_true', help='also show convex collision approximations')
    parser.add_argument('--width', type=float, default=.08, help='source gripper command coordinate, 0..0.1 m')
    args = parser.parse_args()
    if not 0 <= args.width <= .1:
        parser.error('--width must be in 0..0.1 m')
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).parent / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for name, value in [('gripper_joint1', args.width / 2), ('gripper_joint2', -args.width / 2)]:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = value
    mujoco.mj_forward(model, data)
    option, camera = setup_view(model, data, args.collision)
    if args.image:
        from PIL import Image
        with mujoco.Renderer(model, height=480, width=640) as renderer:
            renderer.update_scene(data, camera=camera, scene_option=option)
            Image.fromarray(renderer.render()).save(args.image)
        print(args.image)
    else:
        from mujoco import viewer as mjviewer
        with mjviewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth, viewer.cam.elevation = camera.azimuth, camera.elevation
            viewer.opt.geomgroup[:] = option.geomgroup
            while viewer.is_running():
                viewer.sync()
                time.sleep(.05)


if __name__ == '__main__':
    main()
