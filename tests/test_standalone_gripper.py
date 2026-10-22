"""Issue35: source-vs-component checks only; never calls mj_step."""
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / 'assets/components/gripper'
RESULTS = {}


def named(model, kind, name):
    value = mujoco.mj_name2id(model, getattr(mujoco.mjtObj, 'mjOBJ_' + kind), name)
    if value < 0:
        raise AssertionError('Missing ' + name)
    return value


def contact_qualification(model, ids):
    """MuJoCo3.2.3 filterBodyPair + bitmask + explicit body exclusions.

    No geometry query; compares eligibility for every component geom pair.
    https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_collision_driver.c
    """
    bodies = model.geom_bodyid[ids]
    weld = model.body_weldid[bodies]
    parent = model.body_weldid[model.body_parentid[weld]]
    same = weld[:,None] == weld[None,:]
    disabled = bool(model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT))
    parent_filter = (not disabled) & (weld[:,None]!=0) & (weld[None,:]!=0) & (
        (weld[:,None]==parent[None,:]) | (weld[None,:]==parent[:,None]))
    signature = (np.minimum(bodies[:,None],bodies[None,:]).astype(np.int64)<<16) + np.maximum(bodies[:,None],bodies[None,:])
    excluded = np.isin(signature,model.exclude_signature)
    types, affinity = model.geom_contype[ids], model.geom_conaffinity[ids]
    mask = ((types[:,None]&affinity[None,:])!=0) | ((types[None,:]&affinity[:,None])!=0)
    return mask & ~(same | parent_filter | excluded)


class StandaloneGripper(unittest.TestCase):
    def models(self):
        self.assertTrue((COMPONENT / 'gripper.xml').is_file(), 'Standalone MJCF not delivered')
        if not hasattr(type(self), '_models'):
            type(self)._models = (
                mujoco.MjModel.from_xml_path(str(ROOT / 'mjcf/simulation_mvp.xml')),
                mujoco.MjModel.from_xml_path(str(COMPONENT / 'gripper.xml')),
            )
        return self._models

    def test_standalone_structure_and_unchanged_dependencies(self):
        source, model = self.models()
        self.assertEqual(mujoco.__version__, '3.2.3')
        self.assertEqual((model.nq, model.nv, model.nu, model.neq), (2, 2, 2, 1))
        self.assertEqual(model.ngeom, 699)
        self.assertEqual(model.nmesh, 353)
        for name in ('cube', 'link7', 'flange_link', 'base_link'):
            self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name), -1)
        np.testing.assert_array_equal(model.body_pos[named(model, 'BODY', 'gripper_base')],
                                      source.body_pos[named(source, 'BODY', 'gripper_base')])
        for i in (1, 2):
            b = named(model, 'BODY', 'gripper_link' + str(i))
            ids = np.flatnonzero((model.geom_bodyid == b) & (model.geom_contype != 0))
            self.assertEqual(len(ids), 346)
        manifest = json.loads((COMPONENT / 'manifest.json').read_text())
        original = json.loads((ROOT / 'mjcf/simulation_mvp.manifest.json').read_text())
        for name, expected in manifest['runtime_files_sha256'].items():
            path = ROOT / name
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected, name)
            if name.startswith('meshes/'):
                self.assertEqual(expected, original['input_sha256'][name])
        RESULTS['load'] = {k: int(getattr(model, k)) for k in ('nq', 'nv', 'nu', 'neq', 'ngeom', 'nmesh', 'nbody')}

    def test_compiled_mechanics_materials_and_options_match_source(self):
        source, model = self.models()
        for name in ('gripper_base', 'gripper_link1', 'gripper_link2'):
            a, b = named(source, 'BODY', name), named(model, 'BODY', name)
            for field in ('body_pos', 'body_quat', 'body_mass', 'body_ipos', 'body_iquat', 'body_inertia'):
                np.testing.assert_array_equal(getattr(source, field)[a], getattr(model, field)[b], err_msg=field+name)
        for name in ('gripper_joint1', 'gripper_joint2'):
            a, b = named(source, 'JOINT', name), named(model, 'JOINT', name)
            for field in ('jnt_type', 'jnt_axis', 'jnt_pos', 'jnt_range', 'jnt_limited', 'jnt_solref', 'jnt_solimp'):
                np.testing.assert_array_equal(getattr(source, field)[a], getattr(model, field)[b], err_msg=field+name)
            np.testing.assert_array_equal(source.qpos0[source.jnt_qposadr[a]], model.qpos0[model.jnt_qposadr[b]])
            a, b = named(source, 'ACTUATOR', 'servo_'+name), named(model, 'ACTUATOR', 'servo_'+name)
            for field in ('actuator_gainprm', 'actuator_biasprm', 'actuator_ctrlrange', 'actuator_forcerange',
                          'actuator_gear', 'actuator_dyntype', 'actuator_gaintype', 'actuator_biastype'):
                np.testing.assert_array_equal(getattr(source, field)[a], getattr(model, field)[b], err_msg=field+name)
        for field in ('eq_data', 'eq_solref', 'eq_solimp', 'eq_active0', 'eq_type'):
            np.testing.assert_array_equal(getattr(source, field), getattr(model, field), err_msg=field)
        for field in ('timestep', 'gravity', 'integrator', 'cone', 'impratio', 'solver', 'iterations',
                      'tolerance', 'disableflags', 'enableflags'):
            np.testing.assert_array_equal(getattr(source.opt, field), getattr(model.opt, field), err_msg=field)
        for i in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
            a = named(source, 'GEOM', name)
            self.assertEqual(mujoco.mj_id2name(source,mujoco.mjtObj.mjOBJ_MESH,int(source.geom_dataid[a])),
                             mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_MESH,int(model.geom_dataid[i])))
            self.assertEqual(mujoco.mj_id2name(source,mujoco.mjtObj.mjOBJ_MATERIAL,int(source.geom_matid[a])),
                             mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_MATERIAL,int(model.geom_matid[i])))
            for field in ('geom_type', 'geom_pos', 'geom_quat', 'geom_size', 'geom_friction', 'geom_solref',
                          'geom_solimp', 'geom_condim', 'geom_priority', 'geom_solmix', 'geom_margin', 'geom_gap',
                          'geom_rgba', 'geom_group', 'geom_contype', 'geom_conaffinity'):
                np.testing.assert_array_equal(getattr(source, field)[a], getattr(model, field)[i], err_msg=field+name)
        for i in range(model.nmesh):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, i)
            a = named(source, 'MESH', name)
            for field in ('mesh_pos', 'mesh_quat'):
                np.testing.assert_array_equal(getattr(source, field)[a], getattr(model, field)[i])
            for values, adr, count in (('mesh_vert', 'mesh_vertadr', 'mesh_vertnum'),
                                       ('mesh_face', 'mesh_faceadr', 'mesh_facenum'),
                                       ('mesh_normal', 'mesh_normaladr', 'mesh_normalnum')):
                x, y = getattr(source, adr)[a], getattr(model, adr)[i]
                n, m = getattr(source, count)[a], getattr(model, count)[i]
                self.assertEqual(n, m)
                np.testing.assert_array_equal(getattr(source, values)[x:x+n], getattr(model, values)[y:y+m])
        for i in range(model.nmat):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, i)
            a = named(source, 'MATERIAL', name)
            for field in ('mat_rgba', 'mat_specular', 'mat_shininess', 'mat_texuniform', 'mat_texrepeat'):
                np.testing.assert_array_equal(getattr(source, field)[a], getattr(model, field)[i])
        RESULTS['compiled_source_equivalence'] = dict(bodies=3, joints=2, equality=1, actuators=2,
            geoms=model.ngeom, meshes=model.nmesh, materials=model.nmat, exact=True)

    def test_all_internal_contact_qualifications_match_source(self):
        source,model=self.models()
        original_ids=np.array([named(source,'GEOM',mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i)) for i in range(model.ngeom)])
        a=contact_qualification(source,original_ids)
        b=contact_qualification(model,np.arange(model.ngeom))
        mismatch=int(np.count_nonzero(np.triu(a!=b,1)))
        self.assertEqual(mismatch,0,'Fixed-world extraction changed source contact eligibility')
        RESULTS['contact_qualification']=dict(unordered_geom_pairs=model.ngeom*(model.ngeom-1)//2,
            eligible_pairs=int(np.count_nonzero(np.triu(a,1))),mismatches=mismatch,
            source_excludes=int(source.nexclude),component_excludes=int(model.nexclude),
            policy='Preserve source parent filtering; finger-to-finger eligibility remains enabled')

    def test_installation_frame_transforms_at_three_widths(self):
        source, model = self.models()
        original, standalone = mujoco.MjData(source), mujoco.MjData(model)
        max_position = max_rotation = 0.0
        contacts = []
        for width in (0., .04, .1):
            for m, d in ((source, original), (model, standalone)):
                for name, q in (('gripper_joint1', width/2), ('gripper_joint2', -width/2)):
                    d.qpos[m.jnt_qposadr[named(m, 'JOINT', name)]] = q
                mujoco.mj_forward(m, d)  # static pose, never integration
            frame = named(source, 'BODY', 'flange_link')
            rotation = original.xmat[frame].reshape(3, 3)
            origin = original.xpos[frame]
            for i in range(model.ngeom):
                a = named(source, 'GEOM', mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i))
                position = rotation.T @ (original.geom_xpos[a] - origin)
                orientation = rotation.T @ original.geom_xmat[a].reshape(3, 3)
                max_position = max(max_position, float(np.max(abs(position-standalone.geom_xpos[i]))))
                max_rotation = max(max_rotation, float(np.max(abs(orientation-standalone.geom_xmat[i].reshape(3, 3)))))
            contacts.append(dict(width_m=width, standalone_ncon=int(standalone.ncon),
                pairs=sorted(set(tuple(sorted((mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[c.geom1])),
                    mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[c.geom2])))))
                    for c in standalone.contact[:standalone.ncon])),
                max_penetration_m=max([max(0.,-float(c.dist)) for c in standalone.contact[:standalone.ncon]] or [0.])))
        self.assertLessEqual(max_position, 1e-12)
        self.assertLessEqual(max_rotation, 1e-12)
        RESULTS['frame_equivalence'] = dict(widths_m=[0.,.04,.1], geoms_per_width=model.ngeom,
            max_position_error_m=max_position, max_rotation_matrix_error=max_rotation, static_contacts=contacts)

    def test_one_dimensional_control_contract_without_integration(self):
        _, model = self.models()
        path = ROOT / 'examples/gripper_control.py'
        self.assertTrue(path.is_file(), 'Standalone control entry not delivered')
        spec = importlib.util.spec_from_file_location('gripper_control', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        control = module.GripperController(model)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        old_qpos = data.qpos.copy()
        control.set_target(.1)
        control.advance_command()
        self.assertAlmostEqual(control.command, .0803)
        np.testing.assert_allclose(control.physical_target, [.04015, -.04015], rtol=0, atol=1e-15)
        effort = control.apply(data)
        np.testing.assert_allclose(effort, np.clip(800*(control.physical_target-data.qpos)+data.qfrc_bias,-10,10))
        np.testing.assert_array_equal(data.qpos, old_qpos)
        for value in (-.01, .11, float('nan'), float('inf'), [0.,0.], '0.04'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                control.set_target(value)
        RESULTS['control_contract'] = dict(kp_N_m=800, native_kv_Ns_m=8, force_limit_N=10,
            slew_m_s=.03, command_period_s=.01, timestep_s=.001, no_integration=True)

    def test_catalog_real_entries_and_no_planned_placeholders(self):
        index = ROOT / 'assets/README.md'
        self.assertTrue(index.is_file(), 'Asset catalog not delivered')
        for entry in ('components/gripper/gripper.xml', 'components/gripper/README.md',
                      'assemblies/nero_gripper/README.md', 'scenes/r1_cube_grasp/README.md'):
            self.assertTrue((ROOT/'assets'/entry).is_file(), entry)
        text = index.read_text()
        for word in ('PLANNED', 'BLOCKED', 'left_', 'right_', 'flange_mount'):
            self.assertIn(word, text)
        for planned in ('components/nero', 'components/dh116', 'assemblies/nero_dh116', 'assemblies/dual_nero_gripper'):
            self.assertFalse((ROOT/'assets'/planned).exists(), planned)
        RESULTS['catalog'] = dict(actual_components=1, actual_assembly_indexes=1, actual_scene_indexes=1,
            planned_assets_created=0)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    outcome = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(StandaloneGripper))
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        with args.report.open('x') as f:
            json.dump(dict(passed=outcome.wasSuccessful(), tests_run=outcome.testsRun, results=RESULTS), f, indent=2)
            f.write('\n')
    raise SystemExit(0 if outcome.wasSuccessful() else 1)
