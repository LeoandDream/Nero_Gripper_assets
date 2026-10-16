"""Guard declared scalar-joint metadata without importing MuJoCo."""
import copy
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def assert_contract_matches_mjcf(checks, contract, model):
    dimension = contract["physical_state_dim"]
    order = contract["physical_joint_order"]
    units = contract["units"]
    checks.assertEqual(
        dimension, len(order), "physical_state_dim must match physical_joint_order length"
    )
    checks.assertEqual(dimension, len(units), "physical_state_dim must match units length")

    # Only physical joints count: equality/joint and default/joint are metadata.
    joints = model.findall("./worldbody//joint")
    names = [joint.get("name") for joint in joints]
    checks.assertEqual(order, names, "physical_joint_order must match worldbody joint order")
    checks.assertNotIn("gripper", names, "virtual source joint must not be a physical qpos")
    checks.assertFalse(
        model.findall("./worldbody//freejoint"),
        "freejoint is outside the scalar static contract",
    )
    unit_by_type = {"hinge": "rad", "slide": "m"}
    joint_units = []
    for joint in joints:
        kind = joint.get("type", "hinge")
        checks.assertIn(kind, unit_by_type, "only hinge/slide joints have scalar units")
        joint_units.append(unit_by_type[kind])
    checks.assertEqual(units, joint_units, "units must match MJCF joint types")
    checks.assertEqual(
        contract["actuators"],
        len(model.findall("./actuator/*")),
        "actuators must match MJCF actuator entries",
    )


class ContractMetadataTests(unittest.TestCase):
    def setUp(self):
        self.contract = json.loads(
            (ROOT / "docs/robot-contract.json").read_text(encoding="utf-8")
        )
        self.model = ET.parse(ROOT / "mjcf/nero_gripper.xml").getroot()

    def check_metadata(self):
        assert_contract_matches_mjcf(self, self.contract, self.model)

    def test_delivered_contract_matches_mjcf(self):
        self.check_metadata()

    def test_rejects_dimension_drift(self):
        for field in ("physical_state_dim", "units"):
            with self.subTest(field=field):
                contract = copy.deepcopy(self.contract)
                if field == "physical_state_dim":
                    contract[field] += 1
                else:
                    contract[field].pop()
                with self.assertRaisesRegex(AssertionError, field):
                    assert_contract_matches_mjcf(self, contract, self.model)

    def test_rejects_joint_order_drift(self):
        order = self.contract["physical_joint_order"]
        order[0], order[1] = order[1], order[0]
        with self.assertRaisesRegex(AssertionError, "worldbody joint order"):
            self.check_metadata()

    def test_rejects_joint_unit_drift(self):
        for index, wrong_unit in ((0, "m"), (-1, "rad")):
            with self.subTest(index=index):
                contract = copy.deepcopy(self.contract)
                contract["units"][index] = wrong_unit
                with self.assertRaisesRegex(AssertionError, "MJCF joint types"):
                    assert_contract_matches_mjcf(self, contract, self.model)

    def test_rejects_actuator_count_drift(self):
        self.contract["actuators"] = 1
        with self.assertRaisesRegex(AssertionError, "MJCF actuator entries"):
            self.check_metadata()

    def test_counts_actuator_entries_not_containers(self):
        actuators = ET.SubElement(self.model, "actuator")
        self.check_metadata()  # An empty container contributes zero entries.
        ET.SubElement(actuators, "motor", joint="joint1")
        ET.SubElement(actuators, "position", joint="joint2")
        self.contract["actuators"] = 2
        self.check_metadata()

    def test_rejects_virtual_source_joint_even_if_metadata_agrees(self):
        self.contract["physical_joint_order"].append("gripper")
        self.contract["units"].append("m")
        self.contract["physical_state_dim"] += 1
        body = self.model.find("./worldbody/body")
        ET.SubElement(body, "joint", name="gripper", type="slide")
        with self.assertRaisesRegex(AssertionError, "virtual source joint"):
            self.check_metadata()

    def test_rejects_non_scalar_joints(self):
        for kind in ("ball", "freejoint"):
            with self.subTest(kind=kind):
                model = copy.deepcopy(self.model)
                if kind == "ball":
                    model.find("./worldbody//joint").set("type", "ball")
                else:
                    ET.SubElement(model.find("./worldbody/body"), "freejoint")
                with self.assertRaisesRegex(AssertionError, "scalar"):
                    assert_contract_matches_mjcf(self, self.contract, model)


if __name__ == "__main__":
    unittest.main()
