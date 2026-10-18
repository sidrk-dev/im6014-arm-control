from dataclasses import replace
import unittest

from build_profile import JointEntry, Measurement, parse_json
from build_profiles import (ARM_6_TOOL, BENCH_J1, BuildPlan, MechanicalSetup,
                            PROVISIONAL_DEFINITIONS, SIM_6_MEASURED, SIM_6_NOMINAL,
                            Workflow, bench_j1, provisional_build)
from joint_mapping import JointMap


def joints(count):
    return tuple(JointEntry(
        name=f'J{index + 1}', bus='synthetic-bus', motor_id=index,
        actuator_identity=None, actuator_identity_source='not physically verified',
        encoder='unmarked', mapping=JointMap(index + 1, -1, 0.25),
        mapping_source='synthetic design fixture', mapping_status='declared',
    ) for index in range(count))


class D03ProvisionalBuilds(unittest.TestCase):
    def test_all_later_stages_have_exact_ordered_joint_dimensions(self):
        self.assertEqual([item.build_id for item in PROVISIONAL_DEFINITIONS], [
            'arm_j1_j2', 'arm_j1_j2_j3', 'arm_j1_j2_j3_j4', 'arm_j1_j2_j3_j4_j5',
            'arm_j1_j2_j3_j4_j5_j6', 'arm_6_tool', 'sim_6_nominal', 'sim_6_measured',
        ])
        for definition in PROVISIONAL_DEFINITIONS:
            with self.subTest(build=definition.build_id):
                supplied = joints(len(definition.joint_names))
                plan = provisional_build(definition.build_id, revision='synthetic-rev', joints=supplied)
                self.assertIs(plan.definition, definition)
                self.assertEqual(plan.profile.joint_names(), definition.joint_names)
                self.assertEqual(plan.profile.backend, definition.backend)
                for original, configured in zip(supplied, plan.profile.joints):
                    self.assertEqual(original.mapping, configured.mapping)
                    self.assertEqual(original.motor_id, configured.motor_id)
                    self.assertIsNone(configured.actuator_identity)
                    self.assertEqual(configured.mapping_status, 'declared')
                    self.assertEqual(original.measurements, ())

    def test_missing_geometry_is_unknown_without_nominal_numeric_defaults(self):
        for definition in PROVISIONAL_DEFINITIONS:
            plan = provisional_build(definition.build_id, revision='synthetic-rev', joints=joints(len(definition.joint_names)))
            for joint in plan.profile.joints:
                for name, units in [('body_mass', 'kg'), ('link_length', 'm')]:
                    measurement = joint.measurement(name)
                    self.assertEqual(measurement.status, 'unknown')
                    self.assertEqual(measurement.units, units)
                    self.assertEqual(measurement.source, 'not measured for this build revision')
                    self.assertIsNone(measurement.value)
                    self.assertNotIn('value', measurement.to_dict())

    def test_roundtrip_does_not_upgrade_unknowns_including_measured_named_simulation(self):
        for definition in PROVISIONAL_DEFINITIONS:
            plan = provisional_build(definition.build_id, revision='synthetic-rev', joints=joints(len(definition.joint_names)))
            restored = parse_json(plan.profile.to_json())
            self.assertEqual(restored, plan.profile)
            self.assertEqual(BuildPlan(definition, restored), plan)
            self.assertTrue(all(value.status == 'unknown' for joint in restored.joints for value in joint.measurements))
            for obj in (plan, definition, restored):
                for name in ('calibrated', 'permits', 'allowed_actions'):
                    self.assertFalse(hasattr(obj, name))

    def test_explicit_declared_and_measured_values_keep_provenance(self):
        original = joints(2)
        values = (Measurement('body_mass', 'measured', 'kg', 'synthetic scale record', 0.5),
                  Measurement('link_length', 'declared', 'm', 'synthetic drawing', 0.35))
        supplied = (replace(original[0], measurements=values), original[1])
        plan = provisional_build('arm_j1_j2', revision='synthetic-rev', joints=supplied)
        self.assertEqual(plan.profile.joints[0].measurements, supplied[0].measurements)
        self.assertEqual(plan.profile.joints[1].measurement('body_mass').status, 'unknown')

    def test_simulation_and_physical_workflow_metadata_stay_distinct(self):
        for definition in PROVISIONAL_DEFINITIONS:
            if definition.backend == 'simulation':
                self.assertEqual(definition.mechanical_setup, MechanicalSetup.SIMULATED)
                self.assertEqual(definition.actions_when_commissioned, frozenset({Workflow.INSPECT, Workflow.SIMULATE}))
                self.assertNotIn(Workflow.ARM_JOINT_CONTROL, definition.actions_when_commissioned)
            else:
                self.assertEqual(definition.mechanical_setup, MechanicalSetup.MOUNTED_BASE)
                self.assertIn('link_geometry_verified', definition.required_evidence)
                self.assertNotIn(Workflow.SIMULATE, definition.actions_when_commissioned)
        self.assertIn('tool_and_camera_measured', ARM_6_TOOL.required_evidence)
        self.assertNotEqual(SIM_6_MEASURED.required_evidence, SIM_6_NOMINAL.required_evidence)

    def test_wrong_backends_names_and_dimensions_are_rejected(self):
        for definition in PROVISIONAL_DEFINITIONS:
            supplied = joints(len(definition.joint_names))
            plan = provisional_build(definition.build_id, revision='synthetic-rev', joints=supplied)
            for changed in (replace(plan.profile, backend='demo'), replace(plan.profile, robot='another-robot'), replace(plan.profile, build='bench_j1'),
                            replace(plan.profile, joints=plan.profile.joints[:-1]),
                            replace(plan.profile, joints=tuple(reversed(plan.profile.joints)))):
                with self.subTest(build=definition.build_id, changed=changed):
                    with self.assertRaises(ValueError):
                        BuildPlan(definition, changed)
            with self.assertRaises(ValueError):
                BuildPlan(replace(definition), plan.profile)

    def test_inputs_require_explicit_mapping_identity_revision_and_unique_bus_ids(self):
        for bad in ('bench_j1', 'unknown', ' arm_j1_j2 ', None):
            with self.assertRaises(ValueError):
                provisional_build(bad, revision='synthetic', joints=joints(2))
        for bad in (None, [], (None,)):
            with self.assertRaises(ValueError):
                provisional_build('arm_j1_j2', revision='synthetic', joints=bad)
        for missing in ('revision', 'joints'):
            arguments = dict(revision='synthetic', joints=joints(2))
            del arguments[missing]
            with self.assertRaises(TypeError):
                provisional_build('arm_j1_j2', **arguments)
        supplied = joints(2)
        with self.assertRaises(ValueError):
            provisional_build('arm_j1_j2', revision='synthetic', joints=(supplied[0], replace(supplied[1], motor_id=0)))
        with self.assertRaises(ValueError):
            provisional_build('arm_j1_j2', revision='', joints=supplied)

    def test_geometry_units_and_negative_known_values_are_rejected(self):
        supplied = joints(2)
        for name, units, value in [('body_mass', 'm', 1), ('link_length', 'kg', 1),
                                   ('body_mass', 'kg', -1), ('link_length', 'm', -1)]:
            with self.subTest(name=name, units=units, value=value):
                configured = replace(supplied[0], measurements=(Measurement(name, 'declared', units, 'synthetic', value),))
                with self.assertRaises(ValueError):
                    provisional_build('arm_j1_j2', revision='synthetic', joints=(configured, supplied[1]))

    def test_existing_j1_plan_also_rejects_simulation_backend(self):
        joint = joints(1)[0]
        plan = bench_j1(revision='synthetic', bus=joint.bus, motor_id=joint.motor_id,
                        mapping=joint.mapping, mapping_source=joint.mapping_source,
                        actuator_identity=None, actuator_identity_source=joint.actuator_identity_source)
        with self.assertRaises(ValueError):
            BuildPlan(BENCH_J1, replace(plan.profile, backend='simulation'))


if __name__ == '__main__':
    unittest.main()
