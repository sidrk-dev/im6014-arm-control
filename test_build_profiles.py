from dataclasses import FrozenInstanceError, replace
import unittest

from build_profile import parse_json
from build_profiles import (ARM_J1, BENCH_J1, BuildPlan, MechanicalSetup,
                            Workflow, arm_j1, bench_j1)
from joint_mapping import JointMap


def arguments():
    return dict(revision='synthetic-1', bus='synthetic-bus', motor_id=0,
                mapping=JointMap(6, -1, 0.25), mapping_source='synthetic design',
                actuator_identity=None, actuator_identity_source='not measured')


class D02BuildWorkflows(unittest.TestCase):
    def test_profiles_have_distinct_identity_and_mechanical_requirements(self):
        bench, mounted = bench_j1(**arguments()), arm_j1(**arguments())
        self.assertEqual(bench.profile.build, 'bench_j1')
        self.assertEqual(mounted.profile.build, 'arm_j1')
        self.assertEqual(bench.definition.mechanical_setup, MechanicalSetup.SECURED_BENCH)
        self.assertEqual(mounted.definition.mechanical_setup, MechanicalSetup.MOUNTED_BASE)
        self.assertIn('bench_secured', bench.definition.required_evidence)
        self.assertNotIn('bench_secured', mounted.definition.required_evidence)
        self.assertIn('attached_load_verified', mounted.definition.required_evidence)
        self.assertNotIn('attached_load_verified', bench.definition.required_evidence)

    def test_conditional_workflow_scopes_stay_distinct(self):
        self.assertIn(Workflow.BOUNDED_BENCH_TEST, BENCH_J1.actions_when_commissioned)
        self.assertNotIn(Workflow.ARM_JOINT_CONTROL, BENCH_J1.actions_when_commissioned)
        self.assertIn(Workflow.ARM_JOINT_CONTROL, ARM_J1.actions_when_commissioned)
        self.assertNotIn(Workflow.BOUNDED_BENCH_TEST, ARM_J1.actions_when_commissioned)
        for definition in (BENCH_J1, ARM_J1):
            self.assertIn('current_session_reference', definition.required_evidence)
            self.assertNotIn('arm', {action.value for action in definition.actions_when_commissioned})
            self.assertNotIn('release', {action.value for action in definition.actions_when_commissioned})

    def test_factories_never_invent_mapping_or_calibration(self):
        for factory in (bench_j1, arm_j1):
            with self.subTest(factory=factory.__name__):
                for field in arguments():
                    values = arguments()
                    del values[field]
                    with self.assertRaises(TypeError):
                        factory(**values)
                plan = factory(**arguments())
                joint = plan.profile.joints[0]
                self.assertEqual(joint.mapping, arguments()['mapping'])
                self.assertEqual(joint.mapping_status, 'declared')
                self.assertIsNone(joint.actuator_identity)
                self.assertEqual(joint.motor_id, 0)
                for obj in (plan, plan.definition, plan.profile, joint):
                    self.assertFalse(hasattr(obj, 'permits'))
                    self.assertFalse(hasattr(obj, 'calibrated'))
                    self.assertFalse(hasattr(obj, 'allowed_actions'))

    def test_explicit_configurations_round_trip_without_readiness_upgrade(self):
        for factory in (bench_j1, arm_j1):
            plan = factory(**arguments())
            again = parse_json(plan.profile.to_json())
            self.assertEqual(again, plan.profile)
            self.assertEqual(BuildPlan(plan.definition, again), plan)
            self.assertEqual(again.joints[0].mapping_status, 'declared')
            self.assertNotEqual(plan.definition.required_evidence, frozenset())

    def test_mismatched_or_modified_definitions_are_rejected(self):
        bench, mounted = bench_j1(**arguments()), arm_j1(**arguments())
        with self.assertRaises(ValueError):
            BuildPlan(BENCH_J1, mounted.profile)
        with self.assertRaises(ValueError):
            BuildPlan(ARM_J1, bench.profile)
        with self.assertRaises(ValueError):
            BuildPlan(replace(BENCH_J1, required_evidence=frozenset()), bench.profile)
        with self.assertRaises(ValueError):
            BuildPlan(replace(BENCH_J1, required_evidence=set(BENCH_J1.required_evidence)), bench.profile)
        with self.assertRaises(ValueError):
            BuildPlan(BENCH_J1, replace(bench.profile, joints=()))
        with self.assertRaises(ValueError):
            BuildPlan(BENCH_J1, 'bench_j1')

    def test_values_and_scopes_are_immutable(self):
        plan = arm_j1(**arguments())
        with self.assertRaises(FrozenInstanceError):
            plan.profile = bench_j1(**arguments()).profile
        with self.assertRaises(FrozenInstanceError):
            plan.definition.mechanical_setup = MechanicalSetup.SECURED_BENCH
        with self.assertRaises(AttributeError):
            plan.definition.actions_when_commissioned.add(Workflow.BOUNDED_BENCH_TEST)

    def test_identity_and_mapping_validation_stays_at_d01_boundary(self):
        for factory in (bench_j1, arm_j1):
            for changes in ({'bus':' padded '}, {'motor_id':True}, {'mapping':None},
                            {'revision':''}, {'mapping_source':''}, {'actuator_identity_source':''}):
                with self.subTest(factory=factory.__name__, changes=changes):
                    values = arguments()
                    values.update(changes)
                    with self.assertRaises(ValueError):
                        factory(**values)


if __name__=='__main__':
    unittest.main()
