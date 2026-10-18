from dataclasses import FrozenInstanceError, replace
import json
import unittest

from build_profile import LockedJoint, Measurement, from_dict, parse_json
from build_profiles import BuildPlan, arm_j1
from joint_controller import JointCore
from joint_mapping import JointTarget
from joint_records import DecisionCode, JointSpec, PositionLimits, Target
from test_build_profiles import arguments
from test_controller import DeterministicClock, FakeStation, SYNTHETIC_PROTECTION


def locked(name='J2', *, known=True):
    return LockedJoint(name,
        Measurement('body_mass', 'measured' if known else 'unknown', 'kg',
                    'synthetic aggregate mass fixture', 0.535 if known else None),
        Measurement('locked_angle', 'declared' if known else 'unknown', 'rad',
                    'synthetic lock-angle fixture', 0.5 if known else None))


def profile():
    return arm_j1(**arguments()).profile


class D04LockedJoints(unittest.TestCase):
    def test_locked_mass_and_position_survive_roundtrip_outside_active_coordinates(self):
        active = profile()
        configured = replace(active, locked_joints=(locked(), locked('J3', known=False)))
        restored = parse_json(configured.to_json())
        self.assertEqual(restored, configured)
        self.assertEqual(restored.joint_names(), ('J1',))
        self.assertEqual(restored.locked_joints[0].mass.value, 0.535)
        self.assertEqual(restored.locked_joints[0].position.value, 0.5)
        self.assertEqual(restored.locked_joints[0].mass.source, 'synthetic aggregate mass fixture')
        self.assertEqual(restored.locked_joints[1].mass.status, 'unknown')
        self.assertIsNone(restored.locked_joints[1].mass.value)
        self.assertNotIn('value', restored.to_dict()['locked_joints'][1]['mass'])
        self.assertEqual(configured.joints, active.joints)
        self.assertEqual(BuildPlan(arm_j1(**arguments()).definition, configured).profile, configured)

    def test_named_config_lookup_rejects_locked_and_absent_joints(self):
        configured = replace(profile(), locked_joints=(locked(),))
        self.assertIs(configured.require_active_joint('J1'), configured.joints[0])
        for name in ('J2', 'J3', ' J1 ', '', None):
            with self.subTest(name=name), self.assertRaises(ValueError):
                configured.require_active_joint(name)
        self.assertFalse(hasattr(configured.locked_joints[0], 'motor_id'))
        self.assertFalse(hasattr(configured.locked_joints[0], 'mapping'))

    def test_no_placeholder_mass_or_extra_coordinate_is_created_for_absence(self):
        configured = profile()
        self.assertEqual(configured.locked_joints, ())
        self.assertNotIn('locked_joints', configured.to_dict())
        self.assertEqual(parse_json(configured.to_json()), configured)
        self.assertEqual(configured.joint_names(), ('J1',))

    def test_duplicate_and_active_locked_overlapping_names_are_rejected(self):
        for values in ((locked('J1'),), (locked(), locked()), [locked()], (None,)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                replace(profile(), locked_joints=values)
        with self.assertRaises(ValueError):
            locked(' padded ')
        with self.assertRaises(FrozenInstanceError):
            locked().name = 'J4'

    def test_locked_measurement_names_units_and_values_are_checked(self):
        for mass in (None, Measurement('mass', 'measured', 'kg', 'synthetic', 1),
                     Measurement('body_mass', 'measured', 'g', 'synthetic', 535),
                     Measurement('body_mass', 'measured', 'kg', 'synthetic', -1)):
            with self.subTest(mass=mass), self.assertRaises(ValueError):
                LockedJoint('J2', mass, locked().position)
        for position in (None, Measurement('locked_angle', 'declared', 'degrees', 'synthetic', 45),
                         Measurement('another_angle', 'declared', 'rad', 'synthetic', 1)):
            with self.subTest(position=position), self.assertRaises(ValueError):
                LockedJoint('J2', locked().mass, position)

    def test_json_boundary_rejects_unknown_fields_nulls_missing_and_motor_mappings(self):
        original = replace(profile(), locked_joints=(locked(),)).to_dict()
        for value in (None, {}, [None], [{}]):
            changed = {**original, 'locked_joints':value}
            with self.subTest(value=value), self.assertRaises(ValueError):
                from_dict(changed)
        for field in ('name', 'mass', 'position'):
            record = dict(original['locked_joints'][0]);del record[field]
            with self.subTest(field=field), self.assertRaises(ValueError):
                from_dict({**original, 'locked_joints':[record]})
        for extra in ('motor_id', 'bus', 'mapping'):
            record = {**original['locked_joints'][0], extra:0}
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                from_dict({**original, 'locked_joints':[record]})
        text = json.dumps(original).replace('"name": "J2"', '"name": "J2", "name": "J3"')
        with self.assertRaises(ValueError):
            parse_json(text)

    def test_old_version_one_json_remains_byte_compatible_when_no_locked_joints(self):
        configured = profile()
        self.assertEqual(configured.to_dict().keys(), {'schema_version', 'robot', 'build', 'backend', 'revision', 'joints'})
        self.assertEqual(from_dict(configured.to_dict()).to_json(), configured.to_json())
        self.assertEqual(from_dict({**configured.to_dict(), 'locked_joints':[]}), configured)

    def test_fake_core_has_only_active_dimensions_and_rejects_extra_targets_without_issue(self):
        configured = replace(profile(), locked_joints=(locked(),))
        clock = DeterministicClock(0)
        specs = tuple(JointSpec(joint.name, joint.motor_id, joint.mapping, PositionLimits(-10, 10),
                    100, 100, 100, 100, protection=SYNTHETIC_PROTECTION) for joint in configured.joints)
        core = JointCore(specs, clock=clock, max_reference_step_s=1)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner='synthetic').accepted)
        writes = tuple(station.writes)
        value = JointTarget(q_rad=0.1, dq_rad_s=0, tau_nm=0, kp=0, kd=0)
        target = Target(1, (value, value), created_at_s=0, expires_at_s=1, ownership=core.snapshot().ownership)
        self.assertEqual(station.submit(target).code, DecisionCode.INVALID_ARGUMENT)
        self.assertIsNone(station.consume())
        self.assertEqual(tuple(station.writes), writes)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(tuple(spec.name for spec in core.joint_specs), ('J1',))
        with self.assertRaises(ValueError):
            JointCore((locked(),), clock=clock, max_reference_step_s=1)


if __name__ == '__main__': unittest.main()
