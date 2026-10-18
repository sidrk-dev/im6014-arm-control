from copy import deepcopy
from dataclasses import FrozenInstanceError
import json
import math
import unittest

import build_profile
from joint_mapping import JointMap
from joint_records import PositionLimits


def joint_dict():
    return {
        "name": "j1",
        "bus": "bus0",
        "motor_id": 1,
        "encoder": "rotor_incremental",
        "mapping": {"reduction": 6, "direction": 1, "zero_output_rad": 0.25},
    }


def profile_dict():
    return {
        "schema_version": 1,
        "robot": "im6014-arm",
        "build": "bench_j1",
        "backend": "physical",
        "revision": "71a0a5f",
        "joints": [joint_dict()],
    }


def parse(profile=None):
    if profile is not None:
        profile = deepcopy(profile)
    else:
        profile = profile_dict()
    return build_profile.parse_json(json.dumps(profile))


class D01BuildProfile(unittest.TestCase):
    def test_valid_minimal_profile(self):
        profile = parse()
        self.assertEqual(profile.schema_version, 1)
        self.assertEqual(
            (profile.robot, profile.build, profile.backend, profile.revision),
            ("im6014-arm", "bench_j1", "physical", "71a0a5f"),
        )
        self.assertEqual(profile.joint_names(), ("j1",))
        entry = profile.joints[0]
        self.assertEqual((entry.name, entry.bus, entry.motor_id), ("j1", "bus0", 1))
        self.assertEqual(entry.encoder, "rotor_incremental")
        self.assertEqual(entry.mapping, JointMap(6, 1, 0.25))
        self.assertIsNone(entry.position_limits)
        self.assertEqual(entry.measurements, ())

    def test_position_limits_reuse_existing_validator(self):
        profile = profile_dict()
        profile["joints"][0]["position_limits"] = {"minimum_rad": -3.0, "maximum_rad": 3.0}
        entry = parse(profile).joints[0]
        self.assertEqual(entry.position_limits, PositionLimits(-3.0, 3.0))
        profile["joints"][0]["position_limits"] = {"minimum_rad": 3.0, "maximum_rad": -3.0}
        with self.assertRaises(ValueError):
            parse(profile)

    def test_unknown_schema_version_rejected(self):
        for version in (0, 2, "1", 1.0, True):
            with self.subTest(version=repr(version)):
                profile = profile_dict()
                profile["schema_version"] = version
                with self.assertRaises(ValueError):
                    parse(profile)

    def test_unknown_fields_rejected(self):
        top = profile_dict()
        top["extra"] = 1
        with self.assertRaises(ValueError):
            parse(top)
        joint = profile_dict()
        joint["joints"][0]["vendor_note"] = "x"
        with self.assertRaises(ValueError):
            parse(joint)
        mapping = profile_dict()
        mapping["joints"][0]["mapping"]["gear_ratio"] = 6.0
        with self.assertRaises(ValueError):
            parse(mapping)
        limits = profile_dict()
        limits["joints"][0]["position_limits"] = {"minimum_rad": 0.0, "maximum_rad": 1.0, "soft_stop": 0.5}
        with self.assertRaises(ValueError):
            parse(limits)
        measurements = profile_dict()
        measurements["joints"][0]["measurements"] = {
            "thermal": {"status": "measured", "value": 1, "unit": "j_c"}
        }
        with self.assertRaises(ValueError):
            parse(measurements)

    def test_duplicate_joint_names_rejected(self):
        profile = profile_dict()
        second = joint_dict()
        second["name"] = "j1"
        second["motor_id"] = 2
        profile["joints"] = [joint_dict(), second]
        with self.assertRaises(ValueError):
            parse(profile)

    def test_duplicate_motor_id_on_one_bus_rejected_allowed_on_distinct_buses(self):
        profile = profile_dict()
        second = joint_dict()
        second["name"] = "j2"
        second["bus"] = "bus0"
        second["motor_id"] = 1
        profile["joints"] = [joint_dict(), second]
        with self.assertRaises(ValueError):
            parse(profile)
        allowed = deepcopy(profile)
        allowed["joints"][1]["bus"] = "bus1"
        parsed = parse(allowed)
        self.assertEqual(parsed.joint_names(), ("j1", "j2"))
        slots = [(joint.bus, joint.motor_id) for joint in parsed.joints]
        self.assertEqual(slots, [("bus0", 1), ("bus1", 1)])

    def test_non_finite_and_malformed_numbers_rejected(self):
        for token in ("NaN", "Infinity", "-Infinity", "1e400"):
            with self.subTest(token=token):
                text = json.dumps(profile_dict()).replace('"reduction": 6', '"reduction": ' + token)
                with self.assertRaises(ValueError):
                    build_profile.parse_json(text)
                text = json.dumps(profile_dict()).replace(
                    '"zero_output_rad": 0.25', '"zero_output_rad": ' + token
                )
                with self.assertRaises(ValueError):
                    build_profile.parse_json(text)
        malformed = json.dumps(profile_dict()).replace('"reduction": 6', '"reduction": 1.2.3')
        with self.assertRaises(ValueError):
            build_profile.parse_json(malformed)
        trailing = json.dumps(profile_dict()) + "}"
        with self.assertRaises(ValueError):
            build_profile.parse_json(trailing)

    def test_missing_required_fields_rejected(self):
        profile = profile_dict()
        del profile["build"]
        with self.assertRaises(ValueError):
            parse(profile)
        joint = profile_dict()
        del joint["joints"][0]["mapping"]
        with self.assertRaises(ValueError):
            parse(joint)
        mapping = profile_dict()
        del mapping["joints"][0]["mapping"]["zero_output_rad"]
        with self.assertRaises(ValueError):
            parse(mapping)
        measurement = profile_dict()
        measurement["joints"][0]["measurements"] = {"thermal": {"status": "measured"}}
        with self.assertRaises(ValueError):
            parse(measurement)
        with self.assertRaises(ValueError):
            build_profile.parse_json(json.dumps([1, 2]))

    def test_invalid_mapping_and_identity_values_rejected(self):
        for direction in (0, 2, 1.0, True, "up"):
            with self.subTest(direction=direction):
                profile = profile_dict()
                profile["joints"][0]["mapping"]["direction"] = direction
                with self.assertRaises(ValueError):
                    parse(profile)
        for reduction in (0, -6):
            with self.subTest(reduction=reduction):
                profile = profile_dict()
                profile["joints"][0]["mapping"]["reduction"] = reduction
                with self.assertRaises(ValueError):
                    parse(profile)
        for encoder in ("", "n1", "unknown_convention", 3):
            with self.subTest(encoder=encoder):
                profile = profile_dict()
                profile["joints"][0]["encoder"] = encoder
                with self.assertRaises(ValueError):
                    parse(profile)
        for motor_id in (-1, 15, 1.0, True, "1"):
            with self.subTest(motor_id=motor_id):
                profile = profile_dict()
                profile["joints"][0]["motor_id"] = motor_id
                with self.assertRaises(ValueError):
                    parse(profile)
        for field in ("robot", "build", "backend", "revision"):
            with self.subTest(field=field):
                profile = profile_dict()
                profile[field] = "   "
                with self.assertRaises(ValueError):
                    parse(profile)

    def test_unknown_measurement_preserved_as_unknown_not_defaulted(self):
        profile = profile_dict()
        profile["joints"][0]["measurements"] = {"thermal_mass_j_c": {"status": "unknown"}}
        entry = parse(profile).joints[0]
        unknown = entry.measurement("thermal_mass_j_c")
        self.assertIsNotNone(unknown)
        self.assertEqual(unknown.status, "unknown")
        self.assertIsNone(unknown.value)
        self.assertEqual(entry.to_dict()["measurements"], {"thermal_mass_j_c": {"status": "unknown"}})

        measured = profile_dict()
        measured["joints"][0]["measurements"] = {"zero_offset_rad": {"status": "measured", "value": 0.125}}
        entry = parse(measured).joints[0]
        self.assertEqual(entry.measurement("zero_offset_rad").value, 0.125)

        smuggled = profile_dict()
        smuggled["joints"][0]["measurements"] = {"zero_offset_rad": {"status": "unknown", "value": 0.0}}
        with self.assertRaises(ValueError):
            parse(smuggled)

        profile = profile_dict()
        profile["joints"][0]["measurements"] = {"zero_offset_rad": {"status": "measured", "value": float("nan")}}
        with self.assertRaises(ValueError):
            build_profile.from_dict(profile)
        profile["joints"][0]["measurements"] = {"zero_offset_rad": {"status": "measured", "value": math.inf}}
        with self.assertRaises(ValueError):
            build_profile.from_dict(profile)

    def test_round_trip_deterministic_serialization(self):
        profile = profile_dict()
        profile["joints"][0]["position_limits"] = {"minimum_rad": -3.0, "maximum_rad": 3.0}
        profile["joints"][0]["measurements"] = {
            "a_offset": {"status": "unknown"},
            "b_offset": {"status": "measured", "value": 0.5},
        }
        parsed = parse(profile)
        first = parsed.to_json()
        second = parsed.to_json()
        self.assertEqual(first, second)
        again = build_profile.parse_json(first)
        self.assertEqual(again, parsed)
        self.assertEqual(again.to_json(), first)
        self.assertIsInstance(again.joints[0].mapping.reduction, float)
        self.assertEqual(json.loads(first)["joints"][0]["measurements"]["a_offset"], {"status": "unknown"})
        self.assertEqual(
            json.loads(first)["joints"][0]["position_limits"],
            {"minimum_rad": -3.0, "maximum_rad": 3.0},
        )

    def test_result_is_immutable(self):
        profile = parse()
        entry = profile.joints[0]
        measurement = build_profile.Measurement("thermal", "measured", 1.0)
        cases = [
            (profile, "build", "other"),
            (entry, "motor_id", 9),
            (entry, "measurements", ()),
            (entry.mapping, "reduction", 1.0),
            (measurement, "value", 2.0),
            (measurement, "status", "unknown"),
        ]
        for value, field, replacement in cases:
            with self.subTest(type=type(value).__name__, field=field), self.assertRaises(FrozenInstanceError):
                setattr(value, field, replacement)

    def test_bench_and_mounted_builds_are_distinct(self):
        bench = parse()
        mounted = profile_dict()
        mounted["build"] = "arm_j1"
        mounted = parse(mounted)
        self.assertNotEqual(bench, mounted)
        self.assertNotEqual(bench.to_json(), mounted.to_json())
        self.assertNotEqual(bench.joint_names(), ())
        self.assertEqual(bench.joint_names(), mounted.joint_names())
        self.assertEqual(bench.joints, mounted.joints)

    def test_passive_build_with_no_joints_round_trips(self):
        profile = profile_dict()
        profile["joints"] = []
        parsed = parse(profile)
        self.assertEqual(parsed.joint_names(), ())
        again = build_profile.parse_json(parsed.to_json())
        self.assertEqual(again, parsed)

    def test_measurements_are_sorted_deterministically(self):
        profile = profile_dict()
        profile["joints"][0]["measurements"] = {
            "zero_offset_rad": {"status": "measured", "value": 0.5},
            "arm_mass_kg": {"status": "unknown"},
        }
        entry = parse(profile).joints[0]
        self.assertEqual(
            tuple(measurement.name for measurement in entry.measurements),
            ("arm_mass_kg", "zero_offset_rad"),
        )


if __name__ == "__main__":
    unittest.main()
