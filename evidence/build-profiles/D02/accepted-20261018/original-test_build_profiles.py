from copy import deepcopy
from dataclasses import FrozenInstanceError
import json
import unittest

import arm_manual
import build_profile
import build_profiles


def joint_dict(readiness=None):
    record = {
        "name": "J1",
        "bus": "bus0",
        "motor_id": 1,
        "encoder": "rotor_incremental",
        "mapping": {"reduction": 1, "direction": 1, "zero_output_rad": 0.0},
    }
    if readiness is not None:
        record["readiness"] = readiness
    return record


def profile_dict(mechanical_readiness="bench", joint_readiness=None):
    return {
        "schema_version": 1,
        "robot": "im6014-arm",
        "build": "bench_j1",
        "backend": "physical",
        "revision": "e74ab42",
        "mechanical_readiness": mechanical_readiness,
        "joints": [joint_dict(joint_readiness)],
    }


def parse(profile):
    return build_profile.parse_json(json.dumps(deepcopy(profile)))


class D02MechanicalReadiness(unittest.TestCase):
    def test_profiles_have_distinct_mechanical_readiness(self):
        self.assertEqual(build_profiles.bench_j1().mechanical_readiness, "bench")
        self.assertEqual(build_profiles.arm_j1().mechanical_readiness, "mounted")
        self.assertNotEqual(
            build_profiles.bench_j1().mechanical_readiness,
            build_profiles.arm_j1().mechanical_readiness,
        )

    def test_mechanical_readiness_is_required(self):
        profile = profile_dict()
        del profile["mechanical_readiness"]
        with self.assertRaises(ValueError):
            parse(profile)

    def test_unknown_mechanical_readiness_rejected(self):
        for value in ("", "table", "BENCH", 1, None):
            with self.subTest(value=repr(value)):
                with self.assertRaises(ValueError):
                    parse(profile_dict(mechanical_readiness=value))


class D02AllowedActions(unittest.TestCase):
    def test_allowed_actions_are_distinct(self):
        bench = build_profiles.bench_j1()
        mounted = build_profiles.arm_j1()
        self.assertNotEqual(bench.allowed_actions, mounted.allowed_actions)
        self.assertNotIn("arm", bench.allowed_actions)
        self.assertIn("arm", mounted.allowed_actions)

    def test_bench_actuator_can_never_be_armed(self):
        bench = build_profiles.bench_j1()
        self.assertFalse(bench.permits("arm"))
        self.assertFalse(bench.permits("release"))
        for action in ("check", "configure", "calibrate"):
            self.assertTrue(bench.permits(action))

    def test_mounted_base_permits_motion_actions(self):
        mounted = build_profiles.arm_j1()
        for action in ("check", "configure", "calibrate", "arm", "release"):
            self.assertTrue(mounted.permits(action))
        self.assertFalse(mounted.permits("disconnect"))
        self.assertFalse(mounted.permits("not_an_action"))
        self.assertFalse(mounted.permits(3))

    def test_allowed_actions_reuse_runtime_vocabulary(self):
        self.assertTrue(build_profile.PROFILE_ACTION_VOCABULARY <= arm_manual._ACTIONS)
        self.assertTrue(build_profiles.bench_j1().allowed_actions <= arm_manual._ACTIONS)
        self.assertTrue(build_profiles.arm_j1().allowed_actions <= arm_manual._ACTIONS)

    def test_action_sets_are_immutable_frozensets(self):
        for profile in (build_profiles.bench_j1(), build_profiles.arm_j1()):
            with self.subTest(build=profile.build_id):
                self.assertIsInstance(profile.allowed_actions, frozenset)


class D02IdentityAndSubstitution(unittest.TestCase):
    def test_build_ids_are_distinct(self):
        self.assertEqual(build_profiles.bench_j1().build_id, "bench_j1")
        self.assertEqual(build_profiles.arm_j1().build_id, "arm_j1")
        self.assertNotEqual(build_profiles.bench_j1().build_id, build_profiles.arm_j1().build_id)

    def test_profiles_are_not_substitutable(self):
        bench = build_profiles.bench_j1()
        mounted = build_profiles.arm_j1()
        self.assertFalse(bench.substitutable_with(mounted))
        self.assertFalse(mounted.substitutable_with(bench))
        self.assertTrue(bench.substitutable_with(build_profiles.bench_j1()))
        self.assertFalse(bench.substitutable_with("bench_j1"))

    def test_build_id_matches_serialized_build(self):
        for profile in (build_profiles.bench_j1(), build_profiles.arm_j1()):
            with self.subTest(build=profile.build_id):
                self.assertEqual(json.loads(profile.to_json())["build"], profile.build_id)


class D02JointReadiness(unittest.TestCase):
    def test_default_readiness_is_uncommissioned(self):
        entry = parse(profile_dict()).joints[0]
        self.assertEqual(entry.readiness, "uncommissioned")

    def test_absent_joint_readiness_is_absent_not_movable(self):
        entry = parse(profile_dict(joint_readiness="absent")).joints[0]
        self.assertEqual(entry.readiness, "absent")
        self.assertNotIn(entry.readiness, ("calibrated", "movable"))
        self.assertEqual(entry.to_dict()["readiness"], "absent")
        again = build_profile.parse_json(parse(profile_dict(joint_readiness="absent")).to_json())
        self.assertEqual(again.joints[0].readiness, "absent")

    def test_declared_readiness_levels_round_trip(self):
        for level in build_profile.READINESS_LEVELS:
            with self.subTest(level=level):
                profile = parse(profile_dict(joint_readiness=level))
                self.assertEqual(profile.joints[0].readiness, level)
                self.assertEqual(build_profile.parse_json(profile.to_json()), profile)

    def test_unknown_joint_readiness_rejected(self):
        for value in ("", "installed", "MOVABLE", 1, None):
            with self.subTest(value=repr(value)):
                profile = profile_dict()
                profile["joints"][0]["readiness"] = value
                with self.assertRaises(ValueError):
                    parse(profile)

    def test_build_profiles_declare_expected_joint_readiness(self):
        self.assertEqual(build_profiles.bench_j1().joints[0].readiness, "uncommissioned")
        self.assertEqual(build_profiles.arm_j1().joints[0].readiness, "calibrated")


class D02RoundTripAndImmutability(unittest.TestCase):
    def test_round_trip_is_deterministic(self):
        for profile in (build_profiles.bench_j1(), build_profiles.arm_j1()):
            with self.subTest(build=profile.build_id):
                first = profile.to_json()
                self.assertEqual(first, profile.to_json())
                again = build_profile.parse_json(first)
                self.assertEqual(again, profile)
                self.assertEqual(again.to_json(), first)
                self.assertEqual(again.allowed_actions, profile.allowed_actions)

    def test_repeated_builders_are_equal(self):
        self.assertEqual(build_profiles.bench_j1(), build_profiles.bench_j1())
        self.assertEqual(build_profiles.arm_j1(), build_profiles.arm_j1())

    def test_profiles_are_immutable(self):
        profile = build_profiles.arm_j1()
        with self.assertRaises(FrozenInstanceError):
            profile.mechanical_readiness = "bench"
        with self.assertRaises(FrozenInstanceError):
            profile.joints[0].readiness = "absent"

    def test_mechanical_readiness_survives_serialization(self):
        record = json.loads(build_profiles.arm_j1().to_json())
        self.assertEqual(record["mechanical_readiness"], "mounted")


class D02DoesNotRegressD01(unittest.TestCase):
    def test_unknown_joint_fields_still_rejected(self):
        profile = profile_dict()
        profile["joints"][0]["vendor_note"] = "x"
        with self.assertRaises(ValueError):
            parse(profile)

    def test_invalid_reduction_and_duplicate_slots_still_rejected(self):
        invalid = profile_dict()
        invalid["joints"][0]["mapping"]["reduction"] = 0
        with self.assertRaises(ValueError):
            parse(invalid)
        duplicate = profile_dict()
        second = joint_dict()
        second["name"] = "J1"
        duplicate["joints"] = [joint_dict(), second]
        with self.assertRaises(ValueError):
            parse(duplicate)

    def test_missing_required_joint_mapping_still_rejected(self):
        profile = profile_dict()
        del profile["joints"][0]["mapping"]
        with self.assertRaises(ValueError):
            parse(profile)


if __name__ == "__main__":
    unittest.main()
