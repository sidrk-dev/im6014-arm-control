from dataclasses import FrozenInstanceError, replace
import unittest

from build_profile import Measurement, parse_json
from calibration_binding import CalibrationBinding, bind_calibration
from joint_mapping import JointMap
from test_build_profiles import arguments
from build_profiles import arm_j1
from test_locked_joints import locked

ARTIFACT = b'{"synthetic_calibration_fixture":1}'


def profile():
    return replace(arm_j1(**arguments()).profile, locked_joints=(locked(),))


class D05CalibrationBinding(unittest.TestCase):
    def test_exact_profile_and_opaque_artifact_match(self):
        configured = profile()
        binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic-cal-1', source='synthetic fixture')
        self.assertIsNone(binding.require_match(configured, ARTIFACT))
        self.assertIsNone(binding.require_match(parse_json(configured.to_json()), bytes(ARTIFACT)))
        self.assertEqual(binding.calibration_id, 'synthetic-cal-1')
        self.assertEqual(binding.source, 'synthetic fixture')

    def test_another_revision_build_robot_or_backend_is_rejected(self):
        configured = profile()
        binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic', source='synthetic')
        for changes in ({'revision':'other'}, {'build':'other'}, {'robot':'other'}, {'backend':'simulation'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                binding.require_match(replace(configured, **changes), ARTIFACT)

    def test_actuator_bus_id_mapping_and_provenance_changes_reject(self):
        configured = profile();joint = configured.joints[0]
        binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic', source='synthetic')
        for changes in ({'actuator_identity':'another-tag'}, {'bus':'another-bus'}, {'motor_id':1},
                        {'mapping':JointMap(3, 1, 0)}, {'mapping_source':'another-source'},
                        {'mapping_status':'measured'}, {'encoder':'output_absolute'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                binding.require_match(replace(configured, joints=(replace(joint, **changes),)), ARTIFACT)

    def test_locked_mass_angle_and_active_measurement_changes_reject(self):
        configured = profile();binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic', source='synthetic')
        for changed in (replace(configured, locked_joints=()),
                        replace(configured, locked_joints=(replace(locked(), mass=Measurement('body_mass','measured','kg','synthetic',1)),)),
                        replace(configured, locked_joints=(replace(locked(), position=Measurement('locked_angle','declared','rad','synthetic',1)),)),
                        replace(configured, joints=(replace(configured.joints[0], measurements=(Measurement('body_mass','unknown','kg','not weighed'),)),))):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                binding.require_match(changed, ARTIFACT)

    def test_changed_artifact_is_rejected_even_if_build_matches(self):
        binding = bind_calibration(profile(), ARTIFACT, calibration_id='synthetic', source='synthetic')
        with self.assertRaises(ValueError):
            binding.require_match(profile(), ARTIFACT + b' ')

    def test_strict_input_and_digest_boundaries(self):
        for artifact in (b'', '', None, bytearray(ARTIFACT)):
            with self.subTest(artifact=artifact), self.assertRaises(ValueError):
                bind_calibration(profile(), artifact, calibration_id='synthetic', source='synthetic')
        for name in ('calibration_id','source'):
            for bad in ('', ' padded ', None):
                values=dict(calibration_id='synthetic',source='synthetic');values[name]=bad
                with self.subTest(name=name,bad=bad), self.assertRaises(ValueError):
                    bind_calibration(profile(), ARTIFACT, **values)
        for digest in ('a'*63, 'A'*64, 'x'*64, None):
            with self.subTest(digest=digest), self.assertRaises(ValueError):
                CalibrationBinding('synthetic','synthetic',digest,'a'*64)
        with self.assertRaises(ValueError):
            bind_calibration(None, ARTIFACT, calibration_id='synthetic', source='synthetic')

    def test_binding_is_immutable_metadata_without_live_application(self):
        configured = profile();binding = bind_calibration(configured, ARTIFACT, calibration_id='synthetic', source='synthetic')
        with self.assertRaises(FrozenInstanceError):
            binding.profile_sha256 = 'a'*64
        for name in ('apply','restore','arm','enable','permits','calibrated','allowed_actions'):
            self.assertFalse(hasattr(binding,name))
        self.assertIsNone(configured.joints[0].actuator_identity)
        self.assertEqual(configured.joints[0].mapping_status,'declared')


if __name__ == '__main__': unittest.main()
