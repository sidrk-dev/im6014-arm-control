from dataclasses import FrozenInstanceError, replace
import unittest

from build_profiles import arm_j1
from joint_mapping import JointMap
from test_build_profiles import arguments
from validation_records import (
    ConfigurationArtifact, ValidationContext, ValidationDependency as Dependency,
    ValidationRecord, ValidationStatus as Status,
)


PROFILE = frozenset({Dependency.PROFILE})
ALL = frozenset(Dependency)


def context():
    return ValidationContext(
        arm_j1(**arguments()).profile,
        ConfigurationArtifact(b'{"tool":"synthetic scoop","mass_kg":0.1}', 'synthetic tool fixture'),
        ConfigurationArtifact(b'{"parent":"base","child":"arm","translation_m":[0,0,0]}',
                              'synthetic mounting fixture'),
    )


def result(inputs, dependencies=ALL, status=Status.PASSED):
    return ValidationRecord('synthetic-check', dependencies).report_result(
        inputs, status=status, evidence='synthetic result reference')


class D06ValidationInvalidation(unittest.TestCase):
    def test_new_checks_start_pending(self):
        record = ValidationRecord('synthetic-check', ALL)
        self.assertIs(record.status, Status.PENDING)
        self.assertIsNone(record.evidence)
        self.assertEqual(record.input_hashes, ())

    def test_unchanged_configuration_retains_reported_outcome_and_evidence(self):
        original = context()
        for status in (Status.PASSED, Status.FAILED):
            with self.subTest(status=status):
                record = result(original, status=status)
                updated = record.invalidate_changed(context())
                self.assertEqual(updated.status, status)
                self.assertEqual(updated.evidence, 'synthetic result reference')
                self.assertEqual(updated, record)

    def test_reduction_change_marks_pending_and_clears_old_result(self):
        original = context()
        joint = original.profile.joints[0]
        changed_joint = replace(joint, mapping=replace(joint.mapping, reduction=joint.mapping.reduction * 2))
        changed = replace(original, profile=replace(original.profile, joints=(changed_joint,)))
        record = result(original)
        pending = record.invalidate_changed(changed)
        self.assertIs(pending.status, Status.PENDING)
        self.assertIsNone(pending.evidence)
        self.assertEqual(pending.input_hashes, ())
        self.assertIs(record.status, Status.PASSED)

    def test_tool_and_mounting_content_or_source_changes_invalidate_selected_checks(self):
        original = context()
        for name, artifact in (
            ('tool', ConfigurationArtifact(b'{"tool":"synthetic gripper"}', original.tool.source)),
            ('tool', replace(original.tool, source='another synthetic tool source')),
            ('mounting_transform', ConfigurationArtifact(
                b'{"parent":"base","child":"arm","translation_m":[0.1,0,0]}',
                original.mounting_transform.source)),
            ('mounting_transform', replace(original.mounting_transform, source='another synthetic mount source')),
        ):
            with self.subTest(name=name, artifact=artifact):
                self.assertIs(result(original).invalidate_changed(
                    replace(original, **{name:artifact})).status, Status.PENDING)

    def test_undeclared_tool_or_mount_changes_leave_profile_only_check_unchanged(self):
        original = context()
        record = result(original, PROFILE)
        changed = replace(original, tool=ConfigurationArtifact(None, 'unknown tool'),
                          mounting_transform=ConfigurationArtifact(None, 'unknown mount'))
        self.assertEqual(record.invalidate_changed(changed), record)

    def test_every_result_is_scoped_to_full_build_profile(self):
        original = context()
        record = result(original, PROFILE | {Dependency.TOOL})
        for changes in ({'robot':'another-robot'}, {'build':'another-build'},
                        {'backend':'simulation'}, {'revision':'another-revision'}, {'joints':()}):
            with self.subTest(changes=changes):
                changed = replace(original, profile=replace(original.profile, **changes))
                self.assertIs(record.invalidate_changed(changed).status, Status.PENDING)
        for dependencies in (frozenset({Dependency.TOOL}), frozenset({Dependency.MOUNTING_TRANSFORM})):
            with self.subTest(dependencies=dependencies), self.assertRaises(ValueError):
                ValidationRecord('unscoped', dependencies)

    def test_missing_selected_configuration_cannot_have_completed_result(self):
        original = context()
        for name in ('tool', 'mounting_transform'):
            unknown = replace(original, **{name:ConfigurationArtifact(None, 'not configured')})
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    result(unknown)
                self.assertIs(result(original).invalidate_changed(unknown).status, Status.PENDING)
                self.assertIs(result(unknown, PROFILE).status, Status.PASSED)

    def test_reverting_configuration_does_not_restore_invalidated_pass(self):
        original = context()
        changed = replace(original, tool=ConfigurationArtifact(b'new synthetic tool', 'synthetic fixture'))
        pending = result(original).invalidate_changed(changed)
        self.assertIs(pending.invalidate_changed(original).status, Status.PENDING)
        fresh = pending.report_result(changed, status=Status.PASSED, evidence='new synthetic check')
        self.assertIs(fresh.status, Status.PASSED)
        self.assertEqual(fresh.evidence, 'new synthetic check')
        self.assertIs(fresh.invalidate_changed(original).status, Status.PENDING)

    def test_failed_results_also_become_pending_on_relevant_change(self):
        original = context()
        changed = replace(original, tool=ConfigurationArtifact(b'changed tool', 'synthetic fixture'))
        self.assertIs(result(original, status=Status.FAILED).invalidate_changed(changed).status,
                      Status.PENDING)

    def test_profile_schema_result_does_not_certify_unknown_physical_facts(self):
        original = context()
        self.assertIsNone(original.profile.joints[0].actuator_identity)
        self.assertEqual(original.profile.joints[0].mapping_status, 'declared')
        reported = result(original, PROFILE)
        self.assertIs(reported.status, Status.PASSED)
        self.assertIsNone(original.profile.joints[0].actuator_identity)
        self.assertEqual(original.profile.joints[0].mapping_status, 'declared')

    def test_artifact_input_boundaries_and_frozen_data(self):
        for bad in (b'', '', bytearray(b'a'), True, 1, []):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                ConfigurationArtifact(bad, 'synthetic fixture')
        for bad in ('', ' padded ', None, True):
            with self.subTest(source=bad), self.assertRaises(ValueError):
                ConfigurationArtifact(b'a', bad)
        artifact = ConfigurationArtifact(b'a', 'synthetic fixture')
        with self.assertRaises(FrozenInstanceError):
            artifact.data = b'b'

    def test_tool_and_mount_dependencies_are_independent(self):
        original = context()
        tool_check = result(original, PROFILE | {Dependency.TOOL})
        mount_check = result(original, PROFILE | {Dependency.MOUNTING_TRANSFORM})
        changed_tool = replace(original, tool=ConfigurationArtifact(b'changed tool', 'synthetic'))
        changed_mount = replace(original, mounting_transform=ConfigurationArtifact(b'changed mount', 'synthetic'))
        self.assertEqual(tool_check.invalidate_changed(changed_mount), tool_check)
        self.assertEqual(mount_check.invalidate_changed(changed_tool), mount_check)
        self.assertIs(tool_check.invalidate_changed(changed_tool).status, Status.PENDING)
        self.assertIs(mount_check.invalidate_changed(changed_mount).status, Status.PENDING)

    def test_new_context_types_cannot_override_immutable_fingerprints(self):
        original = context()
        class MutableArtifact(ConfigurationArtifact):
            pass
        class MutableContext(ValidationContext):
            pass
        with self.assertRaises(ValueError):
            replace(original, tool=MutableArtifact(b'a', 'synthetic'))
        with self.assertRaises(ValueError):
            result(original).invalidate_changed(MutableContext(
                original.profile, original.tool, original.mounting_transform))

    def test_record_input_boundaries(self):
        for changes in ({'check_id':''}, {'check_id':' padded '}, {'dependencies':set(PROFILE)},
                        {'dependencies':frozenset()}, {'dependencies':frozenset({'profile'})},
                        {'status':'passed'}, {'evidence':'old evidence'}, {'input_hashes':[]}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(ValidationRecord('synthetic', PROFILE), **changes)
        original = context()
        for status in (Status.PENDING, 'passed', None, True):
            with self.subTest(status=status), self.assertRaises(ValueError):
                ValidationRecord('synthetic', PROFILE).report_result(
                    original, status=status, evidence='synthetic fixture')
        for evidence in ('', ' padded ', None):
            with self.subTest(evidence=evidence), self.assertRaises(ValueError):
                ValidationRecord('synthetic', PROFILE).report_result(
                    original, status=Status.PASSED, evidence=evidence)

    def test_completed_record_requires_exact_unique_dependency_hashes(self):
        for hashes in ((), ((Dependency.TOOL,'a'*64),),
                       ((Dependency.PROFILE,'A'*64),), ((Dependency.PROFILE,'a'*63),),
                       ((Dependency.PROFILE,'a'*64),(Dependency.PROFILE,'b'*64)),
                       ([Dependency.PROFILE,'a'*64],)):
            with self.subTest(hashes=hashes), self.assertRaises(ValueError):
                ValidationRecord('synthetic', PROFILE, Status.PASSED, 'synthetic fixture', hashes)
        record = result(context())
        with self.assertRaises(FrozenInstanceError):
            record.status = Status.PASSED

    def test_context_boundary_and_canonical_profile_roundtrip(self):
        original = context()
        from build_profile import parse_json
        canonical = replace(original, profile=parse_json(original.profile.to_json()))
        self.assertEqual(result(original).invalidate_changed(canonical).status, Status.PASSED)
        for changes in ({'profile':None}, {'tool':b'tool'}, {'mounting_transform':None}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(original, **changes)
        with self.assertRaises(ValueError):
            result(original).invalidate_changed(None)
        with self.assertRaises(ValueError):
            ValidationRecord('synthetic', PROFILE).report_result(
                None, status=Status.PASSED, evidence='synthetic fixture')


if __name__ == '__main__':
    unittest.main()
