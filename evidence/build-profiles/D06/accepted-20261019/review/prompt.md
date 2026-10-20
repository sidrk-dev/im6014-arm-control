Review only this new Workbench D06 artifact. You may inspect only supplied text. No tools, shell, tests, subagents, network, hardware or Git operations. Give actual blocking correctness findings with concrete repro and file/line, or state none. Final source-grounded review required. Do not claim tests you did not execute.

D06 requires changed reduction/tool/mounting transform to mark relevant tests pending. Parent personally authored new isolated source based on D05 d0093b1. BuildProfile canonical serialized data has explicit identity/revision/ordered joints/mapping/reduction/provenance/unknown measurements/locked bodies, but no tool or mounting geometry fields. D05 exact opaque artifact/fullprofile binding is unchanged. New D06 uses strict immutable opaque configuration documents, NO transform parser or semantic certification. Every check must bind full PROFILE; tool/mount dependencies optional per check. Invalidate returns a sticky pending record; caller must retain returned state. report_result is explicitly a caller declaration with unverified evidence reference, not execution or physical readiness. Unknown selected tool/mount artifacts prevent completed records; inner profile unknowns remain unknown even when a schema check passes. Runtime owner/Core/adapter untouched. No persistence or autoapply. Parent16focused checks passed; full baseline pending. Source hashes below.

{
  "validation_records.py": "2122e4f7d6c1b4a817cbf4730e0b014286a8e0edc3c5b29f12becbc0e64fb112",
  "test_validation_invalidation.py": "65b12482b062775cd9d4430a14d05c32d643082911f6bf0919f2954f3d17b2db",
  "docs/validation-invalidation.md": "b0ae40f85919a64cb5fa663cddb751fbfc05c20c563493f2e5a767318b798226",
  "docs/architecture.md": "bc4b08cc832c21d9f84720cdc4226802f30686011401fb9c4b913b4d3b2bc5f5",
  "scripts/check_milestone.py": "b6bc01f1fc164c48682930c0af13abed6e31152c13b92a118bfc20851b18cd1d",
  "scripts/check_offline_baseline.sh": "ca48d1c8b62e3f621d6dd9de25aedd638b80d658aa438cae3e10f2920417e907",
  "test_startup.py": "9dda425f6a481a072fa2d9387d0565c0f6a99e36a65a6408afaca68be7473dcf"
}

FILE validation_records.py
```
from dataclasses import dataclass, replace
from enum import Enum
import hashlib
import json
import re

from build_profile import BuildProfile


class ValidationDependency(Enum):
    PROFILE = 'profile'
    TOOL = 'tool'
    MOUNTING_TRANSFORM = 'mounting_transform'


class ValidationStatus(Enum):
    PENDING = 'pending'
    PASSED = 'passed'
    FAILED = 'failed'


def _text(value, name):
    if type(value) is not str or not value.strip() or value != value.strip():
        raise ValueError(f'{name} must be a non-empty unpadded string')


@dataclass(frozen=True)
class ConfigurationArtifact:
    data: bytes | None
    source: str

    def __post_init__(self):
        _text(self.source, 'configuration source')
        if self.data is not None and (type(self.data) is not bytes or not self.data):
            raise ValueError('configuration data must be non-empty immutable bytes or None for unknown')

    def fingerprint(self) -> str | None:
        if self.data is None:
            return None
        content = json.dumps({'source': self.source,
                              'sha256': hashlib.sha256(self.data).hexdigest()}, sort_keys=True)
        return hashlib.sha256(content.encode('utf-8')).hexdigest()


@dataclass(frozen=True)
class ValidationContext:
    profile: BuildProfile
    tool: ConfigurationArtifact
    mounting_transform: ConfigurationArtifact

    def __post_init__(self):
        if type(self.profile) is not BuildProfile:
            raise ValueError('profile must be a BuildProfile')
        if any(type(value) is not ConfigurationArtifact
               for value in (self.tool, self.mounting_transform)):
            raise ValueError('tool and mounting_transform must be ConfigurationArtifact records')

    def fingerprint(self, dependency: ValidationDependency) -> str | None:
        if dependency is ValidationDependency.PROFILE:
            return hashlib.sha256(self.profile.to_json().encode('utf-8')).hexdigest()
        if dependency is ValidationDependency.TOOL:
            return self.tool.fingerprint()
        if dependency is ValidationDependency.MOUNTING_TRANSFORM:
            return self.mounting_transform.fingerprint()
        raise ValueError('dependency must be a ValidationDependency')


@dataclass(frozen=True)
class ValidationRecord:
    check_id: str
    dependencies: frozenset[ValidationDependency]
    status: ValidationStatus = ValidationStatus.PENDING
    evidence: str | None = None
    input_hashes: tuple[tuple[ValidationDependency, str], ...] = ()

    def __post_init__(self):
        _text(self.check_id, 'check ID')
        if (type(self.dependencies) is not frozenset or not self.dependencies
                or any(not isinstance(item, ValidationDependency) for item in self.dependencies)):
            raise ValueError('dependencies must be a non-empty frozenset of ValidationDependency')
        if ValidationDependency.PROFILE not in self.dependencies:
            raise ValueError('every check must bind its build profile')
        if not isinstance(self.status, ValidationStatus):
            raise ValueError('status must be a ValidationStatus')
        if type(self.input_hashes) is not tuple:
            raise ValueError('input_hashes must be an immutable tuple')
        keys = []
        for pair in self.input_hashes:
            if type(pair) is not tuple or len(pair) != 2:
                raise ValueError('input hashes must contain immutable dependency/digest pairs')
            dependency, digest = pair
            if not isinstance(dependency, ValidationDependency):
                raise ValueError('input hash key must be a ValidationDependency')
            if type(digest) is not str or re.fullmatch('[0-9a-f]{64}', digest) is None:
                raise ValueError('input hash must be a lowercase SHA256 digest')
            keys.append(dependency)
        if len(set(keys)) != len(keys):
            raise ValueError('input hash dependencies must be unique')
        if self.status is ValidationStatus.PENDING:
            if self.evidence is not None or self.input_hashes:
                raise ValueError('pending validation cannot carry a completed result')
        else:
            _text(self.evidence, 'result evidence reference')
            if set(keys) != self.dependencies:
                raise ValueError('completed validation must bind every declared dependency')
        object.__setattr__(self, 'input_hashes', tuple(sorted(
            self.input_hashes, key=lambda pair: pair[0].value)))

    def report_result(self, context: ValidationContext, *,
                      status: ValidationStatus, evidence: str) -> 'ValidationRecord':
        if type(context) is not ValidationContext:
            raise ValueError('context must be a ValidationContext')
        if status not in (ValidationStatus.PASSED, ValidationStatus.FAILED):
            raise ValueError('a reported result must be passed or failed')
        hashes = tuple((item, context.fingerprint(item)) for item in self.dependencies)
        if any(digest is None for _, digest in hashes):
            raise ValueError('required configuration is unknown; validation remains pending')
        return replace(self, status=status, evidence=evidence, input_hashes=hashes)

    def invalidate_changed(self, context: ValidationContext) -> 'ValidationRecord':
        if type(context) is not ValidationContext:
            raise ValueError('context must be a ValidationContext')
        if self.status is ValidationStatus.PENDING:
            return self
        if any(context.fingerprint(dependency) != digest
               for dependency, digest in self.input_hashes):
            return ValidationRecord(self.check_id, self.dependencies)
        return self

```

FILE test_validation_invalidation.py
```
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

```

FILE docs/validation-invalidation.md
```
# Invalidate reported validation after configuration changes

D06 tracks caller-reported software validation results against configuration inputs. A changed profile, selected tool artifact, or selected mounting-transform artifact returns a pending record. The caller must retain that returned record as current state. Reverting configuration does not restore its previous pass. A new explicit result report is required.

`ValidationContext` holds the existing `BuildProfile` and two `ConfigurationArtifact` records. Each artifact contains exact immutable bytes and an explicit source. `None` means unknown. There is no default tool or identity transform. These artifacts are opaque configuration documents. This module does not parse geometry, transform conventions, units or tool properties, and cannot detect a physical change that was not reflected in its inputs.

Every check explicitly declares a nonempty frozen set of `ValidationDependency` values. `PROFILE` is mandatory, so a result cannot move between robots, builds, backends, revisions or joint configurations. All canonical profile changes conservatively invalidate every check. A tool or mount change affects only checks declaring that additional dependency. Artifact whitespace and provenance changes count as changes.

```python
from dataclasses import replace
from build_profiles import arm_j1
from joint_mapping import JointMap
from validation_records import (
    ConfigurationArtifact, ValidationContext, ValidationDependency as Dependency,
    ValidationRecord, ValidationStatus as Status,
)

profile = arm_j1(revision='synthetic-v1', bus='synthetic-bus', motor_id=0,
    mapping=JointMap(1, 1, 0), mapping_source='synthetic example',
    actuator_identity=None, actuator_identity_source='unverified').profile
inputs = ValidationContext(profile,
    ConfigurationArtifact(b'{"tool":"synthetic scoop"}', 'synthetic tool document'),
    ConfigurationArtifact(None, 'mounting transform not configured'))
record = ValidationRecord('synthetic-tool-check',
    frozenset({Dependency.PROFILE, Dependency.TOOL}))
record = record.report_result(inputs, status=Status.PASSED,
    evidence='synthetic offline result reference')
changed = replace(inputs,
    tool=ConfigurationArtifact(b'{"tool":"synthetic gripper"}', 'synthetic tool document'))
record = record.invalidate_changed(changed)
assert record.status is Status.PENDING
assert record.evidence is None
assert record.invalidate_changed(inputs).status is Status.PENDING
```

A missing selected tool or mounting artifact rejects `report_result`. Unknown measurements and declared mappings inside the profile retain their original statuses. A schema check can report a pass for a document that explicitly contains unknown physical facts. That does not establish physical readiness.

`report_result` records a caller declaration, not a test execution. Its evidence string is an unverified reference, not a file or content digest. The caller must supply the actual tested context and actual outcome. This module does not verify that a result ran, certify calibration contents, or prevent deliberate reuse of an older immutable record. It has no persistence parser, live application, saved-zero restoration, fault clearing or motion authority. D05 calibration binding remains a separate exact-profile/artifact check.

```

FILE docs/architecture.md
```
# Repository ownership and component boundaries

## One repository

Use one private `sidrk-dev/im6014-workbench` repository for the bench, future controller integration, workbench adapters, and milestone evidence. These components currently share one developer, one control contract, and one verification cycle. Separate repositories or Git submodules would add version coordination without an independent release boundary.

The bench history is inherited from `sidrk-dev/im6014Development` at `69c8219d764a24d03dadf1df011efb027f0f8996`. The `bench-upstream` remote records that source. Publish workbench changes to `origin`. Review upstream changes before incorporating them into the protocol boundary.

Inspect external foundations at exact revisions and record the revision in evidence. Add a submodule only when a component needs an independent release and its interface is stable. Do not add placeholder submodules for unimplemented milestones.

## Current dependency direction

| Component | Owns | Imports or calls |
| --- | --- | --- |
| `start_dashboard.sh` | Resolve application path and start Python | `motor_dashboard.main` |
| `motor_dashboard.py` | Controls, display, plots, exports, UI heartbeat | `MotorSession`; codec constants; Qt; passive serial enumeration |
| `motor_session.py` | One GUI serial-owner worker, requests, demo transport, bench lifecycle | Bench aliases in `spin_5rpm.py`; pyserial |
| `im6014_codec.py` | Pure packet encoding, strict feedback decoding, CRC and internal reduction | Python standard library |
| `im6014_serial.py` | One request, bounded stream scan, timeout budgets, restoration and optional bounded capture | Pure feedback decoder; Python standard library |
| `im6014_port.py` | Passive configuration and cooperating device lease | OS metadata; injected serial factory |
| `station_timing.py` | Fixed-storage owner loop and exchange-attempt timing observations | Python standard library; called by `arm_manual.py` |
| `joint_records.py` | Frozen public domain records and enums with validated constructor contracts | `joint_mapping.py`; Python standard library |
| `joint_reference.py` | Pure reference planning, codec projection corridors, and release finite-difference rate math | `im6014_codec.py`; `joint_mapping.py`; `joint_records.py`; Python standard library |
| `joint_controller.py` | Transport-free lifecycle, clock, per-joint freshness and faults, identity-checked owner grants, pending targets, reference history, and sequence watermark | `joint_records.py`; `joint_reference.py`; `im6014_codec.py`; Python standard library |
| `validation_records.py` | Pure caller-reported validation status and configuration dependency invalidation | `build_profile.py`; Python standard library; no runtime owner integration |
| `joint_mapping.py` | Pure external joint units and gain conversion | Python standard library |
| `spin_5rpm.py` | Bench aliases, health checks and standalone bounded CLI | Codec and serial boundary; pyserial |
| `fake_serial.py` | Scripted replies and deterministic clock for offline checks | Python standard library; never imported by production |
| Offline tests and startup verifier | Fixtures, rejection guards, visible evidence | Production modules; never imported by production |
| Baseline driver | Run audited checks on exact temporary source copies | Tests and selected Python interpreter |
| `docs/milestones.tsv` | Persistent acceptance results with source snapshots | Evidence paths; no runtime dependency |

The GUI worker and standalone CLI acquire the same cooperating device lease through `im6014_port.open_managed_port`. Aliases of a character device collide by `st_rdev`. The fixed private per-user namespace ignores TMPDIR and XDG settings. A second cooperating opener rejects before serial construction, and unconfirmed close retains ownership. Raw clients can ignore this advisory lock. Neither entry point is yet a complete station process for a six-joint arm.

B01-B10 extract the existing codec into pure functions and the serial exchange into a separate boundary. The bench aliases preserve callers and startup interception. Build profiles and the shared station remain later work. A01-A10 baseline records describe their earlier source revisions.

## MakerMods integration boundary

MakerMods Lab is the planned E01-E10 operator interface. Current inspection revision is `64defd4e5e9641199b1af5b90587080f721374ba`. Its [pinned pyproject](https://github.com/makermods-robotics/makermodslab/blob/64defd4e5e9641199b1af5b90587080f721374ba/pyproject.toml) requires Python `>=3.12,<3.14` and pins its LeRobot fork to `5ce2fe41a63647c20b2ab6e84bcb4cc7295a9b21`.

This repository uses Python 3.12 for its clean bench environment. MakerMods and LeRobot are not installed, integrated, or verified by A01-A10. Their later custom-arm adapter must submit requests through the shared station. It must not open a second serial connection.

## Evidence identities

`evidence/baseline` contains the committed parent-run snapshot. Later verification runs use fresh directories under `evidence/runs`, which are ignored. Fake motor outputs are marked as simulation. Disconnected startup contains no measured motor state. Hardware commissioning remains pending.

Dashboard ingress and owner cleanup are described in [station-independence.md](station-independence.md). The HTTP layer receives the existing owner; server construction/loop/cleanup failures do not skip owner cleanup.

```

FILE scripts/check_milestone.py
```
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess


CHECKS = {
    'B11': 'test_joint_mapping',
    'B12': 'test_numerical_inputs',
    'B13': 'test_port_config',
    'B14': 'test_diagnostic_capture',
    'C01': 'test_bus_owner',
    'C02': 'test_controller.ControllerStateTests',
    'C03': 'test_controller.FeedbackAgeTests',
    'C04': 'test_controller.JointFaultTests',
    'C05': 'test_controller_targets.LatestTargetTests',
    'C06': 'test_controller_targets.CommandSequenceTests',
    'C07': 'test_controller_targets.CommandExpiryTests',
    'C08': 'test_controller_ownership',
    'C09': 'test_controller_limits',
    'C10': 'test_controller_velocity',
    'C11': 'test_controller_acceleration',
    'C12': 'test_controller_lead',
    'C13': 'test_controller_effort',
    'C14': 'test_controller_faults',
    'C15': 'test_station_timing',
    'C16': 'test_station_independence',
    'D01': 'test_build_profile test_build_profile_boundary',
    'D02': 'test_build_profiles',
    'D03': 'test_provisional_builds',
    'D04': 'test_locked_joints',
    'D05': 'test_calibration_binding',
    'D06': 'test_validation_invalidation',
}


def fingerprint(repo):
    paths = sorted(repo.glob('*.py')) + [
        repo / 'scripts/check_offline_baseline.sh',
        repo / 'scripts/check_milestone.py', repo / 'USER_MANUAL.md',
    ]
    return {str(p.relative_to(repo)): {
        'bytes': p.stat().st_size,
        'sha256': hashlib.sha256(p.read_bytes()).hexdigest(),
    } for p in paths}


def main():
    repo = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description='Run one audited offline milestone check.')
    parser.add_argument('unit', choices=CHECKS)
    parser.add_argument('--python', type=Path, default=repo / '.venv/bin/python')
    parser.add_argument('--evidence-root', type=Path, default=repo / 'evidence/runs')
    args = parser.parse_args()
    root = args.evidence_root.resolve()
    if root == repo or root == Path('/') or repo in root.parents and repo / 'evidence/runs' != root and repo / 'evidence/runs' not in root.parents:
        parser.error('Evidence inside the repository must be under evidence/runs')
    out = root / (args.unit + '-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    out.mkdir(parents=True, exist_ok=False)
    before = fingerprint(repo)
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    command = [str(args.python.absolute()), '-B', '-m', 'unittest', '-v', *CHECKS[args.unit].split()]
    result = subprocess.run(command, cwd=repo, capture_output=True, text=True,
                            env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1', QT_QPA_PLATFORM='offscreen'))
    (out / 'stdout.log').write_text(result.stdout)
    (out / 'stderr.log').write_text(result.stderr)
    unchanged = before == fingerprint(repo)
    (out / 'receipt.json').write_text(json.dumps({
        'unit': args.unit, 'command': command, 'cwd': str(repo), 'baseHead': head,
        'sourceFiles': before, 'sourceUnchanged': unchanged,
        'exitCode': result.returncode, 'recordedAt': datetime.now(timezone.utc).isoformat(),
        'physicalIO': 'none; parent reviewed fixtures or pseudo-terminals before execution',
    }, indent=2) + '\n')
    print(result.stdout, end='')
    print(result.stderr, end='')
    print('Receipt:', out / 'receipt.json')
    return result.returncode if unchanged else 2


if __name__ == '__main__':
    raise SystemExit(main())

```

FILE scripts/check_offline_baseline.sh
```
#!/usr/bin/env bash
set -u
set -o pipefail

if (( $# > 2 )); then
    printf 'Usage: %s [absolute-python] [evidence-root]\n' "$0" >&2
    exit 2
fi

caller_dir="$(pwd -P)"
script_path="$(realpath -e -- "$0")" || exit 2
repo_dir="$(cd -- "$(dirname -- "$script_path")/.." && pwd -P)"
python_arg="${1:-$repo_dir/.venv/bin/python}"
evidence_arg="${2:-$repo_dir/evidence/runs}"
case "$python_arg$evidence_arg" in
    *$'\n'*|*$'\t'*) printf 'Paths cannot contain tabs or newlines.\n' >&2; exit 2 ;;
esac

case "$python_arg" in
    /*) ;;
    *) python_arg="$caller_dir/$python_arg" ;;
esac
python_dir_arg="$(dirname -- "$python_arg")"
python_name="$(basename -- "$python_arg")"
python_dir="$(cd -- "$python_dir_arg" 2>/dev/null && pwd -P)" || {
    printf 'Python interpreter does not exist: %s\n' "$python_arg" >&2
    exit 2
}
python_path="$python_dir/$python_name"
if [[ ! -f "$python_path" || ! -x "$python_path" ]]; then
    printf 'Python interpreter must be an executable file: %s\n' "$python_path" >&2
    exit 2
fi

case "$evidence_arg" in
    /*) ;;
    *) evidence_arg="$caller_dir/$evidence_arg" ;;
esac
mkdir -p -- "$evidence_arg" || {
    printf 'Could not create evidence root: %s\n' "$evidence_arg" >&2
    exit 2
}
evidence_root="$(realpath -e -- "$evidence_arg")" || exit 2
if [[ "$evidence_root" == / || "$evidence_root" == "$repo_dir" ]]; then
    printf 'Refusing unsafe evidence root: %s\n' "$evidence_root" >&2
    exit 2
fi
case "$evidence_root/" in
    "$repo_dir/"*)
        case "$evidence_root/" in
            "$repo_dir/evidence/runs/"|"$repo_dir/evidence/runs/"*) ;;
            *) printf 'Evidence inside the repository must be under evidence/runs: %s\n' "$evidence_root" >&2; exit 2 ;;
        esac
        ;;
esac

run_dir="$(mktemp -d -- "$evidence_root/run-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXXXX")" || exit 2
scratch="$(mktemp -d -- "$run_dir/.source-XXXXXXXX")" || exit 2
cleanup() {
    rm -rf -- "$scratch"
}
trap cleanup EXIT

printf 'check\texit_code\n' > "$run_dir/checks.tsv"
failed=0
record_check() {
    local name="$1"
    local code="$2"
    printf '%s\t%s\n' "$name" "$code" >> "$run_dir/checks.tsv"
    if (( code != 0 )); then failed=1; fi
}
run_logged() {
    local name="$1"
    shift
    "$@" > "$run_dir/$name.log" 2>&1
    local code=$?
    record_check "$name" "$code"
    return 0
}

files=(motor_dashboard.py motor_session.py spin_5rpm.py im6014_codec.py im6014_serial.py im6014_transport.py fake_serial.py test_spin_5rpm.py test_dashboard.py test_startup.py test_startup_verifier.py test_protocol.py test_serial_deadline.py evidence/boundary/vendor-crc-vectors.json USER_MANUAL.md joint_mapping.py test_joint_mapping.py test_numerical_inputs.py im6014_port.py test_port_config.py test_diagnostic_capture.py test_bus_owner.py joint_controller.py joint_records.py test_controller.py test_controller_targets.py test_controller_ownership.py test_controller_limits.py test_controller_velocity.py joint_reference.py test_controller_acceleration.py test_controller_lead.py test_controller_effort.py test_controller_faults.py station_timing.py test_station_timing.py arm_manual.py arm_dashboard.py test_station_independence.py build_profile.py test_build_profile.py test_build_profile_boundary.py build_profiles.py test_build_profiles.py test_provisional_builds.py test_locked_joints.py calibration_binding.py test_calibration_binding.py validation_records.py test_validation_invalidation.py)
for name in "${files[@]}"; do
    mkdir -p -- "$scratch/$(dirname -- "$name")" || exit 2
    if ! cp -- "$repo_dir/$name" "$scratch/$name" || ! cmp -s -- "$repo_dir/$name" "$scratch/$name"; then
        printf 'Could not create a byte-identical baseline copy: %s\n' "$name" >&2
        exit 2
    fi
done

if "$python_path" -c 'import sys; print(sys.version)' > "$run_dir/python-version.txt" 2>&1; then
    record_check interpreter 0
else
    code=$?
    record_check interpreter "$code"
fi

cd -- "$scratch" || exit 2
run_logged packet-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" test_spin_5rpm.py
run_logged protocol-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_protocol
run_logged serial-deadline-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_serial_deadline
run_logged milestone-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" -m unittest -v test_joint_mapping test_numerical_inputs test_port_config test_diagnostic_capture test_bus_owner test_controller test_controller_targets test_controller_ownership test_controller_limits test_controller_velocity test_controller_acceleration test_controller_lead test_controller_effort test_controller_faults test_station_timing test_station_independence test_build_profile test_build_profile_boundary test_build_profiles test_provisional_builds test_locked_joints test_calibration_binding test_validation_invalidation
run_logged dashboard-tests env QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 "$python_path" test_dashboard.py
for name in dashboard_preview.png dashboard_dark_preview.png dashboard_test_results.json; do
    if [[ -f "$scratch/$name" ]]; then
        if ! cp -- "$scratch/$name" "$run_dir/$name"; then record_check "copy-$name" 1; fi
    fi
done

run_logged startup-verifier-tests env PYTHONDONTWRITEBYTECODE=1 "$python_path" test_startup_verifier.py
run_logged startup-real env QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 \
    "$python_path" test_startup.py --source real --evidence-dir "$run_dir/startup-real"
run_logged startup-demo env QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1 \
    "$python_path" test_startup.py --source demo --evidence-dir "$run_dir/startup-demo"

printf 'Evidence: %s\n' "$run_dir"
cat "$run_dir/checks.tsv"
exit "$failed"

```

FILE test_startup.py
```
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys
import traceback


def _need(condition, message):
    if not condition:
        raise AssertionError(message)


def validate_report(report):
    _need(report.get("system_exit") is True, "application main did not exit through SystemExit")
    _need(not report.get("callback_errors"), "event-loop callback or close raised an exception")
    _need(report.get("exit_code") == 0, f"application exit code was {report.get('exit_code')!r}")
    _need(not report.get("main_exception"), "application main raised an exception")
    _need(report.get("argv_restored") is True, "sys.argv was not restored")
    _need(report.get("window_captured") is True, "Dashboard constructor did not produce a window")
    _need(report.get("callback_ran") is True, "event-loop verification callback did not run")
    _need(report.get("callback_completed") is True, "event-loop verification callback did not complete")

    source = report.get("source")
    _need(source in ("real", "demo"), f"invalid source mode: {source!r}")
    pre, post = report.get("pre") or {}, report.get("post") or {}
    expected_demo = source == "demo"
    for name, observation in (("pre-close", pre), ("post-close", post)):
        _need(observation.get("bus_is_none") is True, f"{name}: session bus was not None")
        _need(observation.get("state") == "Disconnected", f"{name}: dashboard was not Disconnected")
        _need(observation.get("connected") is False, f"{name}: session reported connected")
        _need(observation.get("armed") is False, f"{name}: session was armed")
        _need(observation.get("tx") == 0 and observation.get("rx") == 0,
              f"{name}: packet counters were not zero")
        _need(observation.get("source_selected_demo") is expected_demo,
              f"{name}: selected source did not match {source}")
    _need(pre.get("window_visible") is True, "dashboard window was not visible")
    _need(pre.get("timer_active") is True, "dashboard UI timer was not active")
    _need(pre.get("worker_alive") is True, "serial-owner worker was not alive before close")
    _need(pre.get("event_loop_tick_seen") is True, "normal UI timer did not update in the event loop")
    _need(post.get("window_visible") is False, "dashboard window remained visible after close")
    _need(post.get("timer_active") is False, "dashboard UI timer remained active after close")
    _need(post.get("worker_alive") is False, "serial-owner worker remained alive after close")
    _need(report.get("close_returned") is True, "normal window close did not succeed")

    counts = report.get("sentinel_counts") or {}
    for name in ("serial_open", "demo_open", "exchange"):
        _need(counts.get(name) == 0, f"forbidden {name} attempt count was {counts.get(name)!r}")

    screenshot = report.get("screenshot") or {}
    _need(screenshot.get("saved") is True, "rendered dashboard screenshot was not saved")
    _need(screenshot.get("width", 0) > 0 and screenshot.get("height", 0) > 0,
          "rendered dashboard screenshot has invalid dimensions")
    _need(screenshot.get("bytes", 0) > 0, "rendered dashboard screenshot is empty")
    digest = screenshot.get("sha256", "")
    _need(len(digest) == 64 and all(c in "0123456789abcdef" for c in digest),
          "rendered dashboard screenshot has an invalid SHA-256")


class ForbiddenMotorIO:
    def __init__(self):
        self.counts = {"serial_open": 0, "demo_open": 0, "exchange": 0}

    def reject(self, kind, *args, **kwargs):
        self.counts[kind] += 1
        raise RuntimeError(f"offline startup verifier blocked {kind}")

    def serial(self, *args, **kwargs):
        return self.reject("serial_open", *args, **kwargs)

    def demo(self, *args, **kwargs):
        return self.reject("demo_open", *args, **kwargs)

    def exchange(self, *args, **kwargs):
        return self.reject("exchange", *args, **kwargs)


def _observe(window, event_loop_tick_seen=False):
    snap = window.session.snapshot()
    return {
        "bus_is_none": window.session.bus is None,
        "state": snap["state"],
        "connected": snap["connected"],
        "armed": snap["armed"],
        "tx": snap["tx"],
        "rx": snap["rx"],
        "source_selected_demo": window.source.currentIndex() == 1,
        "window_visible": window.isVisible(),
        "timer_active": window.timer.isActive(),
        "worker_alive": window.session.thread.is_alive(),
        "event_loop_tick_seen": bool(event_loop_tick_seen),
    }


def _cleanup_after_main(window, state):
    if window is None or not hasattr(window, "session") or not window.session.thread.is_alive():
        return
    try:
        if not state["callback_completed"] and hasattr(window, "timer"):
            state["close_returned"] = bool(window.close())
            state["post"] = _observe(window, False)
        else:
            window.session.close()
    except BaseException:
        state["callback_errors"].append("post-main cleanup failed\n" + traceback.format_exc())


def _fingerprint(root):
    names = ("motor_dashboard.py", "motor_session.py", "spin_5rpm.py", "im6014_codec.py", "im6014_serial.py", "im6014_transport.py", "fake_serial.py", "USER_MANUAL.md",
             "test_spin_5rpm.py", "test_dashboard.py", "test_startup.py", "test_startup_verifier.py", "test_protocol.py", "test_serial_deadline.py", "evidence/boundary/vendor-crc-vectors.json", "joint_mapping.py", "test_joint_mapping.py", "test_numerical_inputs.py", "im6014_port.py", "test_port_config.py", "test_diagnostic_capture.py", "test_bus_owner.py", "joint_controller.py", "joint_records.py", "test_controller.py", "test_controller_targets.py", "test_controller_ownership.py", "test_controller_limits.py", "test_controller_velocity.py", "joint_reference.py", "test_controller_acceleration.py", "test_controller_lead.py", "test_controller_effort.py", "test_controller_faults.py", "station_timing.py", "test_station_timing.py", "arm_manual.py", "arm_dashboard.py", "test_station_independence.py", "build_profile.py", "test_build_profile.py", "test_build_profile_boundary.py", "build_profiles.py", "test_build_profiles.py", "test_provisional_builds.py", "test_locked_joints.py", "calibration_binding.py", "test_calibration_binding.py", "validation_records.py", "test_validation_invalidation.py")
    files = {}
    for name in names:
        path = root / name
        data = path.read_bytes()
        files[name] = {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
    return files


def run(source, evidence_dir):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    evidence_dir = Path(evidence_dir).expanduser().resolve()
    evidence_dir.mkdir(parents=True, exist_ok=False)
    screenshot_path = evidence_dir / "dashboard.png"
    result_path = evidence_dir / "startup-result.json"
    root = Path(__file__).resolve().parent
    original_argv = sys.argv
    original_argv_values = list(original_argv)
    main_argv = [str(root / "motor_dashboard.py")]
    if source == "demo":
        main_argv.append("--demo")

    io = ForbiddenMotorIO()
    state = {
        "app": None,
        "window": None,
        "initial_ping": None,
        "callback_ran": False,
        "callback_completed": False,
        "callback_errors": [],
        "pre": None,
        "post": None,
        "close_returned": False,
        "screenshot": None,
    }

    try:
        from PySide6.QtCore import QTimer
        from PySide6.QtWidgets import QApplication
        import motor_dashboard as dashboard
        import motor_session

        original_dashboard = dashboard.Dashboard
        original_application = dashboard.QApplication

        def add_error(label):
            state["callback_errors"].append(label + "\n" + traceback.format_exc())

        def close_after_inspection():
            if state["callback_ran"]:
                return
            state["callback_ran"] = True
            window = state["window"]
            app = state["app"]
            try:
                if window is None:
                    raise AssertionError("Dashboard constructor did not produce a window")
                app.processEvents()
                event_tick = (state["initial_ping"] is not None and
                              window.session.last_ui > state["initial_ping"])
                state["pre"] = _observe(window, event_tick)
                pixmap = window.grab()
                _need(not pixmap.isNull(), "dashboard grab returned a null pixmap")
                _need(pixmap.width() > 0 and pixmap.height() > 0, "dashboard grab has invalid dimensions")
                _need(pixmap.save(str(screenshot_path), "PNG"), "could not save dashboard PNG")
                image = screenshot_path.read_bytes()
                state["screenshot"] = {
                    "path": str(screenshot_path),
                    "saved": screenshot_path.is_file(),
                    "width": pixmap.width(),
                    "height": pixmap.height(),
                    "bytes": len(image),
                    "sha256": hashlib.sha256(image).hexdigest(),
                }
            except BaseException:
                add_error("startup observation failed")
            finally:
                try:
                    app.setQuitOnLastWindowClosed(False)
                    if window is not None:
                        state["close_returned"] = bool(window.close())
                        state["post"] = _observe(window, False)
                except BaseException:
                    add_error("normal window close failed")
                state["callback_completed"] = True
                app.exit(1 if state["callback_errors"] else 0)

        def watchdog():
            if state["callback_completed"]:
                return
            state["callback_errors"].append("startup event-loop watchdog expired")
            app = state["app"]
            window = state["window"]
            try:
                if window is not None:
                    app.setQuitOnLastWindowClosed(False)
                    state["close_returned"] = bool(window.close())
                    state["post"] = _observe(window, False)
            except BaseException:
                add_error("watchdog cleanup failed")
            app.exit(1)

        class CapturingDashboard(original_dashboard):
            def __init__(self, demo=False):
                state["window"] = self
                super().__init__(demo)
                state["initial_ping"] = self.session.last_ui
                QTimer.singleShot(350, close_after_inspection)

        def capture_application(*args, **kwargs):
            app = original_application(*args, **kwargs)
            state["app"] = app
            app.setQuitOnLastWindowClosed(False)
            QTimer.singleShot(15000, watchdog)
            return app

        from unittest.mock import patch
        with patch.object(dashboard, "Dashboard", CapturingDashboard), \
             patch.object(dashboard, "QApplication", capture_application), \
             patch.object(dashboard, "enumerate_ports", return_value=()), \
             patch.object(motor_session.serial, "Serial", io.serial), \
             patch.object(motor_session, "DemoSerial", io.demo), \
             patch.object(motor_session, "exchange", io.exchange):
            sys.argv = main_argv
            system_exit = False
            exit_code = None
            main_exception = None
            try:
                dashboard.main()
            except SystemExit as error:
                system_exit = True
                exit_code = 0 if error.code is None else error.code
            except BaseException:
                main_exception = traceback.format_exc()
            finally:
                sys.argv = original_argv
                argv_restored = sys.argv is original_argv and sys.argv == original_argv_values

            _cleanup_after_main(state["window"], state)

        report = {
            "created_at": datetime.now(timezone.utc).isoformat(),
            "source": source,
            "argv": main_argv,
            "python": {"executable": sys.executable, "version": sys.version},
            "packages": {
                "PySide6": importlib.metadata.version("PySide6"),
                "pyserial": importlib.metadata.version("pyserial"),
            },
            "source_fingerprint": _fingerprint(root),
            "system_exit": system_exit,
            "exit_code": exit_code,
            "main_exception": main_exception,
            "argv_restored": argv_restored,
            "window_captured": state["window"] is not None,
            "callback_ran": state["callback_ran"],
            "callback_completed": state["callback_completed"],
            "callback_errors": state["callback_errors"],
            "pre": state["pre"],
            "post": state["post"],
            "close_returned": state["close_returned"],
            "sentinel_counts": dict(io.counts),
            "screenshot": state["screenshot"],
        }
        validate_report(report)
        with result_path.open("x", encoding="utf-8") as result_file:
            json.dump(report, result_file, indent=2)
            result_file.write("\n")
        print(f"PASS: {source} dashboard startup and shutdown; evidence: {evidence_dir}", flush=True)
        return 0
    except BaseException as error:
        print(f"FAIL: {source} dashboard startup verification: {error}", file=sys.stderr, flush=True)
        if "report" in locals():
            print(json.dumps(report, indent=2), file=sys.stderr, flush=True)
        else:
            traceback.print_exc()
        return 1
    finally:
        sys.argv = original_argv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("real", "demo"), required=True)
    parser.add_argument("--evidence-dir", required=True)
    args = parser.parse_args()
    return run(args.source, args.evidence_dir)


if __name__ == "__main__":
    raise SystemExit(main())

```

FILE build_profile.py
```
from dataclasses import dataclass
import json
import math

from joint_mapping import JointMap
from joint_records import PositionLimits

SUPPORTED_SCHEMA_VERSIONS = (1,)
ENCODER_CONVENTIONS = ("rotor_incremental", "output_absolute", "unmarked")
MEASUREMENT_STATUSES = ("measured", "declared", "unknown")
BACKENDS = ("physical", "simulation", "demo")

TOP_LEVEL_FIELDS = ("schema_version", "robot", "build", "backend", "revision", "joints", "locked_joints")
REQUIRED_TOP_LEVEL_FIELDS = tuple(name for name in TOP_LEVEL_FIELDS if name != "locked_joints")
JOINT_FIELDS = ("name", "bus", "motor_id", "actuator_identity", "actuator_identity_source", "encoder", "mapping", "mapping_source", "mapping_status", "position_limits", "measurements")
REQUIRED_JOINT_FIELDS = tuple(name for name in JOINT_FIELDS if name not in ("position_limits", "measurements"))
MAPPING_FIELDS = ("reduction", "direction", "zero_output_rad")
LIMIT_FIELDS = ("minimum_rad", "maximum_rad")
MEASUREMENT_FIELDS = ("status", "units", "source", "value")


def _text(value, name):
    if type(value) is not str or not value.strip() or value != value.strip():
        raise ValueError(f"{name} must be a non-empty unpadded string")
    return value


def _object(record, name):
    if not isinstance(record, dict):
        raise ValueError(f"{name} must be a JSON object")
    return record


def _reject_unknown_fields(record, allowed, name):
    for key in record:
        if not isinstance(key, str):
            raise ValueError(f"{name} field names must be strings")
    unknown = sorted(set(record) - set(allowed))
    if unknown:
        raise ValueError(f"{name} has unknown fields: {', '.join(unknown)}")
    return record


def _require_keys(record, required, name):
    missing = sorted(set(required) - set(record))
    if missing:
        raise ValueError(f"{name} is missing required fields: {', '.join(missing)}")
    return record


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a real number")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class Measurement:
    name: str
    status: str
    units: str
    source: str
    value: float | None = None

    def __post_init__(self):
        _text(self.name, "measurement name")
        _text(self.units, "measurement units")
        _text(self.source, "measurement source")
        if self.status not in MEASUREMENT_STATUSES:
            raise ValueError("measurement status must be measured, declared or unknown")
        if self.status == "unknown":
            if self.value is not None:
                raise ValueError("an unknown measurement must not carry a value")
        else:
            if self.value is None:
                raise ValueError(f"a {self.status} measurement requires a value")
            object.__setattr__(self, "value", _number(self.value, "measurement value"))

    def to_dict(self):
        if self.status == "unknown":
            return {"status": "unknown", "units": self.units, "source": self.source}
        return {"status": self.status, "units": self.units, "source": self.source, "value": self.value}


@dataclass(frozen=True)
class JointEntry:
    name: str
    bus: str
    motor_id: int
    actuator_identity: str | None
    actuator_identity_source: str
    encoder: str
    mapping: JointMap
    mapping_source: str
    mapping_status: str
    position_limits: PositionLimits | None = None
    measurements: tuple[Measurement, ...] = ()

    def __post_init__(self):
        _text(self.name, "joint name")
        _text(self.bus, "bus")
        if self.actuator_identity is not None:
            _text(self.actuator_identity, "actuator identity")
        _text(self.actuator_identity_source, "actuator identity source")
        _text(self.mapping_source, "mapping source")
        if self.mapping_status not in ("declared", "measured"):
            raise ValueError("mapping status must be declared or measured")
        if type(self.motor_id) is not int or not 0 <= self.motor_id <= 14:
            raise ValueError("motor_id must be an integer from 0 to 14")
        if self.encoder not in ENCODER_CONVENTIONS:
            raise ValueError("encoder must be a declared encoder convention label")
        if not isinstance(self.mapping, JointMap):
            raise ValueError("mapping must be a JointMap")
        if self.position_limits is not None and not isinstance(self.position_limits, PositionLimits):
            raise ValueError("position_limits must be PositionLimits or None")
        if type(self.measurements) is not tuple or any(
            not isinstance(measurement, Measurement) for measurement in self.measurements
        ):
            raise ValueError("measurements must be a tuple of Measurement")
        names = [measurement.name for measurement in self.measurements]
        if len(names) != len(set(names)):
            raise ValueError("measurement names must be unique")
        object.__setattr__(
            self,
            "measurements",
            tuple(sorted(self.measurements, key=lambda measurement: measurement.name)),
        )

    def measurement(self, name):
        for measurement in self.measurements:
            if measurement.name == name:
                return measurement
        return None

    def to_dict(self):
        record = {
            "name": self.name,
            "bus": self.bus,
            "motor_id": self.motor_id,
            "actuator_identity": self.actuator_identity,
            "actuator_identity_source": self.actuator_identity_source,
            "mapping_source": self.mapping_source,
            "mapping_status": self.mapping_status,
            "encoder": self.encoder,
            "mapping": {
                "reduction": self.mapping.reduction,
                "direction": self.mapping.direction,
                "zero_output_rad": self.mapping.zero_output_rad,
            },
        }
        if self.position_limits is not None:
            record["position_limits"] = {
                "minimum_rad": self.position_limits.minimum_rad,
                "maximum_rad": self.position_limits.maximum_rad,
            }
        if self.measurements:
            record["measurements"] = {
                measurement.name: measurement.to_dict() for measurement in self.measurements
            }
        return record


@dataclass(frozen=True)
class LockedJoint:
    name: str
    mass: Measurement
    position: Measurement

    def __post_init__(self):
        _text(self.name, "locked joint name")
        for field, label, units in ((self.mass, 'body_mass', 'kg'),
                                    (self.position, 'locked_angle', 'rad')):
            if not isinstance(field, Measurement) or field.name != label or field.units != units:
                raise ValueError(f'locked joint {label} must be a {units} Measurement')
        if self.mass.value is not None and self.mass.value < 0:
            raise ValueError('locked body mass must be nonnegative when known')

    def to_dict(self):
        return {'name': self.name, 'mass': self.mass.to_dict(), 'position': self.position.to_dict()}


@dataclass(frozen=True)
class BuildProfile:
    schema_version: int
    robot: str
    build: str
    backend: str
    revision: str
    joints: tuple[JointEntry, ...] = ()
    locked_joints: tuple[LockedJoint, ...] = ()

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version not in SUPPORTED_SCHEMA_VERSIONS:
            raise ValueError("schema_version must be a supported version")
        _text(self.robot, "robot")
        _text(self.build, "build")
        if self.backend not in BACKENDS:
            raise ValueError("backend must be physical, simulation or demo")
        _text(self.revision, "revision")
        if type(self.joints) is not tuple or any(
            not isinstance(joint, JointEntry) for joint in self.joints
        ):
            raise ValueError("joints must be a tuple of JointEntry")
        names = [joint.name for joint in self.joints]
        if len(names) != len(set(names)):
            raise ValueError("joint names must be unique")
        slots = [(joint.bus, joint.motor_id) for joint in self.joints]
        if len(slots) != len(set(slots)):
            raise ValueError("motor ids must be unique per bus")

        if type(self.locked_joints) is not tuple or any(
            not isinstance(joint, LockedJoint) for joint in self.locked_joints
        ):
            raise ValueError('locked_joints must be a tuple of LockedJoint')
        locked_names = [joint.name for joint in self.locked_joints]
        if len(locked_names) != len(set(locked_names)):
            raise ValueError('locked joint names must be unique')
        if set(names) & set(locked_names):
            raise ValueError('a joint cannot be both active and locked')

    def require_active_joint(self, name: str) -> JointEntry:
        _text(name, 'requested joint name')
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise ValueError(f'joint {name} is absent or locked in this build')

    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints)

    def to_dict(self):
        record = {
            "schema_version": self.schema_version,
            "robot": self.robot,
            "build": self.build,
            "backend": self.backend,
            "revision": self.revision,
            "joints": [joint.to_dict() for joint in self.joints],
        }
        if self.locked_joints:
            record['locked_joints'] = [joint.to_dict() for joint in self.locked_joints]
        return record

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, allow_nan=False)


def _parse_limits(record):
    raw = _object(record["position_limits"], "position_limits")
    _reject_unknown_fields(raw, LIMIT_FIELDS, "position_limits")
    _require_keys(raw, LIMIT_FIELDS, "position_limits")
    return PositionLimits(minimum_rad=raw["minimum_rad"], maximum_rad=raw["maximum_rad"])


def _parse_measurement(name, value):
    entry = _object(value, f'measurement {name}')
    _reject_unknown_fields(entry, MEASUREMENT_FIELDS, f'measurement {name}')
    _require_keys(entry, ('status', 'units', 'source'), f'measurement {name}')
    return Measurement(name=name, status=entry['status'], units=entry['units'],
                       source=entry['source'], value=entry.get('value'))


def _parse_measurements(record):
    raw = _object(record['measurements'], 'measurements')
    return tuple(_parse_measurement(name, value) for name, value in raw.items())


def _parse_locked_joint(record):
    _object(record, 'locked joint')
    fields = ('name', 'mass', 'position')
    _reject_unknown_fields(record, fields, 'locked joint')
    _require_keys(record, fields, 'locked joint')
    return LockedJoint(record['name'], _parse_measurement('body_mass', record['mass']),
                       _parse_measurement('locked_angle', record['position']))


def _parse_joint(record):
    _reject_unknown_fields(record, JOINT_FIELDS, "joint")
    _require_keys(record, REQUIRED_JOINT_FIELDS, "joint")
    raw_mapping = _object(record["mapping"], "mapping")
    _reject_unknown_fields(raw_mapping, MAPPING_FIELDS, "mapping")
    _require_keys(raw_mapping, MAPPING_FIELDS, "mapping")
    mapping = JointMap(
        reduction=raw_mapping["reduction"],
        direction=raw_mapping["direction"],
        zero_output_rad=raw_mapping["zero_output_rad"],
    )
    parsed = JointEntry(
        name=record["name"],
        bus=record["bus"],
        motor_id=record["motor_id"],
        actuator_identity=record["actuator_identity"],
        actuator_identity_source=record["actuator_identity_source"],
        mapping_source=record["mapping_source"],
        mapping_status=record["mapping_status"],
        encoder=record["encoder"],
        mapping=mapping,
        position_limits=_parse_limits(record) if "position_limits" in record else None,
        measurements=_parse_measurements(record) if "measurements" in record else (),
    )
    return parsed


def from_dict(raw):
    _object(raw, "build profile")
    _reject_unknown_fields(raw, TOP_LEVEL_FIELDS, "build profile")
    _require_keys(raw, REQUIRED_TOP_LEVEL_FIELDS, "build profile")
    raw_joints = raw["joints"]
    if not isinstance(raw_joints, list):
        raise ValueError("joints must be a JSON array")
    joints = tuple(_parse_joint(_object(joint, f"joints[{index}]")) for index, joint in enumerate(raw_joints))
    raw_locked = raw.get('locked_joints', [])
    if not isinstance(raw_locked, list):
        raise ValueError('locked_joints must be a JSON array')
    return BuildProfile(
        schema_version=raw["schema_version"],
        robot=raw["robot"],
        build=raw["build"],
        backend=raw["backend"],
        revision=raw["revision"],
        joints=joints,
        locked_joints=tuple(_parse_locked_joint(record) for record in raw_locked),
    )


def parse_json(text):
    if not isinstance(text, str):
        raise ValueError("profile JSON must be a string")

    def reject_constant(token):
        raise ValueError(f"profile JSON contains the non-finite token {token}")

    def unique_object(pairs):
        record = {}
        for name, value in pairs:
            if name in record:
                raise ValueError(f"profile JSON repeats field {name}")
            record[name] = value
        return record

    try:
        raw = json.loads(text, parse_constant=reject_constant, object_pairs_hook=unique_object)
    except json.JSONDecodeError as exc:
        raise ValueError(f"profile JSON is malformed: {exc}") from exc
    return from_dict(raw)

```

FILE calibration_binding.py
```
from dataclasses import dataclass
import hashlib
import re

from build_profile import BuildProfile


def _profile_digest(profile: BuildProfile) -> str:
    if not isinstance(profile, BuildProfile):
        raise ValueError('profile must be a BuildProfile')
    return hashlib.sha256(profile.to_json().encode('utf-8')).hexdigest()


def _artifact_digest(artifact: bytes) -> str:
    if type(artifact) is not bytes or not artifact:
        raise ValueError('calibration artifact must be non-empty immutable bytes')
    return hashlib.sha256(artifact).hexdigest()


@dataclass(frozen=True)
class CalibrationBinding:
    calibration_id: str
    source: str
    profile_sha256: str
    artifact_sha256: str

    def __post_init__(self):
        for value in (self.calibration_id, self.source):
            if type(value) is not str or not value.strip() or value != value.strip():
                raise ValueError('calibration ID and source must be non-empty unpadded strings')
        for digest in (self.profile_sha256, self.artifact_sha256):
            if type(digest) is not str or re.fullmatch('[0-9a-f]{64}', digest) is None:
                raise ValueError('calibration binding digests must be lowercase SHA256 hex strings')

    def require_match(self, profile: BuildProfile, artifact: bytes) -> None:
        if self.profile_sha256 != _profile_digest(profile):
            raise ValueError('calibration belongs to a different build revision or configuration')
        if self.artifact_sha256 != _artifact_digest(artifact):
            raise ValueError('calibration artifact content differs from its binding')


def bind_calibration(profile: BuildProfile, artifact: bytes, *,
                     calibration_id: str, source: str) -> CalibrationBinding:
    return CalibrationBinding(calibration_id, source, _profile_digest(profile), _artifact_digest(artifact))

```
