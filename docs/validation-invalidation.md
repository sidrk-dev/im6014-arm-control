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
