# Absent and installed locked joints

D04 keeps active coordinates in `BuildProfile.joints` and installed locked joints in a separate immutable `locked_joints` tuple. An absent joint is omitted from both collections. Active and locked names cannot overlap or repeat. A locked joint has no motor ID, bus or mapping, so it cannot be used as an active `JointSpec`.

Every locked joint explicitly records its aggregate attached-body mass (including whichever motor/link bodies are assigned to that joint) and fixed angle as `Measurement` objects. Mass uses kg and cannot be negative when known; angle uses rad. Each record carries declared/measured/unknown status and source. Unknowns have no numeric value. The profile retains locked mass through JSON round-trip; it never zero-fills absent joints, invents passive masses or adds locked coordinates to the active vector. Avoid double-counting bodies when providing aggregate masses.

```python
from dataclasses import replace
from build_profile import LockedJoint, Measurement
from build_profiles import BuildPlan, arm_j1
from joint_mapping import JointMap

# Synthetic example only; no mechanical installation or calibration is asserted.
plan = arm_j1(revision='example-locked-1', bus='synthetic-bus', motor_id=0,
    mapping=JointMap(1, 1, 0), mapping_source='synthetic example',
    actuator_identity=None, actuator_identity_source='not verified')
locked = LockedJoint('J2',
    Measurement('body_mass', 'unknown', 'kg', 'not weighed for this revision'),
    Measurement('locked_angle', 'unknown', 'rad', 'not measured for this revision'))
configured = replace(plan.profile, locked_joints=(locked,))
plan = BuildPlan(plan.definition, configured)
assert plan.profile.joint_names() == ('J1',)
assert plan.profile.locked_joints[0].mass.value is None
```

`require_active_joint(name)` selects configuration only. It rejects locked and absent names; it grants no readiness or motion permission. The pure Core uses positional targets, not named requests: callers must use the exact active order and must not silently truncate extra coordinates. The fake test constructs its Core only from active entries and proves that an extra coordinate is rejected with no issued reference or write. A locked record itself is rejected as a Core spec.

This is a software configuration boundary. It neither detects a physical lock nor commands one, and does not integrate a new adapter into the current dashboard owner. Full geometry, mass transforms, inertias and gravity compensation remain later dynamics work. D05 calibration binding and D06 validation invalidation are separate. Hardware I/O stays blocked after the adapter incident.

Old schema-version-1 JSON without `locked_joints` still parses; empty locked tuples are omitted when serialized. New locked fields reject unknown keys, missing measurements and any attempted motor/bus/mapping fields.
