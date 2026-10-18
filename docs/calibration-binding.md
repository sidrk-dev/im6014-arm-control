# Bind a calibration artifact to its build

D05 binds an exact existing artifact to the full canonical build profile. It records a calibration ID, source, profile SHA256 and artifact SHA256 as immutable metadata. It does not calculate calibration, parse or certify artifact contents, access files, or apply a saved reference.

```python
from build_profiles import arm_j1
from joint_mapping import JointMap
from calibration_binding import bind_calibration

plan = arm_j1(revision='synthetic-revision', bus='synthetic-bus', motor_id=0,
    mapping=JointMap(1, 1, 0), mapping_source='synthetic example',
    actuator_identity=None, actuator_identity_source='unverified')
artifact = b'synthetic calibration artifact only'
binding = bind_calibration(plan.profile, artifact,
    calibration_id='synthetic-calibration-1', source='synthetic fixture')
binding.require_match(plan.profile, artifact)
```

A different revision, build, backend, robot, actuator identity, bus/ID, mapping, measurement provenance, or locked-body mass/angle changes the full-profile hash. Changed artifact bytes also reject, including a whitespace-only edit. Profiles and nonempty immutable artifact bytes must both match. IDs and sources must be explicit, nonempty and unpadded. The record has no persistence parser; any future input loader must validate its own JSON/duplicate-key boundary.

Unknown actuator identities and declared mappings remain unverified. Matching this binding does not mean that calibration values are correct, the hardware matches the declaration, or motion is allowed. The artifact is opaque because its calibration engine/schema is a separate concern. A real owner must retain fresh current-session references, identity/health checks, fault state and explicit enable. There is no apply/restore/arm method and no new owner integration. Reconnecting must not trust a saved absolute zero.

D06 will track affected validation after configuration changes. This unit provides exact artifact/build binding only; it does not selectively invalidate validation records or commission hardware.
