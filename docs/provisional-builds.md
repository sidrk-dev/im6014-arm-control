# Provisional later build profiles

D03 adds pure metadata for `arm_j1_j2` through `arm_j1_j2_j3_j4_j5_j6`, `arm_6_tool`, `sim_6_nominal` and `sim_6_measured`. Their canonical definitions bind the exact ordered joints and physical or simulation backend. Build names describe intended stages; they do not prove installation, calibration, clearance or readiness.

`provisional_build(build_id, revision=..., joints=...)` requires explicit D01 `JointEntry` objects with bus, ID, mapping and identity provenance. It fills missing `body_mass` (kg) and `link_length` (m) with unknown measurements that have no numeric value. It preserves supplied declared/measured values and their sources. No nominal mass, reduction, direction, zero or link length is silently supplied.

```python
from build_profile import JointEntry
from build_profiles import provisional_build
from joint_mapping import JointMap

# Synthetic configuration example; these numbers are not commissioned facts.
joints = tuple(JointEntry(
    name=f'J{index + 1}', bus='synthetic-bus', motor_id=index,
    actuator_identity=None, actuator_identity_source='not verified',
    encoder='unmarked', mapping=JointMap(1, 1, 0),
    mapping_source='synthetic example only', mapping_status='declared',
) for index in range(2))
plan = provisional_build('arm_j1_j2', revision='example-1', joints=joints)
assert plan.profile.joints[0].measurement('body_mass').status == 'unknown'
```

The `sim_6_measured` name targets a later measured simulation workflow. Unknown measurements stay unknown even in that profile; its definition requires assembled geometry and build-revision matching evidence. Neither simulation definition offers a physical control workflow. Physical definitions require link geometry, load, mounting, clearance and current-session reference evidence; `arm_6_tool` additionally requires measured tool/camera configuration. These are requirements, not verified facts or live permissions.

D04 handles absent joints and installed passive bodies. D05 binds calibration to revisions; D06 handles validation invalidation. This unit integrates none of those behaviors into the current runtime owner. No saved zero or movement authority is restored from JSON.
