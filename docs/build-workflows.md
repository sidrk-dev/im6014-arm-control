# Bench and mounted-base configurations

`build_profiles.bench_j1(...)` and `arm_j1(...)` prepare different immutable `BuildPlan` records. Both require explicit configuration inputs. There is no default motor ID, reduction, direction or physical zero.

```python
from build_profiles import Workflow, arm_j1, bench_j1
from joint_mapping import JointMap

inputs = dict(
    revision='synthetic-build-1', bus='synthetic-bus', motor_id=0,
    mapping=JointMap(1, 1, 0), mapping_source='synthetic example only',
    actuator_identity=None, actuator_identity_source='not identified in this example',
)
bench = bench_j1(**inputs)
mounted = arm_j1(**inputs)
assert bench.profile.build != mounted.profile.build
assert Workflow.BOUNDED_BENCH_TEST in bench.definition.actions_when_commissioned
assert Workflow.ARM_JOINT_CONTROL in mounted.definition.actions_when_commissioned
assert bench.profile.joints[0].mapping_status == 'declared'
```

The zero in this example is synthetic configuration. Loading it never restores a trusted motor reference.

| Definition | Intended setup | Distinct required evidence | Workflow scope after commissioning |
|---|---|---|---|
| `bench_j1` | Secured bench actuator under verification | Bench securement and clearance | Inspection, calibration, bounded bench tests |
| `arm_j1` | Mounted base configuration | Verified base mounting, attached load and arm clearance | Inspection, calibration, arm joint control |

Both also require inspected adapter/wiring, verified actuator identity and a current session reference. These are evidence requirements, not declarations that the facts are true. No named build is automatically ready to move. Mapping provenance stays `declared`, and unknown actuator identity stays `None`.

`actions_when_commissioned` describes a conditional workflow ceiling. It does not expose low-level `arm`, `release`, Enable or motor writes. `BuildPlan` accepts only the canonical workflow definitions and a matching D01 build identity with exactly J1; bench and mounted profiles cannot be swapped. Definition scopes and evidence sets are immutable.

This software unit adds no UI checks, owner state machine, live calibration path or movement authority. Connect/Enable, current feedback, faults and reference checks remain with the existing owner. The definition metadata is canonical Python data; its D01 `profile` alone serializes to JSON. Do not infer live readiness from that serialized build name. D03-D06 and actual owner integration remain later work.
