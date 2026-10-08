### Authored Files
1. [`joint_contract.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/joint_contract.py): Added required finite positive real `a_max_rad_s2` attribute to [`JointSpec`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/joint_contract.py#L82) with strict type and finiteness validation.
2. [`joint_reference.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/joint_reference.py): Updated [`shape_reference_joint`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/joint_reference.py#L193) for $h > 0$ to intersect $[-V, V]$, $[(q_{\min}-q_{\text{prev}})/h, (q_{\max}-q_{\text{prev}})/h]$, and $[v_{\text{prev}}-A\cdot h, v_{\text{prev}}+A\cdot h]$ using exact rational arithmetic (`Fraction`) over stored finite float domain values. Applied independent speed and acceleration intervals to position-step velocity and encoded wire velocity streams, with inward projection step adjustments and exact post-encoding bound validation. Nonzero objectives producing no representable position step fail closed with `DecisionCode.CONSTRAINT_VIOLATION`.
3. [`joint_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/joint_controller.py): Extended the [`claim_dispatch`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/joint_controller.py#L742) write gate to recheck acceleration bounds $|v - v_{\text{prev}}| \le A\cdot h$ for both streams, latching `constraint_fault=True` and stopping on violation. Core line count remains 927 (<1000 lines).
4. [`docs/joint-api.md`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/docs/joint-api.md): Updated API documentation and fixture example for required `a_max_rad_s2`.
5. [`scripts/check_milestone.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/scripts/check_milestone.py): Registered `'C11': 'test_controller_limits.ReferenceAccelerationTests'`.
6. [`test_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/test_controller.py), [`test_controller_expiry.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/test_controller_expiry.py), [`test_controller_owner.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/test_controller_owner.py), [`test_controller_targets.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/test_controller_targets.py): Migrated all [`JointSpec`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/joint_contract.py#L82) callers to supply positive fixture limits.
7. [`test_controller_limits.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/test_controller_limits.py): Migrated existing [`JointSpec`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/joint_contract.py#L82) calls and malformed spec tests, and authored [`ReferenceAccelerationTests`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C11-workbench/test_controller_limits.py#L1648).

### Key Decisions
- **Quantization separation**: Maintained pure mathematical position-step velocity as exact rational `(q_next - q_prev)/h` and tracked wire velocity feedforward independently. Neither substitutes for the other.
- **Incompatible bounds fail closed**: When position limits and acceleration limits form an empty intersection ($(q_{\max} - q_{\text{prev}})/h < v_{\text{prev}} - A\cdot h$), motion withdraws and latches `constraint_fault` without clamping position or jumping.
- **Velocity reversal**: Smoothly reverses through zero under $A\cdot h$ without resetting $v_{\text{prev}}$ or relaxing bounds.
- **Deliberate hold vs. quantization stall**: Zero velocity objective with $q_{\text{goal}} == q_{\text{prev}}$ produces valid hold; nonzero objective unable to produce a representable step under closed constraints triggers `CONSTRAINT_VIOLATION`.

### Contradictions and Resolutions
- Resolved the conflict between earlier descriptions equating reference and encoded velocity by enforcing independent history and bounds for both streams, per mandatory updated quantization semantics.

### Recommended Parent Checks
Command execution tools were unavailable in this worker session; no checks have been run locally. Recommended parent checks:
- `C11`: Audited check of `test_controller_limits.ReferenceAccelerationTests`.
- `milestone-tests`: Full regression suite across controllers and limits.
- `scripts/check_offline_baseline.sh`: Offline clean copy verification script.

```pstack-handoff
{"task":"C11","checkpoint":"C11-acceleration-reference","operation":"run-checks","files":["joint_contract.py","joint_reference.py","joint_controller.py","docs/joint-api.md","scripts/check_milestone.py","test_controller.py","test_controller_expiry.py","test_controller_owner.py","test_controller_targets.py","test_controller_limits.py"],"checks":["C11","milestone-tests","scripts/check_offline_baseline.sh"],"summary":"Run C11 milestone unit check, milestone-tests suite, and offline baseline verification script"}
```