### Authored Files
- [`joint_contract.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/joint_contract.py): Added `DecisionCode.LEAD_OUT_OF_BOUNDS` and required finite positive `max_lead_rad: float` attribute to [`JointSpec`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/joint_contract.py#L84-L144) with validation rejecting non-numeric, boolean, non-positive, infinite, or NaN values.
- [`joint_reference.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/joint_reference.py): Pure reference shaping mathematics updated:
  - [`validate_target_vector`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/joint_reference.py#L179-L216) checks $q_{\text{meas}} - \text{lead} \le q_{\text{req}} \le q_{\text{meas}} + \text{lead}$, returning `LEAD_OUT_OF_BOUNDS` on violation.
  - [`shape_reference_joint`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/joint_reference.py#L219-L547) forms a four-way closed interval intersection: speed $[-V, V]$, acceleration $[v_{\text{prev}}-Ah, v_{\text{prev}}+Ah]$, position $[(q_{\min}-q_{\text{prev}})/h, (q_{\max}-q_{\text{prev}})/h]$, and lead $[(q_{\text{meas}}-\text{lead}-q_{\text{prev}})/h, (q_{\text{meas}}+\text{lead}-q_{\text{prev}})/h]$. Raw position codec grid candidates and decoded endpoints are bounded within the lead interval.
  - [`verify_dispatch_claim`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/joint_reference.py#L600-L687) verifies all closed constraints, rational derivative consistency, and fresh measured sample lead bounds at write gate claim time.
- [`joint_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/joint_controller.py):
  - Updated `submit_target` to verify fresh feedback and pass measured joint positions to `validate_target_vector`.
  - Updated `prepare_refresh` to supply fresh measured positions.
  - Updated `claim_dispatch` to delegate pre-write gate claim verification to `verify_dispatch_claim`, latching appropriate joint fault and stopping motion if any constraint or lead bound is violated by post-preparation feedback drift.
  - Kept core line count at 902 lines (<1000 limit).
- [`test_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/test_controller.py), [`test_controller_expiry.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/test_controller_expiry.py), [`test_controller_owner.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/test_controller_owner.py), [`test_controller_targets.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/test_controller_targets.py): Migrated all `JointSpec` instantiations to provide required finite positive `max_lead_rad`.
- [`test_controller_limits.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/test_controller_limits.py): Migrated existing tests; added `max_lead_rad` validation cases to `test_malformed_spec_and_fault_config_rejected`; passed `q_meas` to existing direct `shape_reference_joint` calls; authored complete `ReferenceLeadTests` suite.
- [`scripts/check_milestone.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/scripts/check_milestone.py): Registered `'C12': 'test_controller_limits.ReferenceLeadTests'`.
- [`docs/joint-api.md`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C12-workbench/docs/joint-api.md): Documented `max_lead_rad`, four-way reference intersection, and pre-write claim gate lead checks; updated code example.

### Key Decisions
- Emitted target remains authoritative; internal `position_step_velocity` is exact rational $(q_{\text{emitted,next}} - q_{\text{emitted,prev}})/h$, independent of encoded command velocity history.
- Pre-write claim verification math lives in `joint_reference.py`, preserving controller encapsulation and keeping `joint_controller.py` well under 1000 lines.
- Target admission rejects out-of-lead requests with `DecisionCode.LEAD_OUT_OF_BOUNDS` without disturbing valid existing goals or sequence watermarks.
- Empty interval intersection in preparation fails closed, latching `constraint_fault` and stopping motion without independent coordinate clamping.

### Recommended Parent Checks
- `C12`
- `milestone-tests`
- `scripts/check_offline_baseline.sh`

### Contradictions
None identified.

```pstack-handoff
{"task":"C12","checkpoint":"C12-lead-bound","operation":"run-checks","files":["joint_contract.py","joint_reference.py","joint_controller.py","test_controller.py","test_controller_expiry.py","test_controller_owner.py","test_controller_targets.py","test_controller_limits.py","scripts/check_milestone.py","docs/joint-api.md"],"checks":["C12","milestone-tests","scripts/check_offline_baseline.sh"],"summary":"Parent verification of C12 reference lead bound checks and offline baseline"}
```