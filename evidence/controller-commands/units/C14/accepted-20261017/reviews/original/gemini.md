### Independent Review: C14 Workbench Protection Implementation

**Inspected Checkout:** [`/tmp/c14-codex-20261017`](file:///tmp/c14-codex-20261017)  
**Primary Files Evaluated:**
- [`joint_controller.py`](file:///tmp/c14-codex-20261017/joint_controller.py)
- [`joint_records.py`](file:///tmp/c14-codex-20261017/joint_records.py)
- [`joint_mapping.py`](file:///tmp/c14-codex-20261017/joint_mapping.py)
- [`joint_reference.py`](file:///tmp/c14-codex-20261017/joint_reference.py)
- [`test_controller_faults.py`](file:///tmp/c14-codex-20261017/test_controller_faults.py)
- [`docs/controller-design.md`](file:///tmp/c14-codex-20261017/docs/controller-design.md)
- [`docs/joint-api.md`](file:///tmp/c14-codex-20261017/docs/joint-api.md)

---

### Detailed Audit by Area

#### 1. Multi-Joint Fault Accumulation and Recovery
- **Isolation:** Each configured joint maintains private, dedicated `_ProtectionHistory` and `JointStatus` instances in `self._protection` and `self._observations`, strictly indexed by `_id_to_index`.
- **Fault Latching:** Latched faults are keyed by unique `spec.name` in `self._faults`. A protection or hardware fault on Joint A does not corrupt or overwrite exposure tracked on Joint B.
- **Recovery Checks:** In [`checked()`](file:///tmp/c14-codex-20261017/joint_controller.py#L359-L381), lines 373–378 iterate across all `self._specs`. If any joint with a latched protection fault (`over_temperature`, `persistent_effort`, or `protection_unavailable`) has `self._protection[i].recovery_s < Fraction(spec.protection.recovery_duration_s)`, `checked()` rejects with `DecisionCode.RECOVERY_PENDING`. Unfaulted joints or joints with purely non-protection causes (e.g. communication timeouts) do not delay recovery or falsely wipe unlatched exposure on other joints.

#### 2. Unknown Temperature, Hardware Errors, and Gaps
- **Fail-Closed Semantics:** [`MotorHealth.temperature_c`](file:///tmp/c14-codex-20261017/joint_records.py#L206-L230) accepts only finite floats or `None`. When `None`, `MotorHealth.healthy` evaluates to `False`, forcing `credible = False` in [`_observe_protection()`](file:///tmp/c14-codex-20261017/joint_controller.py#L158-L188). In [`receive()`](file:///tmp/c14-codex-20261017/joint_controller.py#L295-L330), missing temperature latches `protection_unavailable=True`, transitioning the state machine to `State.FAULT`.
- **Observation Gap Accounting:** Sample time intervals `gap = Fraction(now) - Fraction(prior.received_at_s)` are compared against `min(feedback_max_age_s, policy.max_sample_gap_s)`. If the gap exceeds this bound, or after communication failures/disconnect/stop, continuity is broken (`contiguous = False`, `dt = Fraction(0)`). Existing confirmed `overload_s` is preserved without adding unmeasured time, and `recovery_s` is reset to 0, requiring contiguous healthy samples to regain recovery credit.
- **Errored Samples:** Non-credible samples (`health.healthy == False`) do not contribute time to `overload_s` and immediately reset `recovery_s` to 0.

#### 3. Time Boundaries and Accounting Precision
- **Exact Rational Arithmetic:** Monotonic float timestamps are converted to [`Fraction`](file:///tmp/c14-codex-20261017/joint_controller.py#L173-L184) for interval additions and duration comparisons, preventing cumulative floating-point rounding errors at limit boundaries.
- **Threshold Invariants:**
  - Sustained effort: `abs(sample.tau_nm) > policy.sustained_effort_nm` strictly permits boundary equality (`<=` is non-overload).
  - Temperature: `health.temperature_c >= policy.temperature_limit_c` is inclusive (`<` is non-hot), matching `_protection_healthy()`.
  - Same-timestamp duplicates (`dt == 0`): Do not advance `overload_s` or `recovery_s`.
  - Duration saturation: Counter caps at `policy.overload_duration_s` and `policy.recovery_duration_s` via `min(...)`, ensuring exact duration equality triggers fault latching or clears exposure.

#### 4. State Transitions and Acknowledgment Bypasses
- **Fault Retention:** Latched faults survive target submission, reference suppression, `stop()`, `disconnect()`, reconnecting to a new source, and healthy feedback.
- **Acknowledgment Enforcement:** [`begin_check()`](file:///tmp/c14-codex-20261017/joint_controller.py#L282-L293) strictly requires `acknowledge_fault=True` whenever `bool(self._faults)` is true, returning `DecisionCode.FAULT_ACK_REQUIRED` otherwise.
- **Arming Gates:** [`arm()`](file:///tmp/c14-codex-20261017/joint_controller.py#L383-L440) validates `self._state is State.READY`, `not bool(self._faults)`, complete feedback, fresh feedback, stopped mode (`health.mode == 0`), and `_protection_healthy(i)` for all joints.

#### 5. Atomicity and Exception Safety
- **Clock Regression:** In [`_now()`](file:///tmp/c14-codex-20261017/joint_controller.py#L236-L250), clock regress checks raise `ValueError` before mutating `_observations` or `_protection`.
- **Target Submission and Arming:** Validation occurs prior to mutating internal state (`_pending_target`, `_last_sequence`, `_ownership`, `_history`), leaving core state intact on failure.

#### 6. C13 Compatibility and Reference Dynamics
- Retains reference pacing, reference suppression during quantization waiting, lead bounding, and predicted effort release checks.
- Ownership watermark increments and generation tracking remain intact.

#### 7. Comments and Documentation Review
- **Comments:** Verified that `joint_controller.py` and `joint_records.py` contain zero inline `#` comments.
- **Documentation:** [`docs/controller-design.md`](file:///tmp/c14-codex-20261017/docs/controller-design.md) and [`docs/joint-api.md`](file:///tmp/c14-codex-20261017/docs/joint-api.md) accurately describe the five-parameter `ProtectionPolicy`, `MotorHealth.temperature_c` fail-closed semantics, and contiguous stopped recovery requirements. No misleading claims or unverified physical ratings were introduced.

---

### Conclusion

**Explicitly no findings.** The pure-core software protection implementation adheres strictly to the C14 milestone specifications and preserves all C13 invariants without regressions.