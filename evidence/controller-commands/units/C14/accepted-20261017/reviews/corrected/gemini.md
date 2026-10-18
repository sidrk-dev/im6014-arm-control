# Independent Source Review: C14 Workbench Protection Implementation (Corrected Artifact)

**Workspace Inspected:** `/tmp/c14-codex-fixed-20261017`  
**Verdict:** **Approved (Explicitly No Findings / Clean)**

---

### Scope & Verified Files
An exhaustive read-only inspection was performed across the pure-controller implementation and related contracts:
- [`joint_controller.py`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py)
- [`joint_records.py`](file:///tmp/c14-codex-fixed-20261017/joint_records.py)
- [`joint_mapping.py`](file:///tmp/c14-codex-fixed-20261017/joint_mapping.py)
- [`test_controller_faults.py`](file:///tmp/c14-codex-fixed-20261017/test_controller_faults.py)
- Documentation & baselines: [`docs/joint-api.md`](file:///tmp/c14-codex-fixed-20261017/docs/joint-api.md), [`docs/controller-design.md`](file:///tmp/c14-codex-fixed-20261017/docs/controller-design.md), [`scripts/check_milestone.py`](file:///tmp/c14-codex-fixed-20261017/scripts/check_milestone.py), and [`scripts/check_offline_baseline.sh`](file:///tmp/c14-codex-fixed-20261017/scripts/check_offline_baseline.sh).

---

### Analysis by Verification Area

#### 1. Multi-Joint Fault Accumulation & Recovery
- **Per-Joint Exposure Isolation:** In [`JointCore`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L63), each joint maintains its own isolated [`_ProtectionHistory`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L54-L60) indexed by configured position. Timestamps, overload accumulation (`overload_s`), and recovery duration (`recovery_s`) are computed independently against each joint's own `received_at_s` anchor.
- **Cause Accumulation:** [`_latch_joint_fault()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L201-L236) merges causes into [`JointFault`](file:///tmp/c14-codex-fixed-20261017/joint_records.py#L198) by OR-accumulation (`errors | errors`, `warnings | warnings`, boolean OR for latched flags), keeping causes keyed per joint name in `_faults`.
- **Multi-Joint Recovery Evaluation:** In [`checked()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L365-L387), lines 379–384 iterate across all configured joints; any joint with an active protection cause (`over_temperature`, `persistent_effort`, or `protection_unavailable`) blocks transition with [`DecisionCode.RECOVERY_PENDING`](file:///tmp/c14-codex-fixed-20261017/joint_records.py#L51) until that joint's own `recovery_s >= recovery_duration_s`. `_faults.clear()` is called only when every joint completes verification.

#### 2. Unknown Channels, Errors, & Sampling Gaps
- **Dual-Channel Thermal Fail-Closed:** [`MotorHealth`](file:///tmp/c14-codex-fixed-20261017/joint_records.py#L254) requires explicit finite `driver_c` and `winding_c` (or `None`). If either channel is `None`, [`MotorHealth.healthy`](file:///tmp/c14-codex-fixed-20261017/joint_records.py#L292) evaluates to `False`, and [`_observe_protection()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L157-L188) marks `unknown = True`, reliably latching `protection_unavailable`.
- **Additive Causes:** If one channel is hot (`>= driver_limit_c` or `>= winding_limit_c`) while the other is `None`, both `over_temperature` and `protection_unavailable` latch simultaneously in [`_latch_joint_fault()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L326-L335).
- **Observation Gaps:** In [`_observe_protection()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L172-L176), if `gap > min(feedback_max_age_s, policy.max_sample_gap_s)`, `contiguous` is `False` and `dt = Fraction(0)`. This freezes and preserves confirmed `overload_s` without incrementing it, while strictly resetting contiguous `recovery_s` to `Fraction(0)`.
- **Transport Failure Continuity Break:** In [`communication_failed()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L338-L364), [`_break_protection_continuity()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L151-L156) resets `received_at_s` and `recovering` while retaining `overload_s`.

#### 3. Time Boundaries & Monotonic Accounting
- **Exact Rational Arithmetic:** All interval accounting uses `Fraction` (`gap = Fraction(now) - Fraction(prior.received_at_s)`). Interval additions and threshold comparisons (`overload_s >= Fraction(...)` and `recovery_s >= Fraction(...)`) avoid floating-point rounding errors and boundary drift.
- **Identical Timestamps:** When `now == prior.received_at_s`, `gap = Fraction(0)` and `dt = Fraction(0)`, preventing false exposure accumulation on repeated samples at the same timestamp.
- **Clock Monotonicity:** [`_now()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L237-L250) enforces monotonicity and finiteness before mutating any controller state or protection history.

#### 4. State Transitions & Acknowledgment Bypasses
- **Stop & Check Continuity Retention:** In [`begin_check()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L284-L300), `_ProtectionHistory` retains `received_at_s`, `overloaded`, and `overload_s`, resetting only `recovering = False` and `recovery_s = Fraction(0)`. [`stop()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L632-L642) does not reset `_protection`. This completely eliminates the previous defect where repeated software `stop()` or `begin_check()` calls suppressed overload accumulation.
- **Explicit Acknowledgment:** When `bool(self._faults)` is true, [`begin_check()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L289-L290) returns `DecisionCode.FAULT_ACK_REQUIRED` unless `acknowledge_fault=True`.
- **Stopped-Mode Recovery Enforcement:** During `CHECKING`, [`_observe_protection()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L167-L169) only allows `recovering = True` if `health.mode == 0`. Mode 1 observations reset `recovery_s = Fraction(0)`.
- **Arming Gates:** [`arm()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L389-L445) verifies `not bool(self._faults)`, `_protection_healthy()`, `joint.health.mode == 0`, and fresh feedback across all joints before minting an [`Ownership`](file:///tmp/c14-codex-fixed-20261017/joint_records.py#L75) grant.

#### 5. Atomicity & Invariant Integrity
- All validation in [`receive()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L301-L336), [`arm()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L389-L445), and [`consume_target()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L531-L630) precedes state commits.
- Clock failures abort atomically before observations or protection histories are updated.

#### 6. C13 Compatibility
- Ownership generation watermarks and capability checks in [`Ownership`](file:///tmp/c14-codex-fixed-20261017/joint_records.py#L75) and [`_revoke_ownership()`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py#L143-L146) preserve C13 invariants.
- Target sequence tracking via `_last_sequence` remains strictly increasing and monotonic.
- Kinematic reference corridor planning in [`plan_reference()`](file:///tmp/c14-codex-fixed-20261017/joint_reference.py) and effort pacing remain unchanged.

#### 7. Added Commentary & Documentation Audit
- [`joint_controller.py`](file:///tmp/c14-codex-fixed-20261017/joint_controller.py), [`joint_records.py`](file:///tmp/c14-codex-fixed-20261017/joint_records.py), and [`test_controller_faults.py`](file:///tmp/c14-codex-fixed-20261017/test_controller_faults.py) contain zero inline comment syntax (`#`).
- The narrative comment previously flagged in `test_controller_faults.py:14` has been removed.
- Imports in test files have been cleanly folded into existing `joint_records` import tuples.
- Documentation in [`docs/joint-api.md`](file:///tmp/c14-codex-fixed-20261017/docs/joint-api.md#L138-L160) and [`docs/controller-design.md`](file:///tmp/c14-codex-fixed-20261017/docs/controller-design.md#L74-L83) accurately reflects the dual-channel `driver_c` / `winding_c` contract, `ProtectionPolicy`, and exact duration accounting.

---

### Conclusion
The corrected artifact completely resolves the Stop/Check continuity bypasses and satisfies all functional, safety, and architectural constraints. No correctness bugs, bypasses, regressions, or documentation discrepancies were found.