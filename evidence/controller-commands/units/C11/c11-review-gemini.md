# Independent Gemini Code Review: Milestone C11 (Acceleration Limiting)

**Reviewer**: Gemini 3.8 Flash High (Independent Code Review Seat, `antigravity:gemini-3.8-flash-high@default`)  
**Base Revision**: `30606aa07d01f78fb09da8a44f9e38f51dfde6e9`  
**Date**: October 15, 2026  
**Scope**:
- `joint_records.py` (`JointSpec` acceleration configuration)
- `joint_reference.py` (Pure planner, projection, and release-rate math)
- `joint_controller.py` (`JointCore` reference history, lifecycle, and release integration)
- `test_controller_acceleration.py` (New 17-test C11 acceleration test suite)
- Existing test suites regression (`test_controller.py`, `test_controller_targets.py`, `test_controller_limits.py`, `test_controller_velocity.py`, `test_controller_ownership.py`, `test_startup.py`)
- Baseline and startup inventories (`scripts/check_offline_baseline.sh`, `scripts/check_milestone.py`, `test_startup.py`)
- Documentation updates (`docs/architecture.md`, `docs/controller-design.md`, `docs/joint-api.md`)

---

## 1. Architectural & Semantic Contract (S2)

1. **Dual-Corridor Bounding**:
   - The implementation adheres strictly to candidate S2. It independently bounds both:
     1. Command feedforward velocity field `dq_rad_s` slew: $|dq_{\text{cmd}} - d_0| \le A \cdot \min(e, H)$.
     2. Position finite-difference rate: $w_{\text{new}} = (q - q_0)/E$, bounded by $|w_{\text{new}}| \le V$ and $|w_{\text{new}} - w_0| \le A \cdot E$.
   - The caller's requested `dq_rad_s` setpoint is never overwritten with the position derivative $w$. The independent semantics of $q$ (position setpoint to position loop) and $dq$ (velocity feedforward to damping loop) are preserved.

2. **Seeding and Lifecycle Rest**:
   - On explicit `arm()`, `JointCore` seeds history with `JointTarget(joint.sample.q_rad, 0.0, 0.0, 0.0, 0.0)` and `position_rates_rad_s = (0.0, ...)`.
   - Seeding checks `Stop` mode in motor feedback, not physical rest. The zero-speed seed is an internal reference trajectory invariant.
   - On `stop()`, `fault()`, `disconnect()`, or `rearm()`, reference history is cleared to `None`, requiring an explicit `arm()` to seed new references.

3. **Pure Module Boundary**:
   - `joint_reference.py` is strictly stateless and transport-free. It contains no clocks, no mutable attributes, and no device communications.
   - All codec frame formatting and decoding checks verify roundtrip fidelity: normal command encoding must not alter a projected grid code (`decoded["raw_position"] == raw_position`).

---

## 2. Delay Non-Conservatism & Final Release Rate Veto

1. **Mathematical Necessity**:
   - In prototyping, delay between planning ($t_{\text{plan}}$) and final dispatch ($t_{\text{release}}$) was proven non-conservative: if $t_{\text{release}} - t_{\text{plan}} > 0$, an earlier planned deceleration displacement $(q_{\text{cand}} - q_0) / (t_{\text{release}} - t_0)$ produces a smaller average velocity than planned, which during deceleration can increase the acceleration magnitude $|w_{\text{new}} - w_0| / E > A$.
   - `JointCore.consume_target()` addresses this by invoking `position_rates_at_release()` at $t_{\text{release}}$.

2. **Atomic Veto**:
   - If any single joint violates $|w_{\text{new}}| \le V$ or $|w_{\text{new}} - w_0| \le A \cdot E$, `position_rates_at_release` returns `None`.
   - On `None`, `JointCore` commits no state or history, retains `_pending_target`, and returns `None`.
   - When called again on a subsequent tick with accumulated elapsed time $E$, the candidate can be successfully replanned or re-evaluated.

3. **Uncapped Elapsed for Position Rates**:
   - Feedforward slew uses $h = \min(e, H)$ to prevent burst commands after idle.
   - Position rate checks use uncapped $E = t_{\text{release}} - t_0$, allowing the allowable rate envelope $[-A \cdot E, +A \cdot E]$ to expand over long idle periods, ensuring that holding stationary position ($w_{\text{new}} = 0$) remains mathematically feasible.

---

## 3. Quantization & Reset Starvation Prevention

1. **Subgrid Quantization Check**:
   - When a requested goal represents a different grid code from previous reference (`static_pos_raw != prev_pos_raw`), but the acceleration/velocity corridor is too narrow to reach an adjacent grid step (`cand_pos_raw == prev_pos_raw`), `plan_reference` returns `None`.
   - This suppresses the entire vector, preserving `_pending_target` and `_history.issued_at_s` so that elapsed time accumulates across ticks until an adjacent representable code becomes feasible.

2. **Bootstrap Waiver**:
   - On the very first tick after `arm()` (`_history.kind == "seed"`), `allow_bootstrap=True` permits a subgrid stationary hold to establish the initial issued reference.
   - Once issued, `kind` transitions to `"issued"` and subsequent ticks require progress whenever static goals differ.

3. **Settled Goals & Gain-Only Changes**:
   - When static goals match previous issued codes (`static_pos_raw == prev_pos_raw` and `static_speed_raw == prev_speed_raw`), the channel is classified as settled.
   - Gain-only updates (`kp`, `kd`, `tau`) to settled channels pass through without suppression.

---

## 4. Edge Cases & Safety Verifications

1. **Arithmetic Overflow Suppression**:
   - In `plan_reference()` and `position_rates_at_release()`, all multiplications ($A \cdot h$, $A \cdot E$, $V \cdot h$, $E \cdot w$) and additions are wrapped in arithmetic exception checks and `math.isfinite()` guards.
   - Immense finite accelerations (e.g., $10^{308}$) that overflow on multiplication cleanly return `None` rather than raising uncaught exceptions or corrupting state.

2. **Zero Elapsed**:
   - When $E = 0.0$ (same timestamp), `position_rates_at_release` requires exact equality of positions and speeds (`cand.q_rad == prev.q_rad` and `cand.dq_rad_s == prev.dq_rad_s`), returning `previous_rates`. No division by zero occurs.

3. **Reversals**:
   - Full reversals from positive to negative velocity limits are constrained across multiple ticks by the acceleration ramp, both in feedforward `dq` and position finite-difference rates.

4. **Multi-Joint Coupling**:
   - If one joint (e.g. slow axis) cannot make progress or fails the release rate check, the entire vector is suppressed atomically. A fast axis cannot advance independently and reset the time base for the slow axis.

5. **No Outward Tolerances**:
   - No epsilon or outward slop is added to the strict physical bounds:
     `abs(w_new) <= V` and `delta_w <= A_E`.
   - Unit tests verify strict boundaries without outward tolerance allowances.

---

## 5. Verification Evidence Summary

- **Controller Test Suites**: 95 tests passed in 1.155s (`controllers-final.log`).
- **Named Milestone Gate**: C11 passed (`receipt.json`, 17 tests).
- **Probes**: 24 randomized trials over 3,360 ticks with seed 14011 passed (`probes-integrated.log`: 2,352 issued vectors, 7,056 changed channels, 1,008 suppressed attempts, 0 outward tolerance, off-grid bootstrap rate preserved).
- **Full Host Offline Baseline**: All 9 test groups passed exit 0 (`c11-baseline-host/run-20261015T022706Z-QHcHWoUK/checks.tsv`).
- **Visual Evidence**: Independent visual inspection of 4 screenshots confirmed safe disconnected startup and simulated demo operation with zero physical I/O.

---

## 6. Review Findings & Verdict

- **Architecture**: APPROVED.
- **Implementation**: APPROVED.
- **Contract Adherence**: APPROVED.
- **Acceptance Recommendation**: Milestone C11 is fully verified in software and ready for parent acceptance and ledger recording.
