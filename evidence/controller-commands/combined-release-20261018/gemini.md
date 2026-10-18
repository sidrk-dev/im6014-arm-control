# Independent Review: Combined C11–C16 Controller Continuation

- **Base Revision**: `8e89b75395edadb14d0db2075cd4a62edbd1d1a0` (PR10)
- **Exact Head**: `71a0a5f5123f7b64288b795cc1f39330c89c5fe0` (Private Draft PR11)
- **CI & Verification Status**: 2 green CI checks; 9 baseline groups / 256 milestone tests / 19 legacy Qt dashboard groups / 79 guarded arm tests passing; 4 captures viewed, 39 startup hashes per mode, 0 physical serial ports opened.
- **Physical Boundary**: Physical hardware I/O remains blocked post-adapter burnout; all tests executed against fake transport, PTY fixtures, and deterministic clocks; no physical movement or mechanical stopping time claims are established.
- **Reviewer Provenance**: Retained Muse HTTP 400 dropout is recorded as an invocation failure (not an approval); Gemini provenance pinned argv with `modelVerified: false`; no model substitution.

---

## 1. Architectural Architecture & Module Decomposition

The combined C11–C16 continuation refactors the controller into dedicated, single-responsibility modules while strictly maintaining transport-free execution and isolation from the host session:

1. **[`joint_records.py`](file:///tmp/c16-codex-20261017/joint_records.py)**:
   - Houses all public frozen domain records and enums (`State`, `DecisionCode`, `CommunicationFailure`, `Decision`, `Ownership`, `ArmDecision`, `PositionLimits`, `ProtectionPolicy`, `JointSpec`, `JointFault`, `MotorHealth`, `SourceIdentity`, `JointStatus`, `Target`, `Reference`, `Snapshot`).
   - Validates strict types, non-empty strings, uint bounds, finite positive floats, and value ranges in `__post_init__`. Booleans are strictly rejected for float/int fields.

2. **[`joint_reference.py`](file:///tmp/c16-codex-20261017/joint_reference.py)**:
   - Encapsulates pure mathematical reference planning and codec grid projection (`plan_reference`, `position_rates_at_release`, `project_joint_target`, `compute_predicted_effort`, `_pace_position_toward_effort`).
   - Holds zero mutable state, clocks, lifecycle state, or serial transport.

3. **[`joint_controller.py`](file:///tmp/c16-codex-20261017/joint_controller.py)**:
   - Houses [`JointCore`](file:///tmp/c16-codex-20261017/joint_controller.py#L48-L697) managing single-thread lifecycle, clock monotonicity, observation freshness, latched fault attribution, ownership capabilities, pending targets, and reference history.

4. **[`station_timing.py`](file:///tmp/c16-codex-20261017/station_timing.py)**:
   - Implements bounded, fixed-storage host loop and transaction telemetry ([`StationTiming`](file:///tmp/c16-codex-20261017/station_timing.py#L4-L86)) using `__slots__` with zero growing lists or traces.

5. **[`arm_manual.py`](file:///tmp/c16-codex-20261017/arm_manual.py)** & **[`arm_dashboard.py`](file:///tmp/c16-codex-20261017/arm_dashboard.py)**:
   - Incorporates observational timing telemetry in [`ArmManualSession`](file:///tmp/c16-codex-20261017/arm_manual.py#L245) without altering serial write contracts, motion policies, input leases, or ticket validations.
   - Enforces nested `try...finally` server cleanup in [`run_dashboard`](file:///tmp/c16-codex-20261017/arm_dashboard.py#L707-L732).

---

## 2. Cross-Unit Interactions & Mathematical Verification

### A. Reference Corridor Intersection (C11, C12, C13)
In [`plan_reference`](file:///tmp/c16-codex-20261017/joint_reference.py#L182-L318), reference displacement is bounded by simultaneous intersection across four independent constraints:
- **C09 Position Limits**: $[L, U] = [\text{minimum\_rad}, \text{maximum\_rad}]$
- **C10 Velocity Step**: $[q_0 - V \cdot h, q_0 + V \cdot h]$ where $h = \min(\text{elapsed\_s}, \text{max\_step\_s})$
- **C11 Acceleration Corridor**: $[q_0 + \text{elapsed\_s} \cdot w_{\min}, q_0 + \text{elapsed\_s} \cdot w_{\max}]$ where $w_{\min} = \max(-V, w_0 - A \cdot \text{elapsed\_s})$ and $w_{\max} = \min(V, w_0 + A \cdot \text{elapsed\_s})$
- **C12 Measured Lead Corridor**: $[q_{\text{meas}} - \text{lead}, q_{\text{meas}} + \text{lead}]$

If any corridor is disjoint ($q_{\min} > q_{\max}$ or $dq_{\min} > dq_{\max}$), planning returns `None` cleanly without state corruption or false fault latching.

### B. Predicted Combined Effort & Bounded Pacing (C13)
- In [`compute_predicted_effort`](file:///tmp/c16-codex-20261017/joint_reference.py#L106-L146), decoded command feedforward torque and PD terms are converted from output space to joint space:
  $$\tau_{\text{pred}} = \tau_{\text{sample}} + K_{p,\text{dec}} \cdot (q_{\text{cand}} - q_{\text{meas}}) + K_{d,\text{dec}} \cdot (\dot{q}_{\text{cand}} - \dot{q}_{\text{meas}})$$
  Since $K_p$ and $K_d$ scale with $N^2$ and torque scales with $N \cdot \text{direction}$, the signs of the physical direction factor cancel out squared ($(-1)^2 = 1$), ensuring identical joint-domain algebra across negative-reduction actuators.
- When predicted effort exceeds $\text{max\_effort\_nm}$, [`_pace_position_toward_effort`](file:///tmp/c16-codex-20261017/joint_reference.py#L149-L180) calculates the feasible position window $[q_{\text{lo}}, q_{\text{hi}}]$ and bounds pacing strictly to the forward step segment $[\min(q_0, q_{\text{cand}}), \max(q_0, q_{\text{cand}})]$. This prevents reference inversion or counter-directional starvation while preserving macro-goal direction. If the feasible segment is empty or post-projection decoded effort remains over budget, planning vetoes release (`return None`).

### C. Subgrid Progress & Quantization Waiting (C11)
- Under the S2 contract, when requested macro-goals target a different grid code ($\text{static\_pos\_raw} \ne \text{prev\_pos\_raw}$) but the dynamic corridor limits the candidate to the previous code ($\text{cand\_pos\_raw} == \text{prev\_pos\_raw}$), [`plan_reference`](file:///tmp/c16-codex-20261017/joint_reference.py#L298-L302) suppresses candidate release unless `allow_bootstrap` is active.
- Suppression preserves pending targets and reference epoch, allowing physical time to accumulate across ticks until a discrete codec grid step is representable. Statically settled subgrid targets and gain-only updates are permitted.

### D. Multi-Gate Atomic Release Veto (C11, C12, C13)
In [`consume_target`](file:///tmp/c16-codex-20261017/joint_controller.py#L531-L633), candidate release is verified at release time against late changes:
1. Re-samples clock and verifies feedback freshness ([`_tick_at`](file:///tmp/c16-codex-20261017/joint_controller.py#L463-L478)).
2. Verifies ownership generation watermark and target deadline.
3. Checks finite-difference release rates ([`position_rates_at_release`](file:///tmp/c16-codex-20261017/joint_reference.py#L321-L373)):
   $$|w_{\text{new}}| \le V \quad \text{and} \quad |w_{\text{new}} - w_0| \le A \cdot \text{release\_elapsed}$$
   At $\text{release\_elapsed} == 0$, exact position and speed matches are required, returning $w_0$ without division by zero.
4. Re-checks measured lead: $|q_{\text{cand}} - q_{\text{meas}}| \le \text{max\_measured\_lead\_rad}$.
5. Re-checks predicted effort: $|\tau_{\text{pred}}| \le \text{max\_effort\_nm}$.
Failure of any gate triggers an atomic veto: candidate is dropped, pending target is retained, and reference history remains at the previous timestamp.

---

## 3. Fault Credit & Protection Lifecycle (C14)

### A. Separation of Predicted and Reported Effort
- **Command Effort (C13)**: Evaluated at planning and release against `JointSpec.max_effort_nm`; vetoes or paces software references.
- **Reported Effort (C14)**: Measured from feedback [`JointSample.tau_nm`](file:///tmp/c16-codex-20261017/joint_mapping.py#L141-L148) in [`_observe_protection`](file:///tmp/c16-codex-20261017/joint_controller.py#L157-L188); accumulates confirmed physical exposure time toward `sustained_effort_nm`.

### B. Dual Temperature & Protection Availability
- [`MotorHealth`](file:///tmp/c16-codex-20261017/joint_records.py#L170-L205) requires explicit `driver_c` and `winding_c` (float or `None`). If either channel is `None`, `MotorHealth.healthy` evaluates to `False`, and [`_latch_joint_fault`](file:///tmp/c16-codex-20261017/joint_controller.py#L201-L239) sets `protection_unavailable = True`.
- Temperature at or above limit ($\ge \text{driver\_limit\_c}$ or $\ge \text{winding\_limit\_c}$) immediately latches `over_temperature = True`, even if the sibling channel is missing or cool.

### C. Exposure Counter & Stopped Recovery Credit
- Exact duration differences ($\Delta t$) accumulate using `fractions.Fraction` only between credible consecutive samples separated by $\Delta t \le \min(\text{feedback\_max\_age\_s}, \text{max\_sample_gap\_s})$.
- Discontinuous gaps, invalid samples, or non-monotonic timestamps freeze exposure credit without resetting it.
- **Recovery Credit**: Once a fault is latched, `recovering` credit can *only* accumulate if `State.CHECKING` is active and motor telemetry confirms `health.mode == 0` (stopped).
- [`checked()`](file:///tmp/c16-codex-20261017/joint_controller.py#L368-L389) rejects with `RECOVERY_PENDING` until recovery duration is satisfied across every faulted joint.
- Exposure history (`overload_s`) survives across `disconnect()`, `connected()`, unacknowledged checks, and `stop()`.

---

## 4. Host Station Timing & Station Independence (C15, C16)

### A. Observational Host Telemetry (C15)
- [`StationTiming`](file:///tmp/c16-codex-20261017/station_timing.py#L4-L86) records loop start intervals, overruns against `SCHEDULE_GAP_MAX_S` (0.08 s), and transaction latencies.
- In [`ArmManualSession._transact_raw`](file:///tmp/c16-codex-20261017/arm_manual.py#L1027-L1066) and [`_loop`](file:///tmp/c16-codex-20261017/arm_manual.py#L2199-L2207), timing failures are caught and recorded as `measurement_errors` without impacting exchange execution, motion checks, or fault latching.
- HTTP projection in [`_project_fields`](file:///tmp/c16-codex-20261017/arm_dashboard.py#L138-L143) strictly filters timing keys through `TIMING_FIELDS`.

### B. Clean Shutdown & Independence (C16)
- [`run_dashboard`](file:///tmp/c16-codex-20261017/arm_dashboard.py#L707-L732) wraps server execution in nested `try...finally` blocks. If server binding fails or `server_close()` raises, `session.close()` is guaranteed to execute.
- HTTP connection resets cannot rearm the arm, renew expired input leases (`INGRESS_LEASE_S = 0.250 s`), or forge local capabilities.

---

## 5. Review Findings & Verdict

- **Cross-Unit Regressions**: None found. Parameter boundaries, corridor math, and fault latches interact cleanly across C11–C16.
- **Typed Public API Migration**: Complete. All records, specs, and policies are imported from [`joint_records.py`](file:///tmp/c16-codex-20261017/joint_records.py), [`joint_reference.py`](file:///tmp/c16-codex-20261017/joint_reference.py), and [`station_timing.py`](file:///tmp/c16-codex-20261017/station_timing.py) across all 16 test suites and documentation.
- **Blockers**: **No findings / No blockers**.

The combined C11–C16 implementation cleanly satisfies its specification, adheres to the established hardware safety boundaries, and is ready for parent commit and checkpointing.