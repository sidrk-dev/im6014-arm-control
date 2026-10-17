Here is the adversarial code review of the C13 changes across [joint_reference.py](file:///tmp/c13-paced-parent-check-20261017/joint_reference.py), [joint_controller.py](file:///tmp/c13-paced-parent-check-20261017/joint_controller.py), and [test_controller_effort.py](file:///tmp/c13-paced-parent-check-20261017/test_controller_effort.py).

---

### Ranked Concrete Findings

#### 1. [Critical Correctness / Safety] Pacing inverts reference direction and causes runaway divergence when non-position effort exceeds the budget
- **Location**: [joint_reference.py:177-195](file:///tmp/c13-paced-parent-check-20261017/joint_reference.py#L177-L195) (`_pace_position_toward_effort`) and [joint_reference.py:417-422](file:///tmp/c13-paced-parent-check-20261017/joint_reference.py#L417-L422) (`plan_reference`).
- **Failing Scenario**:
  A joint is at rest at $q_0 = 0.0\,\text{rad}, q_{\text{meas}} = 0.0\,\text{rad}, \dot{q}_{\text{meas}} = 0.0\,\text{rad/s}$.
  Target requests forward motion: $q_{\text{req}} = 1.0\,\text{rad}, \dot{q}_{\text{req}} = 0.5\,\text{rad/s}, K_p = 20.0, K_d = 4.0, \tau_{\text{ff}} = 0.0$, with $T_{\text{max}} = 1.0\,\text{Nm}$ and corridor $V_h = 0.1\,\text{rad}$.
  1. `cand` plans forward motion with $\dot{q}_{\text{cand}} = 0.5\,\text{rad/s}$.
  2. Damping effort alone is $K_d \cdot (\dot{q}_{\text{cand}} - \dot{q}_{\text{meas}}) = 4.0 \times 0.5 = 2.0\,\text{Nm} > 1.0\,\text{Nm}$.
  3. `_pace_position_toward_effort` computes:
     $$\text{base} = 2.0\,\text{Nm}$$
     $$q_{\text{hi}} = \frac{T_{\text{max}} - \text{base}}{K_p} = \frac{1.0 - 2.0}{20.0} = -0.05\,\text{rad} < q_0$$
     $$q_{\text{lo}} = \frac{-T_{\text{max}} - \text{base}}{K_p} = \frac{-1.0 - 2.0}{20.0} = -0.15\,\text{rad}$$
  4. With $q_{\text{min}} = -0.1\,\text{rad}$ and $q_{\text{max}} = +0.1\,\text{rad}$, the paced corridor becomes $[-0.1, -0.05]\,\text{rad}$.
  5. `project_joint_target(spec, req, -0.1, -0.05, ...)` projects $q_{\text{req}} = 1.0\,\text{rad}$ inward to the nearest bound, yielding $\mathbf{q_{\text{paced}} = -0.05\,\text{rad}}$.
  6. The progress check at line 418 checks only `cand_pos_raw == prev_pos_raw`. Because $-0.05 \ne 0.0$, the progress check **passes**.
  7. Release lead, velocity, and effort checks all pass ($\tau_{\text{pred}} = 2.0 + 20.0 \times (-0.05) = 1.0\,\text{Nm} \le 1.0\,\text{Nm}$).
  8. **Outcome**: The controller commands the joint in **reverse** ($-0.05\,\text{rad}$), away from the commanded goal of $+1.0\,\text{rad}$. On subsequent ticks, this creates a divergent backward crawl away from the target instead of recognizing that no safe progress exists in the requested direction.
- **Minimal Correction**:
  Pacing must only narrow the corridor in the direction of the requested target progress relative to $q_0$. If the effort-feasible interval does not permit progress toward $q_{\text{req}}$ (i.e., for $q_{\text{req}} \ge q_0$, $q_{\text{hi}} < q_0$; for $q_{\text{req}} \le q_0$, $q_{\text{lo}} > q_0$), no safe progress exists and the helper must return `None` to suppress the tick:
  ```python
  if req.q_rad >= cand.q_rad:  # or q0
      paced_min = max(q_min, q_lo, cand.q_rad)
      paced_max = min(q_max, q_hi)
  else:
      paced_min = max(q_min, q_lo)
      paced_max = min(q_max, q_hi, cand.q_rad)
  if paced_min > paced_max:
      return None
  ```

---

#### 2. [Structural / Performance] Excessive redundant codec roundtrips on the reference planning hot path
- **Location**: [joint_reference.py:164-203](file:///tmp/c13-paced-parent-check-20261017/joint_reference.py#L164-L203) and [joint_reference.py:364-406](file:///tmp/c13-paced-parent-check-20261017/joint_reference.py#L364-L406).
- **Issue**:
  When a candidate is paced, `mapping.to_output`, `command`, and `decode_command` execute repeatedly for the exact same joint candidate:
  1. Once in `cand = project_joint_target(...)`.
  2. Once in `compute_predicted_effort(spec, cand, ...)`.
  3. Immediately again inside `_pace_position_toward_effort` on line 165 on the identical `cand`.
  4. Once in `paced = project_joint_target(...)`.
  5. Once in `compute_predicted_effort(spec, paced, ...)`.
  6. A 6th time at lines 385–395 to recover `cand_pos_raw` and `cand_speed_raw`, discarding the raw values `project_joint_target` had already computed.
  7. A 7th time at release in `consume_target()`.
- **Minimal Correction**:
  Pass the decoded terms from step 2 directly into `_pace_position_toward_effort(spec, req, cand, terms, ...)` instead of re-encoding and re-decoding `cand`. Furthermore, avoid re-encoding at line 385 solely to extract raw grid codes that are already verified and returned by `project_joint_target`.

---

#### 3. [Verification Gap] `test_effort_pacing_preserves_macro_progress` subtest `"kp_zero_feedforward_only"` does not test feedforward or pacing
- **Location**: [test_controller_effort.py:319-336](file:///tmp/c13-paced-parent-check-20261017/test_controller_effort.py#L319-L336).
- **Issue**:
  The subtest declares:
  ```python
  target = Target(
      sequence=1,
      joints=(JointTarget(q_rad=1.0, dq_rad_s=0.0, tau_nm=0.0, kp=0.0, kd=0.0),),
      ...
  )
  ```
  `tau_nm` is $0.0$, `kp` is $0.0$, and `kd` is $0.0$. The predicted effort is identically $0.0\,\text{Nm}$, well within `max_effort=1.0`. Neither feedforward effort nor effort pacing is exercised by this subtest.
- **Minimal Correction**:
  Set `tau_nm` to a non-zero valid feedforward value (e.g., $0.8\,\text{Nm} \le T_{\text{max}}$) to test feedforward propagation, and test suppression when feedforward alone or damping alone exceeds $T_{\text{max}}$ with $K_p = 0$.

---

#### 4. [Verification Gap] Missing multi-joint atomic suppression test when pacing fails on one joint
- **Location**: [test_controller_effort.py:260-289](file:///tmp/c13-paced-parent-check-20261017/test_controller_effort.py#L260-L289).
- **Issue**:
  `test_multi_joint_coupling_paces_whole_vector_atomically` tests the scenario where J2 is paced down and J1 is unpaced, and both are emitted together. It does not test the architectural invariant that if J2 cannot find safe progress within the effort limit, the **entire vector is suppressed atomically** and pending targets are retained without emitting J1 alone.
- **Minimal Correction**:
  Add a companion test where J1 has a valid low-effort reference but J2 has an unfeasible reference (e.g., $K_p = 0, K_d \cdot \Delta \dot{q} > T_{\text{max}}$), verifying `consume_target()` returns `None`, no partial issue occurs, and `core.snapshot().pending_target` remains intact.

---

### Verification Summary
- Reviewed diffs and complete sources for [joint_reference.py](file:///tmp/c13-paced-parent-check-20261017/joint_reference.py), [joint_controller.py](file:///tmp/c13-paced-parent-check-20261017/joint_controller.py), [joint_mapping.py](file:///tmp/c13-paced-parent-check-20261017/joint_mapping.py), [joint_records.py](file:///tmp/c13-paced-parent-check-20261017/joint_records.py), and [test_controller_effort.py](file:///tmp/c13-paced-parent-check-20261017/test_controller_effort.py).
- Verified mathematical invariance of joint-level effort prediction across `reduction` and `direction`: $N^2$ gain scaling and canonical sample mapping preserve sign and scale under both $\pm 1$ directions.
- Read-only inspection complete; no files written or modified.