# Adversarial Code Review: Corrected C13 Effort Pacing Artifact

**Review Target:** Milestone C13 Correction  
**Repository Location:** `/tmp/c13-direction-parent-check-20261017`  
**Primary Files Inspected:**  
- [`joint_reference.py`](file:///tmp/c13-direction-parent-check-20261017/joint_reference.py) (lines 152–204, 369–373)  
- [`joint_controller.py`](file:///tmp/c13-direction-parent-check-20261017/joint_controller.py) (lines 389–439, 446–545)  
- [`joint_records.py`](file:///tmp/c13-direction-parent-check-20261017/joint_records.py)  
- [`joint_mapping.py`](file:///tmp/c13-direction-parent-check-20261017/joint_mapping.py)  
- [`test_controller_effort.py`](file:///tmp/c13-direction-parent-check-20261017/test_controller_effort.py)  

---

## 1. Executive Summary

The corrected C13 implementation addresses the real reference inversion defect identified in prior runs (where a forward goal $q_{\text{goal}} = +1.0$ produced an inverted issued reference $q = -0.050712$ solely to cancel damping torque). 

The new fix in [`_pace_position_toward_effort`](file:///tmp/c13-direction-parent-check-20261017/joint_reference.py#L152-L204) constrains the paced corridor by intersecting the signed effort interval $[q_{\text{lo}}, q_{\text{hi}}]$ with the segment between the previously released reference $q_0$ and the greedily bounded candidate $q_{\text{cand}}$:
$$[\text{step}_{\text{lo}}, \text{step}_{\text{hi}}] = [\min(q_0, q_{\text{cand}}), \max(q_0, q_{\text{cand}})]$$

### Verification Verdict: **PASS / APPROVED**
- **Defect Elimination Verified:** Reversal of reference direction for torque cancellation is mathematically impossible under the revised corridor intersection.
- **Suppression vs Inversion Invariant:** When damping or feedforward exceeds budget such that no non-inverted position step can satisfy the effort bound, the intersection collapses ($\text{paced}_{\min} > \text{paced}_{\max}$), returning `None` and preserving the pending target in state `ARMED` without corrupting history.
- **Braking and Kinematic Corridor Integrity:** Legitimate braking steps (where $q_{\text{cand}} < q_0$ despite $q_{\text{req}} > q_0$ due to acceleration constraints) are preserved when within effort limits and suppressed cleanly when outside them.
- **Atomic Multi-Joint Coupling:** Multi-joint vectors are evaluated and paced atomically; infeasibility in one joint suppresses the entire issue.
- **Release-Time Gate:** Re-evaluating predicted effort with fresh observations immediately prior to commit remains active, preventing observation-shift race conditions.

---

## 2. Forensic Analysis of the Correction

### 2.1 Math & Execution Path in [`_pace_position_toward_effort`](file:///tmp/c13-direction-parent-check-20261017/joint_reference.py#L152-L204)

```python
# joint_reference.py:188-193
step_lo = min(q0, cand.q_rad)
step_hi = max(q0, cand.q_rad)
paced_min = max(q_min, q_lo, step_lo)
paced_max = min(q_max, q_hi, step_hi)
if paced_min > paced_max:
    return None
```

1. **Effort Corridor Bounds ($q_{\text{lo}}, q_{\text{hi}}$):**
   - Predicted effort is linear in reference position:
     $$\tau_{\text{pred}}(q) = \text{base} + K_{p,\text{dec}} \cdot q$$
     where $\text{base} = \tau_{\text{ff},\text{dec}} + K_{d,\text{dec}}(\dot{q}_{\text{ref}} - \dot{q}_{\text{meas}}) - K_{p,\text{dec}} \cdot q_{\text{meas}}$.
   - Since $K_{p,\text{dec}} > 0$, the condition $|\tau_{\text{pred}}(q)| \le \tau_{\max}$ defines the closed interval:
     $$q_{\text{lo}} = \frac{-\tau_{\max} - \text{base}}{K_{p,\text{dec}}}, \quad q_{\text{hi}} = \frac{\tau_{\max} - \text{base}}{K_{p,\text{dec}}}$$

2. **Intersection with Motion Segment ($[\text{step}_{\text{lo}}, \text{step}_{\text{hi}}]$):**
   - **Forward step ($q_{\text{cand}} \ge q_0$):**
     $$\text{step}_{\text{lo}} = q_0, \quad \text{step}_{\text{hi}} = q_{\text{cand}}$$
     $$\text{paced}_{\min} = \max(q_{\min}, q_{\text{lo}}, q_0) \ge q_0$$
     $$\text{paced}_{\max} = \min(q_{\max}, q_{\text{hi}}, q_{\text{cand}}) \le q_{\text{cand}}$$
     Any representable grid point $q \in [\text{paced}_{\min}, \text{paced}_{\max}]$ strictly satisfies $q_0 \le q \le q_{\text{cand}}$. A negative step ($q < q_0$) cannot be selected because $\text{paced}_{\min} \ge q_0$. If $q_{\text{hi}} < q_0$, $\text{paced}_{\min} > \text{paced}_{\max}$, returning `None`.
   - **Backward / Reverse step ($q_{\text{cand}} \le q_0$):**
     $$\text{step}_{\text{lo}} = q_{\text{cand}}, \quad \text{step}_{\text{hi}} = q_0$$
     $$\text{paced}_{\min} = \max(q_{\min}, q_{\text{lo}}, q_{\text{cand}}) \ge q_{\text{cand}}$$
     $$\text{paced}_{\max} = \min(q_{\max}, q_{\text{hi}}, q_0) \le q_0$$
     Any representable grid point $q \in [\text{paced}_{\min}, \text{paced}_{\max}]$ strictly satisfies $q_{\text{cand}} \le q \le q_0$. A positive step ($q > q_0$) cannot be selected. If $q_{\text{lo}} > q_0$, $\text{paced}_{\min} > \text{paced}_{\max}$, returning `None`.

3. **Inversion Scenario Elimination:**
   In the reproduced failure scenario ($q_0 = 0.0$, $q_{\text{goal}} = 1.0$, $\dot{q}_{\text{goal}} = 0.5$, $K_d = 4.0$, $\tau_{\max} = 1.0$):
   - Damping torque is $+2.0\text{ Nm} > \tau_{\max}$.
   - To cancel $+2.0\text{ Nm}$, the required position interval is $[q_{\text{lo}}, q_{\text{hi}}] = [-0.15, -0.05]$.
   - Under the prior logic, $q_{\text{req}} = 1.0$ was projected into $[-0.15, -0.05]$, producing $q = -0.050712$.
   - Under the new logic:
     $$\text{step}_{\text{lo}} = 0.0, \quad \text{step}_{\text{hi}} = 0.1$$
     $$\text{paced}_{\min} = \max(-0.1, -0.15, 0.0) = 0.0$$
     $$\text{paced}_{\max} = \min(0.1, -0.05, 0.1) = -0.05$$
     Since $0.0 > -0.05$, the function exits with `None`. The controller suppresses the step cleanly and remains in `ARMED` with the pending target intact.

---

## 3. Review of Boundary Cases and Connected Context

### 3.1 Inertial Braking Steps
In [`test_controller_effort.py#L446-L487`](file:///tmp/c13-direction-parent-check-20261017/test_controller_effort.py#L446-L487):
- **Scenario:** Joint moving at $\dot{q}_0 = -2.0\text{ rad/s}$ with goal $q_{\text{req}} = 1.0\text{ rad}, \dot{q}_{\text{req}} = 2.0\text{ rad/s}$. Due to maximum acceleration limits ($A = 20\text{ rad/s}^2, \Delta t = 0.05\text{ s}$), maximum speed reachable is $-1.0\text{ rad/s}$, forcing $q_{\text{cand}} = 0.45 < q_0 = 0.50$.
- **Behavior with $\tau_{\max} = 50.0\text{ Nm}$:** Effort is $\sim 2.0\text{ Nm} \le 50.0\text{ Nm}$. Pacing is not invoked; legitimate braking step $q_{\text{cand}} < q_0$ is issued.
- **Behavior with $\tau_{\max} = 2.0\text{ Nm}$:** Predicted effort exceeds $2.0\text{ Nm}$. The motion segment is $[\text{step}_{\text{lo}}, \text{step}_{\text{hi}}] = [0.45, 0.50]$. To reduce effort further, $q$ would need to be $< 0.45$ (accelerating further backwards than kinematic limits allow). The intersection $[\text{paced}_{\min}, \text{paced}_{\max}]$ is empty, so `plan_reference` returns `None`. Pacing does not invent an uncontrolled braking step.

### 3.2 Fixture $K_p = 0$ Feedforward vs Over-Budget Damping
In [`test_controller_effort.py#L319-L369`](file:///tmp/c13-direction-parent-check-20261017/test_controller_effort.py#L319-L369):
- When $K_p = 0.0$, position variation has zero torque authority ($\text{slope} = K_{p,\text{dec}} = 0$).
- [`_pace_position_toward_effort`](file:///tmp/c13-direction-parent-check-20261017/joint_reference.py#L176-L177) enforces:
  ```python
  if slope <= 0.0:
      return None
  ```
- **Valid feedforward ($\tau = 0.8\text{ Nm} \le 1.0\text{ Nm}$):** Pacing is bypassed (`abs(tau_pred) <= spec.max_effort_nm`); candidate issues with exact feedforward torque.
- **Over-budget damping ($K_d = 20.0, \dot{q} = 1.0\text{ rad/s} \implies \tau = 20.0\text{ Nm} > 0.5\text{ Nm}$):** Pacing returns `None`, suppressing the candidate and leaving target pending.

### 3.3 Direction Inversion & Gear Reduction Invariance
In [`test_controller_effort.py#L420-L450`](file:///tmp/c13-direction-parent-check-20261017/test_controller_effort.py#L420-L450):
- [`JointMap.to_output`](file:///tmp/c13-direction-parent-check-20261017/joint_mapping.py#L67-L88) and [`JointMap.from_output`](file:///tmp/c13-direction-parent-check-20261017/joint_mapping.py#L90-L113) correctly apply reduction $N$ and direction $s \in \{-1, +1\}$.
- In [`_decoded_effort_terms`](file:///tmp/c13-direction-parent-check-20261017/joint_reference.py#L88-L118):
  $$K_{p,\text{dec}} = \text{dec}[kp] \cdot N^2, \quad K_{d,\text{dec}} = \text{dec}[kd] \cdot N^2, \quad \tau_{\text{ff},\text{dec}} = s \cdot N \cdot \text{dec}[\tau]$$
- Because joint torque scales as $s \cdot N \cdot \tau_{\text{actuator}}$ and position scales as $s \cdot q_{\text{actuator}} / N$, $(s)^2 = 1$, making $K_{p,\text{dec}}$ positive and invariant to direction sign. Both $s = 1$ and $s = -1$ yield identical directional constraint bounds in joint coordinates.

### 3.4 Multi-Joint Vector Atomicity
In [`test_controller_effort.py#L489-L520`](file:///tmp/c13-direction-parent-check-20261017/test_controller_effort.py#L489-L520):
- If joint 1 ($J_1$) is feasible and joint 2 ($J_2$) violates effort and has no feasible step, [`plan_reference`](file:///tmp/c13-direction-parent-check-20261017/joint_reference.py#L374-L375) returns `None` immediately.
- The controller never dispatches a partial or decoupled sub-vector.

---

## 4. Code Quality & Structural Integrity Audit

| Criterion | Evaluation |
| :--- | :--- |
| **Simplicity & Conciseness** | The fix introduces exactly 4 lines of bounds clamping (`step_lo`, `step_hi`, `paced_min`, `paced_max`) and 1 parameter (`q0`). No additional classes, auxiliary states, or secondary limiters were introduced. |
| **No Silent Gain or Torque Clipping** | Feedforward torque and gains are preserved without modification. Only the reference position is paced within representable discrete bounds. |
| **State Machine Safety** | On suppression, `self._pending_target` is preserved, `self._history` remains untouched, and state stays `ARMED` / `MOVING`. |
| **Quantization Discipline** | Projected position is re-checked through [`compute_predicted_effort`](file:///tmp/c13-direction-parent-check-20261017/joint_reference.py#L200-L202) using full encode/decode round-trip before returning, eliminating grid rounding leaks. |

---

## 5. Conclusion

The correction in [`joint_reference.py`](file:///tmp/c13-direction-parent-check-20261017/joint_reference.py) and test coverage in [`test_controller_effort.py`](file:///tmp/c13-direction-parent-check-20261017/test_controller_effort.py) satisfies all acceptance criteria for Milestone C13. The reference direction inversion defect has been eliminated at the root by geometric corridor intersection, with zero regression on legitimate braking steps, multi-joint coupling atomicity, or release-gate verification.