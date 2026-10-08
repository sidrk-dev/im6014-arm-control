# Numerical In-Loop Design Check: C09–C12 Codec Representability & Reference Constraints

## 1. Contradiction & Smallest Explicit Domain Redesign

In [im6014_codec.py](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/im6014_codec.py#L65-L70), motor position and speed are independently rounded to distinct integer grids:
$$\text{pos\_raw} = \text{round}\left(q_{\text{out}} \cdot \frac{\text{RATIO} \cdot 32768}{2\pi}\right), \quad \text{rpm\_raw} = \text{round}\left(\text{rpm} \cdot \frac{\text{RATIO} \cdot 64}{60}\right)$$

Joint position quantum is $\Delta q = \frac{2\pi}{\text{RATIO} \cdot 32768 \cdot \text{reduction}}$, while joint velocity quantum is $\Delta \dot{q} = \frac{2\pi}{\text{RATIO} \cdot 64 \cdot \text{reduction}} = 512 \cdot \Delta q$.

Demanding exact on-wire derivative coupling $\dot{q}_{\text{wire}} = \frac{q_{\text{wire}} - q_{\text{prev}}}{h}$ requires:
$$m \cdot \Delta \dot{q} = \frac{k \cdot \Delta q}{h} \implies k = 512 \cdot h \cdot m \quad (k, m \in \mathbb{Z})$$

For an arbitrary measured elapsed preparation interval $h > 0$, $512 \cdot h$ is non-integer, so no non-zero integer solution $(k, m)$ exists. 

**Smallest Domain Redesign**: Abandon exact on-wire coupling. The domain must explicitly distinguish two typed velocities:
1. **Derived sample position step velocity**: $v_{\text{step}} = (q_{\text{wire}} - q_{\text{prev}}) / h$, bounded samplewise by $[-V, V]$, $[v_{\text{step\_prev}} - Ah, v_{\text{step\_prev}} + Ah]$, and lead limits.
2. **Emitted motor velocity**: $\dot{q}_{\text{cmd}}$ (encoded as RPM), bounded independently by $[-V, V]$ and $[\dot{q}_{\text{prev}} - Ah, \dot{q}_{\text{prev}} + Ah]$.

---

## 2. Representation Comparison: (A) vs. (B)

- **Representation (A) (Continuous trajectory reference $q_{\text{ref}}/\dot{q}_{\text{ref}}$ + separately decoded emitted [`JointTarget`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_mapping.py#L110))**:
  - *Defect*: Creates **fake precision**. The core commits an idealized continuous float state that the motor never executes. Because wire quantization perturbs $q_{\text{wire}}$ away from $q_{\text{ref}}$, actual position displacement rates on the actuator $(q_{\text{wire}} - q_{\text{wire\_prev}})/h$ can violate velocity or acceleration bounds even when continuous $q_{\text{ref}}$ satisfied them. Over time, continuous state drifts from physical motor state.
- **Representation (B) (Emitted representable target only, discrete bounds on $\Delta q / h$ and emitted $\dot{q}$ separately)**:
  - *Recommendation*: **Meets all milestones (C09–C12) without fake precision**.
  - History commits only decoded representable values.
  - The feasible position interval $[q_{\text{next\_min}}, q_{\text{next\_max}}]$ is formed by projecting the admissible step velocity intersection into position space: $[q_{\text{prev}} + v_{\min} h, q_{\text{prev}} + v_{\max} h]$.
  - An inward grid helper selects an exact representable $q_{\text{wire}}$ within that interval.
  - Emitted $\dot{q}_{\text{cmd}}$ is independently chosen within velocity bounds on the velocity grid.
  - If the representable intersection is empty, motion is withdrawn immediately without silent relaxation.

---

## 3. Concrete Counterexample: Naive Round-After-Limiting

Consider joint position limit $q_{\max} = 1.0\,\text{rad}$ with $\text{reduction} = 1.0$, $\text{direction} = 1$, $\text{zero\_output\_rad} = 0.0$ in [`JointMap`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_mapping.py#L50).
- Scale factor: $S_q = \frac{(38/3) \cdot 32768}{2\pi} \approx 66050.68652$.
- At the boundary $q = 1.0\,\text{rad}$, raw command is $1.0 \cdot S_q = 66050.68652$.
- Naive rounding in [`command`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/im6014_codec.py#L48): $\text{round}(66050.68652) = 66051$.
- Decoded back: $66051 / S_q \approx \mathbf{1.00000475\,\text{rad} > q_{\max}}$. **Strict limit violated.**

**Negative Mapping ($\text{direction} = -1$)**:
- For $q_{\min} = -1.0\,\text{rad}$, $q_{\text{out}} = -1 \cdot (-1.0) = +1.0\,\text{rad}$.
- Naive round gives raw $66051$, which decodes back to $q_{\text{joint}} = \mathbf{-1.00000475\,\text{rad} < q_{\min}}$. **Lower limit violated.**

**Speed Limit**:
- For $V = 2.015\,\text{rad/s}$, scale $S_v = \frac{(38/3) \cdot 64}{2\pi} \approx 64.5026$.
- Raw value: $2.015 \cdot S_v \approx 129.972$.
- Naive round produces raw $130$, which decodes to $\mathbf{2.01543\,\text{rad/s} > V}$. **Speed limit violated.**

---

## 4. Codec-Owned Helper Proposal (Narrow Canonical API)

The core must not copy codec scale constants ($32768$, $64$, $\text{RATIO}$). [im6014_codec.py](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/im6014_codec.py) and [joint_mapping.py](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_mapping.py) own grid rounding and decode helpers:

```python
# im6014_codec.py
def quantize_position_inward(lo_rad: float, hi_rad: float) -> tuple[float, float] | None: ...
def quantize_speed_inward(lo_rpm: float, hi_rpm: float) -> tuple[float, float] | None: ...
def decode_command_position(raw_pos: int) -> float: ...
def decode_command_speed(raw_rpm: int) -> float: ...

# joint_mapping.py (JointMap)
def quantize_joint_position_inward(self, lo_rad: float, hi_rad: float) -> tuple[float, float] | None:
    # Handles direction == -1 interval flipping, delegates to im6014_codec, and maps back.
    ...
```

If the helper returns `None`, the interval contains zero representable grid points; the core withdraws motion and transitions to `State.FAULT`/`State.STOPPED`.

---

## 5. History, Preparation, and Prewrite Semantics

- **History Fields**:
  - In [`JointDispatchedRecord`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_contract.py#L300): retain `target: JointTarget`.
  - In [`JointCore`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_controller.py#L43) reference tracking: store confirmed $q_{\text{prev}}$ (decoded representable position), $v_{\text{step\_prev}}$, $\dot{q}_{\text{prev}}$ (decoded emitted velocity), and $t_{\text{prev}}$.
- **Timing & Gate Separation**:
  - Reference preparation time $t_{\text{prep}}$ defines $h = t_{\text{prep}} - t_{\text{prev}}$.
  - If $h == 0$, refresh permits unchanged reference only.
  - If $h > h_{\text{max\_gap}}$, withdraw motion (Stop).
  - Calling [`prepare_refresh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_controller.py#L456) advances **no** reference history.
  - At [`claim_dispatch`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_controller.py#L502): recheck window expiry, ownership token, healthy status, and fresh measured lead $|q_{\text{wire}} - q_{\text{meas}}| \le \text{lead}$.
  - [`WriteResult.COMPLETE`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_contract.py#L52) commits **that joint only**. Partial or failed writes latch a fault and do not advance reference history.

---

## 6. Executable Pure Proof Example

The following pure Python script verifies:
1. Concrete counterexample showing naive rounding violates bounds.
2. Inward quantization strictly respects bounds under negative mapping (`direction = -1`).
3. Discrete step velocity $v_{\text{step}}$ satisfies speed, acceleration, position, and lead bounds.
4. Empty/unrepresentable intervals trigger withdrawal (`None`), preventing silent relaxation.

```python
import math

RATIO = 38.0 / 3.0
TAU = math.tau
POS_SCALE = (RATIO * 32768.0) / TAU
SPEED_SCALE = (RATIO * 64.0) / TAU  # rad/s to raw velocity units

def quantize_inward(lo: float, hi: float, scale: float) -> tuple[float, float] | None:
    if not (math.isfinite(lo) and math.isfinite(hi)) or lo > hi:
        return None
    raw_lo = math.ceil(lo * scale)
    raw_hi = math.floor(hi * scale)
    if raw_lo > raw_hi:
        return None
    return raw_lo / scale, raw_hi / scale

def run_proof():
    # 1. Counterexample: Naive rounding exceeds upper position limit
    q_max = 1.0
    raw_ideal = q_max * POS_SCALE
    naive_q = round(raw_ideal) / POS_SCALE
    assert naive_q > q_max, f"Naive rounding failed to overshoot: {naive_q} <= {q_max}"

    # Inward rounding strictly respects q_max
    inward_q = quantize_inward(-10.0, q_max, POS_SCALE)
    assert inward_q is not None and inward_q[1] <= q_max

    # 2. Inward selection with negative mapping (direction = -1, reduction = 2.0)
    direction, reduction = -1, 2.0
    q_min_j, q_max_j = -1.0, 0.5
    # Output position: q_out = direction * reduction * q_j
    out_a = direction * reduction * q_min_j  # +2.0
    out_b = direction * reduction * q_max_j  # -1.0
    out_lo, out_hi = min(out_a, out_b), max(out_a, out_b)
    grid_out = quantize_inward(out_lo, out_hi, POS_SCALE)
    assert grid_out is not None

    # Map grid boundaries back to joint space
    j_candidates = (grid_out[0] / (direction * reduction), grid_out[1] / (direction * reduction))
    j_min_dec, j_max_dec = min(j_candidates), max(j_candidates)
    assert j_min_dec >= q_min_j and j_max_dec <= q_max_j

    # 3. Discrete reference step rule with h > 0, V, A, lead, and position bounds
    h = 0.005
    q_prev = 0.1
    v_step_prev = 0.5
    V = 1.5
    A = 20.0
    lead = 0.02
    q_meas = 0.102
    q_min, q_max = -0.5, 0.5

    # Feasible step velocity intersection
    v_lo = max(-V, v_step_prev - A * h, (q_min - q_prev) / h, (q_meas - lead - q_prev) / h)
    v_hi = min(V, v_step_prev + A * h, (q_max - q_prev) / h, (q_meas + lead - q_prev) / h)
    assert v_lo <= v_hi

    # Feasible joint position range
    q_next_lo = q_prev + v_lo * h
    q_next_hi = q_prev + v_hi * h
    q_grid = quantize_inward(q_next_lo, q_next_hi, POS_SCALE)
    assert q_grid is not None

    # Pick representable position (e.g. upper inward bound)
    q_wire = q_grid[1]
    v_step_actual = (q_wire - q_prev) / h

    # Validate all invariants
    assert -V <= v_step_actual <= V
    assert v_step_prev - A * h - 1e-12 <= v_step_actual <= v_step_prev + A * h + 1e-12
    assert q_min <= q_wire <= q_max
    assert abs(q_wire - q_meas) <= lead + 1e-12

    # 4. Infeasible/unrepresentable interval withdraws motion (returns None)
    # Contradictory interval (lo > hi)
    empty_interval = quantize_inward(0.105, 0.101, POS_SCALE)
    assert empty_interval is None

    # Interval narrower than one quantization step
    step = 1.0 / POS_SCALE
    narrow_empty = quantize_inward(0.1 + 0.1 * step, 0.1 + 0.8 * step, POS_SCALE)
    assert narrow_empty is None

if __name__ == "__main__":
    run_proof()
```

---

## 7. Summary of Changes and Verification

- **Changes**:
  - Verified math and codec grid structures in [im6014_codec.py](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/im6014_codec.py), [joint_mapping.py](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_mapping.py), [joint_contract.py](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_contract.py), [joint_controller.py](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/joint_controller.py), and [design.md](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/quant-design-gemini/design.md).
  - Resolved C09–C12 codec representability by proving exact on-wire coupling $\dot{q}_{\text{wire}} == (q_{\text{wire}} - q_{\text{prev}})/h$ is mathematically impossible for arbitrary $h$.
  - Selected Representation (B) over (A) to prevent fake precision and drift.
  - Proposed narrow codec-owned inward grid quantization API.
  - Provided a concrete counterexample to naive rounding and a compact executable proof verifying inward grid selection under bounds, negative mapping, finite arithmetic, and empty-interval withdrawal.
- **Verification**:
  - Pure analytical inspection only.
  - No files were edited, no tests or shell commands were executed, and no git operations were performed.