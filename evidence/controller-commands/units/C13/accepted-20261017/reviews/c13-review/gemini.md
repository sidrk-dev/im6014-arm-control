## Findings

### 1. [Critical] Permanent Controller Livelock and Starvation on Effort-Constrained Macro-Targets
**Location**: `joint_reference.py:300-312` (`plan_reference`) and `joint_controller.py:484-485` (`consume_target`)
**Finding**: 
`plan_reference` suppresses candidate reference vectors by returning `None` whenever `abs(tau_pred) > spec.max_effort_nm`, rather than bounding the reference displacement corridor or finding a feasible smaller grid step. Because `project_joint_target` greedily projects toward the requested macro-target (constrained only by position limits, velocity step, and lead limits), any distant target where $K_p \cdot \min(V \cdot h, \text{lead}) > \text{max\_effort\_nm}$ causes `project_joint_target` to select the boundary of the corridor, predict an effort exceeding `max_effort_nm`, and return `None`.

Because no reference is emitted, reference history (`q0`) never advances, the actuator never moves, and `q_meas` does not change. On subsequent ticks, the corridor $[q_{\min}, q_{\max}]$ remains identical (or widens up to `max_step_s`), causing `project_joint_target` to select the exact same candidate, predict the exact same excessive effort, and return `None` indefinitely. Unlike C12 lead pacing—which intersects $[q_{\text{meas}} - \text{lead}, q_{\text{meas}} + \text{lead}]$ directly into the corridor to pace large macro-targets forward safely—C13 completely freezes and starves valid macro-targets until they expire at `expires_at_s`.
**Evidence**:
In `joint_reference.py:300-312`:
```python
cand = project_joint_target(spec, req, q_min, q_max, dq_min, dq_max)
...
tau_pred = compute_predicted_effort(spec, cand, q_meas, dq_meas)
if tau_pred is None or abs(tau_pred) > spec.max_effort_nm:
    return None
```
Consider an armed joint with $q_0 = 0.0$, $q_{\text{meas}} = 0.0$, $\text{lead} = 0.5\text{ rad}$, $K_p = 20.0$, and $\text{max\_effort\_nm} = 1.0\text{ Nm}$. A target requests $q = 1.0\text{ rad}$. The corridor allows up to $\min(V \cdot h, \text{lead}) = 0.5\text{ rad}$. `project_joint_target` selects $q_{\text{cand}} = 0.5\text{ rad}$. `compute_predicted_effort` computes $\tau_{\text{pred}} \approx 20.0 \times 0.5 = 10.0\text{ Nm} > 1.0\text{ Nm}$, returning `None`. On every subsequent tick, $q_{\text{cand}}$ remains $0.5\text{ rad}$ and $\tau_{\text{pred}}$ remains $10.0\text{ Nm}$. The controller deadlocks and never emits a reference, even though a smaller step (e.g. $q_{\text{cand}} = 0.04\text{ rad}$, yielding $\tau_{\text{pred}} = 0.8\text{ Nm} \le 1.0\text{ Nm}$) is safe and feasible.
**Suggestion**:
Incorporate the effort bounds directly into the planning corridor for position (e.g. bounding $|q - q_{\text{meas}}| \le \frac{\text{max\_effort\_nm} - |\tau_{\text{ff}}| - K_d |\dot{q} - \dot{q}_{\text{meas}}|}{K_p}$ when $K_p > 0$) prior to grid projection, just as C12 did for lead pacing, so that `project_joint_target` chooses the nearest representable step that satisfies effort constraints.

---

### 2. [Warning] False Test Claim and Verification Gap for Release-Time Observation Shift
**Location**: `test_controller_effort.py:283-307` (`test_release_time_velocity_shift_vetoes_release`)
**Finding**:
`test_release_time_velocity_shift_vetoes_release` purports to verify that a velocity shift occurring right before release triggers the final release-time veto in `consume_target` (`joint_controller.py:527-529`). However, the test invokes `core.receive(...)` *before* calling `core.consume_target()`. As a result, the shifted velocity observation is already present when `consume_target()` calls `plan_reference(...)` at planning time (line 482). `plan_reference` evaluates the predicted effort, fails, and returns `None` at line 484. The release-time veto at lines 518–529 is never reached or tested.
**Evidence**:
In `test_controller_effort.py:297-304`:
```python
clock.advance(0.05)
core.receive(
    spec.motor_id,
    JointSample(q_rad=0.0, dq_rad_s=-1.0, tau_nm=0.0),
    MotorHealth(mode=1, errors=0, warnings=0, timeout=False),
)

ref = core.consume_target()
self.assertIsNone(ref)
```
In `consume_target()` (`joint_controller.py:472-485`):
```python
measured_velocities = tuple(joint.sample.dq_rad_s for joint in self._observations)
candidate_joints = plan_reference(..., measured_velocities=measured_velocities)
if candidate_joints is None:
    return None
```
Because `core.receive` updated `self._observations` before `consume_target` was entered, `measured_velocities` contained `-1.0` during planning. `plan_reference` returned `None` at line 484, short-circuiting before line 527 was ever reached. In contrast, `test_controller_lead.py:376-405` correctly hooked `core._now` to mutate observations mid-consume (between planning and release).
**Suggestion**:
Update the test to inject the observation change between the planning phase and the release phase (e.g. by wrapping `core._now` as in `test_controller_lead.py`), ensuring the release-time check at lines 527–529 is actually exercised.

---

### 3. [Warning] Broken API Example and Incomplete Documentation in `docs/joint-api.md`
**Location**: `docs/joint-api.md:86`, `docs/joint-api.md:96-98`
**Finding**:
1. The offline example in `docs/joint-api.md:96-98` instantiates `JointSpec` without passing `max_effort_nm`. Because `max_effort_nm` is a required positional argument with no default, executing this documented example raises `TypeError: JointSpec.__init__() missing 1 required positional argument: 'max_effort_nm'`.
2. `docs/joint-api.md` under "Latest target" (line 86) describes submission checks as checking ownership followed by position intervals returning `POSITION_LIMIT`, omitting any mention of feedforward effort verification or the new `EFFORT_LIMIT` rejection code.
3. The description of `consume_target()` in `docs/joint-api.md` omits any documentation of predicted effort calculation or effort-based candidate suppression.
**Evidence**:
Running the documented example in `docs/joint-api.md`:
```python
core = JointCore(
    (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-0.5, 0.5),
               max_velocity_rad_s=0.2, max_acceleration_rad_s2=1.0, max_measured_lead_rad=0.05),),
    clock=clock, max_reference_step_s=0.02,
)
```
fails with:
`TypeError: JointSpec.__init__() missing 1 required positional argument: 'max_effort_nm'`
**Suggestion**:
Update `docs/joint-api.md` to include `max_effort_nm` in the example instantiation, document `DecisionCode.EFFORT_LIMIT` in the transition and submission rules, and document the effort monitoring stage in `consume_target()`.

---

### 4. [Warning] Unhandled `AttributeError` on Non-Spec Inputs in `compute_predicted_effort`
**Location**: `joint_reference.py:77-116` (`compute_predicted_effort`)
**Finding**:
`compute_predicted_effort` explicitly checks `isinstance(..., (int, float))` and `math.isfinite` for `q_meas` and `dq_meas`, but performs no validation on `spec`. If `spec` is `None` or lacks a `mapping` attribute, line 101 (`mapping = spec.mapping`) raises `AttributeError`. Line 115 only catches `(ArithmeticError, TypeError, ValueError)`. Because `compute_predicted_effort` is a pure public function in `joint_reference.py` designed to return `None` on invalid inputs, it should fail closed and return `None` instead of propagating an unhandled crash.
**Evidence**:
Calling `compute_predicted_effort(None, cand, 0.0, 0.0)` raises:
`AttributeError: 'NoneType' object has no attribute 'mapping'`
**Suggestion**:
Add `if not isinstance(spec, JointSpec): return None` or include `AttributeError` in the caught exception tuple.

---

### 5. [Warning] Code Duplication and Dual-Path Inversion of `JointMap.from_output`
**Location**: `joint_reference.py:118-128` (`compute_predicted_effort`)
**Finding**:
In `compute_predicted_effort`, decoded packet fields are manually converted back to joint units using raw, unchecked arithmetic:
```python
tau_ff_dec = s * N * dec["torque_nm"]
q_ref_dec = s * (dec["position_rad"] - q0) / N
dq_ref_dec = s * (dec["rpm"] * math.tau / 60.0) / N
```
This directly duplicates the canonical mapping logic in `JointMap.from_output(OutputSample(dec["position_rad"], dec["rpm"], dec["torque_nm"]))` (`joint_mapping.py:85-103`). It bypasses the verified `_divide`, `_multiply`, and `_subtract` overflow/underflow helpers, creating an unneeded duplicate conversion path across module boundaries.
**Evidence**:
`joint_mapping.py:85-103` defines `from_output`, which already converts output position, RPM, and motor torque to `JointSample(q_rad, dq_rad_s, tau_nm)`. Inverting the units manually inside `compute_predicted_effort` duplicates the transformation rules and risks silent drift if reduction or unit handling semantics change.
**Suggestion**:
Use `sample = spec.mapping.from_output(OutputSample(dec["position_rad"], dec["rpm"], dec["torque_nm"]))` and extract `q_ref_dec = sample.q_rad`, `dq_ref_dec = sample.dq_rad_s`, and `tau_ff_dec = sample.tau_nm`.

---

### 6. [Nit] Dead Import and Missing Direct Unit Tests for `plan_reference` in `test_controller_effort.py`
**Location**: `test_controller_effort.py:34`
**Finding**:
`plan_reference` is imported at line 34 of `test_controller_effort.py`, but is never invoked or tested directly in any test case. The parameter validation added to `plan_reference` for `measured_velocities` (e.g. checking for boolean types, NaN, infinity, and vector length mismatches at lines 194–202 of `joint_reference.py`) has zero direct unit test coverage, unlike the thorough coverage given to `measured_positions` in `test_controller_lead.py`.
**Evidence**:
Grep search confirms `plan_reference` appears only once in `test_controller_effort.py` (on the `import` line).
**Suggestion**:
Add direct unit tests for `plan_reference(..., measured_velocities=...)` covering valid execution and error cases (mismatched length, non-finite values, boolean rejection).

---

### 7. [Nit] Quadruple Redundant Codec Encode/Decode Passes per Reference Dispatch
**Location**: `joint_reference.py:300, 310, 316` and `joint_controller.py:527`
**Finding**:
During target consumption, each joint undergoes up to four full command packet encodings and decodings:
1. `project_joint_target` encodes and decodes `command()`.
2. `compute_predicted_effort` within `plan_reference` encodes and decodes `command()`.
3. `plan_reference` (lines 316–328) encodes and decodes `command()` again to extract raw codes for progress checking.
4. `consume_target` at release time (line 527) calls `compute_predicted_effort`, which encodes and decodes `command()` a fourth time.
Each pass computes CRC32 and unpacks struct fields. While deterministic, this repeated serialization/deserialization is redundant.
**Evidence**:
The decoded parameters (`dec["position_rad"]`, `dec["rpm"]`, `dec["kp"]`, etc.) are already established during initial projection. Re-encoding and re-decoding four separate times per joint per cycle adds incidental complexity and CPU overhead.
**Suggestion**:
Pass the already projected/decoded command dictionary or raw parameters from `project_joint_target` into `compute_predicted_effort` rather than re-encoding the packet from scratch.