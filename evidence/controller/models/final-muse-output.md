## Findings

### 1. [warning] `arm()` does not enforce Stop mode
**Location**: `joint_controller.py:505-518` vs `489-503`
**Finding**: `checked()` requires `health.mode == 0` for every joint, but `arm()` only requires `valid`, fresh and `healthy` (errors/warnings/timeout + comm-error). A healthy FOC (`mode==1`) observation after `READY` arms successfully.
**Evidence**: `healthy` is `errors==0 and warnings==0 and not timeout` plus `communication_error is None`; mode is not part of it. `checked()` has explicit `if any(joint.health.mode != 0 ...): NOT_STOPPED`, `arm()` has no equivalent. Path: `READY` (mode 0) -> `receive(id,SAMPLE,MotorHealth(1,0,0,False))` stays `READY` (receive does not change state when healthy) -> `arm()` succeeds with FOC feedback. `submit_target`/`consume_target` also never check mode, so FOC can be commanded while motor already reports FOC.
**Suggestion**: Add the same `NOT_STOPPED` guard to `arm()`, or document why `READY->ARMED` is allowed to change mode without a re-check. `motor_session._handle('arm')` does `healthy(latest,0)`, so the GUI layer disagrees with the core contract.

### 2. [warning] `JointFault` permits inconsistent `communication_error` / `communication_errors`
**Location**: `joint_controller.py:95-118`
**Finding**: Sync only handles one-sided `None`. If both are set but disagree, the inconsistency is silently kept.
**Evidence**: `if communication_error is not None and not communication_errors: ... elif communication_errors and communication_error is None: ...` — no `else`. `JointFault("J1",0,communication_error=TIMEOUT,communication_errors=(DRIVER_FAILURE,))` constructs without error, leaving `communication_error` outside `communication_errors` and `affected_joints`/snapshot logic ambiguous. Intent states at most one record per joint and a union of three fixed codes; constructor should enforce `communication_error in (None,)+communication_errors`.
**Suggestion**: Reject mismatched pairs, or normalise `communication_error` to `communication_errors[-1]`.

### 3. [warning] Session error accounting uses substring match instead of types
**Location**: `motor_session.py:220-225`
**Finding**: `if 'CRC' in str(error)` / `if 'deadline' in str(error)` classifies transport outcomes.
**Evidence**: `im6014_serial` already provides typed `InvalidFeedbackError`, `TransactionTimeoutError`, `TimeoutError` and `capture_exchange` maps them to fixed `OUTCOMES`. String matching mis-counts a `RuntimeError("...deadline...")` from unrelated code as a reply timeout, misses a typed timeout whose message changes, and counts any driver message containing `CRC` as a CRC error. Tests only cover the current message wording.
**Suggestion**: Use `isinstance(error, InvalidFeedbackError)` / `TransactionTimeoutError` for counters.

### 4. [warning] Numeric-string channel ignored, degrades to unverified instead of rejecting
**Location**: `im6014_port.py:286-293,296-327`
**Finding**: `_channel` only accepts `int>=0` or strings containing `if`/`interface`. A plain `"3"` returns `None`.
**Evidence**: `_metadata` calls `_channel(channel)` then falls back to interface/location/by-id. `dict(serial_number='FTC2RQ7H', channel='3', ...)` therefore yields `interface=None`, and `inspect_port` treats it as unknown/incomplete (unverified, open allowed) rather than matching or explicitly rejecting a wrong channel. `int` channel `3` matches. This makes validation depend on caller type in a way not documented in `joint-api.md`.
**Suggestion**: Accept decimal-string integers in `_channel` or reject them explicitly.

### 5. [warning] `snapshot()` / `tick()` raise on clock regression instead of faulting
**Location**: `joint_controller.py:388-401,520-541,612-618`
**Finding**: `_age` raises `ValueError` if `age<0` or non-finite. `tick()`, `_all_fresh()`, `snapshot()` propagate it.
**Evidence**: A non-monotonic clock (the exact condition `_age` detects) makes `snapshot()` uncallable, so the operator cannot inspect `state`/`faults`/`feedback_ages_s` after the failure. `consume_target` calls `tick()` first, so a regressing clock raises instead of returning `None` and latching a stale fault, violating "submission/consumption must never release stale targets" via exception rather than safe `None`/FAULT. Clock misuse should either latch `FAULT` or make `snapshot()` infallible.
**Suggestion**: Keep strict validation in `receive`/`checked`/`arm`/`submit`, but make `snapshot()` never raise for age calculation and make `tick()` convert regression to stale faults.

### 6. [warning] Duplicated fresh/healthy gate in four places
**Location**: `joint_controller.py:489-503,505-518,543-581,583-593`
**Finding**: `checked`, `arm`, `submit_target`, and `consume_target` (via `tick` + healthy check) each re-implement "all valid + all fresh + all healthy".
**Evidence**: Same three comprehensions repeated with slight variation; `consume_target` duplicates again after calling `tick()`. Future change to freshness (e.g., adding the missing mode gate from #1) must be applied in four spots. This is the exact drift that already caused #1.
**Suggestion**: Single `_fresh_healthy(now)` helper returning a `DecisionCode` or `None`, used by all four paths.

### 7. [warning] `existing = ...get(name) or JointFault(...)` relies on truthiness
**Location**: `joint_controller.py:359`
**Finding**: `JointFault` is a frozen dataclass with no `__bool__`; today it is always truthy, so `or` works, but the pattern breaks silently if `__bool__`/`__len__` is ever added.
**Evidence**: `self._faults.get(name) or JointFault(...)` vs explicit `if existing is None`. Same file uses `bool(self._faults)` correctly for dicts, but here the fallback depends on the value type staying truthy.
**Suggestion**: `existing = self._faults.get(name); if existing is None: existing = JointFault(...)`.

### 8. [nit] `healthy()` boolean expression needs parentheses
**Location**: `spin_5rpm.py:19-20`
**Finding**: `if f['mode'] != mode or f['errors'] or f['warnings'] or f['timeout'] and not allow_timeout:` is correct only because `and` binds tighter than `or`, but readers must recall precedence for a safety gate.
**Evidence**: Parent inspection already noted this in `docs/bench-review.md` and declined a style change. For a motor gate, explicit `(... or ... or ... or (f['timeout'] and not allow_timeout))` costs nothing.
**Suggestion**: Add parentheses.

### 9. [nit] `im6014_port.py` duplicates trusted-directory validation
**Location**: `im6014_port.py:117-171`
**Finding**: `_validate_private_directory` and `_validate_tmp_root` repeat lstat/open/fstat/compare with different uid/mode constants.
**Evidence**: ~25 lines each, same TOCTOU pattern (`lstat` -> `os.open(O_NOFOLLOW|O_DIRECTORY)` -> `fstat` -> compare dev/ino). Divergence risk if one path is hardened later.
**Suggestion**: One `_validate_dir(path, uid, mode)` helper called twice.

### 10. [nit] Misleading `command()` mode error text
**Location**: `im6014_codec.py:50-51`
**Finding**: `raise ValueError('Individual ID 0–14 and Stop/FOC modes only')` is raised for a bad `mode` alone, after `motor_id` was already validated.
**Evidence**: Caller passing `command(0, mode=2)` gets an error mentioning Individual ID, complicating CLI/GUI messages that surface `str(error)` directly.
**Suggestion**: Separate messages for ID vs mode.
