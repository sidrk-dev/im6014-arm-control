## Findings

### 1. [critical] `disconnect()` leaves in-flight claim alive — receipt accepted while DISCONNECTED
**Location**: `joint_controller.py: disconnect()` vs `dispatch_result()`
**Finding**: `stop()`, `_latch_joint_fault()`, `_withdraw_motion()`, `_withdraw_expired_goal()` all deliberately preserve `self._dispatch_claim` so an in-flight receipt is not lost. `disconnect()` does the same by omission: it clears `state/source/observations/motion` but never clears or rejects `self._dispatch_claim`.
**Evidence**: Call path `prepare_refresh() -> claim_dispatch(id,0)=True -> disconnect() -> dispatch_result(id,0,COMPLETE)`:
- `claim_dispatch` succeeds, `disconnect` sets `DISCONNECTED` + `_withdraw_motion()` (`_prepared_plan=None`) but `claim` remains.
- `dispatch_result` checks `claim is None` — false — clears claim, records `JointDispatchAttempt` + `JointDispatchedRecord`, calls `_now()` + `_housekeeping()` (no-op in `DISCONNECTED`), `still_current` is False so no `MOVING`, then `return ACCEPTED`.
Result is an `ACCEPTED` dispatch recorded in `dispatched_records/dispatch_attempts` while `snapshot().state is DISCONNECTED`. Intent requires stop/expiry/fault/replacement not lose receipt *without reviving motion*; disconnect is un-specified but accepting/recording work while disconnected misattributes ownership and leaks a pre-disconnect claim into a new connection's history (`last_sequence/dispatched_records` survive `disconnect` by design).
**Suggestion**: Decide explicitly: cancel claim on `disconnect()` or reject `dispatch_result` when `state is DISCONNECTED`.

### 2. [critical] `snapshot()` advances/monitors clock and can raise — diagnostics unavailable exactly when needed
**Location**: `joint_controller.py: snapshot()` -> `_now()` -> `_age()`
**Finding**: Read-only `snapshot()` calls `self._now()` when any `received_at is not None`, updating `_now_high_water_s` and raising `ValueError` on invalid/regressed clock.
**Evidence**: Path in `test_controller_expiry.py:test_complete_write_is_recorded_then_invalid_end_clock_faults_joint`: after a `COMPLETE` write with `clock.value=nan|0.5`, `core.snapshot()` raises `ValueError`, caller must manually restore `clock.value=claim_time` before fault/preserved receipt becomes observable. A regressed clock between `receive()` and `snapshot()` also makes `_age()` raise. This couples observation to clock progression and hides the “preserved known write” the intent requires to survive clock faults.
Intent says “snapshots also require that clock contract” per docs, but that makes post-fault forensics depend on fixing the clock first.

### 3. [warning] `tick()` swallows housekeeping cause
**Location**: `joint_controller.py: tick()` vs `submit_target()/prepare_refresh()/claim_dispatch()`
**Finding**: `tick()` runs `self._housekeeping(now)` and discards the returned `DecisionCode`, always returning `ACCEPTED` (except `DISCONNECTED/CONNECTING`).
**Evidence**: `READY/ARMED/MOVING + stale -> _latch_joint_fault -> FAULT`, or `admitted + now>=expires -> STOPPED/FAULT`, yet `tick().accepted is True`. `submit_target`/`prepare_refresh`/`claim_dispatch` propagate the same code as rejection. Caller cannot distinguish no-op vs fault-vs-expiry without an extra `snapshot()`. Verification gap for the “bounded refresh / expiry disarms to STOPPED” claim.

### 4. [warning] `issue_window()` is stateless — spurious issue invalidates prior valid window
**Location**: `joint_controller.py: issue_window()` vs `submit_target()` window check
**Finding**: `issue_window` has no state guard; any caller in any state advances `_next_window_id` and overwrites `_offered_window`.
**Evidence**: `ARMED: w1=issue_window(0.5); submit(Target(...,w1))` accepted. Then `w2=issue_window(...)` (even in `FAULT/DISCONNECTED` or without intent to use). `submit(Target(...,w1))` now hits `target.window.id < offered.id -> EXPIRED`, not `INVALID_ARGUMENT`. A valid admitted goal is unaffected until expiry, but any not-yet-submitted computation on `w1` is retroactively expired by a later issue. Allows DoS of a valid window and conflates “superseded” with “expired” (tests assert `EXPIRED` for old `w0` in `test_max_window...`).

### 5. [warning] Zero-byte / clock-only failures recorded as `DRIVER_FAILURE`
**Location**: `joint_controller.py: _record_dispatch_failure()` <- `dispatch_result()`; `im6014_serial.py: exchange()` final-deadline path
**Finding**: Every non-`COMPLETE` outcome faults with `CommunicationFailure.DRIVER_FAILURE`, regardless of cause.
**Evidence**:
- `exchange` path `before_write()->True` then `_remaining(deadline)` raises before `bus.write` (tested in `test_expiry_after_successful_claim...` and `test_exchange_reports...`): `on_write(FAILED)` -> `dispatch_result(...,FAILED)` -> `_record_dispatch_failure` -> fault `DRIVER_FAILURE`, `dispatch_attempts.result=FAILED`, `write_count==0`. No driver interaction occurred; cause is local deadline, not driver.
- `dispatch_result(COMPLETE)` path where `self._now()` raises after record is already stored: `except Exception: _record_dispatch_failure(); return FAULT_LATCHED`. `dispatch_attempts` stays `COMPLETE` while `faults[0].communication_error is DRIVER_FAILURE` and observation is overwritten with `DRIVER_FAILURE`. Complete-write evidence and fault cause are inconsistent; clock fault is relabeled as driver fault.
Preserves receipt per intent but pollutes attributed-cause taxonomy (`TIMEOUT/INVALID_FEEDBACK/DRIVER_FAILURE` are supposed to be fixed causes).

### 6. [warning] `ValueError` from bus mislabeled + restore masks primary
**Location**: `im6014_serial.py: exchange()` `except ValueError` + `finally` restore
**Finding**: Any `ValueError` inside `try` (including `bus.reset_input_buffer()/read/write` raising `ValueError`, or timeout-setter rejection) is converted to `RuntimeError('Serial timeout configuration failed')`.
**Evidence**: Path `bus.reset_input_buffer()->ValueError` -> caught as timeout-config fault, losing reset/read/write context. `failure` saved as original `ValueError`, but `finally` restore-failure message uses that `failure`, discarding the intermediate `RuntimeError` chain. Tests only assert `RuntimeError` is raised, not that cause is accurate.

### 7. [nit] Duplicated encode-for-validation in three places
**Location**: `joint_controller.py: submit_target()`, `prepare_refresh()`, `test_controller.py: FakeStation.refresh()`
**Finding**: Admission validation (`mapping.to_output()` + `command(...)` bench-limit check) is copy-pasted in `submit_target` and `prepare_refresh`, and again in the test adapter. Drift risk: bench limits (`Kd 0-6/Kp 0-30/torque ±3.5`) or `watchdog/mode` policy must be updated in two core paths.
**Evidence**: Both core methods `try: for spec,jt in zip(...): output=...; command(...) except ValueError: reject/None`. No shared ` _validate_encodable()` helper. Not behavior-breaking today (tests pin numeric anchors), but structural duplication around the C07 “codec representability” gate.
