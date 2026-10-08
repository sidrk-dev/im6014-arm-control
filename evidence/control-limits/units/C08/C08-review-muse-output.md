## Findings

### 1. [critical] In-flight `_dispatch_claim` survives every revocation and blocks new owner / reconnect
**Location**: `JointCore._revoke_authority`, `claim_owner`, `stop`, `disconnect`, `_latch_joint_fault`, `_withdraw_expired_goal`, `begin_connect`, `begin_check`
**Finding**: `_revoke_authority` clears `current_owner`, `offered_window`, `admitted_target`, `prepared_plan` but never clears `_dispatch_claim`. `begin_connect` and `begin_check` reject when `_dispatch_claim is not None`, `prepare_refresh` returns `None`, and `claim_dispatch` returns `False` while a claim is outstanding.
**Evidence**: Trace: `ARMED` -> `submit` -> `prepare_refresh` -> `claim_dispatch(0)` succeeds -> `stop()` / `claim_owner(new)` / `communication_failed` / `disconnect()` / expiry revokes goal but leaves `_dispatch_claim` set -> `begin_check` returns `INVALID_STATE`, `begin_connect` returns `INVALID_STATE`, new owner's `claim_dispatch(1)` returns `False`, `prepare_refresh` returns `None`. Only `dispatch_result(old_plan)` clears it. `dispatch_result` does not check owner, so new owner's liveness depends on old owner's bus outcome arriving. After `disconnect` with outstanding claim, `DISCONNECTED` cannot `begin_connect` until old receipt arrives. There is no claim timeout/cancel; after revoke `admitted_target` is `None` so housekeeping can no longer expire it. Intent says stop/fault/disconnect/expiry/takeover "revoke authority and pending goals" and docs say invalidation "without discarding that in-flight receipt" plus "blocks another joint claim and plan preparation" — but blocking of `begin_check`/`begin_connect` and cross-owner inheritance is undocumented and makes a hung/missing old receipt brick the core for the new owner.
**Suggestion**: Decide explicitly: either cancel on revoke and reject late receipt, or document that new owner must wait for old bus timeout and provide a bounded cancel/expire for the claim. At minimum clear or expire the claim on `disconnect` and on generation-exhaustion failure.

### 2. [warning] Late receipt for invalidated plan overwrites per-joint record with stale sequence
**Location**: `JointCore.dispatch_result` COMPLETE path, `test_controller_expiry.py::test_invalidated_plan_receipt_records_write_without_reviving_old_goal`
**Finding**: `dispatch_result` always writes `dispatched_records[index]` and `dispatch_attempts[index]` from `claim.plan` even when `still_current` is false. After replacement, old `sequence==1` overwrites the slot while `admitted_target.sequence==2`.
**Evidence**: Test asserts `snapshot.dispatched_records[0].sequence == 1` while `snapshot.admitted_target is replacement(seq2)` and `state is ARMED`. Reader sees current goal 2 but last actual write 1 with no marker that record is stale relative to admitted goal. Same applies across owners: old owner's receipt overwrites slot after takeover when no current goal, which is intended attribution, but when a new goal is already admitted it creates sequence skew. Intent "retain original owner attribution without restoring motion" is met, but record-vs-goal consistency is ambiguous.
**Suggestion**: Keep attribution but avoid silent overwrite confusion — e.g. do not overwrite a slot that already holds a newer sequence, or document that `dispatched_records` is last-bus-write evidence not per-goal progress.

### 3. [warning] `issue_window` has no owner/state gate and can invalidate legitimate window
**Location**: `JointCore.issue_window`
**Finding**: Any caller can call `issue_window` in any state and overwrite `_offered_window`. `submit_target` then rejects old `window.id < offered.id` as `EXPIRED`.
**Evidence**: No `current_owner` check, no state check. Within trusted-station model this is benign, but with owner generations the window is only bound at `submit` time. A second issuer — even same owner — silently invalidates the first client's window. `claim_owner` correctly clears `offered_window`, but concurrent issuers under one owner still race. Future C16 binding will need issue-time binding.
**Suggestion**: Leave behavior for C08 but document that window issuance is trusted-caller only, or bind `offered_window` to `current_owner` at issue time and reject `issue_window` with no owner.

### 4. [warning] Owner check allows subclass with overridden `__eq__`
**Location**: `JointCore.arm`, `JointCore.submit_target`, `JointCore.prepare_refresh`, `JointCore.claim_dispatch`, `joint_contract.Target/OwnerToken` validation
**Finding**: Contract uses `isinstance(owner, OwnerToken)` and core uses `owner != self._current_owner`. A subclass overriding `__eq__` could forge equality.
**Evidence**: `Target.__post_init__` accepts any `isinstance` subclass. `arm(None)` is correctly rejected, and exact-value tests pass, but `class Evil(OwnerToken): def __eq__(...)` would pass `isinstance` and `!=`. Current trust boundary says station callers trusted and C16 will bind connection identity, so not exploitable now, but exact-match intent ("exact connection-bound tokens", "prevents client-supplied epoch/generation from creating authority") calls for exact type plus field comparison.
**Suggestion**: Use `type(token) is OwnerToken` on entry and compare `(epoch,generation,connection_id)` tuple explicitly.

### 5. [warning] Owner gates duplicated in four places with subtly different sources
**Location**: `arm`, `submit_target`, `prepare_refresh`, `claim_dispatch`, `dispatch_result.still_current`
**Finding**: Same concept spelled as `current is None or admitted.owner != current`, `prepared.target.owner != current`, `target.owner != current`, plus extended `still_current` with `prepared is plan and admitted is target and state in (ARMED,MOVING) and not faults and current==plan.owner`.
**Evidence**: Today `prepared.target is admitted_target` when prepared exists, so checks coincide, but drift risk is high. Example: future change to one gate will miss others. `prepare_refresh`/`claim_dispatch` return `None`/`False` silently on owner mismatch, indistinguishable from expiry/health/state.
**Suggestion**: Extract `_has_authority_for(token)->bool` helper and use everywhere; keep silent `None`/`False` for API compat but centralize logic.

### 6. [warning] Exhaustion check happens after revocation, destroying good session on failed claim
**Location**: `JointCore.claim_owner`
**Finding**: `connection_id` format is validated before revoke (good), but `next_generation > 2**64-1` is checked after `_revoke_authority` and state demotion. Failed claim leaves `current_owner=None`, `offered_window=None`, `admitted=None`, `STOPPED`/`FAULT`.
**Evidence**: `test_exhausted_generation_fails_closed` codifies this: valid `tok_max` session in `MOVING` is destroyed by `claim_owner("overflow")` raising `OverflowError`. With 2**64 this is unreachable in practice, but validation-order principle is violated: failed admission should not mutate. Intent says exhaustion revokes, so test matches intent, but availability would be better preserved by pre-checking exhaustion before revoking and then failing closed via explicit stop if policy requires it.
**Suggestion**: Pre-check exhaustion (and any future admission preconditions) before `_revoke_authority`, or document that any `claim_owner` attempt — even failing — intentionally kills the session.

### 7. [warning] `claim_owner` state demotion expression is hard to audit
**Location**: `JointCore.claim_owner: stop_for_claim`
**Finding**: `stop_for_claim = state in (CHECKING,READY,ARMED,MOVING) or (was_owned and state not in (DISCONNECTED,CONNECTING))` plus `if DISCONNECTED/CONNECTING: pass elif FAULT or faults: FAULT elif stop_for_claim: STOPPED` is correct per tests but tangled.
**Evidence**: Second disjunct already covers `CONNECTED,STOPPED,FAULT` when owned; first covers unowned `CHECKING/READY/ARMED/MOVING`. `CONNECTING` preservation is implemented but docs "Core and ownership limits" only mention preserving `FAULT`/`DISCONNECTED`, omitting `CONNECTING`. Reader must enumerate 9 states to verify.
**Suggestion**: Table-driven transition or explicit comment enumerating owned vs unowned per state; fix docs to include `CONNECTING`.

### 8. [warning] Bounded-storage test verifies names, not bound
**Location**: `test_controller_owner.py::test_bounded_storage_after_many_claims`
**Finding**: Asserts absence of `_owner_history`, `_owners`, `claim`, `generation`, `latest_generation` and identity of two token refs.
**Evidence**: Passes with current O(1)+O(joints) storage (`latest/current` plus per-joint `dispatched_records`/`dispatch_attempts` owners), but would also pass with history stored under another name. Does not measure growth. Tests are stated as evidence not verdict, but this one locks implementation-detail names.
**Suggestion**: Assert behavioral bound — e.g. snapshot size / `gc` object count stable across 10k claims — or drop name checks.

### 9. [nit] Minor boundary/quality issues
**Location**: `joint_controller.py`, `joint_contract.py`, tests, docs
**Finding**:
- `connection_id` with leading/trailing spaces (`" a "`) passes `strip()` check but is stored verbatim; equality then requires exact spaces.
- `_epoch = secrets.token_hex(16)` generated before joint-spec validation; invalid constructor args still consume entropy.
- `test_controller_owner.py` imports `command`, `MotorHealth`; `test_controller_expiry.py` imports `MotorHealth`, `OwnerToken` without direct use.
- Class and method docstrings repeat identical trusted-station/C16 paragraph.
**Evidence**: Direct reading; no behavior break.
**Suggestion**: Normalize or reject padded IDs, validate specs before epoch, drop unused imports, deduplicate trust-boundary comment.

No findings in `joint_mapping.py`, `im6014_codec.py`, `im6014_serial.py` for C08 scope — diff shows no changes there and owner/window logic does not alter mapping/codec representability or exchange deadline semantics.
