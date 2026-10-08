# Adversarial Code Review: Milestone C08 Ownership Generations & Tokens

**Review Target**: Milestone C08 (`OwnerToken`, core generation tracking, authority revocation, and attributed dispatch records)  
**Assigned Workspace**: `/tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench`  
**Inspected Files**:
- [`joint_contract.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py)
- [`joint_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py)
- [`im6014_serial.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/im6014_serial.py)
- [`im6014_codec.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/im6014_codec.py)
- [`joint_mapping.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_mapping.py)
- [`test_controller_owner.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_controller_owner.py)
- [`test_controller_expiry.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_controller_expiry.py)
- [`test_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_controller.py)
- [`test_controller_targets.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_controller_targets.py)
- [`docs/joint-api.md`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/docs/joint-api.md)
- [`docs/controller-limits-design.md`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/docs/controller-limits-design.md)

---

## Executive Summary

The C08 changes introduce immutable [`OwnerToken`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L225-L235) instances, uint64 monotonic generations, random process epochs, and owner attribution across [`Target`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L267-L284), [`JointDispatchedRecord`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L323-L345), and [`JointDispatchAttempt`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L349-L376).

While the happy paths and regression tests pass, adversarial inspection reveals **2 critical lifecycle/safety flaws**, **3 state integrity and boundary design issues**, and **3 maintainability and test smell concerns**. Crucially, `dispatch_result` falsely reports `ACCEPTED` when safety housekeeping latches a fault, `disconnect()` leaks active claims and allows subsequent failed writes to mutate a disconnected core into a latched `FAULT` state, and ownership tokens can be claimed on disconnected cores before any motor connection is established.

---

## Findings

### 1. [Critical] [`JointCore.dispatch_result`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L606-L668) Returns `ACCEPTED` When Stale-Feedback Fault Latches

**Location**: [`joint_controller.py:647-660`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L647-L660)  
**Symbol**: [`JointCore.dispatch_result`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L606-L668)

#### Mechanism
When a write completes (`result is WriteResult.COMPLETE`), [`JointCore.dispatch_result`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L606-L668) commits the record and calls [`_housekeeping(now)`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L280-L301):
```python
housekeeping = self._housekeeping(now)
still_current = (
    self._prepared_plan is plan
    and self._admitted_target is plan.target
    and self._state in (State.ARMED, State.MOVING)
    and not self._faults
    and self._current_owner is not None
    and self._current_owner == plan.target.owner
)
if still_current and housekeeping is None:
    self._next_dispatch_index = index + 1
    self._state = State.MOVING
return self._accepted()
```
If feedback aged out during write execution:
1. [`_housekeeping`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L280-L301) invokes [`_latch_joint_fault(..., stale=True)`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L213-L240), latches faults into `self._faults`, revokes authority via [`_revoke_authority()`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L148-L152), and sets `self._state = State.FAULT`.
2. `still_current` evaluates to `False`. The core does not transition to `MOVING`.
3. **Execution falls through and unconditionally returns `self._accepted()` (`Decision(True, DecisionCode.ACCEPTED)`).**

#### Impact & Inconsistency
The calling station or supervisor receives `accepted == True`. The caller is told that dispatch succeeded normally, while internally the controller has entered `State.FAULT`, latched hardware faults, and revoked owner authority.

Furthermore, this directly contradicts the error handling three lines earlier in lines [641-645](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L641-L645):
```python
except Exception:
    self._record_dispatch_failure(index, CommunicationFailure.CLOCK_FAILURE)
    return self._rejected(DecisionCode.FAULT_LATCHED)
```
When clock validation fails, `dispatch_result` returns `FAULT_LATCHED`. When stale feedback trips a safety fault during housekeeping, `dispatch_result` returns `ACCEPTED`.

#### Suggestion
Propagate the housekeeping decision or return `FAULT_LATCHED` when a fault was latched:
```python
if self._state is State.FAULT or bool(self._faults):
    return self._rejected(DecisionCode.FAULT_LATCHED)
```

---

### 2. [Critical] [`JointCore.disconnect`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L680-L685) Leaves `_dispatch_claim` Active, Allowing Late Write Failures to Transition `DISCONNECTED` $\to$ `FAULT` and Wedge Reconnection

**Location**: [`joint_controller.py:680-685`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L680-L685), [`joint_controller.py:196-211`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L196-L211), and [`joint_controller.py:329-333`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L329-L333)  
**Symbols**: [`JointCore.disconnect`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L680-L685), [`JointCore._record_dispatch_failure`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L196-L211), [`JointCore.begin_connect`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L329-L333)

#### Mechanism
1. Client claims a write for joint 0: [`JointCore.claim_dispatch`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L574-L604) sets `self._dispatch_claim`.
2. Station or client calls [`JointCore.disconnect()`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L680-L685):
   ```python
   def disconnect(self) -> Decision:
       self._state = State.DISCONNECTED
       self._source = None
       self._clear_observations()
       self._revoke_authority()
       return self._accepted()
   ```
   `_revoke_authority()` clears `_current_owner`, `_offered_window`, `_admitted_target`, and `_prepared_plan`, but **leaves `self._dispatch_claim` intact**.
3. If [`begin_connect()`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L329-L333) is called before the serial exchange concludes, line 330 rejects:
   ```python
   if self._state is not State.DISCONNECTED or self._dispatch_claim is not None:
       return self._rejected(DecisionCode.INVALID_STATE)
   ```
4. When the in-flight serial write finishes:
   - **On `WriteResult.COMPLETE`**: [`dispatch_result`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L606-L668) records `JointDispatchedRecord` and returns `DecisionCode.ACCEPTED` while `self._state` is `DISCONNECTED`.
   - **On `WriteResult.FAILED` or `PREWRITE_DEADLINE`**: [`_record_dispatch_failure`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L196-L211) invokes [`_latch_joint_fault`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L213-L240), setting:
     ```python
     self._state = State.FAULT
     ```
     **The controller state mutates from `State.DISCONNECTED` to `State.FAULT`.**

#### Impact
A disconnected controller is now in `State.FAULT`. Because [`begin_connect()`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L329-L333) requires `State.DISCONNECTED`, reconnection is permanently blocked (`INVALID_STATE`) until the caller invokes `disconnect()` a second time. Even then, the latched fault persists in `self._faults`, forcing the next session to provide `acknowledge_fault=True` on [`begin_check`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L345-L356).

#### Suggestion
Explicitly clear or invalidate `self._dispatch_claim = None` in [`JointCore.disconnect()`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L680-L685), or disallow `dispatch_result` from mutating state into `FAULT` once `DISCONNECTED`.

---

### 3. [Medium] [`JointCore.claim_owner`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L154-L194) Permits Pre-Connection Ownership Claims Across the Transport Boundary

**Location**: [`joint_controller.py:168-181`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L168-L181) and [`joint_controller.py:335-343`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L335-L343)  
**Symbols**: [`JointCore.claim_owner`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L154-L194), [`JointCore.connected`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L335-L343)

#### Mechanism
In [`claim_owner`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L154-L194):
```python
was_owned = self._current_owner is not None
stop_for_claim = self._state in (
    State.CHECKING,
    State.READY,
    State.ARMED,
    State.MOVING,
) or (was_owned and self._state not in (State.DISCONNECTED, State.CONNECTING))
self._revoke_authority()
if self._state in (State.DISCONNECTED, State.CONNECTING):
    pass
elif self._state is State.FAULT or self._faults:
    self._state = State.FAULT
elif stop_for_claim:
    self._state = State.STOPPED
```
Line 175 explicitly allows claiming owner in `DISCONNECTED` and `CONNECTING`. The newly generated [`OwnerToken`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L225-L235) is assigned to `self._current_owner`.
When [`connected(source)`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L335-L343) is subsequently called:
```python
def connected(self, source: SourceIdentity) -> Decision:
    if self._state is not State.CONNECTING:
        return self._rejected(DecisionCode.INVALID_STATE)
    if not isinstance(source, SourceIdentity):
        return self._rejected(DecisionCode.INVALID_SOURCE)
    self._clear_observations()
    self._source = source
    self._state = State.CONNECTED
    return self._accepted()
```
`connected()` clears observations and sets `_source`, but **does not clear `self._current_owner`**.

#### Impact
A client can claim generation 1 before establishing communication with any motor hardware. The core connects to `SourceIdentity A`, arms, and executes goals under that pre-connection token. If `disconnect()` is called and a new motor bus `SourceIdentity B` is connected, claiming before connection allows authority to cross physical boundaries without being tied to the connected lifecycle.

This conflicts with [`docs/joint-api.md`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/docs/joint-api.md#L97-L101):
> *"Reconnecting requires claiming a fresh generation."*
> *"Connect, check, and arm are distinct steps."*

#### Suggestion
Disallow `claim_owner` in `DISCONNECTED` and `CONNECTING` by raising `INVALID_STATE` (or `ValueError`), or have [`connected()`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L335-L343) explicitly invoke `_revoke_authority()`.

---

### 4. [Medium] Incomplete Relational Validation in [`Snapshot`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L380-L457) Permits Impossible State Combinations

**Location**: [`joint_contract.py:450-455`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L450-L455)  
**Symbol**: [`Snapshot.__post_init__`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L396-L455)

#### Mechanism
In [`Snapshot.__post_init__`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L396-L455):
```python
if self.latest_owner is not None and not isinstance(self.latest_owner, OwnerToken):
    raise ValueError("latest_owner must be an OwnerToken or None")
if self.current_owner is not None and not isinstance(self.current_owner, OwnerToken):
    raise ValueError("current_owner must be an OwnerToken or None")
```
No relational constraints exist between `latest_owner` and `current_owner`. Consequently:
1. `Snapshot(..., current_owner=tok, latest_owner=None)` is valid, representing active ownership with no issued ownership history.
2. `Snapshot(..., current_owner=tok_a, latest_owner=tok_b)` (where `tok_a != tok_b`) is accepted, representing a core where the currently authorized owner is not the latest owner issued.

#### Impact
In [`JointCore`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L45), `_current_owner` can only ever be `None` or identical to `_latest_owner`. By omitting relational validation in the frozen contract, deserialized snapshots or mock fixtures can construct corrupt invariants that downstream station consumers cannot reconcile.

#### Suggestion
Add validation:
```python
if self.current_owner is not None and self.current_owner != self.latest_owner:
    raise ValueError("current_owner must match latest_owner when present")
```

---

### 5. [Medium] [`JointCore.prepare_refresh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L526-L572) Sequence Exhaustion Leaves Core In `MOVING` Without Explicit Disarm

**Location**: [`joint_controller.py:559-563`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L559-L563)  
**Symbol**: [`JointCore.prepare_refresh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L526-L572)

#### Mechanism
In [`prepare_refresh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L526-L572):
```python
plan_id = self._next_plan_id
if plan_id > (1 << 64) - 1:
    raise OverflowError("plan ID sequence exhausted")
self._next_plan_id += 1
```
In contrast to [`claim_owner`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L154-L194) (which executes `_revoke_authority()` and stops active motion before raising `OverflowError`), [`prepare_refresh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L526-L572) raises `OverflowError` immediately.
When this raises:
- The core remains in `State.MOVING` or `State.ARMED`.
- `_admitted_target` remains active.
- `_current_owner` retains authority.
- `_prepared_plan` retains the previously generated plan.

#### Impact
The controller does not fail closed into `STOPPED` upon plan sequence exhaustion. It relies on ambient unhandled exception handling and intent window expiry rather than active disarming.

#### Suggestion
Invoke `_revoke_authority()` and transition to `State.STOPPED` before raising `OverflowError`.

---

### 6. [Low] Unsanitized Whitespace in [`OwnerToken.connection_id`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L225-L235) Permitted by Loose Validation

**Location**: [`joint_contract.py:234-235`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L234-L235) and [`joint_controller.py:161-166`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L161-L166)  
**Symbols**: [`OwnerToken.__post_init__`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L229-L235), [`JointCore.claim_owner`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L154-L194)

#### Mechanism
`claim_owner` and `OwnerToken` check `not connection_id.strip()`, but do not strip the value or reject strings containing unstripped whitespace:
```python
if type(self.connection_id) is not str or not self.connection_id.strip() or len(self.connection_id) > 64:
    raise ValueError("connection_id must be a bounded non-empty string")
```
Passing `"station-client "` (with whitespace) passes validation. The token stores `"station-client "`. In equality checks:
`OwnerToken(epoch, gen, "station-client ") != OwnerToken(epoch, gen, "station-client")`.

#### Impact
Trivial whitespace discrepancies between client JSON messages and station binding strings will cause silent equality mismatches (`INVALID_OWNER`) that are visually indistinguishable in inspection logs.

#### Suggestion
Strictly enforce `if self.connection_id != self.connection_id.strip(): raise ValueError(...)`.

---

### 7. [Low / Maintainability] Triplicated Output Codec Encoding Pipeline

**Location**: [`joint_controller.py:502-516`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L502-L516), [`joint_controller.py:543-557`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L543-L557), and [`test_controller.py:79-93`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_controller.py#L79-L93)

#### Mechanism
The loop converting joint targets through mapping and validating bench limits via `command(...)` is verbatim duplicated across:
1. [`submit_target`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L464-L524) (validation before admission)
2. [`prepare_refresh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_controller.py#L526-L572) (validation before plan preparation)
3. `FakeStation.refresh` (packet creation for dispatch)

#### Impact
Encoded byte frames are generated and discarded twice per target refresh cycle before being encoded a third time for serial dispatch. If bench limits or mode policies change, discrepancies across the three loops create drift risks.

#### Suggestion
Extract a private helper `_validate_encodable(specs, joint_targets)` or cache the canonical encoded commands inside [`RefreshPlan`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/joint_contract.py#L288-L318).

---

### 8. [Low / Maintainability] Brittle White-Box Attribute Asserts in Targeted Tests

**Location**: [`test_controller_owner.py:379-385`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_controller_owner.py#L379-L385)  
**Symbol**: [`ControllerOwnerTests.test_bounded_storage_after_many_claims`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_controller_owner.py#L368-L386)

#### Mechanism
In `test_bounded_storage_after_many_claims`:
```python
self.assertFalse(hasattr(core, "_owner_history"))
self.assertFalse(hasattr(core, "_owners"))
self.assertFalse(hasattr(core, "claim"))
self.assertIs(core._latest_owner, snap.latest_owner)
self.assertFalse(hasattr(snap, "generation"))
self.assertFalse(hasattr(snap, "latest_generation"))
```
The test inspects private implementation details rather than verifying bounded behavior via memory inspection or public snapshot contracts.

#### Impact
These assertions exist solely as negative assertions against names rejected during earlier drafting revisions. They provide no guarantee that another unbounded structure (e.g. `_claims`) is not introduced under a different name.

---

## Verification Summary

Inspection of the baseline harness confirms:
- **Baseline Scripts**: [`scripts/check_milestone.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/scripts/check_milestone.py#L22) and [`scripts/check_offline_baseline.sh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/scripts/check_offline_baseline.sh#L81) correctly register `C08` and [`test_controller_owner.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_controller_owner.py).
- **Source Fingerprint**: [`test_startup.py:114`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_startup.py#L114) includes `test_controller_owner.py`.
- **Targeted Test Coverage**: All 13 test cases in [`ControllerOwnerTests`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-review-workbench/test_controller_owner.py#L30) cover generation increments, isolation, and takeover boundaries. However, none of the existing tests exercise stale feedback occurring simultaneously with `dispatch_result(COMPLETE)` or the disconnect-with-active-claim state leak identified above.