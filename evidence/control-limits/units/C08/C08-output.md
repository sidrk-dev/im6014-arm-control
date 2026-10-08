### Summary of Changes

#### Changed Paths
- [`joint_contract.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/joint_contract.py): Defined frozen `OwnerToken(epoch, generation, connection_id)` with type and bound validation. Updated [`Target`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/joint_contract.py#L32), [`JointDispatchedRecord`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/joint_contract.py#L42), and [`JointDispatchAttempt`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/joint_contract.py#L48) to require `owner: OwnerToken`. Added `owner` and `current_owner` fields and generation aliases to [`Snapshot`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/joint_contract.py#L65).
- [`joint_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/joint_controller.py): Implemented process-fresh unpredictable `epoch` via stdlib `secrets.token_hex(16)`, monotonic `uint64` generation counter with fail-closed non-wrapping exhaustion, explicit trusted station claim via `claim_owner(connection_id)`, required `owner: OwnerToken` on `arm(owner)`, full token equality matching on admission and dispatch claim, strict O(1) owner storage, and single-helper authority revocation on stop, fault, disconnect, expiry, and takeover. Total line count remains at 711 lines (< 1,000 limit).
- [`docs/joint-api.md`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/docs/joint-api.md): Documented C08 Owner Generations, trusted station caller boundaries, token structure, and revocation semantics.
- [`test_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/test_controller.py): Updated `FakeStation` helper lifecycle with `claim(connection_id)` and migrated all core/station `arm(owner)` calls to supply explicit tokens.
- [`test_controller_targets.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/test_controller_targets.py) & [`test_controller_expiry.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/test_controller_expiry.py): Migrated all `Target` constructions and `arm` calls across existing test fixtures to supply valid `OwnerToken` instances.
- [`test_controller_owner.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/test_controller_owner.py): Authored targeted C08 test suite covering token immutability, delayed generation rejection, connection/epoch mismatch rejections, takeover joint blocking, in-flight write recording without moving after revoke, generation exhaustion, bounded O(1) storage, and preservation of unexpired goals against high-sequence stale owner attempts.
- [`scripts/check_milestone.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/scripts/check_milestone.py): Registered `'C08': 'test_controller_owner'`.
- [`scripts/check_offline_baseline.sh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/scripts/check_offline_baseline.sh): Added `test_controller_owner.py` to baseline file copyset and `test_controller_owner` to milestone tests.
- [`test_startup.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C08-workbench/test_startup.py): Added `test_controller_owner.py` to source fingerprint.

---

### Key Decisions
1. **Unpredictable Epoch & uint64 Monotonicity**: Process-fresh epoch generated via `secrets.token_hex(16)` with no persisted global state. Generation increments monotonically on each claim; counter exhaustion fails closed at `2^64 - 1` without wrapping or resetting.
2. **Strict Authority Revocation**: A centralized `_revoke_authority()` helper revokes `_current_owner` and clears active goals, plans, and offered windows without dropping `_last_sequence` or reusing epochs. Revocation occurs on Stop, fault, disconnect, expired goal, and takeover.
3. **Exact Token Matching**: Both admission and final dispatch claim compare `target.owner == self._current_owner`. Incoming stale or forged tokens are rejected without mutating healthy goals or sequence counters.
4. **Takeover and In-flight Safety**: Explicit takeover enters `STOPPED` (or retains `FAULT`), requiring fresh Stop check and explicit arm with the new token. Writes completing in flight during revocation record the original dispatch attempt/record accurately without setting `MOVING`.
5. **Bounded O(1) Storage**: Retains only `_latest_owner` and `_current_owner` with no unbounded history or mapping collections.

---

### Recommended Parent Checks
1. `scripts/check_milestone.py C08`
2. `scripts/check_offline_baseline.sh`

```pstack-handoff
{"task":"C08","checkpoint":"C08","operation":"run-checks","files":["docs/joint-api.md","joint_contract.py","joint_controller.py","scripts/check_milestone.py","scripts/check_offline_baseline.sh","test_controller.py","test_controller_expiry.py","test_controller_owner.py","test_controller_targets.py","test_startup.py"],"checks":["scripts/check_milestone.py C08","scripts/check_offline_baseline.sh"],"summary":"Execute offline milestone C08 check and offline baseline regression check for owner generations"}
```