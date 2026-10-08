### Authored and Modified Files

- [`joint_contract.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_contract.py): Extracted and defined frozen domain records: [`IntentWindow`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_contract.py#L90-L95), [`Target`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_contract.py#L98-L102) (requiring non-optional `window`), [`RefreshPlan`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_contract.py#L105-L111), [`JointDispatchedRecord`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_contract.py#L114-L120), [`Snapshot`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_contract.py#L123-L135) (distinguishing admitted target, prepared plan, dispatched per-joint records, and telemetry), enum [`WriteResult`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_contract.py#L42-L46) (`COMPLETE`, `PARTIAL`, `FAILED`), and decision code `DecisionCode.EXPIRED`.
- [`joint_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_controller.py): Implemented monotonic clock validation with persistent `_now_high_water_s`, window issuance ([`issue_window`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_controller.py#L106-L121) bounded by `max_window_s`), target admission against exact offered window, atomic three-stage write cycle ([`prepare_refresh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_controller.py#L210-L245), [`claim_dispatch`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_controller.py#L247-L290), [`dispatch_result`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/joint_controller.py#L292-L320)), goal expiration disarm to `STOPPED` (latched fault preserved), plan invalidation, and removal of legacy `consume_target`.
- [`im6014_serial.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/im6014_serial.py): Added optional keyword hooks `before_write` and `on_write` in [`exchange`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/im6014_serial.py#L110-L167) and [`capture_exchange`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/im6014_serial.py#L170-L219). `before_write` executes after buffer reset and timeout configuration, directly beside `bus.write(packet)`; cancellation raises `WriteCancelledError` with zero wire bytes sent.
- [`test_controller_expiry.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/test_controller_expiry.py): Authored 15 unit tests covering exact expiry boundary rejection, client TTL forgery prevention, multi-joint write interruption on expiry, cancellation on clock advance during buffer flush, repeated refresh bounds, invalid sequence handling, and clock regression protection before observation replacement.
- [`test_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/test_controller.py) & [`test_controller_targets.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/test_controller_targets.py): Updated `FakeStation` and target tests to supply required `IntentWindow` and use truthful `refresh()` semantics.
- [`scripts/check_milestone.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/scripts/check_milestone.py), [`scripts/check_offline_baseline.sh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/scripts/check_offline_baseline.sh), & [`test_startup.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/test_startup.py): Registered milestone check target `'C07': 'test_controller_expiry'` and updated fingerprint/baseline copysets.
- [`docs/joint-api.md`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/docs/joint-api.md) & [`docs/architecture.md`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C07-workbench/docs/architecture.md): Documented C07 contracts, expiry semantics, prepared-vs-dispatched state tracking, and module boundaries.

---

### Key Decisions

1. **Station Domain Expiry & Window Binding**: Windows are minted exclusively by `JointCore.issue_window(valid_for_s)` using monotonic timestamps. Submitted targets must match the latest offered window ID and bounds exactly; forging or extending expiry boundaries triggers immediate rejection (`DecisionCode.EXPIRED`). Boundary condition `now == expires_at_s` is strictly rejected.
2. **Clock Regression and Emergency Disarm**: Global monotonic high-water mark `_now_high_water_s` persists across reconnects. Clock regression raises an error before mutating observation timestamps in `receive()`. Conversely, `JointCore.stop()` does not sample the clock, guaranteeing emergency stopping works even if the time source fails.
3. **Prepared vs. Dispatched Separation**: `prepare_refresh()` validates wire mapping and forms a single immutable `RefreshPlan`. `claim_dispatch(plan_id, index)` strictly validates per-joint order and freshness immediately before writing. Only successful writes recorded via `dispatch_result()` advance a joint's dispatched record and logical `MOVING` state. Partial/failed writes latch `DRIVER_FAILURE` and cancel remaining plan dispatches without retrying writes.
4. **Repeated Refresh Semantics**: An admitted goal remains valid and can be refreshed until its original window expires. Issuing new windows or rejecting invalid requests preserves the current valid goal and sequence watermark. When the goal window expires, internal housekeeping invalidates plans and disarms to `STOPPED` (retaining any latched fault).

---

### Checks for Parent Execution

1. **Milestone C07 Verification**:
   ```bash
   .venv/bin/python scripts/check_milestone.py C07
   ```
2. **Offline Regression Baseline**:
   ```bash
   ./scripts/check_offline_baseline.sh
   ```
3. **Direct Unit Test Suites**:
   ```bash
   .venv/bin/python -m unittest -v test_controller test_controller_targets test_controller_expiry test_startup
   ```