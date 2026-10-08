### Summary of Changes for Milestone C14

Implemented C14 persistent temperature and continuous effort protection latches with explicit per-joint fixture thresholds, pure transition arithmetic, union fault tracking, and dwell-gated recovery.

#### Authored & Modified Files
- [`joint_protection.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/joint_protection.py): Pure discrete protection records and transition mathematics:
  - `ProtectionPolicy`: Frozen required policy (`driver_trip_c`, `driver_reset_c < trip`, `winding_trip_c`, `winding_reset_c < trip`, `continuous_effort_nm <= max_effort_nm`, `continuous_dwell_s > 0`, `recovery_dwell_s > 0`).
  - `JointProtectionState`: Finite per-joint timers for continuous overload accumulation and healthy Stop recovery dwell.
  - `check_joint_limits()` and `transition_protection()`: Pure limit evaluation and state progression functions.
- [`joint_contract.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/joint_contract.py):
  - Added `DecisionCode.TEMPERATURE_OUT_OF_BOUNDS`.
  - Added required `protection: ProtectionPolicy` to [`JointSpec`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/joint_contract.py#L110).
  - Extended [`MotorHealth`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/joint_contract.py#L256) with required finite real `driver_c` and `winding_c` fields.
  - Extended [`JointFault`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/joint_contract.py#L380) with `driver_temperature_limit`, `winding_temperature_limit`, and `continuous_effort_limit` cause flags and union tracking.
- [`joint_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/joint_controller.py) (937 lines, within the <1000 line cap):
  - Integrated per-joint protection state into core lifecycle.
  - Added measured violation checking to observation receipt, housekeeping, and dispatch gates.
  - Enforced recovery gate in `checked()` requiring explicit acknowledgment (`begin_check(acknowledge_fault=True)`) and contiguous fresh healthy Stop telemetry meeting `recovery_dwell_s`.
- [`test_controller_thermal.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/test_controller_thermal.py): Authored `ThermalPersistentEffortTests` covering:
  - Policy & health validation;
  - Exact trip/reset equality boundaries for driver and winding temperatures;
  - Peak effort immediate trips vs continuous effort dwell accumulation;
  - Fluctuating effort timer reset and telemetry interruption handling;
  - Multi-cause union persistence across Stop, reconnect, healthy packets, and new targets;
  - Acknowledged recovery dwell equality and multi-joint freshness synchronization.
- **Caller Migration**:
  - Migrated all `JointSpec` and `MotorHealth` constructor invocations across [`test_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/test_controller.py), [`test_controller_effort.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/test_controller_effort.py), [`test_controller_expiry.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/test_controller_expiry.py), [`test_controller_lead.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/test_controller_lead.py), [`test_controller_limits.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/test_controller_limits.py), [`test_controller_owner.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/test_controller_owner.py), and [`test_controller_targets.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/test_controller_targets.py).
- **Registration & Documentation**:
  - Registered C14 in [`scripts/check_milestone.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/scripts/check_milestone.py).
  - Added new source/test modules to copyset and milestone tests in [`scripts/check_offline_baseline.sh`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/scripts/check_offline_baseline.sh).
  - Updated fingerprint set in [`test_startup.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/test_startup.py).
  - Narrowly documented C14 semantics in [`docs/joint-api.md`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/docs/joint-api.md).

#### Key Decisions & Contradictions
- Factored pure limit evaluation and transition arithmetic into [`joint_protection.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/joint_protection.py) to avoid duplicating limit checks across lifecycle gates and keep [`joint_controller.py`](file:///tmp/pstack-c07-c16-20261008-smgcfm_v/C14-workbench/joint_controller.py) safely below 1,000 lines (937 lines).
- `continuous_effort_nm` is bounded $\le \text{max\_effort\_nm}$; effort exceeding peak faults immediately, whereas continuous overload accumulates only while fresh valid telemetry arrives without interruption.
- Recovery dwell requires all joints to sustain fresh Stop-mode telemetry below reset thresholds simultaneously; any gap, non-Stop packet, or elevated telemetry resets the recovery timer without clearing latched causes.

#### Verification Note
Per worker policy, provider commands and test runs are parent-owned. All checks are submitted via handoff.

```pstack-handoff
{"task":"C14","checkpoint":"C14-persistent-protection","operation":"run-checks","files":["joint_contract.py","joint_controller.py","joint_protection.py","test_controller.py","test_controller_effort.py","test_controller_expiry.py","test_controller_lead.py","test_controller_limits.py","test_controller_owner.py","test_controller_targets.py","test_controller_thermal.py","test_startup.py","scripts/check_milestone.py","scripts/check_offline_baseline.sh","docs/joint-api.md"],"checks":["C14","milestone-tests","scripts/check_offline_baseline.sh"],"summary":"Verify C14 persistent temperature and continuous effort protection latches"}
```