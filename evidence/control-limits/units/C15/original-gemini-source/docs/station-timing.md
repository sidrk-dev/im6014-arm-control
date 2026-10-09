# Station Timing and Step Coordination

`station_timing.py` and `station_step.py` provide hardware-independent timing instrumentation and synchronous single-writer step coordination for the IM6014 workbench.

## Domain Records and Classes

- `StationTimingConfig(nominal_period_s, capacity=256)`: Nominal period configuration, kept distinct from actual loop measurements.
- `JointTransactionRecord`: Immutable per-joint record correlating:
  - `cycle_index`, `joint_index`, `joint_name`, `motor_id`
  - `plan_id`, `sequence`, `owner`
  - `write_result`: `WriteResult` (COMPLETE, PARTIAL, FAILED, PREWRITE_DEADLINE) or `None`
  - `outcome`: typed string (`OK`, `TIMEOUT`, `INVALID_FEEDBACK`, `WRITE_CANCELLED`, `PREWRITE_DEADLINE`, `DRIVER_FAILURE`, `SKIPPED`, `CLOCK_FAILURE`)
  - `started_s`, `ended_s`, `duration_s`: actual execution timestamps
  - `feedback_received`, `feedback_timestamp_s`, `feedback_age_s`
- `LoopStepRecord`: Immutable per-iteration record capturing:
  - `actual_period_s`: actual start-to-start period ($t_{\text{start}, k} - t_{\text{start}, k-1}$)
  - `execution_duration_s`: actual iteration execution time ($t_{\text{end}} - t_{\text{start}}$)
  - `overrun` and `overrun_s`: positive execution overrun beyond nominal period
  - `overrun_joints`: joint names exceeding fair time budget
  - `missed_slots_skipped`: integer expired slots skipped without catch-up burst
  - `feedback_ages_s`: per-joint feedback age vector
  - `sampling_skew_s`: spread across valid receive timestamps ($\max - \min$)
  - `transactions`: tuple of `JointTransactionRecord`
- `TimingCounters`: Event counters saturating at `UINT64_MAX` ($2^{64} - 1$).
- `StationTimingTracker`: Thread-confined timing tracker with bounded deques and monotonic checks.
- `StationStepCoordinator`: Synchronous single-writer coordinator connecting `JointCore`, transport, and timing tracker. Exercises canonical driver `exchange()` with `before_write` claim gate and `on_write` result reporting.
