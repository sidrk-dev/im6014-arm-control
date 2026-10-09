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

## Clock and cycle ordering

The tracker has one monotonic high-water mark across loop cycles and per-joint transactions, including the Stop queries performed by `check_step()`. A transaction cannot start before the prior recorded event ended, a cycle cannot end before any recorded transaction ended, and `start_cycle()` rejects a second start while a cycle remains active. Rejected timestamps do not advance the high-water mark or update counters.

Overrun and missed-slot boundaries use exact arithmetic over the accepted finite float values. Missed-slot counts saturate at `UINT64_MAX`, while the next due boundary is computed from the full missed count. If that future boundary cannot be represented as a finite float strictly greater than the current time, the scheduler raises `ValueError` rather than rewinding or returning infinity.

`StationStepCoordinator.run_steps()` uses `time.sleep` when no `sleep_fn` is supplied. It checks the monotonic clock again after every wake, records actual start-to-start periods, and never runs catch-up bursts. Each next due time is anchored to the last actual cycle start plus the nominal period; a late start counts expired slots and schedules the following cycle from that actual start.

## Failure and feedback accounting

After each exchange, the coordinator retains the driver's actual `WriteResult` and checks the `Decision` returned by `dispatch_result()`. A completed write followed by a core clock failure remains recorded as a complete write and a `CLOCK_FAILURE`; the coordinator stops the cycle without accepting the reply as feedback. Read timeouts and invalid feedback are sent to `JointCore.communication_failed()` with their typed cause. A pure pre-write cancellation remains `WRITE_CANCELLED` and is not reported as a motor communication failure. Later mode-1 writes are skipped after a cycle failure.

The `feedback_ages_s` vector comes from the core snapshot and therefore uses the core's clock. A coordinator may use a distinct clock for transaction and loop timestamps. `sampling_skew_s` is computed from successful feedback receives in the current cycle only; old observations retained by the core do not contribute, and fewer than two current-cycle receives produce `None`. An idle cycle has no receive samples and reports `None` skew while keeping the full snapshot age vector.

## Benchmark fixture

Run `python scripts/benchmark_station_timing.py` from any working directory; the script adds its workbench root before importing project modules. The default is five repeats of 50 cycles. At least five repeats are required unless `--profile` is supplied; cycles, total run size, and nominal period have finite upper bounds. The JSON receipt separates setup Stop queries, mode-1 work, and shutdown Stop queries, and includes raw transaction correlation, observed feedback ages and skew, driver read/write counts, failures, and final core state. Failed or missing work produces a nonzero exit status. The in-memory transport measures host software and scheduler behavior only; it does not establish physical bus or actuator timing.
