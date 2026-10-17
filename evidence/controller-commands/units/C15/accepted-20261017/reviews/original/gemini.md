# C15 Workbench Artifact Independent Review

## 1. Scope & Verification Basis

An independent review was performed of the C15 timing telemetry addition in [`/tmp/c15-codex-20261017`](file:///tmp/c15-codex-20261017) against accepted C14 (`f43d7aae87b01f8c41ae3be89491ad91d35973c4`).

The audit verified:
- [`station_timing.py`](file:///tmp/c15-codex-20261017/station_timing.py): [`StationTiming`](file:///tmp/c15-codex-20261017/station_timing.py#L4-L84) class implementation, slots, monotonicity checks, and snapshot generation.
- [`arm_manual.py`](file:///tmp/c15-codex-20261017/arm_manual.py): [`ArmManualSession.__init__`](file:///tmp/c15-codex-20261017/arm_manual.py#L248-L260), [`ArmManualSession.snapshot`](file:///tmp/c15-codex-20261017/arm_manual.py#L1000-L1015), [`ArmManualSession._transact_raw`](file:///tmp/c15-codex-20261017/arm_manual.py#L1028-L1064), and [`ArmManualSession._loop`](file:///tmp/c15-codex-20261017/arm_manual.py#L2195-L2205).
- [`arm_dashboard.py`](file:///tmp/c15-codex-20261017/arm_dashboard.py): `TOP_LEVEL_FIELDS`, `TIMING_FIELDS`, and `_project_fields()` allowlist filtering.
- Pipeline and registration scripts: [`scripts/check_milestone.py`](file:///tmp/c15-codex-20261017/scripts/check_milestone.py#L29), [`scripts/check_offline_baseline.sh`](file:///tmp/c15-codex-20261017/scripts/check_offline_baseline.sh#L81-L101), [`test_startup.py`](file:///tmp/c15-codex-20261017/test_startup.py#L112-L113), [`test_station_timing.py`](file:///tmp/c15-codex-20261017/test_station_timing.py#L1-L115), and [`tests/test_arm_timing.py`](file:///tmp/c15-codex-20261017/tests/test_arm_timing.py#L1-L188).
- Concurrency, error masking, cancellation paths, and hardware safety invariants.

---

## 2. Concrete Finding

### [P2] End-of-exchange observation timestamp is sampled under lock contention, conflating lock wait with bus exchange latency

- **Location**: [`arm_manual.py:1049-1056`](file:///tmp/c15-codex-20261017/arm_manual.py#L1049-L1056) in [`ArmManualSession._transact_raw`](file:///tmp/c15-codex-20261017/arm_manual.py#L1028-L1064):
  ```python
  finally:
      with self.lock:
          try:
              self._timing.transaction_finished(
                  started_ns, self._timing_clock_ns(), succeeded=succeeded,
              )
          except Exception:
              self._timing.transaction_unavailable(succeeded=succeeded)
  ```
- **Problem**:
  In [`_transact_raw`](file:///tmp/c15-codex-20261017/arm_manual.py#L1028-L1064), `capture_exchange` runs outside of `self.lock`. When the exchange returns (or raises an exception), control enters the `finally` block. However, `self._timing_clock_ns()` is evaluated **inside** the `with self.lock:` statement.
  
  If a concurrent thread (e.g. an HTTP reader servicing `GET /api/state` via [`ArmManualSession.snapshot`](file:///tmp/c15-codex-20261017/arm_manual.py#L1000-L1015), or a command submitter invoking `request()`) is currently holding `self.lock`, the owner thread blocks waiting to acquire `self.lock`. That post-exchange lock-wait time is included in `self._timing_clock_ns() - started_ns`.

  Per [`docs/station-timing.md`](file:///tmp/c15-codex-20261017/docs/station-timing.md):
  - `transaction_latency_s` is documented as the exchange-attempt duration *excluding* queue/scheduling wait.
  - `loop_period_s` is documented as including dispatch, I/O, and waiting.
  
  Evaluating `self._timing_clock_ns()` after waiting for `self.lock` pollutes `transaction_latency_s` and `transaction_max_s` with application lock-contention latency.

- **Reproducible Scenario**:
  1. The owner thread begins an exchange at $t_0$, sampling `started_ns`.
  2. The exchange executes and completes at $t_0 + 2\text{ ms}$.
  3. Meanwhile, a concurrent worker thread handling an HTTP request enters [`ArmManualSession.snapshot`](file:///tmp/c15-codex-20261017/arm_manual.py#L1000-L1015), acquiring `self.lock` and holding it for $30\text{ ms}$.
  4. The owner thread enters `finally` and blocks on `with self.lock:`.
  5. At $t_0 + 32\text{ ms}$, the lock is released and acquired by the owner thread.
  6. `self._timing_clock_ns()` executes, measuring an ending timestamp of $t_0 + 32\text{ ms}$.
  7. `transaction_latency_s` records $0.032\text{ s}$ ($32\text{ ms}$) instead of the actual $0.002\text{ s}$ ($2\text{ ms}$) exchange duration.

- **Recommended Remediation**:
  Sample the ending clock timestamp immediately upon entering `finally` before acquiring `self.lock`, matching the isolated error handling of `started_ns`:
  ```python
  finally:
      try:
          ended_ns = self._timing_clock_ns()
      except Exception:
          ended_ns = None
      with self.lock:
          try:
              self._timing.transaction_finished(
                  started_ns, ended_ns, succeeded=succeeded,
              )
          except Exception:
              self._timing.transaction_unavailable(succeeded=succeeded)
  ```

---

## 3. Systematic Invariant Checks

1. **Mutable-State & Lock Ownership**:
   - [`StationTiming`](file:///tmp/c15-codex-20261017/station_timing.py#L4-L84) defines `__slots__` and stores scalar counters and one timestamp anchor. It maintains fixed $O(1)$ memory without growing traces or mutable collections.
   - All accesses to `self._timing` within [`ArmManualSession`](file:///tmp/c15-codex-20261017/arm_manual.py#L248) (`_loop`, `_transact_raw`, and `snapshot`) are guarded by `self.lock` (`threading.RLock`).
   - [`StationTiming.snapshot`](file:///tmp/c15-codex-20261017/station_timing.py#L71-L84) constructs a fresh dictionary; internal attributes are not exposed for caller mutation.

2. **Exchange Paths, Failures & Pre-Write Cancellations**:
   - `capture_exchange` is invoked only from [`_transact_raw`](file:///tmp/c15-codex-20261017/arm_manual.py#L1028-L1064).
   - `succeeded` is initialized to `False` and only flipped to `True` when `capture_exchange` completes without raising.
   - If `before_write` cancels an exchange ([`WriteCancelledError`](file:///tmp/c15-codex-20261017/im6014_serial.py#L61)) or serial I/O fails (e.g. `OSError`, `TransactionTimeoutError`), the `finally` block records the attempt with `succeeded=False` in `self._timing` before re-raising the original exception. Original errors are not suppressed or altered.

3. **Clock Invalidation & Error Isolation**:
   - In [`ArmManualSession._loop`](file:///tmp/c15-codex-20261017/arm_manual.py#L2195-L2205), exceptions from `self._timing_clock_ns()` or `loop_started()` are trapped with `self._timing.loop_unavailable()`. The loop continues unhindered; owner lifecycle, job processing, and stops are not blocked.
   - In [`ArmManualSession._transact_raw`](file:///tmp/c15-codex-20261017/arm_manual.py#L1028-L1064), exceptions reading `started_ns` or within `transaction_finished()` invoke `transaction_unavailable()`. This increments `_measurement_errors`, records unknown latency (`None`), and ensures serial transactions proceed without failing due to clock faults.
   - [`StationTiming._valid_ns`](file:///tmp/c15-codex-20261017/station_timing.py#L28-L31) enforces exact `int` (rejecting `bool`, floats, strings, negative numbers, and $> 2^{64}-1$). Backward timestamps raise `ValueError` and break continuity without corrupting existing counters.

4. **Telemetry Allowlist & Field Projection**:
   - `TIMING_FIELDS` in [`arm_dashboard.py:60-64`](file:///tmp/c15-codex-20261017/arm_dashboard.py#L60-L64) explicitly allowlists the 12 timing metrics.
   - [`_project_fields`](file:///tmp/c15-codex-20261017/arm_dashboard.py#L133-L162) drops unexpected keys, sanitizes strings, preserves `None`, and discards non-finite numbers (`NaN`, `Inf`). No session IDs, tickets, or serial tokens can leak through the `timing` structure.

5. **Safety Invariants & Hardware Boundary**:
   - Stop verification ([`_stop`](file:///tmp/c15-codex-20261017/arm_manual.py#L1072-L1124)), mode-1 gates ([`_mode1_gate`](file:///tmp/c15-codex-20261017/arm_manual.py#L1620-L1625)), deadman switches, and lease timeouts are unchanged from C14.
   - Physical I/O remains blocked by default; startup remains disconnected with fake transport. Hardware control operations require explicit opt-in.