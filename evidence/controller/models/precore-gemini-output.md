# Independent Code Review: B11–B14 and C01–C03

**Workspace:** `/tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review`  
**Inspected Scope:** Pure joint algebra ([`joint_mapping.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_mapping.py)), strict numeric input gating ([`im6014_codec.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_codec.py)), passive port inspection & cooperating device locking ([`im6014_port.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_port.py)), bounded diagnostic capture & exchange ([`im6014_serial.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_serial.py)), CLI runner ([`spin_5rpm.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/spin_5rpm.py)), GUI worker ([`motor_session.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/motor_session.py)), transport-free joint core & per-joint freshness ([`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py)), and their test suites.

---

## Verdict Summary

- **Verdict:** **CONDITIONAL PASS** (0 blocking regressions against agreed contracts; 1 actionable defect regarding timing-induced exception masking in diagnostic capture; notes and test gaps identified below).
- **Scope boundary:** C04–C06 (error status, target bounds, sequence checks) are confirmed out-of-scope for this snapshot and not demanded.
- **Contract compliance:**
  - Passive inspection reports unknown identity as unverified while rejecting known mismatches.
  - Device lease closure requires explicit confirmation; missing or `True` `is_open` retains lease ownership.
  - Physical I/O was not performed; labels (`source='physical'`) record caller provenance rather than physical proof.

---

## Actionable Finding: Capture Timing Masking Exchange Failures

### Finding 1: Clock exception during capture finalization masks exchange failure and drops diagnostic record
- **File & Symbol:** [`im6014_serial.py:capture_exchange`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_serial.py#L219-L257) (lines 242–257)
- **Mechanism & Reachability:**
  In `capture_exchange`, when `exchange_fn` fails (e.g. with [`TransactionTimeoutError`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_serial.py#L45), [`InvalidFeedbackError`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_serial.py#L55), or a serial driver exception), the `except Exception as error:` block records the outcome and executes `raise`. In the following `finally:` block:
  ```python
  finally:
      ended = capture._now()
      if outcome not in _OUTCOMES:
          outcome = 'DRIVER_FAILURE'
      capture._append(_PacketRecord(...))
  ```
  `capture._now()` calls `self._clock()`, asserting that the returned timestamp is a finite real number in `[0, 1e12]`. If an injected custom clock raises an exception or returns an invalid value (`NaN`, `inf`, negative, or non-numeric) on this second invocation, `capture._now()` raises `ValueError`.
  In Python, an exception raised inside `finally:` **unconditionally suppresses and replaces** the in-flight primary exception being re-raised from `try/except`.
- **Concrete Reproduction:**
  ```python
  import im6014_serial as transport

  # Clock succeeds on entry (started), but fails on exit (ended)
  clock_vals = iter([1.0, float("nan")])
  capture = transport.PacketCapture(clock=lambda: next(clock_vals))

  def failing_exchange(bus, packet, motor_id):
      raise transport.TransactionTimeoutError("No validated motor feedback within deadline")

  # Caller receives ValueError instead of TransactionTimeoutError
  try:
      transport.capture_exchange(
          capture, failing_exchange, None, b"R" * 20, 0,
          source="fixture", identity="test", connection=1, port="", baud=6000000,
      )
  except Exception as e:
      assert type(e) is ValueError  # TransactionTimeoutError was masked
      assert "Capture clock must return a finite monotonic number" in str(e)

  # Furthermore, the diagnostic record was lost because _append was never reached:
  assert len(capture.snapshot()["records"]) == 0
  ```
- **Operational Impact:**
  Callers relying on communication fault attribution—such as [`motor_session.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/motor_session.py#L220-L225) inspecting `'CRC' in str(error)` or `'deadline' in str(error)`—receive `ValueError` instead of the communication exception. As a consequence, timeout and CRC counters are bypassed, and no entry is appended to `capture._records`.
  *(Note: When the default `time.monotonic` system clock is used, the system clock does not raise, so production execution is unaffected. However, the injected clock interface explicitly permitted by `PacketCapture(clock=...)` makes this reachable in test fixtures and custom runners).*
- **Smallest Correction:**
  In [`im6014_serial.py:capture_exchange`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_serial.py#L249-L257), guard `capture._now()` in `finally:` so secondary clock evaluation failures do not discard the primary exception or prevent appending the record:
  ```python
  finally:
      try:
          ended = capture._now()
      except Exception:
          ended = started
      if outcome not in _OUTCOMES:
          outcome = 'DRIVER_FAILURE'
      capture._append(_PacketRecord(
          source, identity, connection, port, baud, motor_id, started, ended,
          request.hex(), reply.hex(), decoded, outcome, rejected, crc_rejected, tail.hex(),
      ))
  ```
- **Severity:** **Medium** (Actionable timing/masking defect under clock failure; non-blocking for default production runtime).

---

## Milestone Verification & API Regressions

### B11: External Joint Mapping ([`joint_mapping.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_mapping.py))
- **Correctness:** External reduction $N > 0$, sign $s \in \{-1, 1\}$, offset $q_0$, velocity scaling ($dq \leftrightarrow rpm$), torque scaling ($\tau_{out} = s \tau_{joint} / N$), and quadratic gain scaling ($kp/N^2, kd/N^2$) accurately mirror the design algebra in [`docs/controller-design.md`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/docs/controller-design.md#L23-L33).
- **Invariants:** Dataclasses are frozen; finite number checks reject booleans, NaN, infinities, and overflow; clipping is never performed, delegating physical/codec boundaries to the encoder.
- **Regressions:** None. Completely isolated pure-math module with zero transport or hardware imports.

### B12: Numerical Input Rejection ([`im6014_codec.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_codec.py), [`im6014_serial.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_serial.py))
- **Correctness:** [`command()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_codec.py#L48) and [`feedback()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_codec.py#L78) strictly validate integer IDs in `[0, 14]` and reject `bool` as integer IDs or modes. Control inputs are checked for finiteness before protocol bounds checks. `exchange()` validates deadline positivity and finiteness before any serial bus manipulation.
- **Regressions:** None. Protocol golden bytes and error limits remain identical.

### B13: Explicit Port Configuration ([`im6014_port.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_port.py))
- **Correctness:** [`PortConfig`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_port.py#L22) enforces explicit paths, 4/6 Mbps bauds, and coupled `(expected_serial, expected_channel)` settings. [`inspect_port()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_port.py#L399) reads passive metadata without opening or sending bytes; non-FTDI USB VIDs are actively rejected even when serial numbers match. Alternate routes (`None/None`) and incomplete metadata explicitly report identity as unverified.
- **Regressions:** None.

### B14: Diagnostic Capture ([`im6014_serial.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_serial.py))
- **Correctness:** Ring capacity is capped (default 256). Packet records contain bounded prefixes (20B request, 26B reply, 25B tail), allowlisted decoded fields, and fixed outcome strings. Raw exception strings and environment data are excluded from export snapshots.
- **Regressions:** CLI [`spin_5rpm.py:run()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/spin_5rpm.py#L37) keeps capture opt-in (`capture=None` retains uncaptured behavior).

### C01: Cooperating Bus Owner ([`im6014_port.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_port.py))
- **Correctness:** Bus lease is acquired before serial factory invocation, keyed by character-device `st_rdev` in a private `0700` user directory under `/tmp`. Symlink aliases correctly collide on the same lockfile inode. Lockfiles are never unlinked.
- **Closure Guarantee:** In [`ManagedPort.close()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_port.py#L82), failure to confirm `bus.is_open is False` retains the lock file descriptor, preventing lease drops when mock or driver teardown is unconfirmed.
- **Regressions:** Factory injection remains supported at both bench sites.

### C02: Controller State Transitions ([`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py))
- **Correctness:** Full state machine implemented: `DISCONNECTED` $\rightarrow$ `CONNECTING` $\rightarrow$ `CONNECTED` $\rightarrow$ `CHECKING` $\rightarrow$ `READY` $\rightarrow$ `ARMED`. Invalid transitions reject cleanly with typed [`DecisionCode`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py#L29).
- **Parent Latched Fault Correction:** Calling [`stop()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py#L382) or [`disconnect()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py#L391) preserves `fault_latched`. Subsequent check attempts reject with `FAULT_ACK_REQUIRED` until [`begin_check(acknowledge_fault=True)`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py#L300) is called and confirmed by a successful [`checked()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py#L338) cycle. Healthy packets alone cannot clear the latch.
- **Regressions:** None. Core is strictly transport-free.

### C03: Per-Joint Feedback Age ([`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py))
- **Correctness:** Each joint observation is stamped with monotonic time from the core's clock. [`checked()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py#L338) and [`arm()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py#L354) verify that all active joints have feedback ages within `feedback_max_age_s` (default 0.5s). [`tick()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/joint_controller.py#L369) transitions active states (`READY`, `ARMED`, `MOVING`) to `FAULT` if observations age out. Unknown IDs and invalid records do not update timestamps. Timestamp 0.0 is explicitly accepted.
- **Regressions:** None.

---

## Test Suite Assessment & Challenged Weak Tests

1. **Diagnostic Capture Edge Coverage ([`test_diagnostic_capture.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/test_diagnostic_capture.py)):**
   - **Gaps:** `test_capture_metadata_is_validated_before_exchange` only tests `source='unknown'`. It never verifies `source='physical'` with `port=''` rejection, capacity outside `1..256`, unsupported bauds (e.g. 115200 or 5000000), or `connection < 0`.
   - **Timing Masking:** No test validates behavior when `capture._now()` fails in `finally:` during an exchange failure.
2. **Device Locking & Character Device Rejection ([`test_bus_owner.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/test_bus_owner.py)):**
   - **Gaps:** Does not test passing a regular file or directory to [`open_managed_port`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/im6014_port.py#L245) to exercise `PortOwnershipError('Configured port is not a character device')`.
   - Does not exercise `bus.fileno()` raising an exception during verification.
3. **Controller Input and State Edge Cases ([`test_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/test_controller.py)):**
   - **Gaps:** [`test_health_and_source_records_reject_invalid_values`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/precore-gemini-review/test_controller.py#L281) only tests `SourceIdentity("other", "x", 1)`. Empty string identity (`identity=""`), negative connections, and boolean connection flags are untested.
   - `test_joint_set_is_nonempty_unique_and_owned` tests empty joint sets and duplicate IDs, but does not verify rejection when `mapping` is not a `JointMap`.
   - `Snapshot` feedback age formatting is only tested when either all joints are unreceived (`None, None`) or all joints are received; the heterogeneous case where joint 0 is received while joint 1 remains unreceived is untested.
   - `test_checked_rejects_feedback_older_than_limit` confirms rejection on stale observations in `CHECKING`, but does not test that subsequent receipt of fresh Stop frames permits recovery to `READY`.

---

## Conclusion

The frozen checkout at B11–B14 and C01–C03 adheres to the agreed architectural boundaries and safety contracts. No breaking API regressions or bypasses of physical lease ownership exist. Finding 1 represents an actionable defect in diagnostic exception handling under custom clock anomalies that can be resolved with a minimal local try/except block in `capture_exchange`. All other inspected mechanisms pass review.