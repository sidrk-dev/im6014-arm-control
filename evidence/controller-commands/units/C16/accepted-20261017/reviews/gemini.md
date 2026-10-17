# Independent Adversarial Review: C16 Workbench Artifact

## Scope & Inspected Artifacts

- **Production code:** [`arm_dashboard.py`](file:///tmp/c16-codex-20261017/arm_dashboard.py#L714-L733)
- **Architecture & Station Documentation:** [`docs/architecture.md`](file:///tmp/c16-codex-20261017/docs/architecture.md#L46-L47), [`docs/station-independence.md`](file:///tmp/c16-codex-20261017/docs/station-independence.md)
- **Baseline & Milestone Automation:** [`scripts/check_milestone.py`](file:///tmp/c16-codex-20261017/scripts/check_milestone.py#L30), [`scripts/check_offline_baseline.sh`](file:///tmp/c16-codex-20261017/scripts/check_offline_baseline.sh#L81-L101), [`test_startup.py`](file:///tmp/c16-codex-20261017/test_startup.py#L112-L119)
- **Focused Verification Suite:** [`test_station_independence.py`](file:///tmp/c16-codex-20261017/test_station_independence.py)
- **Evidence Logs & Receipts:** [`cleanup-before.log`](file:///tmp/c16-codex-evidence-20261017/cleanup-before.log), [`cleanup-after.log`](file:///tmp/c16-codex-evidence-20261017/cleanup-after.log), [`milestone.log`](file:///tmp/c16-codex-evidence-20261017/milestone.log), [`final-milestone.log`](file:///tmp/c16-codex-evidence-20261017/final-milestone.log), [`baseline.log`](file:///tmp/c16-codex-evidence-20261017/baseline.log), [`final-baseline.log`](file:///tmp/c16-codex-evidence-20261017/final-baseline.log), [`guarded-arm-result.json`](file:///tmp/c16-codex-evidence-20261017/guarded-arm-result.json), [`final-milestone/receipt.json`](file:///tmp/c16-codex-evidence-20261017/final-milestone/C16-20261017T205800143340Z/receipt.json).

---

## Detailed Evaluation by Lens

### 1. Correctness & Resource Cleanup in [`arm_dashboard.py`](file:///tmp/c16-codex-20261017/arm_dashboard.py)
- **Before C16:** [`run_dashboard`](file:///tmp/c16-codex-20261017/arm_dashboard.py#L707-L732) had two distinct resource leak vulnerabilities:
  1. If `ArmDashboardServer(("127.0.0.1", port), session)` failed to construct (e.g., port bind failure, OS permission error), execution never entered the `try...finally` block; the newly created or supplied [`ArmManualSession`](file:///tmp/c16-codex-20261017/arm_manual.py) remained unclosed with its background worker thread alive.
  2. In the `finally` block, `server.server_close()` executed directly before `session.close()`. If `server.server_close()` threw an exception, `session.close()` was bypassed entirely.
- **Under C16:**
  ```python
  server = None
  try:
      server = ArmDashboardServer(("127.0.0.1", port), session)
      ...
      try:
          server.serve_forever()
      except KeyboardInterrupt:
          print("\nShutting down server...")
  finally:
      try:
          if server is not None:
              server.server_close()
      finally:
          session.close()
  ```
- **Evaluation:**
  - Constructing the server inside the outer `try` block guarantees `session.close()` runs even when server binding raises.
  - The nested `try...finally` inside the cleanup block guarantees that any exception in `server.server_close()` does not skip `session.close()`.
  - [`ArmManualSession.close()`](file:///tmp/c16-codex-20261017/arm_manual.py) is idempotent and safe to call on already-closed sessions.
  - Verified across [`cleanup-before.log`](file:///tmp/c16-codex-evidence-20261017/cleanup-before.log) (2 failures reproduced) and [`cleanup-after.log`](file:///tmp/c16-codex-evidence-20261017/cleanup-after.log) (both pass).

### 2. Single Ownership & Station Independence
- **Owner Preservation:** [`ArmDashboardServer`](file:///tmp/c16-codex-20261017/arm_dashboard.py#L655-L671) solely receives `session` as an ingress boundary and does not own session creation, lifecycle, or the underlying bus.
- **Second Owner Exclusion:** [`test_station_independence.py`](file:///tmp/c16-codex-20261017/test_station_independence.py) patches `ArmManualSession` with `side_effect=AssertionError('second owner')` across loop failures, bind failures, and HTTP restarts ([`test_http_restart_preserves_one_owner_and_cannot_renew_stale_input`](file:///tmp/c16-codex-20261017/test_station_independence.py#L160-L185)). All packet transmissions are verified to execute exclusively on the single initial owner thread `self.arm.thread`.
- **Observer Isolation:** [`test_new_http_observer_cannot_inherit_control_or_rearm_stopped_owner`](file:///tmp/c16-codex-20261017/test_station_independence.py#L187-L210) proves that creating a new HTTP observer session (`POST /api/session`) does not disturb an existing owner, cannot seize motion authority, and cannot submit jog commands without completing the full prerequisite calibration and arm sequence.

### 3. Autoarm Prevention & Input Expiry
- **No Autoarm:** Neither restarting the HTTP server, querying `/api/state`, nor opening an observer session can transition `armed` from `False` to `True`.
- **Lease Invalidation:** When the HTTP ingress terminates or fails to refresh within `INGRESS_LEASE_S` (0.2s), the session trips input loss, issues mode-0 Stop packets across configured joints, and latches `stop_confirmed`. Old tickets are rejected with `ValueError`.

### 4. Verification & Evidence Honesty
- **Initial Fixture Defect:** The initial fixture draft in [`observer-pending-neutral-before.py`](file:///tmp/c16-codex-evidence-20261017/observer-pending-neutral-before.py) failed ([`milestone.log`](file:///tmp/c16-codex-evidence-20261017/milestone.log), [`baseline.log`](file:///tmp/c16-codex-evidence-20261017/baseline.log)) because it invoked `self.ready()` and immediately waited for `not self.arm.snapshot()['armed']`. Because no jog motion packets had been submitted, `intent` was `None`, so input ticket expiry was inactive (the arm legitimately held in armed mode-0 awaiting initial input).
- **Test-Only Correction:** The revised fixture in [`test_station_independence.py`](file:///tmp/c16-codex-20261017/test_station_independence.py#L187-L192) initiates an explicit operator Stop before observer creation, verifying that observer arrival cannot rearm a stopped station. Production authority logic in [`arm_manual.py`](file:///tmp/c16-codex-20261017/arm_manual.py) remained completely untouched (byte-identical).
- **Physical Safety:** [`test_station_independence.py`](file:///tmp/c16-codex-20261017/test_station_independence.py#L18-L35) mocks `serial.Serial`, `enumerate_ports`, and `open_managed_port` with `AssertionError`, asserting zero calls in `tearDown`. [`guarded-arm-result.json`](file:///tmp/c16-codex-evidence-20261017/guarded-arm-result.json) confirms 79 regression tests passed with `physicalSerialConstructorCalls: 0`.

### 5. Source-Copy & Fingerprint Completeness
- All 39 files in `files` in [`scripts/check_offline_baseline.sh`](file:///tmp/c16-codex-20261017/scripts/check_offline_baseline.sh#L81) match the 39 files tracked by `_fingerprint` in [`test_startup.py`](file:///tmp/c16-codex-20261017/test_startup.py#L112-L119). Both properly include `arm_manual.py`, `arm_dashboard.py`, and `test_station_independence.py`.
- [`scripts/check_milestone.py`](file:///tmp/c16-codex-20261017/scripts/check_milestone.py#L30) registers `'C16': 'test_station_independence'`.
- Isolated scratch run in [`final-baseline.log`](file:///tmp/c16-codex-evidence-20261017/final-baseline.log) passed all 10 verification groups (exit code 0 across packet, protocol, serial-deadline, milestone, Qt dashboard, startup verifier, real startup, and demo startup checks).

### 6. Complexity Budget & Code Quality
- Production changes are strictly confined to 12 lines in [`arm_dashboard.py`](file:///tmp/c16-codex-20261017/arm_dashboard.py#L717-L732). Total file length remains 748 lines (well below the 1,000-line review ceiling).
- No extraneous classes, background threads, or architectural bloat were introduced.

---

## Findings

**No findings.**

The C16 implementation addresses the verified cleanup and ownership defects directly, maintains zero physical serial calls, preserves single-ownership and disarm invariants, and adheres to repository boundary and fingerprint requirements.