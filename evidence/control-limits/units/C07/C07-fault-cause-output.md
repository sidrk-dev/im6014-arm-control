# C07 fault-cause authoring report

Updated only `joint_contract.py`, `joint_controller.py`, `im6014_serial.py`, `test_controller_expiry.py`, and `docs/joint-api.md` in the assigned fault-cause workbench.

- Added `CommunicationFailure.CLOCK_FAILURE` and `CommunicationFailure.PREWRITE_DEADLINE`, plus `WriteResult.PREWRITE_DEADLINE` for a claimed dispatch canceled by the final deadline check before `bus.write`.
- Parameterized `_record_dispatch_failure` with a typed `CommunicationFailure`. Complete writes followed by an invalid or regressing core clock retain the complete attempt and dispatched record, then latch `CLOCK_FAILURE` and withdraw motion. A pre-write deadline outcome resolves the claim, records no complete dispatch, and latches the matching cause. Raised writes, invalid write returns, and partial writes still map to `DRIVER_FAILURE`.
- Updated focused regressions to assert exact causes and outcomes, preserve acknowledgement and invalid-clock boundaries, and check that the zero-write path does not create a dispatched record. Documentation distinguishes the known pre-write cancellation from partial/unknown writes and makes no claim about physical protection.
- Updated the fault-record documentation to describe the first-occurrence union as bounded by the finite communication-failure enum.

Only read-only source inspection was performed. No tests, scripts, Git operations, network calls, or hardware operations were run; parent acceptance checks remain required.
