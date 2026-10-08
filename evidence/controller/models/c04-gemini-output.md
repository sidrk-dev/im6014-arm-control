Implemented milestone **C04** in [joint_controller.py](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py) and authored unit tests in [test_controller.py](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/test_controller.py).

### Summary of Changes

1. **Attributed Per-Joint Fault Records & Fixed Communication Failures** ([joint_controller.py](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py)):
   - Added [`CommunicationFailure`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L44-L67) enum defining fixed codes (`TIMEOUT`, `INVALID_FEEDBACK`, `DRIVER_FAILURE`) with string matching and aliases [`FailureCode`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L66) and [`CommunicationErrorCode`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L67). Arbitrary exception text and unknown codes are rejected.
   - Added immutable [`JointFault`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L103-L176) dataclass storing attributed `name`, `motor_id`, `errors` (uint32 mask, preserving unknown bits), `warnings` (uint3 mask), `timeout` (exact bool), `communication_error`, and `stale` (exact bool).
   - Enforced uint32 error validation (`0 <= errors <= 0xFFFFFFFF`), uint3 warning validation (`0 <= warnings <= 7`), and exact boolean timeouts in both [`MotorHealth`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L179-L198) and [`JointFault`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L103-L176).
   - Separated live observations ([`JointStatus`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L216-L265)) from latched fault causes: subsequent healthy feedback refreshes observation data without clearing latched fault attribution.
   - Bounded fault storage in [`JointCore`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L353-L725) to at most one record per active joint. Multiple fault events on a joint are merged via bounded bitwise-OR/union (`errors | new_errors`, `warnings | new_warnings`, `timeout or new_timeout`, union of communication errors) without growing history.
   - Added [`JointCore.communication_failed(motor_id, code)`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L591-L622) (with aliases `communication_failure` and `record_communication_failure`): known motor IDs attribute failure and transition to `FAULT`, while unknown IDs or invalid codes reject with `UNKNOWN_JOINT` or `INVALID_ARGUMENT` leaving core state unchanged.
   - Updated [`JointCore.tick()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L666-L690): staleness detection checks each joint independently against `feedback_max_age_s`, identifying and attributing faults specifically to stale joints.
   - Updated [`Snapshot`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L267-L351) to expose immutable tuples `faults`, `affected_names`, and `affected_joints`.
   - Preserved latched causes across `stop()`, `disconnect()`, and reconnects; causes are cleared exclusively upon successful [`JointCore.checked()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L634-L650) following explicit `begin_check(acknowledge_fault=True)` with fresh healthy Stop feedback on all active joints.
   - Ensured fault occurrences disarm only the affected [`JointCore`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/joint_controller.py#L353-L725) instance; other instances stay independent.

2. **Test Suite** ([test_controller.py](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/test_controller.py)):
   - Preserved existing C02 [`ControllerStateTests`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/test_controller.py#L224-L467) and C03 [`FeedbackAgeTests`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/test_controller.py#L76-L221).
   - Authored new [`JointFaultTests`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c04-gemini/test_controller.py#L469-L860) covering:
     - `test_stale_j2_attributes_only_stale_joint_with_refreshed_j1`: Injected clock where J1 is refreshed and J2 ages out; `tick()` attributes staleness solely to J2 while J1 remains healthy and fresh.
     - `test_unknown_bits_and_32bit_error_mask_retained`: Verifies retention of unknown error bits (e.g., bit 31 `0x80000000`), uint3 warning limits, boolean timeouts, and input validation bounds.
     - `test_healthy_after_fault_preserves_latched_attribution`: Verifies that healthy feedback received after a joint fault updates observation fields without erasing latched cause attribution or clearing `FAULT` state.
     - `test_multiple_faults_on_joint_preserves_prior_flags_with_bounded_storage`: Verifies bitwise OR and union preservation across multiple events with bounded storage.
     - `test_communication_failures_with_fixed_enum_codes`: Tests fixed enum communication failure codes attributing failure to known joints.
     - `test_unknown_ids_and_invalid_codes_reject_unchanged`: Rejection of unknown motor IDs, invalid ID types, and arbitrary exception strings without modifying state or fault records.
     - `test_failed_recovery_retains_causes`: Checks that unacknowledged checks, premature checks, non-stop mode feedback, and stale feedback fail recovery and retain causes.
     - `test_successful_recovery_clears_fault_causes`: Verifies cause retention through `stop()`, `disconnect()`, and reconnect, with full clearance upon acknowledged successful check.
     - `test_independent_second_core_unchanged_by_fault`: Verifies that a fault on one core disarms that build while an independent second core remains in `ARMED` without interference.
     - `test_immutability_of_fault_records_and_snapshot_collections`: Verifies frozen records and immutable tuple snapshot collections.

### Verification

Per worker contract and parent project override: edits were completed and reviewed read-only. Tests were authored but not run; no Git writes, package operations, or physical I/O were performed.