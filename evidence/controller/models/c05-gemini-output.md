### Summary of Changes

Completed implementation of milestone **C05** in `/tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini`.

#### 1. [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/joint_controller.py)
- **`Target` Dataclass**: Introduced frozen [`Target`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/joint_controller.py#L209-L219) with `joints: tuple[JointTarget, ...]`, validating tuple type, non-emptiness, and `JointTarget` elements using existing [`JointTarget`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/joint_mapping.py#L110-L123) numeric bounds.
- **`Snapshot` pending target**: Added [`pending_target: Target | None = None`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/joint_controller.py#L229) with type validation in [`__post_init__`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/joint_controller.py#L267-L268).
- **Slot Management & Validation in [`JointCore.submit_target`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/joint_controller.py#L540-L576)**:
  - Verifies state is `ARMED` or `MOVING`, observations complete, feedback fresh, and joints healthy.
  - Validates `Target` type, tuple structure, and exact matching vector length against configured specs.
  - Validates each joint element using pure [`JointMap.to_output`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/joint_mapping.py#L64-L83) and [`im6014_codec.command`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/im6014_codec.py#L48-L75) (mode 1 FOC) without duplicating codec bounds or clipping.
  - Invalid target submissions leave the prior pending target untouched.
- **Consumption via [`JointCore.consume_target`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/joint_controller.py#L578-L612)**:
  - Allowed only in `ARMED` or `MOVING`.
  - Stale feedback discovered during consumption transitions the core to `FAULT`, latches stale joint attribution, clears pending, and returns `None`.
  - When fresh and healthy with a pending target present, clears the pending slot, sets logical state to `State.MOVING`, and returns the target once.
- **Lifecycle & Fault Invalidation**:
  - `stop()`, `disconnect()`, `begin_check()`, unhealthy `receive()`, `communication_failed()`, and `tick()` stale detection clear `_pending_target`.
  - Preserved C02–C04 fault attribution and acknowledged check recovery.

#### 2. [`test_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/test_controller.py)
- **Extended [`FakeStation`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/test_controller.py#L24-L88)**:
  - Added [`submit(target)`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/test_controller.py#L56-L57) forwarding to `core.submit_target(target)`.
  - Added [`consume()`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/test_controller.py#L59-L76) consuming pending targets and writing actual codec FOC packets with mapped outputs.
- **Transition Matrix**:
  - Reached logical `State.MOVING` via public target submission and consumption in [`_core_at`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/test_controller.py#L277-L285).
  - Added `State.MOVING` into [`test_invalid_transition_matrix_preserves_snapshot`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/test_controller.py#L475).
- **Authored [`LatestTargetTests`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c05-gemini/test_controller.py#L867-L1280)**:
  - `test_ten_thousand_accepted_updates_leave_one_newest_target_and_one_consume`: 10,000 accepted updates leave one newest pending target, one consume enters `MOVING`, and second consume returns `None`.
  - `test_immutable_owned_vector`: Verifies immutability of `Target` and `Snapshot.pending_target`, and type rejections for non-tuple, empty tuple, or non-`JointTarget` inputs.
  - `test_malformed_incomplete_and_invalid_mapped_outputs_preserve_prior_target`: Rejections for non-`Target`, wrong vector length, and codec overflow preserve prior pending target.
  - `test_mapped_torque_outside_codec_bounds_rejects`: Torques outside ±3.5 Nm (including non-unity gear reductions) reject and preserve prior target.
  - `test_successful_fake_adapter_uses_actual_codec_foc_packets_with_independent_expected_mappings`: Verifies FOC packet bytes written by fake adapter match independently calculated mapping numbers.
  - `test_no_writes_on_rejection_stale_fault_stop_disconnect`: Verifies zero FOC packets written on rejection, stale, fault, Stop, or disconnect.
  - `test_stale_feedback_discovered_on_consume_clears_pending_and_latches_fault`: Verifies stale feedback discovered on consume latches fault, clears pending, and returns `None`.
  - `test_only_armed_and_moving_may_submit_and_consume`: Verifies submission and consumption restrictions across all lifecycle states.

### Status
- **Actual files changed**: `joint_controller.py`, `test_controller.py`.
- **Checks**: Tests authored but not run, per worker contract instructions. Ready for parent execution and acceptance checks.