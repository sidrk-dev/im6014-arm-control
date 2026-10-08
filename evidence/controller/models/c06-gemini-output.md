### Summary of Changes

Completed implementation of milestone **C06** in the dedicated worktree `/tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini`.

#### 1. [`joint_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/joint_controller.py)
- **`DecisionCode` Enum**:
  - Added [`DecisionCode.DUPLICATE_OR_OLDER = "duplicate_or_older"`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/joint_controller.py#L43) for fixed rejection of duplicate or older sequence submissions.
- **`Target` Dataclass**:
  - Migrated to frozen [`Target(sequence: int, joints: tuple[JointTarget, ...])`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/joint_controller.py#L210-L224) with no legacy constructors or aliases.
  - Enforced exact integer sequence excluding booleans, bounded strictly in $[0, 2^{64}-1]$ inclusive (`type(self.sequence) is int and 0 <= self.sequence <= (1 << 64) - 1`), raising `ValueError` on bad types or out-of-range values.
  - Preserved complete non-empty tuple of `JointTarget` validation.
- **`Snapshot` Watermark Exposure**:
  - Added [`last_sequence: int = -1`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/joint_controller.py#L236) to `Snapshot`, validating that `last_sequence` is an integer in $[-1, 2^{64}-1]$ in [`__post_init__`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/joint_controller.py#L279).
- **`JointCore` Monotonic Watermark Lifecycle**:
  - Initialized `self._last_sequence = -1` in [`JointCore.__init__`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/joint_controller.py#L331).
  - Exposed via [`Snapshot.last_sequence`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/joint_controller.py#L630).
  - In [`JointCore.submit_target`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/joint_controller.py#L543-L581):
    - Reused existing C05 validation: verifies state (`ARMED` or `MOVING`), observation completeness, freshness, health, target structure/vector length, and mapped outputs through [`im6014_codec.command`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/im6014_codec.py#L48-L75) bounds first.
    - Any vector or mapping invalidity rejects with `DecisionCode.INVALID_ARGUMENT`, preserving both `_pending_target` and `_last_sequence`.
    - If `target.sequence <= self._last_sequence`, rejects with `DecisionCode.DUPLICATE_OR_OLDER`, preserving both `_pending_target` and `_last_sequence`.
    - Strictly newer sequences (`target.sequence > self._last_sequence`) advance `_last_sequence` and replace `_pending_target` atomically only on acceptance.
  - Consumption, `stop()`, all fault paths (`receive` bad health, `communication_failed`, `tick` stale check, consumption stale check), `disconnect()`, reconnect, and acknowledged check recovery retain `_last_sequence`.

#### 2. [`test_controller.py`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/test_controller.py)
- **Migrated Existing `Target` Callers**:
  - Updated every existing `Target` construction and test across [`ControllerStateTests`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/test_controller.py#L278) and [`LatestTargetTests`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/test_controller.py#L899-L1335) to use `Target(sequence, joints)` with monotonic sequences.
  - Preserved public reachability of logical `State.MOVING` via consumption and full state transition coverage.
  - Verified preservation of `Snapshot.last_sequence` across fault paths, Stop, and disconnect in existing tests.
- **Authored [`CommandSequenceTests`](file:///tmp/pstack-b11-c06-20261007-1mwdi1cr/c06-gemini/test_controller.py#L1347-L1645)**:
  - `test_actual_order_1_5_5_2_6_ends_at_6`: Submits sequences $1, 5, 5, 2, 6$; verifies acceptance of $1, 5, 6$; verifies rejections of duplicate $5$ and older $2$ with `DecisionCode.DUPLICATE_OR_OLDER` and zero fake packet writes; verifies consumption consumes target $6$ and sets `MOVING`.
  - `test_boundaries_0_and_uint64max`: Tests boundary $0$ accepted from initial $-1$, duplicate $0$ rejected; tests boundary $2^{64}-1$ accepted, duplicate rejected, and smaller value rejected as older.
  - `test_sequence_type_and_range_rejections`: Tests `ValueError` for `bool` (`True`, `False`), negative integers, floats (`0.0`, `1.5`, NaN, $\pm\infty$), strings, too-large numbers ($2^{64}$, $2^{64}+1$), `None`, and arbitrary objects; tests frozen immutability of `target.sequence`.
  - `test_invalid_higher_vector_or_mapping_preserves_pending_and_watermark_and_never_blocks_smaller`: Submits target with seq $5$; submits invalid higher vector length (seq $10$) and mapped torque outside codec bounds (seq $10$); verifies rejection with `INVALID_ARGUMENT` and preservation of watermark $5$ and pending target; submits valid seq $7$ ($7 < 10$) and verifies acceptance.
  - `test_watermark_survives_consumption_stop_fault_disconnect_reconnect_and_recovery`: Verifies watermark survives consumption, Stop, fault, disconnect, reconnect, and acknowledged check recovery, continuing to reject duplicate/older and accepting strictly newer targets.
  - `test_rejected_state_and_stale_submissions_preserve_watermark_and_pending`: Verifies rejected states and stale feedback submissions preserve watermark and pending target.
  - `test_new_core_starts_negative_one_and_independent_watermarks`: Verifies new core starts with watermark $-1$ and separate core instances maintain independent watermarks.

### Tests Authored (Not Run)
Per worker contract instructions, tests were authored but not run (no command execution, Git writes, package mutations, or physical I/O). Ready for parent acceptance execution via `scripts/check_milestone.py C06`.