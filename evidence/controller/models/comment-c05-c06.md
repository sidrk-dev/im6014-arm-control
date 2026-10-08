# Comment review: C05/C06

Report only. Reviewed the changes from `acd5605` through current HEAD plus current working edits in these six files: `joint_controller.py`, `test_controller.py`, `test_controller_targets.py`, `scripts/check_milestone.py`, `scripts/check_offline_baseline.sh`, and `test_startup.py`. The untracked `test_controller_targets.py` was read directly in full; it is absent from the tracked Git diff.

Proposed touched file: `test_controller_targets.py` only. Delete 26 comment lines. Each is narration already expressed by the adjacent setup, operation, assertion, or test name; none qualifies for a keep exception.

## Exact proposed deletions

Paths and line numbers below refer to the reviewed working copy in `/home/sid/Documents/code/im6014Workbench`.

| File:line | Delete this complete line | Evidence |
| --- | --- | --- |
| test_controller_targets.py:482 | `        # 1: accepted` | Lines 483–487 submit sequence 1 and assert acceptance, sequence, and pending target. |
| test_controller_targets.py:489 | `        # 5: accepted` | Lines 490–494 submit sequence 5 and assert acceptance, sequence, and pending target. |
| test_controller_targets.py:496 | `        # 5: duplicate rejected, preserves pending and watermark, 0 writes` | Lines 497–504 assert every narrated result. |
| test_controller_targets.py:506 | `        # 2: older rejected, preserves pending and watermark, 0 writes` | Lines 507–513 assert every narrated result. |
| test_controller_targets.py:515 | `        # 6: accepted` | Lines 516–520 submit sequence 6 and assert acceptance, sequence, and pending target. |
| test_controller_targets.py:522 | `        # Public consumption consumes the latest accepted target (6) and reaches MOVING` | Lines 524–529 assert consumption of `t6`, MOVING, cleared pending target, sequence 6, and two writes. |
| test_controller_targets.py:535 | `        # Boundary 0 is accepted initially (-1 watermark)` | Line 533 asserts the initial -1; lines 536–539 submit and assert sequence 0. |
| test_controller_targets.py:541 | `        # Duplicate 0 is rejected` | Lines 542–546 assert duplicate rejection and preserved state. |
| test_controller_targets.py:548 | `        # Boundary uint64 max is accepted` | Lines 549–553 name the maximum and assert acceptance. |
| test_controller_targets.py:555 | `        # Duplicate uint64 max is rejected` | Lines 556–559 assert duplicate rejection at the maximum. |
| test_controller_targets.py:561 | `        # Smaller than uint64 max is rejected as older` | Lines 562–566 construct maximum minus 1 and assert `DUPLICATE_OR_OLDER`. |
| test_controller_targets.py:569 | `        # bool, negative, float, string, too-large reject with ValueError` | The literal invalid inputs at 570–587 and `assertRaises(ValueError)` at 589 express the cases. |
| test_controller_targets.py:603 | `        # Invalid higher vector (wrong length) with seq=10` | Line 604 visibly creates sequence 10 with one joint; lines 606–609 assert rejection and preserved state. |
| test_controller_targets.py:611 | `        # Invalid higher mapping (torque out of codec bounds) with seq=10` | Lines 612–617 construct sequence 10 with torque 10 and assert `INVALID_ARGUMENT` and preserved state; this is scenario narration, not a non-obvious protocol constraint. |
| test_controller_targets.py:624 | `        # Valid smaller sequence (7 < 10, but 7 > 5) is accepted!` | Lines 625–629 submit sequence 7 and assert acceptance and state. The test name already states the rejected larger input cannot block a smaller valid sequence. |
| test_controller_targets.py:643 | `        # Survives consumption` | Lines 644–648 consume the target and assert sequence 10 remains. |
| test_controller_targets.py:650 | `        # Survives Stop` | Lines 651–653 stop and assert sequence 10 remains. |
| test_controller_targets.py:655 | `        # Recover to ARMED` | Lines 656–658 check, arm, and assert sequence 10 remains. |
| test_controller_targets.py:660 | `        # Survives fault (unhealthy packet)` | Lines 661–664 construct unhealthy status, receive it, assert FAULT, and assert sequence 10 remains. |
| test_controller_targets.py:666 | `        # Survives Stop in fault` | Lines 667–669 stop, assert FAULT, and assert sequence 10 remains. |
| test_controller_targets.py:671 | `        # Survives disconnect` | Lines 672–674 disconnect and assert DISCONNECTED and sequence 10. |
| test_controller_targets.py:676 | `        # Survives connect` | Lines 677–679 connect and assert CONNECTED and sequence 10. |
| test_controller_targets.py:681 | `        # Survives successful acknowledged recovery check and re-arm` | Lines 682–688 visibly acknowledge the fault, check, arm, and assert READY/ARMED and sequence 10. |
| test_controller_targets.py:690 | `        # Enforces sequence order: 10 duplicate rejects, 9 older rejects, 11 accepts` | Lines 691–706 directly assert all three cases. |
| test_controller_targets.py:715 | `        # Stale submission: clock advances beyond feedback_max_age_s` | Lines 716–722 advance the clock and assert `STALE_FEEDBACK` and preserved sequence/pending target. |
| test_controller_targets.py:724 | `        # Rejected states test` | The `rejected_states` tuple and loop at 725–743 already state and assert the scenario. |

## MUST KILL flags

None. The changed comments contain no workaround justification, suppression, hidden constraint, or own-code surprise requiring a symbol rename, extraction, type change, or rearchitecture.

## Skips

- `joint_controller.py`: unchanged module/class docstrings are outside the change scope; the module docstring also states the public ownership and immutable snapshot contract. No newly introduced comments were found.
- `test_controller.py`: the change already removes the `FakeStation` docstring; no remaining changed comments require deletion.
- `scripts/check_milestone.py`, `test_startup.py`: no introduced or modified comments. The startup module docstring is unchanged and outside the change scope.
- `scripts/check_offline_baseline.sh`: no introduced or modified comments; its unchanged shebang is executable interpreter syntax.
- Docs and evidence artifacts were excluded as requested.

Only read-only Git/source inspection was performed. No tests, packages, hardware I/O, Git mutations, or checkout edits were performed. The reported 41 passing controller checks came from the parent and were not rerun by this reviewer.
