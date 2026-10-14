# Comment audit for C07 through pending C10

Report only. No inspected source files were edited. No tests, Git mutations, subagents, or hardware operations were performed.

The audit follows `comment-audit-contract.json` and the installed `agents/comment-sicko.md` role. Scope is the changed application Python and test files in `.worktrees/controller-c07-c10` against `4d41d5b`, including the untracked `test_controller_velocity.py`. All comments and docstrings in those files were inspected, including existing text outside changed hunks. Python token and AST inspection found three comments and four docstring blocks. A suppression search found no lint, type, or warning suppressions.

Recommended deletion count: **4 blocks, 4 source lines**. Applied deletion count: **0**. Retained public contract docstrings: **3 blocks**. `MUST KILL` flags: **0**.

## Recommended deletions

| File and line | Text | Reason |
| --- | --- | --- |
| `test_controller.py:24` | `# Wide values keep legacy tests focused on codec rejection; they are not arm limits.` | Internal test-fixture narration. `WIDE_LIMITS = PositionLimits(-1e12, 1e12)` already states the wide interval, and the test file contains lifecycle and health checks as well as validation checks. No external constraint or public API contract requires the comment. |
| `test_controller_ownership.py:21` | Same comment | Internal fixture narration copied into a new ownership test module. Its tests primarily establish grant identity, generation, and revocation. The nearby `WIDE_LIMITS` name and value expose the interval directly. No keep exception applies. |
| `test_controller_targets.py:20` | Same comment | Internal fixture narration. The wide interval is explicit in `WIDE_LIMITS`; tests such as `test_mapped_torque_outside_codec_bounds_rejects` name the validation behavior they isolate. No keep exception applies. |
| `test_startup.py:1` | `"""Verify the real dashboard entry point starts and closes without motor I/O."""` | Narrates the verifier's purpose rather than defining its callable inputs, outputs, or caller obligations. `ForbiddenMotorIO` and `validate_report` express the motor-I/O checks. This docstring already existed at `4d41d5b`; it is included because the assigned scope covers comments throughout each changed file. |

## Retained docstrings and exact exceptions

| File and line | Symbol | Keep exception and supporting code |
| --- | --- | --- |
| `joint_controller.py:1-5` | Module / `JointCore` | Public API contract. It requires one calling thread and promises no owned transport and immutable returned records. `JointCore` mutates its own lifecycle and reference state without locks, uses no serial transport, returns frozen dataclasses, and builds snapshot collection fields as tuples. Thread confinement is a caller obligation; the code does not enforce thread identity. |
| `joint_controller.py:322` | `Reference` | Public API contract. It distinguishes the preserved requested target from the complete bounded, codec-representable issued vector. `consume_target()` projects every requested joint with `_project_joint_target`, then returns `Reference(target, tuple(candidate_joints), pending.accepted_at_s, release_now)`. `_project_joint_target` verifies the position corridor, velocity bounds, and exact codec grid codes. This wording describes records issued by the core; the dataclass constructor alone does not establish configured joint bounds. |
| `joint_controller.py:426` | `JointCore` | Public API contract. It states the caller's thread-confinement obligation and the fixed ordered joint set. The constructor stores `tuple(joints)` and derives ID-to-index lookup from that order; `joint_specs` and `snapshot()` expose the same order. |

The module and `JointCore` docstrings already existed at the comparison base. The `Reference` docstring is new in the audited diff.

## MUST KILL flags

None. The deletion candidates are narration around already named values and explicit assertions. No comment conceals an internal behavior that requires a rename, extraction, type change, or redesign.

## Files inspected and skips

- `joint_controller.py`: no comments; three public contract docstrings retained.
- `scripts/check_milestone.py`: no comments, docstrings, or suppressions.
- `test_controller.py`: one deletion candidate.
- `test_controller_limits.py`: no comments, docstrings, or suppressions.
- `test_controller_ownership.py`: one deletion candidate.
- `test_controller_targets.py`: one deletion candidate.
- `test_startup.py`: one existing module-docstring deletion candidate.
- `test_controller_velocity.py`: included despite being untracked; no comments, docstrings, or suppressions.

Markdown documentation, shell scripts, unchanged Python files, behavioral testing, and source edits are outside this report's assigned scope. No foreign dependency or protocol comment needed an additional `/how` or `/why` investigation.

## Concurrent writer and audited snapshot

The parent reported an active writer correcting release-time checks in `JointCore.consume_target`. The inspected method already includes the second `release_now` clock sample and its health, ownership, pending-object, and expiry checks. `Reference` is the only retained docstring whose supporting path touches that changing method. The keep decision depends on the final method still producing the complete projected vector. The parent should recheck this small inventory after any further writer changes.

SHA-256 values for the final inventory read:

| File | SHA-256 |
| --- | --- |
| `joint_controller.py` | `181d88e9890477576ae2fb9df5e8402f6f6b9666f31ae12eb14054e3f975ba3e` |
| `scripts/check_milestone.py` | `97d63f128e98900224b2e24b8ec3e2826efb81756f2f783aabd2139704162a41` |
| `test_controller.py` | `fd7c5e8b2179427905fde7bb14d09980a193d1bbc7d58914efc9e040096da43c` |
| `test_controller_limits.py` | `40cd9bb96977208427f7df67d0437cb34f24ca6f1cb59779274faec4e8f0c9a0` |
| `test_controller_ownership.py` | `beba2cf9cfec9b1be17541af510bab8781f0a4e85349e4fc361c14e3d35cd0db` |
| `test_controller_targets.py` | `2d7a66a9749b0de5cbdd83af19c714d809ceac2fcd625e79c93534f778abb369` |
| `test_startup.py` | `e41d33d645c7f78c207a2e14de952297659d32593675715770da8fe7394bfaa9` |
| `test_controller_velocity.py` | `ded9e140df3c52a0baca065e8c9c5dcdb9ccea5df90ef523701adb95a00469b7` |
