# C10 review-fix handoff

In `joint_controller.py`, `JointCore.arm()` and `JointCore.consume_target()` now let programming `TypeError` exceptions surface while continuing to reject expected `ArithmeticError` and `ValueError` projection failures. `consume_target()` checks expiry without the unreachable accepted-time comparison because clock samples are globally monotonic.

In `test_controller_velocity.py`, all 16 C10 behavior tests remain. The two injected-TypeError regressions assert public state and packet behavior. Pending target, state, grant, fault, and sequence checks use `Snapshot`; reference-history checks use actual returned `Reference` values and bounded follow-up requests. No direct `JointCore` private-attribute assertions remain. Clock-regression cases restore the fake clock to the last good sample before taking a snapshot.

Changed files: `joint_controller.py`, `test_controller_velocity.py`.

The worker ran no tests. The parent reported that all 16 tests were run before these fixes, with the original 14 passing and both TypeError regressions failing as expected. Parent-run post-fix checks remain required.
