# C11 history repair report

Authored changes in the isolated C11-history workbench:

- Moved `_submit_fresh_target` and the zero-objective bounded-deceleration fixture into `ReferenceAccelerationTests`, where their acceleration helpers and A=4 fixture apply.
- Hardened `claim_dispatch`: for positive cadence it now recomputes the exact rational position derivative from the proposed and confirmed endpoint positions. A mismatch latches the joint constraint fault and Stops before a dispatch claim. The existing zero-cadence exact cached-reference path is unchanged.
- Reworked the position- and encoded-stream gate fixtures to use canonical decoded packets and self-consistent endpoint, target, packet, and history data. Each fixture isolates its intended acceleration stream and checks history remains unchanged with no dispatch attempt.
- Added a focused regression for a forged zero derivative whose endpoint position changed, plus API/design wording for the final-gate invariant.

Files authored: `joint_controller.py`, `test_controller_limits.py`, `docs/joint-api.md`, and `docs/controller-limits-design.md` in the assigned workbench.

No checks or test code were run in this authoring-only lane. Changes are authored, not verified; parent verification is pending.
