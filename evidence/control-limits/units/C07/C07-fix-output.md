# C07 authoring report

Implemented the assigned C07 fixes in the workbench across `joint_controller.py`, `joint_contract.py`, `im6014_serial.py`, the three controller test files, and `docs/joint-api.md`.

- `submit_target` now performs stale-feedback and admitted-goal expiry housekeeping before validating a replacement. Expiry at equality withdraws the old goal, moves the core to `STOPPED`, preserves the sequence watermark, and rejects the replacement. Stale feedback latches attributed faults and clears motion. Invalid replacements preserve an unexpired healthy goal.
- Dispatch now has one private claim record holding the plan, joint index, and valid claim-start time. It blocks reprepare and the next claim until an outcome arrives and survives Stop, expiry, fault, target replacement, or disconnect. Complete receipts are retained after invalidation but only a still-current plan can transition the core to `MOVING`. A bad end clock after a known complete write retains the record and attempt outcome, then latches `DRIVER_FAILURE` and withdraws motion.
- `Snapshot.dispatch_attempts` stores the latest bounded attempt per joint, distinguishing `COMPLETE`, `PARTIAL`, and `FAILED` while `dispatched_records` remains limited to complete writes. The API docs identify `dispatched_at_s` as the valid claim/write start time.
- `exchange` classifies and reports the write result immediately after return, then checks the deadline before feedback. Raised writes report `FAILED` once. Callback failures stop the transaction without retrying or reading feedback.
- Removed the `pending_target` aliases and contract-record `joint_controller.__all__` reexports, retained `JointCore` as the module export, migrated callers to `joint_contract`, and removed redundant integer/bool checks. Updated the ten-thousand-target fixture to use a test-only freshness limit beyond its goal expiry.

Added regressions for expired-goal renewal, valid versus invalid replacement handling, complete and partial writes at deadline, raised writes, cancellation and callback failure safety, invalid/regressing core end clocks, serialized claims, duplicate/out-of-order receipts, and Stop/expiry/replacement invalidation.

Corrected the integrated end-clock regression so `exchange.before_write` owns the only claim while the test retains the valid claim-start timestamp for its assertions.

Closed a post-claim/pre-write deadline gap: if the final deadline gate fails after the pre-write hook grants a claim, `exchange` reports one `FAILED` result to resolve and fault that joint even though `bus.write` was never called. Added a deterministic regression for zero writes, one failed receipt, attributed fault, and acknowledged check recovery. Documented that this `FAILED` outcome can represent a known zero-write case.

No tests, scripts, packages, hardware, or network operations were run for this follow-up. The parent reported the preceding snapshot passed targeted C07 (24 tests), all four external reproductions, and affected core/serial regressions (89 tests). This zero-write deadline change and its regression await parent checks.
