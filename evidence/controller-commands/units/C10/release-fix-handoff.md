# C10 release-time freshness fix

`consume_target()` now separates the planning sample from the release sample. After projecting the complete candidate vector, it takes one final monotonic sample, runs the freshness tick at that sample, and rechecks active state, the current ownership generation, and the request deadline before constructing a `Reference`. Expiry during projection discards the pending request while preserving its sequence. Stale feedback follows the existing attributed fault and revocation path. A successful reference and reference-history timestamp use the final sample; its movement bound remains based on the earlier planning sample. A clock regression during projection raises before committing candidate history.

Regression methods added in `test_controller_velocity.py`:

- `test_projection_time_passing_target_deadline_discards_without_release`
- `test_projection_time_staling_feedback_revokes_without_release`
- `test_projection_compute_delay_stamps_final_release_time`
- `test_clock_regression_during_projection_preserves_history_and_writes_nothing`

Changed files: `joint_controller.py`, `test_controller_velocity.py`, `docs/joint-api.md`, and `docs/controller-design.md`.

No tests were run by this worker. The parent ran the failing-before checkpoint at `release-failing-before/C10-20261014T053813094153Z`; all original C10 tests passed, and all four new regressions failed as expected. The parent owns post-fix checks. `joint_controller.py` remains under the 1,000-line limit (976 lines at handoff).
