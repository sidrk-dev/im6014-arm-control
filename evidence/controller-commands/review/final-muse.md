Verdict: C07-C10 logic matches intent for offline fake use, but not fully proven. No physical claim made - correct. 3 correctness/robustness blockers + 3 test/evidence weaknesses must fix before `software verified` stands. Rest are future milestones already explicit.

Real blockers:

1. [Blocker - correctness/observability] `joint_controller.py:955-960 Snapshot.snapshot()` calls `_now()` which mutates `_last_now` (`547-560`) and can raise `ValueError`. Read-only diagnostic becomes fallible mutator. On clock regression you lose ability to observe FAULT/pending/watermark precisely when needed. Violates command-query separation. Docs `docs/joint-api.md:120` even documents `snapshots also require clock contract` - that is the bug.
Fix: add `_peek_now()` that validates finite but does not update `_last_now` for age computation, never raise from `snapshot()`; if `age<0` return `None`/saturate and leave fault handling to `tick()/consume_target()`. Update `docs/joint-api.md:60,120`.

2. [Blocker - silent fallback/inconsistent] `joint_controller.py:721 arm()`, `899 consume_target()` catch `(ArithmeticError,TypeError,ValueError)` and convert to reject/`None`, while `798 submit_target()` catches only `ValueError`. `TypeError` is programming error (wrong type passed to mapping/codec), not invalid joint data. Swallowing causes retain-forever livelock in `consume_target:899-900` instead of surfacing bug.
Fix: narrow all three to `(ArithmeticError,ValueError)` - minimal: change `721,899` to `except (ArithmeticError,ValueError)`. Let `TypeError` propagate.

3. [Blocker - fragile roundtrip, liveness] `joint_controller.py:808-860 _project_joint_target()` re-encodes via floats `mapping.to_output(safe_target)->command()` then requires `decoded[raw]==projected raw:854-855`. Float `direction*N` / `/N`, `60/tau` roundtrip can be off-by-1 near half-LSB for some `reduction/direction/zero_output_rad`. Result is spurious `ValueError` -> `consume_target:None` retain, even though `im6014_codec.py:114-146 command_with_raw_position(raw_position,raw_rpm=...)` could preserve codes exactly.
Fix: verify with preserved-raw encode: use `command_with_raw_position(raw_position,raw_rpm=raw_speed,...torque/kp/kd...)` for issue verification; keep ordinary `command()` only for `submit_target` validation. Same for `test_controller.py:68-81 FakeStation.consume()` - encode preserved raws or document float-equality assumption.

Test/maintainability weaknesses - would not fail without enforcement:

4. [Major - weak test] `test_controller_limits.py:174-199 test_requested_limits_are_joint_domain...` computes `expected_packet` via same `mapping.to_output(requested)`. With `limits -0.5..0.5`, `requested 0.2`, output e.g. `0.37-0.6=-0.23` both in-range. Passes even if core checked output domain. Does not prove joint-units.
Fix: add discriminating cases: large `zero_output_rad` where joint in-range but output out-of-codec-range and vice-versa, assert `POSITION_LIMIT` vs `ACCEPTED` + inward projection. E.g. `JointMap(6,-1,5.0)`, limits `[-0.5,0.5]`.

5. [Major - private coupling] `test_controller_velocity.py:92-93,101,197,237,273-276,311-315,344-345,375-379,397-399,429-431` asserts `core._reference_positions,_last_reference_at_s,_pending_target,_ownership,_state,_faults`. Bypasses `Snapshot/Reference` abstraction, locks implementation.
Fix: expose read-only `last_reference` property or assert only via `snapshot()` + `Reference.accepted_at_s/issued_at_s/joints`; delete private asserts.

6. [Minor - dead code] `joint_controller.py:880 now<pending.accepted_at_s` unreachable given `547-560` monotonic `_last_now>=accepted_at`. `886 elapsed=now-last_ref` no finite check.
Fix: delete dead clause or document reentrancy guard; add `if not math.isfinite(elapsed) or elapsed<0: raise ValueError`.

Decision trail vs supplied outputs:

- Strong: C10 4-fail-before -> pass-after. `release failing-before stderr:4 FAIL` (`clock_regression_during_projection`, `compute_delay_stamps`, `passing_deadline`, `staling_feedback`) vs `final C10 stderr:14 OK`. Supports `controller-design.md:57` final-sample fix. `crossjudge Muse` initial `child-failed PermissionError` + corrected `complete submissionCount 1 rejectedPermissions 0` supports synthesis claim of one OS submission.
- Weak/unsupported: `docs/controller-design.md:7` `Both OpenCode reviewers selected Sol. Parent scores 5,4,4,5,5,5 vs 2,3,2,2,2,3` - only Muse findings supplied, no second reviewer file, no score sheets, no `evidence/controller/models` in boundary. Downgrade to single-reviewer or supply.
- Weak: C08 `corrected two stale fixture assumptions` cites `initial-failure.log` not supplied; cannot verify test-only vs core-weakening. C09 `sandbox tmp UID explains failures` cites `sandbox-environment-failure/checks.tsv` not supplied. C09/C10 `three images inspected` - PNG bytes not in review material, `visual-review.md:5` + `visual inspection` description only. Mark as parent-attested, not independently verifiable here.
- `synthesis.md` staging contradiction flagged by Muse (point 1) was correctly fixed by per-row mandatory fields + sequential commits - no longer an issue. Precedence `shape->expiry->ownership->limits->codec->sequence` in `joint_controller.py:768-802` matches `docs/joint-api.md:72-73`; note `EXPIRY` before `INVALID_OWNERSHIP` (`test_controller_ownership.py:191-194`) is intentional but leaks expiry to unauthenticated caller - document or reorder to ownership-first if capability-first intent is strict.

Explicit non-blockers (already declared, do not flag as failures): physical station, acceleration, measured-position lead, effort prediction, physical U2D2 commissioning, mechanical travel/torque/speed guarantees (`docs/controller-design.md:63`, `docs/joint-api.md:74,116`, intent). `JointCore` required `max_reference_step_s` breaking change (`425-434`) safe because boundary states production bench/browser do not call new core - must supply explicit limits at future station integration. Core file `976` lines `<1000` - watch, no split required now.
