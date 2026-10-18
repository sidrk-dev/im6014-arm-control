# C14 independent Codex source review

What this change does: C14 requires an immutable protection policy per joint and an explicit finite or unknown temperature in health feedback. It adds per-joint latched thermal, unavailable-temperature, and sustained reported-effort causes, with acknowledged stopped recovery.

**Verdict: changes required. Fix finding 1 before acceptance.** This verdict applies to the pinned artifact below, before any parent corrections.

## Artifact and inspection boundary

- Checkout: `/tmp/c14-codex-20261017`, based on accepted C13 `a64407937e6e64f5238ce91b26113d177d0affbd`.
- Manifest: `/tmp/c14-codex-evidence-20261017/review/source-hashes.json`. I read the actual source and diff, and SHA256 values for all 18 manifest entries matched the checkout during this review. Controller hash: `8d79153c2d204669b02c3ac3eae0d44f68a0c9abe5e600501832bec7a0dbe00e`.
- Read `AGENTS.md`, the relevant workbench requirements and ledger, `design.md`, complete controller/protection code, record and mapping contracts, all new fault tests, the reference release path, caller searches, fixture migration diffs, and changed verification registries/docs.
- No checkout edits, Git mutations, package operations, test execution, hardware operations, network requests, provider calls, or services. This report is the sole written artifact.
- Parent evidence inspected: the C14 receipt and stderr show 34 tests passing on the pinned source; `controller-third.log` reports 152 tests passing; the baseline reports 233 milestone tests passing and nine successful groups in `checks.tsv`. These are inspected parent-run records, not reviewer-run results or physical proof. I did not inspect the baseline screenshots.

## Must fix

1. **Software Stop/check calls can suppress overload accounting indefinitely** — `joint_controller.py:152-156`, `joint_controller.py:148-150`, `joint_controller.py:178-179`, `joint_controller.py:289`, `joint_controller.py:631`.

   **What this is:** Overload duration advances only when the previous and current credible samples both report overload. `_break_protection_continuity()` retains the accumulated total but discards the previous sample's time and overload flag.

   **Problem:** Every `stop()` invokes that reset, including repeated Stops while already STOPPED. Starting a software check resets it again through `_clear_observations()`. Neither operation establishes a sensor failure, elapsed observation gap, or changed measured effort. With policy `(70, 5, 1, .5, .25)`, report healthy, cool `tau_nm=6` samples at `t=0, .25, .5, .75, 1, ...`, and call `stop()` after each receipt. Every new sample sees `prior.overloaded == False`, so `overload_s` stays zero and `persistent_effort` never latches. Any already accumulated partial total can likewise be frozen below the trip duration. This is a source-derived deterministic repro; I did not execute it.

   The same defect appears in repeated checks: start STOPPED; for each sample time, call `begin_check()`, receive the overloaded sample in mode 0, call `checked()` (UNHEALTHY), then `stop()` before the next check. A continuous series of valid measured overload observations still never produces the persistent fault. No rearm or hardware command is needed to demonstrate it.

   **Fix:** Preserve the overload sample anchor across software Stop and check transitions when sensor/source continuity has not actually been lost. Clearing current observations for check freshness and invalidating recovery authorization can remain separate from measured-overload history. Actual communication failures, source changes and excessive timestamp gaps can still interrupt eligible intervals. A healthy recovery interval remains the exposure reset boundary.

   **If skipped:** Lifecycle requests can prevent the new protection cause from ever reaching its threshold, even while every observed sample reports overload. The existing test at `test_controller_faults.py:206` checks retention of a previously accumulated total across one Stop/check, so it does not catch this bypass.

   **Parent-run check requested:** Add deterministic regressions for repeated `stop()` between every valid overload observation, and repeated Stop/check/overloaded-feedback cycles. Assert that the persistent cause latches once enough credible sample intervals have elapsed. Also retain the existing real-gap, stopped recovery, and acknowledgment tests when changing the reset path.

## Integration contract gap

2. **The temperature scalar has no defined sensor identity or aggregation rule** — `joint_records.py:109`, `joint_records.py:252`, `docs/joint-api.md:144-150`.

   The actual decoder exposes both `driver_c` and `winding_c` (`im6014_codec.py:297`). Existing bench gates inspect both and use different limits (`spin_5rpm.py:27`, `arm_manual.py:1193-1197`). A future adapter that forwards only a cool driver temperature could hide a hot winding; forwarding only the winding can hide a hot driver.

   No production adapter currently calls this core, and the fixture temperatures are explicitly synthetic, so this is **not an observed production regression or an additional blocker for the stated pure-controller milestone**. Before integration, define whether the scalar conservatively summarizes every required channel, including how missing channels behave, or model the channels and their limits explicitly. The raw bench thresholds cited here are existing source facts, not newly validated ratings. Test whichever source contract is chosen.

## Other source conclusions

- Unknown temperature makes `MotorHealth.healthy` false and latches `protection_unavailable`; temperature at the policy threshold latches immediately. There is no trusted room-temperature default.
- Fault causes accumulate by OR and remain attributed to configured joints. New targets, reference suppression, healthy packets and reconnect do not clear an existing latch or grant ownership. A new unhealthy observation during CHECKING restores FAULT, so a new acknowledgment is required.
- Protection recovery requires stopped healthy samples after an acknowledged check, respects sample-gap continuity, and is checked per affected protection joint. Complete recovery returns READY and still requires explicit arm. I found no acknowledgment bypass in those paths.
- The measured-effort authority is `JointSample.tau_nm`; `JointMap.from_output` maps it once into joint units. C14 does not replace this with predicted effort. C13's planner, intended-step effort pacing, complete-vector suppression and final release veto remain unchanged.
- Exact `Fraction` timestamp differences avoid subtraction overflow and duration-rounding errors in the new counters; `_now()` rejects invalid/regressing time before observation mutation. Counts saturate at policy durations. I found no new unbounded event history.
- Active `JointSpec` and `MotorHealth` constructors were migrated, including the API example and negative-constructor tests. The new test module enters the milestone runner, offline copyset, baseline invocation and startup fingerprint. No production integration is implied.

## Added-code comment and deslop audit

This is a read-only audit against the Pstack no-comments/deslop criteria, not a claim that the full editing workflow ran.

- No new production commentary or lint/type suppressions were found. No application workaround comments require a `MUST KILL` reshape flag.
- `test_controller_faults.py:14` adds a fixture-rating disclaimer already conveyed by the synthetic source identity and API/design docs. Recommend naming the constant `SYNTHETIC_PROTECTION` consistently and removing this one added narration line. Recommended comment deletions: 1; actual deletions: 0; restored comments/reruns/encodings: 0. No architecture sketch is needed.
- The migration prepends a separate `from joint_records import ProtectionPolicy` to eight existing test modules despite their existing `joint_records` imports. In the acceleration and lead modules it also moves the existing module docstring out of its Python docstring position. Fold the name into each existing import block. This is optional cleanup, not the acceptance blocker.
- Skipped existing comments outside newly added code, physical behavior, screenshot QA, independent runtime reproduction and external-review-seat results.

**Not checked:** Reviewer-run tests, real timing, temperature/effort sensor validity, production adapters, or physical protection. Parent owns the deterministic repro and subsequent source revision.
