# Independent combined C11–C16 source review

What this change does: C11–C14 extend the transport-free joint controller with acceleration and measured-lead corridors, encoded combined-effort pacing, and explicit temperature and sustained reported-effort protection. C15 adds observations to the existing production owner; C16 fixes dashboard cleanup around that owner. The production owner still does not use JointCore.

**FINAL verdict: fix finding 1 before accepting the combined software release.** I found one concrete verification gap and no additional runtime correctness or structural blocker in the inspected source. This verdict applies to the exact artifact below. It does not authorize merging PR10 or PR11, deployment, service changes, physical I/O, or a physical protection claim.

## Artifact and work actually performed

- Checkout: `/tmp/c16-codex-20261017`.
- Reviewed head: `71a0a5f5123f7b64288b795cc1f39330c89c5fe0`.
- Comparison base, PR10 head: `8e89b75395edadb14d0db2075cd4a62edbd1d1a0`.
- The checkout was clean. All **31/31 authored file SHA-256 values** in `/tmp/controller-release-review-20261017/source-hashes.json` matched the actual files.
- I inspected the actual Git diff, complete current JointCore/JointSpec/JointReference code and connected mapping/codec code, migrated callers, relevant tests, requirements and docs, and saved C11–C16 evidence. Historical evidence copies were distinguished from current callers.
- I independently compared the ArmManualSession method ASTs against the base without importing product code: 65 methods are unchanged; only `__init__`, `snapshot`, `_transact_raw`, and `_loop` differ. No owner methods were added or removed. The `arm_web` diff is empty.
- I ran **no tests, product checks, network/provider requests, hardware, device, serial, or service operations**. Hash calculation, source parsing and read-only Git inspection are the only computations supporting my independent source confirmations. Saved test outcomes below are evidence I read, not checks I executed.
- Only this report was written. Repository files and Git metadata were not changed. No subagents were dispatched.

## Must fix

### 1. [P2] The final measured-lead veto has a test that never reaches it

**Location:** `test_controller_lead.py:381–409`, especially `shifted_now` at lines 391–404. Connected production flow: `joint_controller.py:534`, `556–570`, `603–614`; planner corridor intersection: `joint_reference.py:347–350`.

**What this is:** `test_release_time_lead_check_veto` is intended to prove that feedback changing after planning can veto an otherwise valid candidate at the final release guard. The saved C12 Gemini review explicitly relies on this test for that claim (`evidence/controller-commands/units/C12/c12-review-gemini.md:87–97`).

**Problem and concrete path:** The replacement `_now` changes the observation on its **first** invocation. `consume_target()` invokes `_now()` at line 534, before collecting measured positions at line 556 and before calling the planner. Therefore the planner already receives `q_meas=-0.2`, not zero. With the test's seed `q0=0`, `V=1`, `A=2`, elapsed time `0.02`, and previous position rate zero, the position-rate corridor is approximately `[-0.0008, 0.0008]`. The shifted measured-lead corridor is `[-0.25, -0.15]`. These do not overlap, so the planner returns `None` and consumption exits at lines 569–570. The final release sample and lead veto at lines 610–611 are never reached. The comment at test line 405 incorrectly claims that the candidate was planned using zero measured position.

Consequently, deleting the final measured-lead veto would leave this test passing. That conclusion follows from the source path; I did not execute that mutation. The C13 effort test correctly shifts velocity after a real planner result, but its measured position stays zero, so it does not cover this separate lead guard.

**Smallest fix:** Replace this clock monkeypatch with a wrapper around the real `joint_controller.plan_reference`. Require a non-`None` candidate, then submit a fresh shifted position through `core.receive` before returning that candidate. Zero gains/feedforward can isolate the measured-lead condition from the later effort condition. Assert that the complete issue is suppressed, the exact pending target and ownership remain, and a retry after restored feedback matches an uninterrupted control core, establishing retained history as well as the veto.

**Required evidence:** The parent should execute the corrected case on the current source and a disposable mutation removing only the final lead veto. The normal source must pass; the mutation must fail because an issue is incorrectly emitted. Then refresh the affected test/source fingerprints and release checks using the resulting exact head.

**If skipped:** The release packet claims final-gate coverage that would not detect removal of that gate. The current production guard itself looks correct; this is an acceptance-evidence defect, not a claim that the current source emits an out-of-bound reference.

## Cross-unit source assessment

| Area | Independent source conclusion |
| --- | --- |
| C11 acceleration | Position finite-difference history and desired-speed history remain separate. Planning uses the capped step for displacement and desired-speed slew, and uncapped elapsed time for position-rate acceleration. Release recomputes position rates using the release epoch. An infeasible joint suppresses the complete vector. Suppression and valid target replacement do not reset issued history. |
| C12 measured lead | The lead corridor is intersected with the static, displacement and position-rate corridors before projection. The final runtime guard reads current observations and uses a strict closed bound. Distant submitted goals retain C09 acceptance semantics. The runtime implementation is coherent; finding 1 concerns its purported final-gate test. |
| C13 combined effort | Position, desired speed, feedforward and gains are actually encoded/decoded before effort evaluation. Gain conversion uses reduction squared; signed mapping is preserved. Pacing intersects the signed effort corridor with the already bounded step segment, preventing effort cancellation from inventing a reverse step while retaining legitimate braking. A `kp=0` infeasible candidate suppresses instead of creating a false position solution. The final effort guard reads current position and velocity. |
| C14 protection lifetime | Protection history is separate from reference history. Stop clears reference/ownership but preserves exposure and the overload observation anchor. Disconnect, connection and communication failure break continuity while preserving confirmed exposure. Beginning a check resets recovery credit without erasing overload duration. Acknowledged stopped recovery is bounded and contiguous; new unhealthy feedback returns to FAULT. Both temperature channels are explicit, unknown and hot causes can accumulate, and a successful recovery grants no owner. |
| C11–C14 interaction | Reference suppression, effort pacing, target replacement, expiry and release veto do not clear measured overload exposure. A temperature or persistent-effort fault clears pending/reference state and revokes ownership; subsequent release state/identity guards prevent a candidate from being committed. Current freshness, strict bounds and sequence watermark rules remain present. |
| C15 owner observations | The clock for observations is independent of the authority clock. Transaction end sampling occurs before the publishing-lock wait, on success and failure. Invalid observations become unknown measurements while retaining the exchange outcome. Counters use fixed storage; HTTP has an explicit scalar allowlist. Existing owner Enable, Stop, lease, ticket, generation and movement methods are unchanged by source comparison. |
| C16 cleanup | Server construction is inside the owner cleanup scope. Nested `finally` ensures an owner-close attempt even if server cleanup raises. The HTTP server receives the existing owner. Existing observer creation only allocates session state; it cannot transfer authority or rearm. The established owner input-expiry path remains independent of HTTP requests. Cleanup attempts are not represented as physical Stop confirmation. |

The required constructor contracts and canonical imports are migrated in current repository callers and the documented example. The baseline copyset and startup fingerprints include the new modules. Required protection values have no fabricated physical defaults. Historical probe scripts retain their original API context and were not treated as current executable examples.

## Saved evidence inspected

- **C11/C12:** Both saved `baseline/checks.tsv` files contain nine zero-exit groups. I read the C11 24-trial/3,360-tick probe record and the C12 review/triage. Those records do not resolve finding 1: the C12 reviewer specifically cites the misleading test.
- **C13:** The accepted packet retains the macro-progress and reverse-cancellation failures and corrections, decoded mapping probes, final effort-feedback-shift veto and recovery evidence, cleaned-source equivalence evidence, and the independent review. Current `_pace_position_toward_effort` and the C13 test now retain the intended step segment and shift feedback after successful planning, respectively.
- **C14:** `stop-check-before-after.json` identifies the original failure and corrected pass, with the corrected controller hash matching this head. The saved recovery-epoch script and result separately test loss of recovery credit between acknowledged checks. I read the source and test paths for repeated Stop/check, reconnect, communication gap, unknown/hot channels, and independent joint histories.
- **C15:** `lock-boundary-before-after.json` records the actual original 52 ms observation for a 2 ms fake exchange plus 50 ms publishing-lock contention and the corrected 2 ms observation, on success and failure. The current code samples the endpoint before that lock. The five-run records and saved worker profile concern DemoSerial; they establish no physical rate or speedup. Earlier enumeration and fixture failures remain disclosed.
- **C16/current baseline:** Saved `checks.tsv` has nine successful groups; `milestone-tests.log` reports 256 tests, and the legacy dashboard record contains 19 groups. The guarded arm result reports 79 successful tests. Seven focused C16 cases are recorded. These are overlapping evidence groups and are **not summed as unique tests**.
- The current parent proof records four captures inspected by the parent and 39 matching fingerprints in each disconnected startup mode, with zero serial/demo opens or exchanges. I read these records; I did not personally inspect the images or rerun startup.
- `/tmp/c16-codex-evidence-20261017/exact-head-ci.json` records success for head `71a0a5f` in run `39069353294`. `pr-checks-green.log` records two passing offline checks, for runs `39069353294` and `39069635229`. The earlier PR snapshot shows the second run in progress, so the later check log is the terminal evidence. I made no live CI request.
- The combined Muse receipt records an actual HTTP 400 failure, exit 1, one submission and no review. It is a dropout, not approval. The independent conclusion here does not substitute for that configured seat. I did not use the combined Gemini review's conclusions in reaching my own findings.

## Comments, deslop and maintainability audit

The decomposition improves the source boundary: JointCore is 675 lines, down from 976 at the base; public records occupy 478 lines and pure reference math 503. StationTiming is 84 lines. The already large production owner receives a narrow observation change, with its other 65 methods unchanged. No runtime file crosses from below 1,000 lines to above it in this change. I found no concrete missed decomposition, new owner, unnecessary compatibility layer, speculative physical default, correctness suppression or broad policy branch that warrants an additional release blocker.

The runtime docstrings describe API contracts, numerical semantics or ownership boundaries and earn their place. I found no added lint/type suppression or comment asserting an unenforced runtime constraint. The lead-test clock hook and its false narrative are the specific reshape target in finding 1; moving the feedback change after the real planner makes the test timing explicit through executable assertions.

There are 39 `#` comment lines in the new lead test, plus test-module coverage inventories, that mostly repeat scenario setup or assertions. Removing that narration is optional cosmetic cleanup under the deslop review; it does not establish a second correctness finding. This was a nonwriter audit: **0 comment deletions, 0 restorations, 0 reruns, 0 new suppressions, and no application-code edits**. No outside constraint needs an approval flow or a new architecture sketch.

## Remaining boundaries

The required parent action is the concrete regression correction and verification in finding 1. After it, source hashes and exact-head evidence must describe the new artifact. The runtime review otherwise found no combined C11–C16 blocker.

PR10 dependency handling, PR11 draft status, merge/deployment approval and physical hardware work remain separately held. Passing software checks cannot establish loaded-arm stopping, actuator effort accuracy, replacement-adapter communication or production integration of JointCore. No new checks were executed by this reviewer.
