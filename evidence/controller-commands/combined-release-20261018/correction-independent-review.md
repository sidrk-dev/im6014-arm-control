# Independent review of the final-lead test correction

**FINAL verdict: the bounded correction resolves finding 1 from my original combined C11–C16 review.** I accept this correction as software test evidence. No additional finding was identified in this diff. Full baseline, fingerprint refresh, commit and exact-head CI remain parent-owned and were not complete when this review was assigned. This is not merge, deployment or hardware authorization.

## Artifact and scope confirmed

- Original artifact: `/tmp/c16-codex-20261017`, head `71a0a5f5123f7b64288b795cc1f39330c89c5fe0`.
- Corrected checkout: `/tmp/controller-release-fix-20261017`, based on that same head with only `test_controller_lead.py` modified in its working tree.
- I compared all 31 authored files against the original. Only `test_controller_lead.py` differs. Its SHA-256 is `e751e1a6e57162b275bd986676659886e8fa12378a2eb1ac03632fb0a97e8bf9`.
- Runtime `joint_controller.py` retains SHA-256 `4f9e4fb8865052787db085cbf900c52c1eac7e5706984de3726f645e4f7d1f17`.
- All 31 hashes in the original checkout still match the original manifest. The original full report was preserved, with SHA-256 `e94e1045493c513f60e98de6a5f83424c107fc6cb33e84d2876ef205d7b59fa0`.
- I inspected the exact source diff, connected planner/consume/station behavior, the saved normal and mutation logs, mutation result JSON, the earlier failed invocation and the corrected three-module log. I executed no tests, providers, network, hardware, device, serial or service operations. Only this correction report was written.

## Why this resolves the finding

The corrected `test_release_time_lead_check_veto` at `test_controller_lead.py:381–418` wraps `joint_controller.plan_reference` and invokes the actual planner first. It requires a non-`None` result, then changes the measured position through `core.receive` before returning that candidate. Therefore the ordinary candidate planning succeeds using the original observation, and the release path must inspect the later observation. The old first-clock hook and its incorrect explanatory comment are gone.

The target now has zero feedforward, position gain and damping gain. This makes predicted effort zero and isolates the measured-lead veto from the following effort veto. Feedback stays fresh, healthy and in the same monotonic epoch, so stale-feedback or fault rejection cannot replace the intended condition.

The test verifies one planner invocation, no reference issue, no station packet construction, preservation of the exact pending target and ownership object, the ARMED state and an absent fault latch. It then restores the observation and compares the resulting reference values and issue time with an uninterrupted control core. This supplies the requested recovery/history evidence as observable behavior.

## Saved execution evidence read

| Evidence | Recorded result | Independent interpretation |
| --- | --- | --- |
| `lead-corrected.log` | One corrected test, pass | The normal runtime satisfies the strengthened case. I read the log; I did not execute it. |
| `lead-mutant.log` | One test, failure at `assertIsNone(issued)` | Removing the final lead guard causes an unsafe-for-the-configured-lead reference to be emitted. This is the required sensitivity failure, not an import or fixture failure. |
| `lead-mutation-results.json` | Corrected exit 0; mutant exit 1 | Consistent with both raw logs. |
| `corrected-controller-tests.log` | Earlier run failed importing nonexistent `test_controller_protection` | This remains failed evidence and is not counted as a passing suite. |
| `corrected-controller-tests-final.log` | 46 passing tests across lead, acceleration and effort | This is the corrected three-module run. The individual and mutation runs overlap these tests and are not added as unique coverage. |

I also inspected the actual mutation. The mutant and corrected `test_controller_lead.py` files are identical. Among 37 Python files present in both root copysets, only `joint_controller.py` differs. Its diff deletes exactly the two-line final measured-lead condition and return at original lines 610–611; the later effort check remains.

The mutant's recorded emitted position is `0.000787174351678158` rad after measured position changes to `-0.2` rad. Their separation is about `0.200787` rad, exceeding the configured `0.05` rad lead. With zero gains and feedforward, the final effort guard cannot hide the removed lead guard. The failure therefore directly demonstrates the protection intended by the corrected test.

## Maintainability and remaining gates

This is a focused test-only change. It replaces private clock/observation mutation with the same explicit post-planning observation pattern used by the effort regression, adds behavioral assertions, and introduces no runtime abstraction, policy, suppression or explanatory workaround comment. No broader restructuring is warranted for this correction.

My original review remains an honest report of head `71a0a5f`. This addendum closes its single verification finding for the corrected test bytes above. The parent must still refresh manifests and the relevant release evidence for the resulting committed artifact and complete its full baseline and exact-head CI gates. The software source review is not evidence of physical protection, production JointCore integration, merge readiness or deployment approval.
