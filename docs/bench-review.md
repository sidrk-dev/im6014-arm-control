# B11-C01 review dispositions

Muse completed the requested free-model route with one submission, no permission requests and clean wrapper cleanup. Its model identity and cost are backend reports. MiMo failed with HTTP 502 after one submission and clean cleanup. No output was delivered; the failed seat is retained without retry. This batch review remains incomplete. The prompt SHA256 is 8bb313a789b247dbca948fc54e2be74c95d67b9c4b0ff4691c4e302b7e2775d9 and the supplied files correspond to the C01 snapshot plus parent documentation updates. Full inline material remains in the run directory.

| Finding | Parent inspection and action |
| --- | --- |
| Unknown default identity should prevent opening | Rejected as a changed contract. Passive inspection reports unknown identity; known mismatches reject. Both bench entry points require communication checks before arming. Documented the unverified-open behavior explicitly. |
| Missing is_open should release ownership | Rejected. A missing status cannot confirm closure. The serial factory contract requires pyserial open status; retaining the lease is intentional. |
| Intermediate overflow may reject a mathematically finite final result | Retained documented arithmetic rejection. Existing extreme-value test deliberately checks it; hardware-scale numeric anchors pass. |
| Excessive torque clipping and zero vectors lack tests | Added both checks; 12 parent-run mapping tests pass. |
| Known wrong adapter/VID lacks checks | Found a real gap: non-FTDI VID could be outweighed by matching labels. VID mismatch now rejects; new test includes absent and FTDI manufacturer labels. All 11 port checks pass. |
| Capture invalid clocks, capacity and provenance edges lack checks | Reproduced end-clock masking with fake I/O: one test failed with two failed subtests and six errors before correction. End time now exports None on unavailable/regressing timing, preserving exchange result/error; start numeric overflow raises ValueError before exchange. All ten capture tests pass. |
| More lock namespace/permission and minimal-mock cases | Existing tests cover unsafe directory modes, symlinks, device aliases and retained unconfirmed close. No current defect established in the extra cases; include in final review disposition. |
| Timeout boolean precedence needs parentheses | Expression inspected; Python precedence is correct. A style-only change is unnecessary. |

This review is partial. C02-C06 still require the final configured panel, comment review and independent verifier.

## Gemini preliminary review and C04 parent inspection

Gemini completed its B11-C03 read-only review and independently identified the capture end-clock failure. The parent reproduced and fixed it in 7e7b940, preserving the original result or exception and marking unavailable end timing as null. Additional suggested boundary cases remain input to final verification; they were test coverage suggestions without demonstrated failures.

The C04 Gemini writer completed in its isolated worktree. Parent inspection removed redundant aliases and custom enum equality, kept one canonical fault API, separated serial timeout from actuator watchdog timeout, and updated current communication health. Eleven C04 tests and all 25 controller tests passed after those corrections. The original draft patch and exact model receipts are retained. A parent bookkeeping script initially stored a normalized event wrapper instead of its event; this was corrected from the original receipt without rerunning inference.

Parent review also reproduced premature lease release for falsey unknown serial close states (`None`, `0`, empty list). Three subcases failed before correction. Close now requires the exact boolean `False` before releasing the lease. All 16 ownership tests pass, including regular-file/directory rejection and cleanup after failed fd verification. Expanded capture metadata/capacity rejection cases pass within the 10 capture tests. These cases address Gemini's preliminary coverage suggestions.

## Comment review

The fresh comment reviewer proposed eleven deletions and one structural flag. The parent accepted ten deletions immediately; the FakeStation docstring deletion waits for the active C05 writer to finish. Fifteen public/platform contract blocks remain. The retained Linux lockfile comment describes the external flock inode constraint, already exercised by cross-process PTY checks.

The structural flag reproduced an unreachable retained owner when the original exception refused an added attribute. The parent reused the existing `PortCloseError` API for failed cleanup, preserving the original error as `__cause__`. Both open failure and context exit now expose a typed retryable owner, with no silent attribute fallback. No new abstraction or architect sketch was needed. All 17 owner checks pass. Two parent comment-edit indentation errors were caught by checks and corrected before acceptance.

C05 Gemini implementation passed parent inspection after shared stale detection replaced the duplicate consume path, redundant typed-record validation was removed, and every fault uses one `_latch_joint_fault` operation that also clears pending. Nine C05 checks pass, including all three fault entry paths and the 10,000-update newest-target case. The deferred FakeStation comment deletion is complete. The target test class will move to its own file after the C06 writer finishes its scoped two-file task.

C06 Gemini runtime changes were accepted after parent inspection. The rejected-state test loop was corrected to construct each state through shared public-action fixtures and assert the actual state; an invalid second-joint output now proves whole-vector rejection preserves pending and watermark. Target and sequence tests moved to `test_controller_targets.py`, and test discovery plus startup fingerprints include it. All 41 controller checks pass; C06 has seven dedicated tests and C05 has nine after migration. No runtime module or new test file exceeds 1,000 lines.

Fresh C05/C06 comment review found 26 narration comments, all deleted, with no structural flags. The combined comment passes deleted 37 blocks/38 lines; none restored, no report reruns, no suppressions or unenforced constraints remain. Unused copied test imports were removed. The parent executed the offline controller documentation example successfully.

## Final panel and parent dispositions

All three seats received identical complete material at 075973b, prompt SHA256 `07d621d1ffc469b9bc1e39d43d8dce1a4b83bd89083c0c4d067444f9514a76b8`. Muse and Gemini completed. MiMo failed HTTP 502 after one submission, no output or tool denials, and clean cleanup; it remains a dropout without retry or replacement. Requested and backend-reported model evidence does not independently verify provider identity or hidden reasoning depth. Verbatim outputs and receipts are retained in evidence/controller/models. Gemini also implemented C04-C06 and reviewed B11-C03; parent tests and corrections determined acceptance.

| Seat / finding | Disposition | Source-grounded reason or correction |
| --- | --- | --- |
| Muse 1 arming in FOC | Act | Added current Stop-mode gate. Regression failed before and passes after; submission and consumption still accept healthy FOC while moving. |
| Muse 2 and Gemini 3.1 public fault inconsistency | Act | Reject duplicate union codes and a summary outside the union. Three constructor subcases failed before. Most recent repeated TIMEOUT need not be the union tail, which records first occurrence. |
| Muse 3 string error counters | Act | Typed TransactionTimeoutError/TimeoutError counters and actual CRC rejected-frame count. Five fake failure cases failed before and pass after, preserving the original exception. |
| Muse 4 numeric-string channel | Act | Decimal metadata strings match or reject known wrong channel. Public PortConfig remains strict integer. Matching-string regression failed before. |
| Muse 5 clock failure | Dismissed | Explicit monotonic clock contract; invalid clock raises before target release. Snapshot is not advertised infallible. Documented caller responsibility. |
| Muse 6 repeated health gates | Consider | Kept explicit action-specific gates. `_all_fresh` and `_latch_joint_fault` already centralize freshness and fault mutation. Stop-mode applies to checked/arm, not moving submission. |
| Muse 7 hypothetical fault truthiness | Dismissed | JointFault has no boolean override; no defect demonstrated. |
| Muse 8 and 10 precedence/error wording | Noted | Existing precedence is correct and tested; codec error describes allowed individual commands. No behavior defect. |
| Muse 9 directory guards | Consider | Root and per-user directory ownership/mode contracts differ. Preserve explicit security checks rather than generic parameterization. |
| Gemini 1.1 sequence gate order | Dismissed | Contract requires complete mapped validation before acceptance; invalid payloads take INVALID_ARGUMENT precedence. CRC is pure computation with no I/O. No measured performance regression was supplied. |
| Gemini 1.2 redundant health checks | Consider | Fault latching preempts unhealthy decisions; kept explicit consumer guards rather than deferring fault transitions. No incorrect accepted action. |
| Gemini 1.3 retained unhealthy pending | Dismissed | Public receive/communication failure immediately calls `_latch_joint_fault`, clearing pending. No reachable unhealthy pending path; all three fault entry paths exercised by C05. |
| Gemini 2.1 lock pre-creation denial | Noted | Real availability limit on hostile multi-user hosts. Unsafe namespace rejects before opening; documentation now states administrator remediation. No environment-dependent alternative namespace introduced. |
| Gemini 2.2 lockfile pruning | Dismissed | Device-number reuse uses the same inode; deleting a live/contended inode can split ownership. Persistent empty files are intentional, documented. |
| Gemini 3.2 sequence saturation | Noted | Uint64 maximum intentionally exhausts a stream; documented new-core boundary. Reserved sentinel or reset would alter C06, and epochs are C08. |
| Gemini 4.1 negative rounding jitter | Dismissed | For finite monotonic ordered readings, subtraction cannot reverse order; negative zero compares equal to zero. Clamping would conceal a genuinely regressing injected clock. |
| Gemini 4.2 bounded close wait | Noted | Existing two-second join bounds UI waiting; retained owner blocks reconnect if worker does not finish. No safe forced close while worker owns I/O. Driver guarantees and hard cancellation remain unverified. |
| Gemini 4.3 mapping protocol limits | Dismissed | Mapping is pure units/domain conversion. Canonical codec owns wire limits and submit validates all mapped outputs through it; duplicating limits would drift. |

Agreement: both completed reviewers found the public fault-constructor gap. Both raised clock handling, resolved by the documented monotonic-clock contract. Muse alone found the arm, channel-string and counter defects; all were reproduced and fixed. Gemini's repeated-cause example describes a first-occurrence union rather than chronological history. Review assertions were checked against public transitions, tests and original source.

Post-review checks: 66 affected checks pass; full nine-group offline regression passes. All four UI images inspected. Controller now has 43 checks, port configuration 12 and capture 11. The original failing logs and fresh source/artifact hashes are retained. These results use fake I/O, PTYs and DEMO; no new physical I/O.
