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
