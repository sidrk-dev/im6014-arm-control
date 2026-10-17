# C15 software acceptance

Codex directly implemented this checkpoint from accepted C14 f43d7aa. The source manifest pins the ten reviewed runtime/test/document files. Original source, failed checks and raw reviews remain retained. Existing agent workspaces and live services were preserved.

The existing ArmManualSession owner now reports observed loop intervals, exchange-attempt latency/failures, overrun count and its existing per-joint feedback ages through snapshot and the explicit HTTP allowlist. One fixed-storage StationTiming record uses an independent observation clock and no new owner, thread or motion policy. Unknown durations remain null. Counts persist across reconnects within the owner lifetime.

## Actual verification

The focused checks passed10 pure and12 actual-owner/HTTP tests. The full guarded arm suite passed79 tests (including those12 owner checks). The offline baseline passed all9 groups, including249 milestone tests and19 legacyQt groups. The parent viewed all4 actual captures; both disconnected startup modes matched36 current hashes and0 serial/demo opens or exchanges. A source AST comparison proved65 other owner methods unchanged.

The retained original timestamp boundary reported52ms for a2ms fake exchange plus50ms post-exchange lock contention. Both success and failure regressions failed before correction and passed afterward. End sampling now precedes the publishing-lock wait. Broken/invalid clocks through close, original error preservation, real pre-write cancellation with0writes and reconnect counter continuity passed.

Five separate unprofiled DemoSerial runs each completed63 mode0 exchanges across IDs0/1/2, exactly matching timing counters. A separate worker-thread profile identifies wait/poll scheduling as the fake workload limiter. No speedup or physical rate claim follows.

Native Sol and configured Gemini independently found the original lock-boundary issue. A fresh nonwriter Sol accepts the correction; corrected Gemini returned no findings. Muse HTTP400 dropouts and modelVerifiedfalse/pinned-argv provenance are retained honestly. See review-triage.json.

## Verification boundary correction

Earlier full arm tests left an old HTTP fixture's serial metadata discovery unpatched; those logs are not proof of no enumeration. They did not open motor ports or command physical hardware. The first guard then broke3 supplied classification fixtures. The final guard replaces comports, by-id lookup and physical serial construction before loading tests while allowing their nested fake fixtures; all79 tests pass. Original failures are retained.

## Limits

These are software and fixture results. Host exchange-attempt latency includes existing exchange/capture bookkeeping, not just wire time. Physical RS485 latency, actuator tracking and achievable control rate remain unverified. Hardware I/O stays blocked after the adapter incident. Pure JointCore production integration remains separate. This is not a merge or deployment.
