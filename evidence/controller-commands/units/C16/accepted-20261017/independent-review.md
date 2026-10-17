What this change does: C16 makes dashboard cleanup attempt owner shutdown when HTTP server construction fails and when server cleanup raises. It adds regression coverage using the actual manual owner and DemoSerial, including HTTP restart, input expiry, and observer authority.

FINAL verdict: ACCEPT software-only C16. No blocking findings.

Reviewed artifact: `/tmp/c16-codex-20261017`, based on accepted C15 `2c3bd5bfc9613cbc6d1b6814c97db12b298c830b` (also the artifact's current HEAD). This is a source and saved-evidence review. I ran no tests, provider commands, network requests, serial discovery, device operations, or physical I/O. The only file I created or edited is this report.

## Scope and identity

I read `AGENTS.md`, the workbench C16 requirement, the architecture and milestone context, the native worker contract, the review prompt and hash manifest, the actual diff, and the connected owner/HTTP paths. C16 requires dashboard failure not to create a second owner or automatically rearm the robot; it does not require a replacement station service or production JointCore integration.

I independently recomputed all seven SHA256 values in `review/source-hashes.json`; all match. A static AST comparison against C15 identifies only `run_dashboard` as changed in the production dashboard module. `arm_manual.py` is byte-identical to C15: SHA256 `fde1954031c442dd671f8c1df792bf140803b67bf71f598341c3ece868e567de`.

## Cleanup and authority review

- `arm_dashboard.py:717-731` now places server construction, startup printing, and serving inside the session cleanup scope. A bind failure reaches `session.close()` without dereferencing an unavailable server. A normal return, serving exception, or KeyboardInterrupt also reaches cleanup. The inner `finally` ensures a raising `server_close()` cannot skip the owner close attempt. There is no retry or fallback that constructs another owner.
- `ArmDashboardServer` receives the supplied owner at `arm_dashboard.py:668`; it does not construct or replace it. Its direct HTTP lifecycle remains separate from `run_dashboard`, which already owned cleanup of supplied and implicit sessions before this change. Restarting only the HTTP server with an existing owner therefore preserves that worker and connection.
- The unchanged `ArmManualSession.close()` latches Stop and revokes tickets before requesting work from the owner thread. The worker drains pending jobs, attempts mode-0 Stop for each configured joint, and closes resources on the owner thread. `_transact_raw` enforces that thread identity. An unconfirmed resource close retains the bus/managed ownership for retry; C16 does not force-release it or fabricate confirmation.
- Owner input expiry and the mode-1 write gate remain independent of HTTP. Stop clears tickets and changes the authority generation. State reads and observer session creation do not Enable, renew an input lease, or transfer control. Explicit Enable still requires the existing Stop/Check/calibration/health gates and neutral input. Creating an observer does not itself interrupt a valid owner.

The seven new tests exercise the two cleanup regressions, supplied-owner reuse, actual three-joint demo Stop/close, KeyboardInterrupt, owner-thread write identity, HTTP restart after input expiry, and a new HTTP observer after explicit Stop. Their physical constructors and discovery are rejected or replaced with fixtures. Existing owner tests additionally cover observer rejection, queued stale tickets, teardown close failure, and three-joint Stop failures. I inspected these tests; I did not execute them.

## Saved evidence inspected

The original two-check cleanup log records both owners still alive before the fix; `cleanup-after.log` records both checks passing. `focused-final.log` is an earlier seven-test run with one failed observer fixture and must not be described as green. The first baseline likewise has `milestone-tests = 1`.

The corrected fixture explicitly Stops before testing observer restart. This matches the existing owner contract: Enable waiting for the first neutral input has not yet accepted an input lease. I verified there is no production authority change accompanying that correction.

The final milestone receipt reports exit code 0 for seven tests, and every source hash in that receipt matches the reviewed checkout. `focused-after-fixture-correction.log` records seven passing tests. The guarded arm log and result record 79 passing tests and zero physical serial constructor calls. The final baseline `run-20261017T205800Z-RMdxkcEc/checks.tsv` records all nine groups at exit code 0, with 256 milestone tests in its log.

I also read `parent-final-proof.json`. It records 19 legacy Qt groups, four captures inspected by the parent, zero startup serial/demo/exchange sentinel calls in both modes, and a reconstruction of the failed observer fixture matching the failed baseline's recorded source hash. Screenshot inspection and that reconstruction are parent attestations; I did not perform them myself.

I statically compared the baseline copy list with the startup fingerprint list: both contain the same 39 files, including `arm_manual.py`, `arm_dashboard.py`, and the new C16 tests. All 39 hashes in each final real/demo startup record match the current artifact. These are observations of parent-run evidence, not tests run by this reviewer.

## No-comments, deslop, and code-quality pass

I applied these review lenses personally under the no-subagent/read-only assignment. The production diff adds no comments, suppressions, casts, fallback ownership paths, dependencies, or speculative abstraction. The nested cleanup blocks express the existing two-resource lifecycle directly and are justified by the reproduced failures. The dashboard remains 747 lines; the new focused test module is 213 lines. No concrete structural regression or necessary cleanup was found. Source edits and comment deletions: zero.

Verdict: Ship within the software-only C16 scope. This review does not authorize merge or deployment and does not establish physical stopping time, real replacement-adapter communication, forced-process termination behavior, or integration of the transport-free JointCore with the production owner. I did not inspect rendered screenshots or independently query CI; those are parent-owned evidence and acceptance decisions.
