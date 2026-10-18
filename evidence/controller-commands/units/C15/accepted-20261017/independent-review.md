What this change does: C15 adds host observations of the existing ArmManualSession owner loop and exchange attempts. Snapshots and GET /api/state report loop periods, the existing 80 ms overrun budget, transaction duration and outcome counters, and the existing separate feedback ages. It adds no bus owner, thread, motion permission, or controller limit.

Verdict: Accept the corrected C15 software scope. No concrete must-fix findings.

## Review identity and limits

- Reviewed checkout: `/tmp/c15-codex-fixed-20261017`.
- Independently confirmed HEAD is accepted C14 `f43d7aae87b01f8c41ae3be89491ad91d35973c4`; inspected its tracked diff and all four new files.
- Read `AGENTS.md`, the workbench requirements and C15 criterion, milestone ledger, architecture, timing documentation, supplied worker contract, and connected source and fixtures.
- Assumed the existing single serial owner with concurrent snapshots and local HTTP clients. These counters belong to that owner instance; they are not shared across station processes.
- This was an independent read-only review. I ran source reads, read-only Git inspection, and SHA-256 comparisons only. I ran no tests, workload, device discovery, HTTP request, hardware operation, installation, provider call, or subagent. I edited only this report.

## Source identity

All 10 entries in `fixed/review/source-hashes.json` independently matched the actual checkout bytes. They were checked again after the review.

| File | Matching SHA-256 |
| --- | --- |
| `station_timing.py` | `3b3287331739baa21b3229d88c00c9515af914dc53fd745d8d0e7aa6ed11681c` |
| `arm_manual.py` | `fde1954031c442dd671f8c1df792bf140803b67bf71f598341c3ece868e567de` |
| `arm_dashboard.py` | `3ddc867ef5c62d72fd9225eb768fe4f5cfb8aac758f220ce7c12b69303404569` |
| `test_station_timing.py` | `8002de43659e7d5b1b6a4465b825fc68f746759b6840053da89d82d0fcbe5e0a` |
| `tests/test_arm_timing.py` | `6f0f83f3cadb1df5737b623dfe26998b3bf2a7783e8c40285e6cc9fa46fda67d` |
| `scripts/check_milestone.py` | `715d2a8ae8e50f46ef7f7c05f17483efe056b1ff0a21a3caeed2786e23e06e77` |
| `scripts/check_offline_baseline.sh` | `37854f289ff4727827bca915c9f0c2f28ab38d64ca913796cbf3d284c3e730eb` |
| `test_startup.py` | `b1a44aa57e0f00d6ad011640c4a713bd0b0e5163e969042e0b8416d2c8594e56` |
| `docs/station-timing.md` | `4735884f105bed69f47a564c74250d2ceee2bbe0aaee839c166f5ead1cb8bdf3` |
| `docs/architecture.md` | `ae8c77205b348f2ceda79b9f06a1a9ea149647c319b024d63b141606ea7d3b47` |

## Source review conclusions

- **Measurement boundary:** `arm_manual.py:1034-1060` samples the start immediately before `capture_exchange` and samples completion in `finally`, before acquiring the counter-publishing lock. Both return and exception paths use this boundary. Queue waiting and the later publishing-lock wait are excluded. The duration includes the existing exchange wrapper and capture bookkeeping; it is host exchange-attempt latency, not wire-only or actuator response latency.
- **Failures and cancellation:** The same `finally` counts every completed attempt, including a pre-write cancellation. It does not increment a successful-write counter. I traced `capture_exchange` and the real `exchange` cancellation path (`im6014_serial.py:230-281,378-381`): a false pre-write hook raises before `bus.write`. Timing publication does not catch or replace that transport exception. Existing Stop and fault handling still receive it.
- **Broken observation clock and close:** Clock exceptions become unavailable samples. Invalid or backward readings are rejected by the pure accumulator and converted to measurement errors by the owner. The original transaction outcome is retained. Loop failure clears the timing anchor, so recovery does not manufacture a long interval across missing observations. This independent clock does not replace the authority clock. I traced `close`, `_service_close`, `_stop`, `_close_resource`, and `_trip_fault`; ordinary clock exceptions and invalid readings do not bypass their work.
- **Counting and storage:** `StationTiming` stores counters, extrema, latest durations, and one anchor in fixed slots. It retains no sample list. The first loop entry is only an anchor, zero duration is valid, and overruns use strict `duration > budget`. Unavailable duration and valid zero remain distinguishable. Connect/disconnect resets joint authority but does not replace the timing accumulator; counters therefore persist for the owner lifetime as documented.
- **HTTP and feedback:** Snapshot publication remains under the existing lock. The new nested `TIMING_FIELDS` allowlist reuses finite scalar projection (`arm_dashboard.py:60-64,133-145,223-226`); unknown members and nonfinite numbers are omitted. Per-joint feedback-age calculations remain separate. The current timing HTTP fixture replaces `_get_serial_ports` and also makes `enumerate_ports` fail if called, so its request does not depend on the global port cache suppressing discovery.
- **Quality:** I applied the requested no-comments, deslop, and strict code-quality lenses directly. New application commentary is limited to the module purpose. The accumulator has one cohesive job, and the observation fault handling sits at its two owner integration boundaries. The broad `Exception` handling has a concrete purpose: an observation failure must not interfere with the action being observed. I found no new suppression, workaround comment, duplicate timing implementation, unnecessary wrapper, structural regression, or obvious simplification that warrants blocking this scope. The existing large owner module already exceeded 1,000 lines before this change.

## Parent-run evidence inspected

These are results recorded by the parent, not checks I executed.

| Evidence | Recorded result and interpretation |
| --- | --- |
| `fixed/pure.log` | 10 pure timing tests pass, including budget equality, invalid samples, continuity recovery, detached snapshots, and fixed storage. |
| `fixed/owner-checks.log` | 12 owner checks pass, including both contention outcomes, broken/invalid clock through close, original failure text, real cancellation with zero writes, reconnect counters, and HTTP projection. |
| `lock-wait-before-corrected-fixture.log` | Retained original implementation fails both success and failure contention cases: 0.052 seconds instead of 0.002. The corrected final owner log passes the same named regression. |
| `fixed/check-arm-no-enumeration.py`, `fixed/guarded-arm.log`, `fixed/guarded-arm-result.json` | Final guarded run records 79 passing arm tests and zero physical serial constructor calls. The guard supplies `comports`, replaces by-id lookup, and blocks serial construction before loading tests. Nested port-classification fixtures remain effective. |
| `fixed/five-runs.json`, `fixed/assess-timing.py` | Five separate unprofiled DemoSerial workloads each record 63 mode-0 exchanges across IDs 0, 1, and 2. Transaction counts match the workload counter; failures and measurement errors are zero. The profile is a separate sixth run and is not used to assert a speedup or physical control rate. |
| `fixed/baseline.log`, `fixed/baseline/run-20261017T204653Z-vBCE8ctr/checks.tsv`, milestone log | The baseline completed during review: all nine groups record exit 0; the milestone group records 249 passing tests. I read the results; I did not rerun them or visually review the generated screenshots. |

Earlier failed evidence remains material and is not a passing result. `guarded-arm-regression.log` and its result JSON retain the original 75-test run with three failed port-classification fixtures: the USB adapter disappeared, enumeration failure lost its error, and the legacy default became empty. A global empty handler snapshot bypassed those tests' supplied port fixtures. The final runner fixes that fixture boundary by replacing the lower discovery functions before test loading. Separately, `fixed/owner-checks-first.log` retains one failed failure-path fixture assertion and one denied local HTTP socket; it is superseded by the final 12-pass log, not relabeled successful.

## Acceptance boundary

This verdict approves the reviewed software timing behavior. It does not establish physical RS485 latency, actuator tracking, stopping time, achievable control rate, production station integration, or deployment readiness. The transport-free controller is unchanged. Git actions, ledger acceptance, checks, installation, deployment, and any physical operation remain with the parent under its existing authorization.

Not checked: tests were not independently executed; hardware, deployment, real bus throughput, and screenshot appearance were not validated in this review.

Verdict: Accept corrected C15 software scope; no must-fix findings.
