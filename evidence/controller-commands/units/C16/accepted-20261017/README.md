# C16 software acceptance

Codex directly implemented C16 from accepted C15 2c3bd5b after exact-head CI39068523452 passed. The source manifest identifies the seven reviewed files. Only arm_dashboard.run_dashboard changed at runtime; arm_manual remains byte-identical. Existing worktrees and live services were preserved.

## Actual behavior and verification

Two original cleanup paths left the owner thread alive: server construction failure and an exception from server_close. The retained original source and parent reproduction fail both cases. The correction places server construction inside the owner cleanup scope and makes owner close a finally action even if server cleanup raises. Both regressions pass afterward. Cleanup attempts existing Stop and resource closure; unconfirmed closure is not represented as confirmed.

Seven focused checks use the actual owner and DemoSerial, rejecting all physical constructors and metadata discovery. They cover implicit/supplied owner cleanup, all three simulated IDs receiving Stop on a loop failure, keyboard interruption, HTTP restart preserving owner/bus/write-thread identity, input expiry without HTTP requests, stale tickets, and a new observer after explicit Stop. Opening an observer does not itself interrupt a valid owner.

All79 guarded arm tests pass. The final audited baseline passes all9 groups, including256 milestone tests and19 legacyQt groups. The parent viewed4 actual captures and verified39 current source hashes in each disconnected real/demo startup record, with0 serial/demo opens or exchanges. Existing C01 tests establish cooperating port exclusion separately.

The first observer fixture incorrectly expected observer creation to Stop a valid owner. A later fixture expected lease expiry while Enable was still waiting for its first neutral input, when no input lease existed. These were fixture assumptions, not runtime defects. Raw failures and the first failed baseline remain retained. The reconstructed pending-neutral fixture matches that baseline's recorded source hash. The corrected observer fixture starts from explicit Stop; no authority code changed.

## Review and limits

Source-grounded review and actual provider provenance/dropouts are in review-triage.json and reviews. The fresh native independent review accepts software-only C16. The parent owns acceptance and Git.

No physical motor ports were opened. Hardware I/O remains blocked after the adapter incident. This does not establish physical stopping time, forced-process termination behavior, replacement-adapter communication, deployment readiness or production integration of the transport-free JointCore. No automatic rearming or new owner is added. This checkpoint is not a merge or deployment.
