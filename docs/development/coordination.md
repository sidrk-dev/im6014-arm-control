# Coordinate contributors

## Check the base before editing

Observed on 2026-10-09 around 08:00 UTC:

| Branch | Observed head | State |
| --- | --- | --- |
| `main` | `244811bb6b924355852c37cddde8d00fbb0fab50` | B11-C06 landed |
| `codex/controller-limits-station` | `9e03d918b4fb96ea58ea09b4c9e98c119a0ff32e` | C07-C15 software verified; unmerged |
| `codex/arm-jog-dashboard` | `9e03d918b4fb96ea58ea09b4c9e98c119a0ff32e` | Published arm integration base |

This is a dated observation. Inspect current Git state before starting or integrating work. The user invited an independent Opus 5.5 collaborator, whose start time is unknown. No new Opus branch or open PR was observed; that does not prove the agent is inactive locally.

The user prioritizes three-joint manual jogging. C16's station-independence candidate is rejected and retained outside active source. Remaining milestones are paused. [Tasks](tasks.tsv) records reserved and available scopes.

## Register a scope

Agree with the coordinator on task ID, owner, base commit, allowed files, dependencies, acceptance checks, and output location before editing. The coordinator is the single writer of the shared registry. An available task is a proposal, not an assignment accepted by another agent.

Use a distinct checkout or worktree and a distinct branch. Never share an index or edit another worker's checkout. Do not rename paths an active writer needs. Keep the integration head fixed until dependent workers settle and their source is inspected. Recheck remote and local changes before applying a handoff.

Pstack workers obey their prepared contract, with parent-owned Git, installations, checks, and hardware. Independent user-approved collaborators can use their own accounts and branches and submit draft PRs. The integration coordinator alone lands work on the active branch. Keep model routing in the operator's existing configuration.

## Reserve active control work

The arm team owns model conversion, `arm_station.py`, `arm_dashboard.py`, `arm_web/`, the arm dispatch guard, their tests, and integration. The shared station step, controller, port, exact reference, and protection boundaries remain under the integrator's review. Consult the registry before editing any of these files.

Separate proposals for Opus are:

- Read the accepted controller/protection/timing code and report concrete offline findings in `docs/reviews/opus-control.md`. This is a review assignment; runtime edits require a new scope.
- Write `docs/user/arm-jogging.md` after working controls exist. Inspect the actual interface and operator workflow before documenting commands.

Neither proposal requires editing the arm team's runtime files. The coordinator registers an accepted scope before it starts.

## Hand off actual state

Supply changed paths and hashes, the base revision, raw commands/logs/exit codes, retained failures, resource handles, and the exact remaining actions. Include what could not be checked. Preserve receipts and source in durable run storage; temporary `/tmp` paths alone are insufficient.

Before pausing or handing off, record the current revision, changed paths, commands and results, active work, blockers, and exact next steps. Re-read the live repository state before resuming.

## Land verified units

Use Pstack poteto-mode to choose the workflow. Ground behavior, agree on module boundaries, implement an isolated unit, run actual checks, and inspect the configured review panel. Apply accepted findings, no-comments/deslop, a decision-trail audit, and fresh independent verification. Preserve failed seats and failed checks; never substitute a report for execution evidence.

Only the exact verified commit with green checks can be merged. Physical operations remain blocked on operator commissioning and the unresolved timeout acknowledgment. Fake fixtures and PTYs cannot validate physical clearance, gains, gravity support, or emergency stopping time.
