# Model availability and parent review

The runtime role configuration is the operator's `~/.codex/pstack-models.md`. This document records observations, not a second editable routing configuration.

| Seat | Exact assigned descriptor | Route |
| --- | --- | --- |
| Orchestration and design | `codex:gpt-6.1-sol@xhigh` | Native Codex agent with exact requested model and effort |
| Implementation | `codex:gpt-6-luna@max` | Native Codex agent with exact requested model and effort |
| Exploration and design | `antigravity:gemini-3.8-flash-high@default` | Pstack runner, explicit API-spend deny |
| Reviewer | `opencode:opencode/muse-spark-1.3-contributor-free@default` | User-requested OpenCode-Wrap reviewer launcher |
| Reviewer | `opencode:opencode/mimo-v2.6-flash-free@default` | Same restricted supplied-text reviewer route |

Read `evidence/baseline/models/observed-executions.json` and the referenced native attestations and terminal receipts. Model and effort names describe requested dispatch flags or provider reports. They do not expose hidden reasoning depth. Both OpenCode reviewers share one provider. Their model diversity is not separate-provider billing or infrastructure diversity.

## Review procedure

1. The parent records source state and resolves the configured seat from a frozen model-sheet snapshot.
2. Give a writer one bounded unit and an isolated worktree. The parent owns Git and package operations.
3. Inspect the resulting source diff. Run relevant offline checks independently using the selected interpreter.
4. Send complete review material inline to Muse and MiMo through the installed reviewer launcher. They receive no repository tools. Reject any tool permission request.
5. Check each finding against source and real evidence. Record accepted fixes and dismissed findings with reasons.
6. Inspect screenshots. Save actual results and remaining dependencies in the milestone ledger.

Same-model continuation is limited to two total executions after an inspected, corrected permission interruption or completed parent handoff. It does not bypass a denial or substitute another model. No fallback model chains were configured for this run.

## Current result

All five descriptors executed assigned work in this run. Luna delivered a partial task after a denied cache-cleanup command; the parent preserved its edits, set the script executable separately, and ran the complete checks. Both final text reviewers completed without tool requests. Read [the finding-by-finding verdict](review.md) before interpreting a receipt as acceptance. Exact native flags and pinned external requests are execution evidence, not independent identity or reasoning-depth attestations.
