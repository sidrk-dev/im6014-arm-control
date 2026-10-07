# A01-A10 code and audit review

## Intent

Implement the first ten baseline milestones, preserve the original bench, establish one private repository and a reproducible environment, and prove disconnected startup without physical motor I/O.

## Reviewers

- Muse Spark 1.3 contributor free: three actionable code/evidence findings. Terminal receipt complete; one submission; zero rejected permissions.
- MiMo v2.6 Flash free: eight findings, mostly evidence and diagnostic improvements. Terminal receipt complete; one submission; zero rejected permissions.
- Both reviews used the same supplied-text prompt, with its SHA256 recorded in the model execution index. Both models use OpenCode; they are independent model reviews on one provider.

The reviewed source matches the initial implementation commit `36e3d62`. Parent corrections follow in `25e8acd` and the final code snapshot identified by `startup/artifact-map.json`. Raw review text and receipts remain unchanged under `evidence/baseline/models`. The review packet contained bounded current-turn transcript excerpts, not the complete transcript. Later publication and screenshot inspection were outside its frozen cutoff.

## Act on

| Finding | Model | Parent decision and evidence |
| --- | --- | --- |
| Receipt omitted the two inherited test files | Muse | Fingerprint all eight copied files. Missing files fail. Final receipts and artifact map identify their bytes. |
| Behavioral gate accepts another interpreter | Muse | Document version pinning at installation and evidence review. The clean environment and CI install the hash lock; the verifier records the actual interpreter. No hardcoded version guard is needed for its configurable behavioral check. |
| Generated script uses an escaped continuation | Muse | Use a parenthesized context manager. Seven actual failure cases still pass. |
| Executable mode not yet committed at packet cutoff | MiMo 1 | Confirmed mode `100755` in the committed tree; direct invocation passed GitHub CI. |
| Historical receipt paths are not portable | MiMo 3 | Preserve raw execution receipts and add `startup/artifact-map.json` with committed relative screenshot paths and hashes. Scratch argv remains historical evidence. |
| Final screenshot inspection lacks a strong pointer | Muse attention; MiMo 4 | Parent opened both final committed PNG paths, compared hashes, and recorded exact paths in the artifact map. Independent verifier also inspected both source selections. |
| Callback failure headline reports only exit code | MiMo 5 | Validate callback errors before the exit code; retain the full failure report. |
| Cleanup repeats its terminal action | MiMo 6 | Use two branches while retaining cleanup for a partially constructed window. The first normal-close observations remain immutable. |
| Model aggregate omits Luna and is stale | Both; MiMo 7 | Add Luna and final panel records. Preserve the worker's partial-task/denied-command outcome and the parent's acceptance checks. |
| Seven passing fault subtests are not individually logged | MiMo 8 | Print each case only after its exit, diagnostic, and absent success receipt assertions pass. The final log contains seven case lines. |
| Decision row can be mistaken for an unobserved second run | MiMo attention | Append a clarification for the 03:13:46 row. The first complete driver run had passed; the second was observed later. Use final checks/logs as the current result. |

## Consider

No unresolved baseline blockers.

## Noted

The inherited UI clips its long default adapter text in the combo box. The existing dashboard test writes an optional image to `/tmp/dashboard_small.png`. That path was explicitly authorized for the independent verification, run serially, and excluded from committed evidence. Neither changes the disconnected startup result.

External model identifiers are requested routes and launcher/backend reports. Receipts have `modelVerified: false`; they do not independently attest provider identity or applied hidden reasoning. Current availability is not a promise about future quota or account access.

## Dismissed

MiMo 2's claim that the original result summaries preceded terminal observations is contradicted by the full current-turn tool record. `audit-timing.json` preserves exit 0 and the 18-group log observation at 02:54:30, before `baseline-results.json` was written at 02:55:04. The clean dashboard exit was observed at 02:57:39, before `clean-environment.json` was written at 02:58:24. These JSONs are explicitly labeled parent attestations; their numeric fields summarize observed results rather than automatically capturing process status. The reviewer could not see all of those tool outputs in the bounded excerpts.

The reviewers' outstanding GitHub/CI comments described their pre-publication packet. The branch later passed actual CI. Current remote verification and the PR verdict establish publication, rather than treating the old review as proof of it.

## Agreement map

Both reviewers found the actual-main event-loop check, count-before-rejection sentinels, disconnected pre/post state, and worker/timer shutdown sound. Both noticed stale model metadata. Muse emphasized complete fingerprints; MiMo emphasized independently readable case logs and audit timing. Parent accepted concrete evidence improvements and checked timing against full tool results.

## Attention

Reviewed by Muse Spark 1.3 and MiMo v2.6 Flash. The concrete baseline flags above are resolved. Physical J1 mounting, calibration, live communication, gravity-loaded shutdown, and CUDA compute remain unverified and belong to later work.
