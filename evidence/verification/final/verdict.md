# Independent final verification

Verdict: PASS+NOTES. No blocking source, behavior, lifecycle, fingerprint, ledger, or model-record gap was found for the frozen A01-A10 candidate. The notes below describe raw-capture whitespace, inherited UI clipping, historical receipt paths, and the remaining parent-owned publication boundary.

PR: https://github.com/sidrk-dev/im6014-workbench/pull/1

| Identity | Verified value |
| --- | --- |
| Frozen checkout HEAD | `91634b433b3b917458fa51d2fbe14bceb8c57299` |
| Patch base | `7a6881e6da6884fa1e796a662e9e3f347b7d349c` |
| Stable patch ID | `ea1f7768acbaf5f486aadca6b665f79ade304837` |
| Original bench HEAD | `69c8219d764a24d03dadf1df011efb027f0f8996` |
| Parent verified source snapshot | `818073cb6167fa3645f8f382056c9f18a96c966a` |

The verifier checkout was clean before execution and remains clean. The driver is recorded as Git mode `100755` and was invoked directly. `actual.diff` preserves the exact binary/full-index patch used for the stable patch-ID calculation. `git-audit.json` records the diff SHA256, modes, source hashes, and read-only identity checks.

The reproducible binary-diff method is `git diff --binary 7a6881e6da6884fa1e796a662e9e3f347b7d349c 91634b433b3b917458fa51d2fbe14bceb8c57299 | git patch-id --stable`. I also ran that exact method, which exited 0 and returned the patch ID above. `patch-id.log` preserves its output. The parent independently reported the same ID using this method.

## Actual execution

I read the contract, driver, inherited tests, new tests, production entry and control paths, documentation, review findings, ledger, and receipts before accepting their claims. I did not write the candidate implementation.

I ran `scripts/check_offline_baseline.sh /home/sid/Documents/code/im6014Workbench/.venv/bin/python /tmp/pstack-a01-a10-20261006-bv_ho4fx/final-verification` from the frozen checkout. The actual driver process exited 0. This run is independent of earlier-head verification and the committed CI receipt.

The actual run is preserved at `run-20261007T033511Z-9hzNOvAI`. `driver.log`, its `checks.tsv`, subprocess logs, PNGs, and startup JSONs remain available.

| Driver check | Actual exit |
| --- | --- |
| interpreter | 0 |
| packet-tests | 0 |
| dashboard-tests | 0 |
| startup-verifier-tests | 0 |
| startup-real | 0 |
| startup-demo | 0 |

The dashboard log and generated JSON agree on 18 groups. The verifier log reports five tests and `OK`. It explicitly records each of seven rejected startup faults: `serial_open`, `demo_open`, `exchange`, `screenshot`, `close`, `missing_window`, and `exit_code`. Each case asserts process exit 1, its expected diagnostic, and absence of a success receipt before printing its passing case result.

The selected interpreter is Python 3.12.14. All five installed package versions equal the five-package hash lock. An additional read-only `pip check` exited 0 and printed `No broken requirements found.` I inspected the committed clean-install, freeze, and dependency-check evidence rather than performing an installation.

Temporary storage and XDG/Pip cache, configuration, data, and runtime paths were under the assigned evidence root. The legacy `/tmp/dashboard_small.png` write was explicitly authorized and is omitted from evidence. No physical serial I/O or physical connected operation was performed. No source/docs edits, Git mutations, package installation, or GitHub/PR writes were made.

## Startup evidence and images

Both fresh reports and both committed reports show `Disconnected`, disarmed, disconnected, bus `None`, TX 0, RX 0, and zero serial-open, demo-open, and exchange sentinel counts before and after close. The normal event loop updated the UI before inspection. The window, timer, and worker were active before close and stopped after normal close. Both source modes exited through `SystemExit(0)`, restored argv, and reported no main or callback exceptions.

I opened all four actual PNGs through `view_image`: fresh real/demo and committed final real/demo. All are 1200 by 970. The real images show `REAL HARDWARE · Disconnected` and the real source selection. The demo images show `DEMO · no hardware · Disconnected` and the demo source selection. Telemetry is blank, reply counters are zero, and the check, acknowledgment, arm, and run controls are disabled. The title, instructions, plots, and source identity are readable. The inherited long adapter string is clipped inside its combo box in the real image. That cosmetic limitation is already recorded in `docs/review.md` and does not affect this baseline result.

| PNG | Actual SHA256 |
| --- | --- |
| Fresh real | `c9c8aacd45192e2a68ec99cc2037252c91823b1558a8a945058ded0bdd04c63f` |
| Fresh demo | `39ef4ec17774f424052f67c9f493a9eb7eba8aa0c5a0f0ca82a62e43f318f245` |
| Committed real | `ea8a1331cc4420295049fc0492b7cfe5616426559f0c4e2a9d897d3cfd2619df` |
| Committed demo | `b1c8a333447529b3e5f9a70aac5d320603068dbb21069277632382b79379d2f0` |

Each PNG hash and byte count matches its own receipt. The committed hashes match the artifact map. Its receipt and PNG mappings are valid existing repository-relative paths. The raw historical absolute screenshot paths match the map's historical-path fields. Fresh PNG hashes need not equal historical PNG hashes.

All eight fingerprints in both fresh reports, both committed reports, and both artifact-map entries exactly match the checkout files, including lengths. Reading those eight files directly from the parent's recorded source snapshot also produces the same fingerprints. `artifact-audit.json` preserves the complete comparisons and observed lifecycle states.

## Ledger, source preservation, and model evidence

All ten ledger rows are present exactly once and in order. Every row has status `software verified`, nonempty expected/actual/evidence/review/revision/dependency fields, and existing evidence pointers. `ledger-audit.json` records the row-specific source and log comparison.

| Rows | Independent finding |
| --- | --- |
| A01 | Original HEAD/status and plan SHA match the recorded provenance. Imported plan bytes match. |
| A02/A04 | Launcher, real main, GUI/session ownership, every session exchange path, shutdown, and alternate CLI traffic match the documented control audit. |
| A03 | Historical raw logs and JSON agree; the frozen-head rerun independently passes both inherited suites and all 18 groups. |
| A05 | Environment claims retain observation context. The NVIDIA raw GLX capture reports dedicated 8188 MB and no unified memory. CUDA compute remains unverified. |
| A06 | Historical installation evidence supports the labeled parent attestation; actual selected interpreter/packages and current pip check agree. |
| A07 | Actual dependency direction and separate GUI/CLI serial ownership match the architecture. The archived MakerMods pyproject matches its recorded hash, Python range, and LeRobot pin. |
| A08 | Ten complete TSV rows persist. |
| A09 | All five requested descriptors occur. Raw receipts, native attestations, partial Luna outcome, output hashes, and final review prompt hashes agree. |
| A10 | Fresh driver, lifecycle states, seven actual failures, eight fingerprints, portable artifact mapping, and actual PNG inspection agree. |

The three production Python modules, launcher, requirements file, manual, inherited test files, and inherited dark preview are byte identical to the original `69c8219` tree in both the verifier and original checkouts. The original checkout remains at that HEAD with only the previously recorded untracked plan file.

The external model receipts are honest about pinned argv or requested model IDs and retain `modelVerified: false`. The OpenCode reviewer receipts use the requested wrapper launcher, approved spend choice, one submission, and zero rejected permissions. Gemini records the deny spend choice. Native Sol/Luna records are parent attestations of requested flags and responses. Luna's partial handoff after the denied command remains explicit. These records do not independently attest provider identity or hidden reasoning depth. `model-audit.json` contains the detailed checks. I did not dispatch models again.

## Nonblocking notes and evidence limits

`git diff --check` exited 2 solely because `evidence/baseline/glxinfo-nvidia.txt:41` and `evidence/baseline/glxinfo.txt:30` contain a new blank line at EOF. They are raw stdout captures. I preserved them and did not treat their whitespace as a behavior or evidence defect. The exact diagnostic is in `diff-check.log` and its exit is in `auxiliary-checks.json`.

My first audit script invocation exited 1 because its case-line parser required `PASS` at column zero. Unittest prefixes the first printed `serial_open` case with its test label on that line. The actual log contains all seven cases. I corrected the evidence-only parser, preserved `audit-first-summary.json`, and reran the audit; it exited 0 with no failures. No candidate file changed.

The raw startup receipts retain historical scratch argv and execution paths. Their committed relative mappings and hashes resolve correctly. The committed CI receipt identifies an earlier head; it is historical evidence and was not used as proof of this candidate. This verdict establishes the fresh local check of the exact frozen HEAD above. The parent separately reported both GitHub CI checks green at this frozen head. Current PR/main identity, auto-merge null, queue-entry null, and current CI status were supplied by the parent; this worker did not independently query or change GitHub.

Phase E remains correctly unchecked until the parent's GitHub landing and handback. That is not a source defect. Physical J1 mounting/calibration, live motor communication, gravity-loaded shutdown, torque-off, CUDA compute, and later arm milestones remain outside the established result.

Supporting files: `git-audit.json`, `execution-audit.json`, `artifact-audit.json`, `ledger-audit.json`, `model-audit.json`, `upstream-audit.json`, `audit-summary.json`, `auxiliary-checks.json`, `audit-first-summary.json`, `parent-update.json`, `audit.py`, `audit.log`, `driver.log`, `dependency-check.log`, `diff-check.log`, `patch-id.log`, `actual.diff`, and the complete fresh run directory.
