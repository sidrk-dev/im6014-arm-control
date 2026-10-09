# Contribute to Workbench

## Choose a task

Read [coordination](docs/development/coordination.md), inspect the [task registry](docs/development/tasks.tsv), and agree on an isolated scope with the coordinator. Start from its specified commit, not an assumed default branch. Preserve unrelated local changes.

Install the locked environment using the [README](README.md). The selected Linux/Python environment is verified; other platforms need their own evidence. Propose dependency changes with the affected callers and checks.

## Check a change

Use the offline baseline for a full regression run:

```bash
scripts/check_offline_baseline.sh
```

For focused checks:

```bash
.venv/bin/python -B -m unittest -v test_joint_mapping
.venv/bin/python -B -m unittest -v test_controller_limits
.venv/bin/python -B scripts/check_milestone.py C15
env QT_QPA_PLATFORM=offscreen .venv/bin/python -B test_dashboard.py
```

For model conversion, install the separate converter environment and run the suite in [the arm model guide](docs/arm-model.md). NumPy is a converter dependency. The dashboard runtime does not import it.

Use the test modules suited to the change. The milestone checker records raw logs and source hashes. Automated development checks use fake I/O and isolated PTYs. Read the scripts before execution; never substitute a physical serial port.

Dispatched file-only workers request checks through their prepared Pstack handoff. Do not claim checks ran when only source inspection was possible. Independent collaborators can run checks in their own authorized environment and provide the actual commands and results.

## Submit a reviewable change

Describe the behavior, affected files, source revision, and actual results. Link raw failures as well as passing evidence. State remaining physical or environment boundaries. Keep requested, dispatched, and measured values distinct.

The coordinator inspects configured reviews, applies accepted fixes, cleans comments/prose, and audits the decision trail. A fresh nonwriter verifier checks the final artifact. The coordinator opens or updates the private PR and merges only the verified head with green exact-commit CI. Do not force-push shared branches or merge another contributor's work.

CI is `.github/workflows/offline-baseline.yml`, using Ubuntu 24.04 and Python 3.12.14. It runs the offline baseline and the separate synthetic arm-model suite, preserving their evidence artifacts. Full private-CAD source checks run locally; their absence is an explicit CI skip. Green CI establishes those software checks, not physical motor performance.

## Keep the code maintainable

Put validation at external boundaries. Keep domain records immutable and reference arithmetic exact. Prefer a focused module to a growing script, and remove obsolete callers when replacing an internal API. Use the [layout plan](docs/development/layout-plan.md) for the coordinated package move.

Keep user procedures in user docs, development procedures in contributor docs, and historical evidence in the ledger. Write commands that exist at the documented revision. Do not describe planned controls as available.
