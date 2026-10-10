# Contributing

Read [coordination](docs/development/coordination.md) and the [task registry](docs/development/tasks.tsv) before writing. Use an isolated branch/worktree with an agreed base commit and file scope. Keep unrelated changes and retained failed evidence intact. The original Development checkout is separate and must stay untouched.

## Development checks

Install the locked environment in the README. Run focused checks first:

```bash
.venv/bin/python -m unittest discover -s tests -p "test_arm_*.py"
node --test tests/test_arm_input.mjs tests/test_arm_presets.mjs
```

Run the offline baseline before delivery:

```bash
scripts/check_offline_baseline.sh
```

It uses fake I/O and isolated PTYs. Browser checks use the running local dashboard. A rendered CAD model is not physical feedback. Hardware commands belong to the coordinating parent and require the user's concrete authorization.

## Review and delivery

State intent, affected files, exact source revision, actual commands/results and remaining boundaries. Preserve failed attempts alongside corrections. File-only workers hand checks to the parent rather than claiming them. The coordinator triages configured model reviews and owns Git operations. A fresh nonwriter verifies the final artifact before an attached private PR, exact-head green CI and guarded merge. Do not force-push or merge another contributor's branch.

## Where code belongs

- `arm_manual.py`: the one-owner J1 controller; never create a second serial writer.
- `arm_dashboard.py`: loopback HTTP boundary and explicit observational schema.
- `arm_web/`: plain UI, input reducer and actual CAD renderer.
- `arm_assets/`: generated model, mesh binaries and provenance.
- `im6014_*.py`: protocol and transport shared with the preserved bench UI.
- `tools/`: offline model conversion and its checks/dependencies.
- `docs/`: user procedures, contributor coordination and architecture.
- `evidence/`: measured results and historical verification records.

Keep finite values and validation at boundaries, exact wire/reference arithmetic, bounded queues, explicit Stop, and actual resource ownership. Simplify code without weakening those contracts. Contributors must migrate all callers in the same change when moving an API.
