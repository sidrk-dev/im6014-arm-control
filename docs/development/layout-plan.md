# Repository layout migration

The current root mixes library modules, tests, launchers, and a preview image. The target keeps application code in a package and gives tests, tools, assets, and user docs distinct locations. One private repository remains appropriate; no submodules or additional services are needed.

## Target layout

```text
src/im6014_workbench/     codec, transport, controller, station, and dashboard modules
src/im6014_workbench/web/ browser assets shipped with the application
 tests/                  unit and fake/PTY integration tests
scripts/launchers/        application launchers
scripts/                 offline checks and benchmarks
tools/model_conversion/  offline CAD parser and generator
assets/arm/              generated visual model, manifest, and clean URDF
docs/user/               operator procedures
docs/development/        architecture and contributor procedures
```

This is a migration target, not a claim that these directories already exist. Keep modules at one package level initially; retain the existing component boundaries without adding speculative subpackages. Add packaging metadata and include browser/model assets explicitly when the move is implemented.

## Move one verified unit

Start after the arm writers settle. First inventory actual import and path callers in:

- Runtime modules and tests, including startup interception and script-relative resources.
- `start_dashboard.sh` and the eventual arm launcher.
- `scripts/check_offline_baseline.sh`, including its explicit source-copy list and immutable vendor CRC fixture.
- `scripts/check_milestone.py`, benchmarks, and CI.
- Model tools, runtime asset loading, user commands, and development docs.

Move modules and tests, update imports and source-copy paths, and update launchers/CI in the same unit. Migrate callers before removing old internal APIs; remove temporary aliases once no caller needs them. Preserve the supported user launch behavior while changing implementation paths. Do not move or edit retained failed candidates, private provider receipts, or raw CAD run storage.

Run the focused suites and full offline baseline on the moved source. Inspect disconnected bench startup and actual arm fixture controls, Stop/close behavior, and rendered captures. Verify installed-package imports and resource loading outside the source checkout. Record source hashes and retain failures. Integrate only after configured review and fresh independent verification.

The bench remains a single-motor Qt interface. The new browser arm interface uses one station/Core and managed port authority for three joints. Layout changes cannot introduce another controller or serial owner, claim a physical shutdown guarantee, or enable gravity/impedance modes before commissioning.
